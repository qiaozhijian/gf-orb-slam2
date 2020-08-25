/*
								+--------------------------------+
								|                                |
								|     ***  Vertex class  ***     |
								|                                |
								|  Copyright © -tHE SWINe- 2015  |
								|                                |
								|            Vertex.h            |
								|                                |
								+--------------------------------+
*/

#pragma once
#ifndef __VARIANT_VERTEX_CLASS_INCLUDED
#define __VARIANT_VERTEX_CLASS_INCLUDED

/**
 *	@file Vertex.h
 *	@brief variant vertex class
 *	@author -tHE SWINe-
 *	@date 2015
 */

#include "../UberLame_src/Vector.h"

typedef float TScalar; /**< @brief scalar type (float or double; float is generally sufficient for rendering) */

class CVertex {
protected:
	unsigned int n_dim;
	const TScalar *p_data;
	// and type?

public:
	CVertex(unsigned int _n_dim = 0, TScalar *_p_data = 0)
		:n_dim(_n_dim), p_data(_p_data)
	{}

	CVertex(const CVertex &r_other)
		:n_dim(r_other.n_dim), p_data(r_other.p_data)
	{}

	unsigned int n_Dimension() const
	{
		return n_dim;
	}

	bool b_Has_Position() const
	{
		return n_dim == 3 || n_dim == 6 || n_dim == 7;
	}

	bool b_Has_Pose() const
	{
		return n_dim == 6 || n_dim == 7;
	}

	Vector3f v_Position() const
	{
		_ASSERTE(b_Has_Position());
		if(n_dim == 3)
			return Vector3f(p_data[0], p_data[1], p_data[2]);
		else if(n_dim == 6) {
			Vector3f v_pos(p_data[0], p_data[1], p_data[2]);
			Vector3f v_rot(p_data[3], p_data[4], p_data[5]);
			Quatf t_rotation(v_rot, quat::from_axis_angle);
			return -t_rotation.t_Conjugate().v_Transform(v_pos); // cheaper to use quaternion than to convert to a matrix
			// make a transformation matrix for the given camera
		} else {
			// untested

			_ASSERTE(n_dim == 7);
			Vector3f v_pos(p_data[0], p_data[1], p_data[2]);
			return v_pos; // not inverse
			// make a transformation matrix for the given camera
		}
	}

	std::pair<Matrix3f, Vector3f> t_Pose() const
	{
		_ASSERTE(b_Has_Pose());
		if(n_dim == 6) {
			Vector3f v_pos(p_data[0], p_data[1], p_data[2]);
			Vector3f v_rot(p_data[3], p_data[4], p_data[5]);
			Quatf t_rotation(v_rot, quat::from_axis_angle);
			Matrix3f t_transform = t_rotation.t_Inverse().t_ToMatrix(); // 2015-06-15 - changed sign here; a negative matrix has determinant -1 and is not a rotation matrix; conversions to and from quaternions are then broken; need to change signs of axes of the system when drawing the cameras (dir really, up and right do not matter as all the +-1 combinations are taken and so it does not change the visual result)
			v_pos = t_transform * -v_pos; // 2015-06-15 - changed sign here to compensate
			// make a transformation matrix for the given camera

			return std::make_pair(t_transform, v_pos); // return [R|t]
		} else {
			// untested

			_ASSERTE(n_dim == 7);
			Vector3f v_pos(p_data[0], p_data[1], p_data[2]);
			Quatf t_rotation(p_data[3], p_data[4], p_data[5], p_data[6]);
			// this is not inverse

			return std::make_pair(t_rotation.t_ToMatrix(), v_pos); // return [R|t]

			/*
			Quatf q(v.p_data[3], v.p_data[4], v.p_data[5], v.p_data[6]);
			Vector3f r = q.v_ToAxisAngle();
			v.p_data[3] = r.x;
			v.p_data[4] = r.y;
			v.p_data[5] = r.z;

			Vector3f p(v.p_data[0], v.p_data[1], v.p_data[2]);

			Vector3f v_pos = q.v_Transform(-p); // flip there ...
			Quatf t_rotation = q;
			Matrix3f t_transform = -t_rotation.t_ToMatrix().t_Transpose(); // ... and back
			v_pos = t_transform * v_pos;
			// make a transformation matrix for the given camera
			*/
			// it is suspicious that the position is direct (not inverse
			// as in 6D) but the rotation is the same as in 6D
		}
	}
};

#endif // !__VARIANT_VERTEX_CLASS_INCLUDED
