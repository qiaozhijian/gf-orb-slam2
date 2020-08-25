/*
								+--------------------------------+
								|                                |
								|  ***  Geometry streaming  ***  |
								|                                |
								|  Copyright © -tHE SWINe- 2016  |
								|                                |
								|        GeomStreamers.h         |
								|                                |
								+--------------------------------+
*/

#pragma once
#ifndef __GEOMETRY_STREAMING_ALGOS_INCLUDED
#define __GEOMETRY_STREAMING_ALGOS_INCLUDED

/**
 *	@file GeomStreamers.h
 *	@brief animated geometry streaming
 *	@author -tHE SWINe-
 *	@date 2016
 */

/**
 *	@brief streaming parameters stored as an enum
 */
enum {
	b_camera_centers_are_points = false, // draw points in place of camera centers?

	n_max_buffer_size_frames = 8, // reduce this to go from batch to streaming (must be at least two)
	n_max_buffer_size_B = 512 * 1048576, // maximum buffer size, in bytes
	// streaming parameters for the VBOs

	n_max_texture_size_frames = 31, // try to avoid uploading the texture at the same time as uploading the VBOs
	n_max_texture_size_B = 512 * 1048576,
	n_max_texture_size_px = n_max_texture_size_B / sizeof(Vector4f) // RGBA32F
	// streaming parameters for the camera poses texture
};

struct TSolutionFrameInfo {
	size_t n_cur_frame;
	size_t n_next_frame;
	double f_sub_frame;

	TSolutionFrameInfo(size_t _n_cur_frame, size_t _n_next_frame, double _f_sub_frame)
		:n_cur_frame(_n_cur_frame), n_next_frame(_n_next_frame), f_sub_frame(_f_sub_frame)
	{}
};

/**
 *	@brief algorithm for streaming points with projective textures
 */
class CFancyPointsStreamer {
public:
	/**
	 *	@brief data to be stored in a camera poses texture
	 *
	 *	The parameters for each camera are stored in rows, so that the first column
	 *	contains rotations of all cameras at time step zero, the second column contains
	 *	positions and so on. Need 4 RGBA float samples (!) to get a camera in the shader.
	 *
	 *	This puts a hard limit on the number of cameras (16k cameras on most NVIDIA GPUs).
	 *	A single texture can thus hold up to 4k animation frames (assuming 16k max texture size).
	 *	We could stack the cameras in multiple columns (would require adding the s coordinate
	 *	to vertex attributes). Can we use texture buffers? Are those supported in ES at all?
	 */
	struct TCameraData {
		Vector4f v_rot; /**< @brief rotation quaternion (x, y, z, w) */
		Vector4f v_pos_weight; /**< @brief position of the camera and weight (zero weight disables the camera) */
		Vector4f v_K; /**< @brief the first four nonzeros of the intrinsics matrix (when read in column major order; fx, fy, cx and cy) */
		Vector4f v_texcoords; /**< @brief texture coordinates pointing where this camera image is stored in the megatexture */
	};

protected:
	CGLTexture_2D *m_p_camera_poses_texture;
	std::pair<size_t, size_t> m_t_camera_poses_tex_first_frame_range; /**< @brief frame range inside m_p_camera_poses_texture when there are multiple solution frames present (first, last) inclusive */
	CGLMultiArraySetup<>/*CGLArraySetup*/ *m_p_cam_tex_projected_points;
	std::pair<size_t, size_t> m_t_textured_points_first_frame_range; /**< @brief frame range inside m_p_cam_tex_projected_points when there are multiple solution frames present (first, last) inclusive */
	std::vector<size_t> m_drawable_vertex_list_camtex; /**< @brief list of cumsums of the numbers of drawable vertices in m_p_cam_tex_projected_points (one per solution frame in m_t_textured_points_first_frame_range plus a leading zero) */
	// camera poses and intrinsics packed for the GPU, points with camera references

public:
	CFancyPointsStreamer()
		:m_p_camera_poses_texture(0), m_t_camera_poses_tex_first_frame_range(1, 0),
		m_p_cam_tex_projected_points(0), m_t_textured_points_first_frame_range(1, 0)
	{}

	~CFancyPointsStreamer()
	{
		Free_GLBuffers();
	}

	void Free_GLBuffers()
	{
		if(m_p_camera_poses_texture)
			delete m_p_camera_poses_texture;
		m_p_camera_poses_texture = 0;
		m_t_camera_poses_tex_first_frame_range = std::make_pair(size_t(1), size_t(0));
		if(m_p_cam_tex_projected_points)
			delete m_p_cam_tex_projected_points;
		m_p_cam_tex_projected_points = 0;
		m_t_textured_points_first_frame_range = std::make_pair(size_t(1), size_t(0));
		m_drawable_vertex_list_camtex.clear();
	}

	void Draw_FancyPoints(const Matrix4f &t_mvp, const Matrix4f &t_model, const Vector3f &v_right,
		const Vector3f &v_up, const TSolutionFrameInfo &t_frame, const CSolutionData &r_solution,
		const CGraphData &r_graph, int n_original_images_width, int n_original_images_height,
		const Matrix3f &proj_colorize_image_matrix, int n_height, float f_principal_camera_fov_degrees,
		bool b_anim_color_blend, const CGLTexture_2D *p_megatexture, const std::vector<Vector4f> &cam_texcoord_list,
		const TCamTexProjectionShader &camera_tex_proj_shader,
		const TCamTexProjectionLerpShader &camera_tex_proj_lerp_shader)
	{
		_ASSERTE(p_megatexture && !r_graph.edge_list.empty()); // this needs the camera images loaded and alo needs the graph to be loaded

		size_t n_cur_frame = t_frame.n_cur_frame;
		size_t n_next_frame = t_frame.n_next_frame;
		double f_sub_frame = t_frame.f_sub_frame; // not const, will want to swap them

		/*std::pair<Matrix3f, Vector3f> t_Rt = vertex_list[0].t_Pose(); // cam 0
		//std::pair<Matrix3f, Vector3f> t_Rt = vertex_list[1].t_Pose(); // cam 1
		Vector4f v_rot = Quatf(t_Rt.first);
		const Vector3f &v_pos = t_Rt.second; // rename
		// get a transformation matrix for the first camera*/ // debug

		// for each camera, we have:
		//	K matrix (just 4 numbers really)
		//	rotation (4D quaternion, could go down to 3 by normalizing)
		//	translation (3D position)
		//	max-weight (1D scalar, to be able to enable and disable the cameras)
		//	min/max texture coordinates
		//
		// per-vertex data are:
		//	camera id 0
		//	camera id 1
		//	camera id 2
		//	camera id 3 (floats, indexing the texture y)
		//
		// uniforms:
		//	horizontal size of texel to skip to the next camera param
		//	current time step coordinate (next time step coordinate & weight)

		if(!m_p_camera_poses_texture || m_t_camera_poses_tex_first_frame_range.first > n_cur_frame ||
		   m_t_camera_poses_tex_first_frame_range.second < n_next_frame) {
			if(m_p_camera_poses_texture)
				delete m_p_camera_poses_texture;
			m_p_camera_poses_texture = 0;

			m_t_camera_poses_tex_first_frame_range.first = 1;
			m_t_camera_poses_tex_first_frame_range.second = 0;
			// clear the range

			std::vector<size_t> cameras_cumsums;
			std::vector<TCameraData> cameras_data;
			cameras_cumsums.push_back(0);
			size_t n_max_cameras = 0;
			for(size_t j = n_cur_frame, n_last_ins_frame = min(max(n_next_frame + 1, n_cur_frame +
			   n_max_texture_size_frames), r_solution.vertex_range_list.size()); j < n_last_ins_frame; ++ j) {

				size_t n_last_frame_camera_data_size = cameras_data.size();

				size_t n_linear_camera_index = 0;
				for(size_t i = r_solution.vertex_range_list[j].first, n = i + r_solution.vertex_range_list[j].second; i < n; ++ i) {
					if(r_solution.vertex_list[i].b_Has_Pose() && n_linear_camera_index < cam_texcoord_list.size()) {
						std::pair<Matrix3f, Vector3f> t_Rt = r_solution.vertex_list[i].t_Pose();
						TCameraData t_data;
						t_data.v_rot = Quatf(t_Rt.first).v_Normalized(); // rot
						t_data.v_pos_weight.xyz(t_Rt.second); // pos
						t_data.v_pos_weight.w = 1; // weight

						CGraphData::TIntrinsicsInfo t_intrin = (r_graph.camera_intrinsics.empty())?
							CGraphData::TIntrinsicsInfo(1, 1, .5, .5, 0) : (r_graph.camera_intrinsics.count(i))?
							(*r_graph.camera_intrinsics.lower_bound(i)).second : (*(-- r_graph.camera_intrinsics.end())).second;
						const float f_image_w = n_original_images_width;
						const float f_image_h = n_original_images_height;
						const Matrix3f &m = proj_colorize_image_matrix;
						const float f_scale_x = m[0][0], f_scale_y = m[1][1]; // unitless
						const float f_offset_x = m[2][0], f_offset_y = m[2][1]; // in pixels
						// gather information needed to construct the intrinsics matrix K

						t_data.v_K = Vector4f(t_intrin.fx * f_scale_x / f_image_w, t_intrin.fy * f_scale_y / f_image_h,
							(t_intrin.cx + f_offset_x) / f_image_w, (t_intrin.cy + f_offset_y) / f_image_h);
						t_data.v_texcoords = cam_texcoord_list[n_linear_camera_index];
						t_data.v_texcoords.z -= t_data.v_texcoords.x;
						t_data.v_texcoords.w -= t_data.v_texcoords.y;
						cameras_data.push_back(t_data);
						++ n_linear_camera_index;
					}
				}

				size_t n_cameras = cameras_data.size() - cameras_cumsums.back();

				if(j > n_next_frame && (cameras_cumsums.size() * 4 *
				   max(n_max_cameras, n_cameras)) > n_max_texture_size_px) {
					cameras_data.resize(n_last_frame_camera_data_size);
					// remove the pixels that were added in the last round

					printf("debug: camera parameters do not fit at once\n");

					break;
				}
				// quit if the buffer is too large

				if(n_max_cameras < n_cameras)
					n_max_cameras = n_cameras;
				cameras_cumsums.push_back(cameras_data.size());
			}
			// gather the camera parameters

			_ASSERTE(!cameras_cumsums.empty());
			const size_t n_anim_frames = cameras_cumsums.size() - 1;

			size_t n_cameras = n_max_cameras;
			size_t n_tex_width = 4 * n_anim_frames; // make POT?
			size_t n_tex_height = n_cameras + 1; // one more row to set camera weights to zero (using a dummy camera)
			std::vector<Vector4f> cameras_tex(n_tex_width * n_tex_height, Vector4f(0, 0, 0, 0)); // set all weights to zero
			for(size_t x = 0; x < n_anim_frames; ++ x) {
				const size_t n_first_cam = cameras_cumsums[x], n_last_cam = cameras_cumsums[x + 1];
				for(size_t y = 0, h = n_last_cam - n_first_cam; y < h; ++ y) {
					const TCameraData &r_t_data = cameras_data[y + n_first_cam];
					cameras_tex[4 * x + 0 + y * n_tex_width] = r_t_data.v_rot;
					cameras_tex[4 * x + 1 + y * n_tex_width] = r_t_data.v_pos_weight;
					cameras_tex[4 * x + 2 + y * n_tex_width] = r_t_data.v_K;
					cameras_tex[4 * x + 3 + y * n_tex_width] = r_t_data.v_texcoords;
				}
			}
			// assemble the camera texture

			m_p_camera_poses_texture = new CGLTexture_2D(int(n_tex_width), int(n_tex_height),
				GL_RGBA32F, false, GL_RGBA, GL_FLOAT, &cameras_tex.front());
			m_p_camera_poses_texture->Bind();
			m_p_camera_poses_texture->Set_Wrap_S(GL_CLAMP_TO_EDGE);
			m_p_camera_poses_texture->Set_Wrap_T(GL_CLAMP_TO_EDGE);
			m_p_camera_poses_texture->Set_Min_Filter(GL_NEAREST);
			m_p_camera_poses_texture->Set_Mag_Filter(GL_NEAREST);

			m_t_camera_poses_tex_first_frame_range.first = n_cur_frame;
			m_t_camera_poses_tex_first_frame_range.second = n_cur_frame + n_anim_frames - 1;
		}
		// alloc and set up the camera texture

		glActiveTexture(GL_TEXTURE0);
		p_megatexture->Bind();
		// color texture

		glActiveTexture(MEGATEXTURE_NEXT_FREE_TEXUNIT);
		m_p_camera_poses_texture->Bind();
		// texture with camera data

		if(!m_p_cam_tex_projected_points || m_t_textured_points_first_frame_range.first > n_cur_frame ||
		   m_t_textured_points_first_frame_range.second < n_next_frame) {
			if(m_p_cam_tex_projected_points)
				delete m_p_cam_tex_projected_points;
			m_p_cam_tex_projected_points = 0;

			m_t_textured_points_first_frame_range.first = 1;
			m_t_textured_points_first_frame_range.second = 0;

			m_drawable_vertex_list_camtex.clear();
			m_drawable_vertex_list_camtex.push_back(0);
			// this is supposed to be an exclusive cumsum, need to start with zero

			std::vector<float> pos_cam_id_list;
			_ASSERTE(r_graph.edge_range_list.size() == r_solution.vertex_range_list.size());
			for(size_t j = n_cur_frame, n_last_ins_frame = max(n_next_frame + 1, min(n_cur_frame +
			   n_max_buffer_size_frames, r_solution.vertex_range_list.size())); j < n_last_ins_frame; ++ j) {
				size_t n_last_frame_vertex_data_size = pos_cam_id_list.size();

				const size_t n_vertex_num = r_solution.vertex_range_list[j].second;
				std::vector<std::pair<Vector4f, size_t> > cam_coords(n_vertex_num,
					std::make_pair(Vector4f(n_vertex_num + 1, n_vertex_num + 1,
					n_vertex_num + 1, n_vertex_num + 1), size_t(0)));
				// camera coordinates for up to four cameras for each point (here n_vertex_num + 1
				// points to a dummy camera in the bottom part of the camera texture and gets zero
				// weight in the shader)

				std::vector<size_t> camera_ids;
				for(size_t i = r_solution.vertex_range_list[j].first, k = 0, n = i + n_vertex_num; i < n; ++ i, ++ k) {
					if(r_solution.vertex_list[i].b_Has_Pose())
						camera_ids.push_back(k);
				}
				// cameras are packed in the texture, need linear id instead of graph id; make a lookup table

				//const int n_cam_tex_height = m_p_camera_poses_texture->n_Height();
				size_t n_last_camera_id = size_t(-1), n_last_linear_camera_id = 0;
				for(size_t i = 0, n = r_graph.edge_range_list[j]; i < n; ++ i) {
					size_t n_camera_id = r_graph.edge_list[i].first;
					size_t n_point_id = r_graph.edge_list[i].second;
					size_t n_used_cams;
					if((n_used_cams = cam_coords[n_point_id].second) < 4) {
						size_t n_linear_camera_id = (n_last_camera_id == n_camera_id)?
							n_last_linear_camera_id : (n_last_linear_camera_id =
							std::lower_bound(camera_ids.begin(), camera_ids.end(),
							n_last_camera_id = n_camera_id) - camera_ids.begin()); // cache this, the edges are usually sorted (and this runs for every point)
						_ASSERTE(camera_ids[n_linear_camera_id] == n_camera_id);
						// cameras are packed in the texture, need linear id instead of graph id

						cam_coords[n_point_id].first[n_used_cams] = float(n_linear_camera_id + .5f) /*/ n_cam_tex_height*/; // can't use this here; the texture may upload in the meantime and change size
						cam_coords[n_point_id].second = n_used_cams + 1;
					}
				}
				// assign cameras to the points (no special reasoning here about distance
				// or relative attitude, just first fit)

				for(size_t i = r_solution.vertex_range_list[j].first, k = 0; k < n_vertex_num; ++ i, ++ k) {
					if(!r_solution.vertex_list[i].b_Has_Pose() && r_solution.vertex_list[i].b_Has_Position()) {
						Vector3f v_pos = r_solution.vertex_list[i].v_Position();
						pos_cam_id_list.insert(pos_cam_id_list.end(), &v_pos.x, &v_pos.x + 3);
						Vector4f v_cam_ids = cam_coords[k].first; //(0, 1, 1, 1); // t_odo - specify point cameras
						pos_cam_id_list.insert(pos_cam_id_list.end(), &v_cam_ids.x, &v_cam_ids.x + 4);
					}
				}
				// generate interleaved list

				if(j > n_next_frame && pos_cam_id_list.size() > n_max_buffer_size_B / sizeof(float)) {
					pos_cam_id_list.resize(n_last_frame_vertex_data_size);
					// remove the pixels that were added in the last round

					printf("debug: camera parameters do not fit at once\n");

					break;
				}
				// quit if the buffer is too large

				_ASSERTE(!(pos_cam_id_list.size() % 7));
				m_drawable_vertex_list_camtex.push_back(pos_cam_id_list.size() / 7);
			}
			// generate vertex lists

			/*m_p_cam_tex_projected_points = new CGLArraySetup(&pos_cam_id_list.front(),
				pos_cam_id_list.size() * sizeof(float), 7 * sizeof(float),
				GL_FLOAT, 3, 0, GL_FLOAT, 4, 3 * sizeof(float), GL_POINTS);*/
			m_p_cam_tex_projected_points = new CGLMultiArraySetup<>(
				TGLVBOConfig(pos_cam_id_list)(/* arrays specified at drawing time */),
				GL_POINTS, m_drawable_vertex_list_camtex.back());

			m_t_textured_points_first_frame_range.first = n_cur_frame;
			_ASSERTE(m_drawable_vertex_list_camtex.size() >= 2); // zero and at least one frame (bot there should be at least two)
			m_t_textured_points_first_frame_range.second = n_cur_frame + m_drawable_vertex_list_camtex.size() - 2; // one is zero and the other one is subtracted to make this an inclusive index
		}
		// prepare the geometry

		_ASSERTE(n_next_frame >= n_cur_frame);
		_ASSERTE(n_cur_frame >= m_t_textured_points_first_frame_range.first &&
			n_next_frame <= m_t_textured_points_first_frame_range.second);
		const size_t n_off = m_t_textured_points_first_frame_range.first;
		// make sure the frame is in the range

		const size_t n_cur_rel = n_cur_frame - n_off, n_next_rel = n_next_frame - n_off;
		size_t n_vertex_num = m_drawable_vertex_list_camtex[n_cur_rel + 1] - m_drawable_vertex_list_camtex[n_cur_rel];
		size_t n_vertex_num_next = m_drawable_vertex_list_camtex[n_next_rel + 1] - m_drawable_vertex_list_camtex[n_next_rel];
		size_t n_first_vertex = m_drawable_vertex_list_camtex[n_cur_rel/* + 1*/];
		size_t n_first_vertex_next = m_drawable_vertex_list_camtex[n_next_rel/* + 1*/];
		// calculate the vertex ranges using lists with cumsums

		_ASSERTE(n_cur_frame >= m_t_camera_poses_tex_first_frame_range.first &&
			n_next_frame <= m_t_camera_poses_tex_first_frame_range.second);
		const size_t n_tex_off = m_t_camera_poses_tex_first_frame_range.first;
		float f_first_tex = float((n_cur_frame - n_tex_off) * 4) / m_p_camera_poses_texture->n_Width();
		float f_first_tex_next = float((n_next_frame - n_tex_off) * 4) / m_p_camera_poses_texture->n_Width();
		// the same for the camera poses texture

		bool b_swapped = n_vertex_num > n_vertex_num_next;
		if(b_swapped) {
			std::swap(n_vertex_num, n_vertex_num_next);
			std::swap(n_first_vertex, n_first_vertex_next);
			std::swap(f_first_tex, f_first_tex_next);
			f_sub_frame = 1 - f_sub_frame;
		}

		float f_point_sprite_scaling_factor = float(n_height / tan(f_principal_camera_fov_degrees / 180 * f_pi * .5));
		//camera_tex_proj_shader.Bind(t_mvp, t_model, v_up, v_right, f_point_sprite_scaling_factor,
		//	1.0f / m_p_camera_poses_texture->n_Width(), f_first_tex);
		if(n_cur_frame == n_next_frame) {
			camera_tex_proj_shader.Bind(t_mvp, t_model, v_up, v_right, f_point_sprite_scaling_factor,
				1.0f / m_p_camera_poses_texture->n_Width(), m_p_camera_poses_texture->n_Height(), f_first_tex);
			m_p_cam_tex_projected_points->Draw_Attribs(GL_POINTS, 0, n_vertex_num, (
				GLVertexAttribPtrVSpec(0, 4, GL_FLOAT, false, 7 * sizeof(float), 3 * sizeof(float) + 7 * sizeof(float) * n_first_vertex),
				GLVertexAttribPtrVSpec(1, 3, GL_FLOAT, false, 7 * sizeof(float), 0 + 7 * sizeof(float) * n_first_vertex),
				GLVertexAttribPtrVDisable(2),
				GLVertexAttribPtrVDisable(3)), true);
			// the final frame of the animation
		} else {
			camera_tex_proj_lerp_shader.Bind(t_mvp, t_model, v_up, v_right, f_point_sprite_scaling_factor,
				1.0f / m_p_camera_poses_texture->n_Width(), m_p_camera_poses_texture->n_Height(), f_first_tex,
				f_first_tex_next, f_sub_frame);

			//m_p_cam_tex_projected_points->Draw(GL_POINTS, n_first_vertex, n_vertex_num);
			m_p_cam_tex_projected_points->Draw_Attribs(GL_POINTS, 0, n_vertex_num, (
				GLVertexAttribPtrVSpec(0, 4, GL_FLOAT, false, 7 * sizeof(float), 3 * sizeof(float) + 7 * sizeof(float) * n_first_vertex),
				GLVertexAttribPtrVSpec(1, 3, GL_FLOAT, false, 7 * sizeof(float), 0 + 7 * sizeof(float) * n_first_vertex),
				GLVertexAttribPtrVSpec(2, 4, GL_FLOAT, false, 7 * sizeof(float), 3 * sizeof(float) + 7 * sizeof(float) * n_first_vertex_next),
				GLVertexAttribPtrVSpec(3, 3, GL_FLOAT, false, 7 * sizeof(float), 0 + 7 * sizeof(float) * n_first_vertex_next)), true);
			// animate the existing points

			if(!b_swapped) {
				camera_tex_proj_lerp_shader.Bind(t_mvp, t_model, v_up, v_right, f_point_sprite_scaling_factor,
					1.0f / m_p_camera_poses_texture->n_Width(), m_p_camera_poses_texture->n_Height(),
					f_first_tex, f_first_tex_next, 1, // lerp towards f_first_tex_next!
					min(1.0, 4 * f_sub_frame), 0, 1, 0, (b_anim_color_blend)? 1 - f_sub_frame : 0); // green for new
			} else {
				camera_tex_proj_lerp_shader.Bind(t_mvp, t_model, v_up, v_right, f_point_sprite_scaling_factor,
					1.0f / m_p_camera_poses_texture->n_Width(), m_p_camera_poses_texture->n_Height(),
					f_first_tex, f_first_tex_next, 1, // lerp towards f_first_tex_next!
					min(1.0, 4 * f_sub_frame), 0, 0, 1, (b_anim_color_blend)? 1 - f_sub_frame : 0); // blue for fading away
			}
			glEnable(GL_BLEND);
			glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
			glVertexAttrib4f(0, 1, 1, 1, 1);
			glVertexAttrib4f(1, 0, 0, 0, 1); // make sure that there are no special values and the lerp indeed yields the correct positions
			m_p_cam_tex_projected_points->Draw_Attribs(GL_POINTS, n_vertex_num, n_vertex_num_next - n_vertex_num, (
				GLVertexAttribPtrVDisable(0),
				GLVertexAttribPtrVDisable(1)/*,
				GLVertexAttribPtrVSpec(2, 4, GL_FLOAT, false, 7 * sizeof(float), 3 * sizeof(float) + 7 * sizeof(float) * n_first_vertex_next),
				GLVertexAttribPtrVSpec(3, 3, GL_FLOAT, false, 7 * sizeof(float), 0 + 7 * sizeof(float) * n_first_vertex_next)*/), true); // attribs 2 and 3 already set above, no need to change them
			glDisable(GL_BLEND); // disable again, the rest of the points is rendered without blending
			// fade in the new points
		}

		/*if(b_swapped) {
			std::swap(n_vertex_num, n_vertex_num_next);
			std::swap(n_first_vertex, n_first_vertex_next);
			std::swap(f_first_tex, f_first_tex_next);
			f_sub_frame = 1 - f_sub_frame;
		}*/
	}
};

/**
 *	@brief algorithm for streaming flat-shaded points, optionally
 *		with per-vertex colors or palette colors
 */
class CFlatPointsStreamer {
protected:
	std::pair<size_t, size_t> m_t_points_array_first_frame_range; /**< @brief frame range inside m_p_points_array_covs and m_p_points_array_nc when there are multiple solution frames present (first, last) inclusive */
	std::vector<size_t> m_drawable_vertex_list_covs; /**< @brief list of cumsums of the numbers of drawable vertices with covariance (one per solution frame in m_t_points_array_first_frame_range plus a leading zero) */
	std::vector<size_t> m_drawable_vertex_list_nc; /**< @brief list of cumsums of the numbers of drawable vertices without covariance (one per solution framein m_t_points_array_first_frame_range plus a leading zero) */
	CGLMultiArraySetup<2> *m_p_points_array_covs; /**< @brief vertex array with vertices and uncertainty values */
	CGLMultiArraySetup<2> *m_p_points_array_nc; /**< @brief vertex array with vertices without uncertainty values */

public:
	CFlatPointsStreamer()
		:m_t_points_array_first_frame_range(1, 0), m_p_points_array_covs(0),
		m_p_points_array_nc(0)
	{}

	~CFlatPointsStreamer()
	{
		Free_GLBuffers();
	}

	void Free_GLBuffers()
	{
		if(m_p_points_array_covs)
			delete m_p_points_array_covs;
		m_p_points_array_covs = 0;
		if(m_p_points_array_nc)
			delete m_p_points_array_nc;
		m_p_points_array_nc = 0;
		m_drawable_vertex_list_covs.clear();
		m_drawable_vertex_list_nc.clear();
		m_t_points_array_first_frame_range = std::make_pair(size_t(1), size_t(0));
	}

	void Draw_Points(const Matrix4f &t_mvp, const TSolutionFrameInfo &t_frame,
		const CSolutionData &r_solution, const CMarginalsData &r_covariances,
		bool b_display_covs, bool b_anim_color_blend, const CGLTexture_2D &r_gradient_texture,
		const TConstantColorShader &const_color_shader,
		const TConstantColorLerpShader &const_color_lerp_shader,
		const TTextureShader &texture_shader, const TTextureLerpShader &texture_lerp_shader,
		const TColorShader &color_shader, const TColorLerpShader &color_lerp_shader)
	{
		size_t n_cur_frame = t_frame.n_cur_frame;
		size_t n_next_frame = t_frame.n_next_frame;
		double f_sub_frame = t_frame.f_sub_frame; // not const, will want to swap them)

		_ASSERTE(r_solution.vertex_color_list.empty() || r_solution.vertex_color_list.size() == r_solution.vertex_list.size());
		bool b_have_colors = !r_solution.vertex_color_list.empty();

		if(r_solution.vertex_range_list.size() < 2) {
			if(!m_p_points_array_covs) {
				std::vector<float> vertex_cov_list; // t1_p3_f
				std::vector<float> vertex_nc_list; // p3_f // vertices without covariance
				std::vector<Vector3ub> vertex_col_list, vertex_nc_col_list;
				for(size_t i = 0, n = r_solution.vertex_list.size(), m = r_covariances.cov_list.size(); i < n; ++ i) {
					if(m > i && r_solution.vertex_list[i].b_Has_Position() &&
					   (b_camera_centers_are_points || !r_solution.vertex_list[i].b_Has_Pose())) {
						Vector3f v_pos = r_solution.vertex_list[i].v_Position();
						float p_vertex[4] = {r_covariances.cov_list[i], v_pos[0], v_pos[1], v_pos[2]};
						vertex_cov_list.insert(vertex_cov_list.end(), p_vertex, p_vertex + 4);
						// points/poses with covariance

						if(b_have_colors)
							vertex_col_list.push_back(r_solution.vertex_color_list[i]);
					} else if(r_solution.vertex_list[i].b_Has_Position() &&
					   (b_camera_centers_are_points || !r_solution.vertex_list[i].b_Has_Pose())) {
						Vector3f v_pos = r_solution.vertex_list[i].v_Position();
						vertex_nc_list.insert(vertex_nc_list.end(), &v_pos[0], &v_pos[0] + 3);
						// points/poses without covariance

						if(b_have_colors)
							vertex_nc_col_list.push_back(r_solution.vertex_color_list[i]);
					}
				}
				// generate vertex lists

				if(b_have_colors) {
					m_p_points_array_covs = new CGLMultiArraySetup<2>((
						TGLVBOConfig(vertex_cov_list)((
							GLVertexAttribPtrSpec(0, 3, GL_FLOAT, false, 4 * sizeof(float), sizeof(float)),
							GLVertexAttribPtrSpec(1, 1, GL_FLOAT, false, 4 * sizeof(float), 0)
						)), TGLVBOConfig(vertex_col_list)((
							GLVertexAttribPtrSpec(2, 3, GL_UNSIGNED_BYTE, true, 3 * sizeof(uint8_t), 0)
						))), GL_POINTS, vertex_cov_list.size() / 4);
					if(!vertex_nc_list.empty()) {
						m_p_points_array_nc = new CGLMultiArraySetup<2>((
							TGLVBOConfig(vertex_nc_list)((
								GLVertexAttribPtrSpec(0, 3, GL_FLOAT, false, 3 * sizeof(float), 0)
							)), TGLVBOConfig(vertex_nc_col_list)((
								GLVertexAttribPtrSpec(1, 3, GL_UNSIGNED_BYTE, true, 3 * sizeof(uint8_t), 0)
							))), GL_POINTS, vertex_nc_list.size() / 3);
					}
				} else {
					m_p_points_array_covs = new CGLMultiArraySetup<2>(
						TGLVBOConfig(vertex_cov_list)((
							GLVertexAttribPtrShort(3, GL_FLOAT, 4 * sizeof(float), sizeof(float)),
							GLVertexAttribPtrShort(1, GL_FLOAT, 4 * sizeof(float), 0)
						)), GL_POINTS, vertex_cov_list.size() / 4);
					if(!vertex_nc_list.empty()) {
						m_p_points_array_nc = new CGLMultiArraySetup<2>(
							TGLVBOConfig(vertex_nc_list)((
								GLVertexAttribPtrShort(3, GL_FLOAT, 0, 0)
							)), GL_POINTS, vertex_nc_list.size() / 3);
					}
				}
			}
			if(b_display_covs) {
				glActiveTexture(GL_TEXTURE0);
				r_gradient_texture.Bind();
				texture_shader.Bind(t_mvp);
				if(b_have_colors) {
					m_p_points_array_covs->Draw_Attribs((
						//GLVertexAttribPtrVSpec(0, 3, GL_FLOAT, false, 4 * sizeof(float), sizeof(float)), // no need to respecify
						GLVertexAttribPtrVSpec(1, 1, GL_FLOAT, false, 4 * sizeof(float), 0),
						GLVertexAttribPtrVDisable(2)), true); // disable the attrib with colors
				} else {
					m_p_points_array_covs->Draw_Attribs((
						//GLVertexAttribPtrVSpec(0, 3, GL_FLOAT, false, 4 * sizeof(float), sizeof(float)), // no need to respecify
						GLVertexAttribPtrVSpec(1, 1, GL_FLOAT, false, 4 * sizeof(float), 0)), true); // make sure the attrib with covariances is enabled
				}
				if(m_p_points_array_nc) {
					const_color_shader.Bind(t_mvp, 1, 0, 0);
					if(b_have_colors) {
						m_p_points_array_nc->Draw_Attribs((
							//GLVertexAttribPtrVSpec(0, 3, GL_FLOAT, false, 3 * sizeof(float), 0), // no need to respecify
							GLVertexAttribPtrVDisable(1)), true);
					} else
						m_p_points_array_nc->Draw(); // just draw, noone will change the attribs
				}
			} else {
				if(b_have_colors) {
					color_shader.Bind(t_mvp);
					m_p_points_array_covs->Draw_Attribs((
						//GLVertexAttribPtrVSpec(0, 3, GL_FLOAT, false, 4 * sizeof(float), sizeof(float)), // no need to respecify
						GLVertexAttribPtrVDisable(2), // disable the attrib with covariances
						++
						GLVertexAttribPtrVSpec(1, 3, GL_UNSIGNED_BYTE, true, 3 * sizeof(uint8_t), 0)
						), true);
					if(m_p_points_array_nc) {
						m_p_points_array_nc->Draw_Attribs((
							//GLVertexAttribPtrVSpec(0, 3, GL_FLOAT, false, 3 * sizeof(float), 0), // no need to respecify
							++
							GLVertexAttribPtrVSpec(1, 3, GL_UNSIGNED_BYTE, true, 3 * sizeof(uint8_t), 0)
							), true);
					}
					// draw with attrib 2 as attrib 1 (alternately could change color_shader attrib bindings at run time)
				} else {
					const_color_shader.Bind(t_mvp, 1, 0, 0);
					m_p_points_array_covs->Draw_Attribs((
						//GLVertexAttribPtrVSpec(0, 3, GL_FLOAT, false, 4 * sizeof(float), sizeof(float)), // no need to respecify
						GLVertexAttribPtrVDisable(1)), true); // disable the attrib with covariances, will draw without it
					if(m_p_points_array_nc)
						m_p_points_array_nc->Draw(); // just draw, noone will change the attribs
				}
			}
			// draw vertices without animation
		} else {
			// t_odo - support animated colors
			// t_odo - add option to load the colors separately from the solution
			// the color buffer needs to have the same dimension as the vertex buffer (need colors for all stages of all vertices; many colors!)

			if(!m_p_points_array_covs || m_t_points_array_first_frame_range.first > n_cur_frame ||
			   m_t_points_array_first_frame_range.second < n_next_frame) { // not allocated or the required frames not present
				//printf("debug: (re)filling the points buffers, frame is " PRIsize "\n", n_cur_frame);

				if(m_p_points_array_nc)
					delete m_p_points_array_nc;
				m_p_points_array_nc = 0;
				if(m_p_points_array_covs)
					delete m_p_points_array_covs;
				m_p_points_array_covs = 0;
				// in case it was allocated, then it contained some bad frames.

				m_drawable_vertex_list_nc.clear();
				m_drawable_vertex_list_covs.clear();
				m_t_points_array_first_frame_range.first = 1;
				m_t_points_array_first_frame_range.second = 0;
				// nothing yet

				/*drawable_vertex_list_ref.clear();
				drawable_vertex_list_nc_ref.clear();
				// debug*/

				// cant use vertex_range_list for rendering; need to generate m_drawable_vertex_list_covs,
				// m_drawable_vertex_list_nc and m_drawable_camera_list

				// need to constrain the vertices to have some continuity (for interpolation) but that should generally be guaranteed
				// could also check continuity and in case there is none, resort to uninterpolated rendering

				m_drawable_vertex_list_covs.push_back(0);
				m_drawable_vertex_list_nc.push_back(0);
				// these are supposed to be exclusive cumsums, need to start with zero

				std::vector<Vector3ub> vertex_col_list; // vertex colors for vertices with covariance
				std::vector<Vector3ub> vertex_nc_col_list; // vertex colors for vertices without covariance
				std::vector<Vector4f> vertex_cov_list; // t1f_v3f // wertices with covariance
				std::vector<Vector3f> vertex_nc_list; // v3f // vertices without covariance
				for(size_t i = n_cur_frame, n_last_ins_frame = min(max(n_next_frame + 1, n_cur_frame +
				   n_max_buffer_size_frames), r_solution.vertex_range_list.size()); i < n_last_ins_frame; ++ i) {
					std::pair<size_t, size_t> vertex_range = r_solution.vertex_range_list[i];

					size_t n_last_frame_drawable_num = vertex_cov_list.size(),
						n_last_frame_drawable_nc_num = vertex_nc_list.size();

					for(size_t j = vertex_range.first, m = j + vertex_range.second; j < m; ++ j) {
						if(r_solution.vertex_list[j].b_Has_Position() &&
						   (b_camera_centers_are_points || !r_solution.vertex_list[j].b_Has_Pose())) {
							if(r_covariances.cov_range_list.size() > i && r_covariances.cov_range_list[i].second > j - vertex_range.first) {
								size_t n_cov = r_covariances.cov_range_list[i].first + j - vertex_range.first;
								vertex_cov_list.push_back(Vector4f(r_covariances.cov_list[n_cov], r_solution.vertex_list[j].v_Position()));
								// note that there could be interesting failures if one frame has
								// covariances and the next one does not
								
								if(b_have_colors)
									vertex_col_list.push_back(r_solution.vertex_color_list[j]);
							} else {
								vertex_nc_list.push_back(r_solution.vertex_list[j].v_Position());

								if(b_have_colors)
									vertex_nc_col_list.push_back(r_solution.vertex_color_list[j]);
							}
							// just put it in the list

							if(i > n_next_frame && (vertex_nc_list.size() > n_max_buffer_size_B / sizeof(Vector3f) ||
							   vertex_cov_list.size() > n_max_buffer_size_B / sizeof(Vector4f)))
								break;
							// do we have too much data in the buffers yet? (no need to count colors,
							// they would have smaller size than either of the buffers and we are only
							// looking at the individual buffers rather than the sum)
						}
					}
					// go through the solution frame and add its vertices

					if(i > n_next_frame && (vertex_nc_list.size() > n_max_buffer_size_B / sizeof(Vector3f) ||
					   vertex_cov_list.size() > n_max_buffer_size_B / sizeof(Vector4f))) {
						vertex_cov_list.resize(n_last_frame_drawable_num);
						vertex_nc_list.resize(n_last_frame_drawable_nc_num);
						if(b_have_colors) {
							vertex_col_list.resize(n_last_frame_drawable_num);
							vertex_nc_col_list.resize(n_last_frame_drawable_nc_num);
						}
						// remove the vertices that were added in the last round

						printf("debug: vertex buffers do not fit at once\n");

						break;
					}
					// quit if the buffer is too large

					m_drawable_vertex_list_covs.push_back(vertex_cov_list.size());
					m_drawable_vertex_list_nc.push_back(vertex_nc_list.size());
					// update the counters - not sure about the vertices at all, without the cameras

					/*drawable_vertex_list_ref.push_back(vertex_cov_list.size() - n_last_frame_drawable_num);
					drawable_vertex_list_nc_ref.push_back(vertex_nc_list.size() - n_last_frame_drawable_nc_num);
					// debug*/
				}
				// assemble the frame lists

				_ASSERTE(m_drawable_vertex_list_covs.size() == m_drawable_vertex_list_nc.size()); // the same number of frames
				size_t n_inserted_frame_num = m_drawable_vertex_list_covs.size() - 1; // minus one! one is the zero!
				// both vertices and vertices_nc have the same number of frames
				// in them - no sense keeping separate frame counts there, wouldnt
				// be able to draw consistent solutions without the other

				//printf("debug: " PRIsize " solution frames buffered, " PRIsize " vertices inside\n",
				//	n_inserted_frame_num, vertex_nc_list.size());

				/*m_drawable_vertex_list_covs.push_back(0);
				m_drawable_vertex_list_nc.push_back(0);
				stl_ut::ExclusiveScan(m_drawable_vertex_list_covs.begin(), m_drawable_vertex_list_covs.end());
				stl_ut::ExclusiveScan(m_drawable_vertex_list_nc.begin(), m_drawable_vertex_list_nc.end());*/
				// calculate cumulative sums
				// those already are cumulative sums!

				if(b_have_colors) {
					m_p_points_array_covs = new CGLMultiArraySetup<2>((
						TGLVBOConfig(vertex_cov_list)(/*(
							GLVertexAttribPtrSpec(0, 3, GL_FLOAT, false, 4 * sizeof(float), sizeof(float)), // t_odo - remove vertex attrib specs from here, they are anyway reset below
							GLVertexAttribPtrSpec(1, 1, GL_FLOAT, false, 4 * sizeof(float), 0)
						)*/), TGLVBOConfig(vertex_col_list)(/*(
							GLVertexAttribPtrSpec(2, 3, GL_UNSIGNED_BYTE, true, 3 * sizeof(uint8_t), 0)
						)*/)), GL_POINTS, vertex_cov_list.size() / 4);
					if(!vertex_nc_list.empty()) {
						m_p_points_array_nc = new CGLMultiArraySetup<2>((
							TGLVBOConfig(vertex_nc_list)(/*(
								GLVertexAttribPtrSpec(0, 3, GL_FLOAT, false, 3 * sizeof(float), 0)
							)*/), TGLVBOConfig(vertex_nc_col_list)(/*(
								GLVertexAttribPtrSpec(1, 3, GL_UNSIGNED_BYTE, true, 3 * sizeof(uint8_t), 0)
							)*/)), GL_POINTS, vertex_nc_list.size() / 3);
					}
				} else {
					m_p_points_array_covs = new CGLMultiArraySetup<2>(
						TGLVBOConfig(vertex_cov_list)(/*(
							GLVertexAttribPtrShort(3, GL_FLOAT, 4 * sizeof(float), sizeof(float)),
							GLVertexAttribPtrShort(1, GL_FLOAT, 4 * sizeof(float), 0)
						)*/), GL_POINTS, vertex_cov_list.size());
					if(!vertex_nc_list.empty()) {
						m_p_points_array_nc = new CGLMultiArraySetup<2>(
							TGLVBOConfig(vertex_nc_list)(/*(
								GLVertexAttribPtrShort(3, GL_FLOAT, 0, 0)
							)*/), GL_POINTS, vertex_nc_list.size() / 3);
					}
				}
				// build the varrays (do not enable or specify any attributes as those change when drawing)

				m_t_points_array_first_frame_range.first = n_cur_frame;
				m_t_points_array_first_frame_range.second = n_cur_frame + (n_inserted_frame_num - 1);
				// remember frame ranges in the buffer
			}
			// build and upload the arrays; no matter which mode (covs / colors / red),
			// we are only using attributes 0 - 3 (since we never draw covs and colors at the same time)

			_ASSERTE(n_next_frame >= n_cur_frame);
			_ASSERTE(n_cur_frame >= m_t_points_array_first_frame_range.first &&
				n_next_frame <= m_t_points_array_first_frame_range.second);
			const size_t n_off = m_t_points_array_first_frame_range.first;
			// make sure the frame is in the range

			/*size_t n_vertex_num_ref = drawable_vertex_list_ref[n_cur_frame - n_off];
			size_t n_vertex_num_next_ref = drawable_vertex_list_ref[n_next_frame - n_off];
			size_t n_vertex_nc_num_ref = drawable_vertex_list_nc_ref[n_cur_frame - n_off];
			size_t n_vertex_nc_num_next_ref = drawable_vertex_list_nc_ref[n_next_frame - n_off];
			size_t n_first_vertex_ref = 0, n_first_vertex_nc_ref = 0;
			for(size_t i = m_t_points_array_first_frame_range.first; i < n_cur_frame; ++ i) {
				n_first_vertex_ref += drawable_vertex_list_ref[i - n_off];
				n_first_vertex_nc_ref += drawable_vertex_list_nc_ref[i - n_off];
			}
			size_t n_first_vertex_next_ref = n_first_vertex_ref;
			size_t n_first_vertex_nc_next_ref = n_first_vertex_nc_ref;
			for(size_t i = n_cur_frame; i < n_next_frame; ++ i) { // inclusive / exclusive
				n_first_vertex_next_ref += drawable_vertex_list_ref[i - n_off];
				n_first_vertex_nc_next_ref += drawable_vertex_list_nc_ref[i - n_off];
			}
			// reference calculation of the vertex ranges*/

			//const size_t n_off = m_t_points_array_first_frame_range.first;
			const size_t n_cur_rel = n_cur_frame - n_off, n_next_rel = n_next_frame - n_off;
			size_t n_vertex_num = m_drawable_vertex_list_covs[n_cur_rel + 1] - m_drawable_vertex_list_covs[n_cur_rel];
			size_t n_vertex_num_next = m_drawable_vertex_list_covs[n_next_rel + 1] - m_drawable_vertex_list_covs[n_next_rel];
			size_t n_vertex_nc_num = m_drawable_vertex_list_nc[n_cur_rel + 1] - m_drawable_vertex_list_nc[n_cur_rel];
			size_t n_vertex_nc_num_next = m_drawable_vertex_list_nc[n_next_rel + 1] - m_drawable_vertex_list_nc[n_next_rel];
			size_t n_first_vertex = m_drawable_vertex_list_covs[n_cur_rel/* + 1*/];
			size_t n_first_vertex_next = m_drawable_vertex_list_covs[n_next_rel/* + 1*/];
			size_t n_first_vertex_nc = m_drawable_vertex_list_nc[n_cur_rel/* + 1*/];
			size_t n_first_vertex_nc_next = m_drawable_vertex_list_nc[n_next_rel/* + 1*/];
			// calculate the vertex ranges using lists with cumsums

			/*_ASSERTE(n_vertex_num == n_vertex_num_ref);
			_ASSERTE(n_vertex_num_next == n_vertex_num_next_ref);
			_ASSERTE(n_vertex_nc_num == n_vertex_nc_num_ref);
			_ASSERTE(n_vertex_nc_num_next == n_vertex_nc_num_next_ref);
			_ASSERTE(n_first_vertex == n_first_vertex_ref);
			_ASSERTE(n_first_vertex_next == n_first_vertex_next_ref);
			_ASSERTE(n_first_vertex_nc == n_first_vertex_nc_ref);
			_ASSERTE(n_first_vertex_nc_next == n_first_vertex_nc_next_ref);
			// debug*/

			/*_ASSERTE(n_vertex_num <= n_vertex_num_next || !n_vertex_num_next);
			_ASSERTE(n_vertex_nc_num <= n_vertex_nc_num_next || !n_vertex_nc_num_next);*/ // can have more or less points arbitrarily, they even fade out using a different color
			_ASSERTE(n_first_vertex_next > n_first_vertex ||
				n_cur_frame == n_next_frame || (!n_vertex_num /*&& !n_vertex_num_next*/));
			_ASSERTE(n_first_vertex_nc_next > n_first_vertex_nc ||
				n_cur_frame == n_next_frame || (!n_vertex_nc_num /*&& !n_vertex_nc_num_next*/));
			// calculate the vertex ranges (could avoid the for loop if we stored m_drawable_vertex_list_covs and m_drawable_vertex_list_nc with cumsums)
			// sometimes it can happen that the covs fall out and then the next frame has zero size
			// it is easy to support that kind of scenarion, one just swaps cur / next and instead
			// of fading the points in we fade them out

			bool b_swapped = n_vertex_num > n_vertex_num_next /*&& !n_vertex_num_next*/;
			if(b_swapped) {
				std::swap(n_vertex_num, n_vertex_num_next);
				std::swap(n_first_vertex, n_first_vertex_next);
				f_sub_frame = 1 - f_sub_frame;
			}
			// in case all the vertices went away

			if(b_display_covs) {
				glActiveTexture(GL_TEXTURE0);
				r_gradient_texture.Bind();
				if(n_cur_frame == n_next_frame) { // at the end of the animation
					texture_shader.Bind(t_mvp);
					m_p_points_array_covs->Draw_Attribs(GL_POINTS, 0, n_vertex_num,
						(GLVertexAttribPtrVSpec(0, 3, GL_FLOAT, false, sizeof(Vector4f), sizeof(float) + n_first_vertex * sizeof(Vector4f)),
						GLVertexAttribPtrVSpec(1, 1, GL_FLOAT, false, sizeof(Vector4f), 0 + n_first_vertex * sizeof(Vector4f)),
						GLVertexAttribPtrVDisable(2),
						GLVertexAttribPtrVDisable(3)), true);
					// draw constant points with covariances
				} else {
					texture_lerp_shader.Bind(t_mvp, f_sub_frame);
					m_p_points_array_covs->Draw_Attribs(GL_POINTS, 0, n_vertex_num,
						(GLVertexAttribPtrVSpec(0, 3, GL_FLOAT, false, sizeof(Vector4f), sizeof(float) + n_first_vertex * sizeof(Vector4f)),
						GLVertexAttribPtrVSpec(1, 1, GL_FLOAT, false, sizeof(Vector4f), 0 + n_first_vertex * sizeof(Vector4f)),
						GLVertexAttribPtrVSpec(2, 3, GL_FLOAT, false, sizeof(Vector4f), sizeof(float) + n_first_vertex_next * sizeof(Vector4f)),
						GLVertexAttribPtrVSpec(3, 1, GL_FLOAT, false, sizeof(Vector4f), 0 + n_first_vertex_next * sizeof(Vector4f))), true);
					// draw existing vertices, lerp positions and covarinces

					if(!b_swapped) {
						texture_lerp_shader.Bind(t_mvp, 0, min(1.0, 4 * f_sub_frame),
							0, 1, 0, (b_anim_color_blend)? 1 - f_sub_frame : 0); // green for new
					} else {
						texture_lerp_shader.Bind(t_mvp, 0, min(1.0, 4 * f_sub_frame),
							0, 0, 1, (b_anim_color_blend)? 1 - f_sub_frame : 0); // blue for fading away
					}
					//glDepthMask(0); // no, the points are then occluded by the grid and when they become solid, they "pop" in. the grid hidden by the points when they are transparent is much less noticeable.
					glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
					glEnable(GL_BLEND);
					glVertexAttrib4f(2, 0, 0, 0, 1);
					glVertexAttrib4f(3, 0, 0, 0, 1); // make sure that there are no special values and the lerp indeed yields the correct positions
					m_p_points_array_covs->Draw_Attribs(GL_POINTS,
						/*n_first_vertex_next - n_first_vertex +*/ // skip the vertices existing in the cur frame
						n_vertex_num, // skip the vertices of the cur frame to shift to the next one
						n_vertex_num_next - n_vertex_num, // the new vertices
						(GLVertexAttribPtrVSpec(0, 3, GL_FLOAT, false, sizeof(Vector4f), sizeof(float) + n_first_vertex_next * sizeof(Vector4f)),
						GLVertexAttribPtrVSpec(1, 1, GL_FLOAT, false, sizeof(Vector4f), 0 + n_first_vertex_next * sizeof(Vector4f)),
						GLVertexAttribPtrVDisable(2),
						GLVertexAttribPtrVDisable(3)), true); // point to the next frame and disable attribs 2 - 3
					glDisable(GL_BLEND);
					//glDepthMask(1);
					// draw a range of the new vertices, indexed in a mapped part of the buffer
				}
			} else if(b_have_colors) {
				if(n_cur_frame == n_next_frame) { // at the end of the animation
					color_shader.Bind(t_mvp);
					m_p_points_array_covs->Draw_Attribs(GL_POINTS, 0, n_vertex_num,
						(GLVertexAttribPtrVSpec(0, 3, GL_FLOAT, false, sizeof(Vector4f), sizeof(float) + n_first_vertex * sizeof(Vector4f)),
						++
						GLVertexAttribPtrVSpec(1, 3, GL_UNSIGNED_BYTE, true, sizeof(Vector3ub), n_first_vertex * sizeof(Vector3ub)),
						GLVertexAttribPtrVDisable(2),
						GLVertexAttribPtrVDisable(3)), true);
					// put positions in attrib 0, colors in attrib 1, disable attribs 2 - 3
				} else {
					color_lerp_shader.Bind(t_mvp, f_sub_frame);
					m_p_points_array_covs->Draw_Attribs(GL_POINTS, 0, n_vertex_num,
						(GLVertexAttribPtrVSpec(0, 3, GL_FLOAT, false, sizeof(Vector4f), sizeof(float) + n_first_vertex * sizeof(Vector4f)),
						GLVertexAttribPtrVSpec(1, 3, GL_FLOAT, false, sizeof(Vector4f), sizeof(float) + n_first_vertex_next * sizeof(Vector4f)),
						++
						GLVertexAttribPtrVSpec(2, 3, GL_UNSIGNED_BYTE, true, sizeof(Vector3ub), n_first_vertex * sizeof(Vector3ub)),
						GLVertexAttribPtrVSpec(3, 3, GL_UNSIGNED_BYTE, true, sizeof(Vector3ub), n_first_vertex_next * sizeof(Vector3ub))), true);
					// draw the existing vertices, lerp positions and colors

					if(!b_swapped) {
						color_lerp_shader.Bind(t_mvp, 0, min(1.0, 4 * f_sub_frame),
							0, 1, 0, (b_anim_color_blend)? 1 - f_sub_frame : 0); // green for new
					} else {
						color_lerp_shader.Bind(t_mvp, 0, min(1.0, 4 * f_sub_frame),
							0, 0, 1, (b_anim_color_blend)? 1 - f_sub_frame : 0); // blue for fading away
					}
					//glDepthMask(0); // no, the points are then occluded by the grid and when they become solid, they "pop" in. the grid hidden by the points when they are transparent is much less noticeable.
					glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
					glEnable(GL_BLEND);
					glVertexAttrib4f(1, 0, 0, 0, 1);
					glVertexAttrib4f(3, 0, 0, 0, 1); // make sure that there are no special values and the lerp indeed yields the correct positions
					m_p_points_array_covs->Draw_Attribs(GL_POINTS,
						/*n_first_vertex_next - n_first_vertex +*/ // skip the vertices existing in the cur frame
						n_vertex_num, // skip the vertices of the cur frame to shift to the next one
						n_vertex_num_next - n_vertex_num, // the new vertices
						(GLVertexAttribPtrVSpec(0, 3, GL_FLOAT, false, sizeof(Vector4f), sizeof(float) + n_first_vertex_next * sizeof(Vector4f)),
						++
						GLVertexAttribPtrVDisable(1),
						GLVertexAttribPtrVSpec(2, 3, GL_UNSIGNED_BYTE, true, sizeof(Vector3ub), n_first_vertex_next * sizeof(Vector3ub)),
						GLVertexAttribPtrVDisable(3)), true); // point to the next frame and disable attribs 2 - 5
					glDisable(GL_BLEND);
					//glDepthMask(1);
					// draw a range of the new vertices, indexed in a mapped part of the buffer
				}
			} else {
				if(n_cur_frame == n_next_frame) {
					const_color_shader.Bind(t_mvp, 1, 0, 0);
					m_p_points_array_covs->Draw_Attribs(GL_POINTS, 0, n_vertex_num,
						(GLVertexAttribPtrVSpec(0, 3, GL_FLOAT, false, sizeof(Vector4f), sizeof(float) + n_first_vertex * sizeof(Vector4f)),
						GLVertexAttribPtrVDisable(1),
						GLVertexAttribPtrVDisable(2),
						GLVertexAttribPtrVDisable(3)), true);
				} else {
					const_color_lerp_shader.Bind(t_mvp, f_sub_frame, 1, 0, 0);
					m_p_points_array_covs->Draw_Attribs(GL_POINTS, 0, n_vertex_num,
						(GLVertexAttribPtrVSpec(0, 3, GL_FLOAT, false, sizeof(Vector4f), sizeof(float) + n_first_vertex * sizeof(Vector4f)),
						GLVertexAttribPtrVSpec(1, 3, GL_FLOAT, false, sizeof(Vector4f), sizeof(float) + n_first_vertex_next * sizeof(Vector4f)),
						GLVertexAttribPtrVDisable(2),
						GLVertexAttribPtrVDisable(3)), true);

					if(!b_swapped) {
						const_color_shader.Bind(t_mvp,
							(b_anim_color_blend)? f_sub_frame : 1,
							(b_anim_color_blend)? 1 - f_sub_frame : 0,
							(b_anim_color_blend)? .5 * max(.0, .5 - 2 * fabs(.5 - f_sub_frame)) : 0,
							min(1.0, 4 * f_sub_frame)); // fade in the new points
					} else {
						const_color_shader.Bind(t_mvp,
							(b_anim_color_blend)? f_sub_frame : 1,
							0,
							(b_anim_color_blend)? 1 - f_sub_frame : 0,
							min(1.0, 4 * f_sub_frame)); // fade out the points that went missing in the next frame
					}

					//glDepthMask(0); // no, the points are then occluded by the grid and when they become solid, they "pop" in. the grid hidden by the points when they are transparent is much less noticeable.
					glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
					glEnable(GL_BLEND);

					/*m_p_points_array_covs->Draw_Attribs(GL_POINTS,
						n_first_vertex_next - n_first_vertex + // skip the vertices existing in the cur frame
						n_vertex_num, // skip the vertices of the cur frame to shift to the next one
						n_vertex_num_next - n_vertex_num, // the new vertices
						(//GLVertexAttribPtrVSpec(0, 3, GL_FLOAT, false, sizeof(Vector4f), sizeof(float) + n_first_vertex * sizeof(Vector4f)),
						GLVertexAttribPtrVDisable(1),
						GLVertexAttribPtrVDisable(2),
						GLVertexAttribPtrVDisable(3)), true);*/ // point to the next frame and disable attrib 1
					// works unless b_swapped is true, then we would need negative first point index (the 2nd argument to Draw_Attribs())
					// this was just a development fragment, draws the same points as the one below

					m_p_points_array_covs->Draw_Attribs(GL_POINTS,
						/*n_first_vertex_next - n_first_vertex +*/ // skip the vertices existing in the cur frame
						n_vertex_num, // skip the vertices of the cur frame to shift to the next one
						n_vertex_num_next - n_vertex_num, // the new vertices
						(GLVertexAttribPtrVSpec(0, 3, GL_FLOAT, false, sizeof(Vector4f), sizeof(float) + n_first_vertex_next * sizeof(Vector4f)),
						GLVertexAttribPtrVDisable(1),
						GLVertexAttribPtrVDisable(2),
						GLVertexAttribPtrVDisable(3)), true); // only disable attrib 1
					// this works always

					//m_p_points_array_nc->Draw_Attribs(GL_POINTS,
					//	n_vertex_nc_num/*0*//*n_first_vertex_nc_next - n_first_vertex_nc*/, n_vertex_nc_num_next - n_vertex_nc_num, // the new vertices
					//	(GLVertexAttribPtrVSpec(0, 3, GL_FLOAT, false, 0, n_first_vertex_nc_next/*n_first_vertex_nc*/ * sizeof(Vector3f)),
					//	GLVertexAttribPtrVDisable(1)), true); // disable attrib 1
					// works but is messy

					glDisable(GL_BLEND);
					//glDepthMask(1);
					// draw a range of the new vertices, indexed in a mapped part of the buffer
				}
			}
			// draw vertices with or without covariances, from the array with covariances

			if(b_swapped) {
				std::swap(n_vertex_num, n_vertex_num_next);
				std::swap(n_first_vertex, n_first_vertex_next);
				f_sub_frame = 1 - f_sub_frame;
				b_swapped = false;
			}
			// put it back

			b_swapped = n_vertex_nc_num > n_vertex_nc_num_next /*&& !n_vertex_nc_num_next*/;
			if(b_swapped) {
				std::swap(n_vertex_nc_num, n_vertex_nc_num_next);
				std::swap(n_first_vertex_nc, n_first_vertex_nc_next);
				f_sub_frame = 1 - f_sub_frame;
			}
			// in case all the vertices went away

			if(m_p_points_array_nc) {
				if(b_have_colors) {
					if(n_cur_frame == n_next_frame) { // at the end of the animation
						color_shader.Bind(t_mvp);
						m_p_points_array_nc->Draw_Attribs(GL_POINTS, 0, n_vertex_nc_num,
							(GLVertexAttribPtrVSpec(0, 3, GL_FLOAT, false, sizeof(Vector3f), n_first_vertex_nc * sizeof(Vector3f)),
							++
							GLVertexAttribPtrVSpec(1, 3, GL_UNSIGNED_BYTE, true, sizeof(Vector3ub), n_first_vertex_nc * sizeof(Vector3ub))), true);
						// put positions in attrib 0, colors in attrib 1
					} else {
						color_lerp_shader.Bind(t_mvp, f_sub_frame);
						m_p_points_array_nc->Draw_Attribs(GL_POINTS, 0, n_vertex_nc_num,
							(GLVertexAttribPtrVSpec(0, 3, GL_FLOAT, false, sizeof(Vector3f), n_first_vertex_nc * sizeof(Vector3f)),
							GLVertexAttribPtrVSpec(1, 3, GL_FLOAT, false, sizeof(Vector3f), n_first_vertex_nc_next * sizeof(Vector3f)),
							++
							GLVertexAttribPtrVSpec(2, 3, GL_UNSIGNED_BYTE, true, sizeof(Vector3ub), n_first_vertex_nc * sizeof(Vector3ub)),
							GLVertexAttribPtrVSpec(3, 3, GL_UNSIGNED_BYTE, true, sizeof(Vector3ub), n_first_vertex_nc_next * sizeof(Vector3ub))), true);
						// draw the existing vertices, lerp positions and colors

						if(!b_swapped) {
							color_lerp_shader.Bind(t_mvp, 0, min(1.0, 4 * f_sub_frame),
								0, 1, 0, (b_anim_color_blend)? 1 - f_sub_frame : 0); // green for new
						} else {
							color_lerp_shader.Bind(t_mvp, 0, min(1.0, 4 * f_sub_frame),
								0, 0, 1, (b_anim_color_blend)? 1 - f_sub_frame : 0); // blue for fading away
						}
						//glDepthMask(0); // no, the points are then occluded by the grid and when they become solid, they "pop" in. the grid hidden by the points when they are transparent is much less noticeable.
						glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
						glEnable(GL_BLEND);
						glVertexAttrib4f(1, 0, 0, 0, 1);
						glVertexAttrib4f(3, 0, 0, 0, 1); // make sure that there are no special values and the lerp indeed yields the correct positions
						m_p_points_array_nc->Draw_Attribs(GL_POINTS,
							/*n_first_vertex_nc_next - n_first_vertex_nc +*/ // skip the vertices existing in the cur frame
							n_vertex_nc_num, // skip the vertices of the cur frame to shift to the next one
							n_vertex_nc_num_next - n_vertex_nc_num, // the new vertices
							(GLVertexAttribPtrVSpec(0, 3, GL_FLOAT, false, sizeof(Vector3f), n_first_vertex_nc_next * sizeof(Vector3f)),
							++
							GLVertexAttribPtrVDisable(1),
							GLVertexAttribPtrVSpec(2, 3, GL_UNSIGNED_BYTE, true, sizeof(Vector3ub), n_first_vertex_nc_next * sizeof(Vector3ub)),
							GLVertexAttribPtrVDisable(3)), true); // point to the next frame and disable attribs 2 - 5
						glDisable(GL_BLEND);
						//glDepthMask(1);
						// draw a range of the new vertices, indexed in a mapped part of the buffer
					}
				} else {
					if(n_cur_frame == n_next_frame) { // after the last frame of the animation
						const_color_shader.Bind(t_mvp, 1, 0, 0);
						m_p_points_array_nc->Draw_Attribs(GL_POINTS, 0, n_vertex_nc_num,
							(GLVertexAttribPtrVSpec(0, 3, GL_FLOAT, false, 0, n_first_vertex_nc * sizeof(Vector3f)),
							GLVertexAttribPtrVDisable(1)), true); // disable attrib 1
						// draw a range of points, indexed in a mapped part of the buffer
						// note that this is only an optimization - the bottom branch would work just as well
					} else {
						const_color_lerp_shader.Bind(t_mvp, f_sub_frame, 1, 0, 0);
						m_p_points_array_nc->Draw_Attribs(GL_POINTS, 0, n_vertex_nc_num,
							(GLVertexAttribPtrVSpec(0, 3, GL_FLOAT, false, 0, n_first_vertex_nc * sizeof(Vector3f)),
							GLVertexAttribPtrVSpec(1, 3, GL_FLOAT, false, 0, n_first_vertex_nc_next * sizeof(Vector3f))), true);
						// draw a range of points, indexed in a mapped part of the buffer,
						// each attribute comes from a slightly different part of the buffer

						//printf("fading in %d verts, starting at vertex %d/%d\n", n_vertex_nc_num_next - n_vertex_nc_num,
						//	n_first_vertex_nc_next - n_first_vertex_nc, n_first_vertex_nc_next); // debug

						if(!b_swapped) {
							const_color_shader.Bind(t_mvp,
								(b_anim_color_blend)? f_sub_frame : 1,
								(b_anim_color_blend)? 1 - f_sub_frame : 0,
								(b_anim_color_blend)? .5 * max(.0, .5 - 2 * fabs(.5 - f_sub_frame)) : 0,
								min(1.0, 4 * f_sub_frame)); // fade in the new points
						} else {
							const_color_shader.Bind(t_mvp,
								(b_anim_color_blend)? f_sub_frame : 1,
								0,
								(b_anim_color_blend)? 1 - f_sub_frame : 0,
								min(1.0, 4 * f_sub_frame)); // fade out the points that went missing in the next frame
						}

						glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
						glEnable(GL_BLEND);
						m_p_points_array_nc->Draw_Attribs(GL_POINTS,
							/*n_first_vertex_nc_next - n_first_vertex_nc +*/ // skip the vertices existing in the cur frame
							n_vertex_nc_num, // skip the vertices of the cur frame to shift to the next one
							n_vertex_nc_num_next - n_vertex_nc_num, // the new vertices
							(GLVertexAttribPtrVSpec(0, 3, GL_FLOAT, false, 0, n_first_vertex_nc_next * sizeof(Vector3f)),
							GLVertexAttribPtrVDisable(1)), true); // only disable attrib 1
						glDisable(GL_BLEND);
						// draw a range of points, indexed in a mapped part of the buffer
					}
				}
			}
			// draw from the array where we are missing the covariances

			/*if(b_swapped) {
				std::swap(n_vertex_nc_num, n_vertex_nc_num_next);
				std::swap(n_first_vertex_nc, n_first_vertex_nc_next);
				f_sub_frame = 1 - f_sub_frame;
				b_swapped = false;
			}*/
			// put it back
		}
	}
};

/**
 *	@brief algorithm for streaming camera frusta geometry
 */
class CCameraFrustumStreamer {
protected:
	std::pair<size_t, size_t> m_t_cameras_array_first_frame_range; /**< @brief frame range inside m_p_cameras_array when there are multiple solution frames present (first, last) inclusive */
	std::vector<size_t> m_drawable_camera_list; /**< @brief list of cumsums of the numbers of drawable pose vertices (one per solution frame in m_t_cameras_array_first_frame_range plus a leading zero) */
	CGLMultiElementArraySetup<> *m_p_cameras_array; /**< @brief vertex array with camera frusta */

public:
	CCameraFrustumStreamer()
		:m_t_cameras_array_first_frame_range(1, 0), m_p_cameras_array(0)
	{}

	~CCameraFrustumStreamer()
	{
		Free_GLBuffers();
	}

	void Free_GLBuffers()
	{
		if(m_p_cameras_array)
			delete m_p_cameras_array;
		m_p_cameras_array = 0;
		m_drawable_camera_list.clear();
		m_t_cameras_array_first_frame_range = std::make_pair(size_t(1), size_t(0));
	}

	void Draw_Cameras(const Matrix4f &t_mvp, const TSolutionFrameInfo &t_frame, const CSolutionData &r_solution,
		float f_camera_size, float f_camera_aspect, float f_camera_fov_degrees, Vector3f v_color,
		const TCameraShader &camera_shader, const TCameraLerpShader &camera_lerp_shader)
	{
		size_t n_cur_frame = t_frame.n_cur_frame;
		size_t n_next_frame = t_frame.n_next_frame;
		double f_sub_frame = t_frame.f_sub_frame; // not const, will want to swap them)

		const float f_camera_image_plane_size = float(f_camera_size * tan(f_camera_fov_degrees / 360.0f * f_pi));
		if(r_solution.vertex_range_list.size() < 2) {
#if 1
			if(!m_p_cameras_array) {
				/*std::vector<Vector4f> cameras_pos_buffer; // 3D pos + [0, 4] index of the frustum corner
				std::vector<Vector4f> cameras_rot_buffer;*/
				std::vector<Vector4f> cameras_interleaved_buffer;
				//std::vector<Vector3f> cameras_vtx_buffer;
				std::vector<uint32_t> cameras_idx_buffer;

				for(size_t i = 0, n = r_solution.vertex_list.size(); i < n; ++ i) {
					if(r_solution.vertex_list[i].b_Has_Pose()) {
						std::pair<Matrix3f, Vector3f> t_Rt = r_solution.vertex_list[i].t_Pose();
						Matrix3f &t_transform = t_Rt.first; // rename
						const Vector3f &v_pos = t_Rt.second; // rename
						// get a transformation matrix for the given camera

						_ASSERTE(!(cameras_interleaved_buffer.size() % 2) && // pos and rot pairs
							cameras_idx_buffer.size() * 5 == cameras_interleaved_buffer.size() / 2 * 16); // every camera has 5 vertices and 16 indices

						//uint32_t n_base_idx = cameras_vtx_buffer.size();
						_ASSERTE(cameras_interleaved_buffer.size() / 2 <= UINT32_MAX - 4);
						uint32_t n_base_idx = uint32_t(cameras_interleaved_buffer.size() / 2);
						uint32_t p_frustum_lines[] = {
							0 + n_base_idx, 1 + n_base_idx,
							0 + n_base_idx, 2 + n_base_idx,
							0 + n_base_idx, 3 + n_base_idx,
							0 + n_base_idx, 4 + n_base_idx,
							1 + n_base_idx, 2 + n_base_idx,
							2 + n_base_idx, 3 + n_base_idx,
							3 + n_base_idx, 4 + n_base_idx,
							4 + n_base_idx, 1 + n_base_idx // could be 12 indices using line strips and primitive restart
						};
						cameras_idx_buffer.insert(cameras_idx_buffer.end(), p_frustum_lines,
							p_frustum_lines + (sizeof(p_frustum_lines) / sizeof(p_frustum_lines[0])));
						// push indices of the lines

						cameras_interleaved_buffer.reserve(cameras_interleaved_buffer.size() + 10);
						Vector4f v_rot = Quatf(t_transform)/*.t_Positive()*/;

						/*{
							Matrix3f t_reconst = Quatf(t_transform).t_ToMatrix();
							float x = v_rot.x, y = v_rot.y, z = v_rot.z, w = v_rot.w;
							float x2 = x + x, y2 = y + y, z2 = z + z;
							float xx2 = x * x2, yy2 = y * y2, zz2 = z * z2;
							float yz2 = y * z2, wx2 = w * x2, xy2 = x * y2;
							float wz2 = w * z2, xz2 = x * z2, wy2 = w * y2;
							Matrix3f t_mat;
							t_mat.Right(Vector3f(1.0 - yy2 - zz2,       xy2 - wz2,       xz2 + wy2));
							t_mat.Up(Vector3f(      xy2 + wz2, 1.0 - xx2 - zz2,       yz2 - wx2));
							t_mat.Dir(Vector3f(      xz2 - wy2,       yz2 + wx2, 1.0 - xx2 - yy2));
							float f_norm = (t_mat - t_transform).f_SquaredNorm();
						}*/
						// debug

						for(int j = 0; j < 5; ++ j) {
							cameras_interleaved_buffer.push_back(Vector4f(v_pos, j));
							cameras_interleaved_buffer.push_back(v_rot);
						}
						// push the positions and rotations interleaved (same pose for each vertex + varying local corner index)
					}
				}
				m_p_cameras_array = new CGLMultiElementArraySetup<>(
					TGLVBOConfig(cameras_interleaved_buffer)((
					GLVertexAttribPtrShort(4, GL_FLOAT, 2 * sizeof(Vector4f), sizeof(Vector4f)),
					GLVertexAttribPtrShort(4, GL_FLOAT, 2 * sizeof(Vector4f), 0)
					)), GL_LINES, TGLVBOConfig(cameras_idx_buffer), cameras_idx_buffer.size(), GL_UNSIGNED_INT);
				// pack it all into a single array
			}
			glEnable(GL_BLEND);
			glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
			camera_shader.Bind(t_mvp, f_camera_size, f_camera_image_plane_size,
				f_camera_image_plane_size / f_camera_aspect, v_color.x, v_color.y, v_color.z);
			m_p_cameras_array->Draw();
			// draw camera wireframes
#else // 1
			if(!m_p_cameras_array) {
				const float f_camera_image_plane_size = float(f_camera_size * tan(f_camera_fov_degrees / 360.0f * f_pi));

				std::vector<Vector3f> cameras_buffer;
				for(size_t i = 0, n = vertex_list.size(); i < n; ++ i) {
					if(vertex_list[i].b_Has_Pose()) {
						std::pair<Matrix3f, Vector3f> t_Rt = vertex_list[i].t_Pose();
						Matrix3f &t_transform = t_Rt.first; // rename
						const Vector3f &v_pos = t_Rt.second; // rename
						// get a transformation matrix for the given camera

						Vector3f a, b, c, d, v_up = t_transform.v_Up() * f_camera_image_plane_size,
							v_right = t_transform.v_Right() * f_camera_image_plane_size,
							v_dir = t_transform.v_Dir() * f_camera_size; // 2015-06-15 - changed sign here
						a = v_pos + v_dir + v_up + v_right;
						b = v_pos + v_dir - v_up + v_right;
						c = v_pos + v_dir - v_up - v_right;
						d = v_pos + v_dir + v_up - v_right;
						Vector3f _a = v_pos + t_transform * Vector3f(f_camera_image_plane_size, f_camera_image_plane_size, f_camera_size);
						Vector3f _b = v_pos + t_transform * Vector3f(f_camera_image_plane_size, -f_camera_image_plane_size, f_camera_size);
						Vector3f _c = v_pos + t_transform * Vector3f(-f_camera_image_plane_size, -f_camera_image_plane_size, f_camera_size);
						Vector3f _d = v_pos + t_transform * Vector3f(-f_camera_image_plane_size, f_camera_image_plane_size, f_camera_size);
						if((a - _a).f_Length() > 1e-3f)
							fprintf(stderr, "error: a != _a\n");
						if((b - _b).f_Length() > 1e-3f)
							fprintf(stderr, "error: b != _b\n");
						if((c - _c).f_Length() > 1e-3f)
							fprintf(stderr, "error: c != _c\n");
						if((d - _d).f_Length() > 1e-3f)
							fprintf(stderr, "error: d != _d\n");
						cameras_buffer.reserve(cameras_buffer.size() + 16);
						cameras_buffer.push_back(v_pos);
						cameras_buffer.push_back(a);
						cameras_buffer.push_back(v_pos);
						cameras_buffer.push_back(b);
						cameras_buffer.push_back(v_pos);
						cameras_buffer.push_back(c);
						cameras_buffer.push_back(v_pos);
						cameras_buffer.push_back(d);
						cameras_buffer.push_back(a);
						cameras_buffer.push_back(b);
						cameras_buffer.push_back(b);
						cameras_buffer.push_back(c);
						cameras_buffer.push_back(c);
						cameras_buffer.push_back(d);
						cameras_buffer.push_back(d);
						cameras_buffer.push_back(a);
					}
				}
				m_p_cameras_array = new CGLArraySetup((cameras_buffer.empty())? 0 : &cameras_buffer.front(),
					cameras_buffer.size() * sizeof(Vector3f), 0, GL_FLOAT, 3, 0, 0, 0, 0, GL_LINES);
			}
			const_color_shader.Bind(t_mvp, v_color.x, v_color.y, v_color.z);
			m_p_cameras_array->Draw();
#endif // 1
			// this works but does not support animation without respecifying everything at every frame
		} else {
			if(!m_p_cameras_array || m_t_cameras_array_first_frame_range.first > n_cur_frame ||
			   m_t_cameras_array_first_frame_range.second < n_next_frame) { // not allocated or the required frames not present
				//printf("debug: (re)filling the cameras buffers, frame is " PRIsize "\n", n_cur_frame);

				if(m_p_cameras_array)
					delete m_p_cameras_array;
				m_p_cameras_array = 0;
				// in case it was allocated, then it contained some bad frames.

				m_drawable_camera_list.clear();
				m_t_cameras_array_first_frame_range.first = 1;
				m_t_cameras_array_first_frame_range.second = 0;
				// nothing yet

				// cant use vertex_range_list for rendering; need to generate m_drawable_vertex_list_covs,
				// m_drawable_vertex_list_nc and m_drawable_camera_list

				// need to constrain the vertices to have some continuity (for interpolation) but that should generally be guaranteed
				// could also check continuity and in case there is none, resort to uninterpolated rendering

				m_drawable_camera_list.push_back(0);
				// these are supposed to be exclusive cumsums, need to start with zero

				size_t n_max_frame_index_num = 0;
				std::vector<Vector4f> cameras_interleaved_buffer; // t4f_v4f // [Rt] vertices + local corner position
				for(size_t i = n_cur_frame, n_last_ins_frame = min(max(n_next_frame + 1, n_cur_frame +
				   n_max_buffer_size_frames), r_solution.vertex_range_list.size()); i < n_last_ins_frame; ++ i) {
					std::pair<size_t, size_t> vertex_range = r_solution.vertex_range_list[i];

					size_t n_last_frame_drawable_num = cameras_interleaved_buffer.size();
					//size_t n_last_frame_idx_num = cameras_idx_buffer.size();

					size_t n_cur_frame_index_num = 0;
					for(size_t j = vertex_range.first, m = j + vertex_range.second; j < m; ++ j) {
						if(r_solution.vertex_list[j].b_Has_Pose()) {
							std::pair<Matrix3f, Vector3f> t_Rt = r_solution.vertex_list[j].t_Pose();
							Matrix3f &t_transform = t_Rt.first; // rename
							const Vector3f &v_pos = t_Rt.second; // rename
							// get a transformation matrix for the given camera

							_ASSERTE(!(cameras_interleaved_buffer.size() % 2) /*&& // pos and rot pairs
								cameras_idx_buffer.size() * 5 == cameras_interleaved_buffer.size() / 2 * 16*/); // every camera has 5 vertices and 16 indices

							_ASSERTE(cameras_interleaved_buffer.size() / 2 <= UINT32_MAX - 4);
							/*uint32_t n_base_idx = uint32_t(cameras_interleaved_buffer.size() / 2);
							uint32_t p_frustum_lines[] = {
								0 + n_base_idx, 1 + n_base_idx,
								0 + n_base_idx, 2 + n_base_idx,
								0 + n_base_idx, 3 + n_base_idx,
								0 + n_base_idx, 4 + n_base_idx,
								1 + n_base_idx, 2 + n_base_idx,
								2 + n_base_idx, 3 + n_base_idx,
								3 + n_base_idx, 4 + n_base_idx,
								4 + n_base_idx, 1 + n_base_idx // could be 12 indices using line strips and primitive restart
							};
							cameras_idx_buffer.insert(cameras_idx_buffer.end(), p_frustum_lines,
								p_frustum_lines + (sizeof(p_frustum_lines) / sizeof(p_frustum_lines[0])));*/
							n_cur_frame_index_num += 16;
							// push indices of the lines

							Vector4f v_rot = Quatf(t_transform)/*.t_Positive()*/;

							cameras_interleaved_buffer.reserve(cameras_interleaved_buffer.size() + 10);
							for(int k = 0; k < 5; ++ k) {
								cameras_interleaved_buffer.push_back(Vector4f(v_pos, k));
								cameras_interleaved_buffer.push_back(v_rot);
							}
							// push the positions and rotations interleaved (same pose for each vertex + varying local corner index)

							if(i > n_next_frame && (cameras_interleaved_buffer.size() >
							   n_max_buffer_size_B / sizeof(Vector4f) || n_cur_frame_index_num/*cameras_idx_buffer.size()*/ >
							   n_max_buffer_size_B / sizeof(uint32_t)))
								break;
							// do we have too much data in the buffers yet?
						}
					}
					// go through the solution frame and add its vertices

					if(i > n_next_frame && (cameras_interleaved_buffer.size() >
					   n_max_buffer_size_B / sizeof(Vector4f) || n_cur_frame_index_num/*cameras_idx_buffer.size()*/ >
					   n_max_buffer_size_B / sizeof(uint32_t))) {
						cameras_interleaved_buffer.resize(n_last_frame_drawable_num);
						//cameras_idx_buffer.resize(n_last_frame_idx_num);
						// remove the vertices that were added in the last round

						printf("debug: vertex buffers do not fit at once\n");

						break;
					}
					// quit if the buffer is too large

					if(n_max_frame_index_num < n_cur_frame_index_num)
						n_max_frame_index_num = n_cur_frame_index_num;

					_ASSERTE(!(cameras_interleaved_buffer.size() % 2));
					m_drawable_camera_list.push_back(cameras_interleaved_buffer.size() / 2); // this is in vertices now
					// update the counters - not sure about the vertices at all, without the cameras
				}
				// assemble the frame lists

				_ASSERTE(!(n_max_frame_index_num % 16));
				std::vector<uint32_t> cameras_idx_buffer;
				for(size_t i = 0; i < n_max_frame_index_num; i += 16) {
					uint32_t n_base_idx = uint32_t(i / 16 * 5);
					uint32_t p_frustum_lines[] = {
						0 + n_base_idx, 1 + n_base_idx,
						0 + n_base_idx, 2 + n_base_idx,
						0 + n_base_idx, 3 + n_base_idx,
						0 + n_base_idx, 4 + n_base_idx,
						1 + n_base_idx, 2 + n_base_idx,
						2 + n_base_idx, 3 + n_base_idx,
						3 + n_base_idx, 4 + n_base_idx,
						4 + n_base_idx, 1 + n_base_idx // could be 12 indices using line strips and primitive restart
					};
					cameras_idx_buffer.insert(cameras_idx_buffer.end(), p_frustum_lines,
						p_frustum_lines + (sizeof(p_frustum_lines) / sizeof(p_frustum_lines[0])));
				}
				// generate just a single index buffer
				
				size_t n_inserted_frame_num = m_drawable_camera_list.size() - 1; // minus one! one is the zero!

				//printf("debug: " PRIsize " solution frames buffered, " PRIsize " vertices inside\n",
				//	n_inserted_frame_num, cameras_interleaved_buffer.size() / 2);

				m_p_cameras_array = new CGLMultiElementArraySetup<>(
					TGLVBOConfig(cameras_interleaved_buffer)((
					GLVertexAttribPtrShort(4, GL_FLOAT, 2 * sizeof(Vector4f), sizeof(Vector4f)),
					GLVertexAttribPtrShort(4, GL_FLOAT, 2 * sizeof(Vector4f), 0)
					)), GL_LINES, TGLVBOConfig(cameras_idx_buffer), cameras_idx_buffer.size(), GL_UNSIGNED_INT);
				// build the varray

				m_t_cameras_array_first_frame_range.first = n_cur_frame;
				m_t_cameras_array_first_frame_range.second = n_cur_frame + (n_inserted_frame_num - 1);
				// remember frame ranges in the buffer
			}

			_ASSERTE(n_next_frame >= n_cur_frame);
			_ASSERTE(n_cur_frame >= m_t_cameras_array_first_frame_range.first &&
				n_next_frame <= m_t_cameras_array_first_frame_range.second);
			// make sure the frame is in the range

			const size_t n_off = m_t_cameras_array_first_frame_range.first;
			const size_t n_cur_rel = n_cur_frame - n_off, n_next_rel = n_next_frame - n_off;
			size_t n_vertex_num = m_drawable_camera_list[n_cur_rel + 1] - m_drawable_camera_list[n_cur_rel];
			size_t n_vertex_num_next = m_drawable_camera_list[n_next_rel + 1] - m_drawable_camera_list[n_next_rel];
			size_t n_first_vertex = m_drawable_camera_list[n_cur_rel/* + 1*/];
			size_t n_first_vertex_next = m_drawable_camera_list[n_next_rel/* + 1*/];
			// calculate the vertex ranges using lists with cumsums

			//_ASSERTE(n_vertex_num <= n_vertex_num_next); // lets try to lift this restriction
			_ASSERTE(n_first_vertex_next > n_first_vertex ||
				n_cur_frame == n_next_frame || (!n_vertex_num && !n_vertex_num_next));
			// calculate the vertex ranges (could avoid the for loop if we stored m_drawable_vertex_list_covs and m_drawable_vertex_list_nc with cumsums)

			bool b_swapped = n_vertex_num > n_vertex_num_next /*&& !n_vertex_num_next*/;
			if(b_swapped) {
				std::swap(n_vertex_num, n_vertex_num_next);
				std::swap(n_first_vertex, n_first_vertex_next);
				f_sub_frame = 1 - f_sub_frame;
			}
			// in case some of the vertices went away

			glEnable(GL_BLEND);
			glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

			const size_t n_stride = 2 * sizeof(Vector4f);
			if(n_cur_frame == n_next_frame) {
				camera_shader.Bind(t_mvp, f_camera_size, f_camera_image_plane_size,
					f_camera_image_plane_size / f_camera_aspect, v_color.x, v_color.y, v_color.z);
				m_p_cameras_array->Draw_Attribs(GL_LINES, n_vertex_num * 16 / 5, 0,
					(GLVertexAttribPtrVSpec(0, 4, GL_FLOAT, false, n_stride, sizeof(Vector4f) + n_first_vertex * n_stride),
					GLVertexAttribPtrVSpec(1, 4, GL_FLOAT, false, n_stride, 0 + n_first_vertex * n_stride),
					GLVertexAttribPtrVDisable(2),
					GLVertexAttribPtrVDisable(3)), true); // disable attribs 2 and 3
				// simply use the indices to select the cameras to draw
			} else {
				camera_lerp_shader.Bind(t_mvp, f_sub_frame, f_camera_size, f_camera_image_plane_size,
					f_camera_image_plane_size / f_camera_aspect, v_color.x, v_color.y, v_color.z);
				m_p_cameras_array->Draw_Attribs(GL_LINES, n_vertex_num * 16 / 5, 0,
					(GLVertexAttribPtrVSpec(0, 4, GL_FLOAT, false, n_stride, sizeof(Vector4f) + n_first_vertex * n_stride),
					GLVertexAttribPtrVSpec(1, 4, GL_FLOAT, false, n_stride, 0 + n_first_vertex * n_stride),
					GLVertexAttribPtrVSpec(2, 4, GL_FLOAT, false, n_stride, sizeof(Vector4f) + n_first_vertex_next * n_stride),
					GLVertexAttribPtrVSpec(3, 4, GL_FLOAT, false, n_stride, 0 + n_first_vertex_next * n_stride)), true);
				// need to map the rorresponding parts of the vertex buffer

				Vector3f v_lerp_color;
				if(!b_swapped)
					v_lerp_color = Vector3f(f_sub_frame, 1, f_sub_frame);
				else
					v_lerp_color = Vector3f(f_sub_frame, 0, 1 - f_sub_frame);
				v_lerp_color = v_lerp_color + (v_color - v_lerp_color) * f_sub_frame;
				camera_shader.Bind(t_mvp, f_camera_size, f_camera_image_plane_size,
					f_camera_image_plane_size / f_camera_aspect,
					v_lerp_color.x, v_lerp_color.y, v_lerp_color.z, min(1.0, 4 * f_sub_frame)); // fade in the new cameras
				m_p_cameras_array->Draw_Attribs(GL_LINES, (n_vertex_num_next - n_vertex_num) * 16 / 5,
					n_vertex_num * 16 / 5 * sizeof(uint32_t),
					(GLVertexAttribPtrVSpec(0, 4, GL_FLOAT, false, n_stride, sizeof(Vector4f) + n_first_vertex_next * n_stride),
					GLVertexAttribPtrVSpec(1, 4, GL_FLOAT, false, n_stride, 0 + n_first_vertex_next * n_stride),
					GLVertexAttribPtrVDisable(2),
					GLVertexAttribPtrVDisable(3)), true); // disable attribs 2 and 3
				// blend in the new cameras (no movement animation here)
			}
			// draw animated cameras

			/*if(b_swapped) {
				std::swap(n_vertex_num, n_vertex_num_next);
				std::swap(n_first_vertex, n_first_vertex_next);
				f_sub_frame = 1 - f_sub_frame;
				b_swapped = false;
			}*/
			// put it back
		}
	}
};

/**
 *	@brief algorithm for streaming camera images, fitting the base of their respective frusta
 *	@note This is potentially not pixel-perfect. Although the math employed to calculate the vertex
 *		positions is the same, the shader optimizer may potentially cause the results to differ,
 *		yielding an effect similar to T-junctions. However, the frusta are rendered with line
 *		smoothing and likely hide any such artifacts. It would have been better to use transform
 *		feedback for this.
 */
class CCameraTextureStreamer {
protected:
	std::pair<size_t, size_t> m_t_cameras_tex_array_first_frame_range; /**< @brief frame range inside m_p_cameras_tex_array when there are multiple solution frames present (first, last) inclusive */
	std::vector<size_t> m_drawable_camera_tex_list; /**< @brief list of cumsums of the numbers of drawable pose vertices (one per solution frame in m_t_cameras_tex_array_first_frame_range plus a leading zero) */
	CGLMultiElementArraySetup<2> *m_p_cameras_tex_array;

public:
	CCameraTextureStreamer()
		:m_t_cameras_tex_array_first_frame_range(1, 0), m_p_cameras_tex_array(0)
	{}

	~CCameraTextureStreamer()
	{
		Free_GLBuffers();
	}

	void Free_GLBuffers()
	{
		m_t_cameras_tex_array_first_frame_range = std::make_pair(size_t(1), size_t(0));
		m_drawable_camera_tex_list.clear();
		if(m_p_cameras_tex_array)
			delete m_p_cameras_tex_array;
		m_p_cameras_tex_array = 0;
	}

	void Draw_CameraImages(const Matrix4f &t_mvp, const TSolutionFrameInfo &t_frame, const CSolutionData &r_solution,
		float f_camera_size, float f_camera_aspect, float f_camera_fov_degrees, float f_camera_tex_alpha,
		const CGLTexture_2D *p_megatexture, const std::vector<Vector4f> &cam_texcoord_list,
		const TCameraTextureShader &camera_tex_shader, const TCameraTextureLerpShader &camera_tex_lerp_shader)
	{
		size_t n_cur_frame = t_frame.n_cur_frame;
		size_t n_next_frame = t_frame.n_next_frame;
		double f_sub_frame = t_frame.f_sub_frame; // not const, will want to swap them)

		const float f_camera_image_plane_size = float(f_camera_size * tan(f_camera_fov_degrees / 360.0f * f_pi));
		if(r_solution.vertex_range_list.size() < 2) {
			if(p_megatexture && f_camera_tex_alpha > 1.0 / 256) {
				if(!m_p_cameras_tex_array) {
					std::vector<float> cameras_interleaved_buffer;
					std::vector<uint32_t> cameras_idx_buffer;

					size_t n_camera_tex_index = 0;
					for(size_t i = 0, n = r_solution.vertex_list.size(); i < n; ++ i) {
						if(r_solution.vertex_list[i].b_Has_Pose() && n_camera_tex_index < cam_texcoord_list.size()) { // in case this is a camera and still have texcoords for it
							std::pair<Matrix3f, Vector3f> t_Rt = r_solution.vertex_list[i].t_Pose();
							Matrix3f &t_transform = t_Rt.first; // rename
							const Vector3f &v_pos = t_Rt.second; // rename
							// get a transformation matrix for the given camera

							Vector4f v_texcoords = cam_texcoord_list[n_camera_tex_index];
							++ n_camera_tex_index;
							// get texcoords for the given camera image

							_ASSERTE(!(cameras_interleaved_buffer.size() % 10)); // 4D pos, 4D rot, 2D tex
							uint32_t n_base_idx = uint32_t(cameras_interleaved_buffer.size() / 10);
							uint32_t p_screen_triangles[] = {
								0 + n_base_idx, 1 + n_base_idx, 2 + n_base_idx, // note that these are -1 compared to what is passed in texcoord so 0 corresponds to 1 (corner) rather than 0 (center of projection)
								2 + n_base_idx, 3 + n_base_idx, 0 + n_base_idx // could be 4 indices + primitive restart using tristrips
							};
							cameras_idx_buffer.insert(cameras_idx_buffer.end(), p_screen_triangles,
								p_screen_triangles + (sizeof(p_screen_triangles) / sizeof(p_screen_triangles[0])));
							// push indices of the triangles

							cameras_interleaved_buffer.reserve(cameras_interleaved_buffer.size() + 10);
							Vector4f v_rot = Quatf(t_transform)/*.t_Positive()*/;

							// x = [0,  1,  1, -1, -1]
							// y = [0,  1, -1, -1,  1]
							//
							//  3+---------+4
							//   |         |
							//   |    0    |
							//   |         |
							//  2+---------+1
							//

							Vector2f p_tex[4] = {
								Vector2f(v_texcoords.z, v_texcoords.w),
								Vector2f(v_texcoords.z, v_texcoords.y),
								Vector2f(v_texcoords.x, v_texcoords.y),
								Vector2f(v_texcoords.x, v_texcoords.w)
							};

							for(int j = 0; j < 4; ++ j) {
								float p_data[] = {
									v_pos.x, v_pos.y, v_pos.z, float(j + 1), // + 1 to skip the center-of-projection vertex
									v_rot.x, v_rot.y, v_rot.z, v_rot.w,
									p_tex[j].x, p_tex[j].y
								};
								cameras_interleaved_buffer.insert(cameras_interleaved_buffer.end(),
									p_data, p_data + 10);
							}
							// push the positions and rotations interleaved (same pose for each vertex + varying local corner index)
						}
					}
					// t_odo - cache the buffer

					m_p_cameras_tex_array = new CGLMultiElementArraySetup<2>(
						TGLVBOConfig(cameras_interleaved_buffer)((
						GLVertexAttribPtrShort(4, GL_FLOAT, 2 * sizeof(Vector4f) + sizeof(Vector2f), sizeof(Vector4f)),
						GLVertexAttribPtrShort(4, GL_FLOAT, 2 * sizeof(Vector4f) + sizeof(Vector2f), 0),
						GLVertexAttribPtrShort(2, GL_FLOAT, 2 * sizeof(Vector4f) + sizeof(Vector2f), 2 * sizeof(Vector4f))
						)), GL_TRIANGLES, TGLVBOConfig(cameras_idx_buffer), cameras_idx_buffer.size(), GL_UNSIGNED_INT);
					// pack it all into a single array
				}

				glActiveTexture(GL_TEXTURE0);
				p_megatexture->Bind();
				camera_tex_shader.Bind(t_mvp, f_camera_size, f_camera_image_plane_size,
					f_camera_image_plane_size / f_camera_aspect, 1, 1, 1, f_camera_tex_alpha); // not full alpha
				if(f_camera_tex_alpha < 1 - 1.0f / 256) {
					glEnable(GL_BLEND);
					glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
					glDepthMask(0);
				} else
					glDisable(GL_BLEND);
				m_p_cameras_tex_array->Draw();
				if(f_camera_tex_alpha < 1 - 1.0f / 256)
					glDepthMask(1);
			}
			// draw camera images
			// t_odo - support animation of those things too
			// t_odo - add commandline to only load the camera images only to render them here (not for projections or for vertex colors)
		} else {
			if(p_megatexture && f_camera_tex_alpha > 1.0 / 256) {
				if(!m_p_cameras_tex_array || m_t_cameras_tex_array_first_frame_range.first > n_cur_frame ||
				   m_t_cameras_tex_array_first_frame_range.second < n_next_frame) {
					if(m_p_cameras_tex_array)
						delete m_p_cameras_tex_array;
					m_p_cameras_tex_array = 0;

					m_drawable_camera_tex_list.clear();
					m_t_cameras_tex_array_first_frame_range.first = 1;
					m_t_cameras_tex_array_first_frame_range.second = 0;
					// nothing yet

					// cant use vertex_range_list for rendering; need to generate m_drawable_vertex_list_covs,
					// m_drawable_vertex_list_nc and m_drawable_camera_list

					// need to constrain the vertices to have some continuity (for interpolation) but that should generally be guaranteed
					// could also check continuity and in case there is none, resort to uninterpolated rendering

					m_drawable_camera_tex_list.push_back(0);
					// these are supposed to be exclusive cumsums, need to start with zero

					size_t n_max_frame_index_num = 0;
					std::vector<Vector4f> cameras_interleaved_buffer;
					for(size_t i = n_cur_frame, n_last_ins_frame = min(max(n_next_frame + 1, n_cur_frame +
					   n_max_buffer_size_frames), r_solution.vertex_range_list.size()); i < n_last_ins_frame; ++ i) {
						std::pair<size_t, size_t> vertex_range = r_solution.vertex_range_list[i];

						size_t n_last_frame_drawable_num = cameras_interleaved_buffer.size();
						//size_t n_last_frame_idx_num = cameras_idx_buffer.size();
						size_t n_cur_frame_index_num = 0;

						size_t n_camera_tex_index = 0;
						for(size_t j = vertex_range.first, m = j + vertex_range.second; j < m; ++ j) {
							if(r_solution.vertex_list[j].b_Has_Pose() && n_camera_tex_index < cam_texcoord_list.size()) { // in case this is a camera and still have texcoords for it
								std::pair<Matrix3f, Vector3f> t_Rt = r_solution.vertex_list[j].t_Pose();
								Matrix3f &t_transform = t_Rt.first; // rename
								const Vector3f &v_pos = t_Rt.second; // rename
								// get a transformation matrix for the given camera

								//Vector4f v_texcoords = cam_texcoord_list[n_camera_tex_index];
								++ n_camera_tex_index;
								// get texcoords for the given camera image

								_ASSERTE(!(cameras_interleaved_buffer.size() % 2/*10*/)); // 4D pos, 4D rot // 2D tex moved to another buffer
								/*uint32_t n_base_idx = uint32_t((cameras_interleaved_buffer.size() - n_last_frame_drawable_num) / 10); // each frame starts with index 0
								uint32_t p_screen_triangles[] = {
									0 + n_base_idx, 1 + n_base_idx, 2 + n_base_idx, // note that these are -1 compared to what is passed in texcoord so 0 corresponds to 1 (corner) rather than 0 (center of projection)
									2 + n_base_idx, 3 + n_base_idx, 0 + n_base_idx // could be 4 indices + primitive restart using tristrips
								};
								cameras_idx_buffer.insert(cameras_idx_buffer.end(), p_screen_triangles,
									p_screen_triangles + (sizeof(p_screen_triangles) / sizeof(p_screen_triangles[0])));*/
								n_cur_frame_index_num += 6;
								// push indices of the triangles

								cameras_interleaved_buffer.reserve(cameras_interleaved_buffer.size() + 2/*10*/);
								Vector4f v_rot = Quatf(t_transform)/*.t_Positive()*/;

								// x = [0,  1,  1, -1, -1]
								// y = [0,  1, -1, -1,  1]
								//
								//  3+---------+4
								//   |         |
								//   |    0    |
								//   |         |
								//  2+---------+1
								//

								/*Vector2f p_tex[4] = {
									Vector2f(v_texcoords.z, v_texcoords.w),
									Vector2f(v_texcoords.z, v_texcoords.y),
									Vector2f(v_texcoords.x, v_texcoords.y),
									Vector2f(v_texcoords.x, v_texcoords.w)
								};*/

								for(int k = 0; k < 4; ++ k) {
									/*float p_data[] = {
										v_pos.x, v_pos.y, v_pos.z, float(k + 1), // + 1 to skip the center-of-projection vertex
										v_rot.x, v_rot.y, v_rot.z, v_rot.w,
										p_tex[k].x, p_tex[k].y
									};*/
									Vector4f p_data[2] = {Vector4f(v_pos, float(k + 1)), v_rot};
									cameras_interleaved_buffer.insert(cameras_interleaved_buffer.end(),
										p_data, p_data + 2/*10*/);
								}
								// push the positions and rotations interleaved (same pose for each vertex + varying local corner index)

								size_t n_cur_frame_vertex_num = n_max_frame_index_num / 6 * 4;
								if(i > n_next_frame &&
								   (cameras_interleaved_buffer.size() > n_max_buffer_size_B / sizeof(Vector4f) || // pos/rot buffer
								   /*cameras_idx_buffer.size()*/n_cur_frame_index_num > n_max_buffer_size_B / sizeof(uint32_t) || // index buffer (common)
								   n_cur_frame_vertex_num > n_max_buffer_size_B / sizeof(Vector2f)))
									break;
								// do we have too much data in the buffers yet?
							}
						}

						size_t n_cur_frame_vertex_num = n_max_frame_index_num / 6 * 4;

						if(i > n_next_frame &&
						   (cameras_interleaved_buffer.size() > n_max_buffer_size_B / sizeof(Vector4f) || // pos/rot buffer
						   /*cameras_idx_buffer.size()*/n_cur_frame_index_num > n_max_buffer_size_B / sizeof(uint32_t) || // index buffer (common)
						   n_cur_frame_vertex_num > n_max_buffer_size_B / sizeof(Vector2f))) { // texcoord buffer (common)
							cameras_interleaved_buffer.resize(n_last_frame_drawable_num);
							//cameras_idx_buffer.resize(n_last_frame_idx_num);
							// remove the vertices that were added in the last round

							printf("debug: vertex buffers do not fit at once\n");

							break;
						}
						// quit if the buffer is too large

						if(n_max_frame_index_num < n_cur_frame_index_num)
							n_max_frame_index_num = n_cur_frame_index_num;

						_ASSERTE(!(cameras_interleaved_buffer.size() % 2));
						m_drawable_camera_tex_list.push_back(cameras_interleaved_buffer.size() / 2); // this is in vertices now
						// update the counters - not sure about the vertices at all, without the cameras
					}
					// assemble the frame lists

					_ASSERTE(!(n_max_frame_index_num % 6));
					size_t n_max_frame_vertex_num = n_max_frame_index_num / 6 * 4;
					_ASSERTE(n_max_frame_vertex_num / 4 <= cam_texcoord_list.size());
					std::vector<Vector2f> cameras_texcoords_buffer;
					for(size_t i = 0, n_camera_tex_index = 0; i < n_max_frame_vertex_num; i += 4, ++ n_camera_tex_index) {
						Vector4f v_texcoords = cam_texcoord_list[n_camera_tex_index];
						Vector2f p_tex[4] = {
							Vector2f(v_texcoords.z, v_texcoords.w),
							Vector2f(v_texcoords.z, v_texcoords.y),
							Vector2f(v_texcoords.x, v_texcoords.y),
							Vector2f(v_texcoords.x, v_texcoords.w)
						};
						cameras_texcoords_buffer.insert(cameras_texcoords_buffer.end(), p_tex, p_tex + 4);
					}
					// generate just a single texcoords buffer

					std::vector<uint32_t> cameras_idx_buffer;
					for(size_t i = 0; i < n_max_frame_index_num; i += 6) {
						uint32_t n_base_idx = uint32_t(i / 6 * 4);
						uint32_t p_screen_triangles[] = {
							0 + n_base_idx, 1 + n_base_idx, 2 + n_base_idx, // note that these are -1 compared to what is passed in texcoord so 0 corresponds to 1 (corner) rather than 0 (center of projection)
							2 + n_base_idx, 3 + n_base_idx, 0 + n_base_idx // could be 4 indices + primitive restart using tristrips
						};
						cameras_idx_buffer.insert(cameras_idx_buffer.end(), p_screen_triangles,
							p_screen_triangles + (sizeof(p_screen_triangles) / sizeof(p_screen_triangles[0])));
					}
					// generate just a single index buffer

					size_t n_inserted_frame_num = m_drawable_camera_tex_list.size() - 1; // minus one! one is the zero!// build the varray

					m_p_cameras_tex_array = new CGLMultiElementArraySetup<2>((
						TGLVBOConfig(cameras_interleaved_buffer)(/*(
						GLVertexAttribPtrShort(4, GL_FLOAT, 2 * sizeof(Vector4f) + sizeof(Vector2f), sizeof(Vector4f)),
						GLVertexAttribPtrShort(4, GL_FLOAT, 2 * sizeof(Vector4f) + sizeof(Vector2f), 0),
						GLVertexAttribPtrShort(2, GL_FLOAT, 2 * sizeof(Vector4f) + sizeof(Vector2f), 2 * sizeof(Vector4f))
						)*/),
						TGLVBOConfig(cameras_texcoords_buffer)(
						GLVertexAttribPtrSpec(2, 2, GL_FLOAT, false, sizeof(Vector2f), 0) // the array with texcoords is common and its vertex attrib never changes
						)), GL_TRIANGLES, TGLVBOConfig(cameras_idx_buffer), cameras_idx_buffer.size(), GL_UNSIGNED_INT);
					// pack it all into a single array

					m_t_cameras_tex_array_first_frame_range.first = n_cur_frame;
					m_t_cameras_tex_array_first_frame_range.second = n_cur_frame + (n_inserted_frame_num - 1);
					// remember frame ranges in the buffer
				}
				// t_odo - cache the buffer

				_ASSERTE(n_next_frame >= n_cur_frame);
				_ASSERTE(n_cur_frame >= m_t_cameras_tex_array_first_frame_range.first &&
					n_next_frame <= m_t_cameras_tex_array_first_frame_range.second);
				// make sure the frame is in the range

				const size_t n_off = m_t_cameras_tex_array_first_frame_range.first;
				const size_t n_cur_rel = n_cur_frame - n_off, n_next_rel = n_next_frame - n_off;
				size_t n_vertex_num = m_drawable_camera_tex_list[n_cur_rel + 1] - m_drawable_camera_tex_list[n_cur_rel];
				size_t n_vertex_num_next = m_drawable_camera_tex_list[n_next_rel + 1] - m_drawable_camera_tex_list[n_next_rel];
				size_t n_first_vertex = m_drawable_camera_tex_list[n_cur_rel/* + 1*/];
				size_t n_first_vertex_next = m_drawable_camera_tex_list[n_next_rel/* + 1*/];
				// calculate the vertex ranges using lists with cumsums

				//_ASSERTE(n_vertex_num <= n_vertex_num_next); // lets try to lift this restriction
				_ASSERTE(n_first_vertex_next > n_first_vertex ||
					n_cur_frame == n_next_frame || (!n_vertex_num && !n_vertex_num_next));
				// calculate the vertex ranges (could avoid the for loop if we stored m_drawable_vertex_list_covs and m_drawable_vertex_list_nc with cumsums)

				glActiveTexture(GL_TEXTURE0);
				p_megatexture->Bind();

				//camera_tex_shader.Bind(t_mvp, f_camera_size, f_camera_image_plane_size,
				//	f_camera_image_plane_size / f_camera_aspect, 1, 1, 1, f_camera_tex_alpha); // not full alpha

				if(f_camera_tex_alpha < 1 - 1.0f / 256) {
					glEnable(GL_BLEND);
					glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
					glDepthMask(0);
				} else
					glDisable(GL_BLEND);

				bool b_swapped = n_vertex_num_next < n_vertex_num;
				if(b_swapped) {
					std::swap(n_vertex_num_next, n_vertex_num);
					std::swap(n_first_vertex_next, n_first_vertex);
					f_sub_frame = 1 - f_sub_frame;
				}

				const size_t n_stride = 2 * sizeof(Vector4f) /*+ sizeof(Vector2f)*/;
				if(n_cur_frame == n_next_frame) {
					camera_tex_shader.Bind(t_mvp, f_camera_size, f_camera_image_plane_size,
						f_camera_image_plane_size / f_camera_aspect, 1, 1, 1, f_camera_tex_alpha);
					m_p_cameras_tex_array->Draw_Attribs(GL_TRIANGLES, n_vertex_num * 6 / 4, 0,
						(GLVertexAttribPtrVSpec(0, 4, GL_FLOAT, false, n_stride, sizeof(Vector4f) + n_first_vertex * n_stride),
						GLVertexAttribPtrVSpec(1, 4, GL_FLOAT, false, n_stride, 0 + n_first_vertex * n_stride),
						//GLVertexAttribPtrVSpec(2, 2, GL_FLOAT, false, n_stride, 2 * sizeof(Vector4f) + n_first_vertex_next * n_stride),
						GLVertexAttribPtrVDisable(3),
						GLVertexAttribPtrVDisable(4)), true);
					// draw the final frame using the simple non-animated shader
				} else {
					camera_tex_lerp_shader.Bind(t_mvp, f_sub_frame, f_camera_size, f_camera_image_plane_size,
						f_camera_image_plane_size / f_camera_aspect, 1, 1, 1, f_camera_tex_alpha);
					m_p_cameras_tex_array->Draw_Attribs(GL_TRIANGLES, n_vertex_num * 6 / 4, 0,
						(GLVertexAttribPtrVSpec(0, 4, GL_FLOAT, false, n_stride, sizeof(Vector4f) + n_first_vertex * n_stride),
						GLVertexAttribPtrVSpec(1, 4, GL_FLOAT, false, n_stride, 0 + n_first_vertex * n_stride),
						//GLVertexAttribPtrVSpec(2, 2, GL_FLOAT, false, n_stride, 2 * sizeof(Vector4f) + n_first_vertex_next * n_stride),
						GLVertexAttribPtrVSpec(3, 4, GL_FLOAT, false, n_stride, sizeof(Vector4f) + n_first_vertex_next * n_stride),
						GLVertexAttribPtrVSpec(4, 4, GL_FLOAT, false, n_stride, 0 + n_first_vertex_next * n_stride)), true);
					// draw the existing cameras

					if(!b_swapped) {
						camera_tex_shader.Bind(t_mvp, f_camera_size, f_camera_image_plane_size,
							f_camera_image_plane_size / f_camera_aspect,
							f_sub_frame, 1, f_sub_frame, f_camera_tex_alpha * min(1.0, 4 * f_sub_frame)); // t_odo - flash // fade in the new cameras
					} else {
						camera_tex_shader.Bind(t_mvp, f_camera_size, f_camera_image_plane_size,
							f_camera_image_plane_size / f_camera_aspect,
							f_sub_frame, 0, 1 - f_sub_frame, f_camera_tex_alpha * min(1.0, 4 * f_sub_frame)); // fade out the cameras that went missing in the next frame
					}

					glEnable(GL_BLEND);
					glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
					glDepthMask(0);

					m_p_cameras_tex_array->Draw_Attribs(GL_TRIANGLES, (n_vertex_num_next - n_vertex_num) * 6 / 4, n_vertex_num * 6 / 4 * sizeof(uint32_t),
						(GLVertexAttribPtrVDisable(3),
						GLVertexAttribPtrVDisable(4),
						//GLVertexAttribPtrVSpec(2, 2, GL_FLOAT, false, n_stride, 2 * sizeof(Vector4f) + n_first_vertex_next * n_stride),
						GLVertexAttribPtrVSpec(0, 4, GL_FLOAT, false, n_stride, sizeof(Vector4f) + n_first_vertex_next * n_stride),
						GLVertexAttribPtrVSpec(1, 4, GL_FLOAT, false, n_stride, 0 + n_first_vertex_next * n_stride)), true);
					// draw the cameras being added

					glDepthMask(1);
				}
				// t_odo - actually only need an index buffer as long as the longest frame

				/*if(b_swapped) {
					b_swapped = false;
					std::swap(n_vertex_num_next, n_vertex_num);
					std::swap(n_first_vertex_next, n_first_vertex);
					f_sub_frame = 1 - f_sub_frame;
				}*/
				// swap back

				if(f_camera_tex_alpha < 1 - 1.0f / 256)
					glDepthMask(1);
			}
			// draw camera images
		}
	}
};

/**
 *	@brief algorithm for streaming edge geometries
 */
class CEdgesStreamer {
protected:
	CGLElementArraySetup *m_p_edges_array; /**< @brief vertex array with edges */
	//bool m_b_selection_dirty;

public:
	CEdgesStreamer()
		:m_p_edges_array(0)//, m_b_selection_dirty(true)
	{}

	~CEdgesStreamer()
	{
		Free_GLBuffers();
	}

	void Free_GLBuffers()
	{
		if(m_p_edges_array)
			delete m_p_edges_array;
		m_p_edges_array = 0;
	}

	void On_SelectionChanged()
	{
		//m_b_selection_dirty = true;
		Free_GLBuffers(); // just do that, much easier
	}

	void Draw_Edges(const Matrix4f &t_mvp, const TSolutionFrameInfo &t_frame,
		const CSolutionData &r_solution, const CGraphData &r_graph,
		bool b_edges_blend_add, float f_edge_alpha, bool b_bright_edges,
		const std::vector<size_t> &selection_buffer,
		const TConstantColorShader &const_color_shader)
	{
		size_t n_cur_frame = t_frame.n_cur_frame;
		size_t n_next_frame = t_frame.n_next_frame;
		double f_sub_frame = t_frame.f_sub_frame; // not const, will want to swap them

		glBlendFunc(GL_SRC_ALPHA, (b_bright_edges && b_edges_blend_add)? GL_ONE : GL_ONE_MINUS_SRC_ALPHA);
		//glBlendFunc(GL_SRC_ALPHA_SATURATE, GL_ONE); // nope, doesn't quite do what we want
		glEnable(GL_BLEND);
		glDisable(GL_DEPTH_TEST);
		//glDepthMask(GL_FALSE);
		glDisable(0x0B20/*GL_LINE_SMOOTH*/); // otherwise really costy
		if(b_bright_edges)
			const_color_shader.Bind(t_mvp, 1, 1, 1, f_edge_alpha);
		else
			const_color_shader.Bind(t_mvp, 0, 0, 0, f_edge_alpha);

		/*if(m_b_selection_dirty && m_p_edges_array) {
			delete m_p_edges_array;
			m_p_edges_array = 0;
		}
		m_b_selection_dirty = false;*/
		if(r_solution.vertex_range_list.size() < 2) {
			if(!m_p_edges_array) {
				std::vector<Vector3f> vertex_data;
				std::vector<uint32_t> index_list;
				{
					std::map<size_t, size_t> camera_index_map, vertex_index_map;
					for(size_t i = 0, n = r_graph.edge_list.size(), m = r_solution.vertex_list.size(); i < n; ++ i) {
						std::pair<size_t, size_t> e = r_graph.edge_list[i];
						if(!selection_buffer.empty()) {
							if(!stl_ut::t_Sorted_Find(selection_buffer.begin(), selection_buffer.end(), e.first).first &&
							   !stl_ut::t_Sorted_Find(selection_buffer.begin(), selection_buffer.end(), e.second).first)
								continue;
							// if none of the edge vertices are selected, do not display the edge
						}

						if(e.first < m && e.second < m && r_solution.vertex_list[e.first].b_Has_Position() &&
						   r_solution.vertex_list[e.second].b_Has_Position()) {
							for(int j = 0; j < 2; ++ j) {
								size_t n_vertex = (!j)? e.first : e.second, n_draw_index;
								if(!vertex_index_map.count(n_vertex)) {
									vertex_index_map[n_vertex] = n_draw_index = vertex_data.size();
									vertex_data.push_back(r_solution.vertex_list[n_vertex].v_Position());
								} else
									n_draw_index = vertex_index_map[n_vertex];
								_ASSERTE(n_draw_index <= UINT32_MAX);
								index_list.push_back(uint32_t(n_draw_index));
							}
						}
						// if both vertices are in the list and both have positions
						// (nevermind if these are both cameras or points)
					}
				}
				m_p_edges_array = new CGLElementArraySetup((vertex_data.empty())? 0 : &vertex_data[0],
					vertex_data.size() * sizeof(Vector3f), 0, GL_FLOAT, 3, 0, 0, 0, 0,
					(index_list.empty())? 0 : &index_list[0], index_list.size(), GL_UNSIGNED_INT, GL_LINES);
			}
			m_p_edges_array->Draw();
		} else {
			// todo - draw animated edges to vertices in the current frame. note that selection is disabled, at least for now.
		}
		glEnable(0x0B20/*GL_LINE_SMOOTH*/); // otherwise really costy
		glEnable(GL_DEPTH_TEST);
		//glDepthMask(GL_TRUE);
		glDisable(GL_BLEND);
	}
};

#endif // !__GEOMETRY_STREAMING_ALGOS_INCLUDED
