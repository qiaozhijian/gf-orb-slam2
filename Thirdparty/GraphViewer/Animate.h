/*
								+--------------------------------+
								|                                |
								|*** Camera animation support ***|
								|                                |
								|  Copyright © -tHE SWINe- 2014  |
								|                                |
								|           Animate.h            |
								|                                |
								+--------------------------------+
*/

#pragma once
#ifndef __CAMERA_SPLINE_ANIMATION_UTILS_INCLUDED
#define __CAMERA_SPLINE_ANIMATION_UTILS_INCLUDED

/**
 *	@file Animate.h
 *	@brief a simple camera animation support using Kochanek-Bartels splines
 *	@author -tHE SWINe-
 *	@date 2014
 */

#include "../UberLame_src/Vector.h"
#include "../UberLame_src/lml/KochanekBartels.h"
#include "../UberLame_src/Timer.h"
#include "../UberLame_src/StlUtils.h"

// todo - write code snippets one needs to include to support animations

class CUnrealTimer {
protected:
	mutable CDeltaTimer m_timer;
	mutable double m_f_time;
	double m_f_speed;
	bool m_b_paused;

public:
	/**
	 *	@brief default constructor
	 */
	inline CUnrealTimer()
		:m_f_time(0), m_f_speed(1.0), m_b_paused(false)
	{}

	/**
	 *	@brief resets timer (sets time to zero)
	 *	@note This does not affect pause state.
	 */
	inline void Reset()
	{
		m_f_time = 0;
		m_timer.Reset();
	}

	/**
	 *	@brief applies offset to the current time; this offset is deleted upon calling Reset()
	 *	@param[in] f_additive_offset is additive time offset, in seconds
	 */
	inline void Offset(double f_additive_offset)
	{
		m_f_time += f_additive_offset;
	}

	/**
	 *	@brief resets timer (sets time to zero)
	 *	@deprecated This function is deprecated in favor of Reset().
	 */
	inline void ResetTimer()
	{
		Reset();
	}

	inline double f_Speed() const
	{
		return m_f_speed;
	}

	inline void SetSpeed(double f_speed = 1.0)
	{
		f_Time(); // force time sample before changing the speed
		m_f_speed = f_speed;
	}

	inline bool b_Paused() const
	{
		return m_b_paused;
	}

	inline void Pause(bool b_paused)
	{
		f_Time(); // force delta timer sample before changing the pause state
		// (increments by the time until now if not paused, or starts timing the time after unpaused otherwise)

		m_b_paused = b_paused;
	}

	/**
	 *	@brief gets time in seconds
	 *	@return Returns time since creation of this object or since
	 *		the last call to ResetTimer(), in seconds.
	 *	@note This should cope nicely with counter overflows.
	 */
	inline double f_Time() const
	{
		if(!m_b_paused)
			m_f_time += m_f_speed * m_timer.f_Time();
		else
			m_timer.Reset(); // do the sample!
		return m_f_time;
	}

	/**
	 *	@brief gets timer frequency
	 *	@return Returns timer frequency (inverse of the smallest time step).
	 */
	inline int64_t n_Frequency() const
	{
		return m_timer.n_Frequency();
	}
};

class CCameraAnimation {
public:
	typedef CKochanekBartelsQuatSpline<Vector3f> _TySpline;
	typedef _TySpline::_TyKeyPoint _TyKeyPoint;
	typedef _TySpline::_TyPoint _TyPoint;
	typedef CSplineSampler_ExplicitTime_UniformStep<_TySpline> _TySampler;

protected:
	CUnrealTimer m_timer;
	std::vector<_TyKeyPoint> m_restore_keypoint_list;
	std::vector<_TyKeyPoint> m_keypoint_list;
	_TySpline m_spline;
	stl_ut::CUniquePtr<_TySampler> m_p_sampler; // todo - no need to be a pointer
	bool m_b_animate;
	bool m_b_extrapolate;
	double m_f_anim_poses_offset;

public:
	CCameraAnimation()
		:m_b_animate(false), m_b_extrapolate(false), m_f_anim_poses_offset(0)
	{}

	void onTogglePause()
	{
		m_timer.Pause(!m_timer.b_Paused());
	}

	bool b_Paused() const
	{
		return m_timer.b_Paused();
	}

	double f_Time() const
	{
		return m_timer.f_Time();
	}

	void onPushSplinePose(double f_time, Vector3f v_camera_pos, float f_angle_x,
		float f_angle_y, float f_angle_z, bool b_trackball_style = false)
	{
		Matrix4f t_camera_matrix;
		t_camera_matrix.Identity();
		if(b_trackball_style)
			t_camera_matrix.Translate(v_camera_pos);
		t_camera_matrix.RotateZ(f_angle_z);
		t_camera_matrix.RotateX(f_angle_x);
		t_camera_matrix.RotateY(f_angle_y);
		if(!b_trackball_style)
			t_camera_matrix.Translate(v_camera_pos);
		// calculate final camera matrix (with pitch and roll)

		onPushSplinePose(f_time, t_camera_matrix);
		// calls below
	}

	void onPushSplinePose(double f_time, Matrix4f t_camera_matrix, float f_tension = 0, float f_continuity = 0, float f_bias = 0) // throw(std::bad_alloc)
	{
		if(m_keypoint_list.empty())
			m_f_anim_poses_offset = f_time;
		// offset the time so that the first keypoint would always be at time zero

		_TyKeyPoint kp(-t_camera_matrix.t_Inverse().v_Transform_Pos(Vector3f(0, 0, 0)),
			Quatf(t_camera_matrix.t_RotationPart()).t_Positive(), f_time - m_f_anim_poses_offset, f_tension, f_continuity, f_bias);
		// make a keypoint

		m_keypoint_list.push_back(kp);
		// put it in the spline
	}

	void onPushAnimation() // a bit like reset but the current animation will be popped back once the new one finishes playing (or could just make several animation object instances)
	{
		if(!m_restore_keypoint_list.empty())
			fprintf(stderr, "warning: animation push overwrites existing keyframes\n");
		m_b_animate = false;
		m_b_extrapolate = false;
		m_p_sampler.Destroy();
		m_restore_keypoint_list = m_keypoint_list;
		m_keypoint_list.clear();
		m_timer.Reset();
	}

	void onPopAnimation()
	{
		if(!m_restore_keypoint_list.empty()) {
			m_keypoint_list = m_restore_keypoint_list;
			m_restore_keypoint_list.clear();
			// pop animation

			m_p_sampler.Destroy(); // don't remember that
			m_timer.Reset();

#ifdef _DEBUG
			printf("debug: popping previous animation (" PRIsize " keyframes)\n", m_keypoint_list.size());
#endif // _DEBUG
		}
	}

	void onResetAnimation()
	{
		m_b_animate = false;
		m_b_extrapolate = false;
		m_p_sampler.Destroy();
		m_keypoint_list.clear();
		m_timer.Reset();

		onPopAnimation();
	}

	void onLoadAnimation(const char *p_s_filename = "animation.txt") // throw(std::bad_alloc)
	{
		onResetAnimation();
		FILE *p_fr;
#if defined(_MSC_VER) && !defined(__MWERKS__) && _MSC_VER >= 1400
		if(fopen_s(&p_fr, p_s_filename, "r"))
#else //_MSC_VER && !__MWERKS__ && _MSC_VER >= 1400
		if(!(p_fr = fopen(p_s_filename, "r")))
#endif //_MSC_VER && !__MWERKS__ && _MSC_VER >= 1400
			return;
		size_t n_point_num;
		fscanf(p_fr, PRIsize "\n", &n_point_num);
		std::vector<_TyKeyPoint> kps(n_point_num);
		for(size_t i = 0; i < n_point_num; ++ i) {
			_TyKeyPoint &kp = kps[i];
			fscanf(p_fr, "%f %f %f %f %f %f %f %f %f %f %f\n",
				&kp.f_time, &kp.f_tension, &kp.f_continuity, &kp.f_bias,
				&kp.v_position.x, &kp.v_position.y, &kp.v_position.z,
				&kp.t_rotation.x, &kp.t_rotation.y, &kp.t_rotation.z, &kp.t_rotation.w);
		}
		fclose(p_fr);
		printf("loaded " PRIsize " keypoints\n", n_point_num);
		m_keypoint_list.clear();
		m_keypoint_list.insert(m_keypoint_list.begin(), kps.begin(), kps.end());
		// load animation data

		m_spline.Erase();
		m_spline.InsertBack(m_keypoint_list.begin(), m_keypoint_list.end());
		if(!m_spline.b_Empty()) {
			m_spline.Insert(0, m_spline.r_Point(0));
			m_spline.PushBack(m_spline.r_Point(m_spline.n_Point_Num() - 1));
		}
		// duplicate the first and the last point

		m_p_sampler = stl_ut::CUniquePtr<_TySampler>(new _TySampler(m_spline));
		// prepare a sampler for rendering

		m_b_animate = false;
		m_b_extrapolate = false;
		// do not run the animation just yet
	}

	void onSaveAnimation(const char *p_s_filename = "animation.txt") const
	{
		FILE *p_fw;
#if defined(_MSC_VER) && !defined(__MWERKS__) && _MSC_VER >= 1400
		if(fopen_s(&p_fw, p_s_filename, "w"))
#else //_MSC_VER && !__MWERKS__ && _MSC_VER >= 1400
		if(!(p_fw = fopen(p_s_filename, "w")))
#endif //_MSC_VER && !__MWERKS__ && _MSC_VER >= 1400
			return;
		fprintf(p_fw, PRIsize "\n", m_keypoint_list.size());
		for(size_t i = 0, n = m_keypoint_list.size(); i < n; ++ i) {
			const _TyKeyPoint &kp = m_keypoint_list[i];
			fprintf(p_fw, "%.15f %.15f %.15f %.15f %.15f %.15f %.15f %.15f %.15f %.15f %.15f\n",
				kp.f_time, kp.f_tension, kp.f_continuity, kp.f_bias,
				kp.v_position.x, kp.v_position.y, kp.v_position.z,
				kp.t_rotation.x, kp.t_rotation.y, kp.t_rotation.z, kp.t_rotation.w);
		}
		fclose(p_fw);
		printf("saved " PRIsize " keypoints\n", m_keypoint_list.size());
	}

	float f_AnimationLength() const
	{
		return (m_keypoint_list.empty())? 0 : m_keypoint_list.back().f_time;
	}

	bool b_Empty() const
	{
		return m_keypoint_list.empty();
	}

	void onPlayAnimation() // throw(std::bad_alloc)
	{
		if(m_keypoint_list.empty())
			return; // can't animate
		m_b_animate = true;
		m_spline.Erase();
		m_spline.InsertBack(m_keypoint_list.begin(), m_keypoint_list.end());
		if(!m_spline.b_Empty()) {
			m_spline.Insert(0, m_spline.r_Point(0));
			m_spline.PushBack(m_spline.r_Point(m_spline.n_Point_Num() - 1));
		}
		//float f_len = m_spline.f_Length(); // this takes some time, if done with great precision
		// duplicate the first and the last point
		m_p_sampler = stl_ut::CUniquePtr<_TySampler>(new _TySampler(m_spline));
		m_timer.Reset();
	}

	void onStopAnimation()
	{
		m_b_animate = false;
	}

	void onEnableExtrapolation()
	{
		m_b_extrapolate = true;
	}

	void onDisableExtrapolation()
	{
		m_b_extrapolate = true;
	}

	bool b_Active() const
	{
		return m_b_animate;
	}

	const size_t n_Real_KeyFrame_Num() const
	{
		return m_keypoint_list.size();
	}

	const size_t n_KeyFrame_Num() const
	{
		return m_spline.n_Point_Num();
	}

	const _TyKeyPoint &t_KeyFrame(size_t n_index) const
	{
		return m_spline.r_Point(n_index);
	}

	// this is used for debug rendering of the animation spline
	_TyPoint t_Interpolate2(double f_time) // not const, may need to set up the sampler
	{
		if(m_keypoint_list.empty())
			return _TyPoint(Vector3f(0, 0, 0), Quatf(0, 0, 0, 1));
		// handle empty

		if(m_spline.n_Point_Num() != m_keypoint_list.size() + 2) {
			m_spline.Erase();
			m_spline.InsertBack(m_keypoint_list.begin(), m_keypoint_list.end());
			if(!m_spline.b_Empty()) {
				m_spline.Insert(0, m_spline.r_Point(0));
				m_spline.PushBack(m_spline.r_Point(m_spline.n_Point_Num() - 1));
			}
		}
		// make sure the spline is up to date

		if(!m_p_sampler)
			m_p_sampler = stl_ut::CUniquePtr<_TySampler>(new _TySampler(m_spline));
		// prepare a sampler for interpolation (no need to reallocate it as it uses
		// constant time sampling and the sampler itself has no significant state)

		if(!m_b_extrapolate && m_spline.r_Point(m_spline.n_Point_Num() - 1).f_time < f_time) {
			_TyKeyPoint p = m_spline.r_Point(m_spline.n_Point_Num() - 1); // stay at the last keypoint
			_TyPoint pt;
			pt.t_rotation = p.t_rotation;
			pt.v_position = p.v_position;

			m_b_animate = false; // end animation after it is finished
			onPopAnimation();

			return pt;
		} else
			return m_p_sampler->v_Position(float(f_time)); // extrapolate
	}

	// this is used for camera matrix
	Matrix4f t_Interpolate(double f_time) // not const, changes m_b_animate once it hits the end frame
	{
		Matrix4f t_camera_matrix;
		t_camera_matrix.Identity();
		if(m_b_animate) { // this handles empty splines
			_ASSERTE(m_p_sampler);
			if(!m_b_extrapolate && m_spline.r_Point(m_spline.n_Point_Num() - 1).f_time < f_time) {
				_TyKeyPoint p = m_spline.r_Point(m_spline.n_Point_Num() - 1); // stay at the last keypoint
				t_camera_matrix.Set_RotationPart(p.t_rotation.t_ToMatrix());
				t_camera_matrix.Translate(p.v_position);

				m_b_animate = false; // end animation after it is finished
				onPopAnimation();
			} else {
				_TyPoint p = m_p_sampler->v_Position(float(f_time));
				t_camera_matrix.Set_RotationPart(p.t_rotation.t_ToMatrix());
				t_camera_matrix.Translate(p.v_position);
			}
		}
		// sample and convert to a matrix

		return t_camera_matrix;
	}

	static void PrintHelp()
	{
		printf("animation controls:\n"
			"n: onPushSplinePose()\n"
			"m: onPlayAnimation()\n"
			",: onTogglePause()\n"
			".: onStopAnimation()\n"
			"j: onResetAnimation()\n"
			"k: onSaveAnimation()\n"
			"l: onLoadAnimation()\n");
	}

#ifdef __ON_KEYPRESS_BIND_TO_GLOBALS
	static bool onKeyPress(CCameraAnimation &camera, int n_key)
#else // __ON_KEYPRESS_BIND_TO_GLOBALS
	static bool onKeyPress(CCameraAnimation &camera, int n_key, Matrix4f t_current_camera_matrix)
#endif // __ON_KEYPRESS_BIND_TO_GLOBALS
	{
		switch(n_key) {
		case 'n':
			{
#ifndef __ON_KEYPRESS_BIND_TO_GLOBALS
				const Matrix4f &t_camera_matrix = t_current_camera_matrix; // rename
#else // !__ON_KEYPRESS_BIND_TO_GLOBALS
				Matrix4f t_camera_matrix;
				t_camera_matrix.Identity();
#ifdef __GL_WINDOW_TRACKBALL_STYLE_CAMERA
				t_camera_matrix.Translate(v_camera_pos);
#endif // __GL_WINDOW_TRACKBALL_STYLE_CAMERA
				t_camera_matrix.RotateZ(f_angle_z);
				t_camera_matrix.RotateX(f_angle_x);
				t_camera_matrix.RotateY(f_angle_y);
#ifndef __GL_WINDOW_TRACKBALL_STYLE_CAMERA
				t_camera_matrix.Translate(v_camera_pos);
#endif // !__GL_WINDOW_TRACKBALL_STYLE_CAMERA
				// or make the camera from global variables (not entirely portable / reusable)
#endif // !__ON_KEYPRESS_BIND_TO_GLOBALS

				camera.onPushSplinePose(camera.f_Time(), t_camera_matrix);
			}
			break;
		case 'm':
			camera.onPlayAnimation();
			if(camera.b_Paused())
				camera.onTogglePause();
			break;
		case 'j':
			camera.onResetAnimation();
			if(camera.b_Paused())
				camera.onTogglePause();
			break;
		case 'k':
			camera.onSaveAnimation();
			break;
		case 'l':
			camera.onLoadAnimation();
			break;
		case ',':
			camera.onTogglePause();
			break;
		case '.':
			camera.onStopAnimation();
			break;
		default:
			return false; // event not handled
		}
		return true;
	}

private:
	CCameraAnimation(const CCameraAnimation &r_other); // no-copy
	CCameraAnimation &operator =(const CCameraAnimation &r_other); // no-copy
};

#include "MT_PNG_Codec.h"

/**
 *	@brief nice offscreen renderer with temporal and spatial antialiassing
 */
class CGLThreadedFrameWriter {
public:
	enum {
		n_spatial_supersample_num = 4, // this is actually sqrt of the number of samples
		n_omni_spatial_supersample_num = 2, // this is actually sqrt of the number of samples
		n_temporal_supersample_num = 32 // temporal AA
	};

protected:
	struct TAccumShader : public CGLESShader {
		GLint n_texture_unit_uniform;
		GLint n_weight_uniform;

		bool Compile()
		{
			const char *p_s_vertex_shader =
				"precision highp float;\n"
				"attribute vec2 v_pos;\n"
				"\n"
				"varying vec2 v_texcoord;\n"
				"\n"
				"void main()\n"
				"{\n"
				"    gl_Position = vec4(v_pos, 0.0, 1.0);\n"
				"    v_texcoord = v_pos * .5 + .5;\n"
				"}\n";
			const char *p_s_fragment_shader =
				"precision highp float;\n"
				"\n"
				"varying vec2 v_texcoord;\n"
				"\n"
				"uniform float f_weight;\n"
				"uniform sampler2D n_texture_unit;\n"
				"\n"
				"void main()\n"
				"{\n"
				"    gl_FragColor = texture2D(n_texture_unit, v_texcoord) * f_weight;\n"
				"}\n";
			// vertex / fragment shader source code

			const char *p_s_config =
				"vertex {\n"
				"	v_pos: 0;\n"
				"}\n";
			// shader configuration (doesn't actually need the newlines)
			// note this can't override layout qualifiers in the shader code

			std::string compile_log, config_log, link_log;
			if(!CompileConfigureLink(p_s_vertex_shader, p_s_fragment_shader,
			   p_s_config, compile_log, config_log, link_log, true))
				return false;
			// use the comfy function

			n_texture_unit_uniform = n_Get_Uniform_Location("n_texture_unit");
			n_weight_uniform = n_Get_Uniform_Location("f_weight");
			// get addresses of the uniforms

			CGLESShader::Bind();
			// bind the shader

			Uniform1i(n_texture_unit_uniform, 0);
			// always assume the texture in unit 0

			return true;
		}

		void Bind(float f_weight) const
		{
			CGLESShader::Bind();
			// bind the shader

			Uniform1f(n_weight_uniform, f_weight);
			// set the weight
		}
	};

	struct TAccumShader_Omni : public CGLESShader {
		GLint n_texture_unit_uniform;
		GLint n_weight_uniform, n_jitter_uniform;

		bool Compile()
		{
			const char *p_s_vertex_shader =
				"precision highp float;\n"
				"attribute vec2 v_pos;\n"
				"\n"
				"varying vec2 v_texcoord;\n"
				"\n"
				"uniform vec2 v_jitter;\n"
				"\n"
				"void main()\n"
				"{\n"
				"    gl_Position = vec4(v_pos, 0.0, 1.0);\n"
				"    v_texcoord = (v_pos + v_jitter) * .5 + .5;\n" // jitter in texcoords instead, this fixes the vertical seam
				"}\n";
			const char *p_s_fragment_shader =
				"precision highp float;\n"
				"\n"
				"varying vec2 v_texcoord;\n"
				"\n"
				"uniform float f_weight;\n"
				"uniform samplerCube n_texture_unit;\n"
				"\n"
				"void main()\n"
				"{\n"
				"    vec2 v_ntx = vec2(v_texcoord.x * -2.0 + 1.0, v_texcoord.y - .5) *\n" // x is [-1, +1] and flipped, y is [-.5, .5]
				"        3.1415926535897932384626433832795028841971;\n"
				// normalized texture coordinates in radians, with zero meridian and zero parallel in the middle

				"    vec2 v_lonlat = vec2(v_ntx.x / 1.0 + 3.1415926535897932384626433832795028841971 / 2.0, v_ntx.y + .0);\n"
				// no shift applied (1.0 corresponds to cosine of the central meridian,
				// pi/2 and .0 correspond to longitude and latitude of the origin, respectively)

				/*"    v_lonlat.y /= 3.1415926535897932384626433832795028841971;\n"
				"    v_lonlat.y *= 2.0;\n"
				"    v_lonlat.y = asin(v_lonlat.y);\n"*/ // stretch towards the poles
				/*"    v_lonlat.y = sin(v_lonlat.y) * .5 * 3.1415926535897932384626433832795028841971;\n"*/ // compress away from poles
				// correction needed? nopenope. just make sure that handbrake does not crop it! that's the root of evil.

				//"    vec3 v_dir = sin(v_lonlat.y) * vec3(cos(v_lonlat.x), sin(v_lonlat.x), 1.0);\n" // wiki
				"    vec3 v_dir = vec3(cos(v_lonlat.y) * cos(v_lonlat.x), sin(v_lonlat.y) * -1.0, cos(v_lonlat.y) * sin(v_lonlat.x));\n" // should it be like this?
				// convert to spherical for sampling

				//"    gl_FragColor = (vec4(normalize(v_dir) * .5 + .5, 1.0) * .995 + .005 * texture(n_texture_unit, v_dir)) * f_weight;\n"
				// debug - see coords as colors

				"    gl_FragColor = texture(n_texture_unit, v_dir) * f_weight;\n"
				"}\n";
			// vertex / fragment shader source code

			const char *p_s_config =
				"vertex {\n"
				"	v_pos: 0;\n"
				"}\n";
			// shader configuration (doesn't actually need the newlines)
			// note this can't override layout qualifiers in the shader code

			std::string compile_log, config_log, link_log;
			if(!CompileConfigureLink(p_s_vertex_shader, p_s_fragment_shader,
			   p_s_config, compile_log, config_log, link_log, true))
				return false;
			// use the comfy function

			n_texture_unit_uniform = n_Get_Uniform_Location("n_texture_unit");
			n_weight_uniform = n_Get_Uniform_Location("f_weight");
			n_jitter_uniform = n_Get_Uniform_Location("v_jitter");
			// get addresses of the uniforms

			CGLESShader::Bind();
			// bind the shader

			Uniform1i(n_texture_unit_uniform, 0);
			// always assume the texture in unit 0

			return true;
		}

		void Bind(float f_weight, Vector2f v_jitter = Vector2f(0, 0)) const
		{
			CGLESShader::Bind();
			// bind the shader

			Uniform1f(n_weight_uniform, f_weight);
			Uniform2f(n_jitter_uniform, v_jitter.x, v_jitter.y);
			// set the weight
		}
	};

protected:
	int m_n_width, m_n_height;
	float m_f_fps;

public:
	CGLThreadedFrameWriter()
		:m_n_width(1280), m_n_height(720), m_f_fps(48)
	{}

	void Set_Resolution(int n_width, int n_height) // e.g. 1920 x 1080 or 1280 x 720
	{
		_ASSERTE(n_width > 0 && n_height > 0);
		m_n_width = n_width;
		m_n_height = n_height;
	}

	void Set_Framerate(float f_fps) // e.g. 48 or 23.976
	{
		m_f_fps = f_fps;
	}

	template <class CRenderFrame>
	void Render_Single_Omni(const char *p_s_filename, CRenderFrame renderer,
		double f_time, bool b_need_warmup = false) // throw(std::bad_alloc, std::runtime_error)
	{
		const double f_start = f_time, f_elapse = 1.0 / m_f_fps;
		_ASSERTE(m_f_fps > 0);
		_ASSERTE(f_start >= 0);
		_ASSERTE(f_elapse >= 0);

#ifndef GL_TEXTURE_CUBE_MAP_SEAMLESS
#define GL_TEXTURE_CUBE_MAP_SEAMLESS 0x884F 
#endif // !GL_TEXTURE_CUBE_MAP_SEAMLESS
		if(CGLES20ExtensionHandler::b_SupportedExtension("GL_ARB_seamless_cube_map"))
			glEnable(GL_TEXTURE_CUBE_MAP_SEAMLESS);
		// use seamless cube-map filtering where available

		int p_wp[4];
		glGetIntegerv(GL_VIEWPORT, p_wp);
		const int n_width = p_wp[2], n_height = p_wp[3];
		// remember the original viewport

		double f_area = double(m_n_width) * m_n_height;
		double f_cube_face_area = f_area / 6;
		double f_cube_face_size = sqrt(f_cube_face_area) * 1.225;
		// choose size of a single cube face

		int n_max_cube_size;
		glGetIntegerv(GL_MAX_CUBE_MAP_TEXTURE_SIZE, &n_max_cube_size);
		const int n_cube_size = min(n_max_cube_size, n_Align_Up_POT(max(int(f_cube_face_size), 16), 16));
		// decide on cube map resolution

		printf("debug: chosen cube size %d (area " PRIvalueMP "pix, needed " PRIvalueMP "pix, ratio %g)\n",
			n_cube_size, PRIvalueMPparams(double(n_cube_size) * n_cube_size * 6),
			PRIvalueMPparams(f_area), double(n_cube_size) * n_cube_size * 6 / f_area);

		const int n_max_fbo_dim = CGLFrameBufferObject::n_Get_MaxSize();
		const int n_supersample_limit = n_max_fbo_dim / n_cube_size;
		const int n_AA_scale = min((int)n_omni_spatial_supersample_num, n_supersample_limit);
		// decide on spatial antialiassing

		const int n_fbo_width2 = n_cube_size * n_AA_scale; // make this square for cube-maps
		const int n_fbo_height2 = n_cube_size * n_AA_scale; // render at high resolution
		const int n_fbo_width = m_n_width;
		const int n_fbo_height = m_n_height; // store at lower to facilitate spatial antialiassing
		const bool b_use_mipmaps = true;//float(n_fbo_width2) / n_fbo_width > 2;
		// t_odo - make this setting part of the object

		printf("debug: AA cube size %d (memory " PRIsizeB "B)\n", n_fbo_width2,
			PRIsizeBparams(double(n_cube_size) * n_cube_size * 6 * sizeof(uint32_t) * ((b_use_mipmaps)? 1 + 1.0 / 3 : 1)));

		static const float p_fs_quad[] = {-1, 1, -1, -1, 1, 1, 1, -1};
		CGLArraySetup fs_quad(p_fs_quad, sizeof(p_fs_quad), 0, GL_FLOAT, 2, 0, 0, 0, 0, GL_TRIANGLE_STRIP);
		// need a fullscreen quad for accumulation

		glActiveTexture(GL_TEXTURE0);
		// !!

		CGLFrameBufferObject fbo(n_fbo_width2, n_fbo_height2, 1, true, 0, 0, // render to texture
			true, false, GL_DEPTH_COMPONENT24_OES, 0,
			false, false, 0, 0);
		CGLTexture_Cube color_tex(n_fbo_width2, /*n_fbo_height2,*/ GL_RGBA8_OES, b_use_mipmaps);
		CGLFrameBufferObject accum_fbo(n_fbo_width, n_fbo_height, 1,
			false, /*GL_RGBA32F*/0x8814, 0, false, false, 0, 0); // GL_RGBA32F not in OES yet
		TAccumShader_Omni accum_shader_omni;
		if(!fbo.b_Status() || !color_tex.b_Status() ||
		   !accum_fbo.b_Status() || !accum_shader_omni.Compile() || !fs_quad.b_Status()) {
			fprintf(stderr, "error: failed to init FBO\n");
			return;
		}
		// make a framebuffer, texture and other floating point framebuffer for accumulation

		color_tex.Bind();
		if(CGLES20ExtensionHandler::b_SupportedExtension("GL_EXT_texture_filter_anisotropic")) {
			float f_anisotropy;
			glGetFloatv(GL_MAX_TEXTURE_MAX_ANISOTROPY_EXT, &f_anisotropy);
			glTexParameterf(color_tex.n_Target(), GL_TEXTURE_MAX_ANISOTROPY_EXT, f_anisotropy);
		}

		const size_t n_frame_num = 1; // a single frame
		const double f_frame_time = 1 / m_f_fps;

		size_t n_worker_num = 1; // one is enough, we have just a single frame to render
		// get number of threads

		CMTEncoder encoder(n_fbo_width, n_fbo_height, true, true, n_worker_num);
		// starts the threads, throws exceptions if low on memory or if the threads failed to start

		Matrix4f t_front, t_left, t_back, t_right, t_bottom, t_top;
		t_front.Identity();
		t_bottom.RotationX(-f_pi / 2); // todo - add other rotations?
		t_top.RotationX(f_pi / 2); // todo - add other rotations?
		t_left.RotationY(-f_pi / 2); //t_left.Scale(-1, 1, 1); // todo - add other rotations?
		t_back.RotationY(f_pi); // todo - add other rotations?
		t_right.RotationY(f_pi / 2); //t_right.Scale(-1, 1, 1); // todo - add other rotations?

		const Matrix4f p_cube_views[] = {
			t_front, t_left, t_back, t_right, t_bottom, t_top
		};
		const GLenum p_cube_faces[] = {
			GL_TEXTURE_CUBE_MAP_POSITIVE_Z, // front
			GL_TEXTURE_CUBE_MAP_NEGATIVE_X, // left
			GL_TEXTURE_CUBE_MAP_NEGATIVE_Z, // back
			GL_TEXTURE_CUBE_MAP_POSITIVE_X, // right
			GL_TEXTURE_CUBE_MAP_NEGATIVE_Y, // bottom
			GL_TEXTURE_CUBE_MAP_POSITIVE_Y // top
		};
		// cube map faces and the corresponding view transforms

		{
			if(f_start > 0 && b_need_warmup) {
				for(size_t i = 0, n = ceil(f_start * 10); i < n; ++ i) {
					for(int j = 0; j < 6; ++ j)
						renderer((i + 1) * (f_start / n), p_cube_views[j]);
				}
			}
			// render the start frames but throw them away. this is required by the animation
			// system, which cannot easily skip time

			glActiveTexture(GL_TEXTURE0);
			// !!

			float p_ss_weights[n_temporal_supersample_num];
			{
				float f_ss_weight_sum = 0;
				for(int j = 0; j < n_temporal_supersample_num; ++ j) {
					p_ss_weights[j] = int(n_temporal_supersample_num - 2 * fabs(j - (n_temporal_supersample_num - 1) / 2.0)); // flat
					_ASSERTE(p_ss_weights[j] >= 0 && p_ss_weights[j] < 255); // fit into uint8_t (MMX / SSE)
					f_ss_weight_sum += p_ss_weights[j];
				}
				//_ASSERTE(p_ss_weights[0] < 1e-3f); // the first one is zero
				const float f_inv_sum = 1.0 / f_ss_weight_sum;
				for(int j = 0; j < n_temporal_supersample_num; ++ j)
					p_ss_weights[j] *= f_inv_sum;
			}
			// prepare for temporal supersampling

			std::string s_filename;
			for(size_t i = 0; i < n_frame_num; ++ i) {
				TBmp *p_bitmap = encoder.p_Get_EmptyBitmap(); // there is one extra bitmap
				for(int j = 0; j < n_temporal_supersample_num; ++ j) {
					for(int k = 0; k < 6; ++ k) {
						fbo.Bind();
						fbo.Bind_ColorTextureFace(0, color_tex, 0, p_cube_faces[k]);
						glViewport(0, 0, n_fbo_width2, n_fbo_height2);
						{
							renderer(f_start + (i + float(j) / (n_temporal_supersample_num + 1)) * f_frame_time,
								p_cube_views[k]);
						}
						fbo.Bind_ColorTexture(0, 0, 0);
						fbo.Release();
					}
					// draw one frame, several times to each cube face

					glActiveTexture(GL_TEXTURE0);
					// !!

					accum_fbo.Bind(GL_FRAMEBUFFER); // !! otherwise OpenGL 4+ won't read from the FBO but from the backbuffer instead
					glViewport(0, 0, n_fbo_width, n_fbo_height); // rescaling done on GPU
					{
						color_tex.Bind();
						if(b_use_mipmaps)
							glGenerateMipmap(color_tex.n_Target());
						if(!j) {
							glClearColor(0, 0, 0, 0);
							glClear(GL_COLOR_BUFFER_BIT);
						}
						// clear the accumulator
						
						static const Vector2f p_jitter[] = {
							Vector2f( 0.285561,  0.188437), Vector2f( 0.360176, -0.065688),
							Vector2f(-0.111751,  0.275019), Vector2f(-0.055918, -0.215197),
							Vector2f(-0.080231, -0.470965), Vector2f( 0.138721,  0.409168),
							Vector2f( 0.384120,  0.458500), Vector2f(-0.454968,  0.134088),
							Vector2f( 0.179271, -0.331196), Vector2f(-0.307049, -0.364927),
							Vector2f( 0.105354, -0.010099), Vector2f(-0.154180,  0.021794),
							Vector2f(-0.370135, -0.116425), Vector2f( 0.451636, -0.300013),
							Vector2f(-0.370610,  0.387504) // from the red book (the same as rand() but likely much cheaper)
						};
						static int n_jitter = 0; // a bit uncool, i guess
						n_jitter = (n_jitter + 1) % (sizeof(p_jitter) / sizeof(p_jitter[0]));

						glEnable(GL_BLEND);
						glBlendFunc(GL_ONE, GL_ONE);
						accum_shader_omni.Bind(p_ss_weights[j], // weighted accumulation (the weights sum to one)
							p_jitter[n_jitter] * 2 / Vector2f(n_fbo_width, n_fbo_height)); // apply jitter
						fs_quad.Draw();
						glDisable(GL_BLEND);
						// accumulate the last frame

						if(j + 1 == n_temporal_supersample_num) {
							glFlush();
							glFinish();
							glReadPixels(0, 0, n_fbo_width, n_fbo_height, GL_RGBA, GL_UNSIGNED_BYTE, p_bitmap->p_buffer);
						}
						// read back
					}
					accum_fbo.Release();
					// weighted accumulate in hardware
				}

				if(!encoder.Enqueue_Bitmap(p_s_filename, p_bitmap))
					throw std::runtime_error("queue Put() failed");
				// just fill a work-item
			}
		}
		// render the frames offscreen

		glViewport(0, 0, n_width, n_height);
		// restore the previous viewport

		if(!encoder.Synchronize())
			fprintf(stderr, "error: encoder join failed\n");
		// synchronize with the workers
	}

	template <class CRenderFrame>
	void Render_Single(const char *p_s_filename, CRenderFrame renderer,
		double f_time, bool b_need_warmup = false) // throw(std::bad_alloc, std::runtime_error)
	{
		const double f_start = f_time, f_elapse = 1.0 / m_f_fps;
		_ASSERTE(m_f_fps > 0);
		_ASSERTE(f_start >= 0);
		_ASSERTE(f_elapse >= 0);

		int p_wp[4];
		glGetIntegerv(GL_VIEWPORT, p_wp);
		const int n_width = p_wp[2], n_height = p_wp[3];
		// remember the original viewport

		const int n_max_dim = max(m_n_width, m_n_height);
		const int n_max_fbo_dim = CGLFrameBufferObject::n_Get_MaxSize();
		const int n_supersample_limit = n_max_fbo_dim / n_max_dim;
		const int n_AA_scale = min((int)n_spatial_supersample_num, n_supersample_limit);
		// decide on spatial antialiassing

		const int n_fbo_width2 = m_n_width * n_AA_scale;
		const int n_fbo_height2 = m_n_height * n_AA_scale; // render at high resolution
		const int n_fbo_width = m_n_width;
		const int n_fbo_height = m_n_height; // store at lower to facilitate spatial antialiassing
		const bool b_use_mipmaps = float(n_fbo_width2) / n_fbo_width > 2;
		// t_odo - make this setting part of the object

		static const float p_fs_quad[] = {-1, 1, -1, -1, 1, 1, 1, -1};
		CGLArraySetup fs_quad(p_fs_quad, sizeof(p_fs_quad), 0, GL_FLOAT, 2, 0, 0, 0, 0, GL_TRIANGLE_STRIP);
		// need a fullscreen quad for accumulation

		glActiveTexture(GL_TEXTURE0);
		// !!

		CGLFrameBufferObject fbo(n_fbo_width2, n_fbo_height2, 1, true, 0, 0, // render to texture
			true, false, GL_DEPTH_COMPONENT24_OES, 0,
			false, false, 0, 0);
		CGLTexture_2D color_tex(n_fbo_width2, n_fbo_height2, GL_RGBA8_OES, b_use_mipmaps);
		CGLFrameBufferObject accum_fbo(n_fbo_width, n_fbo_height, 1,
			false, /*GL_RGBA32F*/0x8814, 0, false, false, 0, 0); // GL_RGBA32F not in OES yet
		TAccumShader accum_shader;
		if(!fbo.b_Status() || !color_tex.b_Status() ||
		   !accum_fbo.b_Status() || !accum_shader.Compile() || !fs_quad.b_Status()) {
			fprintf(stderr, "error: failed to init FBO\n");
			return;
		}
		// make a framebuffer, texture and other floating point framebuffer for accumulation

		const size_t n_frame_num = 1; // a single frame
		const double f_frame_time = 1 / m_f_fps;

		size_t n_worker_num = 1; // one is enough, we have just a single frame to render
		// get number of threads

		CMTEncoder encoder(n_fbo_width, n_fbo_height, true, true, n_worker_num);
		// starts the threads, throws exceptions if low on memory or if the threads failed to start

		{
			if(f_start > 0 && b_need_warmup) {
				for(size_t i = 0, n = ceil(f_start * 10); i < n; ++ i)
					renderer((i + 1) * (f_start / n));
			}
			// render the start frames but throw them away. this is required by the animation
			// system, which cannot easily skip time

			glActiveTexture(GL_TEXTURE0);
			// !!

			float p_ss_weights[n_temporal_supersample_num];
			{
				float f_ss_weight_sum = 0;
				for(int j = 0; j < n_temporal_supersample_num; ++ j) {
					p_ss_weights[j] = int(n_temporal_supersample_num - 2 * fabs(j - (n_temporal_supersample_num - 1) / 2.0)); // flat
					_ASSERTE(p_ss_weights[j] >= 0 && p_ss_weights[j] < 255); // fit into uint8_t (MMX / SSE)
					f_ss_weight_sum += p_ss_weights[j];
				}
				//_ASSERTE(p_ss_weights[0] < 1e-3f); // the first one is zero
				const float f_inv_sum = 1.0 / f_ss_weight_sum;
				for(int j = 0; j < n_temporal_supersample_num; ++ j)
					p_ss_weights[j] *= f_inv_sum;
			}
			// prepare for temporal supersampling

			std::string s_filename;
			for(size_t i = 0; i < n_frame_num; ++ i) {
				TBmp *p_bitmap = encoder.p_Get_EmptyBitmap(); // there is one extra bitmap
				for(int j = 0; j < n_temporal_supersample_num; ++ j) {
					fbo.Bind();
					fbo.Bind_ColorTexture(0, color_tex, 0);
					glViewport(0, 0, n_fbo_width2, n_fbo_height2);
					{
						renderer(f_start + (i + float(j) / (n_temporal_supersample_num + 1)) * f_frame_time);
					}
					fbo.Bind_ColorTexture(0, 0, 0);
					fbo.Release();
					// draw one frame

					glActiveTexture(GL_TEXTURE0);
					// !!

					accum_fbo.Bind(GL_FRAMEBUFFER); // !! otherwise OpenGL 4+ won't read from the FBO but from the backbuffer instead
					glViewport(0, 0, n_fbo_width, n_fbo_height); // rescaling done on GPU
					{
						color_tex.Bind();
						if(b_use_mipmaps)
							glGenerateMipmap(color_tex.n_Target());
						if(!j) {
							glClearColor(0, 0, 0, 0);
							glClear(GL_COLOR_BUFFER_BIT);
						}
						// clear the accumulator

						glEnable(GL_BLEND);
						glBlendFunc(GL_ONE, GL_ONE);
						accum_shader.Bind(p_ss_weights[j]); // weighted accumulation (the weights sum to one)
						fs_quad.Draw();
						glDisable(GL_BLEND);
						// accumulate the last frame

						if(j + 1 == n_temporal_supersample_num) {
							glFlush();
							glFinish();
							glReadPixels(0, 0, n_fbo_width, n_fbo_height, GL_RGBA, GL_UNSIGNED_BYTE, p_bitmap->p_buffer);
						}
						// read back
					}
					accum_fbo.Release();
					// weighted accumulate in hardware
				}

				if(!encoder.Enqueue_Bitmap(p_s_filename, p_bitmap))
					throw std::runtime_error("queue Put() failed");
				// just fill a work-item
			}
		}
		// render the frames offscreen

		glViewport(0, 0, n_width, n_height);
		// restore the previous viewport

		if(!encoder.Synchronize())
			fprintf(stderr, "error: encoder join failed\n");
		// synchronize with the workers
	}

	/**
	 *	@brief converts a user-specified pattern to a printf pattern
	 *	@param[in,out] r_s_pattern is pattern string; on input it contains a sequence of '#' one
	 *		for each digit, on output it contains PRIzize with the appropriate number of digits
	 *		specified (only the first sequence is replaced, any other ones are left as is)
	 *	@note Any percent signs in the pattern are escaped.
	 */
	static bool Make_Pattern(std::string &r_s_pattern)
	{
		for(size_t n_pos = 0; (n_pos = r_s_pattern.find('%', n_pos)) != std::string::npos;) {
			if(n_pos == r_s_pattern.length())
				r_s_pattern.insert(n_pos ++, 1, '%');
			else if(r_s_pattern[n_pos + 1] != '%')
				r_s_pattern.insert(++ n_pos, 1, '%');
			++ n_pos;
		}
		// escape all the percent signs

		size_t n_pos = r_s_pattern.find('#');
		if(n_pos == std::string::npos)
			return false;
		// find the first occurence of hash tags

		size_t b = n_pos, e = r_s_pattern.length();
		while(++ b < e && r_s_pattern[b] == '#');
		e = b;
		b = n_pos;
		// find the last occurence of hash tag

		size_t n_digit_num = e - b;
		r_s_pattern.erase(b, e - b);
		char p_s_pattern[32];
		if(!stl_ut::Format(p_s_pattern, sizeof(p_s_pattern), "%%0" PRIsize _PRIsize, n_digit_num))
			return false;
		r_s_pattern.insert(b, p_s_pattern);

		return true;
	}

	template <class CRenderFrame>
	void Render_Sequence(const char *p_s_filename_pattern, CRenderFrame renderer,
		double f_start, double f_elapse) // throw(std::bad_alloc, std::runtime_error)
	{
		_ASSERTE(m_f_fps > 0);
		_ASSERTE(f_start >= 0);
		_ASSERTE(f_elapse >= 0);

		int p_wp[4];
		glGetIntegerv(GL_VIEWPORT, p_wp);
		const int n_width = p_wp[2], n_height = p_wp[3];
		// remember the original viewport

		std::string s_pattern = p_s_filename_pattern; // throws
		{
			if(!Make_Pattern(s_pattern))
				throw std::runtime_error("invalid pattern");
			/*for(size_t n_pos = 0; (n_pos = s_pattern.find('%', n_pos)) != std::string::npos;) {
				if(n_pos == s_pattern.length())
					s_pattern.insert(n_pos ++, 1, '%');
				else if(s_pattern[n_pos + 1] != '%')
					s_pattern.insert(++ n_pos, 1, '%');
				++ n_pos;
			}
			// escape all the percent signs

			size_t n_pos = s_pattern.find('#');
			if(n_pos == std::string::npos)
				throw std::runtime_error("invalid pattern");
			// find the first occurence of hash tags

			size_t b = n_pos, e = s_pattern.length();
			while(++ b < e && s_pattern[b] == '#');
			e = b;
			b = n_pos;
			// find the last occurence of hash tag

			size_t n_digit_num = e - b;
			s_pattern.erase(b, e - b);
			char p_s_pattern[256];
			if(!stl_ut::Format(p_s_pattern, sizeof(p_s_pattern), "%%0" PRIsize _PRIsize, n_digit_num))
				throw std::runtime_error("invalid pattern");
			s_pattern.insert(b, p_s_pattern);*/

			p_s_filename_pattern = s_pattern.c_str(); // !!
		}
		// change the "filename_#####.png" pattern to "filename_%05u.png" pattern

		const int n_max_dim = max(m_n_width, m_n_height);
		const int n_max_fbo_dim = CGLFrameBufferObject::n_Get_MaxSize();
		const int n_supersample_limit = n_max_fbo_dim / n_max_dim;
		const int n_AA_scale = min((int)n_spatial_supersample_num, n_supersample_limit);
		// decide on spatial antialiassing

		const int n_fbo_width2 = m_n_width * n_AA_scale;
		const int n_fbo_height2 = m_n_height * n_AA_scale; // render at high resolution
		const int n_fbo_width = m_n_width;
		const int n_fbo_height = m_n_height; // store at lower to facilitate spatial antialiassing
		const bool b_use_mipmaps = float(n_fbo_width2) / n_fbo_width > 2;
		// t_odo - make this setting part of the object

		static const float p_fs_quad[] = {-1, 1, -1, -1, 1, 1, 1, -1};
		CGLArraySetup fs_quad(p_fs_quad, sizeof(p_fs_quad), 0, GL_FLOAT, 2, 0, 0, 0, 0, GL_TRIANGLE_STRIP);
		// need a fullscreen quad for accumulation

		glActiveTexture(GL_TEXTURE0);
		// !!

		CGLFrameBufferObject fbo(n_fbo_width2, n_fbo_height2, 1, true, 0, 0, // render to texture
			true, false, GL_DEPTH_COMPONENT24_OES, 0,
			false, false, 0, 0);
		CGLTexture_2D color_tex(n_fbo_width2, n_fbo_height2, GL_RGBA8_OES, b_use_mipmaps);
		CGLFrameBufferObject accum_fbo(n_fbo_width, n_fbo_height, 1,
			false, /*GL_RGBA32F*/0x8814, 0, false, false, 0, 0); // GL_RGBA32F not in OES yet
		TAccumShader accum_shader;
		if(!fbo.b_Status() || !color_tex.b_Status() ||
		   !accum_fbo.b_Status() || !accum_shader.Compile() || !fs_quad.b_Status()) {
			fprintf(stderr, "error: failed to init FBO\n");
			return;
		}
		// make a framebuffer, texture and other floating point framebuffer for accumulation

		const size_t n_frame_num = size_t(ceil(f_elapse * m_f_fps));
		const double f_frame_time = 1 / m_f_fps;

		size_t n_worker_num = max(size_t(2), CThread::n_CPU_Num()) - 1;
		// get number of threads

		CMTEncoder encoder(n_fbo_width, n_fbo_height, true, true, n_worker_num);
		// starts the threads, throws exceptions if low on memory or if the threads failed to start

		/*_TyQueue queue(n_worker_num), return_queue(n_worker_num);
		if(!queue.b_Status() || !return_queue.b_Status())
			throw std::runtime_error("queue initialization failed");

		std::vector<CWorker> worker_list(n_worker_num);
		std::vector<TBmp*> bitmap_list;
		for(size_t i = 0; i < n_worker_num + 1; ++ i) {
			TBmp *p_bitmap;
			if(!(p_bitmap = TBmp::p_Alloc(n_fbo_width, n_fbo_height)))
				throw std::bad_alloc(); // rethrow
			bitmap_list.push_back(p_bitmap);
			// make a list of bitmaps

			if(i < n_worker_num) {
				TWorkItem t_empty;
				t_empty.p_bitmap = p_bitmap;
				if(!return_queue.Put(t_empty))
					throw std::runtime_error("queue Put() failed");
				// put the bitmaps to the return queue
				
				worker_list[i].Configure(queue, return_queue);
				worker_list[i].Start();
				// start the worker
			}
		}*/
		//std::vector<Vector3i> supersampled_bitmap;
		// alloc memory and init the workers

		CTimer elapsed;
		// elapsed time timer

		double f_GPU_time = 0;
		// pure GPU time timer

		{
			if(f_start > 0) {
				for(size_t i = 0, n = ceil(f_start * 10); i < n; ++ i)
					renderer((i + 1) * (f_start / n));
			}
			// render the start frames but throw them away. this is required by the animation
			// system, which cannot easily skip time

			glActiveTexture(GL_TEXTURE0);
			// !!

			float p_ss_weights[n_temporal_supersample_num];
			{
				float f_ss_weight_sum = 0;
				for(int j = 0; j < n_temporal_supersample_num; ++ j) {
					p_ss_weights[j] = int(n_temporal_supersample_num - 2 * fabs(j - (n_temporal_supersample_num - 1) / 2.0)); // flat
					_ASSERTE(p_ss_weights[j] >= 0 && p_ss_weights[j] < 255); // fit into uint8_t (MMX / SSE)
					f_ss_weight_sum += p_ss_weights[j];
				}
				//_ASSERTE(p_ss_weights[0] < 1e-3f); // the first one is zero
				const float f_inv_sum = 1.0 / f_ss_weight_sum;
				for(int j = 0; j < n_temporal_supersample_num; ++ j)
					p_ss_weights[j] *= f_inv_sum;
			}
			// prepare for temporal supersampling

			std::string s_filename;
			for(size_t i = 0; i < n_frame_num; ++ i) {
				/*supersampled_bitmap.clear();
				supersampled_bitmap.resize(n_fbo_width * n_fbo_height, Vector3i(0, 0, 0));*/
				// clear the accumulator

				const double f_frame_start_time = elapsed.f_Time();
				// see when we started

				TBmp *p_bitmap = encoder.p_Get_EmptyBitmap(); // there is one extra bitmap
				for(int j = 0; j < n_temporal_supersample_num; ++ j) {
					fbo.Bind();
					fbo.Bind_ColorTexture(0, color_tex, 0);
					glViewport(0, 0, n_fbo_width2, n_fbo_height2);
					{
						renderer(f_start + (i + float(j) / (n_temporal_supersample_num + 1)) * f_frame_time);
					}
					fbo.Bind_ColorTexture(0, 0, 0);
					fbo.Release();
					// draw one frame

					glActiveTexture(GL_TEXTURE0);
					// !!

					accum_fbo.Bind(GL_FRAMEBUFFER); // !! otherwise OpenGL 4+ won't read from the FBO but from the backbuffer instead
					glViewport(0, 0, n_fbo_width, n_fbo_height); // rescaling done on GPU
					{
						color_tex.Bind();
						if(b_use_mipmaps)
							glGenerateMipmap(color_tex.n_Target());
						if(!j) {
							glClearColor(0, 0, 0, 0);
							glClear(GL_COLOR_BUFFER_BIT);
						}
						// clear the accumulator

						glEnable(GL_BLEND);
						glBlendFunc(GL_ONE, GL_ONE);
						accum_shader.Bind(p_ss_weights[j]); // weighted accumulation (the weights sum to one)
						fs_quad.Draw();
						glDisable(GL_BLEND);
						// accumulate the last frame

						if(j + 1 == n_temporal_supersample_num) {
							glFlush();
							glFinish();
							glReadPixels(0, 0, n_fbo_width, n_fbo_height, GL_RGBA, GL_UNSIGNED_BYTE, p_bitmap->p_buffer);
						}
						// read back
					}
					accum_fbo.Release();
					// weighted accumulate in hardware

					/*glReadPixels(0, 0, n_fbo_width, n_fbo_height, GL_RGBA, GL_UNSIGNED_BYTE, p_bitmap->p_buffer);
					const uint8_t w = p_ss_weights[j]; // antialiass
					for(size_t k = 0, n = n_fbo_width * n_fbo_height; k < n; ++ k) {
						uint32_t n_pixel = p_bitmap->p_buffer[k]; // antialiass
						supersampled_bitmap[k].x += ((n_pixel >> 16) & 0xff) * w;
						supersampled_bitmap[k].y += ((n_pixel >> 8) & 0xff) * w;
						supersampled_bitmap[k].z += (n_pixel & 0xff) * w;
					}*/
					// this takes a lot of time, should do that on GPU with a float buffer
				}
				/*for(size_t k = 0, n = n_fbo_width * n_fbo_height; k < n; ++ k) {
					int r = max(0, min(255, int((supersampled_bitmap[k].x + f_ss_weight_sum / 2) * f_inv_sum)));
					int g = max(0, min(255, int((supersampled_bitmap[k].y + f_ss_weight_sum / 2) * f_inv_sum)));
					int b = max(0, min(255, int((supersampled_bitmap[k].z + f_ss_weight_sum / 2) * f_inv_sum)));
					p_bitmap->p_buffer[k] = (r << 16) | (g << 8) | b | 0xff000000U;
				}*/
				// accumulate and average

				f_GPU_time += elapsed.f_Time() - f_frame_start_time;
				// accumulate GPU-only time

				if(!stl_ut::Format(s_filename, p_s_filename_pattern, i)) {
					//p_bitmap->Delete();
					throw std::bad_alloc(); // rethrow
				}
#if 0
				//glReadPixels(0, 0, n_fbo_width, n_fbo_height, GL_RGBA, GL_UNSIGNED_BYTE, p_bitmap->p_buffer);
				//if(n_fbo_height <= 720) {
					p_bitmap->Flip(true);
					CPngCodec::Save_PNG(s_filename.c_str(), *p_bitmap, true); // ARGB
				/*} else { // sampler is too buggy to use right now
					TBmp *p_small = p_Resample_BMP(p_bitmap, 1280, 720, false); // lofi downsample first
					p_small->Flip(true); // then flip, with less data
					CPngCodec::Save_PNG(s_filename.c_str(), *p_small, true); // ARGB
					p_small->Delete();
				}*/
#else // 0
				/*TWorkItem t_empty;
				if(!return_queue.Get(t_empty)) // get an empty bitmap
					throw std::runtime_error("queue Get() failed");
				//glReadPixels(0, 0, n_fbo_width, n_fbo_height, GL_RGBA, GL_UNSIGNED_BYTE, t_empty.p_bitmap->p_buffer);
				memcpy(t_empty.p_bitmap->p_buffer, p_bitmap->p_buffer, n_fbo_width * n_fbo_height * sizeof(uint32_t));
				t_empty.s_filename.swap(s_filename);
				if(!queue.Put(t_empty)) // put it to be processed*/
				if(!encoder.Enqueue_Bitmap(s_filename.c_str(), p_bitmap)) {
					encoder.Synchronize(); // this might throw and when an exception is thrown while handling another exception, runtime kills this instead of being caught (would need multiple nested handlers maybe?)
					throw std::runtime_error("queue Put() failed");
				}
				// just fill a work-item
#endif // 0
				// t_odo - reuse producer/consumer queue for parallel .png encoding (the slowest step)

				double f_elapsed = elapsed.f_Time();
				double f_estimated = (f_elapsed * n_frame_num) / (i + 1);
				double f_remaining = max(0, f_estimated - f_elapsed);
				printf("rendered " PRIsize " / " PRIsize " frames (%.2f FPS, %.2f GPU FPS, " PRItime ")   \r",
					i, n_frame_num, (i + 1) / f_elapsed, (i + 1) / f_GPU_time, PRItimeparams(f_remaining));
				// print some stats
			}
		}
		// render the frames offscreen

		glViewport(0, 0, n_width, n_height);
		// restore the previous viewport

		/*queue.Signal_Finished();
		for(size_t i = 0; i < n_worker_num; ++ i)
			worker_list[i].WaitForFinish();*/
		if(!encoder.Synchronize())
			fprintf(stderr, "error: encoder join failed\n");
		printf("rendered " PRIsize " / " PRIsize " frames%45s\n", n_frame_num, n_frame_num, "");
		// synchronize with the workers

		/*for(size_t i = 0; i < n_worker_num; ++ i)
			bitmap_list[i]->Delete();*/
		// delete the bitmaps
	}

	template <class CRenderFrame>
	void Render_Sequence_Omni(const char *p_s_filename_pattern, CRenderFrame renderer,
		double f_start, double f_elapse) // throw(std::bad_alloc, std::runtime_error)
	{
		_ASSERTE(m_f_fps > 0);
		_ASSERTE(f_start >= 0);
		_ASSERTE(f_elapse >= 0);

#ifndef GL_TEXTURE_CUBE_MAP_SEAMLESS
#define GL_TEXTURE_CUBE_MAP_SEAMLESS 0x884F 
#endif // !GL_TEXTURE_CUBE_MAP_SEAMLESS
		if(CGLES20ExtensionHandler::b_SupportedExtension("GL_ARB_seamless_cube_map"))
			glEnable(GL_TEXTURE_CUBE_MAP_SEAMLESS);
		// use seamless cube-map filtering where available

		int p_wp[4];
		glGetIntegerv(GL_VIEWPORT, p_wp);
		const int n_width = p_wp[2], n_height = p_wp[3];
		// remember the original viewport

		std::string s_pattern = p_s_filename_pattern; // throws
		{
			if(!Make_Pattern(s_pattern))
				throw std::runtime_error("invalid pattern");
			/*for(size_t n_pos = 0; (n_pos = s_pattern.find('%', n_pos)) != std::string::npos;) {
				if(n_pos == s_pattern.length())
					s_pattern.insert(n_pos ++, 1, '%');
				else if(s_pattern[n_pos + 1] != '%')
					s_pattern.insert(++ n_pos, 1, '%');
				++ n_pos;
			}
			// escape all the percent signs

			size_t n_pos = s_pattern.find('#');
			if(n_pos == std::string::npos)
				throw std::runtime_error("invalid pattern");
			// find the first occurence of hash tags

			size_t b = n_pos, e = s_pattern.length();
			while(++ b < e && s_pattern[b] == '#');
			e = b;
			b = n_pos;
			// find the last occurence of hash tag

			size_t n_digit_num = e - b;
			s_pattern.erase(b, e - b);
			char p_s_pattern[256];
			if(!stl_ut::Format(p_s_pattern, sizeof(p_s_pattern), "%%0" PRIsize _PRIsize, n_digit_num))
				throw std::runtime_error("invalid pattern");
			s_pattern.insert(b, p_s_pattern);*/

			p_s_filename_pattern = s_pattern.c_str(); // !!
		}
		// change the "filename_#####.png" pattern to "filename_%05u.png" pattern

		double f_area = double(m_n_width) * m_n_height;
		double f_cube_face_area = f_area / 6;
		double f_cube_face_size = sqrt(f_cube_face_area) * 1.225;
		// choose size of a single cube face

		int n_max_cube_size;
		glGetIntegerv(GL_MAX_CUBE_MAP_TEXTURE_SIZE, &n_max_cube_size);
		const int n_cube_size = min(n_max_cube_size, n_Align_Up_POT(max(int(f_cube_face_size), 16), 16));
		// decide on cube map resolution

		printf("debug: chosen cube size %d (area " PRIvalueMP "pix, needed " PRIvalueMP "pix, ratio %g)\n",
			n_cube_size, PRIvalueMPparams(double(n_cube_size) * n_cube_size * 6),
			PRIvalueMPparams(f_area), double(n_cube_size) * n_cube_size * 6 / f_area);

		const int n_max_fbo_dim = CGLFrameBufferObject::n_Get_MaxSize();
		const int n_supersample_limit = n_max_fbo_dim / n_cube_size;
		const int n_AA_scale = min((int)n_omni_spatial_supersample_num, n_supersample_limit);
		// decide on spatial antialiassing

		const int n_fbo_width2 = n_cube_size * n_AA_scale; // make this square for cube-maps
		const int n_fbo_height2 = n_cube_size * n_AA_scale; // render at high resolution
		const int n_fbo_width = m_n_width;
		const int n_fbo_height = m_n_height; // store at lower to facilitate spatial antialiassing
		const bool b_use_mipmaps = true;//float(n_fbo_width2) / n_fbo_width > 2;
		// t_odo - make this setting part of the object

		printf("debug: AA cube size %d (memory " PRIsizeB "B)\n", n_fbo_width2,
			PRIsizeBparams(double(n_cube_size) * n_cube_size * 6 * sizeof(uint32_t) * ((b_use_mipmaps)? 1 + 1.0 / 3 : 1)));

		static const float p_fs_quad[] = {-1, 1, -1, -1, 1, 1, 1, -1};
		CGLArraySetup fs_quad(p_fs_quad, sizeof(p_fs_quad), 0, GL_FLOAT, 2, 0, 0, 0, 0, GL_TRIANGLE_STRIP);
		// need a fullscreen quad for accumulation

		glActiveTexture(GL_TEXTURE0);
		// !!

		CGLFrameBufferObject fbo(n_fbo_width2, n_fbo_height2, 1, true, 0, 0, // render to texture
			true, false, GL_DEPTH_COMPONENT24_OES, 0,
			false, false, 0, 0);
		CGLTexture_Cube color_tex(n_fbo_width2, /*n_fbo_height2,*/ GL_RGBA8_OES, b_use_mipmaps);
		CGLFrameBufferObject accum_fbo(n_fbo_width, n_fbo_height, 1,
			false, /*GL_RGBA32F*/0x8814, 0, false, false, 0, 0); // GL_RGBA32F not in OES yet
		TAccumShader_Omni accum_shader_omni;
		if(!fbo.b_Status() || !color_tex.b_Status() ||
		   !accum_fbo.b_Status() || !accum_shader_omni.Compile() || !fs_quad.b_Status()) {
			fprintf(stderr, "error: failed to init FBO\n");
			return;
		}
		// make a framebuffer, texture and other floating point framebuffer for accumulation

		color_tex.Bind();
		if(CGLES20ExtensionHandler::b_SupportedExtension("GL_EXT_texture_filter_anisotropic")) {
			float f_anisotropy;
			glGetFloatv(GL_MAX_TEXTURE_MAX_ANISOTROPY_EXT, &f_anisotropy);
			glTexParameterf(color_tex.n_Target(), GL_TEXTURE_MAX_ANISOTROPY_EXT, f_anisotropy);
		}

		const size_t n_frame_num = size_t(ceil(f_elapse * m_f_fps));
		const double f_frame_time = 1 / m_f_fps;

		size_t n_worker_num = max(size_t(2), CThread::n_CPU_Num()) - 1;
		// get number of threads

		CMTEncoder encoder(n_fbo_width, n_fbo_height, true, true, n_worker_num);
		// starts the threads, throws exceptions if low on memory or if the threads failed to start

		/*_TyQueue queue(n_worker_num), return_queue(n_worker_num);
		if(!queue.b_Status() || !return_queue.b_Status())
			throw std::runtime_error("queue initialization failed");

		std::vector<CWorker> worker_list(n_worker_num);
		std::vector<TBmp*> bitmap_list;
		for(size_t i = 0; i < n_worker_num + 1; ++ i) {
			TBmp *p_bitmap;
			if(!(p_bitmap = TBmp::p_Alloc(n_fbo_width, n_fbo_height)))
				throw std::bad_alloc(); // rethrow
			bitmap_list.push_back(p_bitmap);
			// make a list of bitmaps

			if(i < n_worker_num) {
				TWorkItem t_empty;
				t_empty.p_bitmap = p_bitmap;
				if(!return_queue.Put(t_empty))
					throw std::runtime_error("queue Put() failed");
				// put the bitmaps to the return queue
				
				worker_list[i].Configure(queue, return_queue);
				worker_list[i].Start();
				// start the worker
			}
		}*/
		//std::vector<Vector3i> supersampled_bitmap;
		// alloc memory and init the workers

		CTimer elapsed;
		// elapsed time timer

		double f_GPU_time = 0;
		// pure GPU time timer

		Matrix4f t_front, t_left, t_back, t_right, t_bottom, t_top;
		t_front.Identity();
		t_bottom.RotationX(-f_pi / 2); // todo - add other rotations?
		t_top.RotationX(f_pi / 2); // todo - add other rotations?
		t_left.RotationY(-f_pi / 2); //t_left.Scale(-1, 1, 1); // todo - add other rotations?
		t_back.RotationY(f_pi); // todo - add other rotations?
		t_right.RotationY(f_pi / 2); //t_right.Scale(-1, 1, 1); // todo - add other rotations?

		const Matrix4f p_cube_views[] = {
			t_front, t_left, t_back, t_right, t_bottom, t_top
		};
		const GLenum p_cube_faces[] = {
			GL_TEXTURE_CUBE_MAP_POSITIVE_Z, // front
			GL_TEXTURE_CUBE_MAP_NEGATIVE_X, // left
			GL_TEXTURE_CUBE_MAP_NEGATIVE_Z, // back
			GL_TEXTURE_CUBE_MAP_POSITIVE_X, // right
			GL_TEXTURE_CUBE_MAP_NEGATIVE_Y, // bottom
			GL_TEXTURE_CUBE_MAP_POSITIVE_Y // top
		};
		// cube map faces and the corresponding view transforms

		{
			if(f_start > 0) {
				for(size_t i = 0, n = ceil(f_start * 10); i < n; ++ i) {
					for(int j = 0; j < 6; ++ j)
						renderer((i + 1) * (f_start / n), p_cube_views[j]);
				}
			}
			// render the start frames but throw them away. this is required by the animation
			// system, which cannot easily skip time

			glActiveTexture(GL_TEXTURE0);
			// !!

			float p_ss_weights[n_temporal_supersample_num];
			{
				float f_ss_weight_sum = 0;
				for(int j = 0; j < n_temporal_supersample_num; ++ j) {
					p_ss_weights[j] = int(n_temporal_supersample_num - 2 * fabs(j - (n_temporal_supersample_num - 1) / 2.0)); // flat
					_ASSERTE(p_ss_weights[j] >= 0 && p_ss_weights[j] < 255); // fit into uint8_t (MMX / SSE)
					f_ss_weight_sum += p_ss_weights[j];
				}
				//_ASSERTE(p_ss_weights[0] < 1e-3f); // the first one is zero
				const float f_inv_sum = 1.0 / f_ss_weight_sum;
				for(int j = 0; j < n_temporal_supersample_num; ++ j)
					p_ss_weights[j] *= f_inv_sum;
			}
			// prepare for temporal supersampling

			std::string s_filename;
			for(size_t i = 0; i < n_frame_num; ++ i) {
				/*supersampled_bitmap.clear();
				supersampled_bitmap.resize(n_fbo_width * n_fbo_height, Vector3i(0, 0, 0));*/
				// clear the accumulator

				const double f_frame_start_time = elapsed.f_Time();
				// see when we started

				TBmp *p_bitmap = encoder.p_Get_EmptyBitmap(); // there is one extra bitmap
				for(int j = 0; j < n_temporal_supersample_num; ++ j) {
					for(int k = 0; k < 6; ++ k) {
						fbo.Bind();
						fbo.Bind_ColorTextureFace(0, color_tex, 0, p_cube_faces[k]);
						glViewport(0, 0, n_fbo_width2, n_fbo_height2);
						{
							renderer(f_start + (i + float(j) / (n_temporal_supersample_num + 1)) * f_frame_time,
								p_cube_views[k]);
						}
						fbo.Bind_ColorTexture(0, 0, 0);
						fbo.Release();
					}
					// draw one frame, several times to each cube face

					glActiveTexture(GL_TEXTURE0);
					// !!

					accum_fbo.Bind(GL_FRAMEBUFFER); // !! otherwise OpenGL 4+ won't read from the FBO but from the backbuffer instead
					glViewport(0, 0, n_fbo_width, n_fbo_height); // rescaling done on GPU
					{
						color_tex.Bind();
						if(b_use_mipmaps)
							glGenerateMipmap(color_tex.n_Target());
						if(!j) {
							glClearColor(0, 0, 0, 0);
							glClear(GL_COLOR_BUFFER_BIT);
						}
						// clear the accumulator

						static const Vector2f p_jitter[] = {
							Vector2f( 0.285561,  0.188437), Vector2f( 0.360176, -0.065688),
							Vector2f(-0.111751,  0.275019), Vector2f(-0.055918, -0.215197),
							Vector2f(-0.080231, -0.470965), Vector2f( 0.138721,  0.409168),
							Vector2f( 0.384120,  0.458500), Vector2f(-0.454968,  0.134088),
							Vector2f( 0.179271, -0.331196), Vector2f(-0.307049, -0.364927),
							Vector2f( 0.105354, -0.010099), Vector2f(-0.154180,  0.021794),
							Vector2f(-0.370135, -0.116425), Vector2f( 0.451636, -0.300013),
							Vector2f(-0.370610,  0.387504) // from the red book (the same as rand() but likely much cheaper)
						};
						static int n_jitter = 0; // a bit uncool, i guess
						n_jitter = (n_jitter + 1) % (sizeof(p_jitter) / sizeof(p_jitter[0]));

						glEnable(GL_BLEND);
						glBlendFunc(GL_ONE, GL_ONE);
						accum_shader_omni.Bind(p_ss_weights[j], // weighted accumulation (the weights sum to one)
							p_jitter[n_jitter] * 2 / Vector2f(n_fbo_width, n_fbo_height)); // apply jitter
						fs_quad.Draw();
						glDisable(GL_BLEND);
						// accumulate the last frame

						if(j + 1 == n_temporal_supersample_num) {
							glFlush();
							glFinish();
							glReadPixels(0, 0, n_fbo_width, n_fbo_height, GL_RGBA, GL_UNSIGNED_BYTE, p_bitmap->p_buffer);
						}
						// read back
					}
					accum_fbo.Release();
					// weighted accumulate in hardware

					/*glReadPixels(0, 0, n_fbo_width, n_fbo_height, GL_RGBA, GL_UNSIGNED_BYTE, p_bitmap->p_buffer);
					const uint8_t w = p_ss_weights[j]; // antialiass
					for(size_t k = 0, n = n_fbo_width * n_fbo_height; k < n; ++ k) {
						uint32_t n_pixel = p_bitmap->p_buffer[k]; // antialiass
						supersampled_bitmap[k].x += ((n_pixel >> 16) & 0xff) * w;
						supersampled_bitmap[k].y += ((n_pixel >> 8) & 0xff) * w;
						supersampled_bitmap[k].z += (n_pixel & 0xff) * w;
					}*/
					// this takes a lot of time, should do that on GPU with a float buffer
				}
				/*for(size_t k = 0, n = n_fbo_width * n_fbo_height; k < n; ++ k) {
					int r = max(0, min(255, int((supersampled_bitmap[k].x + f_ss_weight_sum / 2) * f_inv_sum)));
					int g = max(0, min(255, int((supersampled_bitmap[k].y + f_ss_weight_sum / 2) * f_inv_sum)));
					int b = max(0, min(255, int((supersampled_bitmap[k].z + f_ss_weight_sum / 2) * f_inv_sum)));
					p_bitmap->p_buffer[k] = (r << 16) | (g << 8) | b | 0xff000000U;
				}*/
				// accumulate and average

				f_GPU_time += elapsed.f_Time() - f_frame_start_time;
				// accumulate GPU-only time

				if(!stl_ut::Format(s_filename, p_s_filename_pattern, i)) {
					//p_bitmap->Delete();
					throw std::bad_alloc(); // rethrow
				}
#if 0
				//glReadPixels(0, 0, n_fbo_width, n_fbo_height, GL_RGBA, GL_UNSIGNED_BYTE, p_bitmap->p_buffer);
				//if(n_fbo_height <= 720) {
					p_bitmap->Flip(true);
					CPngCodec::Save_PNG(s_filename.c_str(), *p_bitmap, true); // ARGB
				/*} else { // sampler is too buggy to use right now
					TBmp *p_small = p_Resample_BMP(p_bitmap, 1280, 720, false); // lofi downsample first
					p_small->Flip(true); // then flip, with less data
					CPngCodec::Save_PNG(s_filename.c_str(), *p_small, true); // ARGB
					p_small->Delete();
				}*/
#else // 0
				/*TWorkItem t_empty;
				if(!return_queue.Get(t_empty)) // get an empty bitmap
					throw std::runtime_error("queue Get() failed");
				//glReadPixels(0, 0, n_fbo_width, n_fbo_height, GL_RGBA, GL_UNSIGNED_BYTE, t_empty.p_bitmap->p_buffer);
				memcpy(t_empty.p_bitmap->p_buffer, p_bitmap->p_buffer, n_fbo_width * n_fbo_height * sizeof(uint32_t));
				t_empty.s_filename.swap(s_filename);
				if(!queue.Put(t_empty)) // put it to be processed*/
				if(!encoder.Enqueue_Bitmap(s_filename.c_str(), p_bitmap)) {
					encoder.Synchronize(); // this might throw and when an exception is thrown while handling another exception, runtime kills this instead of being caught (would need multiple nested handlers maybe?)
					throw std::runtime_error("queue Put() failed");
				}
				// just fill a work-item
#endif // 0
				// t_odo - reuse producer/consumer queue for parallel .png encoding (the slowest step)

				double f_elapsed = elapsed.f_Time();
				double f_estimated = (f_elapsed * n_frame_num) / (i + 1);
				double f_remaining = max(0, f_estimated - f_elapsed);
				printf("rendered " PRIsize " / " PRIsize " frames (%.2f FPS, %.2f GPU FPS, " PRItime ")   \r",
					i, n_frame_num, (i + 1) / f_elapsed, (i + 1) / f_GPU_time, PRItimeparams(f_remaining));
				// print some stats
			}
		}
		// render the frames offscreen

		glViewport(0, 0, n_width, n_height);
		// restore the previous viewport

		/*queue.Signal_Finished();
		for(size_t i = 0; i < n_worker_num; ++ i)
			worker_list[i].WaitForFinish();*/
		if(!encoder.Synchronize())
			fprintf(stderr, "error: encoder join failed\n");
		printf("rendered " PRIsize " / " PRIsize " frames%45s\n", n_frame_num, n_frame_num, "");
		// synchronize with the workers

		/*for(size_t i = 0; i < n_worker_num; ++ i)
			bitmap_list[i]->Delete();*/
		// delete the bitmaps
	}
};

#endif // !__CAMERA_SPLINE_ANIMATION_UTILS_INCLUDED
