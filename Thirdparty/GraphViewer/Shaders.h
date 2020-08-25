/*
								+--------------------------------+
								|                                |
								|    ***  OpenGL shaders  ***    |
								|                                |
								|  Copyright © -tHE SWINe- 2014  |
								|                                |
								|           Shaders.h            |
								|                                |
								+--------------------------------+
*/

#pragma once
#ifndef __GRAPH_VIEW_SHADERS_INCLUDED
#define __GRAPH_VIEW_SHADERS_INCLUDED

/**
 *	@file Shaders.h
 *	@brief OpenGL shaders
 *	@author -tHE SWINe-
 *	@date 2014
 */

#define GV_SHADERS_AVOID_INTEGER_ARITHMETICS
// integer arithmetics caused some issues on older GPUs, try to do without

struct TConstantColorShader : public CGLESShader {
	GLint n_mvp_uniform, n_color_uniform, n_scale_uniform;

	bool Compile()
	{
		const char *p_s_vertex_shader =
			"precision highp float;\n"
			"\n"
			"attribute vec3 v_pos;\n"
			"\n"
			"uniform mat4 t_mvp;\n"
			"uniform float f_scale;\n"
			"\n"
			"void main()\n"
			"{\n"
			"    gl_Position = t_mvp * vec4(v_pos, 1.0);\n"
			"    gl_PointSize = 1.0 * f_scale;\n" // not sure it is ok to write this if the primitive rasterized is different from GL_POINTS // todo - find out
			"}\n";
		const char *p_s_fragment_shader =
			"precision highp float;\n"
			"\n"
			"uniform vec4 v_color;\n"
			"\n"
			"void main()\n"
			"{\n"
			"    gl_FragColor = /*vec4(normalize(vec3(gl_PointCoord.x, 1.0, gl_PointCoord.y)), 1.0) + .0 **/ v_color;\n"
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

		n_mvp_uniform = n_Get_Uniform_Location("t_mvp");
		n_color_uniform = n_Get_Uniform_Location("v_color");
		n_scale_uniform = n_Get_Uniform_Location("f_scale");
		// get addresses of the uniforms

		CGLESShader::Bind();
		Uniform1f(n_scale_uniform, 1);

		return true;
	}

	void Bind(const Matrix4f &r_t_mvp, float f_r, float f_g, float f_b, float f_a = 1) const
	{
		CGLESShader::Bind();
		// bind the shader

		UniformMatrix4fv(n_mvp_uniform, 1, false, &r_t_mvp[0][0]);
		Uniform4f(n_color_uniform, f_r, f_g, f_b, f_a);
		// set the uniforms
	}

	void SetScale(float f_scale)
	{
		CGLESShader::Bind();
		// bind the shader

		Uniform1f(n_scale_uniform, f_scale);
	}
};

struct TColorShader : public CGLESShader {
	GLint n_mvp_uniform, n_scale_uniform;

	bool Compile()
	{
		const char *p_s_vertex_shader =
			"precision highp float;\n"
			"\n"
			"attribute vec3 v_pos;\n"
			"attribute vec4 v_col;\n"
			"\n"
			"uniform mat4 t_mvp;\n"
			"uniform float f_scale;\n"
			"\n"
			"varying vec4 v_color;\n"
			"\n"
			"void main()\n"
			"{\n"
			"    gl_Position = t_mvp * vec4(v_pos, 1.0);\n"
			"    gl_PointSize = 1.0 * f_scale;\n" // not sure it is ok to write this if the primitive rasterized is different from GL_POINTS // todo - find out
			"    v_color = v_col;\n"
			"}\n";
		const char *p_s_fragment_shader =
			"precision highp float;\n"
			"\n"
			"varying vec4 v_color;\n"
			"\n"
			"void main()\n"
			"{\n"
			"    gl_FragColor = v_color;\n"
			"}\n";
		// vertex / fragment shader source code

		const char *p_s_config =
			"vertex {\n"
			"	v_pos: 0;\n"
			"	v_col: 1;\n"
			"}\n";
		// shader configuration (doesn't actually need the newlines)
		// note this can't override layout qualifiers in the shader code

		std::string compile_log, config_log, link_log;
		if(!CompileConfigureLink(p_s_vertex_shader, p_s_fragment_shader,
		   p_s_config, compile_log, config_log, link_log, true))
			return false;
		// use the comfy function

		n_mvp_uniform = n_Get_Uniform_Location("t_mvp");
		n_scale_uniform = n_Get_Uniform_Location("f_scale");
		// get addresses of the uniforms

		CGLESShader::Bind();
		Uniform1f(n_scale_uniform, 1);

		return true;
	}

	void Bind(const Matrix4f &r_t_mvp) const
	{
		CGLESShader::Bind();
		// bind the shader

		UniformMatrix4fv(n_mvp_uniform, 1, false, &r_t_mvp[0][0]);
		// set the uniforms
	}

	void SetScale(float f_scale)
	{
		CGLESShader::Bind();
		// bind the shader

		Uniform1f(n_scale_uniform, f_scale);
	}
};

struct TCamTexProjectionShader : public CGLESShader {
	GLint n_mvp_uniform, n_mv_uniform, n_up_uniform,
		n_right_uniform, n_scale_uniform, //n_pru, n_ppu,
		n_sprite_scale_coeff_uniform, n_camera_params_tex_uniform,
		n_cam_poses_texel_width_uniform, n_cam_poses_texture_height_uniform,
		n_cam_poses_texel_s_uniform;

	DECLARE_MEGATEXTURE_UNIFORMS

	bool Compile()
	{
		const char *p_s_vertex_shader =
			"precision highp float;\n"
			"\n"
			"attribute vec3 v_pos;\n"
			"attribute vec4 v_cam_idx_px;\n"
			"\n"
			"uniform mat4 t_mv;\n"
			"uniform mat4 t_mvp;\n"
			"uniform float f_scale;\n"
			"\n"
			//"varying vec4 v_color;\n"
			"varying vec4 v_position;\n" // w is clip w
			//
			"varying mat3 t_0proj_mat;\n"
			"varying vec4 v_0proj_pos_w;\n"
			"varying vec4 v_0K;\n"
			"varying vec4 v_0tex_rect;\n"
			//
			"varying mat3 t_1proj_mat;\n"
			"varying vec4 v_1proj_pos_w;\n"
			"varying vec4 v_1K;\n"
			"varying vec4 v_1tex_rect;\n"
			//
			"varying mat3 t_2proj_mat;\n"
			"varying vec4 v_2proj_pos_w;\n"
			"varying vec4 v_2K;\n"
			"varying vec4 v_2tex_rect;\n"
			//
			"varying mat3 t_3proj_mat;\n"
			"varying vec4 v_3proj_pos_w;\n"
			"varying vec4 v_3K;\n"
			"varying vec4 v_3tex_rect;\n"
			"\n"
			"uniform float f_camera_poses_texel_w, f_camera_poses_height, f_camera_poses_time_s;\n"
			"uniform sampler2D n_camera_params_tex_unit;\n"
			"\n"
			"mat3 t_Quat_to_RotMatrix(vec4 v_quat)\n"
			"{\n"
			"    float x = v_quat.x, y = v_quat.y, z = v_quat.z, w = v_quat.w;\n"
			"    float x2 = x + x, y2 = y + y, z2 = z + z;\n"
			"    float xx2 = x * x2, yy2 = y * y2, zz2 = z * z2;\n"
			"    float yz2 = y * z2, wx2 = w * x2, xy2 = x * y2;\n"
			"    float wz2 = w * z2, xz2 = x * z2, wy2 = w * y2;\n"
			"    return mat3(1.0 - yy2 - zz2,       xy2 + wz2,       xz2 - wy2,\n"
			"                      xy2 - wz2, 1.0 - xx2 - zz2,       yz2 + wx2,\n"
			"                      xz2 + wy2,       yz2 - wx2, 1.0 - xx2 - yy2);\n"
			"}\n"
			"\n"
			"void main()\n"
			"{\n"
			"    vec4 v_cam_idx = v_cam_idx_px / f_camera_poses_height;\n"
			"    {vec2 v_cam = vec2(f_camera_poses_time_s, v_cam_idx.x);\n"
			"    {\n"
			"        vec4 v_proj_rot = texture2D(n_camera_params_tex_unit, v_cam);\n"
			"        vec4 v_proj_pos_w = texture2D(n_camera_params_tex_unit, v_cam + vec2(f_camera_poses_texel_w, .0));\n"
			"        mat3 t_mat = t_Quat_to_RotMatrix(v_proj_rot);\n"
			"        t_0proj_mat = transpose(t_mat);\n"
			"        v_0proj_pos_w.xyz = -(transpose(t_mat) * v_proj_pos_w.xyz);\n"
			"        v_0proj_pos_w.w = v_proj_pos_w.w;\n" // this is going from world to camera local
			"    }\n"
			"    v_0K = texture2D(n_camera_params_tex_unit, v_cam + vec2(f_camera_poses_texel_w * 2.0, .0));\n"
			"    v_0tex_rect = texture2D(n_camera_params_tex_unit, v_cam + vec2(f_camera_poses_texel_w * 3.0, .0));}\n"

			"    {vec2 v_cam = vec2(f_camera_poses_time_s, v_cam_idx.y);\n"
			"    {\n"
			"        vec4 v_proj_rot = texture2D(n_camera_params_tex_unit, v_cam);\n"
			"        vec4 v_proj_pos_w = texture2D(n_camera_params_tex_unit, v_cam + vec2(f_camera_poses_texel_w, .0));\n"
			"        mat3 t_mat = t_Quat_to_RotMatrix(v_proj_rot);\n"
			"        t_1proj_mat = transpose(t_mat);\n"
			"        v_1proj_pos_w.xyz = -(transpose(t_mat) * v_proj_pos_w.xyz);\n"
			"        v_1proj_pos_w.w = v_proj_pos_w.w;\n" // this is going from world to camera local
			"    }\n"
			"    v_1K = texture2D(n_camera_params_tex_unit, v_cam + vec2(f_camera_poses_texel_w * 2.0, .0));\n"
			"    v_1tex_rect = texture2D(n_camera_params_tex_unit, v_cam + vec2(f_camera_poses_texel_w * 3.0, .0));}\n"

			"    {vec2 v_cam = vec2(f_camera_poses_time_s, v_cam_idx.z);\n"
			"    {\n"
			"        vec4 v_proj_rot = texture2D(n_camera_params_tex_unit, v_cam);\n"
			"        vec4 v_proj_pos_w = texture2D(n_camera_params_tex_unit, v_cam + vec2(f_camera_poses_texel_w, .0));\n"
			"        mat3 t_mat = t_Quat_to_RotMatrix(v_proj_rot);\n"
			"        t_2proj_mat = transpose(t_mat);\n"
			"        v_2proj_pos_w.xyz = -(transpose(t_mat) * v_proj_pos_w.xyz);\n"
			"        v_2proj_pos_w.w = v_proj_pos_w.w;\n" // this is going from world to camera local
			"    }\n"
			"    v_2K = texture2D(n_camera_params_tex_unit, v_cam + vec2(f_camera_poses_texel_w * 2.0, .0));\n"
			"    v_2tex_rect = texture2D(n_camera_params_tex_unit, v_cam + vec2(f_camera_poses_texel_w * 3.0, .0));}\n"

			"    {vec2 v_cam = vec2(f_camera_poses_time_s, v_cam_idx.w);\n"
			"    {\n"
			"        vec4 v_proj_rot = texture2D(n_camera_params_tex_unit, v_cam);\n"
			"        vec4 v_proj_pos_w = texture2D(n_camera_params_tex_unit, v_cam + vec2(f_camera_poses_texel_w, .0));\n"
			"        mat3 t_mat = t_Quat_to_RotMatrix(v_proj_rot);\n"
			"        t_3proj_mat = transpose(t_mat);\n"
			"        v_3proj_pos_w.xyz = -(transpose(t_mat) * v_proj_pos_w.xyz);\n"
			"        v_3proj_pos_w.w = v_proj_pos_w.w;\n" // this is going from world to camera local
			"    }\n"
			"    v_3K = texture2D(n_camera_params_tex_unit, v_cam + vec2(f_camera_poses_texel_w * 2.0, .0));\n"
			"    v_3tex_rect = texture2D(n_camera_params_tex_unit, v_cam + vec2(f_camera_poses_texel_w * 3.0, .0));}\n"

			"    vec4 P;\n"
			"    gl_Position = P = t_mvp * vec4(v_pos, 1.0);\n"
			"    gl_PointSize = 1.0 * f_scale;\n" // not sure it is ok to write this if the primitive rasterized is different from GL_POINTS // todo - find out
			//"    v_color = vec4(1.0);\n"
			"    v_position.xyz = v_pos;\n"
			"    v_position.w = P.w * f_scale;\n" // need this for later
			"}\n";
		const char *p_s_fragment_shader =
			"precision highp float;\n"
			"\n"
			"uniform vec3 v_up, v_right;\n"
			"uniform float f_scale;\n"
			"\n"
			//"varying vec4 v_color;\n"
			"varying vec4 v_position;\n" // w is clip w
			//
			"varying mat3 t_0proj_mat;\n"
			"varying vec4 v_0proj_pos_w;\n"
			"varying vec4 v_0K;\n"
			"varying vec4 v_0tex_rect;\n"
			//
			"varying mat3 t_1proj_mat;\n"
			"varying vec4 v_1proj_pos_w;\n"
			"varying vec4 v_1K;\n"
			"varying vec4 v_1tex_rect;\n"
			//
			"varying mat3 t_2proj_mat;\n"
			"varying vec4 v_2proj_pos_w;\n"
			"varying vec4 v_2K;\n"
			"varying vec4 v_2tex_rect;\n"
			//
			"varying mat3 t_3proj_mat;\n"
			"varying vec4 v_3proj_pos_w;\n"
			"varying vec4 v_3K;\n"
			"varying vec4 v_3tex_rect;\n"
			"\n"
			MEGATEXTURE_SAMPLE_FUNCTION_BODY
			"\n"
			"vec2 v_Reproject(vec3 v_worldspace, mat3 v_cam_rot, vec3 v_cam_pos, vec4 v_cam_K, vec4 v_cam_rect)\n"
			"{\n"
			"    vec3 v_camspace = v_cam_rot * v_worldspace + v_cam_pos;\n"
			"    vec3 v_eyespace = vec3(v_camspace.xy * v_cam_K.xy + v_camspace.z * v_cam_K.zw, v_camspace.z);\n"
			"    return v_cam_rect.xy + clamp((v_eyespace.xy / v_eyespace.z), .0, 1.0) * v_cam_rect.zw;\n" // v_tex_rect.xy is origin, v_tex_rect.zw are width and height of the texture rectangle
			"}\n"
			"\n"
			"void main()\n"
			"{\n"
			"    if(f_scale > 2.0 && length(gl_PointCoord - .5) > .5) discard;\n" // discs instead of squares
			"    vec3 v_worldspace = v_position.xyz + ((gl_PointCoord.x - .5) * v_right -\n"
			"                                          (gl_PointCoord.y - .5) * v_up) * v_position.w;\n" // v_up and v_right scaled by 2

			"    vec2 v_tex0 = vec2(-1.0, -1.0);\n"
			"    vec2 v_tex1 = vec2(-1.0, -1.0);\n"
			"    vec2 v_tex2 = vec2(-1.0, -1.0);\n"
			"    vec2 v_tex3 = vec2(-1.0, -1.0);\n"
			"    vec4 v_col0 = (v_0proj_pos_w.w > .0)? " MEGATEXTURE_SAMPLE "(v_tex0 = v_Reproject(v_worldspace, t_0proj_mat, v_0proj_pos_w.xyz, v_0K, v_0tex_rect)) : vec4(.0);\n"
			"    vec4 v_col1 = (v_1proj_pos_w.w > .0)? " MEGATEXTURE_SAMPLE "(v_tex1 = v_Reproject(v_worldspace, t_1proj_mat, v_1proj_pos_w.xyz, v_1K, v_1tex_rect)) : vec4(.0);\n"
			"    vec4 v_col2 = (v_2proj_pos_w.w > .0)? " MEGATEXTURE_SAMPLE "(v_tex2 = v_Reproject(v_worldspace, t_2proj_mat, v_2proj_pos_w.xyz, v_2K, v_2tex_rect)) : vec4(.0);\n"
			"    vec4 v_col3 = (v_3proj_pos_w.w > .0)? " MEGATEXTURE_SAMPLE "(v_tex3 = v_Reproject(v_worldspace, t_3proj_mat, v_3proj_pos_w.xyz, v_3K, v_3tex_rect)) : vec4(.0);\n"

			"    v_col0 *= max(.0, .7071 - length(v_tex0 - vec2(.5, .5)));\n"
			"    v_col1 *= max(.0, .7071 - length(v_tex1 - vec2(.5, .5)));\n"
			"    v_col2 *= max(.0, .7071 - length(v_tex2 - vec2(.5, .5)));\n"
			"    v_col3 *= max(.0, .7071 - length(v_tex3 - vec2(.5, .5)));\n"

			"    gl_FragColor = (v_col0 + v_col1 + v_col2 + v_col3 + vec4(.0, 1e-5, .0, 1e-5)) /\n"
			"        (v_col0.w + v_col1.w + v_col2.w + v_col3.w + 1e-5);\n" // the last term is a hack for dealing with only zero weights
			"}\n";
		// vertex / fragment shader source code

		const char *p_s_config =
			"vertex {\n"
			"	v_cam_idx_px: 0;\n"
			"	v_pos: 1;\n"
			"}\n";
		// shader configuration (doesn't actually need the newlines)
		// note this can't override layout qualifiers in the shader code

		std::string compile_log, config_log, link_log;
		if(!CompileConfigureLink(p_s_vertex_shader, p_s_fragment_shader,
		   p_s_config, compile_log, config_log, link_log, true))
			return false;
		// use the comfy function

		/*n_pru = n_Get_Uniform_Location("v_proj_rot");
		n_ppu = n_Get_Uniform_Location("v_proj_pos_");*/
		n_mv_uniform = n_Get_Uniform_Location("t_mv");
		n_up_uniform = n_Get_Uniform_Location("v_up");
		n_right_uniform = n_Get_Uniform_Location("v_right");
		n_mvp_uniform = n_Get_Uniform_Location("t_mvp");
		n_scale_uniform = n_Get_Uniform_Location("f_scale");
		//n_sprite_scale_coeff_uniform = n_Get_Uniform_Location("f_sprite_scale_coeff");
		n_cam_poses_texel_width_uniform = n_Get_Uniform_Location("f_camera_poses_texel_w");
		n_cam_poses_texture_height_uniform = n_Get_Uniform_Location("f_camera_poses_height");
		n_cam_poses_texel_s_uniform = n_Get_Uniform_Location("f_camera_poses_time_s");
		//n_texture_uniform = n_Get_Uniform_Location("n_tex_unit");
		n_camera_params_tex_uniform = n_Get_Uniform_Location("n_camera_params_tex_unit");
		// get addresses of the uniforms

		CGLESShader::Bind();

		Uniform1f(n_scale_uniform, 1);
		//Uniform1i(n_texture_uniform, 0);
		MEGATEXTURE_UNIFORMS_INITILAIZER;
		Uniform1i(n_camera_params_tex_uniform, MEGATEXTURE_NEXT_FREE_TEXUNIT_INDEX);

		return true;
	}

	void Bind(const Matrix4f &r_t_mvp, const Matrix4f &r_t_mv, Vector3f v_up,
		Vector3f v_right, //Vector4f v_proj_rot, Vector3f v_proj_pos,
		float f_sprite_scale_coeff, float f_cam_poses_texel_width,
		float f_cam_poses_texture_height, float f_cam_poses_timeframe_s = 0) const
	{
		CGLESShader::Bind();
		// bind the shader

		f_cam_poses_timeframe_s += .5 * f_cam_poses_texel_width;

		v_up *= 2 / f_sprite_scale_coeff;
		v_right *= 2 / f_sprite_scale_coeff;
		// compensate gl_PointCoord being [0, 1] rather than [-1, 1] -> scale by 2

		UniformMatrix4fv(n_mv_uniform, 1, false, &r_t_mv[0][0]);
		UniformMatrix4fv(n_mvp_uniform, 1, false, &r_t_mvp[0][0]);
		Uniform3f(n_up_uniform, v_up.x, v_up.y, v_up.z);
		Uniform3f(n_right_uniform, v_right.x, v_right.y, v_right.z);
		/*Uniform4f(n_pru, v_proj_rot.x, v_proj_rot.y, v_proj_rot.z, v_proj_rot.w);
		Uniform3f(n_ppu, v_proj_pos.x, v_proj_pos.y, v_proj_pos.z);*/
		//Uniform1f(n_sprite_scale_coeff_uniform, 2 / f_sprite_scale_coeff); // baked into up and right
		Uniform1f(n_cam_poses_texel_width_uniform, f_cam_poses_texel_width);
		Uniform1f(n_cam_poses_texture_height_uniform, f_cam_poses_texture_height);
		Uniform1f(n_cam_poses_texel_s_uniform, f_cam_poses_timeframe_s);
		// set the uniforms
	}

	void SetScale(float f_scale)
	{
		CGLESShader::Bind();
		// bind the shader

		Uniform1f(n_scale_uniform, f_scale);
	}
};

struct TCamTexProjectionLerpShader : public CGLESShader {
	GLint n_mvp_uniform, n_mv_uniform, n_up_uniform,
		n_right_uniform, n_scale_uniform, //n_pru, n_ppu,
		n_sprite_scale_coeff_uniform, n_camera_params_tex_uniform,
		n_cam_poses_texel_width_uniform,
		n_cam_poses_texture_height_uniform, n_cam_poses_texel_s_uniform,
		n_time_uniform, n_color_uniform, n_opacity_uniform;

	DECLARE_MEGATEXTURE_UNIFORMS

	bool Compile()
	{
		const char *p_s_vertex_shader =
			"precision highp float;\n"
			"\n"
			"attribute vec3 v_pos0;\n"
			"attribute vec4 v_cam_idx0_px;\n"
			"attribute vec3 v_pos1;\n"
			"attribute vec4 v_cam_idx1_px;\n"
			"\n"
			"uniform mat4 t_mv;\n"
			"uniform mat4 t_mvp;\n"
			"uniform float f_scale, f_lerp_time;\n"
			"\n"
			//"varying vec4 v_color;\n"
			"varying vec4 v_position;\n" // w is clip w
			//
			"varying mat3 t_0proj_mat;\n"
			"varying vec4 v_0proj_pos_w;\n"
			"varying vec4 v_0K;\n"
			"varying vec4 v_0tex_rect;\n"
			//
			"varying mat3 t_1proj_mat;\n"
			"varying vec4 v_1proj_pos_w;\n"
			"varying vec4 v_1K;\n"
			"varying vec4 v_1tex_rect;\n"
			//
			"varying mat3 t_2proj_mat;\n"
			"varying vec4 v_2proj_pos_w;\n"
			"varying vec4 v_2K;\n"
			"varying vec4 v_2tex_rect;\n"
			//
			"varying mat3 t_3proj_mat;\n"
			"varying vec4 v_3proj_pos_w;\n"
			"varying vec4 v_3K;\n"
			"varying vec4 v_3tex_rect;\n"
			"\n"
			"uniform float f_camera_poses_texel_w, f_camera_poses_height;\n"
			"uniform vec2 v_camera_poses_time_s;\n"
			"uniform sampler2D n_camera_params_tex_unit;\n"
			"\n"
			"mat3 t_Quat_to_RotMatrix(vec4 v_quat)\n"
			"{\n"
			"    float x = v_quat.x, y = v_quat.y, z = v_quat.z, w = v_quat.w;\n"
			"    float x2 = x + x, y2 = y + y, z2 = z + z;\n"
			"    float xx2 = x * x2, yy2 = y * y2, zz2 = z * z2;\n"
			"    float yz2 = y * z2, wx2 = w * x2, xy2 = x * y2;\n"
			"    float wz2 = w * z2, xz2 = x * z2, wy2 = w * y2;\n"
			"    return mat3(1.0 - yy2 - zz2,       xy2 + wz2,       xz2 - wy2,\n"
			"                      xy2 - wz2, 1.0 - xx2 - zz2,       yz2 + wx2,\n"
			"                      xz2 + wy2,       yz2 - wx2, 1.0 - xx2 - yy2);\n"
			"}\n"
			"\n"
			"vec4 v_Slerp(vec4 v_quat0, vec4 v_quat1, float f_time)\n"
			"{\n"
			"    vec4 v_quat;\n"
			"    {\n"
			"        float f_cos = max(-1.0, dot(v_quat0, v_quat1));\n"
			"        float f_sign = sign(f_cos);\n"
			"        f_cos *= f_sign;\n" // avoid rotating the longer way if the quaternions have the opposite signs
			"        if(f_cos < .99999) {\n"
			"            float f_angle = acos(f_cos);\n"
			"            float f_sin = sin(f_angle);\n" // could use sqrt(1 - f_cos * f_cos)
			"            float f_inv_sin = 1.0 / f_sin;\n"
			"            float f_weight_p = sin((1.0 - f_time) * f_angle) * f_inv_sin;\n"
			"            float f_weight_q = sin(f_time * f_angle) * f_inv_sin * f_sign;\n" // flip sign of either quaternion
			"            v_quat = normalize(v_quat0) * f_weight_p + normalize(v_quat1) * f_weight_q;\n"
			"        } else\n"
			"            v_quat = normalize(mix(v_quat0, v_quat1, f_time));\n" // infinitesimal rotation, simple lerp good enough
			"    }\n"
			"    return v_quat;\n"
			"}\n"
			"\n"
			"void main()\n"
			"{\n"
			"    vec4 v_cam_idx0 = v_cam_idx0_px / f_camera_poses_height;\n"
			"    vec4 v_cam_idx1 = v_cam_idx1_px / f_camera_poses_height;\n"
			"    vec3 v_pos = mix(v_pos0, v_pos1, f_lerp_time);\n"
			"    vec4 v_cam_idx = mix(v_cam_idx0, v_cam_idx1, f_lerp_time);\n"

			"    {vec2 v_cam = vec2(v_camera_poses_time_s.x, v_cam_idx.x);\n"
			"    vec2 v_cam1 = vec2(v_camera_poses_time_s.y, v_cam_idx.x);\n"
			"    {\n"
			"        vec4 v_proj_rot = v_Slerp(texture2D(n_camera_params_tex_unit, v_cam),\n"
			"                                  texture2D(n_camera_params_tex_unit, v_cam1), f_lerp_time);\n"
			"        vec4 v_proj_pos_w = mix(texture2D(n_camera_params_tex_unit, v_cam + vec2(f_camera_poses_texel_w, .0)),\n"
			"                                texture2D(n_camera_params_tex_unit, v_cam1 + vec2(f_camera_poses_texel_w, .0)), f_lerp_time);\n"
			"        mat3 t_mat = transpose(t_Quat_to_RotMatrix(v_proj_rot));\n"
			"        t_0proj_mat = t_mat;\n"
			"        v_0proj_pos_w = vec4(-(t_mat * v_proj_pos_w.xyz), v_proj_pos_w.w);\n" // this is going from world to camera local
			"    }\n"
			"    v_0K = texture2D(n_camera_params_tex_unit, v_cam + vec2(f_camera_poses_texel_w * 2.0, .0));\n" // this does not change over time (not currently anyway, but it could change if we had the intrinsics in the solution and did not read them from the graph file (could add support for VERTEX_INTRINSICS and parse the observations to see which cameras link to which intrinsics))
			"    v_0tex_rect = texture2D(n_camera_params_tex_unit, v_cam + vec2(f_camera_poses_texel_w * 3.0, .0));}\n" // this does not change over time

			"    {vec2 v_cam = vec2(v_camera_poses_time_s.x, v_cam_idx.y);\n"
			"    vec2 v_cam1 = vec2(v_camera_poses_time_s.y, v_cam_idx.y);\n"
			"    {\n"
			"        vec4 v_proj_rot = v_Slerp(texture2D(n_camera_params_tex_unit, v_cam),\n"
			"                                  texture2D(n_camera_params_tex_unit, v_cam1), f_lerp_time);\n"
			"        vec4 v_proj_pos_w = mix(texture2D(n_camera_params_tex_unit, v_cam + vec2(f_camera_poses_texel_w, .0)),\n"
			"                                texture2D(n_camera_params_tex_unit, v_cam1 + vec2(f_camera_poses_texel_w, .0)), f_lerp_time);\n"
			"        mat3 t_mat = transpose(t_Quat_to_RotMatrix(v_proj_rot));\n"
			"        t_1proj_mat = t_mat;\n"
			"        v_1proj_pos_w = vec4(-(t_mat * v_proj_pos_w.xyz), v_proj_pos_w.w);\n" // this is going from world to camera local
			"    }\n"
			"    v_1K = texture2D(n_camera_params_tex_unit, v_cam + vec2(f_camera_poses_texel_w * 2.0, .0));\n"
			"    v_1tex_rect = texture2D(n_camera_params_tex_unit, v_cam + vec2(f_camera_poses_texel_w * 3.0, .0));}\n"

			"    {vec2 v_cam = vec2(v_camera_poses_time_s.x, v_cam_idx.z);\n"
			"    vec2 v_cam1 = vec2(v_camera_poses_time_s.y, v_cam_idx.z);\n"
			"    {\n"
			"        vec4 v_proj_rot = v_Slerp(texture2D(n_camera_params_tex_unit, v_cam),\n"
			"                                  texture2D(n_camera_params_tex_unit, v_cam1), f_lerp_time);\n"
			"        vec4 v_proj_pos_w = mix(texture2D(n_camera_params_tex_unit, v_cam + vec2(f_camera_poses_texel_w, .0)),\n"
			"                                texture2D(n_camera_params_tex_unit, v_cam1 + vec2(f_camera_poses_texel_w, .0)), f_lerp_time);\n"
			"        mat3 t_mat = transpose(t_Quat_to_RotMatrix(v_proj_rot));\n"
			"        t_2proj_mat = t_mat;\n"
			"        v_2proj_pos_w = vec4(-(t_mat * v_proj_pos_w.xyz), v_proj_pos_w.w);\n" // this is going from world to camera local
			"    }\n"
			"    v_2K = texture2D(n_camera_params_tex_unit, v_cam + vec2(f_camera_poses_texel_w * 2.0, .0));\n"
			"    v_2tex_rect = texture2D(n_camera_params_tex_unit, v_cam + vec2(f_camera_poses_texel_w * 3.0, .0));}\n"

			"    {vec2 v_cam = vec2(v_camera_poses_time_s.x, v_cam_idx.w);\n"
			"    vec2 v_cam1 = vec2(v_camera_poses_time_s.y, v_cam_idx.w);\n"
			"    {\n"
			"        vec4 v_proj_rot = v_Slerp(texture2D(n_camera_params_tex_unit, v_cam),\n"
			"                                  texture2D(n_camera_params_tex_unit, v_cam1), f_lerp_time);\n"
			"        vec4 v_proj_pos_w = mix(texture2D(n_camera_params_tex_unit, v_cam + vec2(f_camera_poses_texel_w, .0)),\n"
			"                                texture2D(n_camera_params_tex_unit, v_cam1 + vec2(f_camera_poses_texel_w, .0)), f_lerp_time);\n"
			"        mat3 t_mat = transpose(t_Quat_to_RotMatrix(v_proj_rot));\n"
			"        t_3proj_mat = t_mat;\n"
			"        v_3proj_pos_w = vec4(-(t_mat * v_proj_pos_w.xyz), v_proj_pos_w.w);\n" // this is going from world to camera local
			"    }\n"
			"    v_3K = texture2D(n_camera_params_tex_unit, v_cam + vec2(f_camera_poses_texel_w * 2.0, .0));\n"
			"    v_3tex_rect = texture2D(n_camera_params_tex_unit, v_cam + vec2(f_camera_poses_texel_w * 3.0, .0));}\n"

			"    vec4 P;\n"
			"    gl_Position = P = t_mvp * vec4(v_pos, 1.0);\n"
			"    gl_PointSize = 1.0 * f_scale;\n" // not sure it is ok to write this if the primitive rasterized is different from GL_POINTS // todo - find out
			//"    v_color = vec4(1.0);\n"
			"    v_position.xyz = v_pos;\n"
			"    v_position.w = P.w * f_scale;\n" // need this for later
			"}\n";
		const char *p_s_fragment_shader =
			"precision highp float;\n"
			"\n"
			"uniform vec3 v_up, v_right;\n"
			"uniform float f_scale;\n"
			"\n"
			//"varying vec4 v_color;\n"
			"varying vec4 v_position;\n" // w is clip w
			//
			"varying mat3 t_0proj_mat;\n"
			"varying vec4 v_0proj_pos_w;\n"
			"varying vec4 v_0K;\n"
			"varying vec4 v_0tex_rect;\n"
			//
			"varying mat3 t_1proj_mat;\n"
			"varying vec4 v_1proj_pos_w;\n"
			"varying vec4 v_1K;\n"
			"varying vec4 v_1tex_rect;\n"
			//
			"varying mat3 t_2proj_mat;\n"
			"varying vec4 v_2proj_pos_w;\n"
			"varying vec4 v_2K;\n"
			"varying vec4 v_2tex_rect;\n"
			//
			"varying mat3 t_3proj_mat;\n"
			"varying vec4 v_3proj_pos_w;\n"
			"varying vec4 v_3K;\n"
			"varying vec4 v_3tex_rect;\n"
			"\n"
			MEGATEXTURE_SAMPLE_FUNCTION_BODY
			"\n"
			"vec2 v_Reproject(vec3 v_worldspace, mat3 v_cam_rot, vec3 v_cam_pos, vec4 v_cam_K, vec4 v_cam_rect)\n"
			"{\n"
			"    vec3 v_camspace = v_cam_rot * v_worldspace + v_cam_pos;\n"
			"    vec3 v_eyespace = vec3(v_camspace.xy * v_cam_K.xy + v_camspace.z * v_cam_K.zw, v_camspace.z);\n"
			"    return v_cam_rect.xy + clamp((v_eyespace.xy / v_eyespace.z), .0, 1.0) * v_cam_rect.zw;\n" // v_tex_rect.xy is origin, v_tex_rect.zw are width and height of the texture rectangle
			"}\n"
			"\n"
			"uniform vec4 v_color;\n"
			"uniform float f_opacity;\n"
			"\n"
			"void main()\n"
			"{\n"
			"    if(f_scale > 2.0 && length(gl_PointCoord - .5) > .5) discard;\n" // discs instead of squares
			"    vec3 v_worldspace = v_position.xyz + ((gl_PointCoord.x - .5) * v_right -\n"
			"                                          (gl_PointCoord.y - .5) * v_up) * v_position.w;\n" // v_up and v_right scaled by 2

			"    vec2 v_tex0 = vec2(-1.0, -1.0);\n"
			"    vec2 v_tex1 = vec2(-1.0, -1.0);\n"
			"    vec2 v_tex2 = vec2(-1.0, -1.0);\n"
			"    vec2 v_tex3 = vec2(-1.0, -1.0);\n"
			"    vec4 v_col0 = (v_0proj_pos_w.w > 0.0)? " MEGATEXTURE_SAMPLE "(v_tex0 = v_Reproject(v_worldspace, t_0proj_mat, v_0proj_pos_w.xyz, v_0K, v_0tex_rect)) : vec4(.0);\n"
			"    vec4 v_col1 = (v_1proj_pos_w.w > 0.0)? " MEGATEXTURE_SAMPLE "(v_tex1 = v_Reproject(v_worldspace, t_1proj_mat, v_1proj_pos_w.xyz, v_1K, v_1tex_rect)) : vec4(.0);\n"
			"    vec4 v_col2 = (v_2proj_pos_w.w > 0.0)? " MEGATEXTURE_SAMPLE "(v_tex2 = v_Reproject(v_worldspace, t_2proj_mat, v_2proj_pos_w.xyz, v_2K, v_2tex_rect)) : vec4(.0);\n"
			"    vec4 v_col3 = (v_3proj_pos_w.w > 0.0)? " MEGATEXTURE_SAMPLE "(v_tex3 = v_Reproject(v_worldspace, t_3proj_mat, v_3proj_pos_w.xyz, v_3K, v_3tex_rect)) : vec4(.0);\n"

			"    v_col0 *= max(.0, .7071 - length(v_tex0 - vec2(.5, .5)));\n"
			"    v_col1 *= max(.0, .7071 - length(v_tex1 - vec2(.5, .5)));\n"
			"    v_col2 *= max(.0, .7071 - length(v_tex2 - vec2(.5, .5)));\n"
			"    v_col3 *= max(.0, .7071 - length(v_tex3 - vec2(.5, .5)));\n"

			"    vec4 v_out_color = (v_col0 + v_col1 + v_col2 + v_col3 + vec4(.0, 1e-5, .0, 1e-5)) /\n"
			"        (v_col0.w + v_col1.w + v_col2.w + v_col3.w + 1e-5);\n" // the last term is a hack for dealing with only zero weights
			"    gl_FragColor = mix(v_out_color,\n"
			"        vec4(v_color.xyz, 1.0), v_color.w) * vec4(vec3(1.0), f_opacity);\n"
			"}\n";
		// vertex / fragment shader source code

		const char *p_s_config =
			"vertex {\n"
			"	v_cam_idx0_px: 0;\n"
			"	v_pos0: 1;\n"
			"	v_cam_idx1_px: 2;\n"
			"	v_pos1: 3;\n"
			"}\n";
		// shader configuration (doesn't actually need the newlines)
		// note this can't override layout qualifiers in the shader code

		std::string compile_log, config_log, link_log;
		if(!CompileConfigureLink(p_s_vertex_shader, p_s_fragment_shader,
		   p_s_config, compile_log, config_log, link_log, true))
			return false;
		// use the comfy function

		/*n_pru = n_Get_Uniform_Location("v_proj_rot");
		n_ppu = n_Get_Uniform_Location("v_proj_pos_");*/
		n_mv_uniform = n_Get_Uniform_Location("t_mv");
		n_up_uniform = n_Get_Uniform_Location("v_up");
		n_right_uniform = n_Get_Uniform_Location("v_right");
		n_mvp_uniform = n_Get_Uniform_Location("t_mvp");
		n_scale_uniform = n_Get_Uniform_Location("f_scale");
		n_time_uniform = n_Get_Uniform_Location("f_lerp_time");
		n_color_uniform = n_Get_Uniform_Location("v_color");
		n_opacity_uniform = n_Get_Uniform_Location("f_opacity");
		//n_sprite_scale_coeff_uniform = n_Get_Uniform_Location("f_sprite_scale_coeff");
		n_cam_poses_texel_width_uniform = n_Get_Uniform_Location("f_camera_poses_texel_w");
		n_cam_poses_texture_height_uniform = n_Get_Uniform_Location("f_camera_poses_height");
		n_cam_poses_texel_s_uniform = n_Get_Uniform_Location("v_camera_poses_time_s");
		//n_texture_uniform = n_Get_Uniform_Location("n_tex_unit");
		n_camera_params_tex_uniform = n_Get_Uniform_Location("n_camera_params_tex_unit");
		// get addresses of the uniforms

		CGLESShader::Bind();

		Uniform1f(n_scale_uniform, 1);
		//Uniform1i(n_texture_uniform, 0);
		MEGATEXTURE_UNIFORMS_INITILAIZER;
		Uniform1i(n_camera_params_tex_uniform, MEGATEXTURE_NEXT_FREE_TEXUNIT_INDEX);

		return true;
	}

	void Bind(const Matrix4f &r_t_mvp, const Matrix4f &r_t_mv, Vector3f v_up,
		Vector3f v_right, //Vector4f v_proj_rot, Vector3f v_proj_pos,
		float f_sprite_scale_coeff, float f_cam_poses_texel_width,
		float f_cam_poses_texture_height, float f_cam_poses_timeframe0_s,
		float f_cam_poses_timeframe1_s, float f_lerp_time, float f_opacity = 1,
		float f_r = 1, float f_g = 1, float f_b = 1, float f_a = 0) const
	{
		CGLESShader::Bind();
		// bind the shader

		f_cam_poses_timeframe0_s += .5 * f_cam_poses_texel_width;
		f_cam_poses_timeframe1_s += .5 * f_cam_poses_texel_width;

		v_up *= 2 / f_sprite_scale_coeff;
		v_right *= 2 / f_sprite_scale_coeff;
		// compensate gl_PointCoord being [0, 1] rather than [-1, 1] -> scale by 2

		UniformMatrix4fv(n_mv_uniform, 1, false, &r_t_mv[0][0]);
		UniformMatrix4fv(n_mvp_uniform, 1, false, &r_t_mvp[0][0]);
		Uniform3f(n_up_uniform, v_up.x, v_up.y, v_up.z);
		Uniform3f(n_right_uniform, v_right.x, v_right.y, v_right.z);
		/*Uniform4f(n_pru, v_proj_rot.x, v_proj_rot.y, v_proj_rot.z, v_proj_rot.w);
		Uniform3f(n_ppu, v_proj_pos.x, v_proj_pos.y, v_proj_pos.z);*/
		//Uniform1f(n_sprite_scale_coeff_uniform, 2 / f_sprite_scale_coeff); // baked into up and right
		Uniform1f(n_cam_poses_texel_width_uniform, f_cam_poses_texel_width);
		Uniform1f(n_cam_poses_texture_height_uniform, f_cam_poses_texture_height);
		Uniform2f(n_cam_poses_texel_s_uniform, f_cam_poses_timeframe0_s, f_cam_poses_timeframe1_s);
		Uniform1f(n_time_uniform, f_lerp_time);
		Uniform1f(n_opacity_uniform, f_opacity);
		Uniform4f(n_color_uniform, f_r, f_g, f_b, f_a);
		// set the uniforms
	}

	void SetScale(float f_scale)
	{
		CGLESShader::Bind();
		// bind the shader

		Uniform1f(n_scale_uniform, f_scale);
	}
};

struct TTextureShader : public CGLESShader {
	GLint n_texture_unit_uniform, n_scale_uniform;
	GLint n_mvp_uniform, n_color_uniform;

	bool Compile()
	{
		const char *p_s_vertex_shader =
			"precision highp float;\n"
			"\n"
			"attribute vec3 v_pos;\n"
			"attribute float v_tex;\n"
			"\n"
			"uniform mat4 t_mvp;\n"
			"uniform float f_scale;\n"
			"\n"
			"varying float v_texcoord;\n"
			"\n"
			"void main()\n"
			"{\n"
			"    gl_Position = t_mvp * vec4(v_pos, 1.0);\n"
			"    v_texcoord = v_tex;\n"
			"    gl_PointSize = 1.0 * f_scale;\n" // not sure it is ok to write this if the primitive rasterized is different from GL_POINTS // todo - find out
			"}\n";
		const char *p_s_fragment_shader =
			"precision highp float;\n"
			"\n"
			"uniform vec4 v_color;\n"
			"uniform sampler2D n_texture_unit;\n"
			"\n"
			"varying float v_texcoord;\n"
			"\n"
			"void main()\n"
			"{\n"
			"    gl_FragColor = texture2D(n_texture_unit, vec2(v_texcoord, v_texcoord)) * v_color;\n"
			"}\n";
		// vertex / fragment shader source code

		const char *p_s_config =
			"vertex {\n"
			"	v_pos: 0;\n" // pos always 0
			"	v_tex: 1;\n"
			"}\n";
		// shader configuration (doesn't actually need the newlines)
		// note this can't override layout qualifiers in the shader code

		std::string compile_log, config_log, link_log;
		if(!CompileConfigureLink(p_s_vertex_shader, p_s_fragment_shader,
		   p_s_config, compile_log, config_log, link_log, true))
			return false;
		// use the comfy function

		n_mvp_uniform = n_Get_Uniform_Location("t_mvp");
		n_color_uniform = n_Get_Uniform_Location("v_color");
		n_texture_unit_uniform = n_Get_Uniform_Location("n_texture_unit");
		n_scale_uniform = n_Get_Uniform_Location("f_scale");
		// get addresses of the uniforms

		CGLESShader::Bind();
		// bind the shader

		Uniform1i(n_texture_unit_uniform, 0);
		// always assume the texture in unit 0

		Uniform1f(n_scale_uniform, 1);

		return true;
	}

	void Bind(const Matrix4f &r_t_mvp, float f_r = 1, float f_g = 1, float f_b = 1, float f_a = 1) const
	{
		CGLESShader::Bind();
		// bind the shader

		UniformMatrix4fv(n_mvp_uniform, 1, false, &r_t_mvp[0][0]);
		Uniform4f(n_color_uniform, f_r, f_g, f_b, f_a);
		// set the uniforms
	}

	void SetScale(float f_scale)
	{
		CGLESShader::Bind();
		// bind the shader

		Uniform1f(n_scale_uniform, f_scale);
	}
};

struct TCameraShader : public CGLESShader {
	GLint n_size_uniform;
	GLint n_mvp_uniform, n_color_uniform;

	bool Compile()
	{
		const char *p_s_vertex_shader =
			"precision highp float;\n"
			"\n"
			"#version 130\n"
			//"#extension GL_EXT_gpu_shader4 : enable\n"
			"\n"
			"attribute vec4 v_pos;\n"
			"attribute vec4 v_rot;\n"
			"\n"
			"uniform mat4 t_mvp;\n"
			"uniform vec3 v_size;\n"
			"\n"
			"void main()\n"
			"{\n"

#ifdef GV_SHADERS_AVOID_INTEGER_ARITHMETICS
			/*"    vec3 p_corners[5];\n"
			"    p_corners[0] = vec3(0, 0, 0);\n"
			"    p_corners[1] = vec3(1, 1, 1);\n"
			"    p_corners[2] = vec3(1, -1, 1);\n"
			"    p_corners[3] = vec3(-1, -1, 1);\n"
			"    p_corners[4] = vec3(-1, 1, 1);\n"
			"    vec3 v_local_pos = p_corners[i];\n"*/
			"    vec3 v_local_pos = vec3(\n"                                     //-.5|.5|1.5|2.5|3.5|4.5
			"        (1.0 - 2.0 * step(.0, v_pos.w - 2.5)) * v_size.y,\n"        // [0,  1,  1, -1, -1]
			"        (1.0 - 2.0 * step(abs(v_pos.w - 2.5), 1.0)) * v_size.z,\n"  // [0,  1, -1, -1,  1]
			"        v_size.x\n"                                                 // [0,  1,  1,  1,  1]
			"        ) * step(.5, v_pos.w);\n"
			// version without integer arithmetics
#else // GV_SHADERS_AVOID_INTEGER_ARITHMETICS
			"    int i = int(v_pos.w);\n"
			"    int ni = int(bool(i));\n"
			"    vec3 v_local_pos = vec3(\n"
			"        float(-int(~-i > ni) | ni) * v_size.y,\n"	// [0,  1,  1, -1, -1]
			"        float(-(-~1 & i) + ni) * v_size.z,\n"		// [0,  1, -1, -1,  1]
			"        float(ni) * v_size.x);\n"					// [0,  1,  1,  1,  1]
#endif // GV_SHADERS_AVOID_INTEGER_ARITHMETICS
			// calculate local position

			"    float x = v_rot.x, y = v_rot.y, z = v_rot.z, w = v_rot.w;\n"
			"    float x2 = x + x, y2 = y + y, z2 = z + z;\n"
			"    float xx2 = x * x2, yy2 = y * y2, zz2 = z * z2;\n"
			"    float yz2 = y * z2, wx2 = w * x2, xy2 = x * y2;\n"
			"    float wz2 = w * z2, xz2 = x * z2, wy2 = w * y2;\n"
			"    mat3 t_mat = mat3(1.0 - yy2 - zz2,       xy2 + wz2,       xz2 - wy2,\n"
			"                            xy2 - wz2, 1.0 - xx2 - zz2,       yz2 + wx2,\n"
			"                            xz2 + wy2,       yz2 - wx2, 1.0 - xx2 - yy2);\n"
			// convert quaternion to a rotation matrix

			"    gl_Position = t_mvp * vec4(v_pos.xyz + t_mat * v_local_pos, 1.0);\n"
			"}\n";
		const char *p_s_fragment_shader =
			"precision highp float;\n"
			"\n"
			"uniform vec4 v_color;\n"
			"\n"
			"void main()\n"
			"{\n"
			"    gl_FragColor = v_color;\n" // constant color
			"}\n";
		// vertex / fragment shader source code

		const char *p_s_config =
			"vertex {\n"
			"	v_rot: 0;\n"
			"	v_pos: 1;\n"
			"}\n";
		// shader configuration (doesn't actually need the newlines)
		// note this can't override layout qualifiers in the shader code

		std::string compile_log, config_log, link_log;
		if(!CompileConfigureLink(p_s_vertex_shader, p_s_fragment_shader,
		   p_s_config, compile_log, config_log, link_log, true))
			return false;
		// use the comfy function

		n_mvp_uniform = n_Get_Uniform_Location("t_mvp");
		n_size_uniform = n_Get_Uniform_Location("v_size");
		n_color_uniform = n_Get_Uniform_Location("v_color");
		// get addresses of the uniforms

		CGLESShader::Bind();
		// bind the shader

		return true;
	}

	void Bind(const Matrix4f &r_t_mvp, float f_cam_size = .1f, float f_cam_plane_size_x = .1f,
		float f_cam_plane_size_y = .1f, float f_r = 1, float f_g = 1, float f_b = 1, float f_a = 1) const
	{
		CGLESShader::Bind();
		// bind the shader

		UniformMatrix4fv(n_mvp_uniform, 1, false, &r_t_mvp[0][0]);
		Uniform4f(n_color_uniform, f_r, f_g, f_b, f_a);
		Uniform3f(n_size_uniform, f_cam_size, f_cam_plane_size_x, f_cam_plane_size_y);
		// set the uniforms
	}
};

struct TCameraTextureShader : public CGLESShader {
	GLint n_size_uniform, n_mvp_uniform, n_color_uniform;
	DECLARE_MEGATEXTURE_UNIFORMS

	bool Compile()
	{
		const char *p_s_vertex_shader =
			"precision highp float;\n"
			"\n"
			"#version 130\n"
			//"#extension GL_EXT_gpu_shader4 : enable\n"
			"\n"
			"attribute vec4 v_pos;\n"
			"attribute vec4 v_rot;\n"
			"attribute vec2 v_tex;\n"
			"\n"
			"uniform mat4 t_mvp;\n"
			"uniform vec3 v_size;\n"
			"\n"
			"varying vec2 v_texcoord;\n"
			"\n"
			"void main()\n"
			"{\n"
#ifdef GV_SHADERS_AVOID_INTEGER_ARITHMETICS
			/*"    vec3 p_corners[5];\n"
			"    p_corners[0] = vec3(0, 0, 0);\n"
			"    p_corners[1] = vec3(1, 1, 1);\n"
			"    p_corners[2] = vec3(1, -1, 1);\n"
			"    p_corners[3] = vec3(-1, -1, 1);\n"
			"    p_corners[4] = vec3(-1, 1, 1);\n"
			"    vec3 v_local_pos = p_corners[i];\n"*/
			"    vec3 v_local_pos = vec3(\n"                                     //-.5|.5|1.5|2.5|3.5|4.5
			"        (1.0 - 2.0 * step(.0, v_pos.w - 2.5)) * v_size.y,\n"        // [0,  1,  1, -1, -1]
			"        (1.0 - 2.0 * step(abs(v_pos.w - 2.5), 1.0)) * v_size.z,\n"  // [0,  1, -1, -1,  1]
			"        v_size.x\n"                                                 // [0,  1,  1,  1,  1]
			"        ) * step(.5, v_pos.w);\n"
			// version without integer arithmetics
#else // GV_SHADERS_AVOID_INTEGER_ARITHMETICS
			"    int i = int(v_pos.w);\n"
			"    int ni = int(bool(i));\n"
			"    vec3 v_local_pos = vec3(\n"
			"        float(-int(~-i > ni) | ni) * v_size.y,\n"	// [0,  1,  1, -1, -1]
			"        float(-(-~1 & i) + ni) * v_size.z,\n"		// [0,  1, -1, -1,  1]
			"        float(ni) * v_size.x);\n"					// [0,  1,  1,  1,  1]
#endif // GV_SHADERS_AVOID_INTEGER_ARITHMETICS
			// calculate local position

			"    float x = v_rot.x, y = v_rot.y, z = v_rot.z, w = v_rot.w;\n"
			"    float x2 = x + x, y2 = y + y, z2 = z + z;\n"
			"    float xx2 = x * x2, yy2 = y * y2, zz2 = z * z2;\n"
			"    float yz2 = y * z2, wx2 = w * x2, xy2 = x * y2;\n"
			"    float wz2 = w * z2, xz2 = x * z2, wy2 = w * y2;\n"
			"    mat3 t_mat = mat3(1.0 - yy2 - zz2,       xy2 + wz2,       xz2 - wy2,\n"
			"                            xy2 - wz2, 1.0 - xx2 - zz2,       yz2 + wx2,\n"
			"                            xz2 + wy2,       yz2 - wx2, 1.0 - xx2 - yy2);\n"
			// convert quaternion to a rotation matrix

			"    gl_Position = t_mvp * vec4(v_pos.xyz + t_mat * v_local_pos, 1.0);\n"
			"    v_texcoord = v_tex;\n"
			"}\n";
		const char *p_s_fragment_shader =
			"precision highp float;\n"
			"\n"
			"uniform vec4 v_color;\n"
			"\n"
			"varying vec2 v_texcoord;\n"
			"\n"
			//"uniform sampler2D n_tex_unit;\n"
			MEGATEXTURE_SAMPLE_FUNCTION_BODY
			"\n"
			/*"const vec3 p_LOD_colors[] = vec3[](vec3(1, 1, 1), vec3(1, 0, 0), vec3(1, 0, 1), vec3(1, 1, 0), vec3(0, 1, 0), vec3(0, 1, 1), vec3(0, 0, 1), vec3(0, 0, 0));\n"
			"\n"
			"vec3 v_ColorLookup(float x)\n" // 8 colors corresponding to LOD
			"{\n"
			"    float f_fract = mod(x, 1.0);\n"
			"    int n_layer = min(7, int(x));\n"
			"    int n_layer1 = min(7, n_layer + 1);\n"
			"    return mix(p_LOD_colors[n_layer], p_LOD_colors[n_layer1], f_fract);"
			"}\n"*/
			"\n"
			"void main()\n"
			"{\n"
			/*"    vec2 v_tex_size = vec2(4096.0, 4096.0);\n"
			"    float f_lod = dot(fwidth(v_texcoord * v_tex_size), vec2(.5));\n"
			"    f_lod = max(.0, log(f_lod) / log(2.0));\n"*/ // debug - LOD calculation (alternatively could use a handcrafted LOD texture, for 8 levels that would be 1x1, 2x2, 4x4, 8x8, 16x16, 32x32, 64x64, 128x128)
			"    gl_FragColor = /*vec4(v_ColorLookup(f_lod), 1.0) **/ v_color * " MEGATEXTURE_SAMPLE "(v_texcoord);\n" // texture and color (using BGRA for faster texture streaming)
			"}\n";
		// vertex / fragment shader source code

		const char *p_s_config =
			"vertex {\n"
			"	v_rot: 0;\n"
			"	v_pos: 1;\n"
			"	v_tex: 2;\n"
			"}\n";
		// shader configuration (doesn't actually need the newlines)
		// note this can't override layout qualifiers in the shader code

		std::string compile_log, config_log, link_log;
		if(!CompileConfigureLink(p_s_vertex_shader, p_s_fragment_shader,
		   p_s_config, compile_log, config_log, link_log, true))
			return false;
		// use the comfy function

		n_mvp_uniform = n_Get_Uniform_Location("t_mvp");
		//n_texture_uniform = n_Get_Uniform_Location("n_tex_unit");
		n_size_uniform = n_Get_Uniform_Location("v_size");
		n_color_uniform = n_Get_Uniform_Location("v_color");
		// get addresses of the uniforms

		CGLESShader::Bind();
		// bind the shader

		//Uniform1i(n_texture_uniform, 0);
		MEGATEXTURE_UNIFORMS_INITILAIZER;
		// assume the texture in unit 0

		return true;
	}

	void Bind(const Matrix4f &r_t_mvp, float f_cam_size = .1f, float f_cam_plane_size_x = .1f,
		float f_cam_plane_size_y = .1f, float f_r = 1, float f_g = 1, float f_b = 1, float f_a = 1) const
	{
		CGLESShader::Bind();
		// bind the shader

		UniformMatrix4fv(n_mvp_uniform, 1, false, &r_t_mvp[0][0]);
		Uniform4f(n_color_uniform, f_r, f_g, f_b, f_a);
		Uniform3f(n_size_uniform, f_cam_size, f_cam_plane_size_x, f_cam_plane_size_y);
		// set the uniforms
	}
};

struct TCameraTextureLerpShader : public CGLESShader {
	GLint n_size_uniform, n_mvp_uniform, n_color_uniform, n_time_uniform;
	DECLARE_MEGATEXTURE_UNIFORMS

	bool Compile()
	{
		const char *p_s_vertex_shader =
			"precision highp float;\n"
			"\n"
			"#version 130\n"
			//"#extension GL_EXT_gpu_shader4 : enable\n"
			"\n"
			"attribute vec4 v_pos0;\n"
			"attribute vec4 v_pos1;\n"
			"attribute vec4 v_rot0;\n"
			"attribute vec4 v_rot1;\n"
			"attribute vec2 v_tex;\n" // texcoords are not animated
			"\n"
			"uniform mat4 t_mvp;\n"
			"uniform vec3 v_size;\n"
			"uniform float f_time;\n"
			"\n"
			"varying vec2 v_texcoord;\n"
			"\n"
			"void main()\n"
			"{\n"
			"    vec3 v_pos = mix(v_pos0.xyz, v_pos1.xyz, f_time);\n"
			"    vec4 v_rot;\n"
			"    {\n"
			"        float f_cos = max(-1.0, dot(v_rot0, v_rot1));\n"
			"        float f_sign = sign(f_cos);\n"
			"        f_cos *= f_sign;\n" // avoid rotating the longer way if the quaternions have the opposite signs
			"        if(f_cos < .99999) {\n"
			"            float f_angle = acos(f_cos);\n"
			"            float f_sin = sin(f_angle);\n" // could use sqrt(1 - f_cos * f_cos)
			"            float f_inv_sin = 1.0 / f_sin;\n"
			"            float f_weight_p = sin((1.0 - f_time) * f_angle) * f_inv_sin;\n"
			"            float f_weight_q = sin(f_time * f_angle) * f_inv_sin * f_sign;\n" // flip sign of either quaternion
			"            v_rot = normalize(v_rot0) * f_weight_p + normalize(v_rot1) * f_weight_q;\n"
			"        } else\n"
			"            v_rot = normalize(mix(v_rot0, v_rot1, f_time));\n" // infinitesimal rotation, simple lerp good enough
			"    }\n"
			// spherical interpolation

#ifdef GV_SHADERS_AVOID_INTEGER_ARITHMETICS
			/*"    vec3 p_corners[5];\n"
			"    p_corners[0] = vec3(0, 0, 0);\n"
			"    p_corners[1] = vec3(1, 1, 1);\n"
			"    p_corners[2] = vec3(1, -1, 1);\n"
			"    p_corners[3] = vec3(-1, -1, 1);\n"
			"    p_corners[4] = vec3(-1, 1, 1);\n"
			"    vec3 v_local_pos = p_corners[i];\n"*/
			"    vec3 v_local_pos = vec3(\n"                                      //-.5|.5|1.5|2.5|3.5|4.5
			"        (1.0 - 2.0 * step(.0, v_pos0.w - 2.5)) * v_size.y,\n"        // [0,  1,  1, -1, -1]
			"        (1.0 - 2.0 * step(abs(v_pos0.w - 2.5), 1.0)) * v_size.z,\n"  // [0,  1, -1, -1,  1]
			"        v_size.x\n"                                                  // [0,  1,  1,  1,  1]
			"        ) * step(.5, v_pos0.w);\n"
			// version without integer arithmetics
#else // GV_SHADERS_AVOID_INTEGER_ARITHMETICS
			"    int i = int(v_pos0.w);\n" // for local position generation
			"    int ni = int(bool(i));\n"
			"    vec3 v_local_pos = vec3(\n"
			"        float(-int(~-i > ni) | ni) * v_size.y,\n"	// [0,  1,  1, -1, -1]
			"        float(-(-~1 & i) + ni) * v_size.z,\n"		// [0,  1, -1, -1,  1]
			"        float(ni) * v_size.x);\n"					// [0,  1,  1,  1,  1]
#endif // GV_SHADERS_AVOID_INTEGER_ARITHMETICS
			// calculate local position

			"    float x = v_rot.x, y = v_rot.y, z = v_rot.z, w = v_rot.w;\n"
			"    float x2 = x + x, y2 = y + y, z2 = z + z;\n"
			"    float xx2 = x * x2, yy2 = y * y2, zz2 = z * z2;\n"
			"    float yz2 = y * z2, wx2 = w * x2, xy2 = x * y2;\n"
			"    float wz2 = w * z2, xz2 = x * z2, wy2 = w * y2;\n"
			"    mat3 t_mat = mat3(1.0 - yy2 - zz2,       xy2 + wz2,       xz2 - wy2,\n"
			"                            xy2 - wz2, 1.0 - xx2 - zz2,       yz2 + wx2,\n"
			"                            xz2 + wy2,       yz2 - wx2, 1.0 - xx2 - yy2);\n"
			// convert quaternion to a rotation matrix

			"    gl_Position = t_mvp * vec4(v_pos.xyz + t_mat * v_local_pos, 1.0);\n"
			"    v_texcoord = v_tex;\n"
			"}\n";
		const char *p_s_fragment_shader =
			"precision highp float;\n"
			"\n"
			"uniform vec4 v_color;\n"
			"\n"
			"varying vec2 v_texcoord;\n"
			"\n"
			//"uniform sampler2D n_tex_unit;\n"
			MEGATEXTURE_SAMPLE_FUNCTION_BODY
			"\n"
			"const vec3 p_LOD_colors[] = vec3[](vec3(1, 1, 1), vec3(1, 0, 0), vec3(1, 0, 1), vec3(1, 1, 0), vec3(0, 1, 0), vec3(0, 1, 1), vec3(0, 0, 1), vec3(0, 0, 0));\n"
			"\n"
			"vec3 v_ColorLookup(float x)\n" // 8 colors corresponding to LOD
			"{\n"
			"    float f_fract = mod(x, 1.0);\n"
			"    int n_layer = min(7, int(x));\n"
			"    int n_layer1 = min(7, n_layer + 1);\n"
			"    return mix(p_LOD_colors[n_layer], p_LOD_colors[n_layer1], f_fract);"
			"}\n"
			"\n"
			"void main()\n"
			"{\n"
			/*"    vec2 v_tex_size = vec2(4096.0, 4096.0);\n"
			"    float f_lod = dot(fwidth(v_texcoord * v_tex_size), vec2(.5));\n"
			"    f_lod = max(.0, log(f_lod) / log(2.0));\n"*/ // debug - LOD calculation (alternatively could use a handcrafted LOD texture, for 8 levels that would be 1x1, 2x2, 4x4, 8x8, 16x16, 32x32, 64x64, 128x128)
			"    gl_FragColor = /*vec4(v_ColorLookup(f_lod), 1.0) **/ v_color * " MEGATEXTURE_SAMPLE "(v_texcoord);\n" // texture and color (using BGRA for faster texture streaming)
			"}\n";
		// vertex / fragment shader source code

		const char *p_s_config =
			"vertex {\n"
			"	v_rot0: 0;\n"
			"	v_pos0: 1;\n"
			"	v_tex: 2;\n"
			"	v_rot1: 3;\n"
			"	v_pos1: 4;\n"
			"}\n";
		// shader configuration (doesn't actually need the newlines)
		// note this can't override layout qualifiers in the shader code

		std::string compile_log, config_log, link_log;
		if(!CompileConfigureLink(p_s_vertex_shader, p_s_fragment_shader,
		   p_s_config, compile_log, config_log, link_log, true))
			return false;
		// use the comfy function

		n_mvp_uniform = n_Get_Uniform_Location("t_mvp");
		//n_texture_uniform = n_Get_Uniform_Location("n_tex_unit");
		n_size_uniform = n_Get_Uniform_Location("v_size");
		n_color_uniform = n_Get_Uniform_Location("v_color");
		n_time_uniform = n_Get_Uniform_Location("f_time");
		// get addresses of the uniforms

		CGLESShader::Bind();
		// bind the shader

		//Uniform1i(n_texture_uniform, 0);
		MEGATEXTURE_UNIFORMS_INITILAIZER;
		// assume the texture in unit 0

		return true;
	}

	void Bind(const Matrix4f &r_t_mvp, float f_time, float f_cam_size = .1f, float f_cam_plane_size_x = .1f,
		float f_cam_plane_size_y = .1f, float f_r = 1, float f_g = 1, float f_b = 1, float f_a = 1) const
	{
		CGLESShader::Bind();
		// bind the shader

		UniformMatrix4fv(n_mvp_uniform, 1, false, &r_t_mvp[0][0]);
		Uniform1f(n_time_uniform, f_time);
		Uniform4f(n_color_uniform, f_r, f_g, f_b, f_a);
		Uniform3f(n_size_uniform, f_cam_size, f_cam_plane_size_x, f_cam_plane_size_y);
		// set the uniforms
	}
};

struct TConstantColorLerpShader : public CGLESShader {
	GLint n_mvp_uniform, n_time_uniform, n_color_uniform, n_scale_uniform;

	bool Compile()
	{
		const char *p_s_vertex_shader =
			"precision highp float;\n"
			"\n"
			"attribute vec3 v_pos;\n"
			"attribute vec3 v_pos1;\n"
			"\n"
			"uniform mat4 t_mvp;\n"
			"uniform float f_scale, f_time;\n"
			"\n"
			"void main()\n"
			"{\n"
			"    gl_Position = t_mvp * vec4(mix(v_pos, v_pos1, f_time), 1.0);\n"
			"    gl_PointSize = 1.0 * f_scale;\n" // not sure it is ok to write this if the primitive rasterized is different from GL_POINTS // todo - find out
			"}\n";
		const char *p_s_fragment_shader =
			"precision highp float;\n"
			"\n"
			"uniform vec4 v_color;\n"
			"\n"
			"void main()\n"
			"{\n"
			"    gl_FragColor = v_color;\n"
			"}\n";
		// vertex / fragment shader source code

		const char *p_s_config =
			"vertex {\n"
			"	v_pos: 0;\n"
			"	v_pos1: 1;\n"
			"}\n";
		// shader configuration (doesn't actually need the newlines)
		// note this can't override layout qualifiers in the shader code

		std::string compile_log, config_log, link_log;
		if(!CompileConfigureLink(p_s_vertex_shader, p_s_fragment_shader,
		   p_s_config, compile_log, config_log, link_log, true))
			return false;
		// use the comfy function

		n_mvp_uniform = n_Get_Uniform_Location("t_mvp");
		n_color_uniform = n_Get_Uniform_Location("v_color");
		n_scale_uniform = n_Get_Uniform_Location("f_scale");
		n_time_uniform = n_Get_Uniform_Location("f_time");
		// get addresses of the uniforms

		CGLESShader::Bind();
		Uniform1f(n_scale_uniform, 1);

		return true;
	}

	void Bind(const Matrix4f &r_t_mvp, float f_time, float f_r, float f_g, float f_b, float f_a = 1) const
	{
		CGLESShader::Bind();
		// bind the shader

		UniformMatrix4fv(n_mvp_uniform, 1, false, &r_t_mvp[0][0]);
		Uniform4f(n_color_uniform, f_r, f_g, f_b, f_a);
		Uniform1f(n_time_uniform, f_time);
		// set the uniforms
	}

	void SetScale(float f_scale)
	{
		CGLESShader::Bind();
		// bind the shader

		Uniform1f(n_scale_uniform, f_scale);
	}
};

struct TCameraLerpShader : public CGLESShader {
	GLint n_size_uniform, n_time_uniform;
	GLint n_mvp_uniform, n_color_uniform;

	bool Compile()
	{
		const char *p_s_vertex_shader =
			"precision highp float;\n"
			"\n"
			"#version 130\n"
			//"#extension GL_EXT_gpu_shader4 : enable\n"
			"\n"
			"attribute vec4 v_pos0;\n"
			"attribute vec4 v_rot0;\n"
			"attribute vec4 v_pos1;\n"
			"attribute vec4 v_rot1;\n"
			"\n"
			"uniform mat4 t_mvp;\n"
			"uniform vec3 v_size;\n"
			"uniform float f_time;\n"
			"\n"
			"void main()\n"
			"{\n"
			"    vec3 v_pos = mix(v_pos0.xyz, v_pos1.xyz, f_time);\n"

			"    vec4 v_rot;\n"

			//"    v_rot = normalize(mix(v_rot0, v_rot1, f_time));\n"
			// not a good example of quaternion interpolation

			"    {\n"
			"        float f_cos = max(-1.0, dot(v_rot0, v_rot1));\n"
			"        float f_sign = sign(f_cos);\n"
			"        f_cos *= f_sign;\n" // avoid rotating the longer way if the quaternions have the opposite signs
			"        if(f_cos < .99999) {\n"
			"            float f_angle = acos(f_cos);\n"
			"            float f_sin = sin(f_angle);\n" // could use sqrt(1 - f_cos * f_cos)
			"            float f_inv_sin = 1.0 / f_sin;\n"
			"            float f_weight_p = sin((1.0 - f_time) * f_angle) * f_inv_sin;\n"
			"            float f_weight_q = sin(f_time * f_angle) * f_inv_sin * f_sign;\n" // flip sign of either quaternion
			"            v_rot = normalize(v_rot0) * f_weight_p + normalize(v_rot1) * f_weight_q;\n"
			"        } else\n"
			"            v_rot = normalize(mix(v_rot0, v_rot1, f_time));\n" // infinitesimal rotation, simple lerp good enough
			"    }\n"
			// spherical interpolation

#ifdef GV_SHADERS_AVOID_INTEGER_ARITHMETICS
			/*"    vec3 p_corners[5];\n"
			"    p_corners[0] = vec3(0, 0, 0);\n"
			"    p_corners[1] = vec3(1, 1, 1);\n"
			"    p_corners[2] = vec3(1, -1, 1);\n"
			"    p_corners[3] = vec3(-1, -1, 1);\n"
			"    p_corners[4] = vec3(-1, 1, 1);\n"
			"    vec3 v_local_pos = p_corners[i];\n"*/
			"    vec3 v_local_pos = vec3(\n"                                      //-.5|.5|1.5|2.5|3.5|4.5
			"        (1.0 - 2.0 * step(.0, v_pos0.w - 2.5)) * v_size.y,\n"        // [0,  1,  1, -1, -1]
			"        (1.0 - 2.0 * step(abs(v_pos0.w - 2.5), 1.0)) * v_size.z,\n"  // [0,  1, -1, -1,  1]
			"        v_size.x\n"                                                  // [0,  1,  1,  1,  1]
			"        ) * step(.5, v_pos0.w);\n"
			// version without integer arithmetics
#else // GV_SHADERS_AVOID_INTEGER_ARITHMETICS
			"    int i = int(v_pos0.w);\n"
			"    int ni = int(bool(i));\n"
			"    vec3 v_local_pos = vec3(\n"
			"        float(-int(~-i > ni) | ni) * v_size.y,\n"	// [0,  1,  1, -1, -1]
			"        float(-(-~1 & i) + ni) * v_size.z,\n"		// [0,  1, -1, -1,  1]
			"        float(ni) * v_size.x);\n"					// [0,  1,  1,  1,  1]
#endif // GV_SHADERS_AVOID_INTEGER_ARITHMETICS
			// calculate local position

			"    float x = v_rot.x, y = v_rot.y, z = v_rot.z, w = v_rot.w;\n"
			"    float x2 = x + x, y2 = y + y, z2 = z + z;\n"
			"    float xx2 = x * x2, yy2 = y * y2, zz2 = z * z2;\n"
			"    float yz2 = y * z2, wx2 = w * x2, xy2 = x * y2;\n"
			"    float wz2 = w * z2, xz2 = x * z2, wy2 = w * y2;\n"
			"    mat3 t_mat = mat3(1.0 - yy2 - zz2,       xy2 + wz2,       xz2 - wy2,\n"
			"                            xy2 - wz2, 1.0 - xx2 - zz2,       yz2 + wx2,\n"
			"                            xz2 + wy2,       yz2 - wx2, 1.0 - xx2 - yy2);\n"
			// convert quaternion to a rotation matrix

			"    gl_Position = t_mvp * vec4(v_pos.xyz + t_mat * v_local_pos, 1.0);\n"
			"}\n";
		const char *p_s_fragment_shader =
			"precision highp float;\n"
			"\n"
			"uniform vec4 v_color;\n"
			"\n"
			"void main()\n"
			"{\n"
			"    gl_FragColor = v_color;\n" // constant color
			"}\n";
		// vertex / fragment shader source code

		const char *p_s_config =
			"vertex {\n"
			"	v_rot0: 0;\n"
			"	v_pos0: 1;\n"
			"	v_rot1: 2;\n"
			"	v_pos1: 3;\n"
			"}\n";
		// shader configuration (doesn't actually need the newlines)
		// note this can't override layout qualifiers in the shader code

		std::string compile_log, config_log, link_log;
		if(!CompileConfigureLink(p_s_vertex_shader, p_s_fragment_shader,
		   p_s_config, compile_log, config_log, link_log, true))
			return false;
		// use the comfy function

		n_mvp_uniform = n_Get_Uniform_Location("t_mvp");
		n_size_uniform = n_Get_Uniform_Location("v_size");
		n_color_uniform = n_Get_Uniform_Location("v_color");
		n_time_uniform = n_Get_Uniform_Location("f_time");
		// get addresses of the uniforms

		CGLESShader::Bind();
		// bind the shader

		return true;
	}

	void Bind(const Matrix4f &r_t_mvp, float f_time, float f_cam_size = .1f,
		float f_cam_plane_size_x = .1f, float f_cam_plane_size_y = .1f,
		float f_r = 1, float f_g = 1, float f_b = 1, float f_a = 1) const
	{
		CGLESShader::Bind();
		// bind the shader

		UniformMatrix4fv(n_mvp_uniform, 1, false, &r_t_mvp[0][0]);
		Uniform4f(n_color_uniform, f_r, f_g, f_b, f_a);
		Uniform3f(n_size_uniform, f_cam_size, f_cam_plane_size_x, f_cam_plane_size_y);
		Uniform1f(n_time_uniform, f_time);
		// set the uniforms
	}
};

struct TColorLerpShader : public CGLESShader {
	GLint n_mvp_uniform, n_scale_uniform, n_time_uniform,
		n_color_uniform, n_opacity_uniform;

	bool Compile()
	{
		const char *p_s_vertex_shader =
			"precision highp float;\n"
			"\n"
			"attribute vec3 v_pos0;\n"
			"attribute vec4 v_col0;\n"
			"attribute vec3 v_pos1;\n"
			"attribute vec4 v_col1;\n"
			"\n"
			"uniform mat4 t_mvp;\n"
			"uniform float f_scale, f_time;\n"
			"\n"
			"varying vec4 v_vertex_color;\n"
			"\n"
			"void main()\n"
			"{\n"
			"    gl_Position = t_mvp * vec4(v_pos0 + f_time * (v_pos1 - v_pos0), 1.0);\n"
			"    gl_PointSize = 1.0 * f_scale;\n" // not sure it is ok to write this if the primitive rasterized is different from GL_POINTS // todo - find out
			"    v_vertex_color = v_col0 + f_time * (v_col1 - v_col0);\n"
			"}\n";
		const char *p_s_fragment_shader =
			"precision highp float;\n"
			"\n"
			"varying vec4 v_vertex_color;\n"
			"\n"
			"uniform float f_opacity;\n"
			"uniform vec4 v_color;\n"
			"\n"
			"void main()\n"
			"{\n"
			"    gl_FragColor = mix(v_vertex_color,\n"
			"        vec4(v_color.xyz, 1.0), v_color.w) * vec4(vec3(1.0), f_opacity);\n"
			"}\n";
		// vertex / fragment shader source code

		const char *p_s_config =
			"vertex {\n"
			"	v_pos0: 0;\n"
			"	v_pos1: 1;\n"
			"	v_col0: 2;\n"
			"	v_col1: 3;\n"
			"}\n";
		// shader configuration (doesn't actually need the newlines)
		// note this can't override layout qualifiers in the shader code

		std::string compile_log, config_log, link_log;
		if(!CompileConfigureLink(p_s_vertex_shader, p_s_fragment_shader,
		   p_s_config, compile_log, config_log, link_log, true))
			return false;
		// use the comfy function

		n_mvp_uniform = n_Get_Uniform_Location("t_mvp");
		n_color_uniform = n_Get_Uniform_Location("v_color");
		n_opacity_uniform = n_Get_Uniform_Location("f_opacity");
		n_time_uniform = n_Get_Uniform_Location("f_time");
		n_scale_uniform = n_Get_Uniform_Location("f_scale");
		// get addresses of the uniforms

		CGLESShader::Bind();
		Uniform1f(n_scale_uniform, 1);

		return true;
	}

	void Bind(const Matrix4f &r_t_mvp, float f_lerp, float f_opacity = 1,
		float f_r = 1, float f_g = 1, float f_b = 1, float f_a = 0) const
	{
		CGLESShader::Bind();
		// bind the shader

		UniformMatrix4fv(n_mvp_uniform, 1, false, &r_t_mvp[0][0]);
		Uniform4f(n_color_uniform, f_r, f_g, f_b, f_a);
		Uniform1f(n_time_uniform, f_lerp);
		Uniform1f(n_opacity_uniform, f_opacity);
		// set the uniforms
	}

	void SetScale(float f_scale)
	{
		CGLESShader::Bind();
		// bind the shader

		Uniform1f(n_scale_uniform, f_scale);
	}
};

struct TTextureLerpShader : public CGLESShader {
	GLint n_texture_unit_uniform, n_scale_uniform, n_opacity_uniform, n_time_uniform;
	GLint n_mvp_uniform, n_color_uniform;

	bool Compile()
	{
		const char *p_s_vertex_shader =
			"precision highp float;\n"
			"\n"
			"attribute vec3 v_pos0;\n"
			"attribute float v_tex0;\n"
			"attribute vec3 v_pos1;\n"
			"attribute float v_tex1;\n"
			"\n"
			"uniform mat4 t_mvp;\n"
			"uniform float f_scale, f_time;\n"
			"\n"
			"varying float v_texcoord;\n"
			"\n"
			"void main()\n"
			"{\n"
			"    gl_Position = t_mvp * vec4(mix(v_pos0, v_pos1, f_time), 1.0);\n"
			"    v_texcoord = mix(v_tex0, v_tex1, f_time);\n"
			"    gl_PointSize = 1.0 * f_scale;\n" // not sure it is ok to write this if the primitive rasterized is different from GL_POINTS // todo - find out
			"}\n";
		const char *p_s_fragment_shader =
			"precision highp float;\n"
			"\n"
			"uniform float f_opacity;\n"
			"uniform vec4 v_color;\n"
			"uniform sampler2D n_texture_unit;\n"
			"\n"
			"varying float v_texcoord;\n"
			"\n"
			"void main()\n"
			"{\n"
			"    gl_FragColor = mix(texture2D(n_texture_unit, vec2(v_texcoord, v_texcoord)),\n"
			"        vec4(v_color.xyz, 1.0), v_color.w) * vec4(vec3(1.0), f_opacity);\n"
			"}\n";
		// vertex / fragment shader source code

		const char *p_s_config =
			"vertex {\n"
			"	v_pos0: 0;\n"
			"	v_tex0: 1;\n"
			"	v_pos1: 2;\n"
			"	v_tex1: 3;\n"
			"}\n";
		// shader configuration (doesn't actually need the newlines)
		// note this can't override layout qualifiers in the shader code

		std::string compile_log, config_log, link_log;
		if(!CompileConfigureLink(p_s_vertex_shader, p_s_fragment_shader,
		   p_s_config, compile_log, config_log, link_log, true))
			return false;
		// use the comfy function

		n_mvp_uniform = n_Get_Uniform_Location("t_mvp");
		n_color_uniform = n_Get_Uniform_Location("v_color");
		n_opacity_uniform = n_Get_Uniform_Location("f_opacity");
		n_time_uniform = n_Get_Uniform_Location("f_time");
		n_texture_unit_uniform = n_Get_Uniform_Location("n_texture_unit");
		n_scale_uniform = n_Get_Uniform_Location("f_scale");
		// get addresses of the uniforms

		CGLESShader::Bind();
		// bind the shader

		Uniform1i(n_texture_unit_uniform, 0);
		// always assume the texture in unit 0

		Uniform1f(n_scale_uniform, 1);

		return true;
	}

	void Bind(const Matrix4f &r_t_mvp, float f_lerp, float f_opacity = 1,
		float f_r = 1, float f_g = 1, float f_b = 1, float f_a = 0) const
	{
		CGLESShader::Bind();
		// bind the shader

		UniformMatrix4fv(n_mvp_uniform, 1, false, &r_t_mvp[0][0]);
		Uniform4f(n_color_uniform, f_r, f_g, f_b, f_a);
		Uniform1f(n_time_uniform, f_lerp);
		Uniform1f(n_opacity_uniform, f_opacity);
		// set the uniforms
	}

	void SetScale(float f_scale)
	{
		CGLESShader::Bind();
		// bind the shader

		Uniform1f(n_scale_uniform, f_scale);
	}
};

#endif // !__GRAPH_VIEW_SHADERS_INCLUDED
