/*
								+--------------------------------+
								|                                |
								| ***  OpenGL infinite grid  *** |
								|                                |
								|  Copyright © -tHE SWINe- 2014  |
								|                                |
								|             Grid.h             |
								|                                |
								+--------------------------------+
*/

#pragma once
#ifndef __GLES20_GRID_ICNLUDED
#define __GLES20_GRID_ICNLUDED

/**
 *	@file Grid.h
 *	@brief OpenGL infinite grid primitive
 *	@author -tHE SWINe-
 *	@date 2014
 */

/*

rendering the grid using lines is costy, especially if the grid is infinite. also, antialiassed grid rasterization
is quite costy. the depth / blending issues arise. rendering using lines leads to artifacts: either the grid is
rectangular, which is visible on the horizon, or the grid is made fo infinite lines and it sparse in the diagonals,
which is also visible on the horizon.

could precisely calculate all four corners of the polygon of intersection of the infinite plane and the view
frustum (tricky - there can also be three or none at all, depending on frustum position, would need to use frustum
bounding box + polygon culling).

that is, however unnecessary (and given numerical issues also maybe unwanted, as the polygon might not fit to the
edges of the screen), all that needs to be done is to control the front edge precisely for rasterization of the grid
when the front plane intersects it (closer than the other planes do). could calculate the intersection of the front
plane and the infinite plane, which is a line close to the camera and should be quite precise most of the time. then
can extend this line to infinity a) along the infinite plane, away from the center of projection and b) along the
front plane, again away from the center of projection.

could just draw a fullscreen quad and raytrace the intersection of view rays with the plane, that has just the
disadvantage of costy per pixel computation even when the plane is not visible (can branch that away, will be
contiguous in screen space). this also makes the grid trully infinite even with finite frustum.

for raytracing, need to calculate ray direction (easy part) and also the number of line segments that the ray
crosses (for blending).

what is remarkable is that even the moire patterns match between the old line rasterized version and the raytraced
version. the raytraced version allows much greater view distance, while being much faster to render.

*/

/**
 *	@def __GRID_SEE_SIDE_BY_SIDE
 *	@brief debugging flag; if defined the old rasterized grid is displayed alongside the raytraced one
 *		(old one on the left, new one on the right)
 */
//#define __GRID_SEE_SIDE_BY_SIDE

#ifdef __GRID_SEE_SIDE_BY_SIDE

/**
 *	@brief simple semi-infinite wire-frame grid displayed using vertices at infinity
 */
class CGridGeometry {
protected:
	CGLArraySetup *m_p_vbo; /**< @brief vertex buffers */

public:
	/**
	 *	@brief default constructor
	 *
	 *	@param[in] n_grid_size is number of grid lines in one direction
	 *	@param[in] f_line_distance is distance between the lines
	 *
	 *	@note It is advised to call b_Status() to see if constructor succeeded.
	 */
	CGridGeometry(size_t n_grid_size, float f_line_distance)
		:m_p_vbo(0)
	{
		try {
			size_t m_n_vertex_num = n_grid_size * 4;
			std::vector<float> vertex_list(3 * m_n_vertex_num); // 3D vertices (z is always 0)
			float *p_vertex = &vertex_list[0];
			for(size_t i = 0; i < n_grid_size; ++ i) {
				float f = (float(i) - n_grid_size * .5f) * f_line_distance; // must be signed
				*p_vertex ++ = f;
				*p_vertex ++ = 0;
				//*p_vertex ++ = 0;
				*p_vertex ++ = 1; // position on x-axis

				*p_vertex ++ = 0;
				*p_vertex ++ = 1;
				//*p_vertex ++ = 0;
				*p_vertex ++ = 0; // y+ vector

				*p_vertex ++ = f;
				*p_vertex ++ = 0;
				//*p_vertex ++ = 0;
				*p_vertex ++ = 1; // the same position on x-axis

				*p_vertex ++ = 0;
				*p_vertex ++ = -1;
				//*p_vertex ++ = 0;
				*p_vertex ++ = 0; // y- vector
			}
			_ASSERTE(p_vertex == &vertex_list[0] + 3 * m_n_vertex_num); // 3D!

			m_p_vbo = new CGLArraySetup(&vertex_list[0], vertex_list.size() *
				sizeof(float), 0, GL_FLOAT, 3, 0, 0, 0, 0, GL_LINES); // 3D!
		} catch(std::bad_alloc&) {
			if(m_p_vbo)
				delete m_p_vbo;
			m_p_vbo = 0; // to mark eror
		}
	}

	/**
	 *	@brief destructor; deletes OpenGL objects
	 */
	~CGridGeometry()
	{
		if(m_p_vbo)
			delete m_p_vbo;
	}

	/**
	 *	@brief gets grid OpenGL objects status
	 *	@return Returns true if cunstructor succeeded and grid
	 *		is good to be drawn, otherwise returns false.
	 */
	bool b_Status() const
	{
		return m_p_vbo != 0 && m_p_vbo->b_Status();
	}

	/**
	 *	@brief draws the grid in a single direction
	 *	@note This needs to be called twice to form a complete grid.
	 */
	void Draw() const
	{
		m_p_vbo->Draw();
		/*glEnableClientState(GL_VERTEX_ARRAY);
		glVertexPointer(4, GL_FLOAT, 4 * sizeof(float), m_p_vertex_list);

		glMatrixMode(GL_MODELVIEW);
		glPushMatrix();
		glRotatef(90, 1, 0, 0);
		glDrawArrays(GL_LINES, 0, m_n_vertex_num);
		glRotatef(90, 0, 0, 1);
		glDrawArrays(GL_LINES, 0, m_n_vertex_num);
		glPopMatrix();

		glDisableClientState(GL_VERTEX_ARRAY);*/
	}
};

#endif // __GRID_SEE_SIDE_BY_SIDE

/**
 *	@brief simple infinite wire-frame grid displayed using fullscreen quad and a raytracing shader
 */
class CGrid {
public:
#ifdef __GRID_SEE_SIDE_BY_SIDE

	struct TGridShader : public CGLESShader {
		GLint n_mvp_uniform, n_color_uniform, n_turn_uniform;

		bool Compile()
		{
			const char *p_s_vertex_shader =
				"precision highp float;\n"
				"\n"
				"attribute vec3 v_pos;\n"
				"\n"
				"uniform mat4 t_mvp;\n"
				"uniform bool b_turn;\n"
				"\n"
				"void main()\n"
				"{\n"
				"    vec2 v_grid_pos = (b_turn)? vec2(v_pos.y, -v_pos.x) : v_pos.xy;\n"
				// branch not pathologic, as it is constant over all the threads
				"    gl_Position = t_mvp * vec4(v_grid_pos.x, 0.0, v_grid_pos.y, v_pos.z);\n"
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
			n_turn_uniform = n_Get_Uniform_Location("b_turn");
			// get addresses of the uniforms

			return true;
		}

		void Bind(const Matrix4f &r_t_mvp, bool b_turn, float f_r, float f_g, float f_b, float f_a = 1) const
		{
			CGLESShader::Bind();
			// bind the shader

			UniformMatrix4fv(n_mvp_uniform, 1, false, &r_t_mvp[0][0]);
			Uniform4f(n_color_uniform, f_r, f_g, f_b, f_a);
			Uniform1i(n_turn_uniform, (b_turn)? 1 : 0);
			// set the uniforms
		}
	};

#endif // __GRID_SEE_SIDE_BY_SIDE

	struct TGridShader2 : public CGLESShader {
		GLint n_mvp_uniform, n_mvp_inv_uniform, n_color_uniform, n_width_uniform, n_grid_size_uniform;

		bool Compile()
		{
			const char *p_s_vertex_shader =
				"precision highp float;\n"
				"\n"
				"attribute vec2 v_pos;\n"
				"\n"
				"uniform mat4 t_mvp_inv;\n"
				"\n"
				"varying vec4 v_near4, v_far4;\n"
				"\n"
				"void main()\n"
				"{\n"
				"    gl_Position = vec4(v_pos.x, v_pos.y, 0.0, 1.0);\n"
				"    vec2 v_texcoord = v_pos;\n" // do not scale to [0, 1], this is supposed to be normalized
				"    v_near4 = t_mvp_inv * vec4(v_texcoord.x, v_texcoord.y, -1.0, 1.0);\n"
				"    v_far4 = t_mvp_inv * vec4(v_texcoord.x, v_texcoord.y, 1.0, 1.0);\n"
				"}\n";
			const char *p_s_fragment_shader =
				"precision highp float;\n"
				"\n"
				"uniform mat4 t_mvp;\n"
				"uniform vec4 v_color;\n"
				"uniform float f_line_width_px, f_grid_size;\n"
				"\n"
				"varying vec4 v_near4, v_far4;\n"
				"\n"
				"#define SquarePulse_Integral(x, p, notp) ((floor(x)*(p)) + max(fract(x)-(notp), 0.0))\n"
				"\n"
				"float f_GridAlpha(vec2 v_pos, vec2 d)\n"
				"{\n"
				//"    vec2 v_line = 1.0 - smoothstep(-d * .5 * f_line_width_px,\n"
				//"        d * .5 * f_line_width_px, abs(mod(v_pos.xz + .5, vec2(1.0)) - .5));\n" // smoothstep() is evil, the hermitean thing screws thin lines
				//"    vec2 v_line = 1.0 - clamp((abs(mod(v_pos.xz + .5, vec2(1.0)) - .5) + d * .5 * f_line_width_px) /\n"
				//"        (d * f_line_width_px), vec2(0.0), vec2(1.0));\n"
				//"    vec2 v_line_density = pow(d * f_line_width_px / f_grid_size, vec2(.35));\n"
				//"    if(max(v_line_density.x, v_line_density.y) > 1.0) discard;\n"
				//"    v_line = max(v_line, v_line_density - 1.0);\n" // prevent aliasing (not quite the correct formula)
				// point sampling, lot of noise and moire in the background

				"    vec2 v_line_thickness = d * f_line_width_px, v_blank_thickness = 1.0 - v_line_thickness;\n"
				"    vec2 v_line = (SquarePulse_Integral(v_pos + d * .5, v_line_thickness, v_blank_thickness) -\n"
				"        SquarePulse_Integral(v_pos - d * .5, v_line_thickness, v_blank_thickness)) / d;\n"
				// integral formula

				//"    float f_alpha = v_color.w * max(.0, max(v_line.x, v_line.y) /*+ .125 * min(v_line.x, v_line.y)*/);\n"
				"    float f_alpha = v_color.w * clamp(v_line.x + v_line.y, .0, 1.0);\n" // works too
				"    return sqrt(f_alpha);\n" // sqrt() helps antialiasing it a bit
				"}\n"
				"\n"
				"void main()\n"
				"{\n"
				"    vec3 v_near = v_near4.xyz / v_near4.w, v_far = v_far4.xyz / v_far4.w;\n" // dehomogenization needs to happen in fragment shader as it is nonlinear
				"    vec3 v_dir = normalize(v_far - v_near);\n"
				"    float t = -v_near.y / v_dir.y;\n" // ray-plane intersection; could/will generate a plenty of specials
				"    if(t < 0.0 /*-0.01 / v_dir.y*/ || abs(v_dir.y) < 1e-10)\n" // -near / dir.y seems funky and still leaves a bit of clipping, guess that camera pos would need to be used
				"        discard;\n" // t_odo - handle aliassing at the far plane (only visible when the view is tilted)
				// discard any specials (equals front plane clipping, we could do better than this)

				"    vec3 v_pos = (v_near + t * v_dir), v_pos_orig = v_pos; v_pos.xz /= f_grid_size;\n"
				// worldspace position of the intersection

				//"    vec2 dx = dFdx(v_pos.xz), dy = dFdy(v_pos.xz), d = vec2(sqrt(dx.x * dx.x + dy.x * dy.x), sqrt(dx.y * dx.y + dy.y * dy.y));\n" // take a difference on the domain
				"    vec2 d = fwidth(v_pos.xz);\n" // smoother
				// take a difference on the domain

#if 0
				"    float f_alpha = (f_GridAlpha(v_pos.xz + d * vec2(0.303334, -0.403554), d) +\n"
				"                     f_GridAlpha(v_pos.xz + d * vec2(0.171861, 0.078852), d) +\n"
				"                     f_GridAlpha(v_pos.xz + d * vec2(-0.208361, 0.403554), d) +\n"
				"                     f_GridAlpha(v_pos.xz + d * vec2(-0.303334, -0.087344), d)) * .25;\n"
				// Poisson disc sampling in 2D plane (diamond-shaped tiles)
#elif 1
				"    float f_alpha = (f_GridAlpha(v_pos.xz + d * vec2(0.331444, -0.166896), d) +\n"
				"                     f_GridAlpha(v_pos.xz + d * vec2(0.120716, -0.108198), d) +\n"
				"                     f_GridAlpha(v_pos.xz + d * vec2(0.382426, -0.379622), d) +\n"
				"                     f_GridAlpha(v_pos.xz + d * vec2(-0.092506, -0.157063), d) +\n"
				"                     f_GridAlpha(v_pos.xz + d * vec2(-0.388875, -0.054634), d) +\n"
				"                     f_GridAlpha(v_pos.xz + d * vec2(-0.290520, -0.250025), d) +\n"
				"                     f_GridAlpha(v_pos.xz + d * vec2(0.016195, -0.346893), d) +\n"
				"                     f_GridAlpha(v_pos.xz + d * vec2(-0.384734, -0.447447), d) +\n"
				"                     f_GridAlpha(v_pos.xz + d * vec2(0.295993, 0.048962), d) +\n"
				"                     f_GridAlpha(v_pos.xz + d * vec2(0.236491, 0.401883), d) +\n"
				"                     f_GridAlpha(v_pos.xz + d * vec2(0.394293, 0.250391), d) +\n"
				"                     f_GridAlpha(v_pos.xz + d * vec2(-0.018756, 0.060322), d) +\n"
				"                     f_GridAlpha(v_pos.xz + d * vec2(-0.394293, 0.164049), d) +\n"
				"                     f_GridAlpha(v_pos.xz + d * vec2(0.069644, 0.260414), d) +\n"
				"                     f_GridAlpha(v_pos.xz + d * vec2(-0.221466, 0.298147), d) +\n"
				"                     f_GridAlpha(v_pos.xz + d * vec2(-0.061587, 0.447447), d)) / 16.0;\n"
				// very little improvement over the 4x sampling
#else
				"    float f_alpha = f_GridAlpha(v_pos.xz, d);\n"
#endif
				// sample the grid

				"    f_alpha = clamp(f_alpha, .0, 1.0) * smoothstep(.0, length(1.44 * fwidth(v_dir.y)), abs(v_dir.y));\n"
				// handles aliassing at the far plane (especially visible when the view is tilted)

				"    if(f_alpha < 1.0 / 256.0)\n"
				"        discard;\n"
				// kill fragments that do not belong to any line

				"    gl_FragColor = vec4(v_color.xyz, f_alpha);\n"
				//"    gl_FragColor = vec4(max(v_line_density.x, v_line_density.y) > 1.0? v_color.zyx : v_color.xyz, f_alpha);\n"
				// note that the power formula fails miserably, switch to the integration method when the line density gets over 1.0 (or use it always)

				"    v_pos = v_pos_orig;//.xz *= f_grid_size;\n" // undo!

				"    vec4 v_clip_coord = t_mvp * vec4(v_pos, 1.0);\n"
				"    float f_ndc_depth = v_clip_coord.z / v_clip_coord.w;\n"
				"    gl_FragDepth = min(0.9999999, (gl_DepthRange.far - gl_DepthRange.near) * .5 * \n"
				"        f_ndc_depth + (gl_DepthRange.far + gl_DepthRange.near) * .5);\n"
				// calculate depth, if needed; note that the depth is clamped so that the parts in infinity are not clipped
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

			n_mvp_inv_uniform = n_Get_Uniform_Location("t_mvp_inv");
			n_mvp_uniform = n_Get_Uniform_Location("t_mvp");
			n_color_uniform = n_Get_Uniform_Location("v_color");
			n_width_uniform = n_Get_Uniform_Location("f_line_width_px");
			n_grid_size_uniform = n_Get_Uniform_Location("f_grid_size");
			// get addresses of the uniforms

			return true;
		}

		void Bind(const Matrix4f &r_t_mvp, const Matrix4f &r_t_mvp_inv,
			float f_line_width, float f_grid_size, float f_r, float f_g, float f_b, float f_a = 1) const
		{
			CGLESShader::Bind();
			// bind the shader

			//printf("rendering with w = %f\n", f_line_width);

			UniformMatrix4fv(n_mvp_inv_uniform, 1, false, &r_t_mvp_inv[0][0]);
			UniformMatrix4fv(n_mvp_uniform, 1, false, &r_t_mvp[0][0]);
			Uniform4f(n_color_uniform, f_r, f_g, f_b, f_a);
			Uniform1f(n_width_uniform, f_line_width);
			Uniform1f(n_grid_size_uniform, f_grid_size);
			// set the uniforms
		}
	};

protected:
#ifdef __GRID_SEE_SIDE_BY_SIDE
	CGridGeometry m_grid;
	TGridShader m_shader;
#endif // __GRID_SEE_SIDE_BY_SIDE
	CGLArraySetup m_fullscreen_tri;
	TGridShader2 m_shader2;
	float m_f_line_distance;

	static const float m_p_fullscreen_tri[6];

public:
	/**
	 *	@copydoc CGridGeometry::CGridGeometry
	 */
	CGrid(size_t n_grid_size, float f_line_distance)
#ifdef __GRID_SEE_SIDE_BY_SIDE
		:m_grid(n_grid_size, f_line_distance), m_f_line_distance(f_line_distance),
#else // __GRID_SEE_SIDE_BY_SIDE
		:m_f_line_distance(f_line_distance),
#endif // __GRID_SEE_SIDE_BY_SIDE
		m_fullscreen_tri(m_p_fullscreen_tri, sizeof(m_p_fullscreen_tri),
		2 * sizeof(float), GL_FLOAT, 2, 0, 0, 0, 0, GL_TRIANGLES)
	{
#ifdef __GRID_SEE_SIDE_BY_SIDE
		m_shader.Compile();
#endif // __GRID_SEE_SIDE_BY_SIDE
		m_shader2.Compile();
	}

	/**
	 *	@copydoc CGridGeometry::b_Status
	 */
	bool b_Status() const
	{
		return
#ifdef __GRID_SEE_SIDE_BY_SIDE
			m_grid.b_Status() && m_shader.b_Compiled() &&
#endif // __GRID_SEE_SIDE_BY_SIDE
			m_fullscreen_tri.b_Status() && m_shader2.b_Compiled();
	}

	/**
	 *	@brief draws the grid
	 *
	 *	@param[in] r_t_mvp is modelview-projection matrix
	 *	@param[in] v_color is color of the grid
	 */
	void Draw(const Matrix4f &r_t_mvp, Vector4f v_color, float f_line_width) const
	{
#ifdef __GRID_SEE_SIDE_BY_SIDE
		int p_wp[4];
		glGetIntegerv(GL_VIEWPORT, p_wp);
		const int w = p_wp[2], h = p_wp[3];

		glScissor(0, 0, w / 2, h);
		glEnable(GL_SCISSOR_TEST);

		m_shader.Bind(r_t_mvp, false, v_color.x, v_color.y, v_color.z, v_color.w);
		m_grid.Draw();
		m_shader.Bind(r_t_mvp, true, v_color.x, v_color.y, v_color.z, v_color.w);
		m_grid.Draw(); // draw rotated 90 degrees

		glScissor(w / 2, 0, w / 2, h);
#endif // __GRID_SEE_SIDE_BY_SIDE

		// todo - figure out how to modify the matrix to avoid the grid shaking when the camera is far away from the origin

		m_shader2.Bind(r_t_mvp, r_t_mvp.t_Inverse(), f_line_width, // note that with the integral, it happily rasterizes lines thinner than 1.0
			m_f_line_distance, v_color.x, v_color.y, v_color.z, v_color.w);
		m_fullscreen_tri.Draw();

#ifdef __GRID_SEE_SIDE_BY_SIDE
		glDisable(GL_SCISSOR_TEST);
#endif // __GRID_SEE_SIDE_BY_SIDE
	}
};

const float CGrid::m_p_fullscreen_tri[] = {-1, 1, 3, 1, -1, -3};
// a (2D) fullscreen triangle

#endif // !__GLES20_GRID_ICNLUDED
