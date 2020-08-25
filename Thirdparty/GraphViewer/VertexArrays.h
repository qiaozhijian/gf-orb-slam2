/*
								+---------------------------------+
								|                                 |
								| *** OpenGL multi-VBO arrays *** |
								|                                 |
								|  Copyright  © -tHE SWINe- 2015  |
								|                                 |
								|         VertexArrays.h          |
								|                                 |
								+---------------------------------+
*/

#pragma once
#ifndef __GL_MULTIPLE_VBO_VERTEX_ARRAYS_INCLUDED
#define __GL_MULTIPLE_VBO_VERTEX_ARRAYS_INCLUDED

/**
 *	@file VertexArrays.h
 *	@brief OpenGL reconfigurable vertex arrays
 *	@author -tHE SWINe-
 *	@date 2015
 *
 *	This is very much an experiment, it is likely unneccessarily complicated for everyday use.
 *	However, it is optimal both in memory consumption and speed.
 *
 *	This file assumes that either gl4/BufferObjects.h, gles2/BufferObjects.h or an equivalent file
 *	is included prior including this file (this implementation will work with multiple versions).
 *
 *	The lame CGLArraySetup can only handle vertex array with two vertex attributes, stored in
 *	a single buffer, in interleaved fashion. CGLMultiArraySetup can handle an arbitrary number
 *	of attributes in arbitrary number of buffers, interleaved or not. To use it, one does:
 *
 *	@code
 *	std::vector<Vector2f> vertex_list; // filled with interleaved positions / texcoords
 *	CGLMultiArraySetup<> my_quad(
 *		TGLVBOConfig(vertex_list)((
 *			GLVertexAttribPtrSpec(0, 2, GL_FLOAT, true, 2 * sizeof(Vector2f), sizeof(Vector2f)),
 *			GLVertexAttribPtrSpec(1, 2, GL_FLOAT, true, 2 * sizeof(Vector2f), 0)
 *		)), GL_TRIANGLE_STRIP, 4);
 *	my_quad.Draw();
 *	@endcode
 *
 *	This specifies an array setup with two vertex attributes, taken from a single buffer.
 *	In most cases, one can save some typing by:
 *
 *	@code
 *	std::vector<Vector2f> vertex_list; // filled with interleaved positions / texcoords
 *	CGLMultiArraySetup<> my_quad(
 *		TGLVBOConfig(vertex_list)((
 *			GLVertexAttribPtrShort(2, GL_FLOAT, 2 * sizeof(Vector2f), sizeof(Vector2f)),
 *			GLVertexAttribPtrShort(2, GL_FLOAT, 2 * sizeof(Vector2f), 0)
 *		)), GL_TRIANGLE_STRIP, 4);
 *	my_quad.Draw();
 *	@endcode
 *
 *	Note that here, the vertex attribute ids are generated automatically, each attribute having
 *	one larger id than the previous one and the first one having id zero (mixing with attributes
 *	with explicitly specified id is possible). The normalize flag is set to true. Also note
 *	that GLVertexAttribPtrShort cannot be used if there are multiple buffers (vertex attributes
 *	of each buffer would have the same ids starting from zero).
 *
 *	To have more buffers, one simply specifies multiple TGLVBOConfig:
 *
 *	@code
 *	std::vector<Vector2f> pos_list; // filled with positions
 *	std::vector<Vector2f> tex_list; // filled with texcoords
 *	CGLMultiArraySetup<2> my_quad(( // we need to say we need two VBOs, size of the object increases
 *		TGLVBOConfig(tex_list)((
 *			GLVertexAttribPtrSpec(0, 2, GL_FLOAT, 0, 0)
 *		)),
 *		TGLVBOConfig(pos_list)((
 *			GLVertexAttribPtrSpec(1, 2, GL_FLOAT, 0, 0)
 *		))
 *		), GL_TRIANGLE_STRIP, 4);
 *	my_quad.Draw();
 *	@endcode
 *
 *	This has two buffers, one vertex attribute in each. Note that any combinations of buffers
 *	and attributes are possible. Specifying buffers with no attributes is allowed but the buffers
 *	need to be specified in Draw_Attribs(), see below.
 *
 *	This all has the advantage of easily passing the configuration to the CGLMultiArraySetup.
 *	The number of vertex attributes does not change the size of the object, and is only limited
 *	by OpenGL / drivers / hardware. Even if VAOs are not available, all the attribute configuration
 *	is stored as a simple function pointer which is called before drawing.
 *
 *	The disadvantage is that the arguments to GLVertexAttribPtrSpec or GLVertexAttribPtrShort
 *	all need to be compile-time constants. While this is not a problem for some applications,
 *	other may need to change the array parameters on the fly. This is done using:
 *
 *	@code
 *	std::vector<float> vertex_list; // ...
 *	CGLMultiArraySetup<3> my_quad((
 *		TGLVBOConfig(vertex_list)(
 *			// we do not know what the format will be, leave it blank for now
 *		),
 *		TGLVBOConfig(vertex_list)((
 *			GLVertexAttribPtrSpec(2, 2, GL_FLOAT, true, 2 * sizeof(Vector2f), 0)
 *		)),
 *		TGLVBOConfig(vertex_list)((
 *			GLVertexAttribPtrSpec(3, 2, GL_FLOAT, true, 2 * sizeof(Vector2f), 4)
 *		))
 *		), GL_TRIANGLE_STRIP, 4);
 *	my_quad.Draw_Attribs((
 *		GLVertexAttribPtrVSpec(0, 2, GL_FLOAT, true, 2 * sizeof(Vector2f), sizeof(Vector2f)), // re-specify the format here
 *		GLVertexAttribPtrVSpec(1, 2, GL_FLOAT, true, 2 * sizeof(Vector2f), 0), // re-specify the format here
 *		++ // shift to the next buffer
 *		GLVertexAttribPtrVSpec(2, 2, GL_FLOAT, true, 2 * sizeof(Vector2f), 0),
 *		++
 *		GLVertexAttribPtrVSpec(3, 2, GL_FLOAT, true, 2 * sizeof(Vector2f), 4)
 *		), true);
 *	@endcode
 *
 *	The function Draw_Attribs() allows to set a different array parameters and either store it in
 *	the object's VAO (and then the next Draw() will use the same parameters), or create a temporary
 *	VAO (and then the next Draw() will use the original parameters). If reusing the object's VAO,
 *	it is not required to override all the vertex attributes - the ones that are not specified will
 *	remain the same. There are no checks of attribute count and / or sanity.
 *
 *	Mixing GLVertexAttribPtrVSpec with GLVertexAttribPtrSpec or GLVertexAttribPtrShort is not
 *	implemented at the moment (todo).
 *
 *	Note that GLVertexAttribPtrVSpec cannot be used in CGLMultiArraySetup constructor since the
 *	array config is not stored as a part of the buffer object (it would change the type). Only
 *	compile-time constant vertex attribute specifiers can be used in the constructor.
 *
 *	Note that in some cases (especially when VAOs are not available), one will need to explicitly
 *	disable attributes. Setting dimension zero in GLVertexAttribPtrSpec, GLVertexAttribPtrShort or
 *	GLVertexAttribPtrVSpec results in the attribute being disabled. To enable the attribute again,
 *	one must fully specify all its properties.
 *
 */

#if defined(_MSC_VER) && !defined(__MWERKS__)
#if _MSC_VER >= 1500
#define LVA_STRICTER_TYPE_CHECK 1
#else // _MSC_VER >= 1500
#define LVA_STRICTER_TYPE_CHECK 0
#endif // _MSC_VER >= 1500
#else // _MSC_VER && !__MWERKS__
#define LVA_STRICTER_TYPE_CHECK 1
#endif // _MSC_VER && !__MWERKS__
// fixes some template compatibility issues with old MSVC

#include "VertexArrays.inl"

/**
 *	@def GLVertexAttribPtrSpec
 *	@brief makes an instance of an OpenGL run-time constant vertex attrib pointer specifier
 *
 *	@param[in] n_index Specifies the index of the generic vertex attribute to be modified.
 *	@param[in] n_size Specifies the number of components per generic vertex attribute. Must be 1, 2,
 *		3, 4. Additionally, the symbolic constant GL_BGRA is accepted by glVertexAttribPointer. The
 *		initial value is 4. Specifying size 0 disables the attribute, the other arguments are then ignored.
 *	@param[in] n_data_type specifies the data type of each component in the array. The symbolic constants
 *		GL_BYTE, GL_UNSIGNED_BYTE, GL_SHORT, GL_UNSIGNED_SHORT, GL_INT, and GL_UNSIGNED_INT are
 *		accepted by both functions. Additionally GL_HALF_FLOAT, GL_FLOAT, GL_DOUBLE, GL_FIXED,
 *		GL_INT_2_10_10_10_REV, and GL_UNSIGNED_INT_2_10_10_10_REV are accepted by
 *		glVertexAttribPointer. The initial value is GL_FLOAT.
 *	@param[in] b_normalized Specifies whether fixed-point data values
 *		should be normalized (GL_TRUE) or converted directly as fixed-point values (GL_FALSE)
 *		when they are accessed.
 *	@param[in] n_stride_bytes Specifies the byte offset between consecutive generic vertex attributes. If
 *		stride is 0, the generic vertex attributes are understood to be tightly packed in the array.
 *		The initial value is 0.
 *	@param[in] n_offset_bytes Specifies a offset of the first component of the first generic vertex
 *		attribute in the array in the data store of the buffer currently bound to the
 *		GL_ARRAY_BUFFER target. The initial value is 0.
 */
#define GLVertexAttribPtrSpec(n_index,n_size,n_data_type,b_normalized,n_stride_bytes,n_offset_bytes) \
	(glbuffers_detail::CGLConfigOp_VertexAttribPtr<(n_index), (n_size), (n_data_type), (b_normalized), \
		(n_stride_bytes), (n_offset_bytes)>())

/**
 *	@def GLVertexAttribPtrShort
 *	@brief makes an instance of an OpenGL run-time constant vertex attrib pointer specifier
 *
 *	@param[in] n_size Specifies the number of components per generic vertex attribute. Must be 1, 2,
 *		3, 4. Additionally, the symbolic constant GL_BGRA is accepted by glVertexAttribPointer. The
 *		initial value is 4. Specifying size 0 disables the attribute, the other arguments are then ignored.
 *	@param[in] n_data_type specifies the data type of each component in the array. The symbolic constants
 *		GL_BYTE, GL_UNSIGNED_BYTE, GL_SHORT, GL_UNSIGNED_SHORT, GL_INT, and GL_UNSIGNED_INT are
 *		accepted by both functions. Additionally GL_HALF_FLOAT, GL_FLOAT, GL_DOUBLE, GL_FIXED,
 *		GL_INT_2_10_10_10_REV, and GL_UNSIGNED_INT_2_10_10_10_REV are accepted by
 *		glVertexAttribPointer. The initial value is GL_FLOAT.
 *	@param[in] n_stride_bytes Specifies the byte offset between consecutive generic vertex attributes. If
 *		stride is 0, the generic vertex attributes are understood to be tightly packed in the array.
 *		The initial value is 0.
 *	@param[in] n_offset_bytes Specifies a offset of the first component of the first generic vertex
 *		attribute in the array in the data store of the buffer currently bound to the
 *		GL_ARRAY_BUFFER target. The initial value is 0.
 */
#define GLVertexAttribPtrShort(n_size,n_data_type,n_stride_bytes,n_offset_bytes) \
	(glbuffers_detail::CGLConfigOp_VertexAttribPtr<-1, (n_size), (n_data_type), true, \
		(n_stride_bytes), (n_offset_bytes)>())

/**
 *	@def GLVertexAttribPtrVSpec
 *	@brief makes an instance of an OpenGL run-time variable vertex attrib pointer specifier
 *
 *	@param[in] n_index Specifies the index of the generic vertex attribute to be modified.
 *	@param[in] n_size Specifies the number of components per generic vertex attribute. Must be 1, 2,
 *		3, 4. Additionally, the symbolic constant GL_BGRA is accepted by glVertexAttribPointer. The
 *		initial value is 4. Specifying size 0 disables the attribute, the other arguments are then ignored.
 *	@param[in] n_data_type specifies the data type of each component in the array. The symbolic constants
 *		GL_BYTE, GL_UNSIGNED_BYTE, GL_SHORT, GL_UNSIGNED_SHORT, GL_INT, and GL_UNSIGNED_INT are
 *		accepted by both functions. Additionally GL_HALF_FLOAT, GL_FLOAT, GL_DOUBLE, GL_FIXED,
 *		GL_INT_2_10_10_10_REV, and GL_UNSIGNED_INT_2_10_10_10_REV are accepted by
 *		glVertexAttribPointer. The initial value is GL_FLOAT.
 *	@param[in] b_normalized Specifies whether fixed-point data values
 *		should be normalized (GL_TRUE) or converted directly as fixed-point values (GL_FALSE)
 *		when they are accessed.
 *	@param[in] n_stride_bytes Specifies the byte offset between consecutive generic vertex attributes. If
 *		stride is 0, the generic vertex attributes are understood to be tightly packed in the array.
 *		The initial value is 0.
 *	@param[in] n_offset_bytes Specifies a offset of the first component of the first generic vertex
 *		attribute in the array in the data store of the buffer currently bound to the
 *		GL_ARRAY_BUFFER target. The initial value is 0.
 */
#define GLVertexAttribPtrVSpec(n_index,n_size,n_data_type,b_normalized,n_stride_bytes,n_offset_bytes) \
	(glbuffers_detail::CGLConfigOp_VertexAttribPtr_Var<>((n_index), (n_size), (n_data_type), \
		(b_normalized), (n_stride_bytes), (n_offset_bytes)))

/**
 *	@def GLVertexAttribPtrVDisable
 *	@brief makes an instance of an OpenGL run-time variable vertex attrib pointer specifier
 *
 *	@param[in] n_index Specifies the index of the generic vertex attribute to be disabled.
 */
#define GLVertexAttribPtrVDisable(n_index) \
	(glbuffers_detail::CGLConfigOp_VertexAttribPtr_Var<>((n_index), 0, 0, true, 0, 0))

/**
 *	@brief per-VBO vertex array configuration
 */
struct TGLVBOConfig {
	const uint8_t *m_p_pointer;
	size_t m_n_size;
	GLenum m_n_mode;

	template <class T>
	TGLVBOConfig(const std::vector<T> &r_buffer, size_t n_use_elements = size_t(-1), GLenum n_mode = GL_STATIC_DRAW)
		:m_p_pointer((const uint8_t*)((r_buffer.empty())? 0 : &r_buffer.front())),
		m_n_size(((n_use_elements == size_t(-1))? r_buffer.size() :
		n_use_elements) * sizeof(T)), m_n_mode(n_mode)
	{}

	TGLVBOConfig(const void *p_pointer = 0, size_t n_size = 0, GLenum n_mode = GL_STATIC_DRAW) // need a default constructor
		:m_p_pointer((const uint8_t*)p_pointer), m_n_size(n_size), m_n_mode(n_mode)
	{}

	// an empty function operator for specifying no vertex attributes (use this if you want to specify your attributes later on)
	glbuffers_detail::CGLConfigOp_Buffer<glbuffers_detail::CGLConfigOp_NoConfig> operator ()() const
	{
		typedef glbuffers_detail::CGLConfigOp_NoConfig CNullConfigurer; // "null" configuration
		typedef glbuffers_detail::CGLConfigOp_Buffer<CNullConfigurer> CResult; // caller of this null configuration
		return CResult(*this); // on this buffer
	}

#if LVA_STRICTER_TYPE_CHECK
	// performs more strict type checking; t_vertex_attrib_config_chain must be a specialization of glbuffers_detail::CGLConfigOp_VertexAttribPtr
	template <const int i, const int d, const int t, const bool n, const int s, const int o, class CNextChained>
	glbuffers_detail::CGLConfigOp_Buffer<glbuffers_detail::CGLConfigOp_VertexAttribPtr<i, d, t, n, s, o, CNextChained> >
		operator ()(const glbuffers_detail::CGLConfigOp_VertexAttribPtr<i, d, t, n, s, o, CNextChained> UNUSED(t_vertex_attrib_config_chain)) const
	{
		typedef glbuffers_detail::CGLConfigOp_VertexAttribPtr<i, d, t, n, s, o, CNextChained> CArg;
		typedef glbuffers_detail::CGLConfigOp_Buffer<CArg> CResult;
		return CResult(*this);
	}
#else // LVA_STRICTER_TYPE_CHECK
	template <class CVAConfigChain>
	glbuffers_detail::CGLConfigOp_Buffer<CVAConfigChain> operator ()(const CVAConfigChain UNUSED(t_vertex_attrib_config_chain)) const
	{
		return glbuffers_detail::CGLConfigOp_Buffer<CVAConfigChain>(*this);
	}
#endif // LVA_STRICTER_TYPE_CHECK
	/*template <class CVAConfigChain>
	glbuffers_detail::CGLConfigOp_Buffer<CVAConfigChain> operator ()(const CVAConfigChain UNUSED(t_vertex_attrib_config_chain)) const
	{
		return glbuffers_detail::CGLConfigOp_Buffer<CVAConfigChain>(*this);
	}*/ // flawed, not sure why this is here twice
};

/**
 *	@brief multiple-VBO OpenGL vertex array setup
 *	@tparam _n_max_vbo_num is the maximum number of VBOs
 */
template <const size_t _n_max_vbo_num = 1>
class CGLMultiArraySetup {
protected:
	enum {
		n_max_vbo_num = _n_max_vbo_num
	};

#ifndef __GL_ARRAY_SETUP_DONT_USE_VERTEX_ARRAY_OBJECT
	CGLVertexArrayObject m_vao;
#else // !__GL_ARRAY_SETUP_DONT_USE_VERTEX_ARRAY_OBJECT
	glbuffers_detail::TReconfFunctionPtr m_p_reconf_fun; // used to reconfigure the vertex pointer bindings in case VAO is not available
#endif // !__GL_ARRAY_SETUP_DONT_USE_VERTEX_ARRAY_OBJECT
	GLenum m_n_mode;
	size_t m_n_vertex_num;
	// hot

	CGLArrayBufferObject m_p_vbo[n_max_vbo_num];
	// cold

public:
#if LVA_STRICTER_TYPE_CHECK
	// performs more strict type checking, config must be a specialization of glbuffers_detail::CGLConfigOp_Buffer
	template <class CVAConfigChain, class CNextChained>
	CGLMultiArraySetup(const glbuffers_detail::CGLConfigOp_Buffer<CVAConfigChain,
		CNextChained> &config, GLenum n_mode, size_t n_vertex_num)
#else // LVA_STRICTER_TYPE_CHECK
	template <class CGLBufferConfig>
	CGLMultiArraySetup(const CGLBufferConfig &config, GLenum n_mode, size_t n_vertex_num)
#endif // LVA_STRICTER_TYPE_CHECK
		:m_n_mode(n_mode), m_n_vertex_num(n_vertex_num)
	{
#if LVA_STRICTER_TYPE_CHECK
		typedef glbuffers_detail::CGLConfigOp_Buffer<CVAConfigChain, CNextChained> CGLBufferConfig;
#endif // LVA_STRICTER_TYPE_CHECK
		enum {
			b_enough_buffers = int(CGLBufferConfig::n_buffer_num) <= int(n_max_vbo_num)
		};
		typedef typename glbuffers_detail::CStaticAssert<b_enough_buffers>::NOT_ENOUGH_VERTEX_BUFFERS_FOR_THIS_CONFIGURATION CAssert0;
		// make sure that there is enough buffers for this particular configuration

#ifndef __GL_ARRAY_SETUP_DONT_USE_VERTEX_ARRAY_OBJECT
		m_vao.Bind();
#endif // !__GL_ARRAY_SETUP_DONT_USE_VERTEX_ARRAY_OBJECT
		{
			size_t n_max_vertex_num = config.Run(n_max_vbo_num, m_p_vbo);
			// fill the VBOs

			_ASSERTE(n_vertex_num <= n_max_vertex_num);
			// make sure we will not do an out of bounds access
			// note that this does not take offsets into account, and an out of bounds access can still
			// in fact occur. would have to pass the number of vertices upwards in order to really check
		}
#ifndef __GL_ARRAY_SETUP_DONT_USE_VERTEX_ARRAY_OBJECT
		m_vao.Release();
#else // !__GL_ARRAY_SETUP_DONT_USE_VERTEX_ARRAY_OBJECT
		m_p_reconf_fun = &config.Run_Reconf;
		// grab the address of the function that reconfigures the vertex pointer bindings (in case VAO is not available)
#endif // !__GL_ARRAY_SETUP_DONT_USE_VERTEX_ARRAY_OBJECT
	}

	/**
	 *	@brief gets status of this renderable
	 *	@return Returns true if the array buffer and VAO allocated successfully,
	 *		otherwise returns false.
	 */
	bool b_Status() const
	{
		for(int i = 0; i < n_max_vbo_num; ++ i) {
			if(!m_p_vbo[i].b_Status())
				return false;
		}
#ifndef __GL_ARRAY_SETUP_DONT_USE_VERTEX_ARRAY_OBJECT
		return m_vao.b_Status();
#else // !__GL_ARRAY_SETUP_DONT_USE_VERTEX_ARRAY_OBJECT
		return true;
#endif // !__GL_ARRAY_SETUP_DONT_USE_VERTEX_ARRAY_OBJECT
	}

	/**
	 *	@brief binds the VAO and calls glDrawArrays() to display the geometry
	 */
	inline void Draw() const
	{
/*#ifndef __GL_ARRAY_SETUP_DONT_USE_VERTEX_ARRAY_OBJECT
		m_vao.Bind();
#else // !__GL_ARRAY_SETUP_DONT_USE_VERTEX_ARRAY_OBJECT
		(*m_p_reconf_fun)(n_max_vbo_num, m_p_vbo);
		// reconfigure the vertex pointer bindings
#endif // !__GL_ARRAY_SETUP_DONT_USE_VERTEX_ARRAY_OBJECT
		{
			glDrawArrays(m_n_mode, 0, m_n_vertex_num);
		}
#ifndef __GL_ARRAY_SETUP_DONT_USE_VERTEX_ARRAY_OBJECT
#ifdef __GL_VERTEX_ARRAY_OBJECT_RELEASE_AFTER_DRAWING
		m_vao.Release();
#endif // __GL_VERTEX_ARRAY_OBJECT_RELEASE_AFTER_DRAWING
#endif // !__GL_ARRAY_SETUP_DONT_USE_VERTEX_ARRAY_OBJECT*/
		Draw(m_n_mode, 0, m_n_vertex_num);
	}

	/**
	 *	@brief binds the VAO and calls glDrawArrays() to display the geometry,
	 *		adds override option for some of the parameters
	 *
	 *	@param[in] n_draw_mode Specifies what kind of primitives to render. Symbolic constants GL_POINTS,
	 *		GL_LINE_STRIP, GL_LINE_LOOP, GL_LINES, GL_LINE_STRIP_ADJACENCY, GL_LINES_ADJACENCY,
	 *		GL_TRIANGLE_STRIP, GL_TRIANGLE_FAN, GL_TRIANGLES, GL_TRIANGLE_STRIP_ADJACENCY,
	 *		GL_TRIANGLES_ADJACENCY and GL_PATCHES are accepted.
	 *	@param[in] n_first_vertex Specifies the starting index in the enabled arrays.
	 *	@param[in] n_vertex_num Specifies the number of indices to be rendered.
	 */
	inline void Draw(GLenum n_draw_mode, size_t n_first_vertex, size_t n_vertex_num) const
	{
#ifndef __GL_ARRAY_SETUP_DONT_USE_VERTEX_ARRAY_OBJECT
		m_vao.Bind();
#else // !__GL_ARRAY_SETUP_DONT_USE_VERTEX_ARRAY_OBJECT
		(*m_p_reconf_fun)(n_max_vbo_num, m_p_vbo);
		// reconfigure the vertex pointer bindings
#endif // !__GL_ARRAY_SETUP_DONT_USE_VERTEX_ARRAY_OBJECT
		{
			_ASSERTE(n_first_vertex <= INT_MAX && n_vertex_num <= INT_MAX);
			glDrawArrays(n_draw_mode, int(n_first_vertex), GLsizei(n_vertex_num));
		}
#ifndef __GL_ARRAY_SETUP_DONT_USE_VERTEX_ARRAY_OBJECT
#ifdef __GL_VERTEX_ARRAY_OBJECT_RELEASE_AFTER_DRAWING
		m_vao.Release();
#endif // __GL_VERTEX_ARRAY_OBJECT_RELEASE_AFTER_DRAWING
#endif // !__GL_ARRAY_SETUP_DONT_USE_VERTEX_ARRAY_OBJECT
	}

	/**
	 *	@brief binds the VAO and calls glDrawArrays() to display the geometry,
	 *		adds override option for some of the parameters
	 *
	 *	@param[in] r_var_reconf Specifies reconfiguration of the vertex attribute pointers.
	 *	@param[in] b_remember_new_config Specifies whether the vertex attribute configuration is to be
	 *		stored to the VAO in this object, or to a temporary VAO only for this render (has no effect
	 *		if VAOs are disabled).
	 *
	 *	@note Note that this does not offer any extra safety, compared to calling glVertexAttribPointer()
	 *		directly but it is not less safe either.
	 */
#if LVA_STRICTER_TYPE_CHECK
	// performs more strict type checking, r_var_reconf must be a specialization of glbuffers_detail::CGLConfigOp_VertexAttribPtr_Var
	template <class CNext, const int n_advance>
	inline bool Draw_Attribs(const glbuffers_detail::CGLConfigOp_VertexAttribPtr_Var<CNext, n_advance>
#else // LVA_STRICTER_TYPE_CHECK
	template <class CVariableVertexAttribsConfig>
	inline bool Draw_Attribs(const CVariableVertexAttribsConfig
#endif // LVA_STRICTER_TYPE_CHECK
		&r_var_reconf, bool b_remember_new_config) const
	{
		/*if(b_remember_new_config) {
#ifndef __GL_ARRAY_SETUP_DONT_USE_VERTEX_ARRAY_OBJECT
			m_vao.Bind();
#else // !__GL_ARRAY_SETUP_DONT_USE_VERTEX_ARRAY_OBJECT
			(*m_p_reconf_fun)(n_max_vbo_num, m_p_vbo);
			// reconfigure the vertex pointer bindings
#endif // !__GL_ARRAY_SETUP_DONT_USE_VERTEX_ARRAY_OBJECT

			r_var_reconf.Run(n_max_vbo_num, m_p_vbo);
			// reconfigure

			glDrawArrays(m_n_mode, 0, m_n_vertex_num);
			// draw

#ifndef __GL_ARRAY_SETUP_DONT_USE_VERTEX_ARRAY_OBJECT
#ifdef __GL_VERTEX_ARRAY_OBJECT_RELEASE_AFTER_DRAWING
			m_vao.Release();
#endif // __GL_VERTEX_ARRAY_OBJECT_RELEASE_AFTER_DRAWING
#endif // !__GL_ARRAY_SETUP_DONT_USE_VERTEX_ARRAY_OBJECT
		} else {
#ifndef __GL_ARRAY_SETUP_DONT_USE_VERTEX_ARRAY_OBJECT
			CGLVertexArrayObject vao;
			if(!vao.b_Status())
				return false;
			vao.Bind();
#else // !__GL_ARRAY_SETUP_DONT_USE_VERTEX_ARRAY_OBJECT
			(*m_p_reconf_fun)(n_max_vbo_num, m_p_vbo);
			// reconfigure the vertex pointer bindings
#endif // !__GL_ARRAY_SETUP_DONT_USE_VERTEX_ARRAY_OBJECT

			r_var_reconf.Run(n_max_vbo_num, m_p_vbo);
			// reconfigure

			glDrawArrays(m_n_mode, 0, m_n_vertex_num);
			// draw

#ifndef __GL_ARRAY_SETUP_DONT_USE_VERTEX_ARRAY_OBJECT
#ifdef __GL_VERTEX_ARRAY_OBJECT_RELEASE_AFTER_DRAWING
			vao.Release(); // if not released, will likely remain allocated in GL until some other is bound
#endif // __GL_VERTEX_ARRAY_OBJECT_RELEASE_AFTER_DRAWING
#endif // !__GL_ARRAY_SETUP_DONT_USE_VERTEX_ARRAY_OBJECT
		}
		return true;*/
		return Draw_Attribs(m_n_mode, 0, m_n_vertex_num, r_var_reconf, b_remember_new_config);
	}

	/**
	 *	@brief binds the VAO and calls glDrawArrays() to display the geometry,
	 *		adds override option for some of the parameters
	 *
	 *	@param[in] n_draw_mode Specifies what kind of primitives to render. Symbolic constants GL_POINTS,
	 *		GL_LINE_STRIP, GL_LINE_LOOP, GL_LINES, GL_LINE_STRIP_ADJACENCY, GL_LINES_ADJACENCY,
	 *		GL_TRIANGLE_STRIP, GL_TRIANGLE_FAN, GL_TRIANGLES, GL_TRIANGLE_STRIP_ADJACENCY,
	 *		GL_TRIANGLES_ADJACENCY and GL_PATCHES are accepted.
	 *	@param[in] n_first_vertex Specifies the starting index in the enabled arrays.
	 *	@param[in] n_vertex_num Specifies the number of indices to be rendered.
	 *	@param[in] r_var_reconf Specifies reconfiguration of the vertex attribute pointers.
	 *	@param[in] b_remember_new_config Specifies whether the vertex attribute configuration is to be
	 *		stored to the VAO in this object, or to a temporary VAO only for this render (has no effect
	 *		if VAOs are disabled).
	 *
	 *	@note Note that this does not offer any extra safety, compared to calling glVertexAttribPointer()
	 *		directly but it is not less safe either.
	 */
#if LVA_STRICTER_TYPE_CHECK
	// performs more strict type checking, r_var_reconf must be a specialization of glbuffers_detail::CGLConfigOp_VertexAttribPtr_Var
	template <class CNext, const int n_advance>
	inline bool Draw_Attribs(GLenum n_draw_mode, size_t n_first_vertex, size_t n_vertex_num,
		const glbuffers_detail::CGLConfigOp_VertexAttribPtr_Var<CNext, n_advance>
#else // LVA_STRICTER_TYPE_CHECK
	template <class CVariableVertexAttribsConfig>
	inline bool Draw_Attribs(GLenum n_draw_mode, size_t n_first_vertex, size_t n_vertex_num,
		const CVariableVertexAttribsConfig
#endif // LVA_STRICTER_TYPE_CHECK
		&r_var_reconf, bool b_remember_new_config) const
	{
		if(b_remember_new_config) {
#ifndef __GL_ARRAY_SETUP_DONT_USE_VERTEX_ARRAY_OBJECT
			m_vao.Bind();
#else // !__GL_ARRAY_SETUP_DONT_USE_VERTEX_ARRAY_OBJECT
			(*m_p_reconf_fun)(n_max_vbo_num, m_p_vbo);
			// reconfigure the vertex pointer bindings
#endif // !__GL_ARRAY_SETUP_DONT_USE_VERTEX_ARRAY_OBJECT

			r_var_reconf.Run(n_max_vbo_num, m_p_vbo);
			// reconfigure

			_ASSERTE(n_first_vertex <= INT_MAX && n_vertex_num <= INT_MAX);
			glDrawArrays(n_draw_mode, int(n_first_vertex), GLsizei(n_vertex_num));
			// draw

#ifndef __GL_ARRAY_SETUP_DONT_USE_VERTEX_ARRAY_OBJECT
#ifdef __GL_VERTEX_ARRAY_OBJECT_RELEASE_AFTER_DRAWING
			m_vao.Release();
#endif // __GL_VERTEX_ARRAY_OBJECT_RELEASE_AFTER_DRAWING
#endif // !__GL_ARRAY_SETUP_DONT_USE_VERTEX_ARRAY_OBJECT
		} else {
#ifndef __GL_ARRAY_SETUP_DONT_USE_VERTEX_ARRAY_OBJECT
			CGLVertexArrayObject vao;
			if(!vao.b_Status())
				return false;
			vao.Bind();
#else // !__GL_ARRAY_SETUP_DONT_USE_VERTEX_ARRAY_OBJECT
			(*m_p_reconf_fun)(n_max_vbo_num, m_p_vbo);
			// reconfigure the vertex pointer bindings
#endif // !__GL_ARRAY_SETUP_DONT_USE_VERTEX_ARRAY_OBJECT

			r_var_reconf.Run(n_max_vbo_num, m_p_vbo);
			// reconfigure

			_ASSERTE(n_first_vertex <= INT_MAX && n_vertex_num <= INT_MAX);
			glDrawArrays(n_draw_mode, int(n_first_vertex), GLsizei(n_vertex_num));
			// draw

#ifndef __GL_ARRAY_SETUP_DONT_USE_VERTEX_ARRAY_OBJECT
#ifdef __GL_VERTEX_ARRAY_OBJECT_RELEASE_AFTER_DRAWING
			vao.Release(); // if not released, will likely remain allocated in GL until some other is bound
#endif // __GL_VERTEX_ARRAY_OBJECT_RELEASE_AFTER_DRAWING
#endif // !__GL_ARRAY_SETUP_DONT_USE_VERTEX_ARRAY_OBJECT
		}
		return true;
	}

protected:
	/**
	 *	@brief default constructor; has no effect
	 *	@note This constructor must be overloaded so the buffers are filled with data.
	 */
	CGLMultiArraySetup()
	{}

	// --- design considerations ---

	// could setup all atomically in the ctor
	//		- using a template params - not sure about awkwardness of the syntax
	//			- if VAOs are not supported, dont have to store anything at all, just a pointer to the function to set the arrays ;)
	//		- using realtime params - probably ok, anyway passed to the driver as realtime
	//			- if VAOs are not supported, will have to store the params in an array of MAX_PARAMS size (should not happen in practice though, maybe on old android devices)

	// could add an extra function afterwards to setup more stuff <- no.
	//		- disadvantage of not being able to easily have vertices in more than a single buffer
	//		- disadvantage of the user changing the data in a single buffer between the calls vs. inability to support interleaved buffers
};


/**
 *	@brief multiple-VBO OpenGL vertex element array setup
 *	@tparam _n_max_vbo_num is the maximum number of VBOs
 */
template <const size_t _n_max_vbo_num = 1>
class CGLMultiElementArraySetup : protected CGLMultiArraySetup<_n_max_vbo_num> {
protected:
	enum {
		n_max_vbo_num = _n_max_vbo_num
	};

	GLenum m_n_index_data_type; /**< @brief data type of vertex indices */
	// hot

	CGLElementArrayBufferObject m_ibo;
	// cold

public:
#if LVA_STRICTER_TYPE_CHECK
	// performs more strict type checking, config must be a specialization of glbuffers_detail::CGLConfigOp_Buffer
	template <class CVAConfigChain, class CNextChained>
	CGLMultiElementArraySetup(const glbuffers_detail::CGLConfigOp_Buffer<CVAConfigChain,
		CNextChained> &config, GLenum n_mode, const TGLVBOConfig &r_t_index_buffer,
		size_t n_index_num, GLenum n_index_data_type)
#else // LVA_STRICTER_TYPE_CHECK
	template <class CGLBufferConfig>
	CGLMultiElementArraySetup(const CGLBufferConfig &config, GLenum n_mode,
		const TGLVBOConfig &r_t_index_buffer, size_t n_index_num, GLenum n_index_data_type)
#endif // LVA_STRICTER_TYPE_CHECK
		:CGLMultiArraySetup<_n_max_vbo_num>(config, n_mode, 0), m_n_index_data_type(n_index_data_type) // set vertex num to zero
	{
		this->m_n_vertex_num = n_index_num;
		// reuse this for index num, but dont pass it through the constructor as it checks
		// if the number of vertices fits in the vertex buffers, and here it is completely
		// unrelated - it depends on the index values which we're not going to check

#ifndef __GL_ARRAY_SETUP_DONT_USE_VERTEX_ARRAY_OBJECT
		this->m_vao.Bind();
#endif // !__GL_ARRAY_SETUP_DONT_USE_VERTEX_ARRAY_OBJECT
		{
			m_ibo.Bind();
			_ASSERTE(r_t_index_buffer.m_n_size <= UINT_MAX);
			m_ibo.BufferData(GLuint(r_t_index_buffer.m_n_size),
				r_t_index_buffer.m_p_pointer, r_t_index_buffer.m_n_mode);
			// load the data to the index buffer

			size_t n_required_size = n_GL_TypeSize(n_index_data_type) * n_index_num;
			_ASSERTE(r_t_index_buffer.m_n_size >= n_required_size);
			// make sure there is enough data in the index buffer
		}
#ifndef __GL_ARRAY_SETUP_DONT_USE_VERTEX_ARRAY_OBJECT
		this->m_vao.Release();
#endif // !__GL_ARRAY_SETUP_DONT_USE_VERTEX_ARRAY_OBJECT
	}

	/**
	 *	@brief gets status of this renderable
	 *	@return Returns true if the array buffer and VAO allocated successfully,
	 *		otherwise returns false.
	 */
	bool b_Status() const
	{
		return m_ibo.b_Status() && CGLMultiArraySetup<_n_max_vbo_num>::b_Status();
	}

	/**
	 *	@brief binds the VAO and calls glDrawElements() to display the geometry
	 */
	inline void Draw() const
	{
		Draw(this->m_n_mode, this->m_n_vertex_num);
	}

	/**
	 *	@brief binds the VAO and calls glDrawElements() to display the geometry,
	 *		adds override option for some of the parameters
	 *
	 *	@param[in] n_draw_mode Specifies what kind of primitives to render. Symbolic constants GL_POINTS,
	 *		GL_LINE_STRIP, GL_LINE_LOOP, GL_LINES, GL_LINE_STRIP_ADJACENCY, GL_LINES_ADJACENCY,
	 *		GL_TRIANGLE_STRIP, GL_TRIANGLE_FAN, GL_TRIANGLES, GL_TRIANGLE_STRIP_ADJACENCY,
	 *		GL_TRIANGLES_ADJACENCY and GL_PATCHES are accepted.
	 *	@param[in] n_index_num Specifies the number of elements to be rendered.
	 *	@param[in] n_first_index_byte_offset Specifies a pointer to the location where the indices are stored.
	 */
	inline void Draw(GLenum n_draw_mode, size_t n_index_num, size_t n_first_index_byte_offset = 0) const
	{
#ifndef __GL_ARRAY_SETUP_DONT_USE_VERTEX_ARRAY_OBJECT
		this->m_vao.Bind();
#else // !__GL_ARRAY_SETUP_DONT_USE_VERTEX_ARRAY_OBJECT
		(*this->m_p_reconf_fun)(n_max_vbo_num, this->m_p_vbo);
		// reconfigure the vertex pointer bindings
#endif // !__GL_ARRAY_SETUP_DONT_USE_VERTEX_ARRAY_OBJECT
		{
#ifdef __GL_VERTEX_ARRAY_OBJECT_REBIND_INDEX_BUFFER
			m_ibo.Bind(); // handle defunct VAO on mobile chipsets
#endif // __GL_VERTEX_ARRAY_OBJECT_REBIND_INDEX_BUFFER

			_ASSERTE(n_index_num <= INT_MAX);
			glDrawElements(n_draw_mode, GLsizei(n_index_num),
				m_n_index_data_type, (const void*)n_first_index_byte_offset);
		}
#ifndef __GL_ARRAY_SETUP_DONT_USE_VERTEX_ARRAY_OBJECT
#ifdef __GL_VERTEX_ARRAY_OBJECT_RELEASE_AFTER_DRAWING
		this->m_vao.Release();
#endif // __GL_VERTEX_ARRAY_OBJECT_RELEASE_AFTER_DRAWING
#endif // !__GL_ARRAY_SETUP_DONT_USE_VERTEX_ARRAY_OBJECT
	}

	/**
	 *	@brief binds the VAO and calls glDrawElements() to display the geometry,
	 *		adds override option for some of the parameters
	 *
	 *	@param[in] r_var_reconf Specifies reconfiguration of the vertex attribute pointers.
	 *	@param[in] b_remember_new_config Specifies whether the vertex attribute configuration is to be
	 *		stored to the VAO in this object, or to a temporary VAO only for this render (has no effect
	 *		if VAOs are disabled).
	 *
	 *	@note Note that this does not offer any extra safety, compared to calling glVertexAttribPointer()
	 *		directly but it is not less safe either.
	 */
#if LVA_STRICTER_TYPE_CHECK
	// performs more strict type checking, r_var_reconf must be a specialization of glbuffers_detail::CGLConfigOp_VertexAttribPtr_Var
	template <class CNext, const int n_advance>
	inline bool Draw_Attribs(const glbuffers_detail::CGLConfigOp_VertexAttribPtr_Var<CNext, n_advance>
#else // LVA_STRICTER_TYPE_CHECK
	template <class CVariableVertexAttribsConfig>
	inline bool Draw_Attribs(const CVariableVertexAttribsConfig
#endif // LVA_STRICTER_TYPE_CHECK
		&r_var_reconf, bool b_remember_new_config) const
	{
		return Draw_Attribs(this->m_n_mode, this->m_n_vertex_num, 0, r_var_reconf, b_remember_new_config);
	}

	/**
	 *	@brief binds the VAO and calls glDrawElements() to display the geometry,
	 *		adds override option for some of the parameters
	 *
	 *	@param[in] n_draw_mode Specifies what kind of primitives to render. Symbolic constants GL_POINTS,
	 *		GL_LINE_STRIP, GL_LINE_LOOP, GL_LINES, GL_LINE_STRIP_ADJACENCY, GL_LINES_ADJACENCY,
	 *		GL_TRIANGLE_STRIP, GL_TRIANGLE_FAN, GL_TRIANGLES, GL_TRIANGLE_STRIP_ADJACENCY,
	 *		GL_TRIANGLES_ADJACENCY and GL_PATCHES are accepted.
	 *	@param[in] n_index_num Specifies the number of elements to be rendered.
	 *	@param[in] n_first_index_byte_offset Specifies a pointer to the location where the indices are stored.
	 *	@param[in] r_var_reconf Specifies reconfiguration of the vertex attribute pointers.
	 *	@param[in] b_remember_new_config Specifies whether the vertex attribute configuration is to be
	 *		stored to the VAO in this object, or to a temporary VAO only for this render (has no effect
	 *		if VAOs are disabled).
	 *
	 *	@note Note that this does not offer any extra safety, compared to calling glVertexAttribPointer()
	 *		directly but it is not less safe either.
	 */
#if LVA_STRICTER_TYPE_CHECK
	// performs more strict type checking, r_var_reconf must be a specialization of glbuffers_detail::CGLConfigOp_VertexAttribPtr_Var
	template <class CNext, const int n_advance>
	inline bool Draw_Attribs(GLenum n_draw_mode, size_t n_index_num, size_t n_first_index_byte_offset,
		const glbuffers_detail::CGLConfigOp_VertexAttribPtr_Var<CNext, n_advance>
#else // LVA_STRICTER_TYPE_CHECK
	template <class CVariableVertexAttribsConfig>
	inline bool Draw_Attribs(GLenum n_draw_mode, size_t n_index_num, size_t n_first_index_byte_offset,
		const CVariableVertexAttribsConfig
#endif // LVA_STRICTER_TYPE_CHECK
		&r_var_reconf, bool b_remember_new_config) const
	{
		if(b_remember_new_config) {
#ifndef __GL_ARRAY_SETUP_DONT_USE_VERTEX_ARRAY_OBJECT
			this->m_vao.Bind();
#else // !__GL_ARRAY_SETUP_DONT_USE_VERTEX_ARRAY_OBJECT
			(*this->m_p_reconf_fun)(n_max_vbo_num, this->m_p_vbo);
			// reconfigure the vertex pointer bindings
#endif // !__GL_ARRAY_SETUP_DONT_USE_VERTEX_ARRAY_OBJECT
#ifdef __GL_VERTEX_ARRAY_OBJECT_REBIND_INDEX_BUFFER
			m_ibo.Bind(); // handle defunct VAO on mobile chipsets
#endif // __GL_VERTEX_ARRAY_OBJECT_REBIND_INDEX_BUFFER

			r_var_reconf.Run(n_max_vbo_num, this->m_p_vbo);
			// reconfigure

			_ASSERTE(n_index_num <= INT_MAX);
			glDrawElements(n_draw_mode, GLsizei(n_index_num),
				m_n_index_data_type, (const void*)n_first_index_byte_offset);
			// draw

#ifndef __GL_ARRAY_SETUP_DONT_USE_VERTEX_ARRAY_OBJECT
#ifdef __GL_VERTEX_ARRAY_OBJECT_RELEASE_AFTER_DRAWING
			this->m_vao.Release();
#endif // __GL_VERTEX_ARRAY_OBJECT_RELEASE_AFTER_DRAWING
#endif // !__GL_ARRAY_SETUP_DONT_USE_VERTEX_ARRAY_OBJECT
		} else {
#ifndef __GL_ARRAY_SETUP_DONT_USE_VERTEX_ARRAY_OBJECT
			CGLVertexArrayObject vao;
			if(!vao.b_Status())
				return false;
			vao.Bind();
#else // !__GL_ARRAY_SETUP_DONT_USE_VERTEX_ARRAY_OBJECT
			(*this->m_p_reconf_fun)(n_max_vbo_num, this->m_p_vbo);
			// reconfigure the vertex pointer bindings
#endif // !__GL_ARRAY_SETUP_DONT_USE_VERTEX_ARRAY_OBJECT
#ifdef __GL_VERTEX_ARRAY_OBJECT_REBIND_INDEX_BUFFER
			m_ibo.Bind(); // handle defunct VAO on mobile chipsets
#endif // __GL_VERTEX_ARRAY_OBJECT_REBIND_INDEX_BUFFER

			r_var_reconf.Run(n_max_vbo_num, this->m_p_vbo);
			// reconfigure

			_ASSERTE(n_index_num <= INT_MAX);
			glDrawElements(n_draw_mode, GLsizei(n_index_num),
				m_n_index_data_type, (const void*)n_first_index_byte_offset);
			// draw

#ifndef __GL_ARRAY_SETUP_DONT_USE_VERTEX_ARRAY_OBJECT
#ifdef __GL_VERTEX_ARRAY_OBJECT_RELEASE_AFTER_DRAWING
			vao.Release(); // if not released, will likely remain allocated in GL until some other is bound
#endif // __GL_VERTEX_ARRAY_OBJECT_RELEASE_AFTER_DRAWING
#endif // !__GL_ARRAY_SETUP_DONT_USE_VERTEX_ARRAY_OBJECT
		}
		return true;
	}

protected:
	/**
	 *	@brief default constructor; has no effect
	 *	@note This constructor must be overloaded so the buffers are filled with data.
	 */
	CGLMultiElementArraySetup()
	{}
};

#endif // !__GL_MULTIPLE_VBO_VERTEX_ARRAYS_INCLUDED
