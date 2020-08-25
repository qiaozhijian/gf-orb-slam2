/*
								+---------------------------------+
								|                                 |
								| *** OpenGL multi-VBO arrays *** |
								|                                 |
								|  Copyright  © -tHE SWINe- 2015  |
								|                                 |
								|        VertexArrays.inl         |
								|                                 |
								+---------------------------------+
*/

#pragma once
#ifndef __GL_MULTIPLE_VBO_VERTEX_ARRAYS_INLINES_INCLUDED
#define __GL_MULTIPLE_VBO_VERTEX_ARRAYS_INLINES_INCLUDED

/**
 *	@file VertexArrays.inl
 *	@brief OpenGL reconfigurable vertex arrays
 *	@author -tHE SWINe-
 *	@date 2015
 *
 *	This file is meant to be included from VertexArrays.h and not otherwise.
 */

struct TGLVBOConfig; // forward declaration

/**
 *	@brief OpenGL vertex array setup helper objects
 */
namespace glbuffers_detail {

template <bool b_expression>
class CStaticAssert {
public:
	typedef void NOT_ENOUGH_VERTEX_BUFFERS_FOR_THIS_CONFIGURATION; /**< @brief static assertion tag */
	typedef void BUFFER_CONFIGURATION_MUST_BE_A_LINEAR_CHAIN; /**< @brief static assertion tag */
};

template <>
class CStaticAssert<false> {};

/*
 *	in here, there is a hierarchy of objects:
 *
 *	CGLConfigOp_Buffer - contains VBO configuration ::TGLVBOConfig and the associated attribute configuration chain
 *					   - implements Run() that uploads the data and sets the vertex attribs, returns max number of vertices, given the sizes of buffers and strides of attributes
 *					   - implements Run_Reconf() that just binds the buffer and sets the vertex attribs (used where VAOs are not available)
 *					   - can be chained operator ,() to have more VBOs in a single array setup (CGLConfigOp_Buffer_ChainEnd is the recursion terminator)
 *
 *		::TGLVBOConfig - contains VBO data and is used for uploading the data. its operator () specifies the attribute configuration chain
 *
 *		CGLConfigOp_NoConfig - a dummy object that sets no vertex attrib configuration so that it could be set later when drawing
 *							 - implements Run() to be compatible with CGLConfigOp_VertexAttribPtr
 *
 *		CGLConfigOp_VertexAttribPtr - contains compile-time vertex attrib configuration
 *									- implements Run() that sets the vertex attrib configuration and returns max stride of all the configurations
 *									- can be chained using operator ,() to have more attributed referring to a single VBO (CGLConfigOp_VertexAttribPtr_ChainEnd is the recursion terminator)
 *
 *	CGLConfigOp_VertexAttribPtr_Var - is on a different level than CGLConfigOp_VertexAttribPtr !!! the names are deceiving
 *									- contains variable vertex attrib configuration
 *									- implements *a different* Run() that takes care of binding the buffers and sets the vertex attrib configuration and returns max stride of all the configurations
 *									- can be chained using operator ,() to have more attributed referring to a single VBO (CGLConfigOp_VertexAttribPtr_Var_ChainEnd is the recursion terminator)
 *									- impements operator ++() to shift to the next VBO
 *
 *		TGLVertexAttribConfig_Var - contains the variable argument settings data (id, stride, dimension, ...)
 *								  - implements Run() that sets this data, comparable to that of CGLConfigOp_VertexAttribPtr
 *
 *	todo - rename CGLConfigOp_VertexAttribPtr_Var to CGLConfigOp_BindBuffers (and probably CGLConfigOp_Buffer to CGLConfigOp_BufferUpload)
 *	todo - implement constructor and operator ,() in CGLConfigOp_BindBuffers that takes CGLConfigOp_VertexAttribPtr so that the buffers can be disabled using compile-time constant data
 *
 */

struct CGLConfigOp_Buffer_ChainEnd {
	enum {
		n_buffer_num = 0
	};

	size_t Run(size_t UNUSED(n_max_buffer_num), CGLArrayBufferObject *UNUSED(p_buffer)) const
	{
		return 0;
	}

	static void Run_Reconf(size_t UNUSED(n_max_buffer_num), const CGLArrayBufferObject *UNUSED(p_buffer))
	{}
};

template <class TGLVAConfigChain, class TNextChained_GLConfigOp_Buffer = CGLConfigOp_Buffer_ChainEnd, class TGLVBOConfig_Forward = ::TGLVBOConfig>
class CGLConfigOp_Buffer {
public:
	typedef TNextChained_GLConfigOp_Buffer CNextChained_GLConfigOp_Buffer;
	typedef TGLVAConfigChain CVAConfigChain;
	typedef CGLConfigOp_Buffer<TGLVAConfigChain, TNextChained_GLConfigOp_Buffer> CSelf;

	enum {
		n_buffer_num = CNextChained_GLConfigOp_Buffer::n_buffer_num + 1
	};

protected:
	TGLVBOConfig_Forward m_t_buffer_config;
	CNextChained_GLConfigOp_Buffer m_next_config;

public:
	CGLConfigOp_Buffer(TGLVBOConfig_Forward t_buffer_config = TGLVBOConfig_Forward(),
		CNextChained_GLConfigOp_Buffer t_next_config = CNextChained_GLConfigOp_Buffer())
		:m_t_buffer_config(t_buffer_config), m_next_config(t_next_config)
	{}

#if LVA_STRICTER_TYPE_CHECK
	// performs more strict type checking, t_next_config_op must be a specialization of CGLConfigOp_Buffer
	template <class CVAConfig, class CNextChained>
	CGLConfigOp_Buffer<CVAConfig, CSelf> operator ,(CGLConfigOp_Buffer<CVAConfig, CNextChained> t_next_config_op)
#else // LVA_STRICTER_TYPE_CHECK
	template <class CNextChainOp>
	CGLConfigOp_Buffer<typename CNextChainOp::CVAConfigChain, CSelf> operator ,(CNextChainOp t_next_config_op)
#endif // LVA_STRICTER_TYPE_CHECK
	{
#if LVA_STRICTER_TYPE_CHECK
		typedef CGLConfigOp_Buffer<CVAConfig, CNextChained> CNextChainOp;
#endif // LVA_STRICTER_TYPE_CHECK
		//_ASSERTE(!CNextChained_GLConfigOp_Buffer::n_buffer_num);
		typedef typename CStaticAssert<CNextChainOp::n_buffer_num == 1>::BUFFER_CONFIGURATION_MUST_BE_A_LINEAR_CHAIN CAssert0;
		// if this triggers then there is something typedef-ed or in parentheses,
		// e.g. (TGLVBOConfig(params)(VAs), (TGLVBOConfig(params)(VAs), TGLVBOConfig(params)(VAs)))
		//                                  ^                                                    ^
		// we assume the comma to be left associative.
		// make sure that CNextChainOp::TNextChained_GLConfigOp_Buffer is CGLConfigOp_Buffer_ChainEnd

		return CGLConfigOp_Buffer<typename CNextChainOp::CVAConfigChain,
			CSelf>(t_next_config_op.t_Buffer_Config(), *this);
	}

	TGLVBOConfig_Forward t_Buffer_Config() const
	{
		return m_t_buffer_config;
	}

	size_t Run(size_t n_max_buffer_num, CGLArrayBufferObject *p_buffer) const
	{
		_ASSERTE(n_max_buffer_num >= n_buffer_num); // t_odo - make this a static assert

		size_t n_next_vertex_num = m_next_config.Run(n_max_buffer_num - 1, p_buffer); // do not change the pointer
		// fill the following buffers

		//printf("debug: config buffer %d\n", n_buffer_num - 1); // debug

		_ASSERTE(n_buffer_num > 0); // make sure that the next line does not overflow
		p_buffer += n_buffer_num - 1; // the classes are stacked in reverse as the comma operator is left to right associative
		p_buffer->Bind();
		_ASSERTE(m_t_buffer_config.m_n_size <= UINT_MAX);
		p_buffer->BufferData(GLuint(m_t_buffer_config.m_n_size),
			m_t_buffer_config.m_p_pointer, m_t_buffer_config.m_n_mode);
		// fill the buffer with data

		const int n_vertex_size = CVAConfigChain::Run(); // calculate the vertex size at the same time
		// configure vertex attrib pointers for this buffer

		const size_t n_vertex_num = (n_vertex_size)? m_t_buffer_config.m_n_size / n_vertex_size :
			m_t_buffer_config.m_n_size;
		// calculate the number of vertices accessible from this buffer

		return (CNextChained_GLConfigOp_Buffer::n_buffer_num)? // if this is not the last buffer config in the chain
			min(n_vertex_num, n_next_vertex_num) : n_vertex_num;
		// calculate the number of vertices accessible from all the buffers
	}

	/**
	 *	@brief runs reconfiguration upon the existing buffers
	 *
	 *	@param[in] n_max_buffer_num is the number of the buffers
	 *	@param[in] p_buffer is pointer to the first buffer
	 *
	 *	@note This is used in case VAO is not used and it is neccessary to reconfigure
	 *		the buffers every time before rendering.
	 */
	static void Run_Reconf(size_t n_max_buffer_num, const CGLArrayBufferObject *p_buffer)
	{
		_ASSERTE(n_max_buffer_num >= n_buffer_num);
		CNextChained_GLConfigOp_Buffer::Run_Reconf(n_max_buffer_num - 1, p_buffer); // do not change the pointer
		// reconfig the following buffers

		_ASSERTE(n_buffer_num > 0); // make sure that the next line does not overflow
		p_buffer += n_buffer_num - 1; // the classes are stacked in reverse as the comma operator is left to right associative
		p_buffer->Bind();
		// bind the buffer

		CVAConfigChain::Run();
		// configure vertex attrib pointers for this buffer
	}
};

typedef void (*TReconfFunctionPtr)(size_t n_max_buffer_num, const CGLArrayBufferObject *p_buffer);

class CGLConfigOp_VertexAttribPtr_ChainEnd {
public:
	enum {
		n_index = -1
	};

	static int Run() // no effect
	{
		return 0;
	}
};

// note that this is quite nice in the sense that the config is constant,
// but some applications may still need variable config, e.g. variable
// offset, maybe also variable stride or data type or all but index.

// no configuration for vertex attributes (to be supplied later)
class CGLConfigOp_NoConfig {
public:
	/**
	 *	@brief sets vertex array config
	 *	@return Returns vertex stride / size, in bytes.
	 */
	static inline int Run()
	{
		return 0;
	}
};

template <const int _n_index, const int _n_dimension, const int _n_data_type,
	const bool _b_normalized, const int _n_stride_bytes, const int _n_offset_bytes,
	class CNextInChain = CGLConfigOp_VertexAttribPtr_ChainEnd>
class CGLConfigOp_VertexAttribPtr {
public:
	enum {
		_n_index_ = _n_index,
		n_index = (_n_index == -1)? 0 : _n_index,
		n_dimension = _n_dimension,
		n_data_type = _n_data_type,
		b_normalized = _b_normalized,
		n_stride_bytes = _n_stride_bytes,
		n_offset_bytes = _n_offset_bytes
	};

	typedef CGLConfigOp_VertexAttribPtr<n_index, n_dimension, n_data_type,
		b_normalized, n_stride_bytes, n_offset_bytes, CNextInChain> CSelf;
	typedef CNextInChain TNextChainedConfigOp;

public:
#if 0 // does not work for some reason // todo - try with g++, see what's the problem
	template <const int i, const int d, const int t, const bool n, const int s, const int o, class CNextChained>
	CGLConfigOp_VertexAttribPtr<((i == -1)? n_index + 1 : i), d, t, n, s, o, CSelf> operator ,(CGLConfigOp_VertexAttribPtr<i, d, t, n, s, o, CNextChained> UNUSED(op))
#else // 1
	template <class CNextConfigOp>
	CGLConfigOp_VertexAttribPtr<((CNextConfigOp::_n_index_ == -1)? n_index + 1 : CNextConfigOp::n_index),
		CNextConfigOp::n_dimension, CNextConfigOp::n_data_type, CNextConfigOp::b_normalized,
		CNextConfigOp::n_stride_bytes, CNextConfigOp::n_offset_bytes, CSelf> operator ,(CNextConfigOp UNUSED(op))
#endif // 1
	{
#if 0
		typedef CGLConfigOp_VertexAttribPtr<i, d, t, n, s, o, CNextChained> CNextConfigOp;
#endif // 1
		_ASSERTE(CNextConfigOp::TNextChainedConfigOp::n_index == -1); // make sure that on the right there is a chain of length 1
		return CGLConfigOp_VertexAttribPtr<((CNextConfigOp::_n_index_ == -1)? n_index + 1 : CNextConfigOp::n_index),
			CNextConfigOp::n_dimension, CNextConfigOp::n_data_type, CNextConfigOp::b_normalized,
			CNextConfigOp::n_stride_bytes, CNextConfigOp::n_offset_bytes, CSelf>();
	}

	/**
	 *	@brief sets vertex array config
	 *	@return Returns vertex stride / size, in bytes.
	 */
	static int Run()
	{
		const int n_next_vert_size = TNextChainedConfigOp::Run();
		// run the next one first, they are composed backwards (head recursion?)

		//printf("debug: config attrib %d (offset %d B)\n", n_index, n_offset_bytes); // debug

		int n_vert_size;
		if(n_dimension) {
			glEnableVertexAttribArray(n_index);
			glVertexAttribPointer(n_index, n_dimension, n_data_type,
				b_normalized, n_stride_bytes, (void*)n_offset_bytes);
			n_vert_size = (n_stride_bytes)? n_stride_bytes : n_dimension * n_GL_TypeSize(n_data_type);
		} else {
			glDisableVertexAttribArray(n_index);
			n_vert_size = 1; // they need to be at least 1B
		}

		//_ASSERTE(!n_next_vert_size || n_next_vert_size == n_vert_size); // don't. could send vec2 to one attrib and vec3 to another when e.g. reusing positions as texcoords
		// make sure that the vertex sizes are the same

		return max(n_vert_size, n_next_vert_size); // unable to do tail recursion on account of the max()
		// calculate vertex size
	}
};

struct CGLConfigOp_VertexAttribPtr_Var_ChainEnd {
	enum {
		b_advance_buffer = false
	};

	const CGLArrayBufferObject *Run(size_t UNUSED(n_max_buffer_num), const CGLArrayBufferObject *p_buffer) const
	{
		return p_buffer;
	}
};

struct TGLVertexAttribConfig_Var {
	int n_index;
	int n_dimension;
	GLenum n_data_type;
	bool b_normalized;
	int n_stride_bytes;
	size_t n_offset_bytes;

	TGLVertexAttribConfig_Var(int _n_index, int _n_dimension, GLenum _n_data_type,
		bool _b_normalized, int _n_stride_bytes, size_t _n_offset_bytes)
		:n_index(_n_index), n_dimension(_n_dimension), n_data_type(_n_data_type),
		b_normalized(_b_normalized), n_stride_bytes(_n_stride_bytes), n_offset_bytes(_n_offset_bytes)
	{}

	TGLVertexAttribConfig_Var()
		:n_index(0), n_dimension(0), n_data_type(0), b_normalized(0), n_stride_bytes(0), n_offset_bytes(0)
	{}

	void Run() const
	{
		if(n_dimension) {
			glEnableVertexAttribArray(n_index);
			glVertexAttribPointer(n_index, n_dimension, n_data_type,
				b_normalized, n_stride_bytes, (const void*)n_offset_bytes);
		} else
			glDisableVertexAttribArray(n_index);
	}
};

template <class TNext = CGLConfigOp_VertexAttribPtr_Var_ChainEnd, const int _n_advance_buffer = 0>
class CGLConfigOp_VertexAttribPtr_Var {
public:
	typedef TNext CNext;
	typedef CGLConfigOp_VertexAttribPtr_Var<CNext, _n_advance_buffer> CSelf;

	enum {
		n_advance_buffer = _n_advance_buffer
	};

protected:
	CNext m_t_next_in_chain; // first is next (chained backwards)
	TGLVertexAttribConfig_Var m_t_config; // then is this data

public:
	CGLConfigOp_VertexAttribPtr_Var(int n_index, int n_dimension, GLenum n_data_type,
		bool b_normalized, int n_stride_bytes, size_t n_offset_bytes, CNext t_next_in_chain = CNext())
		:m_t_config(n_index, n_dimension, n_data_type, b_normalized,
		n_stride_bytes, n_offset_bytes), m_t_next_in_chain(t_next_in_chain)
	{}

	CGLConfigOp_VertexAttribPtr_Var(TGLVertexAttribConfig_Var t_config = TGLVertexAttribConfig_Var(),
		CNext t_next_in_chain = CNext())
		:m_t_config(t_config), m_t_next_in_chain(t_next_in_chain)
	{}

	const TGLVertexAttribConfig_Var &t_VertexAttribPtr_Config() const
	{
		return m_t_config;
	}

	// todo - make chaining of variable and constant attributes possible

#if LVA_STRICTER_TYPE_CHECK
	// this perform stronger checking - can't chain with any other type - must be a CGLConfigOp_VertexAttribPtr_Var
	template <class CNextNext, const bool b_next_advance>
	CGLConfigOp_VertexAttribPtr_Var<CSelf, b_next_advance>
		operator ,(CGLConfigOp_VertexAttribPtr_Var<CNextNext, b_next_advance> t_next_attrib_config) const
	{
		_ASSERTE(sizeof(CNextNext) == sizeof(CGLConfigOp_VertexAttribPtr_Var_ChainEnd)); // should compare the types, but ...
		// make sure that on the right there is a size 1 chain

		return CGLConfigOp_VertexAttribPtr_Var<CSelf,
			b_next_advance>(t_next_attrib_config.t_VertexAttribPtr_Config(), *this);
	}
#else // LVA_STRICTER_TYPE_CHECK
	template <class CNextAttrib>
	struct CInferChainRetVal {
		enum { n_advance = CNextAttrib::n_advance_buffer };
		typedef CGLConfigOp_VertexAttribPtr_Var<CSelf, n_advance> _TyResult;
	};

	template <class CNextAttrib>
	typename CInferChainRetVal<CNextAttrib>::_TyResult operator ,(CNextAttrib t_next_attrib_config) const
	{
		_ASSERTE(sizeof(CNextAttrib) ==
			sizeof(CGLConfigOp_VertexAttribPtr_Var<CGLConfigOp_VertexAttribPtr_Var_ChainEnd>)); // should compare the types, but ...
		// make sure that on the right there is a size 1 chain

		return CGLConfigOp_VertexAttribPtr_Var<CSelf,
			CNextAttrib::n_advance_buffer>(t_next_attrib_config.t_VertexAttribPtr_Config(), *this);
	}
#endif // LVA_STRICTER_TYPE_CHECK

	CGLConfigOp_VertexAttribPtr_Var<CNext, n_advance_buffer + 1> operator ++() const
	{
		return CGLConfigOp_VertexAttribPtr_Var<CNext, n_advance_buffer + 1>(m_t_config, m_t_next_in_chain); // "convert" this object to one with a greater buffer advancement (this does not wrap this in another layer, instead it passes the state to a new object)
	}

	const CGLArrayBufferObject *Run(size_t n_max_buffer_num, const CGLArrayBufferObject *p_buffer) const
	{
		const CGLArrayBufferObject *p_rec = m_t_next_in_chain.Run(n_max_buffer_num, p_buffer);
		// first recurse

		if(n_advance_buffer) // compile-time constant
			p_rec += n_advance_buffer;
		_ASSERTE(p_buffer + n_max_buffer_num > p_rec); // make sure we did not shift too far
		// then see if we shift

		//printf("debug: reconfiguting buffer %d, attribute %d\n", p_rec - p_buffer, m_t_config.n_index);

		p_rec->Bind(); // todo - this is binding a lot; we only really need to bind if we are the first or if we advance the buffer
		m_t_config.Run();
		// bind the buffer and run the vertex attrib pointer setup

		return p_rec;
		// return to the next level
	}
};

} // ~glbuffers_detail

#endif // !__GL_MULTIPLE_VBO_VERTEX_ARRAYS_INLINES_INCLUDED
