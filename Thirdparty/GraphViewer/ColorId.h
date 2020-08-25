/*
								+---------------------------------+
								|                                 |
								|      ***   Color IDs   ***      |
								|                                 |
								|  Copyright  © -tHE SWINe- 2015  |
								|                                 |
								|            ColorId.h            |
								|                                 |
								+---------------------------------+
*/

#pragma once
#ifndef __GL_COLOR_ID_INCLUDED
#define __GL_COLOR_ID_INCLUDED

/**
 *	@file ColorId.h
 *	@brief a simple color id convertor / allocator intended for use with OpenGL
 *	@author -tHE SWINe-
 *	@date 2015
 */

#if defined(__OPENGLES2_EMULATOR_INCLUDED) || defined(__OPENGLES2_INCLUDED)

#define GL_FRAMEBUFFER_ATTACHMENT_RED_SIZE							0x8212 /**< @brief GL_ARB_framebuffer_object (part of OpenGL 3.0 core) enum **/
#define GL_FRAMEBUFFER_ATTACHMENT_GREEN_SIZE						0x8213 /**< @brief GL_ARB_framebuffer_object (part of OpenGL 3.0 core) enum **/
#define GL_FRAMEBUFFER_ATTACHMENT_BLUE_SIZE							0x8214 /**< @brief GL_ARB_framebuffer_object (part of OpenGL 3.0 core) enum **/
#define GL_FRAMEBUFFER_ATTACHMENT_ALPHA_SIZE						0x8215 /**< @brief GL_ARB_framebuffer_object (part of OpenGL 3.0 core) enum **/
#define GL_FRONT_LEFT												0x0400 /**< @brief OpenGL 1.0 enum (draw buffer mode) */
// those are missing in OpenGL ES

#endif // __OPENGLES2_EMULATOR_INCLUDED || __OPENGLES2_INCLUDED

/**
 *	@brief a simple color id encoder for use with OpenGL
 *
 *	This allocates ids as colors. RGB will be used if there is enough resolution,
 *	otherwise RGBA will be used. This always uses the more significant bits in case
 *	the required number of bits is smaller than the resolution, so that the ids
 *	are robust to some small roundoff / conversion errors.
 */
class CGLColorId {
protected:
	Vector4i m_v_bit_depth; /**< @brief bit depth per channel, in RGBA order */
	Vector4i m_v_bit_alloc; /**< @brief bits used per channel, in RGBA order  */
	size_t m_n_id_alloc; /**< @brief number of different ids currently allocated */

public:
	/**
	 *	@brief default constructor
	 *	@note This function does not require OpenGL to be initialized.
	 */
	inline CGLColorId()
		:m_v_bit_depth(0, 0, 0, 0), m_n_id_alloc(0)
	{}

	/**
	 *	@brief reads bit depth off of the currently bound framebuffer
	 *
	 *	@note This function requires OpenGL to be initialized.
	 *	@note The bit depth is cached, and will not be queried again until Get_BitDepth()
	 *		is called explicitly. If on the other hand Get_BitDepth() is not called
	 *		explicitly, the bit depth is queried automatically at the first call to any
	 *		function that needs it.
	 */
	inline void Get_BitDepth(bool b_use_alpha = true)
	{
		m_v_bit_depth = v_Get_BitDepth();
		if(!b_use_alpha)
			m_v_bit_depth.w = 0; // don't allocate anything to alpha
	}

	/**
	 *	@brief gets bit depth that this object allocates IDs for
	 *	@return Returns the bit depth at the last call to Get_BitDepth().
	 */
	inline Vector4i v_BitDepth() const
	{
		return m_v_bit_depth;
	}

	/**
	 *	@brief gets the maximum representable number of IDs
	 *	@return Returns the maximum representable number of IDs, based on the bit depth from the
	 *		last call to Get_BitDepth(), or returns 0, if Get_BitDepth() was not called yet.
	 */
	size_t n_Max_ID_Num() const
	{
		size_t n_bits_available = m_v_bit_depth.x + m_v_bit_depth.y + m_v_bit_depth.z + m_v_bit_depth.w;
		if(n_bits_available >= 8 * sizeof(size_t))
			return SIZE_MAX;
		return n_Mask(n_bits_available) + 1;
	}

	/**
	 *	@brief gets the number of allocated IDs
	 *	@return Returns the number of IDs allocated by the last call to Set_ID_Num().
	 */
	inline size_t n_Allocated_ID_Num() const
	{
		return m_n_id_alloc;
	}

	/**
	 *	@brief deterimenes whether the color IDs are using alpha
	 *	@return Returns true if the color IDs are RGBA, or false if RGB suffices.
	 *	@note The result of this function is undefined before Set_ID_Num() is called.
	 */
	inline bool b_Using_Alpha() const
	{
		return m_v_bit_alloc.w != 0;
	}

	/**
	 *	@brief allocates colors for the given number of IDs
	 *	@param[in] n_id_num is number of IDs to allocate
	 *	@return Returns true on success, false on failure.
	 *	@note In case Get_BitDepth() was not called before, this will call it.
	 */
	bool Set_ID_Num(size_t n_id_num)
	{
		int n_bits_required = (n_id_num)? n_Bit_Width(n_id_num - 1) : 1; // number of bits we need to use

		if(!m_v_bit_depth.x && !m_v_bit_depth.y && !m_v_bit_depth.z && !m_v_bit_depth.w)
			Get_BitDepth();
		int n_bits_available = m_v_bit_depth.x + m_v_bit_depth.y + m_v_bit_depth.z + m_v_bit_depth.w;
		if(n_bits_required > n_bits_available)
			return false;

		m_v_bit_alloc = v_ColorID_BitAlloc(n_bits_required, m_v_bit_depth);
		m_n_id_alloc = n_id_num;

		return true;
	}

	/**
	 *	@brief gets the smallest OpenGL unsigned integer data type
	 *		that can hold the colors in the required bit depth
	 *	@return Returns one of GL_UNSIGNED_BYTE, GL_UNSIGNED_SHORT or GL_UNSIGNED_INT.
	 *	@note This is based on the last call to Set_ID_Num().
	 */
	inline GLenum n_Int_DataType() const
	{
		_ASSERTE(m_n_id_alloc > 0); // make sure Set_ID_Num() was called
		int n_max_component_depth = max(max(m_v_bit_alloc.x,
			m_v_bit_alloc.y), max(m_v_bit_alloc.z, m_v_bit_alloc.w));
		if(n_max_component_depth <= 8)
			return GL_UNSIGNED_BYTE;
		else if(n_max_component_depth <= 16)
			return GL_UNSIGNED_SHORT;
		_ASSERTE(n_max_component_depth <= 32);
		return GL_UNSIGNED_INT;
	}

	/**
	 *	@brief converts ID to a MSB un-aligned color
	 *	@param[in] n_id is zero-based id, less than the value returned by n_Allocated_ID_Num()
	 *	@return Returns color as RGBA integer vector. The values are not MSB-aligned,
	 *		rather they are in the same range as the depth of the framebuffer (so in most
	 *		cases just cast them to uint8_t and specify e.g. using glVertexAttrib4ubv()).
	 */
	inline Vector4i v_ID_to_Color(size_t n_id) const
	{
		_ASSERTE(m_n_id_alloc > n_id); // make sure someone called Set_ID_Num()
		return v_ID_to_Color(n_id, m_v_bit_alloc, m_v_bit_depth);
	}

	/**
	 *	@brief converts ID to a color
	 *
	 *	@param[out] p_RGBA_color is filled with RGBA color with the specified number
	 *		of components (must be allocated by the caller to n_color_size_bytes)
	 *	@param[in] n_color_size_bytes is size the p_RGBA_color array is allocated to
	 *	@param[in] n_GL_data_type is data type used (one of GL_UNSIGNED_BYTE,
	 *		GL_UNSIGNED_SHORT or GL_UNSIGNED_INT)
	 *	@param[in] n_id is zero-based id, less than the value returned by n_Allocated_ID_Num()
	 *
	 *	@return Returns color as RGBA integer vector. The values are not MSB-aligned,
	 *		rather they are in the same range as the depth of the framebuffer (so in most
	 *		cases just cast them to uint8_t and specify e.g. using glVertexAttrib4ubv()).
	 */
	void ID_to_Color(void *p_RGBA_color, size_t n_color_size_bytes,
		GLenum n_GL_data_type, size_t n_id)
	{
		int n_bit_width;
		switch(n_GL_data_type) {
		case GL_UNSIGNED_BYTE:
			n_bit_width = 8;
			break;
		case GL_UNSIGNED_SHORT:
			n_bit_width = 16;
			break;
		case GL_UNSIGNED_INT:
			n_bit_width = 32;
			break;
		default:
			_ASSERTE(0); // unknown type
		};
		// determine width

		_ASSERTE(n_color_size_bytes >= unsigned(n_bit_width / 8 * 4));
		// make sure there is enough space in the buffer

		Vector4i v_color = v_ID_to_Color(n_id, m_v_bit_alloc,
			Vector4i(n_bit_width, n_bit_width, n_bit_width, n_bit_width));
		// convert to color, MSB-aligned, no extra shifting required

		switch(n_GL_data_type) {
		case GL_UNSIGNED_BYTE:
			{
				for(int i = 0; i < 4; ++ i) {
					_ASSERTE(m_v_bit_depth[i] <= n_bit_width); // make sure it fit in the given format without clipping
					((uint8_t*)p_RGBA_color)[i] = v_color[i];
				}
			}
			break;
		case GL_UNSIGNED_SHORT:
			{
				for(int i = 0; i < 4; ++ i) {
					_ASSERTE(m_v_bit_depth[i] <= n_bit_width); // make sure it fit in the given format without clipping
					((uint16_t*)p_RGBA_color)[i] = v_color[i];
				}
			}
			break;
		case GL_UNSIGNED_INT:
			{
				for(int i = 0; i < 4; ++ i) {
					_ASSERTE(m_v_bit_depth[i] <= n_bit_width); // make sure it fit in the given format without clipping
					((uint32_t*)p_RGBA_color)[i] = v_color[i];
				}
			}
			break;
		default:
			_ASSERTE(0); // unknown type
		};
		// write to the specified pixel type
	}

	/**
	 *	@brief converts MSB un-aligned color to an ID
	 *
	 *	@param[in] v_color is color as RGBA integer vector (the values are not MSB-aligned)
	 *
	 *	@return Returns the closest zero-based id, which might be outside of the allocated ID range
	 *		in case the color is different from those generated by v_ID_to_Color() or ID_to_Color().
	 *
	 *	@note To get the color, in most cases just use glReadPixels() with GL_UNSIGNED_BYTE (assuming
	 *		bits per channel is 8) and store this 0 - 255 value in int (OpenGL will MSB align the value
	 *		which needs to be undone if the depth is lower - e.g. for 565 it would need to be shifted
	 *		right by 3, 2 and 3 bits).
	 *	@note See also n_Color_to_ID(const void*,size_t,GLenum).
	 */
	inline size_t n_Color_to_ID(Vector4i v_color) const
	{
		_ASSERTE(m_n_id_alloc > 0); // make sure someone called Set_ID_Num()
		return n_Color_to_ID(v_color, m_v_bit_alloc, m_v_bit_depth);
	}

	/**
	 *	@brief converts color to an ID
	 *
	 *	@param[in] p_RGBA_color is color as read back from OpenGL
	 *	@param[in] n_color_size_bytes is size of the color information, in bytes (without padding)
	 *	@param[in] n_readback_data_type is data type used (one of GL_UNSIGNED_BYTE,
	 *		GL_UNSIGNED_SHORT or GL_UNSIGNED_INT)
	 *
	 *	@return Returns the closest zero-based id, which might be outside of the allocated ID range
	 *		in case the color is different from those generated by v_ID_to_Color() or ID_to_Color().
	 */
	size_t n_Color_to_ID(const void *p_RGBA_color, size_t n_color_size_bytes,
		GLenum n_readback_data_type) const
	{
		Vector4i v_color;
		int n_bit_width;
		switch(n_readback_data_type) {
		case GL_UNSIGNED_BYTE:
			n_bit_width = 8;
			_ASSERTE(n_color_size_bytes >= unsigned(n_bit_width / 8 * 4)); // make sure there is enough space in the buffer
			{
				for(int i = 0; i < 4; ++ i) {
					_ASSERTE(m_v_bit_depth[i] <= n_bit_width); // make sure it fit in the given format without clipping
					v_color[i] = ((uint8_t*)p_RGBA_color)[i];
				}
			}
			break;
		case GL_UNSIGNED_SHORT:
			n_bit_width = 16;
			_ASSERTE(n_color_size_bytes >= unsigned(n_bit_width / 8 * 4)); // make sure there is enough space in the buffer
			{
				for(int i = 0; i < 4; ++ i) {
					_ASSERTE(m_v_bit_depth[i] <= n_bit_width); // make sure it fit in the given format without clipping
					v_color[i] = ((uint16_t*)p_RGBA_color)[i];
				}
			}
			break;
		case GL_UNSIGNED_INT:
			n_bit_width = 32;
			_ASSERTE(n_color_size_bytes >= unsigned(n_bit_width / 8 * 4)); // make sure there is enough space in the buffer
			{
				for(int i = 0; i < 4; ++ i) {
					_ASSERTE(m_v_bit_depth[i] <= n_bit_width); // make sure it fit in the given format without clipping
					v_color[i] = ((uint32_t*)p_RGBA_color)[i];
				}
			}
			break;
		default:
			_ASSERTE(0); // unknown type
		};
		// determine bit width of the readback; special formats not supported

		return n_Color_to_ID(v_color, m_v_bit_alloc,
			Vector4i(n_bit_width, n_bit_width, n_bit_width, n_bit_width));
		// shifts inside, no extra operations required
	}

	/**
	 *	@brief gets the current bit depth
	 *	@return Returns bit depth in RGBA order.
	 */
	static Vector4i v_Get_BitDepth()
	{
		GLint n_red_bit_num, n_green_bit_num, n_blue_bit_num, n_alpha_bit_num;
#ifdef __APPLE__
		glGetFramebufferAttachmentParameteriv(GL_FRAMEBUFFER, GL_FRONT_LEFT,
			GL_FRAMEBUFFER_ATTACHMENT_RED_SIZE, &n_red_bit_num);
		glGetFramebufferAttachmentParameteriv(GL_FRAMEBUFFER, GL_FRONT_LEFT,
			GL_FRAMEBUFFER_ATTACHMENT_GREEN_SIZE, &n_green_bit_num);
		glGetFramebufferAttachmentParameteriv(GL_FRAMEBUFFER, GL_FRONT_LEFT,
			GL_FRAMEBUFFER_ATTACHMENT_BLUE_SIZE, &n_blue_bit_num);
		glGetFramebufferAttachmentParameteriv(GL_FRAMEBUFFER, GL_FRONT_LEFT,
			GL_FRAMEBUFFER_ATTACHMENT_ALPHA_SIZE, &n_alpha_bit_num);
		// core context
#else // __APPLE__
		glGetIntegerv(GL_RED_BITS, &n_red_bit_num);
		glGetIntegerv(GL_GREEN_BITS, &n_green_bit_num);
		glGetIntegerv(GL_BLUE_BITS, &n_blue_bit_num);
		glGetIntegerv(GL_ALPHA_BITS, &n_alpha_bit_num);
		// compatible context
#endif // __APPLE__
		// determine framebuffer bit depth (5 5 5 1, 5 6 5 0 or 8 8 8 8 bits)

		return Vector4i(n_red_bit_num, n_green_bit_num, n_blue_bit_num, n_alpha_bit_num);
	}

	/**
	 *	@brief allocates component bits to represent the required number of IDs
	 *
	 *	@param[in] n_number_of_id_s is the required number of IDs
	 *	@param[in] v_bd is bit depth per channel, in RGBA order (must be sufficient
	 *		to represent the number of IDs)
	 *
	 *	@return Returns the number of component bits to be used (in RGBA order).
	 */
	static inline Vector4i v_ColorID_Alloc(size_t n_number_of_id_s, Vector4i v_bd = v_Get_BitDepth())
	{
		int n_bits_required = (n_number_of_id_s)? n_Bit_Width(n_number_of_id_s - 1) : 1; // number of bits we need to use
		return v_ColorID_BitAlloc(n_bits_required, v_bd);
	}

	/**
	 *	@brief allocates component bits to represent the required number of IDs
	 *
	 *	@param[in] n_bits_required is the required number of bits to represent the IDs
	 *	@param[in] v_bd is bit depth per channel, in RGBA order (the sum of the
	 *		components must be greater or equal to n_bits_required)
	 *
	 *	@return Returns the number of component bits to be used (in RGBA order).
	 */
	static Vector4i v_ColorID_BitAlloc(int n_bits_required, Vector4i v_bd = v_Get_BitDepth())
	{
		_ASSERTE(n_bits_required <= v_bd.x + v_bd.y + v_bd.z + v_bd.w); // make sure we have enough bits

		if(n_bits_required <= v_bd.x + v_bd.y + v_bd.z)
			v_bd.w = 0;
		// don't use alpha if not needed

		int n_extra_bits = v_bd.x + v_bd.y + v_bd.z + v_bd.w - n_bits_required;
		_ASSERTE(n_extra_bits >= 0);
		// the number of color bits that we don't need

		int n_zero_channel_num = !v_bd.x + !v_bd.y + !v_bd.z + !v_bd.w;
		int n_nnz_channel_num = 4 - n_zero_channel_num;
		// the number of zero / nonzero channels

		int n_unused_bits_channel = n_extra_bits / n_nnz_channel_num;
		int n_unused_bits_first_channel = n_extra_bits - n_unused_bits_channel * (n_nnz_channel_num - 1);
		_ASSERTE(n_unused_bits_first_channel >= n_unused_bits_channel); // the division rounds down, this can only be larger
		// divide (almost) equally among the channels

		//return v_bd - Vector4i(n_unused_bits_first_channel, n_unused_bits_channel,
		//	n_unused_bits_channel, (v_bd.w)? n_unused_bits_channel : 0);
		// calculate step (works only if the red channel is present)

		int n_largest_component = std::max_element(&v_bd[0], &v_bd[0] + 4) - &v_bd[0];
		Vector4i v_ub((v_bd.x)? n_unused_bits_channel : 0, (v_bd.y)? n_unused_bits_channel : 0,
			(v_bd.z)? n_unused_bits_channel : 0, (v_bd.w)? n_unused_bits_channel : 0);
		// distribute the number of unused bits

		if(n_nnz_channel_num) {
			_ASSERTE(v_bd[n_largest_component] > 0);
			v_ub[n_largest_component] = n_unused_bits_first_channel;
		}
		// assign the potentially larger number of unused bits to the largest channel

		return v_bd - v_ub;
	}

	/**
	 *	@brief converts ID to a color
	 *
	 *	@param[in] n_id is zero-based id, less than the value returned by n_Allocated_ID_Num()
	 *	@param[in] v_bits_used is bit allocation per channel, in RGBA order
	 *		(obtained by calling e.g. v_ColorID_Alloc())
	 *	@param[in] v_bd is bit depth per channel, in RGBA order
	 *
	 *	@return Returns color as RGBA integer vector. The values are not MSB-aligned,
	 *		rather they are in the same range as the depth of the framebuffer (so in most
	 *		cases just cast them to uint8_t and specify e.g. using glVertexAttrib4ubv()).
	 */
	static Vector4i v_ID_to_Color(size_t n_id, Vector4i v_bits_used, Vector4i v_bit_depth)
	{
		Vector4i v_color;
		for(int i = 0; i < 4; ++ i) {
			int n_used_bits = v_bits_used[i], n_width = v_bit_depth[i];
			int n_shift = n_width - n_used_bits;
			_ASSERTE(n_used_bits >= 0 && n_used_bits <= 32);
			v_color[i] = (n_id & n_Mask(n_used_bits)) << n_shift;
			n_id >>= n_used_bits;
		}
		// create RGBA

		_ASSERTE(!n_id);
		// make sure all the bits were encoded

		return v_color;
	}

	/**
	 *	@brief converts color to an ID
	 *
	 *	@param[in] v_color is color as RGBA integer vector (the values are not MSB-aligned)
	 *	@param[in] v_bits_used is bit allocation per channel, in RGBA order
	 *		(obtained by calling e.g. v_ColorID_Alloc())
	 *	@param[in] v_bd is bit depth per channel, in RGBA order
	 *
	 *	@return Returns the closest zero-based id.
	 *
	 *	@note To get the color, in most cases just use glReadPixels() with GL_UNSIGNED_BYTE (assuming
	 *		bits per channel is 8) and store this 0 - 255 value in int (do not try to MSB align).
	 */
	static size_t n_Color_to_ID(Vector4i v_color, Vector4i v_bits_used, Vector4i v_bit_depth)
	{
		_ASSERTE(!(v_color[3] & ~n_Mask(v_bit_depth[3]))); // make sure the number is not larger than it is supposed to be
		//_ASSERTE(!(v_color[3] & n_Mask(v_bit_depth[3] - v_bits_used[3]))); // make sure that the lower bits are zero // note that this would defeat the purpose, we allow the bits to not be zero due to conversion errors and round them in here to obtain "robust" color ids

		size_t n_id = (uint32_t(v_color[3]) >> (v_bit_depth[3] - v_bits_used[3])) & n_Mask(uint32_t(v_bits_used[3]));
		for(int i = 3; i > 0;) {
			-- i; // here

			_ASSERTE(!(v_color[i] & ~n_Mask(v_bit_depth[i]))); // make sure the number is not larger than it is supposed to be
			//_ASSERTE(!(v_color[i] & n_Mask(v_bit_depth[i] - v_bits_used[i]))); // make sure that the lower bits are zero // note that this would defeat the purpose, we allow the bits to not be zero due to conversion errors and round them in here to obtain "robust" color ids

			n_id <<= v_bits_used[i];
			n_id |= (uint32_t(v_color[i]) >> (v_bit_depth[i] - v_bits_used[i])) & n_Mask(uint32_t(v_bits_used[i]));
		}
		// calculate the id

		return n_id;
	}

	/**
	 *	@brief unit tests (does not require OpenGL)
	 */
	static void Test()
	{
#ifdef _DEBUG
		const Vector4i p_depth_list[] = {
			Vector4i(2, 2, 2, 2),
			Vector4i(5, 5, 5, 1),
			Vector4i(5, 6, 5, 0),
			Vector4i(0, 0, 0, 8), // alpha only
			Vector4i(0, 0, 0, 16), // alpha only
			Vector4i(6, 6, 6, 6), // fictive format
			Vector4i(7, 7, 7, 7), // fictive format
			Vector4i(8, 8, 8, 8) // takes forever but does not seem to crash
		};
		const size_t n_depth_num = sizeof(p_depth_list) / sizeof(p_depth_list[0]);
		for(size_t i = 0; i < n_depth_num; ++ i) {
			Vector4i v_bit_depth = p_depth_list[i];
			printf("testing with %d%d%d%d\n", v_bit_depth.x,
				v_bit_depth.y, v_bit_depth.z, v_bit_depth.w);

			CGLColorId cid;
			cid.m_v_bit_depth = v_bit_depth;
			// mimic calling Get_BitDepth()

			size_t n_max_id_num = cid.n_Max_ID_Num();
			_ASSERTE(n_max_id_num > 0);
			_ASSERTE(cid.n_Allocated_ID_Num() == 0);

			const size_t p_ID_num[] = {
				n_max_id_num / 2,
				n_max_id_num,
				1 + rand() % n_max_id_num,
				1 + rand() % n_max_id_num,
				1 + rand() % n_max_id_num,
				1 + rand() % n_max_id_num
			};
			const size_t n_test_num = sizeof(p_ID_num) / sizeof(p_ID_num[0]);

			for(size_t j = 0; j < n_test_num; ++ j) {
				size_t n_id_num = p_ID_num[j];

				bool b_result = cid.Set_ID_Num(n_id_num);
				_ASSERTE(b_result);
				_ASSERTE(cid.n_Allocated_ID_Num() == n_id_num);
				// allocate color IDs

				GLenum n_data_type = cid.n_Int_DataType();
				int n_data_size = (n_data_type == GL_UNSIGNED_BYTE)? 1 :
					(n_data_type == GL_UNSIGNED_SHORT)? 2 : (n_data_type == GL_UNSIGNED_INT)? 3 : -1;
				// see what data type we need to use

				for(size_t n_id = 0; n_id < n_id_num; ++ n_id) {
					{
						Vector4i v_color = cid.v_ID_to_Color(n_id);
						size_t n_back = cid.n_Color_to_ID(v_color);
						_ASSERTE(n_back == n_id);
					}
					// MSB un-aligned test

					{
						union {
							uint8_t p_int8[4];
							uint16_t p_int16[4];
							uint32_t p_int32[4];
						} t_color;

						{
							cid.ID_to_Color(&t_color, sizeof(t_color), n_data_type, n_id);
							size_t n_back = cid.n_Color_to_ID(&t_color, sizeof(t_color), n_data_type);
							_ASSERTE(n_back == n_id);
						}
						// simple case

						{
							for(int j = 0; j < 4; ++ j) {
								int n_low_bits = n_data_size * 8 - v_bit_depth[j];
								switch(n_data_size) {
								case 1:
									t_color.p_int8[j] &= ~n_Mask(n_low_bits);
									break;
								case 2:
									t_color.p_int16[j] &= ~n_Mask(n_low_bits);
									break;
								case 4:
									t_color.p_int32[j] &= ~n_Mask(uint32_t(n_low_bits));
									break;
								};
							}
							size_t n_back2 = cid.n_Color_to_ID(&t_color, sizeof(t_color), n_data_type);
							_ASSERTE(n_back2 == n_id);
						}
						// clear the low bits that are not represented in the framebuffer

						{
							for(int j = 0; j < 4; ++ j) {
								int n_low_bits = n_data_size * 8 - v_bit_depth[j];
								switch(n_data_size) {
								case 1:
									t_color.p_int8[j] |= n_Mask(n_low_bits);
									break;
								case 2:
									t_color.p_int16[j] |= n_Mask(n_low_bits);
									break;
								case 4:
									t_color.p_int32[j] |= n_Mask(uint32_t(n_low_bits));
									break;
								};
							}
							size_t n_back3 = cid.n_Color_to_ID(&t_color, sizeof(t_color), n_data_type);
							_ASSERTE(n_back3 == n_id);
						}
						// raise the low bits that are not represented in the framebuffer
					}
					// MSB aligned test
				}
				// convert id to color and back
			}
			// make sure the ids convert correctly

			if(n_max_id_num < SIZE_MAX) {
				size_t n_last_id_num = cid.m_n_id_alloc;
				Vector4i v_last_bit_alloc = cid.m_v_bit_alloc;
				Vector4i v_last_bit_depth = cid.m_v_bit_depth;
				// remember the last state

				_ASSERTE(!cid.Set_ID_Num(n_max_id_num + 1));
				// make sure that setting more ids will fail

				_ASSERTE(n_last_id_num == cid.m_n_id_alloc &&
					v_last_bit_alloc == cid.m_v_bit_alloc &&
					v_last_bit_depth == cid.m_v_bit_depth);
				// make sure that the state is unchanged by this

				_ASSERTE(!cid.Set_ID_Num(SIZE_MAX)); // remember, n_max_id_num < SIZE_MAX
				// make sure that setting more ids will fail

				_ASSERTE(n_last_id_num == cid.m_n_id_alloc &&
					v_last_bit_alloc == cid.m_v_bit_alloc &&
					v_last_bit_depth == cid.m_v_bit_depth);
				// make sure that the state is unchanged by this
			}
			// test some failure cases as well
		}
#else // _DEBUG
		fprintf(stderr, "warning: this test makes use of assertions, nothing tested in release\n");
#endif // _DEBUG
	}
};

#endif // !__GL_COLOR_ID_INCLUDED
