/**
 *	@file Gradient.h
 *	@brief image in header, generated from 'gradientr.tga'
 *		- using prediction / RLE / Huff / BASE64 pipe (lossy)
 *		- predictors used: left for luminance, left for chrominance, none for alpha
 *	@date 2014-09-21 21:47:17
 *	@author ImageToHeader (built Dec 10 2013)
 */

#pragma once
#ifndef __GRADIENTR_TGA_INCLUDED
#define __GRADIENTR_TGA_INCLUDED

/**
 *	@brief image in header, generated from 'gradientr.tga'
 */
class CGradientImage {
public:
	/**
	 *	@brief gets width of the image
	 *	@return Returns image width, in pixels.
	 */
	static inline int n_Width()
	{
		return 113;
	}

	/**
	 *	@brief gets height of the image
	 *	@return Returns image height, in pixels.
	 */
	static inline int n_Height()
	{
		return 537;
	}

	/**
	 *	@brief determines whether the image contains alpha channel
	 *	@return Returns true if the image contains alpha channel, otherwise false.
	 */
	static inline bool b_Alpha()
	{
		return true;
	}

	/**
	 *	@brief determines whether the image is grayscale
	 *	@return Returns true if the image is grayscale, otherwise false.
	 */
	static inline bool b_Grayscale()
	{
		return false;
	}

	/**
	 *	@brief determines whether the compression is lossless
	 *	@return Returns true if the compression is lossless, otherwise false.
	 */
	static inline bool b_Lossless()
	{
		return false;
	}

	/**
	 *	@brief gets BASE64 compressed image data
	 *	@return Returns BASE64 compressed image data in a null-terminated string.
	 */
	static inline const char *p_s_Base64_Data()
	{
		return "AAAAAAMAAAAAAAAAAQAAAAIAAAACAAAAAwAAAAUAAAAOAAAACwAAAAwAAAAPAAAAEAAAABQAAAAAAAAAAAA"
			"AAP8BAAkMBBH8HBkUBd8hU/4pAgdvOeEPJBAxcRYsBggOQQ3FSRceGC4bNBUrWSbjNycvwT8LUfsTR+thZ1X9N"
			"S1pS+nzHVdtozPN13MfOyMgX4uNpTYKEk+HJTyDW4G3vn3Hy9Xl5+31v2+u7Iwkj/TdBJGEkftrz/1WhJH+++9"
			"RhJGEy0jCZdkYSRy7CSMJl0RhJGEy0jCZdkYSRy+fnE4tIwn8lpGE4tIwnqviiSMJxdkYSRy6CSMJI5e0SRhJG"
			"E4uyMJI5dhJGE/RaRhPhaRhPNfNR4ugkjCSPFoSRy0JI5aEkeLoJIwkjxaEkcrz99PfT/utIwmWkYTLSMJ9VpG"
			"Ey0jCZe5GEn0XRGEkYTrbRy9okjCSMJxdEYSRhOttHL3CSN5/RbEy2jl0EkYSR4tCSPhbEy2jl0EkYSR4voEn+"
			"e0JI9bQkj9toSRy+lRhOtpGE62kYTwtIwnW0jCdb3Iwk+y0jCcWkYTrdEYSRhOtpGE4tIwnW+hG8/3WhJHLQkj"
			"1tCSPvtCSOWhJHrfQJPotCSPwtCSP23zUctiZdEYSRhOLSMJ4W0ctiZdEYSRhOL8UYSUYSRhMuyMJI5aEkcuwk"
			"jCZdEYSRhMuyMJI5aEkcrz/stCSOWhJHLQkj9VoSRy/UJIyJIwnF2RhJHLoJIwkjl7RJGEkYTi7Iwkjl+AkjCT"
			"+20jCfC0jCeanvvpUYTLSMJl0RhJGEy2jlsTi0jCZaRhMuiMJIwmX0I3n9FoSRy6CSMJI5aEkctCSPW0JI5dBJ"
			"GEkctCSOXxRJGEy2jl2EkYTLojCSMJl0RhJGEy2jl2EkYTL3Iwk+y0JI5aEkfNaEkfC0JI5aEkfNT6LQkj4WhJ"
			"H6rQkj4WhJHrT7LSMJ8LSMJ9Vef0WxOtpGE62kYTrbRy2J1tIwnW0jCfzXyRhJGEkYTL5IwkjCSMJ1vkjCSMJI"
			"wmX4Iwkjef7rQkjloSR62hJH32hJHLQkj1voEn/laRhPhfNRy2Jl2RhJHLYmWkYTi7IwkjlsTLsjCSOWxMvxRh"
			"JRhJGEy7IwkjloSR62xMuiMJIwmXZGEkctCSOV5/2WhJHLQkjloSR+q0JI5e4SRn/vtIwnwvcjCT+20jCfC0jC"
			"ea/FRhJGEkYTLaOWxMuyMJI5fmEkYSRhJGEy2jlsTL6Ebz+i0JI/ptCSOWhJH7afRaEkcugkjCSOWhJHLQkj1t"
			"CSOXQSRhJHLQkjl8fOJIwmWkYTLSMJlpGE63RGEkYTLSMJlpGEy0jCfrrz+i2JltHLoJIwkjlsTi2j1tiZbRy6"
			"CSMJI5bE/K0jCfC0jCfVXn9loSR812EkYTLaPhaEkfNfgJIwk99P7rQkj1tCSP22hJHrfQJP3WkYT1WkYTzWkY"
			"T1XzUctiZdkYSRy2JlpGE4uyMJI5bEy7IwkjlsTL8UYSUYSRhMuyMJI++2Jl0RhJGEy7Iwkj4V5/2WhJHLQkjl"
			"oSR+q0JI5e4SRn0WkYT1WkYTLSMJxaRhPVfejCSjCZdEYSRhOLaPFsTLojCSMJl0RhJGE4to8X4qMJIwkjCZbR"
			"y2Jl2RhJHL8wkjCSMJIwmW0ctiZfQjef0WhJH9NoSRy0JI/bT6bSMJ/TfHziSMJltHLsJIwmXRGEkYTLojCSMJ"
			"ltHLsJIwmXxRhKRy0JI5aEket7RJGEkYTLaOWhJHLQkj1vcJI3n9FsTLaOXQSRhJHLYnFtHrbEy2jl0EkYSRy2"
			"JlPstIwnwtIwn1U99PstIwnFpGE81pGE62kYTi0jCeFef7rQkjloSR62hJH32hJHLQkj1tiflaRhMtIwnhbRy2"
			"J4WkYTLSMJ4V5/4WhJHraEkfttCSP810RhJGEy7Iwkj77YmXRGEkYTLsjCSPhXn774+cSRhPC0jCZaRhMvkjCS"
			"MJIwnhaRhMv1IwlIwkjl8hJGEkYSR4tiZe1GEkYSRy+QkjCSMJI8XzUeLoJIwkjxbEy7IwkjloSR4ugkjCSPFs"
			"TL6Ebz99P7bSMJ77aOWxPNaRhPNfHziSMJltHLsJIwmXRGEkYTLojCSMJltHLsJIwmXuRhJ9loSRy0JI9b5CSM"
			"JIwkjxaEkctCSPW9wkjPfT7LSMJ8LSMJ9Vef2XyEkYSRhJHLsJIwmXZGEkeL5CSMJIwkjl+oSRhJEkcugkjCSO"
			"WhJHi+QkjCSMJI5dBJGEkctCSPFP3WhJHLQkj1tCSPvtCSOWhJHrfQJPfT7LQkj4WxMto5bEy2j1tCSPhbEy2j"
			"+VpGE8LSMJlpGE81pGE8LSMJl9CN5/2WhJHLQkjloSR+q0JI5fqEkZEkYTwtIwmWkYTL5IwkjCSMJ4WkYTLSMJ"
			"+unvp9FoSR8LQkj5rQkjloSR8LQkj1p76f3XQSRhJHraEkfVdBJGEkfytIwn1WkYTwtIwn1X0I3n9FsTLaOXQS"
			"RhJHi0JI+FsTLaOXQSRhJHi2J/NaRhPhaRhPqp76f6rSMJ/fT30+u2jloSRy0JI5bEy0jCcXZGEkctCSOWhJHL"
			"YmXuRhJ768/otCSPW2JxaRhMuyMJI8WhJHrbE4tIwn9FPfT/ytCSPwp76e++KJIwmW0cuwkjCZdEYSRhMuiMJI"
			"wmW0cuwkjCZe5GEn2WhJHLQkj5rQkj4WhJHLQkj5qfRbEy2jl0EkYSRy2JxdkYSRy2JltHLoJIwkjlsT+invp+"
			"60JI9bQkj9toSR62hJHin13ZGEkeLQkj9V0EkYSR4tCSPmvmo5bEy7IwkjlsTLSMJxdkYSRy2Jl2RhJHLYmXuR"
			"hLz+i0JI5aEkf02hJHLQkj775onF0RhJGE81pGEy0jCcXRGEkYQv4099P3XQSRhJHi0JI5aEkfNdBJGEkeLQkj"
			"lPruiMJIwmXZGEkcuwkjCZbRy9okjCSMJl2RhJHL7xJGEvOJ1tIwmWkYTLSMJ1tIwnW0jCZaRhMvijCUjloSRy"
			"0JI9b2iSMJIwmW0ctCSOWhJHrfESRvOJ1tIwnFpGE63RGEkYTraRhOLSMJ/lfSownW0jCdbSMJ4WkYTraRhOt7"
			"kYSe+n12kYTLaOWhJHLoJIwkjxbEy0jCZbRy0JI5f8RJGEkZ/daRhPhaRhPC+9RhJGEy7Iwkj77YmXRGEkYTLs"
			"jCSPhfPzicWkYT+S0jCcWkYQvyp76e+n13RGEkYTLSMJl0RhJGEy2jl7RJGEkYTLSMJl/xRhJGEvP/jdEYSRhM"
			"uyMJI5e0SRhJGE4uiMJIwmXZGEkcvcJIz3099PfT30/utCSPW0JI/baEketsT8rSMJlpGEy0jCZbRy2J4WkYTL"
			"SMJlpGEyvP/C0JI9bQkj9toSR/muiMJIwmWkYTLsjCSOWxOt0RhJGEy0jCZdkYSRynvp76fRaEkfNaEkfVaEkf"
			"NaEkeKe+n0WhJH9NoSRy0JI/7bSMJ77aOWxPNaRhPNXn/haEkfhaEkfytIwnvuiMJIwnhaRhPffgjCSM99PstI"
			"wnW0jCdbSMJ4WkYTraRhOtPfT30/daEkctCSPW0JI++0JI5aEketsT8rSMJ6raOWxPC0jCeqvP7LQkjloSRy2J"
			"ltHLYmW0etoSRy0JI5bEy2j/haRhMtIwn6bSMJl9CM99PotIwnqtIwmWkYTi0jCeq9yMJPfT30+i0JI5aEkcug"
			"kjCSPFsTLaOWhJHLQkjl0EkYSRynvvpRJHLQkjl2EkYTLaPFoSRy0JI5aEkcuwkjCZbR/op76f57Qkj8LQkjlP"
			"fT30+i6CSMJI5bEy2jxdBJGEkeLoJIwkjlsTLaPF9Ak+i0JI/C0JI/4W0ctiZdkYSRy2JlpGE4uyMJI5bEy7Iw"
			"kjlsTL3Iwk99PffSiSOWhJH9NoSRy0JI+qnvp/baEkeLQkjloSR9VoSR4tCSP9FeXstIwmWkYTLSMJ1to5bE4t"
			"IwmWkYTLSMJxR315ei0jCfC0jCfbfME4to5e0SRhJGE63RGEkYTi2jl7RJGEkYTi+nyEkeLoJIwkjloSR63QSR"
			"hJHi6CSMJI5aEkco9l+YSRhJGEkYTLSMJl2RhJHi/MJIwkjCSMJl7kYS8u+vLvry+60jCeq2jlsTwtIwnqo/qt"
			"CSP99eXffME4uiMJIwn1WkYTi6IwkjCea+nyEkctCSOWhJHraEkcugkjCSOWhJHLQkj1viJeSMJl+ZGEkYSRhJ"
			"Hi2Jl0RhJGEy/MjCSMJIwkjxR+20JI8WhJHLQkj6rQkjxaEkcry9FoSR8LQkj5rQkjloSR8LQkj1ry/baRhPfb"
			"Ry2J5rSMJ5qP4WhJH4WhJHivL2WhJHLQkj1vkJIwkjCSPFoSRy0JI9b4iSMCeq0jCdbojCSMJ6rSMJxfT5CSPF"
			"0EkYSRy0JI8WxMuyMJI8XQSRhJHLQkjlHsvzCSMJIwkjCZaRhMuyMJI8X5hJGEkYSRhMvijCXkJl7UYSRhJHLQ"
			"kjxe0SRhJGEy9qMJIwkjloSR4ry+60JI5aEketoSR99oSRy0JI9b6BLy+60jCeq2jlsTwtIwnqvn5I5bEy7Iwk"
			"jlsTLSMJxdkYSRy2Jl2RhJHLYmX4owl5IwkjCZdkYSR99sTLojCSMJl2RhJH/hbE4tIwn8lpGE4tIwnqvp8hJH"
			"LYmXZGEkcugkjCSOXQSRhJHLYmXZGEkcv1CSMJeSMJl0RhJGE4uiMJIwmXRGEkYTLojCSMJxfgjCSN5d9HotCS"
			"PhdBJGEketoSRy0JI+F0EkYSRyvL9tpGE99tHLYnmtIwnmviCSMJlpGEy0jCZaRhOt0RhJGEy0jCZaRhMvcjCX"
			"9nouiMJIwmXZGEkcvaJIwkjCcXRGEkYTLsjCSOXuEkY/haRhOtpGE+20jCcX0gkjxdBJGEkctCSPFsTLsjCSPF"
			"0EkYSRy0JI5fT5IwnW0jCdbSMJ4WkYTraRhOt7kYS8u+v7O+vL7rSMJ6raOWxPC0jCeq+fkjlsTLsjCSOWxMtI"
			"wnF2RhJHLYmXZGEkctiZe5GEjvo/ZaEkctCSOWhJH6rQkjl7hJG8u+vLvvx8kYSRhJGEy2jlsTrbRy/MJIwkjC"
			"SMJltHLYnFHfXl6bSMJ/TfEEkYTLaOXYSRhMuiMJIwmXRGEkYTLaOXYSRhMvcjCXl30ei2JltHLoJIwkjxaEkf"
			"C2JltHLoJIwkjxfQJeX89oSR62hJH7bQkjleX+e0JI/C+gS8vZfJGEkYSRhOt0RhJGE63yRhJGEkYTrfQjHffH"
			"+xGEkf6boJIwkj9t8xHLYmXZGEkctiZaRhOLsjCSOWxMuyMJI5bEy9yMJHfR6LQkj4WxMtIwmXZGEkeLQkj4Wx"
			"MtIwmX0+QkjloSRy0JI9bQkjl0EkYSRy0JI5aEket9Al5d9+PkjCSMJIwmWkYTwto5fmEkYSRhJGEy0jCdaPRb"
			"Ey7Iwkjl0EkYSR4tiZbRy2Jl2RhJHLoJIwkjl9/9iMJIwmXZGEkcuwkjCZbRy9okjCSMJl2RhJHL8RJGEoSRhM"
			"to5dhJGEy0jCdbojCSMJltHLsJIwmXuRhLy9loSR+FoSR+qj0WxMto5dBJGEkcticW0etsTLaOXQSRhJHLYmV/"
			"Z7LSMJ1tIwnW0jCeFpGE62kYTrR/ntCSPwvoEvL2WkYTi0jCdbojCSMJ1tIwnFpGE636kbyRhJGEy0jCZbR4ti"
			"cXyRhJGEkYTLSMJltHi+8S8kYSR4tCSPW0JI8WxMuyMJI8WhJHrfESRhHLYmXZGEkctiZaRhOLsjCSOWxMuyMJ"
			"I5bEy9yMJHffME4uiMJIwnmtIwmWkYTi6IwkjCea+PkJIwnhaRhMtIwmXyRhJGEkYTwtIwmXuRhLy9FoSR81oS"
			"R9VoSR81oSR4ry776fJGEy0jCdbSMJlpGE62kYTLSMJ1tIwmX6kbyRhJGEy7Iwkjl2EkYTLaOXtEkYSRhMuyMJ"
			"I5fiJIwlCSMJlpGEy0jCZaRhOt0RhJGEy0jCZaRhMvcjCXl7LQkjloSR81oSR8LQkjloSR81eXfXl7LSMJ8LSM"
			"J9VH890EkYSR/pvwEkYS8u++nyRhPVaRhPNaRhPVaRhMvj5IwkjxaEkcugkjCSPFsTLsjCSPFoSRy/UJIwkjCO"
			"WxMuyMJI5bEy0jCeFtHLYmXZGEkctiZe5GEj/daEkfhaEkffR67QkjloSRy0JI9bQkjl0EkYSRy0JI5aEket9A"
			"kd9HstIwnW0jCea0jCcWkYTraRhP5b7xGEkYTLsjCSOXYSRhMto5e0SRhJGEy7Iwkjl2EkYT/haRhMtIwmWkYT"
			"6rSMJlpGEy0jCfzWhJHLQkj1vkJIwkjCSPFoSRy0JI9b4iSNCeq0jCdbojCSMJ6rSMJxfSCSPqtCSPW6CSMJI+"
			"q0JI5R30ey0jCcWkYTrdEYSRhOtpGE4tIwnW2j+uj7rSMJlpGEy0jCZbRy2J4WkYTLSMJlpGE/32kYT4WkYTwv"
			"vEYSRhMuyMJI++2Jl0RhJGEy7Iwkj4XzROLojCSMJ5rSMJlpGE4uiMJIwnmvpBJHLQkj+m0JI5aEkfVR334iMJ"
			"IwkjCZbRy2Jl2RhJHL8wkjCSMJIwmW0ctiZbR/XR7LSMJlpGEy0jCdbaOWxOLSMJlpGEy0jCeNoSR+FoSR4o9F"
			"0RhJGEy7Iwkjl7RJGEkYTi6IwkjCZdkYSRy+IkjInW0jCcWkYTrdEYSRhOtpGE4tIwnF9IJI8XQSRhJHLQkjxb"
			"Ey7IwkjxdBJGEkctCSOUF++jvo+60jCeq2jlsTwtIwhflfeIwkjCZdkYSRy0JI5dhJGEy6IwkjCZdkYSRy0JI5"
			"fNE4tIwn8lpGE4tIwnqvpBJHLYmXZGEkcugkjCSOXQSRhJHLYmXZGEkcvwEkYSPuuiMJIwnF0RhJGE990RhJGE"
			"4uyMJI/otCSPFoSRy0JI+q0JI8WhJHKO+j2WkYTLSMJlpGE620cticWkYTLSMJlpGE/yo9FpGE998kYSRhJGE4"
			"tIwnvuyMJI/ro9FtHi9okjCSMJltHi2J1to8XtEkYSRhMto/xtIwnW0jCdbSMJ4WkYTraRhOt96MJQkjl0EkYS"
			"Ry0JI8XyEkYSRhJHLoJIwkjloSR4o/xtCSPwtCSP20d9H8LSMJlpGE/TaRhMto//l2EkYTi7Iwkjl0EkYSRy9o"
			"kjCSMJxdkYSRy/ASRhI778RGEkYSRhMto5bEy7Iwkjl+YSRhJGEkYTLaOWxMvijSMJlpGEy6IwkjCZbRy2JxaR"
			"hMtIwmXRGEkYTLaP8bQkjloSR62hJH32hJHLQkj1tCSOUF++jvo9lpGE62kYTLojCSMJ4WkYTraRhMtIwn66PZ"
			"fJGEkYSRhMvkjCSMJIwnW+SMJIwkjCZfqRhJGkYT1WkYTzWkYT1WkYTL4iMJI8WhJH6roJIwkjxaEkf0W0ctiZ"
			"dkYSRy2JlpGE4uyMJI5bEy7IwkjlsTLSMJ+uj13YSRhPC0jCZaRhMvkjCSMJIwnhaRhMvcjCR30ey6IwkjCZaR"
			"hPNaRhOLojCSMJlpGE/lvvEYSRhMuyMJI5dhJGEy2jl7RJGEkYTLsjCSOX4iSMJIkjCZaRhMtIwmWkYTrdEYSR"
			"hMtIwmWkYTL3Iwkey0JI5aEkfNdBJGEkeLQkjloSR81Bfvo76PZaRhOLSMJ1uiMJIwnW0jCcWkYTrbR/XfSIwm"
			"WkYTLSMJl2RhJHi2JlpGEy0jCZaRhMv1IwkjSMJxdEYSRhPqtIwnF0RhJGE8L4iMJI5dBJGEkfVdBJGEkcugkj"
			"CSPhfNE4uiMJIwnmtIwmWkYTi6IwkjCeajvo77/sEYSRhJGEkeLYmXZGEkcv/ISRhJGEkYSR4tiZbR/XfeIwkj"
			"CZaRhMtIwnW2jl7RJGEkYTLSMJlpGE/yvmI5aEkctCSPW9okjCSMJltHLQkjloSR63xEkZE4to5bE4tIwnW6Iw"
			"kjCcW0cticWkYTi+kEkeLoJIwkjloSR63QSRhJHi6CSMJI5aEkco/baRhPhaRhP0WhJHLQkj1tCSPFoSR4tCSO"
			"WhJHraEkeKCIiIv0Uey6IwkjCZaRhPNaRhOLojCSMJlpGEIiIi/noIiIv8LsJIwnF2RhJHLoJIwkjl7RJGEkYT"
			"i7Iwkjl+AkjCQRERF+igiIiL+62JltHrbE4tIwmXZGEkctiZbR62xOLSMJlBEREX6KCIiIv0X3iMJIwmXZGEkc"
			"tCSOXYSRhMuiMJIwmXZGEkctCSJERF/XQRERF+i+kRhMtIwmWkYTLsjCSPFsTLSMJlpGEy0jCZdkYSRIiIi/dQ"
			"RERF+igiIi+2giIi+2+aJxbRy9okjCSMJ1uiMJIwnFtHL2iSMJIwhF/7UkX+mkv33xRJGE4uyMJI5dBJGEkcva"
			"JIwkjCcXZGEkcuwkjCfxtCSPmtCSPqtCSPmtCSJf7LojCSMJl2RhJHLsJIwmW0cvaJIwkjCZdkYSRy/ASRhJL9"
			"9PotiZbRy9okjCSMJ1uyMJI5bEy2jl7RJGEkYQv/20jCcWkYTzWkYTraRhOLSMJ4Ul++n2WhJHwtiZbRy2JltH"
			"raEkfC2JltEv9lpGE8LSMJlpGEy0jCcWkYTwtIwmXuRhJL99PotiZaRhMvajCSMJI8WxMto5bEy0jCZe1GEkYS"
			"Ry+8RhJGEy7Iwkjl2EkYTLaOXtEkYSRhMuyMJI5fgJIwk/42kYT6rSMJ4WkYT6r6EZ9FsTLaOXQSRhJHLYnFtH"
			"rbEy2jl0EkYSRy2IX+uki/620ctCSOWhJHLYmWkYTi7IwkjloSRy0JI5bEy9yMJJfvvpRJHLYmXZGEkcugkjCS"
			"OXQSRhJHLYmXZGEkcuwkjCF9lPotiZdkYSRy6CSMJI8WxMto5bEy7Iwkjl0EkYSR/mtCSOXQSRhJHLQkjloSR6"
			"2hJHLoJIwkjloSR//7SMJ77ojCSMJ4WkYT334IwkjPotiZbRy6CSMJI5bE4to9bYmW0cugkjCSOWxC/13zRMvk"
			"jCSMJIwmW0eL2iSMJIwmXyRhJGEkYTLaJf/dJF/pvpRJHLQkjloSR62hJHLoJIwkjloSRy0JI9bYhfZT6LQkjl"
			"oSRy6CSMJI8WxMto5aEkctCSOXQSRhJHL4iMJI5dBJGEkctCSOWhJHLoJIwkjl0EkYSRy0JI5T/jaRhPffJGEk"
			"YSRhOLSMJ778EYSRkv6LSMJ8LSMJ5qSL/TT/VaEkS/wp768/fXd31/Hvr/u76/v9dcl41yXjXJeNcl41yXjXJe"
			"Ncl41yXjXJeNcl41/L665yt/XXPdb3c91vdz+et/XXPdb3c91bL+uuf/6ue63u57q39dc5W/rrnut7ue6t+yuS"
			"/51z3Vv91cl/51z3Vv9lcl9lcl9lcl9lcl9lcl9lcl9lcl9lcl9lcl9lcl9lckX7K3663663663662X8l5979d"
			"b9db9db9db9db9db9db9dbLxrfrru9db9db9db9dbLxrfprfprfprfFc5b3c91b7rfNvrXOW93PdfPdb7q5629"
			"3P5633Vz1t7ue6t+it/nrnK33VzxW/z1zl891v01v01v01v01v01v013emt8W+bfWucrfFvm391b9db9Nb9Nb9"
			"Nb9Nb9Nb9Nb9Nb9Nb9Nb9Nb7rfNv67fNvi3zb+u3zb+u3zb/nt7uf9Vb/0Xn3v013emt+mt+mt+mt+mt+mt+mt"
			"+mt+mt/Xb5t91vm3/db5t91vm3+6t/nrn/xrfsrfprfprfprfprfprZf57fNv6657q3lc91b+u3zb+uue6t5fP"
			"muS8a5LxrkvGuf81vdzxb3c+FvdyX9Ncl41yXjXPdW+6uetb7q54rfdXPWtl+muS8a5Lxrn+6t91c/21sv665L"
			"xrkvGuS8a5L7K5/zW93Pdb3c/3W93PdWy/TX8v+iu4v/muS+yuS+yuf9FbL+uue6t91c/Xb3c91b7q54rfdXP1"
			"293PdWy/+q5L7K5L7K59Nc+mv/f013ERflWyIv2Vvurni3u5/zVvurn+NbIi/6VvK57q3/bXPdWyIvVWyIvyrf"
			"dXP11vK54rf11z9dbyuf/iuS8a5L/wrnurf+quS8a5LxrkvGv5S8a5+u3u56293PW3u561v+iufz293P563lc/"
			"nt7uf41yXjXJeNcl41yXjXJf+lc91b7q57q3xXPdW8rnurfdXPdW/ZXJeNcl41yXjXJeNcl41z665Lxrn11yXj"
			"X8peNc+uuf7be7n/rXPvt7uS/KufXXJeNc+uuS8a59dcl41z665Lxrn11z/zrfdXP9tb7q5y8+9kX8K3+eucrf"
			"Fc91b/PXOXz3WyL/fW8rnurfdXPdW8vo5t3PdW8rnurfdXPdW/RWy+yt5XP11vurn663+eufrrfdXJF/Jb3c8W"
			"93PdW+6ueLe7ni3u57rz72Rf23n3si/hb3c91byue63u57q3lc91vdz3VvK57r/t3c1si/ZWyL+q3u59V597Ii"
			"IiIv9tbIiIiIi/rvPvZEX/Stl41svGu4vGt++ue6tl91bLxrZeNbLxrZeNbLxrZeNbLxrZeNc+m9f/qAAAAAAA"
			"AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA"
			"AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA"
			"AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA"
			"AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAfuA";
	}

	/**
	 *	@brief gets raw image data
	 *	@return Returns pointer to raw ARGB32 image data on success, 0 on failure.
	 */
	static inline uint32_t *p_Decompress()
	{
		return p_Decode_RLE_Huff_Base64(p_s_Base64_Data(), 113, 537, 22213, 1, 1, 6, 2, 2);
	}

protected:
	/**
	 *	@brief calculates reverse prediction on a single image plane
	 *
	 *	@param[in,out] p_buffer is pointer to image plane data (just the data, no padding or align)
	 *	@param[in] n_width is image width, in pixels
	 *	@param[in] n_height is image height, in pixels
	 *	@param[in] n_pred_type is predictor type (one of 0 = up, 1 = left, 2 = avg, 3 = paeth, 4 = med,
	 *		5 = gap, 6 = no prediction, 7 = up-left, 8 = avg2, 9 = diff, 10 = grad-h, 11 = grad-v)
	 */
	static void UnPredict_ImagePlane(uint8_t *p_buffer, int n_width, int n_height, int n_pred_type)
	{
		int n_first_row = (n_pred_type == 1 || n_pred_type == 6)? 0 : ((n_pred_type == 5)? 2 : 1);
		int n_first_col = (n_pred_type == 0 || n_pred_type == 6)? 0 : ((n_pred_type == 5)? 2 : 1);
		for(int x = 0; x < n_first_col; ++ x) {
			for(int y = n_first_row + 1; y < n_height; ++ y)
				p_buffer[x + n_width * y] += p_buffer[x + n_width * (y - 1)];
		}
		for(int y = 0; y < n_first_row; ++ y) {
			for(int x = n_first_col + 1; x < n_width; ++ x)
				p_buffer[x + n_width * y] += p_buffer[(x - 1) + n_width * y];
		}
		p_buffer += n_first_col + n_first_row * n_width;
		for(int y = n_first_row; y < n_height; ++ y, p_buffer += n_first_col) {
			for(int x = n_first_col; x < n_width; ++ x, ++ p_buffer) {
				if(n_pred_type == 0)
					*p_buffer += p_buffer[-n_width]; // up
				else if(n_pred_type == 1)
					*p_buffer += p_buffer[-1]; // left
				else if(n_pred_type == 2) {
					*p_buffer += (int(p_buffer[-1]) + int(p_buffer[-n_width]) +
						int(p_buffer[-1 - n_width])) / 3; // avg
				} else if(n_pred_type == 3) {
					int a = p_buffer[-1], b = p_buffer[-n_width], c = p_buffer[-1 - n_width];
					int p = a + b - c, pa = abs(p - a), pb = abs(p - b), pc = abs(p - c);
					*p_buffer += (pa <= pb && pa <= pc)? a : ((pb <= pc)? b : c); // paeth
				} else if(n_pred_type == 4 || (n_pred_type == 5 && x + 1 == n_width)) { // med
					int a = p_buffer[-1], b = p_buffer[-n_width], c = p_buffer[-1 - n_width];
					*p_buffer += (c >= max(a, b))? min(a, b) : (c <= min(a, b))? max(a, b) : a + b - c;
				} else if(n_pred_type == 5) { // gap
					_ASSERTE(x + 1 != n_width); // the last column predicted using med (gap looks right)
					int nne = p_buffer[1 - 2 * n_width], n = p_buffer[-n_width], w = p_buffer[-1],
						ww = p_buffer[-2], nn = p_buffer[-2 * n_width], ne = p_buffer[1 - n_width],
						nw = p_buffer[-1 - n_width], n_d = abs(w - nw) + abs(n - nn) +
						abs(ne - nne) - abs(w - ww) - abs(n - nw) - abs(ne - n), n_ad = abs(n_d);
					uint8_t f = (w + n) / 2 + (ne - nw) / 4, g = (n_d > 0)? w : n;
					*p_buffer += (n_ad > 80)? g : (n_ad > 32)? (f + g) / 2 : (n_ad > 8)? (3 * f + g) / 4 : f;
				} else if(n_pred_type == 7) // 6 = no prediction
					*p_buffer += p_buffer[-1 - n_width]; // up-left
				else if(n_pred_type == 8)
					*p_buffer += (int(p_buffer[-1]) + int(p_buffer[-n_width])) / 2; // avg2
				else if(n_pred_type == 9)
					*p_buffer += p_buffer[-1] + p_buffer[-n_width] - p_buffer[-1 - n_width]; // diff
				else if(n_pred_type == 10)
					*p_buffer += p_buffer[-1] + (p_buffer[-n_width] - p_buffer[-1 - n_width]) / 2; // grad-h
				else if(n_pred_type == 11)
					*p_buffer += p_buffer[-n_width] + (p_buffer[-1] - p_buffer[-1 - n_width]) / 2; // grad-v
			}
		}
	}

	/**
	 *	@brief decompresses the image
	 *
	 *	@param[in] p_s_base64 is null-terminated string containing BASE64 compressed image data
	 *	@param[in] n_image_width is image width, in pixels
	 *	@param[in] n_image_height is image height, in pixels
	 *	@param[in] n_huff_stage_size size of the compressed data before Huffman encoding
	 *	@param[in] n_predictor_y is predictor type for the luminance plane
	 *	@param[in] n_predictor_uv is predictor type for the chrominance plane
	 *	@param[in] n_predictor_a is predictor type for the alpha plane
	 *	@param[in] n_subsample is number of chrominance samples per 
	 *	@param[in] n_color_type is color type bitmask (if the first bit is set the image is
	 *		grayscale, if the second bit is set the image has alpha channel, if the third
	 *		bit is set, lossless compression was employed)
	 */
	static uint32_t *p_Decode_RLE_Huff_Base64(const char *p_s_base64,
		int n_image_width, int n_image_height, int n_huff_stage_size,
		int n_predictor_y, int n_predictor_uv, int n_predictor_a,
		int n_subsample, int n_color_type)
	{
		bool b_gray = (n_color_type & 1) != 0, b_alpha = (n_color_type & 2) != 0,
			b_lossless_color = (n_color_type & 4) != 0;
		size_t n_base64_length = strlen(p_s_base64);
		size_t n_base64_out_size = (n_base64_length / 4) * 3 +
			max(int(n_base64_length % 4) - 1, 0);
		size_t n_image_size = size_t(n_image_width) * n_image_height;
		int n_chroma_w = (n_image_width + n_subsample - 1) / n_subsample;
		int n_chroma_h = (n_image_height + n_subsample - 1) / n_subsample;
		size_t n_chroma_size = size_t(n_chroma_w) * n_chroma_h * 2;
		size_t n_image_data_size = n_image_size * ((b_alpha)? 2 : 1) +
			((b_gray)? 0 : n_chroma_size);

		size_t n_prehuff_size = max(n_base64_out_size, n_image_data_size);
		size_t n_temp_buffer_size = n_prehuff_size + n_huff_stage_size;
		uint8_t *p_temp_buffer;
		uint32_t *p_raster;
		if(!(p_temp_buffer = new(std::nothrow) uint8_t[n_temp_buffer_size]) ||
		   !(p_raster = new(std::nothrow) uint32_t[n_image_size])) {
			delete[] p_temp_buffer; // no effect on null
			return 0;
		}

		{
			int p_ic_tab[256];
			for(int i = 0; i < 256; ++ i) {
				p_ic_tab[i] = (isalpha(i))? (i | 32) - 'a' + ((i < 'a')? 0 : 26) :
					(isdigit(i))? i - '0' + 52 : (i == '+')? 62 : 63;
			}
			uint8_t *p_src = (uint8_t*)p_s_base64, *p_dst = p_temp_buffer;
			for(const uint8_t *p_end = p_src + n_base64_length - 3; p_src < p_end; p_src += 4, p_dst += 3) {
				uint8_t p_tmp[] = {uint8_t(p_ic_tab[p_src[1]]), uint8_t(p_ic_tab[p_src[2]])};
				*p_dst = (p_ic_tab[p_src[0]] << 2) | (p_tmp[0] >> 4);
				p_dst[1] = (p_tmp[0] << 4) | (p_tmp[1] >> 2);
				p_dst[2] = (p_tmp[1] << 6) | p_ic_tab[p_src[3]];
			}
			if(n_base64_length % 4 == 3) {
				uint8_t n_tmp = p_ic_tab[p_src[1]];
				*p_dst = (p_ic_tab[p_src[0]] << 2) | (n_tmp >> 4);
				p_dst[1] = (n_tmp << 4) | ((p_ic_tab[p_src[2]] >> 2) & 0xf); // fixed byte order
			} else if(n_base64_length % 4 == 2)
				*p_dst = (p_ic_tab[p_src[0]] << 2) | (p_ic_tab[p_src[1]] >> 4);
		}
		{
			const uint8_t *p_huff_src = p_temp_buffer, *p_huff_end = p_huff_src + n_base64_out_size;
			_ASSERTE(p_huff_src + 16 * sizeof(int32_t) <= p_huff_end);
			const int32_t *p_code_num = (const int32_t*)p_huff_src; p_huff_src += 16 * sizeof(int32_t);
			int p_min_code[16], p_max_code[16], p_table_off[16], n_code_num = 0;
			for(int i = 0, n_cw = 0; i < 16; ++ i, n_cw <<= 1) {
				p_min_code[i] = n_cw;
				p_max_code[i] = (n_cw += p_code_num[i]);
				p_table_off[i] = n_code_num - p_min_code[i];
				n_code_num += p_code_num[i];
			}
			_ASSERTE(p_huff_src + n_code_num <= p_huff_end);
			const uint8_t *p_symbol = p_huff_src; p_huff_src += n_code_num;
			uint8_t *p_huff_dest = p_temp_buffer + n_prehuff_size;
			for(int n_byte = 0, n_bit_num = 0, n_counter = n_huff_stage_size; n_counter --;) {
				for(int i = 0, n_code = 0;; ++ i) {
					_ASSERTE(i < 16);
					if(!(n_bit_num --)) {
						_ASSERTE(p_huff_src < p_huff_end);
						n_byte = *p_huff_src ++;
						n_bit_num = 7;
					}
					n_code = (n_code << 1) | ((n_byte >> 7) & 1);
					n_byte <<= 1;
					if(n_code >= p_min_code[i] && n_code < p_max_code[i]) {
						*p_huff_dest ++ = p_symbol[n_code + p_table_off[i]];
						break;
					}
				}
			}
		}
		{
			const uint8_t *p_rle_src = p_temp_buffer + n_prehuff_size,
				*p_src_end = p_rle_src + n_huff_stage_size;
			for(uint8_t *p_rle_dest = p_temp_buffer; p_rle_src != p_src_end;) {
				uint8_t n_code = *p_rle_src ++;
				bool b_compressed = n_code & 1;
				int n_run_length = (n_code >> 1) + 1;
				_ASSERTE(p_rle_src + ((b_compressed)? 1 : n_run_length) <= p_src_end);
				while(n_run_length --)
					*p_rle_dest ++ = (b_compressed)? *p_rle_src : *p_rle_src ++;
				if(b_compressed)
					++ p_rle_src;
			}
		}

		if(!b_gray) {
			uint8_t *p_y = p_temp_buffer, *p_u = p_y + n_image_size,
				*p_v = p_u + n_chroma_size / 2, *p_a = p_u + n_chroma_size;
			UnPredict_ImagePlane(p_y, n_image_width, n_image_height, n_predictor_y);
			UnPredict_ImagePlane(p_u, n_chroma_w, n_chroma_h, n_predictor_uv);
			UnPredict_ImagePlane(p_v, n_chroma_w, n_chroma_h, n_predictor_uv);
			if(b_alpha)
				UnPredict_ImagePlane(p_a, n_image_width, n_image_height, n_predictor_a);
			uint32_t *p_dest = p_raster;
			for(int y = 0, h = n_image_height; y < h; ++ y) {
				for(int x = 0, w = n_image_width; x < w; ++ x, ++ p_y, ++ p_dest) {
					size_t n_cr_idx = x / n_subsample + size_t(y / n_subsample) * n_chroma_w;
					uint8_t r, g, b;
					if(!b_lossless_color) {
						int Y = *p_y, u = p_u[n_cr_idx] - 128, v = p_v[n_cr_idx] - 128;
						r = uint8_t(max(0, min(255, Y + 1.402f * v)));
						g = uint8_t(max(0, min(255, Y - 0.34414f * u - 0.71414f * v)));
						b = uint8_t(max(0, min(255, Y + 1.772f * u))); // JPEG-like YUV
					} else {
						int8_t u = p_u[n_cr_idx], v = p_v[n_cr_idx];
						g = *p_y - (u >> 1);
						uint8_t s = g + u;
						r = s - (v >> 1);
						b = r + v; // lifting
					}
					if(b_alpha) {
						*p_dest = (*p_a << 24) | (b << 16) | (g << 8) | r;
						++ p_a;
					} else
						*p_dest = 0xff000000U | (b << 16) | (g << 8) | r;
				}
			}
		} else {
			uint8_t *p_a = p_temp_buffer + n_image_size;
			uint32_t *p_dest = p_raster;
			UnPredict_ImagePlane(p_temp_buffer, n_image_width, n_image_height, n_predictor_y);
			if(b_alpha) {
				UnPredict_ImagePlane(p_a, n_image_width, n_image_height, n_predictor_a);
				for(const uint8_t *p_y = p_temp_buffer,
				   *p_end = p_y + n_image_size; p_y != p_end; ++ p_y, ++ p_a)
					*p_dest ++ = (*p_a << 24) | (*p_y << 16) | (*p_y << 8) | *p_y; // gray / alpha
			} else {
				for(const uint8_t *p_y = p_temp_buffer,
				   *p_end = p_y + n_image_size; p_y != p_end; ++ p_y)
					*p_dest ++ = 0xff000000U | (*p_y << 16) | (*p_y << 8) | *p_y; // gray
			}
		}

		delete[] p_temp_buffer;

		return p_raster;
	}
};

#endif // __GRADIENTR_TGA_INCLUDED
