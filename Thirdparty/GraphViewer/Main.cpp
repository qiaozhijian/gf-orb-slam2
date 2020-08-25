/*
								+----------------------------------+
								|                                  |
								|  ***  ÜberLame GLUT window  ***  |
								|                                  |
								|   Copyright © -tHE SWINe- 2009   |
								|                                  |
								|             Main.cpp             |
								|                                  |
								+----------------------------------+
*/

#ifdef _MSC_VER
#pragma warning(disable: 4503)
#endif
#include "../UberLame_src/NewFix.h"
#include "../UberLame_src/CallStack.h"
#include "../UberLame_src/gles2/GLES20Emu.h"
typedef double GLdouble; // needed in glu.h, included through glut.h
#ifdef __APPLE__
#include <GLUT/glut.h>
#else // __APPLE__
#include <GL/glut.h>
#endif // __APPLE__
#ifndef GL_RGBA32F
#define GL_RGBA32F                        0x8814 // need GL_ARB_texture_float
#endif // !GL_RGBA32F
#include <math.h>
#include <time.h>
#include <stdio.h>
#include <string.h>
#include <vector>
#include <string>
#include <algorithm>
#include <map>
#include "../UberLame_src/Vector.h"
#include "../UberLame_src/StlUtils.h"
#include "../UberLame_src/Segregated.h"
#include "../UberLame_src/gles2/Texture.h"
#include "../UberLame_src/gles2/BufferObjects.h"
#include "../UberLame_src/gles2/Shader.h"
#include "../UberLame_src/gles2/TextureUtil.h"
#include "../UberLame_src/gles2/FrameBuffer.h"
#include "../UberLame_src/gles2/Transform.h"
#include "../UberLame_src/Integer.h"
#include "../UberLame_src/Dir.h"
#include "ColorId.h"
#include "Vertex.h"
#include "Grid.h"
#include "VertexArrays.h"
#include "Gradient.h"
#include "Animate.h"
#include "FileList.h"
#include "MegaTexture.h" // before shaders
#include "Shaders.h"
#include "FastIO.h"

#include "eigen/Eigen/Core"
#include "eigen/Eigen/Eigenvalues"

/**
 *	@brief three-dimensional euclidean integer vector
 */
typedef Vector3<uint8_t> Vector3ub;

#pragma pack(1)

struct TVertex_C4UB_V3F {
	uint8_t p_color[4];
	Vector3f v_pos;
};

#pragma pack()

struct TCommandlineArgs {
	const char *p_s_window_title; /**< @brief window title */
	float f_grid_spacing; /**< @brief grid line spacing, in world units */
	float f_movement_speed; /**< @brief movement speed, in world units; this is further divided when using modifier keys */
	float f_camera_size; /**< @brief camera pyramid size */
	float f_camera_fov_degrees; /**< @brief camera pyramid (horizontal) angle, in degrees */
	float f_camera_aspect; /**< @brief camera pyramid base aspect ratio (width:height) */
	float f_principal_camera_fov_degrees; /**< @brief camera (horizontal) field of view, in degrees */
	float f_near_view_distance; /**< @brief near plane distance for OpenGL rendering */
	float f_far_view_distance; /**< @brief far plane distance for OpenGL rendering */
	const char *p_s_animation_dest; /**< @brief animation destination format string */
	const char *p_s_screenshot_dest; /**< @brief screenshot destination format string */
	bool b_screenshot_follows_window; /** @brief if set, then the screenshot framing is the same as window framing */
	float f_video_fps; /**< @brief animation framerate */
	int n_video_width; /**< @brief animation horizontal resolution, in pixels */
	int n_video_height; /**< @brief animation vertical resolution, in pixels */
	bool b_video_omni; /**< @brief omnidirectional mode flag for the video */
	bool b_covs_scale_all; /**< @brief use all covariances for calculating the scale (true) or use only the last frame of the animation (false) */
	bool b_covs_scale_inverse; /**< @brief invert the false-color values */
	bool b_covs_log_scale; /**< @brief use logarithmic scale for false colours palette */
	bool b_covs_scale_exact; /**< @brief exact covariances scale (true) or nice covariance scale (false) */
	bool b_covs_scale_soft_min; /**< @brief soft min bound of covariance scale (true) or use zero (false) */
	float f_covs_scale_offset; /**< @brief covariance scale constant offset */
	float f_covs_scale_factor; /**< @brief covariance scale mixing factor */
	bool b_anim_color_blend; /**< @brief blend in new / blend out lost vertices using a different colour */
	bool b_selection_enabled; /**< @brief selection mode enable flag (can be disabled from commandline) */
	int n_mouse_x_polarity; /**< @brief mouse x-axis sign (default +1) */
	int n_mouse_y_polarity; /**< @brief mouse y-axis sign (default +1) */
	bool b_quality_assessment; /**< @brief quality assessment flag */
	float f_qa_tile_size; /**< @brief quality assessment tile size */
	float f_qa_density_weight; /**< @brief quality assessment density vs. smoothness weight */
	Matrix3f t_colorize_image_matrix; /**< @brief matrix used in calculating flat vertex colors (maps observation coordinates to pixel coordinates) */
	const char *p_s_colorize_input_images; /**< @brief images for calculating flat vertex colors (does not influence the megatexture) */
	int n_colorize_points_fwidth; /**< @brief median filter width used in calculating flat vertex colors */
	const char *p_s_camera_proj_input_images; /**< @brief images used for camera insets and for projective texturing of the point cloud */
	Matrix3f t_proj_colorize_image_matrix; /**< @brief matrix used in projective vertex colors (maps reprojected point coordinates to pixel coordinates) */
	int n_original_images_width; /**< @brief width of the images that were used to generate the graph (size of the images the observations in the graph exist in) */
	int n_original_images_height; /**< @brief height of the images that were used to generate the graph (size of the images the observations in the graph exist in) */
	bool b_auto_view;
	bool b_immed_screenshot;
	bool b_immed_animation;
	const char *p_s_coff_export_file;
	const char *p_s_vertex_colors_export_file;
	size_t n_texture_streaming_cache_MB;
	size_t n_texture_streaming_thread_num;

	Vector3f v_camera_pos;
	float f_angle_x;
	float f_angle_y;
	float f_angle_z;
	int n_width;
	int n_height;
	bool b_display_covs; /**< @brief false-colour precision display flag */
	bool b_fancy_color_points; /**< @brief projective textured points display flag */
	bool b_display_edges; /**< @brief edges display flag */
	bool b_display_cameras; /**< @brief camera frusta display flag */
	bool b_edges_blend_add; /**< @brief edges additive blending flag */
	int n_background; /**< @brief background colour index */
	bool b_upside_down; /**< @brief upside down flip flag */
	bool b_display_grid; /**< @brief grid display flag */
	float f_point_size; /**< @brief 3D point size */
	float f_edge_alpha; /**< @brief edge alpha */
	float f_camera_tex_alpha; /**< @brief camera texture alpha */
	// display settings

	const char *p_s_result_file;
	const char *p_s_graph_file;
	const char *p_s_margs_file;
	const char *p_s_vertex_colors_file;

	TCommandlineArgs()
	{
		Defaults();
	}

	void Defaults()
	{
		p_s_window_title = "GraphViewer";
		f_grid_spacing = 1;
		f_movement_speed = 5;
		f_camera_size = .1f;
		f_camera_fov_degrees = 60;
		f_camera_aspect = 1.0;
		f_principal_camera_fov_degrees = 60;
		f_near_view_distance = .01f;
		f_far_view_distance = 2000;
		p_s_animation_dest = "frame_#####.png";
		p_s_screenshot_dest = "screenshot_###.png";
		b_screenshot_follows_window = false;
		f_video_fps = 48;
		n_video_width = 1280;
		n_video_height = 720;
		b_video_omni = false;
		b_covs_scale_all = false;
		b_covs_scale_inverse = false;
		b_covs_log_scale = false;
		b_covs_scale_exact = false;
		b_covs_scale_soft_min = false;
		f_covs_scale_offset = 0;//480.531;
		f_covs_scale_factor = 0;//.25f;
		b_anim_color_blend = true;
		b_selection_enabled = true;
		n_mouse_x_polarity = 1;
		n_mouse_y_polarity = 1;
		b_quality_assessment = false;
		f_qa_tile_size = .1f;
		f_qa_density_weight = 0;
		t_colorize_image_matrix.Identity();
		p_s_colorize_input_images = 0;
		n_colorize_points_fwidth = 3;
		p_s_camera_proj_input_images = 0;
		t_proj_colorize_image_matrix.Identity();
		n_original_images_width = 4000;
		n_original_images_height = 3000;
		b_auto_view = false;
		b_immed_screenshot = false;
		b_immed_animation = false;
		p_s_coff_export_file = 0;
		p_s_vertex_colors_export_file = 0;

		n_texture_streaming_cache_MB = 800;
		n_texture_streaming_thread_num = max(size_t(2), CThread::n_CPU_Num()) - 1; // note that -2 may lead to better realtime streaming performance as one thread is taken by rendering, one by the driver and the rest would be free to load the images

		p_s_result_file = 0;
		p_s_graph_file = 0;
		p_s_margs_file = 0;
		p_s_vertex_colors_file = 0;

		b_auto_view = false;
		b_immed_screenshot = false;
		b_immed_animation = false;
		p_s_coff_export_file;
		p_s_vertex_colors_export_file;

		/*v_camera_pos = Vector3f(-15.0001, -0.000786595, 14.9034);
		f_angle_x = 1.36333; f_angle_y = -9.34746; f_angle_z = 0;*/ // for testing grid near-plane clipping
		v_camera_pos = Vector3f(-15.5054f, -18.8989f, 25.0036f);
		f_angle_x = 0.326667f;
		f_angle_y = 3.141592f;
		f_angle_z = 0;
		n_width = 800;
		n_height = 600;
		b_display_covs = false;
		b_fancy_color_points = false;
		b_display_edges = false;
		b_display_cameras = true;
		b_edges_blend_add = true;
		n_background = 0;
		b_upside_down = true;
		b_display_grid = true;
		f_point_size = 2;
		f_edge_alpha = 1.0f / 128;
		f_camera_tex_alpha = .5f;
		// display settings
	}

	bool Parse(int n_arg_num, const char **p_arg_list)
	{
		bool b_old_style_arg_list = true;
		for(int i = 1; i < n_arg_num; ++ i) {
			if(*p_arg_list[i] == '-') {
				b_old_style_arg_list = false;
				break;
			}
		}
		// detect old style argument list

		if(b_old_style_arg_list) {
			if(n_arg_num > 1)
				p_s_result_file = p_arg_list[1];
			if(n_arg_num > 2)
				p_s_graph_file = p_arg_list[2];
			if(n_arg_num > 3)
				p_s_margs_file = p_arg_list[3];
			if(n_arg_num > 4)
				p_s_animation_dest = p_arg_list[4];
			if(n_arg_num > 5)
				p_s_coff_export_file = p_arg_list[5];
			// legacy style commandline
		} else {
			for(int i = 1; i < n_arg_num; ++ i) {
				if(!strcmp(p_arg_list[i], "--help") || !strcmp(p_arg_list[i], "-h")) {
					PrintHelp();
					return false;
				} else if(!strcmp(p_arg_list[i], "--write-animation") || !strcmp(p_arg_list[i], "-wa"))
					b_immed_animation = true;
				else if(!strcmp(p_arg_list[i], "--take-screenshot") || !strcmp(p_arg_list[i], "-ts"))
					b_immed_screenshot = true;
				else if(!strcmp(p_arg_list[i], "--screenshot-follows-window") || !strcmp(p_arg_list[i], "-sfw"))
					b_screenshot_follows_window = true;
				else if(!strcmp(p_arg_list[i], "--omni-video") || !strcmp(p_arg_list[i], "-ov"))
					b_video_omni = true;
				else if(!strcmp(p_arg_list[i], "--hide-grid") || !strcmp(p_arg_list[i], "-hg"))
					b_display_grid = false;
				else if(!strcmp(p_arg_list[i], "--display-edges") || !strcmp(p_arg_list[i], "-de"))
					b_display_edges = true;
				else if(!strcmp(p_arg_list[i], "--flat-edges") || !strcmp(p_arg_list[i], "-fe"))
					b_edges_blend_add = false;
				else if(!strcmp(p_arg_list[i], "--hide-cameras") || !strcmp(p_arg_list[i], "-hc"))
					b_display_cameras = false;
				else if(!strcmp(p_arg_list[i], "--bright-background") || !strcmp(p_arg_list[i], "-bb"))
					n_background = 1;
				else if(!strcmp(p_arg_list[i], "--brighter-background") || !strcmp(p_arg_list[i], "-bb2"))
					n_background = 2;
				else if(!strcmp(p_arg_list[i], "--brightest-background") || !strcmp(p_arg_list[i], "-bb3"))
					n_background = 3;
				else if(!strcmp(p_arg_list[i], "--covs-scale-inverse") || !strcmp(p_arg_list[i], "-csi"))
					b_covs_scale_inverse = true;
				else if(!strcmp(p_arg_list[i], "--covs-scale-all") || !strcmp(p_arg_list[i], "-csa"))
					b_covs_scale_all = true;
				else if(!strcmp(p_arg_list[i], "--covs-log-scale") || !strcmp(p_arg_list[i], "-cls"))
					b_covs_log_scale = true;
				else if(!strcmp(p_arg_list[i], "--covs-scale-exact") || !strcmp(p_arg_list[i], "-cse"))
					b_covs_scale_exact = true;
				else if(!strcmp(p_arg_list[i], "--covs-scale-soft-min") || !strcmp(p_arg_list[i], "-cssm"))
					b_covs_scale_soft_min = true;
				else if(!strcmp(p_arg_list[i], "--no-color-blend") || !strcmp(p_arg_list[i], "-ncb"))
					b_anim_color_blend = false;
				else if(!strcmp(p_arg_list[i], "--upside-down") || !strcmp(p_arg_list[i], "-ud"))
					b_upside_down = !b_upside_down;
				else if(!strcmp(p_arg_list[i], "--disable-selection") || !strcmp(p_arg_list[i], "-ds"))
					b_selection_enabled = false;
				else if(!strcmp(p_arg_list[i], "--invert-mouse-x") || !strcmp(p_arg_list[i], "-imx"))
					n_mouse_x_polarity = -1;
				else if(!strcmp(p_arg_list[i], "--invert-mouse-y") || !strcmp(p_arg_list[i], "-imy"))
					n_mouse_x_polarity = -1;
				else if(!strcmp(p_arg_list[i], "--auto-view") || !strcmp(p_arg_list[i], "-av"))
					b_auto_view = true;
				else if(i + 1 == n_arg_num) {
					fprintf(stderr, "error: argument \'%s\': missing value or an unknown argument\n", p_arg_list[i]);
					return false;
				} else if(!strcmp(p_arg_list[i], "--covs-scale-offset") || !strcmp(p_arg_list[i], "-cso")) {
					if(i + 2 >= n_arg_num) {
						fprintf(stderr, "error: argument \'%s\': not enough values, use: "
							"<scale-offset> <scale-factor>\n", p_arg_list[i]);
						return false;
					}
					f_covs_scale_offset = float(atof(p_arg_list[i + 1]));
					f_covs_scale_factor = float(atof(p_arg_list[i + 2]));
					i += 1; // and ++ in the for loop
				}  else if(!strcmp(p_arg_list[i], "--colorize") || !strcmp(p_arg_list[i], "-co")) {
					if(i + 3 >= n_arg_num) {
						fprintf(stderr, "error: argument \'%s\': not enough values, use: "
							"<image-src> <3x3-raster-matrix> <sampling-filter-width>\n", p_arg_list[i]);
						return false;
					}
					p_s_colorize_input_images = p_arg_list[i + 1];
					const char *p_s_matrix = p_arg_list[i + 2];
					Matrix3f &mat = t_colorize_image_matrix; // rename
					if(sscanf(p_s_matrix, "[%f,%f,%f;%f,%f,%f;%f,%f,%f]", &mat[0][0], &mat[0][1],
					   &mat[0][2], &mat[1][0], &mat[1][1], &mat[1][2], &mat[2][0], &mat[2][1], &mat[2][2]) != 9) {
						fprintf(stderr, "error: image matrix must be given as \'[m00,m01,m02;m10,m11,m12;m20,m21,m22]\'\n");
						return false;
					}
					n_colorize_points_fwidth = atol(p_arg_list[i + 3]);
					i += 3; // and ++ in the for loop
				} else if(!strcmp(p_arg_list[i], "--project-camera-images") || !strcmp(p_arg_list[i], "-prci")) {
					if(i + 4 >= n_arg_num) {
						fprintf(stderr, "error: argument \'%s\': not enough values, use: "
							"<image-src> <3x3-raster-matrix> <image-width> <image-height>\n", p_arg_list[i]);
						return false;
					}
					if(p_s_camera_proj_input_images && strcmp(p_s_camera_proj_input_images, p_arg_list[i + 1]))
						fprintf(stderr, "warning: --inset-camera-images changes the image list\n");
					p_s_camera_proj_input_images = p_arg_list[i + 1];
					const char *p_s_matrix = p_arg_list[i + 2];
					Matrix3f &mat = t_proj_colorize_image_matrix; // rename
					if(sscanf(p_s_matrix, "[%f,%f,%f;%f,%f,%f;%f,%f,%f]", &mat[0][0], &mat[0][1],
					   &mat[0][2], &mat[1][0], &mat[1][1], &mat[1][2], &mat[2][0], &mat[2][1], &mat[2][2]) != 9) {
						fprintf(stderr, "error: image matrix must be given as \'[m00,m01,m02;m10,m11,m12;m20,m21,m22]\'\n");
						return false;
					}
					if(mat[0][1] != 0 || mat[1][0] != 0 || mat[0][2] != 0 || mat[1][2] != 0 || mat[2][2] != 1.0f)
						fprintf(stderr, "warning: the projective coloring raster matrix has nonzeros in unexpected places; this might not work\n");
					n_original_images_width = atol(p_arg_list[i + 3]);
					n_original_images_height = atol(p_arg_list[i + 4]);
					i += 4; // and ++ in the for loop
					b_fancy_color_points = true;
				} else if(!strcmp(p_arg_list[i], "--inset-camera-images") || !strcmp(p_arg_list[i], "-ici")) {
					if(p_s_camera_proj_input_images && strcmp(p_s_camera_proj_input_images, p_arg_list[i + 1]))
						fprintf(stderr, "warning: --inset-camera-images changes the image list\n");
					p_s_camera_proj_input_images = p_arg_list[++ i];
				} else if(!strcmp(p_arg_list[i], "--camera") || !strcmp(p_arg_list[i], "-cp")) {
					std::string s_campos = p_arg_list[++ i]; // throws
					std::vector<std::string> field_list;
					stl_ut::Split(field_list, s_campos, ":", 0);
					if(field_list.size() != 6) { // could add a token prefix which would set format (but currently there is only conversion from xyz-pyr to xyz-quat but not vice versa)
						fprintf(stderr, "error: camera pose must be given as \'x:y:z:pitch:yaw:roll\'\n");
						return false;
					}
					v_camera_pos.x = float(atof(field_list[0].c_str()));
					v_camera_pos.y = float(atof(field_list[1].c_str()));
					v_camera_pos.z = float(atof(field_list[2].c_str()));
					f_angle_x = float(atof(field_list[3].c_str()));
					f_angle_y = float(atof(field_list[4].c_str()));
					f_angle_z = float(atof(field_list[5].c_str()));
				} else if(!strcmp(p_arg_list[i], "--window-title") || !strcmp(p_arg_list[i], "-wt"))
					p_s_window_title = p_arg_list[++ i];
				else if(!strcmp(p_arg_list[i], "--video-dest") || !strcmp(p_arg_list[i], "-vd"))
					p_s_animation_dest = p_arg_list[++ i];
				else if(!strcmp(p_arg_list[i], "--screenshot-dest") || !strcmp(p_arg_list[i], "-sd"))
					p_s_screenshot_dest = p_arg_list[++ i];
				else if(!strcmp(p_arg_list[i], "--movement-speed") || !strcmp(p_arg_list[i], "-ms"))
					f_movement_speed = float(atof(p_arg_list[++ i]));
				else if(!strcmp(p_arg_list[i], "--grid-spacing") || !strcmp(p_arg_list[i], "-gs"))
					f_grid_spacing = float(atof(p_arg_list[++ i]));
				else if(!strcmp(p_arg_list[i], "--camera-size") || !strcmp(p_arg_list[i], "-cs"))
					f_camera_size = float(atof(p_arg_list[++ i]));
				else if(!strcmp(p_arg_list[i], "--camera-fov") || !strcmp(p_arg_list[i], "-cf"))
					f_camera_fov_degrees = float(atof(p_arg_list[++ i]));
				else if(!strcmp(p_arg_list[i], "--camera-aspect") || !strcmp(p_arg_list[i], "-ca")) {
					float f_width, f_height;
					if(sscanf(p_arg_list[++ i], "%f:%f", &f_width, &f_height) != 2) {
						fprintf(stderr, "error: bad aspect \'%s\': please enter aspect as width:height\n",
							p_arg_list[i]);
						return false;
					}
					f_camera_aspect = f_width / f_height;
				} else if(!strcmp(p_arg_list[i], "--gl-near-distance") || !strcmp(p_arg_list[i], "-gln"))
					f_near_view_distance = float(atof(p_arg_list[++ i]));
				else if(!strcmp(p_arg_list[i], "--gl-far-distance") || !strcmp(p_arg_list[i], "-glf"))
					f_far_view_distance = float(atof(p_arg_list[++ i]));
				else if(!strcmp(p_arg_list[i], "--gl-field-of-view") || !strcmp(p_arg_list[i], "-glfov"))
					f_principal_camera_fov_degrees = float(atof(p_arg_list[++ i]));
				else if(!strcmp(p_arg_list[i], "--video-fps") || !strcmp(p_arg_list[i], "-vf"))
					f_video_fps = float(atof(p_arg_list[++ i]));
				else if(!strcmp(p_arg_list[i], "--video-width") || !strcmp(p_arg_list[i], "-vw"))
					n_video_width = atol(p_arg_list[++ i]);
				else if(!strcmp(p_arg_list[i], "--video-height") || !strcmp(p_arg_list[i], "-vh"))
					n_video_height = atol(p_arg_list[++ i]);
				else if(!strcmp(p_arg_list[i], "--window-width") || !strcmp(p_arg_list[i], "-ww"))
					n_width = atol(p_arg_list[++ i]);
				else if(!strcmp(p_arg_list[i], "--window-height") || !strcmp(p_arg_list[i], "-wh"))
					n_height = atol(p_arg_list[++ i]);
				else if(!strcmp(p_arg_list[i], "--point-size") || !strcmp(p_arg_list[i], "-ps"))
					f_point_size = float(atof(p_arg_list[++ i]));
				else if(!strcmp(p_arg_list[i], "--edge-alpha") || !strcmp(p_arg_list[i], "-ea"))
					f_edge_alpha = float(atof(p_arg_list[++ i]));
				else if(!strcmp(p_arg_list[i], "--camera-texture-alpha") || !strcmp(p_arg_list[i], "-cta"))
					f_camera_tex_alpha = float(atof(p_arg_list[++ i]));
				else if(!strcmp(p_arg_list[i], "--result-file") || !strcmp(p_arg_list[i], "-i"))
					p_s_result_file = p_arg_list[++ i];
				else if(!strcmp(p_arg_list[i], "--graph-file") || !strcmp(p_arg_list[i], "-g"))
					p_s_graph_file = p_arg_list[++ i];
				else if(!strcmp(p_arg_list[i], "--marginals-file") || !strcmp(p_arg_list[i], "-m"))
					p_s_margs_file = p_arg_list[++ i];
				else if(!strcmp(p_arg_list[i], "--animation-dest") || !strcmp(p_arg_list[i], "-a"))
					p_s_animation_dest = p_arg_list[++ i];
				else if(!strcmp(p_arg_list[i], "--screenshot-dest") || !strcmp(p_arg_list[i], "-s"))
					p_s_screenshot_dest = p_arg_list[++ i];
				else if(!strcmp(p_arg_list[i], "--export-coff") || !strcmp(p_arg_list[i], "-xc"))
					p_s_coff_export_file = p_arg_list[++ i];
				else if(!strcmp(p_arg_list[i], "--save-vertex-colors") || !strcmp(p_arg_list[i], "-svc"))
					p_s_vertex_colors_export_file = p_arg_list[++ i];
				else if(!strcmp(p_arg_list[i], "--load-vertex-colors") || !strcmp(p_arg_list[i], "-lvc"))
					p_s_vertex_colors_file = p_arg_list[++ i];
				else if(!strcmp(p_arg_list[i], "--quality-assessment") || !strcmp(p_arg_list[i], "-qa")) {
					if(i + 2 >= n_arg_num) { // note that i + 2 can only equal as i + 1 == n_arg_num is checked above
						fprintf(stderr, "error: argument \'%s\': missing value or an unknown argument\n", p_arg_list[i]);
						return false;
					}
					b_quality_assessment = true;
					f_qa_tile_size = atof(p_arg_list[++ i]);
					f_qa_density_weight = atof(p_arg_list[++ i]);
				} else {
					fprintf(stderr, "error: argument \'%s\': an unknown argument\n", p_arg_list[i]);
					return false;
				}
			}
			// "parse" cmdline
		}

		if(b_video_omni) {
			/*if(fabs(f_principal_camera_fov_degrees - 90) > 1e-5)
				fprintf(stderr, "error: principal camera field of view must be 90 for omnidirectional rendering\n");*/
			// not needed anymore

			if(fabs(double(n_video_width) / n_video_height - 2) > 1e-5) {
				fprintf(stderr, "warning: aspect ratio for omnidirectional rendering"
					" should be 2:1 (use e.g. -vw 7168 -vh 3584)\n");
			}
		}

		return true;
	}

	static void PrintHelp()
	{
		printf("use either: graphview <result-file> <graph-file> <marginals-file> <animation-dest> <coff-export>\n"
			"or use: graphview [options]\n\n"
			"where options are:\n"
			"--help | -h - displays this help screen\n"
			"--result-file | -i <result-file> - specifies the point cloud file (or a pattern for animated)\n"
			"--graph-file | -g <graph-file> - specifies the point cloud file (or a pattern for animated)\n"
			"--marginals-file | -m - specifies the marginals covariances file (or a pattern for animated)\n"
			"--animation-dest | -a - specifies the animation pattern (default frame_#####.png)\n"
			"--screenshot-dest | -s - specifies the screenshot pattern (default screenshot_###.png)\n"
			"--write-animation | -wa - writes animation and exits\n"
			"--take-screenshot | -ts - takes a screenshot and exits\n"
			"--window-title | -wt <title> - window title\n"
			"--grid-spacing | -gs <spacing> - grid line spacing, in world units\n"
			"--movement-speed | -ms <speed> - movement speed, in world units; this is further divided\n"
			"                                 when using modifier keys\n"
			"--camera-size | -cs <size> - camera pyramid size (default 0.1)\n"
			"--camera-fov | -cf <fov> - horizontal camera field of view, in degrees (default 60)\n"
			"--camera-aspect | -ca <aspect> - camera aspect given as \'w:h\' (default 1:1)\n"
			"--gl-near-distance | -gln <dist> - near plane distance for OpenGL rendering\n"
			"--gl-far-distance | -glf <dist> - far plane distance for OpenGL rendering\n"
			"--gl-field-of-view | -glfov <degrees> - field of view for OpenGL rendering\n"
			"--video-dest | -vd <path> - animation destination format string\n"
			"--screenshot-dest | -sd <path> - screenshot destination format string\n"
			"--video-fps | -vf <fps> - animation framerate (default 48 FPS)\n"
			"--video-width | -vw <size> - animation horizontal resolution, in pixels (default 1280)\n"
			"--video-height | -vh <size> - animation vertical resolution, in pixels (default 720)\n"
			"--screenshot-follows-window | -sfw - screenshot aspect ratio matches that of the viewport\n"
			"                                     (disabled by default; absolute resolution still derived\n"
			"                                     from -vw and -vh)\n"
			"--omni-video | -ov - enables lon-lat omnidirectional mode rendering (make sure to set aspect\n"
			"                     ratio to 2:1, use e.g. \'-vw 7168 -vh 3584\')\n"
			"--camera | -cp <pose-spec> - set camera pose (format \'x:y:z:pitch:yaw:roll\')\n"
			"--auto-view | -av - automatically centers the view on 99%% of the points (this can be used in\n"
			"                    conjunction with -cp to set the camera rotation which is not changed by -av)\n"
			"--window-width | -ww <size> - window width, in pixels\n"
			"--window-height | -wh <size> - window height, in pixels\n"
			"--display-edges | -de - shows edges (default off)\n"
			"--hide-cameras | -hc - hide camera frusta (default not hidden)\n"
			"--flat-edges | -fe - disables edges additive blending (default enabled)\n"
			"--bright-background | -bb - changes background colour (default black)\n"
			"--upside-down | -ud - sets upside down flip (default off)\n"
			"--hide-grid | -hg - hides grid (default shown)\n"
			"--point-size | -ps <size> - 3D point size (default 2.0)\n"
			"--edge-alpha | -ea <alpha> - edge alpha (default 1/128.0)\n"
			"--covs-scale-inverse | -csi - for false color covariance display - invert the values\n"
			"--covs-scale-all | -csa - for false color covariance display - will use all frames\n"
			"                          to determine min / max (default use only the last frame)\n"
			"--covs-log-scale | -cls - for false color covariance display - will use logarithmic\n"
			"                          scale for the false color palette (default linear scale)\n"
			"--covs-scale-exact | -cse - for false color covariance display - will use exact min / max\n"
			"                            scale (defaul damped soft scale)\n"
			"--no-color-blend | -ncb - for false color covariance display - will disable blending\n"
			"                          points using a different color in animations (default enabled)\n"
			"--disable-selection | -ds - disables selection of points by mouse (default enabled)\n"
			"--invert-mouse-x | -imx - inverts mouse x axis (default disabled)\n"
			"--invert-mouse-y | -imy - inverts mouse y axis (default disabled)\n"
			"--quality-assurance | -qa <cell-size> <density-weight> - runs point cloud quality analysis,\n"
			"                          with <cell-size> being a sample size (0.5 might be a good start)\n"
			"                          and <density-weight> does linear interpolation between noise error (0)\n"
			"                          to inverse density (1), 0.5 might be a good start\n"
			"--export-coff | -xc <file.coff> - saves a colored point cloud in the COFF format (either vertex\n"
			"                                  colors or false-color from covariances or vertex degrees,\n"
			"                                  priorized in this order)\n"
			"--save-vertex-colors | -svc <file.ppm> - saves vertex colors as PPM image (one image pixel per each\n"
			"                                         vertex, in scanline order and padded so as to form a nearly\n"
			"                                         square image)\n"
			"--load-vertex-colors | -lvc <file.ppm> - loads vertex colors from a PPM image\n"
			"--colorize | -co <image-src> <3x3-raster-matrix> <filter-width> - calculates vertex colors based\n"
			"    on sampleing the original BA / SfM images at the observation coordinates from the graph\n"
			"--project-camera-images | -prci <image-src> <3x3-raster-matrix> <image-width> <image-height> -\n"
			"    project the original BA / SfM images onto the point cloud (requires megatexture support and\n"
			"    a graph file loaded with camera intrinsics in known format and no distortion; note that this\n"
			"    is a real-time technique and does not generate storable vertex colots)\n"
			"--inset-camera-images | -ici <image-src> - displays the chosen images in the bases of the camera\n"
			"                                           pyramids; this affects --project-camera-images and vice\n"
			"                                           versa\n");
	}
};

class CGraphData {
public:
	struct TIntrinsicsInfo {
		float fx, fy, cx, cy, k;
		// focal lenght, projection centre, the radial distortion param
		// where distorted = undistorted * (1 + k * squaredNorm(undistorted))

		TIntrinsicsInfo() // default; leave uninitialized
		{}

		TIntrinsicsInfo(float _fx, float _fy, float _cx, float _cy, float _k)
			:fx(_fx), fy(_fy), cx(_cx), cy(_cy), k(_k)
		{}
	};

public:
	std::vector<size_t> edge_range_list; /**< @brief list of edge ranges (one per solution frame, lower limit is always 0, the upper limit is the stored value) */
	std::vector<std::pair<size_t, size_t> > edge_list; /**< @brief list of (binary) edges */

	std::vector<Vector2f> observation_list; // parsed from the graph (assumes the edges hold 2D observations in pixels and in known format)
	std::map<size_t, TIntrinsicsInfo> camera_intrinsics; // parsed from the graph file (assumed not optimized or at least not by much)
	// these may not always be parsed

	std::vector<float> degree_list; /**< @brief list of vertex degrees (calculated from graph file, rescaled to [0, 1] for false color rendering) */
	// this is calculated from the graph

public:
	CGraphData()
	{}

	void Clear()
	{
		edge_range_list.clear();
		edge_list.clear();
		observation_list.clear();
		camera_intrinsics.clear();
		degree_list.clear();
	}

	bool Load_GraphFile(const char *p_s_filename, size_t n_vertex_list_size, size_t n_parse_limit = 0,
		bool b_recover_observations = false, bool b_recover_camera_intrinsics = false)
	{
		Clear();

		size_t n_dropped_edge_num = 0;

		std::vector<std::string> split;
		// reusable storage for splitting the lines to tokens

		FILE *p_fr;
#if defined(_MSC_VER) && !defined(__MWERKS__) && _MSC_VER >= 1400
		if(fopen_s(&p_fr, p_s_filename, "r"))
#else //_MSC_VER && !__MWERKS__ && _MSC_VER >= 1400
		if(!(p_fr = fopen(p_s_filename, "r")))
#endif //_MSC_VER && !__MWERKS__ && _MSC_VER >= 1400
			return false;
		std::string s_line;
		std::vector<std::string> number_list;
		while(!feof(p_fr)) {
			if(!stl_ut::ReadLine(s_line, p_fr)) {
				fclose(p_fr);
				return false;
			}
			stl_ut::TrimSpace(s_line);
			if(s_line.empty())
				continue;
			// read a single line

			if(b_recover_camera_intrinsics) {
				/*VERTEX
				VERTEX2
				VERTEX_SE2
				VERTEX3
				// SLAM

				VERTEX_XYZ
				VERTEX:INVD
				// landmark types

				VERTEX_CAM
				VERTEX_CAM:SIM3
				// cameras (supported)

				VERTEX_INTRINSICS
				// dedicated intrinsics vertex

				VERTEX_SCAM
				VERTEX_SPHERON:QUAT
				// stereo and spheron cameras (could be supported)
				*/

				if(s_line.length() > 7 && !strncmp(s_line.c_str(), "VERTEX_", 7)) {
					s_line.erase(0, /*strlen("VERTEX_")*/7);

					if((s_line.length() > 3 && !strncmp(s_line.c_str(), "CAM", 3) && isspace(uint8_t(s_line[3]))) ||
					   (s_line.length() > 8 && !strncmp(s_line.c_str(), "CAM:SIM3", 8) && isspace(uint8_t(s_line[8])))) {
						stl_ut::SplitByMultiple(split, s_line, " \t", 0);
						if(split.size() < 6)
							continue;
						// meh but it is the easiest and there are but a few cameras

						size_t n_camera_id = atol(split[1].c_str()); // camera id is the first
						size_t n_intrin = split.size() - 5;
						TIntrinsicsInfo t_intrin;
						t_intrin.fx = atof(split[n_intrin + 0].c_str());
						t_intrin.fy = atof(split[n_intrin + 1].c_str());
						t_intrin.cx = atof(split[n_intrin + 2].c_str());
						t_intrin.cy = atof(split[n_intrin + 3].c_str());
						t_intrin.k = atof(split[n_intrin + 4].c_str());

						camera_intrinsics[n_camera_id] = t_intrin;
					}
					// camera vertices recognized by SLAM++

					continue;
				}
			}
			// this is getting messy

			/*
			EDGE
			EDGE2
			EDGE_SE2
			EDGE_SE2_XY
			EDGE3
			EDGE3:AXISANGLE
			// 2D / 3D SLAM

			*EDGE_P2MC
			*EDGE_P2MCI
			*EDGE_PROJECT_P2MC
			*EDGE_PROJECT_P2SC
			// BA (supported)

			EDGE_PROJ_OTHER
			EDGE_PROJ_SELF
			// relativer parameterization of BA

			EDGE_SPHERON_XYZ
			// spherical
			*/
			// edges parsed by SLAM++

			int n_cam_index_arg = 1, n_lm_index_arg = 0, n_x_obs_arg = 2, n_y_obs_arg = 3;
			if(s_line.length() > 5 && !strncmp(s_line.c_str(), "EDGE_", 5)) {
				s_line.erase(0, /*strlen("EDGE_")*/5);
				if(s_line.length() > 5 && isspace(uint8_t(s_line[5])) && !strncmp(s_line.c_str(), "P2MCI", 5)) {
					s_line.erase(0, 5);
					n_x_obs_arg = 3;
					n_y_obs_arg = 4;
				} else if(s_line.length() > 4 && isspace(uint8_t(s_line[4])) && !strncmp(s_line.c_str(), "P2MC", 4))
					s_line.erase(0, 4);
				else if(s_line.length() > 12 && isspace(uint8_t(s_line[12])) && !strncmp(s_line.c_str(), "PROJECT_P2MC", 12))
					s_line.erase(0, 12);
				else if(s_line.length() > 12 && isspace(uint8_t(s_line[12])) && !strncmp(s_line.c_str(), "PROJECT_P2SC", 12))
					s_line.erase(0, 12);
				else
					continue;
			} else
				continue;
			// see if this is an edge of known type

			size_t n_camera, n_landmark;
			float f_x, f_y;
			if(b_recover_observations) {
				int n_iargs = max(n_cam_index_arg, n_lm_index_arg) + 1;
				if(n_iargs <= 4 && min(n_x_obs_arg, n_y_obs_arg) == n_iargs &&
				   max(n_x_obs_arg, n_y_obs_arg) == n_iargs + 1) { // simple consecutive args
					size_t p_iarg[4];
					float p_farg[2];
					switch(n_iargs) {
					case 2:
						if(sscanf(s_line.c_str(), PRIsize " " PRIsize " %f %f",
						   p_iarg, p_iarg + 1, p_farg, p_farg + 1) != 2 + 2)
							continue;
						break;
					case 3:
						if(sscanf(s_line.c_str(), PRIsize " " PRIsize " " PRIsize " %f %f",
						   p_iarg, p_iarg + 1, p_iarg + 2, p_farg, p_farg + 1) != 3 + 2)
							continue;
						break;
					default:
						_ASSERTE(n_iargs == 4);
						if(sscanf(s_line.c_str(), PRIsize " " PRIsize " " PRIsize " " PRIsize " %f %f",
						   p_iarg, p_iarg + 1, p_iarg + 2, p_iarg + 3, p_farg, p_farg + 1) != 4 + 2)
							continue;
						break;
					}
					n_camera = p_iarg[n_cam_index_arg];
					n_landmark = p_iarg[n_lm_index_arg];
					f_x = p_farg[n_x_obs_arg - n_iargs];
					f_y = p_farg[n_y_obs_arg - n_iargs];
				} else {
					stl_ut::SplitByMultiple(split, s_line, " \t", 0);
					if(split.size() < size_t(max(n_iargs, max(n_x_obs_arg, n_y_obs_arg) + 1)))
						continue;
					if(sscanf(split[n_cam_index_arg].c_str(), PRIsize, &n_camera) != 1 ||
					   sscanf(split[n_lm_index_arg].c_str(), PRIsize, &n_landmark) != 1 ||
					   sscanf(split[n_x_obs_arg].c_str(), "%f", &f_x) != 1 ||
					   sscanf(split[n_y_obs_arg].c_str(), "%f", &f_y) != 1)
						continue;
				}
			} else {
				size_t p_arg[4];
				int n_args = max(n_cam_index_arg, n_lm_index_arg) + 1;
				switch(n_args) {
				case 2:
					if(sscanf(s_line.c_str(), PRIsize " " PRIsize, p_arg, p_arg + 1) != 2)
						continue;
					n_camera = p_arg[n_cam_index_arg];
					n_landmark = p_arg[n_lm_index_arg];
					break;
				case 3:
					if(sscanf(s_line.c_str(), PRIsize " " PRIsize " " PRIsize,
					   p_arg, p_arg + 1, p_arg + 2) != 3)
						continue;
					n_camera = p_arg[n_cam_index_arg];
					n_landmark = p_arg[n_lm_index_arg];
					break;
				case 4:
					if(sscanf(s_line.c_str(), PRIsize " " PRIsize " " PRIsize
					   " " PRIsize, p_arg, p_arg + 1, p_arg + 2, p_arg + 3) != 4)
						continue;
					n_camera = p_arg[n_cam_index_arg];
					n_landmark = p_arg[n_lm_index_arg];
					break;
				default:
					{
						stl_ut::SplitByMultiple(split, s_line, " \t", 0);
						if(split.size() < size_t(n_args))
							continue;
						if(sscanf(split[n_cam_index_arg].c_str(), PRIsize, &n_camera) != 1 ||
						   sscanf(split[n_lm_index_arg].c_str(), PRIsize, &n_landmark) != 1)
							continue;
					}
					break;
				}
			}
			// pick out the fields by their indices

			/*if(sscanf(s_line.c_str(), "EDGE_P2MCI " PRIsize " " PRIsize, &n_camera, &n_landmark) == 2) // support BA with intrinsics
				edge_list.push_back(std::make_pair(n_camera, n_landmark)); // throws
			else if(sscanf(s_line.c_str(), "EDGE_P2MC " PRIsize " " PRIsize, &n_camera, &n_landmark) == 2)
				edge_list.push_back(std::make_pair(n_camera, n_landmark)); // throws
			else if(sscanf(s_line.c_str(), "EDGE_PROJECT_P2MC " PRIsize " " PRIsize, &n_camera, &n_landmark) == 2)
				edge_list.push_back(std::make_pair(n_camera, n_landmark)); // throws
			else if(sscanf(s_line.c_str(), "EDGE_PROJECT_P2SC " PRIsize " " PRIsize, &n_camera, &n_landmark) == 2) // support stereo
				edge_list.push_back(std::make_pair(n_camera, n_landmark)); // throws
			else
				continue;*/ // no edge added, would not reach parse limit anyways
			// old code

			if(std::max(n_camera, n_landmark) >= n_vertex_list_size) {
				//edge_list.erase(edge_list.end() - 1); // !!
				++ n_dropped_edge_num;
				continue;
			}

			edge_list.push_back(std::make_pair(n_camera, n_landmark)); // throws

			if(b_recover_observations)
				observation_list.push_back(Vector2f(f_x, f_y)); // throws
			// recover observations of the points (useful for point cloud colorizing)

			if(n_parse_limit && edge_list.size() == n_parse_limit)
				break;
			// handle limit
		}
		fclose(p_fr);
		// parse the file

		if(n_dropped_edge_num)
			printf("warning: dropped " PRIsize " edges (output / graph mismatch?)\n", n_dropped_edge_num);
		// be permissive about having more edges

		printf("have " PRIsize " edges\n", edge_list.size());

		return true;
	}

	class CCompareMax {
	public:
		template <class A>
		inline bool operator ()(std::pair<A, A> a, std::pair<A, A> b) const
		{
			return max(a.first, a.second) < max(b.first, b.second);
		}

		template <class A, class B>
		inline bool operator ()(std::pair<A, A> a, B b) const
		{
			return max(a.first, a.second) < b;
		}

		template <class A, class B>
		inline bool operator ()(A a, std::pair<B, B> b) const
		{
			return a < max(b.first, b.second);
		}
	};

	class CRescale {
	protected:
		size_t m_n_min_value;
		float m_f_scale;

	public:
		CRescale(size_t n_min_value, size_t n_max_value)
			:m_n_min_value(n_min_value), m_f_scale(1.0f / std::max(n_max_value - n_min_value, size_t(1)))
		{}

		float operator ()(size_t x) const
		{
			return 1 - (x - m_n_min_value) * m_f_scale;
		}
	};

	void Calculate_EdgeRanges(const std::vector<std::pair<size_t, size_t> > &vertex_range_list)
	{
		edge_range_list.clear();
		if(vertex_range_list.size() > 1) {
			if(edge_list.size() == observation_list.size()) {
				std::stable_sort(stl_ut::p_Make_PairIterator(edge_list.begin(), observation_list.begin()),
					stl_ut::p_Make_PairIterator(edge_list.end(), observation_list.end()),
					stl_ut::CCompareFirstT<CCompareMax>());
				// sort the arrays together!
			} else
				std::stable_sort(edge_list.begin(), edge_list.end(), CCompareMax());
			// sort the graph (probably no change for the incremental BA graphs)

			for(size_t i = 0, n = vertex_range_list.size(); i < n; ++ i) {
				size_t n_vertex_num = vertex_range_list[i].second;
				size_t n_last_vertex = (n_vertex_num)? n_vertex_num - 1 : 0; // index of a vertex
				size_t n_edge_num = std::upper_bound(edge_list.begin(),
					edge_list.end(), n_last_vertex, CCompareMax()) - edge_list.begin();
				// calculate the number of edges that reference vertices lower than n_vertex_num

				edge_range_list.push_back(n_edge_num);
				// put it to the list
			}
		} else
			edge_range_list.push_back(edge_list.size());
		// in case we have a multi-frame solution, sort the graph by the maximum refrenced vertex
		// and generate a list of edge ranges, corresponding to each solution frame
	}

	void Calculate_VertexDegrees(const std::vector<std::pair<size_t, size_t> > &vertex_range_list)
	{
		_ASSERTE(!edge_range_list.empty());
		_ASSERTE(edge_range_list.size() == vertex_range_list.size()); // !!

		std::vector<size_t> int_degree_list;
		std::vector<size_t> frame_degrees;
		// prepare a list of degrees in a single frame

		size_t n_prev_vertex_num = 0;
		for(size_t i = 0, n = edge_range_list.size(); i < n; ++ i) {
			size_t n_vertex_num = vertex_range_list[i].second;
			bool b_monotonic = n_prev_vertex_num <= n_vertex_num;
			n_prev_vertex_num = n_vertex_num;
			size_t n_edge_num = edge_range_list[i];
			// calculate the number of edges that reference vertices lower than n_vertex_num

			if(!b_monotonic)
				frame_degrees.clear(); // !! in case the vertex ranges are not increasing then something funny is going on here, need to recalculate for every frame (then this becomes costly)
			frame_degrees.resize(n_vertex_num, size_t(0));
			// append zeros for new vertices

			for(size_t j = (i && b_monotonic)? edge_range_list[i - 1] : 0; j < n_edge_num; ++ j) {
				std::pair<size_t, size_t> t_edge = edge_list[j];
				//++ frame_degrees[t_edge.first]; // not cameras
				++ frame_degrees[t_edge.second]; // only count degrees of landmarks, cameras would have much more and would greatly exceed the points which would in turn be hardly visible
			}
			// for the new edges, increment the vertex degrees (while keeping the
			// old degrees intact - this only scans through the edges once, hence
			// keeping O(n) for the outer loop)

			int_degree_list.insert(int_degree_list.end(), frame_degrees.begin(), frame_degrees.end());
			// put the frame degrees at the end of the degree list
		}

#ifdef _DEBUG
		// todo - see if the degrees are really correct
#endif // _DEBUG

		_ASSERTE(stl_ut::b_IsWeaklySorted(vertex_range_list.begin(),
			vertex_range_list.end(), stl_ut::CCompareFirst()));
		const size_t n_vertex_list_size = vertex_range_list.back().first +
			vertex_range_list.back().second; // first + number (assumes this is sorted)

		if(int_degree_list.size() < n_vertex_list_size)
			int_degree_list.resize(n_vertex_list_size, size_t(0)); // unreferenced vertices?
		_ASSERTE(int_degree_list.size() == n_vertex_list_size);
		// there must be one degree per each vertex per each solution frame in the list

		size_t n_min_degree = (int_degree_list.empty())? 0 : *std::min_element(int_degree_list.begin(), int_degree_list.end());
		size_t n_max_degree = (int_degree_list.empty())? 0 : *std::max_element(int_degree_list.begin(), int_degree_list.end());
		degree_list.resize(int_degree_list.size());

		std::transform(int_degree_list.begin(), int_degree_list.end(),
			degree_list.begin(), CRescale(n_min_degree, n_max_degree));

		_ASSERTE(observation_list.empty() || observation_list.size() == edge_list.size());

		printf("maximum degree is " PRIsize "\n", n_max_degree);
	}

private:
	CGraphData(const CGraphData &r_other); // no-copy
	CGraphData operator =(const CGraphData &r_other); // no-copy
};

class CSolutionData {
protected:
	CForwardAllocatedPool<TScalar> data_pool; /**< @brief pool of vertex data */

public:
	std::vector<CVertex> vertex_list; /**< @brief list of vertices */
	std::vector<std::pair<size_t, size_t> > vertex_range_list; /**< @brief list of vertex ranges (one per solution frame, each is a pair of starting index and the number of vertices in a given frame) */

	std::vector<Vector3ub> vertex_color_list; /**< @brief list of vertex colors, matches the list of vertices 1:1 */
	// not very sure this should be here

public:
	CSolutionData()
		:data_pool(4096)
	{}

	void Clear()
	{
		data_pool.Clear();
		vertex_list.clear();
		vertex_range_list.clear();
		vertex_color_list.clear();
	}

	/**
	 *	@brief gets the sum of numbers of vertices in all solution frames
	 *
	 */
	size_t n_Combined_Vertex_Num() const
	{
		return vertex_list.size();
	}

	/**
	 *	@brief gets the number of vertices in a given solution frames
	 *
	 */
	size_t n_Vertex_Num(size_t n_frame) const
	{
		return vertex_range_list[n_frame].second;
	}

	/**
	 *	@brief gets the maximum number of vertices in any of the solution frames
	 *
	 */
	size_t n_Max_Vertex_Num() const
	{
		return (vertex_range_list.empty())? 0 : (*std::max_element(vertex_range_list.begin(),
			vertex_range_list.end(), stl_ut::CCompareSecond())).second;
	}

	bool Load_ResultFile(const char *p_s_filename, size_t n_parse_limit = 0)
	{
		data_pool.Clear();
		vertex_list.clear();
		vertex_color_list.clear();
		vertex_range_list.clear();

		std::map<int, size_t> vertex_type_histogram;

		//setlocale(LC_ALL, "C"); // no apparent effect on speed

		bool b_coff_input = false;
		bool b_pts_input = false; // if this is set then b_coff_input is also set

		std::string s_pattern = p_s_filename;
		if(strchr(p_s_filename, '#') && CGLThreadedFrameWriter::Make_Pattern(s_pattern)) {
			CTimer time;
			uint64_t n_files_size = 0, n_total_size = 0;

			std::string s_file;
			for(size_t i = 0;; ++ i) { // no limit
				stl_ut::Format(s_file, s_pattern.c_str(), i);
				// generate a filename

				FILE *p_fr;
#if defined(_MSC_VER) && !defined(__MWERKS__) && _MSC_VER >= 1400
				if(fopen_s(&p_fr, s_file.c_str(), "r"))
#else //_MSC_VER && !__MWERKS__ && _MSC_VER >= 1400
				if(!(p_fr = fopen(s_file.c_str(), "r")))
#endif //_MSC_VER && !__MWERKS__ && _MSC_VER >= 1400
					break;

				if(s_file.rfind(".off") != std::string::npos || s_file.rfind(".coff") != std::string::npos) {
					std::string s_line;
					if(!stl_ut::ReadLine(s_line, p_fr)) {
						fclose(p_fr);
						return false;
					}
					stl_ut::TrimSpace(s_line);
					if((b_coff_input = (s_line == "COFF"))) {
						size_t n_vertices, n_edges, n_triangles;
						if(!stl_ut::ReadLine(s_line, p_fr) || sscanf(s_line.c_str(), " " PRIsize " "
						   PRIsize " " PRIsize "\n", &n_vertices, &n_edges, &n_triangles) != 3 ||
						   n_edges > 0 || n_triangles > 0) {
							fclose(p_fr);
							return false;
						}
					} else
						fseek(p_fr, 0, SEEK_SET); // rewind to re-read the first line
				} else if(s_file.rfind(".pts") != std::string::npos) {
					std::string s_line;
					size_t n;
					if(!stl_ut::ReadLine(s_line, p_fr)) {
						fclose(p_fr);
						return false;
					}
					if((b_coff_input = (sscanf(s_line.c_str(), PRIsize, &n) == 1)))
						b_pts_input = true; // this is a pts input
					else
						fseek(p_fr, 0, SEEK_SET); // rewind to re-read the first line
				}
				// detect coff

				size_t n_range_start = vertex_list.size();
				//size_t n_point_num = 0, n_pose_num = 0;

				bool b_parse_no_more = false;

#if 0
				{CNumFileReader reader(p_fr, true); // use a reader running in a second thread; also serves as a file guard
				for(;;) {
					const std::vector<float> *nl;
					if(!(nl = reader.ReadNumLine())) {
						if(reader.b_EOF()) // will most likely happen here instead
							break;
						return false;
					}
					const std::vector<float> &number_list = *nl; // avoid copying the vectors
#elif 0
				{CFileReader reader(p_fr, true); // use a reader running in a second thread; also serves as a file guard
				std::string s_line;
				std::vector<std::string> number_list;
				for(;;) {
					if(!reader.ReadLine(s_line)) {
						if(reader.b_EOF()) // will most likely happen here instead
							break;
						return false;
					}
					stl_ut::Split(number_list, s_line, " ", 0); // 15 MB/s -> 10 MB/s
					// split it to numbers
#else // 0
				std::string s_line;
				std::vector<std::string> number_list;
				for(size_t n_line = 0; !feof(p_fr); ++ n_line) {
					if(!stl_ut::ReadLine(s_line, p_fr)) {
						fclose(p_fr);
						return false;
					}
					stl_ut::TrimSpace(s_line);
					if(s_line.empty())
						continue;

					stl_ut::SplitByMultiple(number_list, s_line, " \t", 0); // 15 MB/s -> 10 MB/s
					// split it to numbers
#endif // 0
					// read a single line

					_ASSERTE(number_list.size() <= INT_MAX);
					int n_dim = int(number_list.size());

					if(b_coff_input) {
						if(b_pts_input) {
							if(n_dim != 7) {
								fprintf(stderr, "error: \'%s\', line " PRIsize ": invalid pts input: \'%s\'\n",
									s_file.c_str(), n_line, s_line.c_str());
								fclose(p_fr);
								return false;
							}
							// (x, z, y, i, r, g, b)

							Vector3ub v_color(atol(number_list[4].c_str()),
								atol(number_list[5].c_str()), atol(number_list[6].c_str()));
							vertex_color_list.push_back(v_color);
							// parse color, represent as 8-bit
						} else {
							if(n_dim != 7) {
								fprintf(stderr, "error: \'%s\', line " PRIsize ": invalid coff input: \'%s\'\n",
									s_file.c_str(), n_line, s_line.c_str());
								fclose(p_fr);
								return false;
							}
							// (x, z, y, r, g, b, crange)

							Vector3f v_color = Vector3f(atof(number_list[3].c_str()),
								atof(number_list[4].c_str()), atof(number_list[5].c_str())) * (255 / atof(number_list[6].c_str()));
							Vector3ub v_color_ub(uint8_t(max(0, min(255, v_color.x))),
								uint8_t(max(0, min(255, v_color.y))), uint8_t(max(0, min(255, v_color.z))));
							vertex_color_list.push_back(v_color_ub);
							// parse color, represent as 8-bit
						}

						n_dim = 3;
						// vertex itself is only 3D (no cameras in coff/pts)
					}

					TScalar *p_data = data_pool.p_GetRange(n_dim); // negligible cost
					CVertex v(n_dim, p_data);
					for(int i = 0; i < n_dim; ++ i)
						//sscanf(number_list[i].c_str(), "%f", &p_data[i]); // 7 MB/s -> 3.6 MB/s
						p_data[i] = atof(number_list[i].c_str()); // 7 MB/s -> 5 MB/s
						//p_data[i] = number_list[i]; // for CNumFileReader
					/*{
						if(v.n_dim == 7) { // t_odo - handle this directly, without the conversion
							// this is a quaternion, nëed to convert to axis-angle, as in SLAM++ parser

							Quatf q(v.p_data[3], v.p_data[4], v.p_data[5], v.p_data[6]);
							Vector3f r = q.v_ToAxisAngle();
							v.p_data[3] = r.x;
							v.p_data[4] = r.y;
							v.p_data[5] = r.z;

							v.n_dim = 6;
							evl.push_back(v);

							Vector3f p(v.p_data[0], v.p_data[1], v.p_data[2]);
							p = q.v_Transform(-p);
							v.p_data[0] = p.x;
							v.p_data[1] = p.y;
							v.p_data[2] = p.z;
							// t_odo - invtransform the position
						}
					}*/

					if(b_coff_input)
						std::swap(p_data[1], p_data[2]);
					// coff is (x, z, y)

					size_t n_frame_vertex = vertex_list.size() - n_range_start;
					if(!vertex_range_list.empty() && vertex_range_list.back().second > n_frame_vertex &&
					   vertex_list[vertex_range_list.back().first +
					   n_frame_vertex].n_Dimension() != v.n_Dimension()) {
						fprintf(stderr, "error: vertex " PRIsize " has dimension %u"
							" in frame " PRIsize " and dimension %u in frame "
							PRIsize ": data inconsistent\n", n_frame_vertex,
							vertex_list[vertex_range_list.back().first +
							n_frame_vertex].n_Dimension(), vertex_range_list.size(),
							v.n_Dimension(), vertex_range_list.size() + 1);
						b_parse_no_more = true;
						break;
					}
					// t_odo: make sure the vertex has the same dimension as the
					// corresponding vertex in the previous frame, if it existed

					vertex_list.push_back(v); // throws // 10 MB/s -> 7 MB/s
					// parse a vertex, put it in the list

					/*if(v.b_Has_Position())
						++ n_point_num;
					if(v.b_Has_Pose())
						++ n_pose_num;*/

					if(n_parse_limit && vertex_list.size() - n_range_start == n_parse_limit)
						break;
					// handle limit
				}
#if 0
				}
#else // 0
				fclose(p_fr);
#endif // 0
				// parse the file

				if(b_parse_no_more)
					break;
				// we hit an inconsistency, dont continue

				size_t n_frame_vertex_num = vertex_list.size() - n_range_start;
				if(!vertex_range_list.empty() && vertex_range_list.back().second > n_frame_vertex_num) {
					fprintf(stderr, /*"error"*/"warning: frame " PRIsize " smaller than the previous frame ("
						PRIsize " < " PRIsize ")\n", vertex_range_list.size(), n_frame_vertex_num,
						vertex_range_list.back().second);
					//break; // lets try and allow this
				}
				// we will count on the next solution frames generally increase in size

				vertex_range_list.push_back(std::make_pair(n_range_start, n_frame_vertex_num));
				// add this as another range

				/*drawable_vertex_list_nc.push_back(n_point_num);
				drawable_pose_list.push_back(n_pose_num);*/
				// maintain stats

				if(n_parse_limit && vertex_list.size() - n_range_start == n_parse_limit)
					break;
				// handle limit

				n_files_size += TFileInfo(s_file).n_Size64();
				if(vertex_range_list.size() >= 16) {
					if(!n_total_size) {
						n_total_size = n_files_size;
						for(size_t j = i + 1;; ++ j) { // no limit
							stl_ut::Format(s_file, s_pattern.c_str(), j);
							TFileInfo t_file(s_file);
							if(!t_file.b_exists)
								break;
							n_total_size += t_file.n_Size64();
						}

						//vertex_list.reserve(size_t(double(vertex_list.size()) / n_files_size * n_total_size));
						//printf("debug: assuming " PRIsize " vertices\n", vertex_list.capacity());
						// try to avoid reallocations
					}
					// calculate size of all the files

					double f_elapsed = time.f_Time();
					double f_speed = n_files_size / f_elapsed;
					double f_estimated = n_total_size / f_speed;
					double f_remaining = max(0, f_estimated - f_elapsed);
					printf("loaded " PRIsize " solution frames (" PRIsizeB "B/sec, " PRItime ")\r",
						vertex_range_list.size(), PRIsizeBparams(f_speed), PRItimeparams(f_remaining));
				} else
					printf("loaded " PRIsize " solution frames\r", vertex_range_list.size());
				// show some progress
			}

			if(!vertex_range_list.empty()) {
				for(size_t i = vertex_range_list.back().first, n = i + vertex_range_list.back().second; i < n; ++ i) {
					++ vertex_type_histogram[vertex_list[i].n_Dimension()];
					// make a histogram out of the last frame only
				}
			}
			printf("loaded " PRIsize " solution frames (" PRIsizeB "B, " PRIsizeB "B/sec)%10s\n",
				vertex_range_list.size(), PRIsizeBparams(n_files_size), PRIsizeBparams(n_files_size / time.f_Time()), "");
			printf("have " PRIsize " vertices (" PRIsize " vertices overall, " PRIsizeB "B of data)\n",
				(vertex_range_list.empty())? 0 : vertex_range_list.back().second, vertex_list.size(),
				PRIsizeBparams(vertex_list.size() * sizeof(CVertex) + data_pool.n_Capacity() * sizeof(TScalar)));
			printf("in the last frame:\n");
		} else {
			//std::vector<CVertex> evl;

			//size_t n_point_num = 0, n_pose_num = 0;

			FILE *p_fr;
#if defined(_MSC_VER) && !defined(__MWERKS__) && _MSC_VER >= 1400
			if(fopen_s(&p_fr, p_s_filename, "r"))
#else //_MSC_VER && !__MWERKS__ && _MSC_VER >= 1400
			if(!(p_fr = fopen(p_s_filename, "r")))
#endif //_MSC_VER && !__MWERKS__ && _MSC_VER >= 1400
				return false;
			std::string s_line;

			if(strstr(p_s_filename, ".off") || strstr(p_s_filename, ".coff")) {
				if(!stl_ut::ReadLine(s_line, p_fr)) {
					fclose(p_fr);
					return false;
				}
				stl_ut::TrimSpace(s_line);
				if((b_coff_input = (s_line == "COFF"))) {
					size_t n_vertices, n_edges, n_triangles;
					if(!stl_ut::ReadLine(s_line, p_fr) || sscanf(s_line.c_str(), " " PRIsize " "
					   PRIsize " " PRIsize "\n", &n_vertices, &n_edges, &n_triangles) != 3 ||
					   n_edges > 0 || n_triangles > 0) {
						fclose(p_fr);
						return false;
					}
				} else
					fseek(p_fr, 0, SEEK_SET); // rewind to re-read the first line
			} else if(strstr(p_s_filename, ".pts")) {
				std::string s_line;
				size_t n;
				if(!stl_ut::ReadLine(s_line, p_fr)) {
					fclose(p_fr);
					return false;
				}
				if((b_coff_input = (sscanf(s_line.c_str(), PRIsize, &n) == 1)))
					b_pts_input = true; // this is a pts input
				else
					fseek(p_fr, 0, SEEK_SET); // rewind to re-read the first line
			}
			// detect coff

			std::vector<std::string> number_list;
			for(size_t n_line = 0; !feof(p_fr); ++ n_line) {
				if(!stl_ut::ReadLine(s_line, p_fr)) {
					fclose(p_fr);
					return false;
				}
				stl_ut::TrimSpace(s_line);
				if(s_line.empty())
					continue;
				// read a single line

				stl_ut::SplitByMultiple(number_list, s_line, " \t", 0);
				// split it to numbers

				_ASSERTE(number_list.size() <= INT_MAX);
				int n_dim = int(number_list.size());

				if(b_coff_input) {
					if(b_pts_input) {
						if(n_dim != 7) {
							fprintf(stderr, "warning: \'%s\', line " PRIsize ": invalid pts input: \'%s\'\n",
								p_s_filename, n_line, s_line.c_str());
							break;
							/*fclose(p_fr);
							return false;*/
						}
						// (x, z, y, i, r, g, b)

						Vector3ub v_color(atol(number_list[4].c_str()),
							atol(number_list[5].c_str()), atol(number_list[6].c_str()));
						vertex_color_list.push_back(v_color);
						// parse color, represent as 8-bit
					} else {
						if(n_dim != 7) {
							fprintf(stderr, "warning: \'%s\', line " PRIsize ": invalid coff input: \'%s\'\n",
								p_s_filename, n_line, s_line.c_str());
							break;
							/*fclose(p_fr);
							return false;*/
						}
						// (x, z, y, r, g, b, crange)

						Vector3f v_color = Vector3f(atof(number_list[3].c_str()),
							atof(number_list[4].c_str()), atof(number_list[5].c_str())) * (255 / atof(number_list[6].c_str()));
						Vector3ub v_color_ub(uint8_t(max(0, min(255, v_color.x))),
							uint8_t(max(0, min(255, v_color.y))), uint8_t(max(0, min(255, v_color.z))));
						vertex_color_list.push_back(v_color_ub);
						// parse color, represent as 8-bit
					}

					n_dim = 3;
					// vertex itself is only 3D (no cameras in coff/pts)
				}

				TScalar *p_data = data_pool.p_GetRange(n_dim);
				CVertex v(n_dim, p_data);
				for(int i = 0; i < n_dim; ++ i)
					p_data[i] = atof(number_list[i].c_str());
				/*{
					if(v.n_dim == 7) { // t_odo - handle this directly, without the conversion
						// this is a quaternion, nëed to convert to axis-angle, as in SLAM++ parser

						Quatf q(v.p_data[3], v.p_data[4], v.p_data[5], v.p_data[6]);
						Vector3f r = q.v_ToAxisAngle();
						v.p_data[3] = r.x;
						v.p_data[4] = r.y;
						v.p_data[5] = r.z;

						v.n_dim = 6;
						evl.push_back(v);

						Vector3f p(v.p_data[0], v.p_data[1], v.p_data[2]);
						p = q.v_Transform(-p);
						v.p_data[0] = p.x;
						v.p_data[1] = p.y;
						v.p_data[2] = p.z;
						// t_odo - invtransform the position
					}
				}*/
				vertex_list.push_back(v); // throws
				// parse a vertex, put it in the list

				if(b_coff_input)
					std::swap(p_data[1], p_data[2]);
				// coff is (x, z, y)

				++ vertex_type_histogram[n_dim];
				// make a histogram

				/*if(v.b_Has_Position())
					++ n_point_num;
				if(v.b_Has_Pose())
					++ n_pose_num;*/

				if(n_parse_limit && vertex_list.size() == n_parse_limit)
					break;
				// handle limit
			}
			fclose(p_fr);
			// parse the file
			//vertex_list.insert(vertex_list.end(), evl.begin(), evl.end());

			/*drawable_vertex_list_nc.resize(1, n_point_num);
			drawable_pose_list.resize(1, n_pose_num);*/

			vertex_range_list.push_back(std::make_pair(size_t(0), vertex_list.size()));
			printf("have " PRIsize " vertices\n", vertex_list.size());
		}

		if(b_coff_input && vertex_color_list.size() != vertex_list.size()) {
			fprintf(stderr, "error: number of vertex colors does not match the number of vertices"
				" (some of the animation frames not .off?); colors dropped\n");
			vertex_color_list.clear();
		}
		// the numbers must match

		if(b_coff_input) {
			/*if(vertex_range_list.size() > 1) // this is now supported
				fprintf(stderr, "debug: rendering animated color point clouds not supported\n"); // t_odo - add support
			else*/
				fprintf(stderr, "debug: detected .%s input\n", (b_pts_input)? "pts" : "off");
		}
		// notify the user

		//drawable_vertex_list.resize(vertex_range_list.size(), size_t(0));
		// this is unclear until the covs are loaded

		for(std::map<int, size_t>::const_iterator p_hg_it = vertex_type_histogram.begin(),
		   p_end_it = vertex_type_histogram.end(); p_hg_it != p_end_it; ++ p_hg_it)
			printf("\t" PRIsize " of %dD vertices\n", (*p_hg_it).second, (*p_hg_it).first);
		// print the counts of vertices of different dimensions

		return true;
	}

	template <int n_elem>
	class CCompareVecElem {
	public:
		template <class CVecType>
		bool operator ()(const CVecType &r_v_a, const CVecType &r_v_b) const
		{
			return r_v_a[n_elem] < r_v_b[n_elem];
		}
	};

	bool Colorize_PointCloud(const char *p_s_input_images, const Matrix3f &r_t_raster_matrix, int n_filter_width, const CGraphData &r_graph)
	{
		_ASSERTE(p_s_input_images);

		if(n_filter_width <= 0)
			n_filter_width = 1; // must have at least a single sample

		vertex_color_list.clear();

		printf("extracting landmark colors from images ...\n");

		std::vector<std::string> color_image_list;
		if(!Resolve_FileList(color_image_list, p_s_input_images)) {
			fprintf(stderr, "error: failed to resolve input images\n");
			return false;
		}

		printf("\thave a list of " PRIsize " color images\n", color_image_list.size());
		// verbose

		std::vector<size_t> camera_id_list;
		for(size_t i = 0, n = r_graph.edge_list.size(); i < n; ++ i)
			stl_ut::t_Unique_Insert(camera_id_list, r_graph.edge_list[i].first); // throws
		// make a list of cameras

		printf("\tthe graph contains " PRIsize " cameras\n", camera_id_list.size());
		// verbose

		const bool b_have_all_cameras = camera_id_list.size() == color_image_list.size();
		if(camera_id_list.size() > color_image_list.size())
			camera_id_list.erase(camera_id_list.begin() + color_image_list.size(), camera_id_list.end());
		else if(camera_id_list.size() < color_image_list.size())
			color_image_list.erase(color_image_list.begin() + camera_id_list.size(), color_image_list.end()); // fails later
		_ASSERTE(camera_id_list.size() == color_image_list.size());
		_ASSERTE(camera_id_list.size() < INT_MAX);
		const size_t n_camera_num = camera_id_list.size();
		if(!n_camera_num)
			return false;
		// trim either of the lists to have the same number of cameras and images

		std::vector<std::vector<Vector3ub> > observation_color_samples; // empty! // todo - this can be 1D array, each vector contains the same amount of data (n_filter_width * n_filter_width samples)
		// the same order as observation_list

		{
			_ASSERTE(r_graph.observation_list.size() == r_graph.edge_list.size());
			//std::map<size_t, std::vector<size_t> > obs_seen_by_camera;
			std::vector<std::vector<size_t> > obs_seen_by_camera(camera_id_list.size()); // will index with consecutive integers
			for(size_t i = 0, n = r_graph.edge_list.size(); i < n; ++ i) {
				size_t n_camera_id = r_graph.edge_list[i].first; // id in vertex list
				size_t n_cam_index = std::lower_bound(camera_id_list.begin(),
					camera_id_list.end(), n_camera_id) - camera_id_list.begin(); // index among the cameras only (selects the image)
				if(!b_have_all_cameras && n_cam_index >= camera_id_list.size())
					continue;
				_ASSERTE(n_cam_index < camera_id_list.size() && camera_id_list[n_cam_index] == n_camera_id);
				obs_seen_by_camera[n_cam_index].push_back(i);
			}
			// a quick reverse lookup (actually a half of the dual graph)

			size_t n_processed_num = 0;
			#pragma omp parallel
			{
				std::vector<std::vector<Vector3ub> > thread_observation_colors(r_graph.observation_list.size()); // todo - this can be 1D array, each vector contains the same amount of data (n_filter_width * n_filter_width samples, unless we're missing some camera images)
				// the same order as observation_list

				_ASSERTE(n_camera_num <= INT_MAX);
				int _n_camera_num = int(n_camera_num);
				#pragma omp for schedule(dynamic, 1)
				for(int i = 0; i < _n_camera_num; ++ i) {
					const size_t n_this_camera_id = camera_id_list[i];
					//_ASSERTE(obs_seen_by_camera.count(i)); // we use the observations to determine which vertices are cameras so there cannot be an entry missing for any of them (if there was no entry, it wouldnt be in the camera list in the first place)
					const std::vector<size_t> &r_camera_obs = obs_seen_by_camera[i]; // does not throw, due to the above assertion

					TBmp *p_image;
					if(!(p_image = CGLESTextureLoader::p_LoadImage(color_image_list[i].c_str()))) {
						fprintf(stderr, "error: failed to load \'%s\'\n", color_image_list[i].c_str());
						continue;
					}
					const int h1 = p_image->n_height - 1, w = p_image->n_width, w1 = p_image->n_width - 1;
					// load the image

					try {
						size_t n_this_camera_id = camera_id_list[i];
						for(size_t j = 0, m = r_camera_obs.size(); j < m; ++ j) {
							size_t n_obs = r_camera_obs[j];
							std::vector<Vector3ub> &r_dest = thread_observation_colors[n_obs];

							Vector3f v_pos(r_graph.observation_list[n_obs], 1);
							v_pos = r_t_raster_matrix * v_pos;
							Vector2f v_pos_px = v_pos.v_xy() / v_pos.z; // demohog!
							const int n_x = int(v_pos_px.x + .5), n_y = int(v_pos_px.y + .5);
							for(int y = n_y - n_filter_width / 2, ey = y + n_filter_width; y < ey; ++ y) {
								int sy = max(0, min(h1, y));
								for(int x = n_x - n_filter_width / 2, ex = x + n_filter_width; x < ex; ++ x) {
									int sx = max(0, min(w1, x));
									uint32_t n_color = p_image->p_buffer[sx + w * sy];
									if(x < -10)
										n_color = 0xffff0000;
									else if(x > w + 10)
										n_color = 0xffffff00;
									if(y < -10)
										n_color = 0xff00ff00;
									else if(y > h1 + 11)
										n_color = 0xff0000ff;
									r_dest.push_back(Vector3ub(n_color & 0xff,
										(n_color >> 8) & 0xff, (n_color >> 16) & 0xff)); // throws
								}
							}
						}
						// sample the image, remember all the obs (or could filter here once
						// and then filter the filtered samples from multiple cameras again)
					} catch(std::bad_alloc &r_exc) {
						p_image->Delete(); // !!
						fprintf(stderr, "error: not enough memory for image samples, decrease filter size\n");
						throw r_exc;
					}

					p_image->Delete();

					#pragma omp critical
					{
						printf("\r\tprocessed " PRIsize " / " PRIsize " images", ++ n_processed_num, n_camera_num);
					}
				}
				// each thread loads a bunch of images and extracts pixel colors

				#pragma omp critical
				{
					if(observation_color_samples.empty())
						thread_observation_colors.swap(observation_color_samples);
					else {
						_ASSERTE(observation_color_samples.size() == thread_observation_colors.size());
						for(size_t j = 0, m = thread_observation_colors.size(); j < m; ++ j) {
							observation_color_samples[j].insert(observation_color_samples[j].end(),
								thread_observation_colors[j].begin(), thread_observation_colors[j].end());
							{
								std::vector<Vector3ub> free_mem;
								free_mem.swap(thread_observation_colors[j]);
								// keep freeing the memory that is not needed anymore, reduce the peak mem use
							}
						}
					}
				}
				// reduce the results from all the threads at the end
			}
			printf("\n"/*"\tdone. filtering ...\n"*/);
		}
		// extract colors (multiple samples per observation, multiple observations per point)

		vertex_color_list.clear();

		_ASSERTE(r_graph.edge_range_list.size() == vertex_range_list.size());
		for(size_t k = 0, o = r_graph.edge_range_list.size(); k < o; ++ k) {
			const size_t n_frame_edge_num = r_graph.edge_range_list[k];
			const size_t n_frame_vertex_num = vertex_range_list[k].second;

			std::vector<std::vector<size_t> > obs_of_landmark(n_frame_vertex_num/*vertex_list.size()*/); // num_landmarks >> num_cameras, there will be negligible slack there
			for(size_t i = 0, n = n_frame_edge_num/*edge_list.size()*/; i < n; ++ i) {
				if(!b_have_all_cameras && size_t(std::lower_bound(camera_id_list.begin(),
				   camera_id_list.end(), r_graph.edge_list[i].first) -
				   camera_id_list.begin()) >= camera_id_list.size())
					continue;
				obs_of_landmark[r_graph.edge_list[i].second].push_back(i);
			}
			// a fast reverse lookup (the other half of the dual graph)

			const size_t n_first_dest_vertex = vertex_color_list.size();
			vertex_color_list.resize(n_first_dest_vertex + n_frame_vertex_num, Vector3ub(0xff, 0xff, 0xff));
			// initialize everything to white

			_ASSERTE(n_frame_vertex_num/*vertex_list.size()*/ <= INT_MAX);
			const int n_vertex_num = int(n_frame_vertex_num/*vertex_list.size()*/);
			#pragma omp parallel
			{
				std::vector<Vector3ub> color_samples;
				std::vector<int> ranks_r, ranks_g, ranks_b;

				#pragma omp parallel for schedule(dynamic)
				for(int i = 0; i < n_vertex_num; ++ i) {
					const std::vector<size_t> &r_observations = obs_of_landmark[i];
					if(r_observations.empty())
						continue;
					// skip cameras (and any unobserved landmarks)

					color_samples.clear();
					for(size_t j = 0, m = r_observations.size(); j < m; ++ j) {
						size_t n_obs = r_observations[j];
						color_samples.insert(color_samples.end(), observation_color_samples[n_obs].begin(),
							observation_color_samples[n_obs].end());
					}
					_ASSERTE(!color_samples.empty()); // otherwise color_samples[0] below would fail
					// concatenate all the observation colors

					size_t n_total_obs = color_samples.size();
					if(ranks_r.size() < n_total_obs) ranks_r.clear(); ranks_r.resize(n_total_obs);
					if(ranks_g.size() < n_total_obs) ranks_g.clear(); ranks_g.resize(n_total_obs);
					if(ranks_b.size() < n_total_obs) ranks_b.clear(); ranks_b.resize(n_total_obs);
					stl_ut::IOTA(ranks_r.begin(), ranks_r.end());
					stl_ut::IOTA(ranks_g.begin(), ranks_g.end());
					stl_ut::IOTA(ranks_b.begin(), ranks_b.end());
					std::sort(ranks_r.begin(), ranks_r.end(), stl_ut::CCompare_Indirect<Vector3ub,
						CCompareVecElem<2> >(&color_samples[0], color_samples.size()));
					std::sort(ranks_g.begin(), ranks_g.end(), stl_ut::CCompare_Indirect<Vector3ub,
						CCompareVecElem<1> >(&color_samples[0], color_samples.size()));
					std::sort(ranks_b.begin(), ranks_b.end(), stl_ut::CCompare_Indirect<Vector3ub,
						CCompareVecElem<0> >(&color_samples[0], color_samples.size()));
					// sort the components separately

					// ranks_x[i] now contain position of color i in the sequence sorted by channel x

					size_t n_best_color = 0;
					const int n_median = int(n_total_obs / 2);
					size_t n_best_distance = abs(ranks_r.front() - n_median) +
						abs(ranks_g.front() - n_median) + abs(ranks_b.front() - n_median); // should we weight this? should we transform the colors to chrominance/luminance? lossless lifting would work nicely here
					for(size_t j = 1, m = r_observations.size(); j < m; ++ j) {
						size_t n_distance = abs(ranks_r[j] - n_median) +
							abs(ranks_g[j] - n_median) + abs(ranks_b[j] - n_median);
						if(n_distance < n_best_distance) {
							n_best_distance = n_distance;
							n_best_color = j;
						}
					}
					// pick a color with minimum sum of distances from the median in all three channels
					// the alternative would be to sort the colors by their luminance

					vertex_color_list[n_first_dest_vertex + i] = color_samples[n_best_color];
					// produce vertex colors
				}
			}

			printf("\r\tprocessed " PRIsize " / " PRIsize " vertices",
				vertex_range_list[k].first + vertex_range_list[k].second, vertex_list.size());
		}
		printf("\n\tdone\n");

		return true;
	}

	static bool Load_PPM(const char *p_s_filename, std::vector<Vector3ub>::iterator p_begin_it,
		std::vector<Vector3ub>::iterator p_end_it)
	{
		FILE *p_fr;
#if defined(_MSC_VER) && !defined(__MWERKS__) && _MSC_VER >= 1400
		if(fopen_s(&p_fr, p_s_filename, "rb"))
#else //_MSC_VER && !__MWERKS__ && _MSC_VER >= 1400
		if(!(p_fr = fopen(p_s_filename, "rb")))
#endif //_MSC_VER && !__MWERKS__ && _MSC_VER >= 1400
			return false;
		// open for reading (binary!)

		const size_t n_vertex_num = p_end_it - p_begin_it;

		int n_format, n_gv_version, n_scale;
		size_t n_side, n_side2, n_stored_vertex_num;
		if(fscanf(p_fr, "P%d\n#gvvc%d-" PRIsize "\n" PRIsize " " PRIsize "\n%d",
		   &n_format, &n_gv_version, &n_stored_vertex_num, &n_side, &n_side2, &n_scale) != 6) {
			fprintf(stderr, "warning: file \'%s\': not written by graphview\n", p_s_filename);
			n_gv_version = -1;
			fseek(p_fr, 0, SEEK_SET);
			if(fscanf(p_fr, "P%d\n" PRIsize " " PRIsize "\n%d",
			   &n_format, &n_side, &n_side2, &n_scale) != 4) { // todo - have to handle comments, need to read line by line
				fprintf(stderr, "error: file \'%s\': bad PPM: failed to read header\n", p_s_filename);
				fclose(p_fr);
				return false;
			}
			n_stored_vertex_num = n_side * n_side2;
		}
		if(n_format != 6 || n_scale != 255) {
			fprintf(stderr, "error: file \'%s\': bad PPM: only P6 / 255 files supported\n", p_s_filename);
			fclose(p_fr);
			return false;
		}
		if(n_stored_vertex_num > n_side * n_side2) {
			fprintf(stderr, "error: file \'%s\': expected to store " PRIsize
				" vertex colors but only contains " PRIsize "\n", p_s_filename,
				n_stored_vertex_num, n_side * n_side2);
			fclose(p_fr);
			return false;
		}
		if(n_stored_vertex_num < n_vertex_num) {
			fprintf(stderr, "error: file \'%s\': expected to store " PRIsize
				" vertex colors but only stored " PRIsize "\n", p_s_filename,
				n_vertex_num, n_stored_vertex_num);
			fclose(p_fr);
			return false;
		}
		if(n_stored_vertex_num != n_vertex_num) {
			fprintf(stderr, "warning: file \'%s\': expected to store " PRIsize
				" vertex colors but actually stores " PRIsize "\n", p_s_filename,
				n_vertex_num, n_stored_vertex_num);
		}
		char n_newline;
		fread(&n_newline, sizeof(char), 1, p_fr);
		if(!isspace(n_newline)) {
			fprintf(stderr, "warning: file \'%s\': unexpected information in "
				"the header, pixel skew likely\n", p_s_filename);
		}
		// read the header

		fread(&(*p_begin_it).x, sizeof(Vector3ub), n_vertex_num, p_fr);
		// read the binary data

		if(ferror(p_fr)) {
			fclose(p_fr);
			return false;
		}
		fclose(p_fr);
		return true;
	}

	static bool Save_PPM(const char *p_s_filename, std::vector<Vector3ub>::const_iterator p_begin_it,
		std::vector<Vector3ub>::const_iterator p_end_it)
	{
		FILE *p_fw;
#if defined(_MSC_VER) && !defined(__MWERKS__) && _MSC_VER >= 1400
		if(fopen_s(&p_fw, p_s_filename, "wb"))
#else //_MSC_VER && !__MWERKS__ && _MSC_VER >= 1400
		if(!(p_fw = fopen(p_s_filename, "wb")))
#endif //_MSC_VER && !__MWERKS__ && _MSC_VER >= 1400
			return false;
		// open for writing (binary!)

		const size_t n_vertex_num = p_end_it - p_begin_it;

		const size_t n_side = max(1, int(ceil(sqrt(double(n_vertex_num)))));
		const size_t n_side2 = (n_vertex_num + n_side - 1) / n_side; // round up
		_ASSERTE(n_side * n_side2 >= n_vertex_num);
		// need to save it as a tile; most image editors have problems with excessively tall narrow images :-/
		// just scan the image out in data order to get the vertex colors, throw the rest away

		fprintf(p_fw, "P6\n#gvvc10-" PRIsize "\n" PRIsize " " PRIsize "\n255\n",
			n_vertex_num, n_side, n_side2);
		// write the image out as a long column

		fwrite(&(*p_begin_it).x, sizeof(Vector3ub), n_vertex_num, p_fw);
		// write the binary data

		size_t n_pad = n_side * n_side2 - n_vertex_num;
		Vector3ub v_pad(0x00, 0xff, 0x00);
		for(size_t i = 0; i < n_pad; ++ i) {
			if(!(i % 10))
				v_pad = v_pad.v_zxy(); // rotate
			fwrite(&v_pad.x, sizeof(Vector3ub), 1, p_fw);
		}
		// write padding

		if(ferror(p_fw)) {
			fclose(p_fw);
			return false;
		}
		return !fclose(p_fw);
	}

	bool Load_VertexColors(const char *p_s_filename)
	{
		//_ASSERTE(edge_range_list.size() == vertex_range_list.size()); // not sre why this was important
		size_t n_loaded = 0;
		bool b_result = true;
		if(vertex_range_list.size() > 1 && vertex_color_list.size() == vertex_list.size()) {
			std::string s_pattern = p_s_filename, s_filename;
			if(!CGLThreadedFrameWriter::Make_Pattern(s_pattern)) {
				fprintf(stderr, "error: multiple color frames to be loaded but the input path is not a valid pattern\n");
				return false;
			}
			for(size_t i = 0, n = vertex_range_list.size(); i < n; ++ i) {
				if(!stl_ut::Format(s_filename, s_pattern.c_str(), i) ||
				   !Load_PPM(s_filename.c_str(), vertex_color_list.begin() +
				   vertex_range_list[i].first, vertex_color_list.begin() +
				   vertex_range_list[i].first + vertex_range_list[i].second))
					b_result = false;
				else
					++ n_loaded;
			}
		} else {
			if(strchr(p_s_filename, '#') != 0)
				fprintf(stderr, "warning: a single color frame to be loaded but the input path resembles a pattern\n");
			b_result = Load_PPM(p_s_filename, vertex_color_list.begin(), vertex_color_list.end());
			if(b_result)
				++ n_loaded;
		}
		if(!n_loaded)
			vertex_color_list.clear();
		printf("loaded " PRIsize " vertex color frames\n", n_loaded);
		return b_result;
	}

	bool Save_VertexColors(const char *p_s_filename) const
	{
		if(vertex_color_list.empty())
			return false;
		//_ASSERTE(edge_range_list.size() == vertex_range_list.size()); // not sre why this was important
		if(vertex_range_list.size() > 1 && vertex_color_list.size() == vertex_list.size()) {
			std::string s_pattern = p_s_filename, s_filename;
			if(!CGLThreadedFrameWriter::Make_Pattern(s_pattern)) {
				fprintf(stderr, "error: multiple color frames to be saved but the output path is not a valid pattern\n");
				return false;
			}
			for(size_t i = 0, n = vertex_range_list.size(); i < n; ++ i) {
				if(!stl_ut::Format(s_filename, s_pattern.c_str(), i) ||
				   !Save_PPM(s_filename.c_str(), vertex_color_list.begin() +
				   vertex_range_list[i].first, vertex_color_list.begin() +
				   vertex_range_list[i].first + vertex_range_list[i].second))
					return false;
			}
			return true;
		} else {
			if(strchr(p_s_filename, '#') != 0)
				fprintf(stderr, "warning: a single color frame to be saved but the output path resembles a pattern\n");
			return Save_PPM(p_s_filename, vertex_color_list.begin(), vertex_color_list.end());
		}
	}

private:
	CSolutionData(const CSolutionData &r_other); // no-copy
	CSolutionData operator =(const CSolutionData &r_other); // no-copy
};

class CMarginalsData {
public:
	std::vector<std::pair<size_t, size_t> > cov_range_list; /**< @brief list of covariance ranges (one per solution frame) */
	std::vector<float> cov_list; /**< @brief list of vertex uncertainities */

public:
	CMarginalsData()
	{}

	void Clear()
	{
		cov_range_list.clear();
		cov_list.clear();
	}

	static inline float f_safe_log(float x)
	{
		return log(max(double(x), 1e-15));
	}

	void QualityAssessment(const CSolutionData &r_solution, bool b_covs_scale_exact,
		bool b_covs_scale_inverse, float f_qa_tile_size = .1f, float f_qa_density_weight = 0)
	{
		cov_list.clear();
		cov_list.resize(r_solution.vertex_list.size());
		cov_range_list = r_solution.vertex_range_list;
		// the same size as the vertex list

		for(size_t n_range = 0, n_range_num = cov_range_list.size(); n_range < n_range_num; ++ n_range) {
			const size_t n_first_vertex = cov_range_list[n_range].first;
			const size_t n_last_vertex = cov_range_list[n_range].second + n_first_vertex;

			Vector3f v_min(0, 0, 0), v_max(0, 0, 0);
			{
				size_t i;
				for(i = n_first_vertex; i < n_last_vertex; ++ i) {
					if(!r_solution.vertex_list[i].b_Has_Position() || r_solution.vertex_list[i].b_Has_Pose())
						continue;
					v_min = v_max = r_solution.vertex_list[i].v_Position();
					break;
				}
				for(++ i; i < n_last_vertex; ++ i) {
					if(!r_solution.vertex_list[i].b_Has_Position() || r_solution.vertex_list[i].b_Has_Pose())
						continue;
					Vector3f v = r_solution.vertex_list[i].v_Position();
					for(int n = 0; n < 3; ++ n) {
						if(v_min[n] > v[n])
							v_min[n] = v[n];
						if(v_max[n] < v[n])
							v_max[n] = v[n];
					}
				}
			}
			// calculate bounding box

			const Vector3f v_size = (v_max - v_min) / f_qa_tile_size;
			const Vector3i v_dims(int(ceil(v_size.x)), int(ceil(v_size.y)), int(ceil(v_size.z)));
			const Vector3i v_idx(1, v_dims.x, v_dims.x * v_dims.y);
			// calculate the size of a uniform grid

			printf("dims(%d, %d, %d) // max = %d\n", v_dims.x, v_dims.y, v_dims.z, v_dims.x * v_dims.y * v_dims.z);

			std::vector<Vector3f> grad_colors;

			{
				const uint32_t *p_grad = CGradientImage::p_Decompress();
				for(size_t i = 0, h = CGradientImage::n_Height(), w = CGradientImage::n_Width(); i < h; ++ i) {
					uint32_t n_color = p_grad[0 + i * w];
					int r = (n_color >> 16) & 0xff;
					int g = (n_color >> 8) & 0xff;
					int b = n_color & 0xff;
					grad_colors.push_back(Vector3f(b, g, r) / 255.0f);
				}
				delete[] p_grad;
			}
			// load gradient

			size_t n_cluster_count = 0;
			std::vector<int> rev_cluster_map;
			std::map<int, std::pair<size_t, std::vector<Vector3f> > > point_clusters;
			for(size_t i = n_first_vertex; i < n_last_vertex; ++ i) {
				if(!r_solution.vertex_list[i].b_Has_Position() || r_solution.vertex_list[i].b_Has_Pose())
					continue;
				Vector3f v = r_solution.vertex_list[i].v_Position();
				v -= v_min;
				v /= f_qa_tile_size;
				Vector3i vi(int(v.x), int(v.y), int(v.z));
				int n_idx = vi.f_Dot(v_idx); // calculate index
				if(!point_clusters.count(n_idx)) {
					point_clusters[n_idx] = std::make_pair(n_cluster_count, std::vector<Vector3f>());
					rev_cluster_map.push_back(n_idx);
					++ n_cluster_count;
				}
				v = r_solution.vertex_list[i].v_Position(); // !!
				point_clusters[n_idx].second.push_back(v);
			}
			// build point clusters

			_ASSERTE(point_clusters.size() == n_cluster_count);
			printf("generated " PRIsize " clusters\n", n_cluster_count);

			size_t n_min_density = 0, n_max_density = 1;
			if(!point_clusters.empty() && f_qa_density_weight != 0) {
				n_min_density = n_max_density = (*point_clusters.begin()).second.second.size();
				if(b_covs_scale_exact) {
					for(std::map<int, std::pair<size_t, std::vector<Vector3f> > >::const_iterator
					   p_cluster_it = point_clusters.begin(), p_end_it = point_clusters.end();
					   p_cluster_it != p_end_it; ++ p_cluster_it) {
						size_t n_density = (*p_cluster_it).second.second.size();
						if(n_min_density > n_density)
							n_min_density = n_density;
						if(n_max_density < n_density)
							n_max_density = n_density;
					}
				} else if(!point_clusters.empty()) {
					std::vector<size_t> densities;
					densities.reserve(point_clusters.size());
					for(std::map<int, std::pair<size_t, std::vector<Vector3f> > >::const_iterator
					   p_cluster_it = point_clusters.begin(), p_end_it = point_clusters.end();
					   p_cluster_it != p_end_it; ++ p_cluster_it)
						densities.push_back((*p_cluster_it).second.second.size());
					std::sort(densities.begin(), densities.end());
					n_min_density = densities[densities.size() / 10];
					n_max_density = densities[densities.size() - 1 - densities.size() / 10];
					// 10% soft min/max
				}
				printf("cluster density " PRIsize " - " PRIsize "\n", n_min_density, n_max_density);
			}
			// calculate min / max point density if needed
			// note that the min density is low quality due to the border clusters (could do a soft min)

			std::vector<Vector4f> normals(point_clusters.size());
			std::vector<float> norm_densities((f_qa_density_weight != 0)? point_clusters.size() : 0);
			for(size_t i = 0, n = point_clusters.size(); i < n; ++ i) {
				/*normals[i].x = rand() / float(RAND_MAX);
				normals[i].y = rand() / float(RAND_MAX);
				normals[i].z = rand() / float(RAND_MAX);
				normals[i] -= .5f;
				normals[i].Normalize(.5f); // make it shiny
				normals[i] += .5f;*/
				// debug - random colors, as many as clusters

				int n_idx = rev_cluster_map[i];
				// get cluster idx

				const std::vector<Vector3f> &r_points = point_clusters[n_idx].second;
				_ASSERTE(!r_points.empty()); // otherwise it wouldnt have been added to the map
				Vector3f v_mean = r_points.front();
				for(size_t j = 1, m = r_points.size(); j < m; ++ j)
					v_mean += r_points[j];
				v_mean /= float(r_points.size());
				// calculate mean

				Matrix3f R;
				R.SetZero();
				for(size_t j = 0, m = r_points.size(); j < m; ++ j) {
					Vector3f v_eccentricity = r_points[j] - v_mean;
					for(int l = 0; l < 3; ++ l) {
						for(int k = 0; k < 3; ++ k)
							R[l][k] += v_eccentricity[k] * v_eccentricity[l];
					}
				}
				// calculate covariance

				Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eigensolver((Eigen::Map<Eigen::Matrix3f>(&R[0][0]))); // msvc needs the extra pair of parentheses to avoid vexing parse
				Eigen::Vector3f normal = eigensolver.eigenvectors().col(1).cross(eigensolver.eigenvectors().col(2)); // cross product of the two largest eigenvalues
				//Eigen::Vector3f normal = eigensolver.eigenvectors().col(2); // eigenvector, corresponding to the *greatest* eigenvalue
				Vector3f v_normal(normal(0), normal(1), normal(2));
				v_normal.Normalize();
				if(v_normal.y < 0)
					v_normal = -v_normal; // unambiguate
				// calculate normal

				//float f_dist = 0;
				//for(size_t j = 0, m = r_points.size(); j < m; ++ j)
				//	f_dist += v_normal.f_Dot(r_points[j]);
				//f_dist /= r_points.size();
				float f_dist = v_normal.f_Dot(v_mean);
				// finalize the plane equation

				//v_normal.Normalize(.5f);
				//v_normal += .5f;
				// color code

				normals[i] = Vector4f(v_normal, -f_dist);
				// get normal

				if(f_qa_density_weight != 0) {
					norm_densities[i] = max(.0f, min(1.0f, float(pow(double(r_points.size() -
						n_min_density) / n_max_density, 1.0 / 3)))); // per unit of space
				}
				// get density
			}
			// calculate normals / planes

			printf("calculating errors ...\n");

			const float f_high_cov = 10000;

			for(size_t i = n_first_vertex; i < n_last_vertex; ++ i) {
				if(!r_solution.vertex_list[i].b_Has_Position() || r_solution.vertex_list[i].b_Has_Pose()) {
					cov_list[i] = f_high_cov;
					continue;
				}
				Vector3f v = r_solution.vertex_list[i].v_Position();
				v -= v_min;
				v /= f_qa_tile_size;
				Vector3i vi(int(v.x), int(v.y), int(v.z));
				int n_idx = vi.f_Dot(v_idx); // calculate index

				v = r_solution.vertex_list[i].v_Position(); // !!
				Vector4f v_plane = normals[point_clusters[n_idx].first];
				float f_dist = /*fabs*/(v_plane.v_xyz().f_Dot(v) + v_plane.w);
				//f_dist /= f_side;
				f_dist *= f_dist;
				cov_list[i] = f_dist;
			}
			// calculate squared vertex distances

			std::sort(cov_list.begin() + n_first_vertex, cov_list.begin() + n_last_vertex);
			// sort inplace (destroying the content)

			std::vector<float>::const_iterator p_min_dist_it = cov_list.begin() + n_first_vertex;
			std::vector<float>::const_iterator p_max_dist_it = std::unique(cov_list.begin() +
				n_first_vertex, std::lower_bound(cov_list.begin() +
				n_first_vertex, cov_list.begin() + n_last_vertex, f_high_cov));
			// use this for soft max, make sure to skip f_high_cov entries (points with no position)

			size_t n_dist_num = p_max_dist_it - p_min_dist_it;
			float f_min_dist = (b_covs_scale_exact)? *p_min_dist_it : *(p_min_dist_it + (n_dist_num / 10));
			float f_max_dist = (b_covs_scale_exact)? *(p_max_dist_it - 1) : *(p_max_dist_it - (1 + n_dist_num / 10));
			// find min / max

			float f_dist_scale = 1 / (f_max_dist - f_min_dist);

			for(size_t i = n_first_vertex; i < n_last_vertex; ++ i) {
				if(!r_solution.vertex_list[i].b_Has_Position() || r_solution.vertex_list[i].b_Has_Pose()) {
					cov_list[i] = f_high_cov;
					continue;
				}
				Vector3f v = r_solution.vertex_list[i].v_Position();
				v -= v_min;
				v /= f_qa_tile_size;
				Vector3i vi(int(v.x), int(v.y), int(v.z));
				int n_idx = vi.f_Dot(v_idx); // calculate index

				v = r_solution.vertex_list[i].v_Position(); // !!
				Vector4f v_plane = normals[point_clusters[n_idx].first];
				float f_dist = /*fabs*/(v_plane.v_xyz().f_Dot(v) + v_plane.w);
				//f_dist /= f_side;
				f_dist *= f_dist;

				float f_dist_error = max(0, min(1, (f_dist - f_min_dist) * f_dist_scale));
				float f_density_error = (f_qa_density_weight != 0)? 1 - norm_densities[point_clusters[n_idx].first] : 0;

				cov_list[i] = f_dist_error + (f_density_error - f_dist_error) * f_qa_density_weight;

				if(b_covs_scale_inverse)
					cov_list[i] = -cov_list[i];
			}
			// recalculate squared vertex distances, assign vertex precisions
		}
	}

	bool Load_MargsFile(const char *p_s_filename, const std::vector<std::pair<size_t, size_t> > &vertex_range_list,
		bool b_covs_scale_exact, bool b_covs_scale_inverse, bool b_covs_log_scale, bool b_covs_scale_all,
		bool b_covs_scale_soft_min, float f_covs_scale_offset, float f_covs_scale_factor, size_t n_parse_limit = 0,
		bool b_relative_pose_graph = false) // rpg is probably rubbish
	{
		cov_list.clear();
		cov_range_list.clear();

		const float f_high_cov = 10000;
		const float f_max_cov = 2500;

		std::string s_pattern = p_s_filename;
		if(strchr(p_s_filename, '#') && CGLThreadedFrameWriter::Make_Pattern(s_pattern)) {
			CTimer time;
			uint64_t n_files_size = 0, n_total_size = 0;

			std::string s_file;
			for(size_t i = 0;; ++ i) { // no limit
				stl_ut::Format(s_file, s_pattern.c_str(), i);
				// generate a filename

				size_t n_range_start = cov_list.size();

				FILE *p_fr;
#if defined(_MSC_VER) && !defined(__MWERKS__) && _MSC_VER >= 1400
				if(fopen_s(&p_fr, s_file.c_str(), "r"))
#else //_MSC_VER && !__MWERKS__ && _MSC_VER >= 1400
				if(!(p_fr = fopen(s_file.c_str(), "r")))
#endif //_MSC_VER && !__MWERKS__ && _MSC_VER >= 1400
					break;
				std::string s_line;
				std::vector<std::string> number_list;
				while(!feof(p_fr)) {
					if(!stl_ut::ReadLine(s_line, p_fr)) {
						fclose(p_fr);
						return false;
					}
					stl_ut::TrimSpace(s_line);
					if(s_line.empty())
						continue;
					// read a single line

					stl_ut::Split(number_list, s_line, " ", 0);
					// split it to numbers

					std::vector<float> marginal_diag(number_list.size()); // throws
					for(size_t i = 0; i < marginal_diag.size(); ++ i)
						marginal_diag[i] = atof(number_list[i].c_str());
					float f_cov = 0;
					if((b_relative_pose_graph && (marginal_diag.size() == 6 || marginal_diag.size() == 7)) || // relative poses
					   marginal_diag.size() == 3 || marginal_diag.size() == 4 || marginal_diag.size() == 1) { // 4 = inverse depth, 1 = inverse dist or something
						for(size_t i = 0, n = marginal_diag.size(); i < n; ++ i)
							f_cov += marginal_diag[i] * marginal_diag[i];
						f_cov = sqrt(f_cov);
					} else
						f_cov = f_high_cov;
					cov_list.push_back(f_cov); // throws
					// parse a vertex, put it in the list

					if(n_parse_limit && cov_list.size() == n_parse_limit)
						break;
					// handle limit
				}
				fclose(p_fr);
				// parse the file

				size_t n_frame_vertex_num = cov_list.size() - n_range_start;
				if(!cov_range_list.empty() && cov_range_list.back().second > n_frame_vertex_num) {
					fprintf(stderr, "error: covariance frame " PRIsize " smaller than the previous frame ("
						PRIsize " < " PRIsize ")\n", cov_range_list.size(), n_frame_vertex_num,
						cov_range_list.back().second);
					break;
				}
				if(cov_range_list.size() >= vertex_range_list.size()) {
					fprintf(stderr, "error: more covariance frames than vertex frames; ignoring subsequent frames\n");
					break;
				}
				if(vertex_range_list[i].second != n_frame_vertex_num) { // we count on the number of vertices without covariances to not decrease in time; this check is too strict, could really check if the nomber of covless vertices does not decrease
					fprintf(stderr, "error: covariance frame " PRIsize " smaller than the vertex frame ("
						PRIsize " < " PRIsize ")\n", cov_range_list.size(), n_frame_vertex_num,
						vertex_range_list.back().second);
					break;
				}
				// we will count on the next solution frames generally increase in size
				// we also count on the covariances matching the vertices quite closely
				// also, we count on the number of vertices without covariances to not decrease in time

				cov_range_list.push_back(std::make_pair(n_range_start, n_frame_vertex_num));
				// add this as another range

				/*drawable_vertex_list_nc.push_back(n_point_num);
				drawable_pose_list.push_back(n_pose_num);*/
				// maintain stats

				if(n_parse_limit && cov_list.size() - n_range_start == n_parse_limit)
					break;
				// handle limit

				n_files_size += TFileInfo(s_file).n_Size64();
				if(cov_range_list.size() >= 16) {
					if(!n_total_size) {
						n_total_size = n_files_size;
						for(size_t j = i + 1;; ++ j) { // no limit
							stl_ut::Format(s_file, s_pattern.c_str(), j);
							TFileInfo t_file(s_file);
							if(!t_file.b_exists)
								break;
							n_total_size += t_file.n_Size64();
						}
					}
					// calculate size of all the files

					double f_elapsed = time.f_Time();
					double f_speed = n_files_size / f_elapsed;
					double f_estimated = n_total_size / f_speed;
					double f_remaining = max(0, f_estimated - f_elapsed);
					printf("loaded " PRIsize " covariance frames (" PRIsizeB "B/sec, " PRItime ")\r",
						cov_range_list.size(), PRIsizeBparams(f_speed), PRItimeparams(f_remaining));
				} else
					printf("loaded " PRIsize " covariance frames\r", cov_range_list.size());
				// show some progress
			}
			printf("loaded " PRIsize " covariance frames (" PRIsizeB "B, " PRIsizeB "B/sec)%10s\n",
				cov_range_list.size(), PRIsizeBparams(n_files_size), PRIsizeBparams(n_files_size / time.f_Time()), "");
			printf("have " PRIsize " covariances (" PRIsize " covariances overall, " PRIsizeB "B of data)\n",
				(cov_range_list.empty())? 0 : cov_range_list.back().second, cov_list.size(),
				PRIsizeBparams(cov_list.size() * sizeof(float)));
		} else {
			FILE *p_fr;
#if defined(_MSC_VER) && !defined(__MWERKS__) && _MSC_VER >= 1400
			if(fopen_s(&p_fr, p_s_filename, "r"))
#else //_MSC_VER && !__MWERKS__ && _MSC_VER >= 1400
			if(!(p_fr = fopen(p_s_filename, "r")))
#endif //_MSC_VER && !__MWERKS__ && _MSC_VER >= 1400
				return false;
			std::string s_line;
			std::vector<std::string> number_list;
			while(!feof(p_fr)) {
				if(!stl_ut::ReadLine(s_line, p_fr)) {
					fclose(p_fr);
					return false;
				}
				stl_ut::TrimSpace(s_line);
				if(s_line.empty())
					continue;
				// read a single line

				stl_ut::Split(number_list, s_line, " ", 0);
				// split it to numbers

				std::vector<float> marginal_diag(number_list.size()); // throws
				for(size_t i = 0; i < marginal_diag.size(); ++ i)
					marginal_diag[i] = atof(number_list[i].c_str());
				float f_cov = 0;
				if((b_relative_pose_graph && (marginal_diag.size() == 6 || marginal_diag.size() == 7)) || // relative poses
				   marginal_diag.size() == 3 || marginal_diag.size() == 4 || marginal_diag.size() == 1) { // 4 = inverse depth, 1 = inverse dist or something
					for(size_t i = 0, n = marginal_diag.size(); i < n; ++ i)
						f_cov += marginal_diag[i] * marginal_diag[i];
					f_cov = sqrt(f_cov);
				} else
					f_cov = f_high_cov;
				cov_list.push_back(f_cov); // throws
				// parse a vertex, put it in the list

				if(n_parse_limit && cov_list.size() == n_parse_limit)
					break;
				// handle limit
			}
			fclose(p_fr);
			// parse the file

			cov_range_list.push_back(std::make_pair(0, cov_list.size()));
		}

		if(b_relative_pose_graph) {
			/*for(size_t i = 0, n = cov_range_list.size(); i < n; ++ i) {
				double f_last_cam_cov = 0;
				const size_t vf = vertex_range_list[i].first, n_solution_verts = vertex_range_list[i].second - vf;
				const size_t b = cov_range_list[i].first, e = cov_range_list[i].second;
				for(size_t v = b; v < e; ++ v) {
					if(vertex_list[v - b + vf].b_Has_Pose())
						f_last_cam_cov = cov_list[v];
					else if(vertex_list[v - b + vf].b_Has_Position())
						cov_list[v] += f_last_cam_cov;
				}
			}*/ // seems like a bad idea
			// treat marginals in a graph with relative coordinates
		}

#if 1 // "normal" code
		_ASSERTE(cov_range_list.empty() == cov_list.empty());
		if(!cov_range_list.empty()) {
			std::vector<float> covs_copy(cov_list.begin() + ((b_covs_scale_all)? 0 : cov_range_list.back().first),
				cov_list.begin() + ((b_covs_scale_all)? cov_list.size() : cov_range_list.back().first + cov_range_list.back().second)); // throws
			// use only the last solution frame

			std::sort(covs_copy.begin(), covs_copy.end());
			covs_copy.erase(std::lower_bound(covs_copy.begin(), covs_copy.end(), f_high_cov), covs_copy.end());
			float f_min = (b_covs_scale_exact)? ((covs_copy.empty())? 0 : covs_copy.front()) :
				((b_covs_scale_soft_min)? ((covs_copy.empty())? 0 : covs_copy[size_t(covs_copy.size() * .01)]) : 0);
			float f_max = (b_covs_scale_exact)? min(f_max_cov, (covs_copy.empty())? 1 : covs_copy.back()) : // exact
				//((b_covs_scale_unscented)? 0 : f * .25f) + ((b_covs_scale_unscented)? 1 : .75f) *
				f_covs_scale_offset * f_covs_scale_factor + (1 - f_covs_scale_factor) *
				min(f_max_cov, (covs_copy.empty())? 1 : covs_copy[size_t(covs_copy.size() * .90)]); // "nice"
			printf("max cov: %g\n", f_max);

			if(b_covs_log_scale) {
				f_max = f_safe_log(f_max);
				f_min = f_safe_log(f_min);
			}

			float f_scl = 1.0f / (f_max - f_min);
			float f_off = -f_min;
			for(size_t i = 0, n = cov_list.size(); i < n; ++ i) {
				cov_list[i] = (cov_list[i] == f_high_cov)? f_high_cov :
					(((b_covs_log_scale)? f_safe_log(cov_list[i]) : cov_list[i]) + f_off) * f_scl;
				if(b_covs_scale_inverse && cov_list[i] != f_high_cov)
					cov_list[i] = 1 - cov_list[i];
			}
			// scale the covs to 0-1 range
		}
#else // 0 // klemen's code
	_ASSERTE(cov_range_list.empty() == cov_list.empty());
	if(!cov_range_list.empty()) {
		static bool b_covs_scale_soft_min = true;
		static float f_covs_scale_offset = 0;//480.531;
		static float f_covs_scale_factor = 0.9;//.25f;
		b_covs_scale_all = true;
		std::vector<float> covs_copy(cov_list.begin() + ((b_covs_scale_all)? 0 : cov_range_list.back().first),
			cov_list.begin() + ((b_covs_scale_all)? cov_list.size() : cov_range_list.back().first + cov_range_list.back().second)); // throws
		// use only the last solution frame

		std::sort(covs_copy.begin(), covs_copy.end());
		//covs_copy.erase(std::lower_bound(covs_copy.begin(), covs_copy.end(), f_high_cov), covs_copy.end());
		//float f_min = (b_covs_scale_exact)? ((covs_copy.empty())? 0 : covs_copy.front()) : 0;//(covs_copy.empty())? 0 : covs_copy[size_t(covs_copy.size() * .01)];
		//float f_max = (b_covs_scale_exact)? min(f_max_cov, (covs_copy.empty())? 1 : covs_copy.back()) : // exact
		//	480.531f * .25f + .75f * min(f_max_cov, (covs_copy.empty())? 1 : covs_copy[size_t(covs_copy.size() * .90)]); // "nice"
		//printf("max cov: %g\n", f_max);

		bool b_covs_scale_unscented = true;
		b_covs_scale_exact = false;

		covs_copy.erase(std::lower_bound(covs_copy.begin(), covs_copy.end(), f_high_cov), covs_copy.end());

		float f_min = (b_covs_scale_exact)? ((covs_copy.empty())? 0 : covs_copy.front()) : ((b_covs_scale_soft_min)? ((covs_copy.empty())? 0 : covs_copy[size_t(covs_copy.size() * .05)]) : 0);
		float f_max = (b_covs_scale_exact)? min(f_max_cov, (covs_copy.empty())? 1 : covs_copy.back()) : // exact
			((b_covs_scale_unscented)? 0 : .25f) + ((b_covs_scale_unscented)? 1 : .75f) *f_covs_scale_offset * f_covs_scale_factor + (1 - f_covs_scale_factor) *min(f_max_cov, (covs_copy.empty())? 1 : covs_copy[size_t(covs_copy.size()*0.999)]); // "nice"

		printf("limits covA: %g :: %g\n", f_min,f_max);

		float offset_f_min = 0.5 * (f_max-f_min);

		printf("limits offset: %g \n", offset_f_min);

		if(b_covs_log_scale) {
			f_max = f_safe_log(f_max);
			f_min = f_safe_log(f_min);
		}

		printf("limits scale: %g :: %g\n", (offset_f_min/(f_max-f_min)),(offset_f_min/(f_max-f_min)) / (f_max - (f_min)));
		float f_scl = 1 / (f_max - (f_min));
		float f_off = -f_min;
		for(size_t i = 0, n = cov_list.size(); i < n; ++ i) {
			//printf("covA: %g\n", cov_list[i]);

			cov_list[i] = (cov_list[i] > f_high_cov)? 1.0 :
				(((b_covs_log_scale)? f_safe_log(cov_list[i]) : cov_list[i]) + f_off) * f_scl;

			//cov_list[i] = (cov_list[i] > f_max)? 1.0 :
				//(((b_covs_log_scale)? f_safe_log(cov_list[i]) : cov_list[i]) + f_off) * f_scl;

			//printf("covB: %g\n", cov_list[i]);
			if(b_covs_scale_inverse && cov_list[i] != f_high_cov)
				cov_list[i] = 1 - cov_list[i];
		}
		printf("limits cov: %g :: %g\n", f_min,f_max);
		// scale the covs to 0-1 range
	}
#endif // 0

		printf("have " PRIsize " marginal covariances\n", cov_list.size());

		return true;
	}

private:
	CMarginalsData(const CMarginalsData &r_other); // no-copy
	CMarginalsData operator =(const CMarginalsData &r_other); // no-copy
};

#include "GeomStreamers.h"

/*float f_Rand() // unused?
{
	return rand() / float(RAND_MAX) * 2 - 1;
}*/

class CClearFlagUponExit {
protected:
	bool &m_r_b_flag;

public:
	CClearFlagUponExit(bool &r_b_flag)
		:m_r_b_flag(r_b_flag)
	{}

	~CClearFlagUponExit()
	{
		m_r_b_flag = false;
	}
};

static void RenderFrame(double f_time); // forward declaration
static void RenderFrame_Mod(double f_time, const Matrix4f &r_t_view_mod); // forward declaration

class CGLBundle {
protected:
	const TCommandlineArgs m_t_args;

	Vector3f m_v_camera_pos; /**< @brief current camera position */
	float m_f_angle_x; /**< @brief camera rotation about x axis (Euler angle) */
	float m_f_angle_y; /**< @brief camera rotation about y axis (Euler angle) */
	float m_f_angle_z; /**< @brief camera rotation about z axis (Euler angle) */
	CCameraAnimation m_anim; /**< @brief camera animation tool */
	bool m_b_render_running; /**< @brief render in progress flag */
	bool m_b_screenshot_render_running; /**< @brief jittered render in progress flag (only applies if \ref m_b_render_running is also aet) */
	Vector2f m_v_mouse; /**< @brief mouse position in OpenGL coords */
	bool m_b_is_pure_click; // was onMouseMotion() called between mouse down and mouse up?
	int m_n_width; /**< @brief current window width, in pixels */
	int m_n_height; /**< @brief current window height, in pixels */
	std::vector<size_t> m_selection_buffer; /**< @brief contains indices of selected vertices, is sorted and unique */
	bool m_b_selection_lock; /**< @brief selection lock flag (toggled by user) */
	// application state

	bool m_b_display_covs; /**< @brief false-colour precision display flag */
	bool m_b_fancy_color_points; /**< @brief projective textured points display flag */
	bool m_b_display_edges; /**< @brief edges display flag */
	bool m_b_display_cameras; /**< @brief camera frusta display flag */
	bool m_b_edges_blend_add; /**< @brief edges additive blending flag */
	int m_n_background_type; /**< @brief background colour index */
	bool m_b_upside_down; /**< @brief upside down flip flag */
	bool m_b_display_grid; /**< @brief grid display flag */
	float m_f_point_size; /**< @brief 3D point size */
	float m_f_edge_alpha; /**< @brief edge alpha */
	size_t m_n_zoomed_into_camera_vertex; /**< @brief currently zoomed-in pose */
	bool m_b_zoom_into_camera_auto_pause; /**< @brief automatically pause the zoom-in animation */
	float m_f_camera_tex_alpha; /**< @brief camera texture alpha */
	// display settings

	CGrid m_grid; /**< @brief wireframe grid object */
	CGLTexture_2D_FromBitmap<> m_gradient_texture; /**< @brief gradient texture for false-colour rendering */ // t_odo - use CGLTexture instead
	TConstantColorShader m_const_color_shader; /**< @brief shader for constant colour rendering */
	TConstantColorLerpShader m_const_color_lerp_shader; /**< @brief shader for constant colour animated rendering */
	TColorShader m_color_shader; /**< @brief shader for per-vertex colour rendering */
	TColorLerpShader m_color_lerp_shader; /**< @brief shader for per-vertex colour rendering */
	TTextureShader m_texture_shader; /**< @brief shader for false-colour rendering */
	TTextureLerpShader m_texture_lerp_shader; /**< @brief shader for false-colour animated rendering */
	TCamTexProjectionShader m_camera_tex_proj_shader; /**< @brief shader for rendering camera texture projections onto the geometry */
	TCamTexProjectionLerpShader m_camera_tex_proj_lerp_shader; /**< @brief shader for rendering animated camera texture projections onto the geometry */
	//
	TCameraShader m_camera_shader; /**< @brief shader for camera frustum rendering */
	TCameraLerpShader m_camera_lerp_shader; /**< @brief shader for camera frustum animated rendering */
	TCameraTextureShader m_camera_tex_shader; /**< @brief shader for camera texture rendering */
	TCameraTextureLerpShader m_camera_tex_lerp_shader; /**< @brief shader for camera texture rendering */
	// shared OpenGL objects, shaders

	CFancyPointsStreamer m_fap;
	CFlatPointsStreamer m_flp;
	CCameraFrustumStreamer m_cam;
	CCameraTextureStreamer m_ctx;
	CEdgesStreamer m_edg;
	// geometry streamers

	CGLArraySetup *m_p_points_array_selection; /**< @brief vertex array with points with color ids for picking */
	CGLArraySetup *m_p_cameras_array_selection; /**< @brief vertex array with camera frusta with color ids for picking */
	// OpenGL primitives (arrays allocated as needed)

	CSolutionData m_solution;
	CMarginalsData m_marginals;
	CGraphData m_graph;
	// data

	CAsyncImageLoader m_image_loader;
	std::vector<Vector4f> m_cam_texcoord_list; /**< @brief list of texture coordinates for images that each camera sees (points into the megatexture) */
	CGLTexture_2D *m_p_megatexture; /**< @brief pointer to the texture containing atlas of the camera images */
	// megatexture data for rendering camera views

public:
	CGLBundle(TCommandlineArgs t_args) // throw(std::bad_alloc, std::runtime_error)
		:m_t_args(t_args), m_v_camera_pos(t_args.v_camera_pos), m_f_angle_x(t_args.f_angle_x),
		m_f_angle_y(t_args.f_angle_y), m_f_angle_z(t_args.f_angle_z), m_b_render_running(false),
		m_b_screenshot_render_running(false), m_v_mouse(0, 0), m_b_is_pure_click(true),
		m_n_width(t_args.n_width), m_n_height(t_args.n_height), m_b_selection_lock(false),
		m_b_display_covs(t_args.b_display_covs), m_b_fancy_color_points(t_args.b_fancy_color_points),
		m_b_display_edges(t_args.b_display_edges), m_b_display_cameras(t_args.b_display_cameras),
		m_b_edges_blend_add(t_args.b_edges_blend_add), m_n_background_type(t_args.n_background),
		m_b_upside_down(t_args.b_upside_down), m_b_display_grid(t_args.b_display_grid),
		m_f_point_size(t_args.f_point_size), m_f_edge_alpha(t_args.f_edge_alpha),
		m_n_zoomed_into_camera_vertex(size_t(-1)), m_b_zoom_into_camera_auto_pause(false),
		m_f_camera_tex_alpha(t_args.f_camera_tex_alpha), m_grid(2048, t_args.f_grid_spacing),
		m_gradient_texture(p_Load_GradientImage(), GL_RGB, true, true), m_p_points_array_selection(0),
		m_p_cameras_array_selection(0), m_image_loader(t_args.n_texture_streaming_cache_MB * 1048576,
		t_args.n_texture_streaming_thread_num), m_p_megatexture(0)
	{
		m_solution.Load_ResultFile((!t_args.p_s_result_file)? "null" : t_args.p_s_result_file);
		if(m_solution.vertex_range_list.empty())
			m_solution.vertex_range_list.push_back(std::make_pair(size_t(0), size_t(0)));
		m_graph.Load_GraphFile((!t_args.p_s_graph_file)? "null" : t_args.p_s_graph_file, m_solution.n_Max_Vertex_Num(), 0,
			t_args.p_s_colorize_input_images != 0, t_args.p_s_camera_proj_input_images != 0);
		m_graph.Calculate_EdgeRanges(m_solution.vertex_range_list);
		m_graph.Calculate_VertexDegrees(m_solution.vertex_range_list);
		if(t_args.b_fancy_color_points && !m_graph.camera_intrinsics.empty()) {
			float f_fov_degrees = 2 * atan(t_args.n_original_images_width / (2 *
				(*m_graph.camera_intrinsics.begin()).second.fx)) / f_pi * 180;
			printf("debug: field of view of the first camera in the graph was %.2f deg\n", f_fov_degrees);
		}
		if(t_args.p_s_colorize_input_images != 0 && t_args.p_s_graph_file != 0) {
			m_solution.Colorize_PointCloud(t_args.p_s_colorize_input_images, t_args.t_colorize_image_matrix,
				t_args.n_colorize_points_fwidth, m_graph);
		}

		if(!t_args.b_quality_assessment) {
			m_b_display_covs = m_marginals.Load_MargsFile((!t_args.p_s_margs_file)? "null" :
				t_args.p_s_margs_file, m_solution.vertex_range_list, t_args.b_covs_scale_exact,
				t_args.b_covs_scale_inverse, t_args.b_covs_log_scale, t_args.b_covs_scale_all,
				t_args.b_covs_scale_soft_min, t_args.f_covs_scale_offset, t_args.f_covs_scale_factor);
		} else {
			if(t_args.p_s_margs_file)
				fprintf(stderr, "warning: quality assessment requested: marginals file ignored\n");

			m_marginals.QualityAssessment(m_solution, t_args.b_covs_scale_exact,
				t_args.b_covs_scale_inverse, t_args.f_qa_tile_size, t_args.f_qa_density_weight);
			// b_quality_assessment = false;
			//float f_qa_tile_size = .1f, f_qa_density_weight = 0;

			m_b_display_covs = true;
		}
		// load stuff now

		if(t_args.p_s_vertex_colors_export_file) {
			if(m_solution.vertex_color_list.size() == m_solution.vertex_list.size()) {
				if(!m_solution.Save_VertexColors(t_args.p_s_vertex_colors_export_file))
					fprintf(stderr, "error: saving vertex colors failed\n");
			} else {
				if(!Export_FalseVertexColors(t_args.p_s_vertex_colors_export_file))
					fprintf(stderr, "error: saving vertex colors failed\n");
			}
		}
		// export vertex colors in PPM if needed

		if(t_args.p_s_vertex_colors_file) {
			m_solution.vertex_color_list.resize(m_solution.vertex_list.size(), Vector3ub(255, 255, 255));
			m_solution.Load_VertexColors(t_args.p_s_vertex_colors_file);
		}
		// load vertex colors from PPM

		if(t_args.p_s_coff_export_file) {
			if(m_solution.vertex_color_list.size() == m_solution.vertex_list.size()) {
				if(!Export_COFF(m_solution.vertex_list, m_solution.vertex_color_list, t_args.p_s_coff_export_file))
					fprintf(stderr, "error: saving COFF output failed\n");
			} else {
				if(!Export_COFF(m_solution.vertex_list, (m_marginals.cov_list.empty())? m_graph.degree_list :
				   m_marginals.cov_list, t_args.p_s_coff_export_file))
					fprintf(stderr, "error: saving COFF output failed\n");
			}
		}
		// export .coff if neede

		if(m_marginals.cov_list.empty() && !m_graph.degree_list.empty()) {
			printf("no covariances loaded: using vertex degrees for false coloring\n");
			printf("false colors initially disabled\n");
			m_b_display_covs = false;
			m_marginals.cov_list = m_graph.degree_list; // or swap
			_ASSERTE(m_graph.degree_list.size() == m_solution.vertex_list.size());
			m_marginals.cov_range_list = m_solution.vertex_range_list; // must copy
		}
		// use vertex degree in place of covariances

		if(t_args.b_auto_view)
			LookAt_AllPoints();
		// position the camera automatically

		//printf("debug: building shaders ...\n");
		int n_fail_num = 0;
		n_fail_num += (m_const_color_shader.Compile())? 0 : 1;
		n_fail_num += (m_const_color_lerp_shader.Compile())? 0 : 1;
		n_fail_num += (m_camera_shader.Compile())? 0 : 1;
		n_fail_num += (m_camera_tex_shader.Compile())? 0 : 1;
		n_fail_num += (m_camera_tex_lerp_shader.Compile())? 0 : 1;
		n_fail_num += (m_camera_lerp_shader.Compile())? 0 : 1;
		n_fail_num += (m_color_shader.Compile())? 0 : 1;
		n_fail_num += (m_color_lerp_shader.Compile())? 0 : 1;
		n_fail_num += (m_texture_shader.Compile())? 0 : 1;
		n_fail_num += (m_texture_lerp_shader.Compile())? 0 : 1;
		n_fail_num += (m_camera_tex_proj_shader.Compile())? 0 : 1;
		n_fail_num += (m_camera_tex_proj_lerp_shader.Compile())? 0 : 1;
		if(/*!m_const_color_shader.Compile() || !m_const_color_lerp_shader.Compile() ||
		   !m_camera_shader.Compile() || !m_camera_tex_shader.Compile() ||
		   !m_camera_tex_lerp_shader.Compile() || !m_camera_lerp_shader.Compile() ||
		   !m_color_shader.Compile() || !m_color_lerp_shader.Compile() || !m_texture_shader.Compile() ||
		   !m_texture_lerp_shader.Compile() || !m_camera_tex_proj_shader.Compile() ||
		   !m_camera_tex_proj_lerp_shader.Compile()*/n_fail_num)
			fprintf(stderr, "error: %d of the shaders failed to compile\n", n_fail_num); // but try to press on, maybe it will draw something

		if(m_t_args.p_s_camera_proj_input_images && !Initialize_Megatexture())
			throw std::runtime_error("failed to initialize megatexture");
	}

	~CGLBundle()
	{
		Cleanup();
	}

	void Cleanup(bool b_skip_shaders_megatexture = false)
	{
		m_graph.Clear();
		m_solution.Clear();
		m_marginals.Clear();

		m_fap.Free_GLBuffers();
		m_ctx.Free_GLBuffers();
		m_cam.Free_GLBuffers();
		m_flp.Free_GLBuffers();
		m_edg.Free_GLBuffers();

		if(m_p_points_array_selection)
			delete m_p_points_array_selection;
		m_p_points_array_selection = 0;
		if(m_p_cameras_array_selection)
			delete m_p_cameras_array_selection;
		m_p_cameras_array_selection = 0;

		if(!b_skip_shaders_megatexture) {
			if(m_p_megatexture)
				delete m_p_megatexture;
			m_p_megatexture = 0;

			m_const_color_shader.Delete();
			m_const_color_lerp_shader.Delete();
			m_camera_shader.Delete();
			m_camera_lerp_shader.Delete();
			m_camera_tex_shader.Delete();
			m_camera_tex_lerp_shader.Delete();
			m_color_shader.Delete();
			m_color_lerp_shader.Delete();
			m_texture_shader.Delete();
			m_texture_lerp_shader.Delete();
			m_camera_tex_proj_shader.Delete();
			m_camera_tex_proj_lerp_shader.Delete();
		}

		// do not delete m_p_megatexture, m_cam_texcoord_list and color_image_list
		// do not delete m_grid, it is on stack in main()
	}

	void Reload()
	{
		Cleanup(true);
		// delete everything except for shaders and megatexture

		printf("\nreload inputs ...\n");
		m_solution.Load_ResultFile((!m_t_args.p_s_result_file)? "null" : m_t_args.p_s_result_file);
		if(m_solution.vertex_range_list.empty())
			m_solution.vertex_range_list.push_back(std::make_pair(size_t(0), size_t(0)));
		m_graph.Load_GraphFile((!m_t_args.p_s_graph_file)? "null" : m_t_args.p_s_graph_file,
			m_solution.n_Max_Vertex_Num(), 0, m_t_args.p_s_colorize_input_images != 0,
			m_t_args.p_s_camera_proj_input_images != 0);
		m_graph.Calculate_EdgeRanges(m_solution.vertex_range_list);
		m_graph.Calculate_VertexDegrees(m_solution.vertex_range_list);
		if(m_t_args.b_fancy_color_points && !m_graph.camera_intrinsics.empty()) {
			float f_fov_degrees = 2 * atan(m_t_args.n_original_images_width / (2 *
				(*m_graph.camera_intrinsics.begin()).second.fx)) / f_pi * 180;
			printf("debug: field of view of the first camera in the graph was %.2f deg\n", f_fov_degrees);
		}
		if(m_t_args.p_s_colorize_input_images != 0 && m_t_args.p_s_graph_file != 0) {
			m_solution.Colorize_PointCloud(m_t_args.p_s_colorize_input_images, m_t_args.t_colorize_image_matrix,
				m_t_args.n_colorize_points_fwidth, m_graph);
		}
		if(!m_t_args.b_quality_assessment) {
			/*m_b_display_covs =*/ m_marginals.Load_MargsFile((!m_t_args.p_s_margs_file)? "null" :
				m_t_args.p_s_margs_file, m_solution.vertex_range_list, m_t_args.b_covs_scale_exact,
				m_t_args.b_covs_scale_inverse, m_t_args.b_covs_log_scale, m_t_args.b_covs_scale_all,
				m_t_args.b_covs_scale_soft_min, m_t_args.f_covs_scale_offset, m_t_args.f_covs_scale_factor);
			// the user may have changed the display settings, keep her changes
		} else {
			if(m_t_args.p_s_margs_file)
				fprintf(stderr, "warning: quality assessment requested: marginals file ignored\n");

			m_marginals.QualityAssessment(m_solution, m_t_args.b_covs_scale_exact,
				m_t_args.b_covs_scale_inverse, m_t_args.f_qa_tile_size, m_t_args.f_qa_density_weight);
			// b_quality_assessment = false;
			//float f_qa_tile_size = .1f, f_qa_density_weight = 0;

			m_b_display_covs = true;
		}
		// load stuff now

		// [export vertex colors in PPM if needed] // not again

		if(m_t_args.p_s_vertex_colors_file) {
			m_solution.vertex_color_list.resize(m_solution.vertex_list.size(), Vector3ub(255, 255, 255));
			m_solution.Load_VertexColors(m_t_args.p_s_vertex_colors_file);
		}

		// [export .coff if needed] // not again

		if(m_marginals.cov_list.empty() && !m_graph.degree_list.empty()) {
			printf("no covariances loaded: using vertex degrees for false coloring\n");
			/*printf("false colors initially disabled\n");
			m_b_display_covs = false;*/ // the user may have changed this, keep her changes
			m_marginals.cov_list = m_graph.degree_list; // or swap
			_ASSERTE(m_graph.degree_list.size() == m_solution.vertex_list.size());
			m_marginals.cov_range_list = m_solution.vertex_range_list; // must copy
		}
		// use vertex degree in place of covariances

		glutPostRedisplay();
		// repaint

		printf("done\n");
	}

	Matrix4f t_UserCamera() const
	{
		Matrix4f t_camera_matrix;

		t_camera_matrix.Identity();
		t_camera_matrix.RotateZ(m_f_angle_z);
		t_camera_matrix.RotateX(m_f_angle_x);
		t_camera_matrix.RotateY(m_f_angle_y);
		t_camera_matrix.Translate(m_v_camera_pos);
		// calculate final camera matrix (with pitch and roll)

		return t_camera_matrix;
	}

	Matrix4f t_Camera(double f_time)
	{
		if(m_anim.b_Active())
			return m_anim.t_Interpolate(f_time);
		else
			return t_UserCamera();
	}

	static void Perspective_Jitter(Matrix4f &r_t_matrix, float f_fov, float f_aspect,
		float f_near, float f_far, float f_jitter_x, float f_jitter_y) // jitter is in logical [0, 1] coords
	{
		float f_w, f_h;

		f_h = float(tan(f_fov * f_pi / 180 * .5f)) * f_near;
		f_w = f_h * f_aspect;
		// calc half width of frustum in x-y plane at z = f_near

		f_jitter_x *= 2 * f_w;
		f_jitter_y *= 2 * f_h;

		CGLTransform::Frustum(r_t_matrix, -f_w + f_jitter_x, f_w + f_jitter_x,
			-f_h + f_jitter_y, f_h + f_jitter_y, f_near, f_far);
		// calculate frustum
	}

	Matrix4f t_Projection() const
	{
		const float f_near = m_t_args.f_near_view_distance, f_far = m_t_args.f_far_view_distance,
			f_fov = (m_t_args.b_video_omni && m_b_render_running)? 90 : // use 90 for omni rendering
			m_t_args.f_principal_camera_fov_degrees; // use "ugly" numbers
		int p_wp[4];
		glGetIntegerv(GL_VIEWPORT, p_wp);
		const int n_width = p_wp[2], n_height = p_wp[3]; // we might be rendering offscreen right now
		const float f_aspect = (float)n_width / n_height;

		Matrix4f t_projection;
		//if(!m_b_screenshot_render_running)
		if(!m_b_render_running) // let's see what is the effect of jitter on videos (none noticeable)
			CGLTransform::Perspective(t_projection, f_fov, f_aspect, f_near, f_far);
		else {
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
			Perspective_Jitter(t_projection, f_fov, f_aspect, f_near, f_far,
				p_jitter[n_jitter].x * CGLThreadedFrameWriter::n_spatial_supersample_num / n_width,
				p_jitter[n_jitter].y * CGLThreadedFrameWriter::n_spatial_supersample_num / n_height);
			// jitter for scaled-down pixels so that it is not undone by the antialiasing
		}
		// calculate projection matrix

		return t_projection;
	}

	Matrix4f t_ModelMatrix() const
	{
		Matrix4f t_model_matrix;
		t_model_matrix.Identity();
		if(m_b_upside_down)
			t_model_matrix.Scale(1, -1, 1); // mirror // todo - give a choice of mirror or rotate around the z axis (affects camera pose assumption and trajectory rendering and maybe other stuff)
		// calculate object matrix

		return t_model_matrix;
	}

	Matrix4f t_Modelview(double f_time)
	{
		Matrix4f t_model_matrix = t_ModelMatrix();
		// calculate object matrix

		Matrix4f t_camera_matrix = t_Camera(f_time);
		// calculate final camera matrix (with pitch and roll)

		Matrix4f t_modelview = t_camera_matrix * t_model_matrix;
		// fuse object and camera matrix to modelview

		return t_modelview;
	}

	Matrix4f t_MVP(double f_time)
	{
		Matrix4f t_projection = t_Projection();
		// calculate projection matrix

		Matrix4f t_modelview = t_Modelview(f_time);
		// fuse object and camera matrix to modelview

		return t_projection * t_modelview;
		// calculate the mvp matrix
	}

	void Assume_VertexPose(size_t n_id)
	{
#ifdef _DEBUG
		printf("debug: assuming pose of vertex " PRIsize "\n", n_id);
#endif // _DEBUG

		Matrix4f t_original_camera = t_UserCamera();
		Matrix4f t_current_camera = t_Camera(m_anim.f_Time());
		Matrix4f t_vertex_pose;
		{
			t_vertex_pose.Identity();
			t_vertex_pose.Set_RotationPart(m_solution.vertex_list[n_id].t_Pose().first);
			if(m_b_upside_down) {
				t_vertex_pose[0][1] *= -1;
				t_vertex_pose[1][1] *= -1;
				t_vertex_pose[2][1] *= -1;
				t_vertex_pose.RotateY(f_pi); // needed for some reason
				t_vertex_pose.Scale(-1, 1, 1);
				t_vertex_pose.RotateZ(f_pi); // needed for some reason
				// don't ask.
			} else
				t_vertex_pose.RotateX(f_pi); // needed for some reason
			t_vertex_pose.Offset(m_solution.vertex_list[n_id].t_Pose().second);
			if(m_b_upside_down)
				t_vertex_pose[3][1] *= -1;
			t_vertex_pose.Invert_Inplace(); // needed
		}
		// get the poses

		// big todo - abandon pushing and popping of animations, simply use a second spline for the assume
		// pose animation, redirect only pause/stop/reset controls to it (and probably mask out play/load/save)
		// so that it would be possible to copy poses to the "main" animation; think about animating in
		// incremental solutions (but probably that would better be handled by a script that blends between
		// the animation and the incremental solution poses)

		if(m_n_zoomed_into_camera_vertex == size_t(-1) || !m_anim.b_Active())
			m_anim.onPushAnimation();
		else {
			m_anim.onResetAnimation(); // if we were in animation before, it was now popped
			if(!m_anim.b_Empty()) // if it was popped ...
				m_anim.onPushAnimation(); // ... push it back
		}
		// in case the animation is not one of the assuming a vertex pose, save it to be restored later on

		_ASSERTE(!m_anim.n_Real_KeyFrame_Num());

		m_anim.onPushSplinePose(0, t_current_camera);
		m_anim.onPushSplinePose(1, t_vertex_pose, 1); // not smoooth in here
		m_anim.onPushSplinePose(2, t_vertex_pose, 1); // not smoooth in here
		m_anim.onPushSplinePose(3, t_original_camera);
		m_anim.onPlayAnimation();
		if(m_anim.b_Paused())
			m_anim.onTogglePause();
		// set the animation up

#ifdef _DEBUG
		printf("debug: animation has " PRIsize " keyframes, takes %g sec\n",
			m_anim.n_KeyFrame_Num() - 2, m_anim.f_AnimationLength());
#endif // _DEBUG

		m_n_zoomed_into_camera_vertex = n_id;
		m_b_zoom_into_camera_auto_pause = true;
		std::string s_caption;
		stl_ut::Format(s_caption, "%s | view of pose " PRIsize " (%dD)",
			m_t_args.p_s_window_title, n_id, m_solution.vertex_list[n_id].n_Dimension());
		glutSetWindowTitle(s_caption.c_str());

		glutPostRedisplay(); // !!
	}

	void LookAt_AllPoints()
	{
		if(m_solution.vertex_list.empty())
			return;
		// some of the code expects non-empty list

		float f_min_x, f_max_x;
		float f_min_y, f_max_y;
		float f_min_z, f_max_z;
		{
			std::vector<float> x(m_solution.vertex_list.size()), y(m_solution.vertex_list.size()), z(m_solution.vertex_list.size());
			for(size_t i = 0, n = m_solution.vertex_list.size(); i < n; ++ i) { // ignores ranges. all points.
				if(m_solution.vertex_list[i].b_Has_Position()) {
					Vector3f v_pos = m_solution.vertex_list[i].v_Position();
					x[i] = v_pos.x;
					y[i] = v_pos.y;
					z[i] = v_pos.z;
				}
			}
			std::sort(x.begin(), x.end());
			std::sort(y.begin(), y.end());
			std::sort(z.begin(), z.end());
			if(0) { // sharp min / max
				f_min_x = x.front();
				f_max_x = x.back();
				f_min_y = y.front();
				f_max_y = y.back();
				f_min_z = z.front();
				f_max_z = z.back();
			} else {
				size_t n_first = m_solution.vertex_list.size() / 100, n_last =
					std::max(m_solution.vertex_list.size() - 1, n_first) - n_first;
				// divide by 100 to get 1%

				f_min_x = x[n_first];
				f_max_x = x[n_last];
				f_min_y = y[n_first];
				f_max_y = y[n_last];
				f_min_z = z[n_first];
				f_max_z = z[n_last];
			}
		}
		// get a 1% "soft" bounding box

		Vector3f v_points_center(f_min_x + f_max_x, f_min_y + f_max_y, f_min_z + f_max_z);
		v_points_center *= .5f;
		// get the (approximate) center of the points

		/*static bool b_center_added = false;
		if(!b_center_added) {
			TScalar *p_data = data_pool.p_GetRange(3); // negligible cost
			CVertex v(3, p_data);
			for(int i = 0; i < 3; ++ i)
				p_data[i] = v_points_center[i];
			m_solution.vertex_list.push_back(v);
			if(!m_solution.vertex_color_list.empty())
				m_solution.vertex_color_list.push_back(Vector3ub(255, 0, 0));
			++ m_solution.vertex_range_list.back().second;
			// add a red point in the middle

			m_fap.Free_GLBuffers();
			m_ctx.Free_GLBuffers();
			m_cam.Free_GLBuffers();
			m_flp.Free_GLBuffers();
			m_edg.Free_GLBuffers();
			// force the arrays to regen

			b_center_added = true;
		}*/
		// debug - see the points center as a red dot

		Matrix4f t_model_matrix = t_ModelMatrix();
		// calculate object matrix

		m_v_camera_pos = t_model_matrix.t_Inverse().v_Transform_Pos(-v_points_center);
		// we want v_points_center to be at origin and we want to figure out the v_camera_pos which does that
		// we have R = camera rot, T = camera trans, M = model matrix, c = points center
		// (0, 0, 0, 1) = (R * T) * M * c
		// t = M^-1 * -c

		Matrix4f t_camera_matrix = t_UserCamera(); // will need that later
		Matrix4f t_mvp = t_Projection() * t_camera_matrix * t_model_matrix;

		Plane3f p_relevant_frustum[5], t_far;
		t_mvp.Get_FrustumPlanes(p_relevant_frustum[0], p_relevant_frustum[1],
			p_relevant_frustum[2], p_relevant_frustum[3], p_relevant_frustum[4], t_far, true);
		// get the camera frustum in the space of the points

		Vector3f v_rev_view_dir = -p_relevant_frustum[4].v_normal;
		// get reverse view direction in the space of the points

		float f_min_distance = 1e30;
		for(size_t i = 0, n = m_solution.vertex_list.size(); i < n; ++ i) { // ignores ranges. all points.
			if(!m_solution.vertex_list[i].b_Has_Position())
				continue;
			Vector3f v_pos = m_solution.vertex_list[i].v_Position();
			if(v_pos.x < f_min_x || v_pos.x > f_max_x ||
			   v_pos.y < f_min_y || v_pos.y > f_max_y ||
			   v_pos.z < f_min_z || v_pos.z > f_max_z)
				continue;
			// for all the points that are inside of the soft bounding box

			for(int j = 0; j < 5; ++ j) {
				float f_isect;
				if(!p_relevant_frustum[j].Intersect_Ray_t(f_isect, v_pos, v_rev_view_dir))
					continue; // should not really happen unless the frustum is very narrow (like zero degrees)
				// calculate the distance of the point to the plane, along the view direction vector

				if(f_min_distance > f_isect)
					f_min_distance = f_isect;
				// find the point which is the most behind the camera
			}
			// look for intersection of a ray at the point and having the view distance with the frustum
			// planes (except with the far one)
		}
		// find the distance of the furthest point we want to see

		const float f_rel_headroom = 1e-2f; // headroom relative to the point cloud size
		float f_cloud_size = std::max(f_max_x - f_min_x,
			std::max(f_max_y - f_min_y, f_max_z - f_min_z));
		f_min_distance -= f_cloud_size * f_rel_headroom;
		// keep some headroom so that the points at the edges of the frustum do not get clipped

		m_v_camera_pos += t_camera_matrix.t_Inverse().v_Dir() * f_min_distance;
		// calculate the offset of the camera so that it sees all the points, in the space of the points
	}

	void onResize(int n_new_width, int n_new_height)
	{
		glViewport(0, 0, m_n_width = n_new_width, m_n_height = n_new_height);
	}

	void RenderFrame(double f_time) // throw(std::bad_alloc)
	{
		Matrix4f t_identity;
		t_identity.Identity();
		RenderFrame(f_time, t_identity);
	}

	// render with an additional view transform for stereo or omni-directional (screenshot) rendering
	void RenderFrame(double f_time, const Matrix4f t_view_transform) // throw(std::bad_alloc)
	{
		_ASSERTE(m_n_background_type >= 0 && m_n_background_type < 4);
		switch(m_n_background_type) {
		case 0:
			glClearColor(0, 0, 0, 0);
			break;
		case 1:
			glClearColor(.5f, .5f, .5f, 0);
			break;
		case 2:
			glClearColor(.75f, .75f, .75f, 0);
			break;
		case 3:
			glClearColor(1, 1, 1, 0);
			break;
		}
		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

		const Matrix4f t_model = t_ModelMatrix();
		// get the model matrix

		const Matrix4f t_view = t_view_transform * t_Camera(f_time); // needs to be here, so that all sprites are always facing the view plane if rendering into a cubemap
		// get the camera matrix

		const Matrix4f t_modelview = t_view * t_model;
		// calculate the mv matrix

		const Matrix4f t_mvp = t_Projection() * t_modelview;//t_MVP(f_time);
		// calculate the mvp matrix

		const Vector3f v_up = t_model.v_Transform_Dir(t_view.t_Transpose().v_Up().v_Normalized()),
			v_right = t_model.v_Transform_Dir(t_view.t_Transpose().v_Right().v_Normalized());
		// get the up/right vectors for sprites

		int p_wp[4];
		glGetIntegerv(GL_VIEWPORT, p_wp);
		const int n_width = p_wp[2], n_height = p_wp[3]; // we might be rendering offscreen right now
		const float f_thickness_scale = max(1e-5f, n_height / 480.0f); // when minimized, n_height becomes zero or negative
		// for line rasterization

		glEnable(GL_DEPTH_TEST);
		//glPointSize(2 * f_thickness_scale); // written by the shader
		//glDisable(GL_POINT_SMOOTH); // otherwise looks chubby // not enabled by default, not in GLES20

		glDisable(GL_BLEND);
		// no blending for the points

		m_const_color_shader.SetScale(f_thickness_scale * m_f_point_size);
		m_const_color_lerp_shader.SetScale(f_thickness_scale * m_f_point_size);
		m_color_shader.SetScale(f_thickness_scale * m_f_point_size);
		m_color_lerp_shader.SetScale(f_thickness_scale * m_f_point_size);
		m_texture_shader.SetScale(f_thickness_scale * m_f_point_size);
		m_texture_lerp_shader.SetScale(f_thickness_scale * m_f_point_size);
		m_camera_tex_proj_shader.SetScale(f_thickness_scale * m_f_point_size);
		m_camera_tex_proj_lerp_shader.SetScale(f_thickness_scale * m_f_point_size);

		double f_sub_frame; // interpolation between the animation frames
		size_t n_cur_frame, n_next_frame; // indices of animation frames
		if(m_solution.vertex_range_list.size() > 1) {
			f_sub_frame = (max(size_t(1), m_solution.vertex_range_list.size()) - 1) *
				min(1.0, f_time * 2 / m_solution.vertex_range_list.size());
			n_cur_frame = f_sub_frame;
			f_sub_frame -= n_cur_frame;
			// todo - lower_bound in a timestamp array for "real-speed" playback

			n_next_frame = min(max(size_t(1), m_solution.vertex_range_list.size()) - 1, n_cur_frame + 1); // inclusive, the next frame for lerp

			static size_t n_prev_cur_frame = size_t(-1);
			if(n_prev_cur_frame != n_cur_frame) {
				n_prev_cur_frame = n_cur_frame;
				char p_s_title[256];
				stl_ut::Format(p_s_title, sizeof(p_s_title), "Graph Viewer - solution frame " PRIsize, n_cur_frame);
				glutSetWindowTitle(p_s_title);
			}
		} else {
			f_sub_frame = 0;
			n_cur_frame = 0;
			n_next_frame = 0;
		}
		const TSolutionFrameInfo t_sfi(n_cur_frame, n_next_frame, f_sub_frame);
		// animated solution indexing

		_ASSERTE(!m_solution.vertex_range_list.empty());
		// this almost always dereferences vertex_range_list
		// t_odo - add range (0, 0) in case the solution failed to load or check for the range being empty

		if(m_p_megatexture && m_b_fancy_color_points && !m_graph.edge_list.empty()) {
			m_fap.Draw_FancyPoints(t_mvp, t_model, v_right, v_up, t_sfi, m_solution, m_graph,
				m_t_args.n_original_images_width, m_t_args.n_original_images_height,
				m_t_args.t_proj_colorize_image_matrix, n_height, m_t_args.f_principal_camera_fov_degrees,
				m_t_args.b_anim_color_blend, m_p_megatexture, m_cam_texcoord_list, m_camera_tex_proj_shader,
				m_camera_tex_proj_lerp_shader);
			// draw points with texture projection
		} else {
			m_flp.Draw_Points(t_mvp, t_sfi, m_solution, m_marginals, m_b_display_covs,
				m_t_args.b_anim_color_blend, m_gradient_texture, m_const_color_shader,
				m_const_color_lerp_shader, m_texture_shader, m_texture_lerp_shader, m_color_shader,
				m_color_lerp_shader);
			// draw flat-shaded vertices
		}
		// draw points

		glDepthMask(GL_FALSE); // let cameras / edges float above
		glEnable(GL_BLEND);
		glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
		glEnable(0x0B20/*GL_LINE_SMOOTH*/);
		// prepare for drawing the grid

		if(m_b_display_grid) {
#ifdef __GRID_SEE_SIDE_BY_SIDE
#ifdef __APPLE__
			glLineWidth(min(1.0f, .25f * f_thickness_scale)); // using core profile on mac, thick line rasterization is unavailable
#else // __APPLE__
			glLineWidth(.25f * f_thickness_scale);
#endif // __APPLE__
#endif // __GRID_SEE_SIDE_BY_SIDE
			m_grid.Draw(t_mvp, (m_n_background_type < 2)? Vector4f(.3f, 1, .5f, 1) :
				Vector4f(0, .666f, .184f, 1), .25f * f_thickness_scale);
		}
		// draw grid

		glDepthMask(GL_TRUE);

		glEnable(GL_BLEND);
		glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
		m_const_color_shader.Bind(t_mvp, 1, 0, 0);
		// reenable blend for cameras, draw the cameras after the points

#ifdef __APPLE__
		glLineWidth(min(1.0f, 1.0f * f_thickness_scale)); // using core profile on mac, thick line rasterization is unavailable
#else // __APPLE__
		glLineWidth(1.0f * f_thickness_scale);
#endif // __APPLE__

		if(m_b_display_cameras) {
			m_ctx.Draw_CameraImages(t_mvp, t_sfi, m_solution, m_t_args.f_camera_size, m_t_args.f_camera_aspect,
				m_t_args.f_camera_fov_degrees, m_f_camera_tex_alpha, m_p_megatexture, m_cam_texcoord_list,
				m_camera_tex_shader, m_camera_tex_lerp_shader);
			m_cam.Draw_Cameras(t_mvp, t_sfi, m_solution, m_t_args.f_camera_size, m_t_args.f_camera_aspect,
				m_t_args.f_camera_fov_degrees, (m_n_background_type < 2)? Vector3f(1, 1, 1) : Vector3f(0, 0, 0),
				m_camera_shader, m_camera_lerp_shader);
		}
		// draw cameras

		if(0 && !m_anim.b_Empty()) {
#ifdef __APPLE__
			glLineWidth(min(1.0f, 2 * f_thickness_scale)); // using core profile on mac, thick line rasterization is unavailable
#else // __APPLE__
			glLineWidth(2 * f_thickness_scale);
#endif // __APPLE__

			const float f_aspect = float(m_n_width) / m_n_height;
			const float f_camera_size2 = m_t_args.f_camera_size * 2;
			const float f_camera_image_plane_size = float(f_camera_size2 * f_aspect * // note that the principal camera fov is vertical
				tan(m_t_args.f_principal_camera_fov_degrees / 360.0f * f_pi));

			double f_len = m_anim.f_AnimationLength();
			std::vector<Vector3f> anim_trajectory;
			for(size_t i = 0, n = 100; i < n; ++ i) { // would have to guess better
				Vector3f v_pos = -m_anim.t_Interpolate2(f_len * double(i) / (n - 1)).v_position;
				if(m_b_upside_down)
					v_pos.y = -v_pos.y; // !! t_mvp is flipped but the camera spline is not
				anim_trajectory.push_back(v_pos);
			}
			std::vector<Vector3f> anim_keyframes;
			for(size_t i = 0, n = m_anim.n_KeyFrame_Num(); i < n; ++ i) {
				CCameraAnimation::_TyKeyPoint t_kp = m_anim.t_KeyFrame(i);
				Matrix3f t_transform = t_kp.t_rotation.t_Conjugate().t_ToMatrix();
				Vector3f v_pos = -t_kp.v_position;

				Vector3f a, b, c, d, v_up = t_transform.v_Up() * f_camera_image_plane_size / f_aspect,
					v_right = t_transform.v_Right() * f_camera_image_plane_size,
					v_dir = -t_transform.v_Dir() * f_camera_size2;
				if(m_b_upside_down) {
					v_up.y = -v_up.y; 
					v_right.y = -v_right.y; 
					v_dir.y = -v_dir.y; 
					v_pos.y = -v_pos.y;// !! t_mvp is flipped but the camera spline is not
				}
				a = v_pos + v_dir + v_up + v_right;
				b = v_pos + v_dir - v_up + v_right;
				c = v_pos + v_dir - v_up - v_right;
				d = v_pos + v_dir + v_up - v_right;
				anim_keyframes.reserve(anim_keyframes.size() + 16);
				anim_keyframes.push_back(v_pos);
				anim_keyframes.push_back(a);
				anim_keyframes.push_back(v_pos);
				anim_keyframes.push_back(b);
				anim_keyframes.push_back(v_pos);
				anim_keyframes.push_back(c);
				anim_keyframes.push_back(v_pos);
				anim_keyframes.push_back(d);
				anim_keyframes.push_back(a);
				anim_keyframes.push_back(b);
				anim_keyframes.push_back(b);
				anim_keyframes.push_back(c);
				anim_keyframes.push_back(c);
				anim_keyframes.push_back(d);
				anim_keyframes.push_back(d);
				anim_keyframes.push_back(a);
			}
			m_const_color_shader.Bind(t_mvp, .4f, .5f, 1);
			CGLArraySetup traj_arr((anim_trajectory.empty())? 0 : &anim_trajectory.front(),
				anim_trajectory.size() * sizeof(Vector3f), 0, GL_FLOAT, 3, 0, 0, 0, 0, GL_LINE_STRIP);
			traj_arr.Draw();
			CGLArraySetup cam_arr((anim_keyframes.empty())? 0 : &anim_keyframes.front(),
				anim_keyframes.size() * sizeof(Vector3f), 0, GL_FLOAT, 3, 0, 0, 0, 0, GL_LINES);
			cam_arr.Draw();
		}
		// debug rendering of the animation spline (could add t/c/b and knot vector controls
		// if needed, but i guess one can easily edit it in notepad)

		if(m_b_display_edges || !m_selection_buffer.empty()) {
			m_edg.Draw_Edges(t_mvp, t_sfi, m_solution, m_graph, m_b_edges_blend_add, m_f_edge_alpha,
				m_n_background_type < 2, m_selection_buffer, m_const_color_shader);
		}
		// draw edges

		if(m_b_display_covs && (!m_p_megatexture || !m_b_fancy_color_points ||
		   m_graph.edge_list.empty()) && !m_marginals.cov_list.empty() &&
		   !(m_b_render_running && m_t_args.b_video_omni)) { // in case m_b_fancy_color_points is enabled, the marginals are not displayed so do not display the gui. same if there are no marginals calculated, they will not be displayed.
			glActiveTexture(GL_TEXTURE0);
			m_gradient_texture.Bind();
			// !!

			Matrix4f t_identity;
			t_identity.Identity();
			m_texture_shader.Bind(t_identity);
			// bind shader

			const float f_bar_width = .025f;
			float f_x0 = 1 - .1f  - f_bar_width / 2, f_x1 = f_x0 + f_bar_width;
			float f_y0 = .9f, f_y1 = .6f;

			std::vector<Vector2f> vertex_list;
			vertex_list.push_back(Vector2f(0, 0));
			vertex_list.push_back(Vector2f(f_x0, f_y0));
			vertex_list.push_back(Vector2f(1, 1));
			vertex_list.push_back(Vector2f(f_x0, f_y1));
			vertex_list.push_back(Vector2f(0, 0));
			vertex_list.push_back(Vector2f(f_x1, f_y0));
			vertex_list.push_back(Vector2f(1, 1));
			vertex_list.push_back(Vector2f(f_x1, f_y1));

#if 1
			CGLArraySetup grad_bar(&vertex_list[0], vertex_list.size() * sizeof(Vector2f),
				2 * sizeof(Vector2f), GL_FLOAT, 2, 0, GL_FLOAT, 2, sizeof(Vector2f), GL_TRIANGLE_STRIP); // note that texcoord and pos are swapped so that pos is attribute 0
#else // 1
			std::vector<Vector2f> vertex_pos_list, vertex_tex_list;
			vertex_tex_list.push_back(Vector2f(0, 0));
			vertex_pos_list.push_back(Vector2f(f_x0, f_y0));
			vertex_tex_list.push_back(Vector2f(1, 1));
			vertex_pos_list.push_back(Vector2f(f_x0, f_y1));
			vertex_tex_list.push_back(Vector2f(0, 0));
			vertex_pos_list.push_back(Vector2f(f_x1, f_y0));
			vertex_tex_list.push_back(Vector2f(1, 1));
			vertex_pos_list.push_back(Vector2f(f_x1, f_y1));

			/*CGLMultiArraySetup<1> grad_bar(
				TGLVBOConfig(vertex_list)((
					GLVertexAttribPtrSpec(0, 2, GL_FLOAT, false, 2 * sizeof(Vector2f), sizeof(Vector2f)),
					GLVertexAttribPtrSpec(1, 2, GL_FLOAT, false, 2 * sizeof(Vector2f), 0)
				)), GL_TRIANGLE_STRIP, 4);*/
			// a single interleaved buffer with multiple attributes, works nicely

			/*CGLMultiArraySetup<1> grad_bar(
				TGLVBOConfig(vertex_list)((
					GLVertexAttribPtrShort(2, GL_FLOAT, 2 * sizeof(Vector2f), sizeof(Vector2f)),
					GLVertexAttribPtrShort(2, GL_FLOAT, 2 * sizeof(Vector2f), 0),
					GLVertexAttribPtrShort(2, GL_FLOAT, 2 * sizeof(Vector2f), sizeof(float)) // dummy, to see a larger chain
				)), GL_TRIANGLE_STRIP, 4);*/
			// a single interleaved buffer with multiple attributes and automatic attrib numbering, works nicely

			/*CGLMultiArraySetup<1> grad_bar(
				TGLVBOConfig(vertex_list)((
					GLVertexAttribPtrSpec(0, 2, GL_FLOAT, false, 2 * sizeof(Vector2f), sizeof(Vector2f)),
					GLVertexAttribPtrShort(2, GL_FLOAT, 2 * sizeof(Vector2f), 0) // id of the prev + 1
				)), GL_TRIANGLE_STRIP, 4);*/
			// a single interleaved buffer with multiple attributes and semi-automatic attrib numbering, works nicely

			/*CGLMultiArraySetup<1> grad_bar(
				TGLVBOConfig(vertex_list)((
					GLVertexAttribPtrShort(2, GL_FLOAT, 2 * sizeof(Vector2f), sizeof(Vector2f)),
					GLVertexAttribPtrSpec(1, 2, GL_FLOAT, false, 2 * sizeof(Vector2f), 0) // id of the prev + 1
				)), GL_TRIANGLE_STRIP, 4);*/
			// a single interleaved buffer with multiple attributes and semi-automatic attrib numbering, works nicely

			/*CGLMultiArraySetup<2> grad_bar((
				TGLVBOConfig(vertex_pos_list)((
					GLVertexAttribPtrSpec(0, 2, GL_FLOAT, false, 0, 0)
				)),
				TGLVBOConfig(vertex_tex_list)((
					GLVertexAttribPtrSpec(1, 2, GL_FLOAT, false, 0, 0)
				))
				), GL_TRIANGLE_STRIP, 4);*/
			// two buffers, each containing its own attribute, works nicely

			/*CGLMultiArraySetup<3> grad_bar((
				TGLVBOConfig(vertex_list)((
					GLVertexAttribPtrSpec(0, 2, GL_FLOAT, false, 2 * sizeof(Vector2f), sizeof(Vector2f)),
					GLVertexAttribPtrSpec(1, 2, GL_FLOAT, false, 2 * sizeof(Vector2f), 0)
				)),
				TGLVBOConfig(vertex_list)((
					GLVertexAttribPtrSpec(2, 2, GL_FLOAT, false, 2 * sizeof(Vector2f), 0)
				)),
				TGLVBOConfig(vertex_list)((
					GLVertexAttribPtrSpec(3, 2, GL_FLOAT, false, 2 * sizeof(Vector2f), 4)
				))
				), GL_TRIANGLE_STRIP, 4);*/
			// test of multiple buffer chaining, multiple attributes, works nicely

			/*CGLMultiArraySetup<3> grad_bar((
				TGLVBOConfig(vertex_list)((
					GLVertexAttribPtrSpec(1, 1, GL_FLOAT, false, 0, 0), // note that this is a wrong format
					GLVertexAttribPtrSpec(0, 1, GL_FLOAT, false, 0, 0) // note that this is a wrong format
				)),
				TGLVBOConfig(vertex_list)((
					GLVertexAttribPtrSpec(2, 2, GL_FLOAT, false, 2 * sizeof(Vector2f), 0)
				)),
				TGLVBOConfig(vertex_list)((
					GLVertexAttribPtrSpec(3, 2, GL_FLOAT, false, 2 * sizeof(Vector2f), 4)
				))
				), GL_TRIANGLE_STRIP, 4);
			grad_bar.Draw_Attribs((
				GLVertexAttribPtrVSpec(0, 2, GL_FLOAT, false, 2 * sizeof(Vector2f), sizeof(Vector2f)), // re-specify the format here
				GLVertexAttribPtrVSpec(1, 2, GL_FLOAT, false, 2 * sizeof(Vector2f), 0), // re-specify the format here
				++
				GLVertexAttribPtrVSpec(2, 2, GL_FLOAT, false, 2 * sizeof(Vector2f), 0),
				++
				GLVertexAttribPtrVSpec(3, 2, GL_FLOAT, false, 2 * sizeof(Vector2f), 4)
				), true);*/
			// test of multiple buffer chaining, multiple attributes and variable attribute re-specification, works nicely
#endif // 1
			// test multiple VBO GL array setup, works nicely

			glDisable(GL_BLEND);
			grad_bar.Draw(); // note that this only needs to be regenerated when resolution changes
			// gradient bar
		}
		// draw "gui"
	}

	CGLColorId DrawSelection(Matrix4f t_mvp)
	{
		glClearColor(0, 0, 0, 0); // full black!
		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

		int p_wp[4];
		glGetIntegerv(GL_VIEWPORT, p_wp);
		const int n_width = p_wp[2], n_height = p_wp[3]; // we might be rendering offscreen right now
		const float f_thickness_scale = n_height / 480.0f;
		// for line rasterization

		m_color_shader.SetScale(f_thickness_scale * m_f_point_size);

		glDisable(GL_BLEND);
		// no blending for the points

		_ASSERTE(m_solution.vertex_list.size() < SIZE_MAX); // make sure the next line does not overflow
		const size_t n_id_num = m_solution.vertex_list.size() + 1;
		CGLColorId color_id;
		color_id.Get_BitDepth();
		/*printf("bit depth: %d%d%d%d ", color_id.v_BitDepth().x, color_id.v_BitDepth().y,
			color_id.v_BitDepth().z, color_id.v_BitDepth().w);*/ // debug
		if(color_id.n_Max_ID_Num() < n_id_num)
			throw std::runtime_error("error: color_id.n_Max_ID_Num() < n_id_num and multipass rendering not supported"); // todo - allow multipass (low priority, it would require having more than 2^32 vertices, which is well above the size of the graphs that require debug visualisation) // or render with hi-lo id in two passes, that is quite simple
		if(!color_id.Set_ID_Num(n_id_num))
			throw std::runtime_error("error: CGLColorId::Set_ID_Num() failed"); // should not really happen, the line above should have taken care of that
		_ASSERTE(color_id.n_Int_DataType() == GL_UNSIGNED_BYTE); // that's what we count on
		// prepare color ids

		{
			if(!m_p_points_array_selection) {
				const float f_camera_image_plane_size = float(m_t_args.f_camera_size *
					tan(m_t_args.f_camera_fov_degrees / 360.0f * f_pi));

				_ASSERTE(sizeof(TVertex_C4UB_V3F) == 16); // make sure no padding is generated
				std::vector<TVertex_C4UB_V3F> vertex_pick, camera_pick;

				for(size_t i = 0, n = m_solution.vertex_list.size(); i < n; ++ i) {
					TVertex_C4UB_V3F t_vertex;
					color_id.ID_to_Color(t_vertex.p_color, 4 * sizeof(uint8_t),
						GL_UNSIGNED_BYTE, i + 1); // leave id 0 for the background / no-hit
					// generate color id

					if(m_solution.vertex_list[i].b_Has_Pose()) {
						std::pair<Matrix3f, Vector3f> t_Rt = m_solution.vertex_list[i].t_Pose();
						Matrix3f &t_transform = t_Rt.first; // rename
						const Vector3f &v_pos = t_Rt.second; // rename
						// get a transformation matrix for the given camera

						Vector3f p, a, b, c, d, v_up = t_transform.v_Up() *
							f_camera_image_plane_size / m_t_args.f_camera_aspect,
							v_right = t_transform.v_Right() * f_camera_image_plane_size,
							v_dir = t_transform.v_Dir() * m_t_args.f_camera_size; // 2015-06-15 - changed sign here
						p = v_pos;
						a = v_pos + v_dir + v_up + v_right;
						b = v_pos + v_dir - v_up + v_right;
						c = v_pos + v_dir - v_up - v_right;
						d = v_pos + v_dir + v_up - v_right;

						t_vertex.v_pos = p; camera_pick.push_back(t_vertex); vertex_pick.push_back(t_vertex);
						t_vertex.v_pos = a; camera_pick.push_back(t_vertex);
						t_vertex.v_pos = b; camera_pick.push_back(t_vertex);
						//
						t_vertex.v_pos = p; camera_pick.push_back(t_vertex);
						t_vertex.v_pos = b; camera_pick.push_back(t_vertex);
						t_vertex.v_pos = c; camera_pick.push_back(t_vertex);
						//
						t_vertex.v_pos = p; camera_pick.push_back(t_vertex);
						t_vertex.v_pos = c; camera_pick.push_back(t_vertex);
						t_vertex.v_pos = d; camera_pick.push_back(t_vertex);
						//
						t_vertex.v_pos = p; camera_pick.push_back(t_vertex);
						t_vertex.v_pos = d; camera_pick.push_back(t_vertex);
						t_vertex.v_pos = a; camera_pick.push_back(t_vertex);
						//
						if(m_n_zoomed_into_camera_vertex == size_t(-1)) {
							t_vertex.v_pos = a; camera_pick.push_back(t_vertex);
							t_vertex.v_pos = d; camera_pick.push_back(t_vertex);
							t_vertex.v_pos = b; camera_pick.push_back(t_vertex);
							//
							t_vertex.v_pos = b; camera_pick.push_back(t_vertex);
							t_vertex.v_pos = d; camera_pick.push_back(t_vertex);
							t_vertex.v_pos = c; camera_pick.push_back(t_vertex);
							// in case we are zoomed into a vertex, remove the front
							// face so that we could jump to other vertices
						}
					} else if(m_solution.vertex_list[i].b_Has_Position()) {
						t_vertex.v_pos = m_solution.vertex_list[i].v_Position();
						vertex_pick.push_back(t_vertex);
					}
				}
				// generate arrays

				m_p_points_array_selection = new CGLArraySetup((vertex_pick.empty())? 0 : &vertex_pick[0],
					vertex_pick.size() * sizeof(TVertex_C4UB_V3F), sizeof(TVertex_C4UB_V3F), GL_UNSIGNED_BYTE, 4, 0,
					GL_FLOAT, 3, 4 * sizeof(uint8_t), GL_POINTS, true);
				m_p_cameras_array_selection = new CGLArraySetup((camera_pick.empty())? 0 : &camera_pick[0],
					camera_pick.size() * sizeof(TVertex_C4UB_V3F), sizeof(TVertex_C4UB_V3F), GL_UNSIGNED_BYTE, 4, 0,
					GL_FLOAT, 3, 4 * sizeof(uint8_t), GL_TRIANGLES, true);
			}
			// generate the arrays, if needed

			m_color_shader.Bind(t_mvp);
			m_p_points_array_selection->Draw();
			if(m_b_display_cameras)
				m_p_cameras_array_selection->Draw();
			// draw selection
		}
		// render selection

		return color_id;
	}

	size_t n_Selection(int n_x, int n_y)
	{
		if(m_solution.vertex_range_list.size() > 1 || !m_t_args.b_selection_enabled || m_b_render_running) // big todo - reuse the animated color shader, can render color ids even when having multiple solution frames
			return size_t(-1);
		// selection is disabled in case there are multiple solution frames and when
		// rendering (could rewrite the current accumulation buffer)

		glClearColor(0, 0, 0, 0); // full black!
		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

		Matrix4f t_mvp = t_MVP(m_anim.f_Time()); // needless to say there will be problems with picking during animation as this is called asynchronously so there might not be under the cursor what the user actually sees
		// calculate the mvp matrix

		glScissor(n_x - 1, n_y - 1, 3, 3);
		glEnable(GL_SCISSOR_TEST);
		// no need to draw all; we're only interested in what is under the mouse

		CGLColorId color_id = DrawSelection(t_mvp);
		// render selection

		glDisable(GL_SCISSOR_TEST);

		uint8_t p_color[4];
		glPixelStorei(GL_PACK_ALIGNMENT, 1); // !!
		glReadPixels(n_x, n_y, 1, 1, GL_RGBA, GL_UNSIGNED_BYTE, p_color);
		// read back

		size_t n_id = color_id.n_Color_to_ID(p_color, sizeof(p_color), GL_UNSIGNED_BYTE);
		/*printf("decoded color %d\n", int(n_id));
		glutSwapBuffers();*/ // debug
		// decode

		_ASSERTE(n_id <= m_solution.vertex_list.size()); // should generally not overflow, unless something strange happens to the colors
		if(!n_id || n_id > m_solution.vertex_list.size()) // check that in release as well to prevent crashes
			return size_t(-1);
		return n_id - 1;
	}

	void onDisplay()
	{
		if(m_b_render_running) {
			fprintf(stderr, "warning: offscreen render running, go away\n");
			return;
		}

		bool b_was_active = m_anim.b_Active() && !m_anim.b_Paused();
		// see if the animation is active now. if so, render one more frame so that
		// the camera would go back to the current viewpoint after the animation finishes

		if(b_was_active && m_n_zoomed_into_camera_vertex != size_t(-1) &&
		   m_b_zoom_into_camera_auto_pause && m_anim.f_Time() >= 1.5) {
			if(!m_anim.b_Paused())
				m_anim.onTogglePause();
			m_b_zoom_into_camera_auto_pause = false;

#ifdef _DEBUG
			printf("debug: auto-pausing assume-pose animation\n");
#endif // _DEBUG
		}
		// pause the assume camera pose animation

//#define __DEMO_SELECTION_BUFFER
#ifdef __DEMO_SELECTION_BUFFER
		int p_wp[4];
		glGetIntegerv(GL_VIEWPORT, p_wp);
		const int n_width = p_wp[2], n_height = p_wp[3]; // we might be rendering offscreen right now
		glScissor(n_width / 2, 0, n_width / 2, n_height);
		glEnable(GL_SCISSOR_TEST);
		// no need to draw all; we're only interested in what is under the mouse

		DrawSelection(t_MVP(m_anim.f_Time()));
		// render selection

		glScissor(0, 0, n_width / 2, n_height);
#endif // __DEMO_SELECTION_BUFFER

		RenderFrame(m_anim.f_Time());

#ifdef __DEMO_SELECTION_BUFFER
		glDisable(GL_SCISSOR_TEST);
#endif // __DEMO_SELECTION_BUFFER

		glutSwapBuffers();
		// display results

		if((m_anim.b_Active() && !m_anim.b_Paused()) || b_was_active || m_solution.vertex_range_list.size() > 1)
			glutPostRedisplay();
		else {
			if(!m_anim.b_Paused() && m_n_zoomed_into_camera_vertex != size_t(-1)) {
#ifdef _DEBUG
				printf("debug: animation expired: not centered on vertex "
					PRIsize " anymore\n", m_n_zoomed_into_camera_vertex);
#endif // _DEBUG

				glutSetWindowTitle(m_t_args.p_s_window_title); // reset window title
				m_n_zoomed_into_camera_vertex = size_t(-1); // not anymore
			}
		}
		// make sure that the next animation frame will be rendered
	}

	void onMouseClick(int n_button, int n_state, int n_x, int n_y)
	{
		m_v_mouse.x = (2.0f * n_x) / m_n_width - 1;
		m_v_mouse.y = 1 - (2.0f * n_y) / m_n_height;

		if(/*m_b_selection_lock ||*/ m_b_render_running)
			return;
		// if the selection is locked or a render is running, go no further

		if(!n_state) {
			m_b_is_pure_click = true;
			return;
		}
		// not interested in button down, do the work on button up

		if(!m_b_is_pure_click)
			return;
		// if the mouse moved between mouseup / mousedown, ignore the click

		bool b_selection_changed = false;

		switch(n_button) {
		case GLUT_LEFT_BUTTON:
			if(!m_b_selection_lock) {
				//bool b_holding_shift = !!(GetKeyState(VK_SHIFT) >> 8), b_holding_ctrl = !!(GetKeyState(VK_CONTROL) >> 8);
				bool b_holding_shift = !!(glutGetModifiers() & GLUT_ACTIVE_SHIFT),
					b_holding_ctrl = !!(glutGetModifiers() & GLUT_ACTIVE_CTRL); // thanks, glut ;)
				size_t n_id = n_Selection(n_x, m_n_height - 1 - n_y);
				if(!b_holding_ctrl && m_selection_buffer.size() == 1 && m_selection_buffer.front() == n_id) {
					// the same point is selected, nothing really changes
				} else {
					if(!b_holding_ctrl && !b_holding_shift) {
						b_selection_changed = !m_selection_buffer.empty();
						m_selection_buffer.clear();
					}
					if(n_id != size_t(-1)) {
						std::pair<bool, std::vector<size_t>::iterator> t_ii = stl_ut::t_Unique_Insert(m_selection_buffer, n_id);
						if(b_holding_ctrl) {
							if(!t_ii.first) // in case ctrl is held and we selected an already-selected vertex, we remove it from the selection
								m_selection_buffer.erase(t_ii.second);
							b_selection_changed = true; // toggle always changes state: either we just deleted it, or we just inserted it
						} else
							b_selection_changed |= t_ii.first; // selection changed in case we just inserted something
					}
				}
			}
			break;
		case GLUT_RIGHT_BUTTON:
			break;
		case GLUT_MIDDLE_BUTTON:
			{
				size_t n_id = n_Selection(n_x, m_n_height - 1 - n_y);
				if(n_id != size_t(-1) && m_solution.vertex_list[n_id].b_Has_Pose() &&
				   (!m_anim.b_Active() || m_anim.b_Paused()))
					Assume_VertexPose(n_id);
			}
			break;
		};
		//b_selection_dirty |= b_selection_changed;
		if(b_selection_changed)
			m_edg.On_SelectionChanged();
		// handle vertex selection

		if(b_selection_changed)
			glutPostRedisplay();

#ifdef _DEBUG
		if(b_selection_changed) {
			printf("debug: selection(");
			for(size_t i = 0, n = m_selection_buffer.size(); i < n; ++ i)
				printf((i)? ", %d" : "%d", int(m_selection_buffer[i]));
			printf(")\n");
		}
		//printf("selection changed: %s\n", (b_selection_changed)? "true" : "false");
#endif // _DEBUG
		// selection debugging
	}

	void onMouseMotion(int n_button, int n_x, int n_y)
	{
		m_b_is_pure_click = false;
		// the mouse is moving, this is not a click

		float f_new_mouse_x = (2.0f * n_x) / m_n_width - 1;
		float f_new_mouse_y = 1 - (2.0f * n_y) / m_n_height;
		// get current mouse position

		if(n_button == GLUT_LEFT_BUTTON) {
			m_f_angle_y += (f_new_mouse_x - m_v_mouse.x) * m_t_args.n_mouse_x_polarity;
			m_f_angle_x += (f_new_mouse_y - m_v_mouse.y) * m_t_args.n_mouse_y_polarity;
		} else if(n_button == GLUT_RIGHT_BUTTON)
			m_f_angle_z += f_new_mouse_x - m_v_mouse.x;
		// rotate scene

		m_v_mouse.x = f_new_mouse_x;
		m_v_mouse.y = f_new_mouse_y;
		// update mouse coords

		glutPostRedisplay();
	}

	void onPassiveMouseMotion(int n_x, int n_y)
	{
		m_v_mouse.x = (2.0f * n_x) / m_n_width - 1;
		m_v_mouse.y = 1 - (2.0f * n_y) / m_n_height;

		if((!m_anim.b_Active() || m_anim.b_Paused()) && !m_b_render_running) {
			size_t n_id = n_Selection(n_x, m_n_height - 1 - n_y);
			static size_t n_last_id = -1;
			if(n_id != n_last_id) {
				n_last_id = n_id;
				if(n_id != size_t(-1)) {
					std::string s_caption;
					stl_ut::Format(s_caption, "%s | selected vertex " PRIsize " (%dD)",
						m_t_args.p_s_window_title, n_id, m_solution.vertex_list[n_id].n_Dimension());
					glutSetWindowTitle(s_caption.c_str());
				} else if(m_n_zoomed_into_camera_vertex != size_t(-1)) {
					std::string s_caption;
					stl_ut::Format(s_caption, "%s | view of pose " PRIsize " (%dD)",
						m_t_args.p_s_window_title, m_n_zoomed_into_camera_vertex,
						m_solution.vertex_list[m_n_zoomed_into_camera_vertex].n_Dimension());
					glutSetWindowTitle(s_caption.c_str());
				} else
					glutSetWindowTitle(m_t_args.p_s_window_title);
			}
		}
	}

	void onKeyPressed(unsigned char n_key_code, int n_x, int n_y)
	{
		if(n_key_code == 27) {
			Cleanup();
			exit(0);
		} else {
			const float f_movement_speed = m_t_args.f_movement_speed;
#if defined(_WIN32) || defined(_WIN64)
			const float k = (GetKeyState(VK_CONTROL))? f_movement_speed : ((GetKeyState(VK_SHIFT))? f_movement_speed / 50 : f_movement_speed / 10); // these are toggle states, not immediate states, so glut does not help here
#else // _WIN32 || _WIN64
			const float k = (n_key_code >= 'A' && n_key_code <= 'Z')? f_movement_speed : f_movement_speed / 10; // fly fast with shift in linux
#endif // _WIN32 || _WIN64
			switch(n_key_code) {
			case 'w':
			case 'W':
				m_v_camera_pos += Vector3f(-k * float(sin(m_f_angle_y)) * float(cos(m_f_angle_x)), k * float(sin(m_f_angle_x)), float(cos(m_f_angle_y)) * k * float(cos(m_f_angle_x)));
				break;
			case 's':
			case 'S':
				m_v_camera_pos -= Vector3f(-k * float(sin(m_f_angle_y)) * float(cos(m_f_angle_x)), k * float(sin(m_f_angle_x)), float(cos(m_f_angle_y)) * k * float(cos(m_f_angle_x)));
				break;
			case 'a':
			case 'A':
				m_v_camera_pos += Vector3f(k * float(cos(m_f_angle_y)), 0, k * float(sin(m_f_angle_y)));
				break;
			case 'd':
			case 'D':
				m_v_camera_pos -= Vector3f(k * float(cos(m_f_angle_y)), 0, k * float(sin(m_f_angle_y)));
				break;
			case 'i':
				LookAt_AllPoints();
				break;
			case 'e':
				m_b_display_edges = !m_b_display_edges;
				break;
			case 'q':
				m_b_edges_blend_add = !m_b_edges_blend_add;
				break;
			case 'c':
				m_b_display_covs = !m_b_display_covs;
				break;
			case 'g':
				m_b_display_grid = !m_b_display_grid;
				break;
			case 'u':
				m_b_upside_down = !m_b_upside_down;
				break;
			case 'b':
				m_n_background_type = (m_n_background_type + 1) % 4;
				break;
			case 'y':
				m_b_display_cameras = !m_b_display_cameras;
				break;
			case 'r':
				Reload();
				break;
			case 't':
				if(m_f_edge_alpha < 1)
					m_f_edge_alpha *= 1 / .75f;
				break;
			case 'f':
				if(m_f_edge_alpha > 1.0f / 512)
					m_f_edge_alpha *= .75f;
				break;
			case '1':
				if(m_f_camera_tex_alpha > 1.0f / 256)
					m_f_camera_tex_alpha *= .75f;
				break;
			case '2':
				if(m_f_camera_tex_alpha < 1)
					m_f_camera_tex_alpha *= 1 / .75f;
				break;
			case '3':
				if(m_p_megatexture && m_t_args.b_fancy_color_points)
					m_b_fancy_color_points = !m_b_fancy_color_points;
				break;
			case '-':
				if(m_f_point_size > 0)
					-- m_f_point_size;
				break;
			case '+':
				++ m_f_point_size;
				break;
			case '*':
				if(m_solution.vertex_range_list.size() == 1) { // disable if animating
					for(size_t v = m_n_zoomed_into_camera_vertex + 1, n = m_solution.vertex_list.size(); v < n; ++ v) {
						if(m_solution.vertex_list[v].b_Has_Pose()) {
							Assume_VertexPose(v);
							break;
						}
					}
				}
				break;
			case '/':
				if(m_solution.vertex_range_list.size() == 1) { // disable if animating
					for(size_t v = m_n_zoomed_into_camera_vertex, n = m_solution.vertex_list.size(); v > 0 && v < n;) {
						-- v;
						if(m_solution.vertex_list[v].b_Has_Pose()) {
							Assume_VertexPose(v);
							break;
						}
					}
				}
				break;
			case ' ':
				m_b_selection_lock = !m_b_selection_lock;
				break;
			case 'p':
				/*printf("static Vector3f m_v_camera_pos(%g, %g, %g);\n"
					"static float m_f_angle_x = %g, m_f_angle_y = %g, m_f_angle_z = %g;\n",
					m_v_camera_pos.x, m_v_camera_pos.y, m_v_camera_pos.z, m_f_angle_x, m_f_angle_y, m_f_angle_z);*/
				printf("-cp %g:%g:%g:%g:%g:%g\n", m_v_camera_pos.x, m_v_camera_pos.y,
					m_v_camera_pos.z, m_f_angle_x, m_f_angle_y, m_f_angle_z); // commandline
				break;
			case 'v':
				{
					std::string s_pattern = m_t_args.p_s_screenshot_dest; // throws
					bool b_is_numbered_pattern = s_pattern.find('#') != std::string::npos;
					if(b_is_numbered_pattern && !CGLThreadedFrameWriter::Make_Pattern(s_pattern)) {
						fprintf(stderr, "error: uncaught exception: \'%s\'\n", "invalid pattern");
						return;
					}
					std::string s_filename;
					if(b_is_numbered_pattern) {
						for(int i = 0; i < 10000; ++ i) {
							stl_ut::Format(s_filename, s_pattern.c_str(), i);
							if(!TFileInfo(s_filename.c_str()).b_exists)
								break;
						}
						// find an unused screenshot number
					} else
						s_filename.swap(s_pattern); // use the pattern in place of the filename

					m_b_render_running = true;
					CClearFlagUponExit guard(m_b_render_running);
					m_b_screenshot_render_running = true;
					CClearFlagUponExit guard2(m_b_screenshot_render_running); // can use jitter

					CGLThreadedFrameWriter writer;
					writer.Set_Framerate(m_t_args.f_video_fps);
					if(!m_t_args.b_screenshot_follows_window)
						writer.Set_Resolution(m_t_args.n_video_width, m_t_args.n_video_height);
					else {
						float f_ratio = float(m_n_width) / m_n_height;
						int n_apply_width, n_apply_height;
						//if(n_video_width * int(n_video_width / f_ratio) > int(n_video_height * f_ratio) * n_video_height) { // drop rounding
						//if(n_video_width * n_video_width / f_ratio > n_video_height * f_ratio * n_video_height) { // divide by n_video_height * n_video_height (always positive, comparison direction stays)
						//if(n_video_width * n_video_width / (f_ratio * n_video_height * n_video_height) > f_ratio) { // multiply by f_ratio (always positive, comparison direction stays)
						//if(n_video_width * n_video_width / (n_video_height * n_video_height) > f_ratio * f_ratio) { // take a square root (ok, we're not interested in the negative roots)
						if(float(m_t_args.n_video_width) / m_t_args.n_video_height > f_ratio) { // take the bigger area screenshot (can be dangerously big if using really narrow ratios; then just set -vw and -vh to the same dimension)
							n_apply_width = m_t_args.n_video_width;
							n_apply_height = int(m_t_args.n_video_width / f_ratio);
						} else {
							n_apply_width = int(m_t_args.n_video_height * f_ratio);
							n_apply_height = m_t_args.n_video_height;
						}
						writer.Set_Resolution(n_apply_width, n_apply_height);
					}
					try {
						if(!m_t_args.b_video_omni) {
							writer.Render_Single(s_filename.c_str(),
								::RenderFrame, m_anim.f_Time()); // render the animation
						} else {
							writer.Render_Single_Omni(s_filename.c_str(),
								::RenderFrame_Mod, m_anim.f_Time()); // render the animation
						}
					} catch(std::exception &r_exc) {
						fprintf(stderr, "error: uncaught exception: \'%s\'\n", r_exc.what());
						return;
					}
					fprintf(stderr, "written \'%s\'\n", s_filename.c_str());
				}
				break;
			case 'x':
				{
					std::string s_anim_dest = m_t_args.p_s_animation_dest; // throws
					CPath::Get_Path(s_anim_dest);
					TFileInfo dest_dir(s_anim_dest);
					if(!dest_dir.b_exists || !dest_dir.b_directory) {
						fprintf(stderr, "error: invalid animation destination "
							"directory selected (\'%s\')\n", s_anim_dest.c_str());
						return;
					}
				}
				if(!m_anim.b_Empty()) {
					m_b_render_running = true;
					CClearFlagUponExit guard(m_b_render_running);

					m_anim.onSaveAnimation("animation_render-time-backup.tx~");
					// be nice, sometimes the renders take long time and the user may be forced
					// to kill the process (could add ctrl+c handler to renders too)

					m_anim.onPlayAnimation();
					if(m_anim.b_Paused())
						m_anim.onTogglePause();
					m_anim.onEnableExtrapolation(); // enable extrapolation so that the last frame is not broken if the animation disables in the middle of the temporal AA
					CGLThreadedFrameWriter writer;
					writer.Set_Framerate(m_t_args.f_video_fps);
					writer.Set_Resolution(m_t_args.n_video_width, m_t_args.n_video_height);
					try {
						if(!m_t_args.b_video_omni) {
							writer.Render_Sequence(m_t_args.p_s_animation_dest,
								::RenderFrame, 0, m_anim.f_AnimationLength()); // render the animation
						} else {
							writer.Render_Sequence_Omni(m_t_args.p_s_animation_dest,
								::RenderFrame_Mod, 0, m_anim.f_AnimationLength()); // render the animation
						}
					}/* catch(std::runtime_error &r_exc) {
						fprintf(stderr, "error: uncaught runtime error: \'%s\'\n", r_exc.what());
						return;
					}*/ catch(std::exception &r_exc) { // one to catch them all
						fprintf(stderr, "error: uncaught exception: \'%s\'\n", r_exc.what());
						m_anim.onResetAnimation(); // don't animate
						return;
					}
					m_anim.onResetAnimation();
					m_anim.onDisableExtrapolation();
				} else
					fprintf(stderr, "error: no animation loaded\n");
				break;
			default:
				if(m_n_zoomed_into_camera_vertex && m_anim.b_Active() && n_key_code == ',')
					m_b_zoom_into_camera_auto_pause = false; // if the user paused before, do not autopause afterwards, would confuse her
				CCameraAnimation::onKeyPress(m_anim, n_key_code, t_Camera(m_anim.f_Time()));
				if(m_anim.b_Active()) {
					if(m_n_zoomed_into_camera_vertex != size_t(-1)) {
						// title already set
					} else
						glutSetWindowTitle(m_t_args.p_s_window_title); // if we started animation, hide mouseover selection text
				} else if(m_n_zoomed_into_camera_vertex != size_t(-1)) {
#ifdef _DEBUG
					printf("debug: animation stopped: not centered on vertex "
						PRIsize " anymore\n", m_n_zoomed_into_camera_vertex);
#endif // _DEBUG
					m_n_zoomed_into_camera_vertex = size_t(-1);
					glutSetWindowTitle(m_t_args.p_s_window_title);
				}
				break;
			}
			glutPostRedisplay();
		}
	}

	bool Export_FalseVertexColors(const char *p_s_vertex_colors_export_file) // throw(std::bad_alloc)
	{
		std::vector<Vector3ub> false_colors(m_solution.vertex_list.size());
		// allocate space for false colors

		const std::vector<float> &r_colors = (m_marginals.cov_list.empty())?
			m_graph.degree_list : m_marginals.cov_list;
		if(false_colors.size() != r_colors.size())
			return false;
		// get a list of quantities

		TBmp *p_gradient = CTgaCodec::p_Load_TGA("gradient.tga");
		uint32_t *p_colors = (p_gradient)? p_gradient->p_buffer : CGradientImage::p_Decompress();
		if(!p_colors) {
			_ASSERTE(!p_gradient); // make sure we're not leaving memory leaks
			return false;
		}
		const int n_grad_width = (p_gradient)? p_gradient->n_width : CGradientImage::n_Width();
		const int n_grad_height = (p_gradient)? p_gradient->n_height : CGradientImage::n_Height();
		const int n_grad_size = std::max(n_grad_width, n_grad_height);
		// get the gradient image

		for(size_t i = 0, n = m_solution.vertex_list.size(); i < n; ++ i) {
			int color_pos = min(int(n_grad_size * max(.0f, std::min(1.0f, r_colors[i]))), n_grad_size - 1);
			// discretize the color

			uint32_t raw = p_colors[n_grad_width * min(color_pos, n_grad_height - 1) +
				min(color_pos, n_grad_width - 1)];
			// sample pixels from the diagonal so that the gradient may be horizontal or vertical

			false_colors[i].x = raw & 255;
			false_colors[i].y = (raw >> 8) & 255;
			false_colors[i].z = (raw >> 16) & 255;
			//false_colors[i].w = (raw >> 24) & 255; // alpha unused
			// get color
		}
		if(p_gradient)
			p_gradient->Delete();
		else
			delete[] p_colors;

		false_colors.swap(m_solution.vertex_color_list);
		bool b_result = m_solution.Save_VertexColors(p_s_vertex_colors_export_file);
		false_colors.swap(m_solution.vertex_color_list);

		return b_result;
	}

	/**
	 *	@brief prints optimized structure into file with .off format
	 *
	 *	@param[in] r_vertex_list is list of 3D points
	 *	@param[in] r_cov_list is list of uncertainties in range [0, 1] one for each point
	 *	@param[in] p_s_filename is output file name
	 *
	 *	@return Returns true on success, false on failure.
	 */
	static bool Export_COFF(const std::vector<CVertex> &r_vertex_list,
		const std::vector<float> &r_cov_list, const char *p_s_filename)
	{
		TBmp *p_gradient = CTgaCodec::p_Load_TGA("gradient.tga");
		uint32_t *p_colors = (p_gradient)? p_gradient->p_buffer : CGradientImage::p_Decompress();
		if(!p_colors) {
			_ASSERTE(!p_gradient); // make sure we're not leaving memory leaks
			return false;
		}
		const int n_grad_width = (p_gradient)? p_gradient->n_width : CGradientImage::n_Width();
		const int n_grad_height = (p_gradient)? p_gradient->n_height : CGradientImage::n_Height();
		const int n_grad_size = std::max(n_grad_width, n_grad_height);
		// get the gradient image

		FILE *p_fw;
#if defined(_MSC_VER) && !defined(__MWERKS__) && _MSC_VER >= 1400
		if(fopen_s(&p_fw, p_s_filename, "w"))
#else //_MSC_VER && !__MWERKS__ && _MSC_VER >= 1400
		if(!(p_fw = fopen(p_s_filename, "w")))
#endif //_MSC_VER && !__MWERKS__ && _MSC_VER >= 1400
			return false;
		fprintf(p_fw, "COFF\n" PRIsize " 0 0\n", r_vertex_list.size());
		// open the output file

		_ASSERTE(r_vertex_list.size() == r_cov_list.size());
		for(size_t a = 0, n = r_vertex_list.size(); a < n; ++ a) {
			int color_pos = min(int(n_grad_size * max(.0f, min(1.0f, r_cov_list[a]))), n_grad_size - 1);
			// discretize the color

			uint32_t raw = p_colors[n_grad_width * min(color_pos, n_grad_height - 1) +
				min(color_pos, n_grad_width - 1)];
			// sample pixels from the diagonal so that the gradient may be horizontal or vertical

			int r = raw & 255;
			int g = (raw >> 8) & 255;
			int b = (raw >> 16) & 255;
			//int a = (raw >> 24) & 255; // alpha unused
			// get color

			Vector3f v_position = r_vertex_list[a].v_Position();
			fprintf(p_fw, "%f %f %f %d %d %d 255\n", v_position.x, v_position.z, v_position.y, r, g, b); // we flipped y, z when loading, undo it now
		}
		// write output

		if(p_gradient)
			p_gradient->Delete();
		else
			delete[] p_colors;
		// cleanup

		if(ferror(p_fw)) {
			fclose(p_fw);
			return false;
		}
		return !fclose(p_fw);
	}

	/**
	 *	@brief prints optimized structure into file with .off format
	 *
	 *	@param[in] r_vertex_list is list of 3D points
	 *	@param[in] r_color_list is list of RGB8 colors, one for each point
	 *	@param[in] p_s_filename is output file name
	 *
	 *	@return Returns true on success, false on failure.
	 */
	static bool Export_COFF(const std::vector<CVertex> &r_vertex_list,
		const std::vector<Vector3ub> &r_color_list, const char *p_s_filename)
	{
		FILE *p_fw;
#if defined(_MSC_VER) && !defined(__MWERKS__) && _MSC_VER >= 1400
		if(fopen_s(&p_fw, p_s_filename, "w"))
#else //_MSC_VER && !__MWERKS__ && _MSC_VER >= 1400
		if(!(p_fw = fopen(p_s_filename, "w")))
#endif //_MSC_VER && !__MWERKS__ && _MSC_VER >= 1400
			return false;
		fprintf(p_fw, "COFF\n" PRIsize " 0 0\n", r_vertex_list.size());
		// open the output file

		_ASSERTE(r_vertex_list.size() == r_color_list.size());
		for(size_t a = 0, n = r_vertex_list.size(); a < n; ++ a) {
			Vector3ub v_color = r_color_list[a]; // get color
			Vector3f v_position = r_vertex_list[a].v_Position();
			fprintf(p_fw, "%f %f %f %d %d %d 255\n",
				v_position.x, v_position.z, v_position.y, // we flipped y, z when loading, undo it now
				v_color.x, v_color.y, v_color.z);
		}
		// write output

		if(ferror(p_fw)) {
			fclose(p_fw);
			return false;
		}
		return !fclose(p_fw);
	}

protected:
	static TBmp *p_Load_GradientImage() // throw(std::bad_alloc)
	{
		TBmp *p_gradient = CTgaCodec::p_Load_TGA("gradient.tga");
		if(!p_gradient) {
			p_gradient = new TBmp;
			p_gradient->b_alpha = CGradientImage::b_Alpha();
			p_gradient->b_grayscale = CGradientImage::b_Grayscale();
			p_gradient->n_width = CGradientImage::n_Width();
			p_gradient->n_height = CGradientImage::n_Height();
			p_gradient->n_former_bpc = 8;
			if(!(p_gradient->p_buffer = CGradientImage::p_Decompress())) {
				delete p_gradient;
				throw std::bad_alloc(); // rethrow
			}
		}
		// load the gradient image

		return p_gradient;
	}

	bool Initialize_Megatexture() // throw(std::bad_alloc)
	{
		std::vector<std::string> image_list;
		if(!Resolve_FileList(image_list, m_t_args.p_s_camera_proj_input_images)) // use the arg from the commandline
			return false;
		// get images

		if(!image_list.empty()) {
			if(!CGLES20ExtensionHandler::b_SupportedExtension("GL_EXT_texture_array")) // needed for megatextures
				fprintf(stderr, "warning: GL_EXT_texture_array not supported\n");
			else {
				int n_max_size, n_max_layers;
				glGetIntegerv(GL_MAX_ARRAY_TEXTURE_LAYERS_EXT, &n_max_layers);
				glGetIntegerv(GL_MAX_TEXTURE_SIZE, &n_max_size);
				printf("GL_EXT_texture_array supported, max size: %d x %d x %d\n",
					n_max_size, n_max_size, n_max_layers);
				printf("with tile size 128 x 128 it is possible to have %gk x %gk textures (1k = 1024 px)\n",
					(n_max_size * 128) / 1024.0f, (n_max_size * 128) / 1024.0f);
			}
			// see if there is GL_EXT_texture_array

			CImageInfo image_info(image_list);
			if(image_info.n_FailedImage_Num()) {
				fprintf(stderr, "error: " PRIsize " of the images failed to load\n", image_info.n_FailedImage_Num());
				return false;
			}
			printf("debug: have %.2g texels in " PRIsize " images\n",
				double(image_info.n_Pixel_Num()), image_info.n_Image_Num());
			// scan the files, read resolutions

			CVirtualTextureAtlas atlas_builder(8);
			for(size_t i = 0, n = image_info.n_Image_Num(); i < n; ++ i) {
				size_t n_id = atlas_builder.n_Add_Image(image_info.t_ImageInfo(i).n_width,
					image_info.t_ImageInfo(i).n_height);
				if(n_id != i)
					return false;
			}

			int n_max_2D_texture_size = min(16384, CGLTexture_2D::n_Max_Size()); //4096 //CGLTexture_2D::n_Max_Size();
			int n_max_mega_texture_size = n_max_2D_texture_size * 128; // size of the megatexture

			//n_max_2D_texture_size = n_max_mega_texture_size;

			if(!atlas_builder.Solve_BinPacking(n_max_2D_texture_size))
				return false;
			// solve bin packing

			/*for(size_t i = 0, j = 0, n = image_info.n_Image_Num(); i < n; ++ i) {
				TBmp t_image_info = image_info.t_ImageInfo(i);
				if(!m_image_loader.Enqueue_FetchImage(image_info.s_ImageFilename(i),
				   t_image_info.n_width, t_image_info.n_height, 0)) {
					fprintf(stderr, "error: failed to fetch \'%s\'\n",
						image_info.p_s_ImageFilename(i));
				}

				if(i % 5 == 4) {
					Sleep(1000);

					for(size_t m = min(j + 5, i + 1); j < m; ++ j) {
						TBmp t_image_info = image_info.t_ImageInfo(j);
						const TBmp *p_image = m_image_loader.p_LookupImage(image_info.s_ImageFilename(j),
						   t_image_info.n_width, t_image_info.n_height, true, false);
						if(!p_image) {
							fprintf(stderr, "error: failed to load \'%s\'\n",
								image_info.p_s_ImageFilename(j));
						} else {
							std::pair<const char*, const char*> t_fn =
								CPath::t_ShortFileName(image_info.p_s_ImageFilename(j));
							printf("debug: loaded \'%s%s\' (%d x %d)\n",
								t_fn.first, t_fn.second, p_image->n_width, p_image->n_height);
							m_image_loader.Return_Image(p_image); // don't forget to return
						}
					}
					// see if all the images were loaded
				}
				// now and then get the images
			}*/
			// a simple texture loader test

			m_p_megatexture = new CGLTexture_2D(max(8U, atlas_builder.n_Atlas_Width()),
				max(8U, atlas_builder.n_Atlas_Height()), GL_RGB, true);
			CGLTexture_2D &atlas_texture = *m_p_megatexture;
			if(!atlas_texture.b_Status()) {
				fprintf(stderr, "error: failed to create atlas texture\n");
				return false;
			}
			CGLTextureScalingUploader texture_uploader(GL_RGB);
			if(!texture_uploader.b_Status()) {
				fprintf(stderr, "error: failed to create texture uploader\n");
				return false;
			}

			m_cam_texcoord_list.resize(atlas_builder.n_ImageRect_Num());
			for(size_t i = 0, n = atlas_builder.n_ImageRect_Num(); i < n; ++ i) {
				float f_x = float(atlas_builder.r_ImageRect(i).n_x) / atlas_texture.n_Width();
				float f_y = float(atlas_builder.r_ImageRect(i).n_y) / atlas_texture.n_Height();
				float f_w = float(atlas_builder.r_ImageRect(i).n_width) / atlas_texture.n_Width();
				float f_h = float(atlas_builder.r_ImageRect(i).n_height) / atlas_texture.n_Height();
				m_cam_texcoord_list[i] = Vector4f(f_x, f_y, f_x + f_w, f_y + f_h);
			}

			glDisable(GL_DEPTH_TEST);
			glDisable(GL_CULL_FACE);
			texture_uploader.Set_Target(atlas_texture);
			/*glClearColor(.75f, .75f, .75f, 0);
			glClear(GL_COLOR_BUFFER_BIT);*/
			texture_uploader.Debug_FillTileImages(128, 0);

			if(0) {
				for(size_t i = 0, j = 0, n = image_info.n_Image_Num(); i < n; ++ i) {
					for(size_t m = min(i + 8, n); j < m; ++ j) {
						CVirtualTextureAtlas::TImageRectangle t_rect = atlas_builder.r_ImageRect(j);
						if(!m_image_loader.Enqueue_FetchImage(image_info.s_ImageFilename(j),
						  t_rect.n_width, t_rect.n_height, 0)) {
							fprintf(stderr, "error: failed to fetch \'%s\'\n",
								image_info.p_s_ImageFilename(j));
							return false;
						}
						// fetch an image
					}
					// fetch a few images ahead to make the reading threads busy (not too many,
					// we're unable to cope with running out of memory in here)

					CVirtualTextureAtlas::TImageRectangle t_rect = atlas_builder.r_ImageRect(i);
					const TBmp *p_image = m_image_loader.p_LookupImage(image_info.s_ImageFilename(i),
					   t_rect.n_width, t_rect.n_height, true, false);
					// and wait for it to be loaded (a bit slower like that but easier to write code;
					// in the real megatexture we will just poll on frame by frame basis)

					if(!p_image) {
						fprintf(stderr, "error: failed to load \'%s\'\n",
							image_info.p_s_ImageFilename(i));
						return false;
					} else {
						std::pair<const char*, const char*> t_fn =
							CPath::t_ShortFileName(image_info.p_s_ImageFilename(i), 60);
						printf("debug: loaded \'%s%s\' (%d x %d)   \r",
							t_fn.first, t_fn.second, p_image->n_width, p_image->n_height);

						texture_uploader.Draw(t_rect.n_x, t_rect.n_y, t_rect.n_width, t_rect.n_height,
							p_image, atlas_builder.n_Image_Padding(), /*GL_REPEAT*/ /*GL_MIRRORED_REPEAT*/ GL_CLAMP_TO_EDGE);

						m_image_loader.Return_Image(p_image); // don't forget to return
					}
				}
				// a simple "imperative" filling of the atlas (fill by texture)
			} else if(1) {
				const int n_padding = atlas_builder.n_Image_Padding();
				const int n_tile_size = 128;

				for(size_t i = 0, j = 0, n = image_info.n_Image_Num(); i < n; ++ i) {
					for(size_t m = min(i + 8, n); j < m; ++ j) {
						CVirtualTextureAtlas::TImageRectangle t_rect = atlas_builder.r_ImageRect(j);
						if(!m_image_loader.Enqueue_FetchImage(image_info.s_ImageFilename(j),
						  t_rect.n_width, t_rect.n_height, 0)) {
							fprintf(stderr, "error: failed to fetch \'%s\'\n",
								image_info.p_s_ImageFilename(j));
							return false;
						}
						// fetch an image
					}
					// fetch a few images ahead to make the reading threads busy (not too many,
					// we're unable to cope with running out of memory in here)

					CVirtualTextureAtlas::TImageRectangle t_rect = atlas_builder.r_ImageRect(i);
					const TBmp *p_image = m_image_loader.p_LookupImage(image_info.s_ImageFilename(i),
					   t_rect.n_width, t_rect.n_height, true, false);
					// and wait for it to be loaded (a bit slower like that but easier to write code;
					// in the real megatexture we will just poll on frame by frame basis)

					if(!p_image) {
						fprintf(stderr, "error: failed to load \'%s\'\n",
							image_info.p_s_ImageFilename(i));
						return false;
					} else {
						std::pair<const char*, const char*> t_fn =
							CPath::t_ShortFileName(image_info.p_s_ImageFilename(i), 60);
						printf("debug: loaded " PRIsize " / " PRIsize " \'%s%s\' (%d x %d)   \r",
							i + 1, n, t_fn.first, t_fn.second, p_image->n_width, p_image->n_height);

						int n_first_tile_x = (t_rect.n_x - n_padding) / n_tile_size;
						int n_last_tile_x = (t_rect.n_x + n_padding + t_rect.n_width - 1 /*+ n_tile_size - 1*/) / n_tile_size; // inclusive
						int n_first_tile_y = (t_rect.n_y - n_padding) / n_tile_size;
						int n_last_tile_y = (t_rect.n_y + n_padding + t_rect.n_height - 1 /*+ n_tile_size - 1*/) / n_tile_size; // inclusive

						//texture_uploader.Draw(t_rect.n_x, t_rect.n_y, t_rect.n_width, t_rect.n_height,
						//	p_image, atlas_builder.n_Image_Padding(), /*GL_REPEAT*/ /*GL_MIRRORED_REPEAT*/ GL_CLAMP_TO_EDGE);

						float f_texture_to_rectangle_scale_x = 1.0f / t_rect.n_width;
						float f_texture_to_rectangle_scale_y = 1.0f / t_rect.n_height;
						// assume that the scale in height is the same (but likely changes slightly due to rounding)

						for(int n_tile_y = n_first_tile_y; n_tile_y <= n_last_tile_y; ++ n_tile_y) {
							for(int n_tile_x = n_first_tile_x; n_tile_x <= n_last_tile_x; ++ n_tile_x) {
								/*if((n_tile_x ^ n_tile_y) & 1)
									continue;*/
								// debug - skip tiles in checkterboard pattern to make sure there is no overdraw

								int n_x0 = max(int(t_rect.n_x) - n_padding, n_tile_x * n_tile_size);
								int n_y0 = max(int(t_rect.n_y) - n_padding, n_tile_y * n_tile_size);
								int n_x1 = min(int(t_rect.n_x) + n_padding + int(t_rect.n_width),
									(n_tile_x + 1) * n_tile_size);
								int n_y1 = min(int(t_rect.n_y) + n_padding + int(t_rect.n_height),
									(n_tile_y + 1) * n_tile_size);
								// get rectangle in the megatexture coordinates (those are all positive)

								float n_src_x0 = (n_x0 - int(t_rect.n_x)) * f_texture_to_rectangle_scale_x;
								float n_src_y0 = (n_y0 - int(t_rect.n_y)) * f_texture_to_rectangle_scale_y;
								float n_src_x1 = (n_x1 - int(t_rect.n_x)) * f_texture_to_rectangle_scale_x;
								float n_src_y1 = (n_y1 - int(t_rect.n_y)) * f_texture_to_rectangle_scale_y;
								// and the corresponding rectangle in source texture coordinates (this will be *fun* under scale)

								texture_uploader.Draw(n_x0, n_y0, n_x1 - n_x0, n_y1 - n_y0,
									n_src_x0, n_src_y0, n_src_x1 - n_src_x0, n_src_y1 - n_src_y0,
									p_image, GL_CLAMP_TO_EDGE, n_tile_x == n_first_tile_x &&
									n_tile_y == n_first_tile_y);
								// upload tile by tile
							}
						}

						m_image_loader.Return_Image(p_image); // don't forget to return
					}
				}
				// a more complicated "imperative" filling of the atlas (fill by texture and tile by tile)
			} else if(0) {
				const int n_padding = atlas_builder.n_Image_Padding();
				const int n_tile_size = 128;

				std::multimap<Vector2i, size_t> tile_images;
				for(size_t i = 0, j = 0, n = atlas_builder.n_ImageRect_Num(); i < n; ++ i) {
					CVirtualTextureAtlas::TImageRectangle t_rect = atlas_builder.r_ImageRect(i);

					int n_first_tile_x = (t_rect.n_x - n_padding) / n_tile_size;
					int n_last_tile_x = (t_rect.n_x + n_padding + t_rect.n_width - 1) / n_tile_size; // inclusive
					int n_first_tile_y = (t_rect.n_y - n_padding) / n_tile_size;
					int n_last_tile_y = (t_rect.n_y + n_padding + t_rect.n_height - 1) / n_tile_size; // inclusive

					for(int n_tile_y = n_first_tile_y; n_tile_y <= n_last_tile_y; ++ n_tile_y) {
						for(int n_tile_x = n_first_tile_x; n_tile_x <= n_last_tile_x; ++ n_tile_x)
							tile_images.insert(std::make_pair(Vector2i(n_tile_x, n_tile_y), i));
					}
				}
				// make a lookup of the images to tiles; this is specific to our implementation that
				// uses the megatexture as texture atlas. true megatexture would have the tiles stored
				// in the opportune order and sizes (and also probably prescaled with mipmaps)

				int n_tile_num_x = (atlas_builder.n_Atlas_Width() + n_tile_size - 1) / n_tile_size;
				int n_tile_num_y = (atlas_builder.n_Atlas_Height() + n_tile_size - 1) / n_tile_size;
				// number of tiles in both directions

#if 0
				for(int n_tile_y = 0; n_tile_y < n_tile_num_y; ++ n_tile_y) {
					for(int n_tile_x = 0; n_tile_x < n_tile_num_x; ++ n_tile_x) { // could use a Hilbert curve here to improve locality
#else
				int n_hilbert_level = n_Log2_Ceil(max(n_tile_num_x, n_tile_num_y));
				std::vector<Vector2i> traverse_order;
				CFastHilbert::Generate(traverse_order, n_hilbert_level);
				for(size_t h = 0, l = traverse_order.size(); h < l; ++ h) {
					int n_tile_x = traverse_order[h].x, n_tile_y = traverse_order[h].y;
					if(n_tile_x < n_tile_num_x && n_tile_y < n_tile_num_y) {
#endif
						typedef std::multimap<Vector2i, size_t>::const_iterator _TyImgIt;
						std::pair<_TyImgIt, _TyImgIt> images =
							tile_images.equal_range(Vector2i(n_tile_x, n_tile_y));
						// get a list of images in this particular tile

						for(_TyImgIt p_img_it = images.first; p_img_it != images.second; ++ p_img_it) {
							size_t n_img_id = (*p_img_it).second;

							CVirtualTextureAtlas::TImageRectangle t_rect = atlas_builder.r_ImageRect(n_img_id);
							if(!m_image_loader.Enqueue_FetchImage(image_info.s_ImageFilename(n_img_id),
							  t_rect.n_width, t_rect.n_height, 0)) {
								fprintf(stderr, "error: failed to fetch \'%s\'\n",
									image_info.p_s_ImageFilename(n_img_id));
								return false;
							}
							// fetch an image
						}
						// fetch all the images in the tile (some may already have been loaded)

						for(_TyImgIt p_img_it = images.first; p_img_it != images.second; ++ p_img_it) {
							size_t n_img_id = (*p_img_it).second;

							CVirtualTextureAtlas::TImageRectangle t_rect = atlas_builder.r_ImageRect(n_img_id);
							const TBmp *p_image = m_image_loader.p_LookupImage(image_info.s_ImageFilename(n_img_id),
							   t_rect.n_width, t_rect.n_height, true, false);
							// and wait for it to be loaded (a bit slower like that but easier to write code;
							// in the real megatexture we will just poll on frame by frame basis)

							if(!p_image) {
								fprintf(stderr, "error: failed to load \'%s\'\n",
									image_info.p_s_ImageFilename(n_img_id));
								return false;
							}
							std::pair<const char*, const char*> t_fn =
								CPath::t_ShortFileName(image_info.p_s_ImageFilename(n_img_id));
							printf("debug: (re)referenced \'%s%s\' (%d x %d)   \r",
								t_fn.first, t_fn.second, p_image->n_width, p_image->n_height);

							float f_texture_to_rectangle_scale_x = 1.0f / t_rect.n_width;
							float f_texture_to_rectangle_scale_y = 1.0f / t_rect.n_height;
							// assume that the scale in height is the same (but likely changes slightly due to rounding)

							int n_x0 = max(int(t_rect.n_x) - n_padding, n_tile_x * n_tile_size);
							int n_y0 = max(int(t_rect.n_y) - n_padding, n_tile_y * n_tile_size);
							int n_x1 = min(int(t_rect.n_x) + n_padding + int(t_rect.n_width),
								(n_tile_x + 1) * n_tile_size);
							int n_y1 = min(int(t_rect.n_y) + n_padding + int(t_rect.n_height),
								(n_tile_y + 1) * n_tile_size);
							// get rectangle in the megatexture coordinates (those are all positive)

							float n_src_x0 = (n_x0 - int(t_rect.n_x)) * f_texture_to_rectangle_scale_x;
							float n_src_y0 = (n_y0 - int(t_rect.n_y)) * f_texture_to_rectangle_scale_y;
							float n_src_x1 = (n_x1 - int(t_rect.n_x)) * f_texture_to_rectangle_scale_x;
							float n_src_y1 = (n_y1 - int(t_rect.n_y)) * f_texture_to_rectangle_scale_y;
							// and the corresponding rectangle in source texture coordinates (this will be *fun* under scale)

							texture_uploader.Draw(n_x0, n_y0, n_x1 - n_x0, n_y1 - n_y0,
								n_src_x0, n_src_y0, n_src_x1 - n_src_x0, n_src_y1 - n_src_y0,
								p_image, GL_CLAMP_TO_EDGE, true); // this is *very* inefficient, we always the upload the image again and again, definitely need to batch this
							// upload tile by tile

							m_image_loader.Return_Image(p_image); // don't forget to return
							// todo - make a guard for this, this is easy to forget
						}
					}
				}

				// a practical "on-demand" filling of the atlas (load the textures by their tile refs;
				// can get slow due to the order of tile loads and cache size--this would presumedly
				// not happen at runtime as we see more or less contiguous parts of the texture)
				// alternatively, we could first get a list of tiles to upload, get a shopping list of
				// textures and then for each texture upload into all the tiles at once before returning it
			}

			atlas_texture.Bind();
			glGenerateMipmap(atlas_texture.n_Target());
			float f_anisotropy;
			glGetFloatv(GL_MAX_TEXTURE_MAX_ANISOTROPY_EXT, &f_anisotropy);
			glTexParameteri(atlas_texture.n_Target(), GL_TEXTURE_MAX_ANISOTROPY_EXT, f_anisotropy);
			// generate mipmaps, just for the time being until we have a true megatexture implemented

			printf("\ndone\n");

			/*TBmp *p_readback = TBmp::p_Alloc(atlas_texture.n_Width(), atlas_texture.n_Height());
			texture_uploader.Debug_ReadBack(p_readback);
			CPngCodec::Save_PNG("debug_tex_atlas_tiled.png", *p_readback, true);
			p_readback->Delete();*/
			// save image of the page in the texture atlas (this is slow, especially with large pages)

			texture_uploader.Release();

			m_image_loader.Show_Stats();
		}
		// initialize megatexture

		return true;
	}
};

static CGLBundle *p_gl_bundle = 0; // points to an object in main()

void RenderFrame(double f_time)
{
	if(p_gl_bundle)
		p_gl_bundle->RenderFrame(f_time);
}

void RenderFrame_Mod(double f_time, const Matrix4f &r_t_view_mod)
{
	if(p_gl_bundle)
		p_gl_bundle->RenderFrame(f_time, r_t_view_mod);
}

void onResize(int n_new_width, int n_new_height)
{
	if(p_gl_bundle)
		p_gl_bundle->onResize(n_new_width, n_new_height);
}

// here we render frame
void onDisplay()
{
	if(p_gl_bundle)
		p_gl_bundle->onDisplay();

	int n;
	if((n = glGetError()) != GL_NO_ERROR)
		fprintf(stderr, "error: OpenGL error(s) occured during drawing: 0x%04x\n", n);
}

static int n_button = -1; // button for onMouseMotion()

void onMouseClick(int _n_button, int n_state, int n_x, int n_y)
{
	if(p_gl_bundle)
		p_gl_bundle->onMouseClick(_n_button, n_state, n_x, n_y);
	n_button = _n_button;
}

void onMouseMotion(int n_x, int n_y)
{
	if(p_gl_bundle)
		p_gl_bundle->onMouseMotion(n_button, n_x, n_y);
}

void onPassiveMouseMotion(int n_x, int n_y)
{
	if(p_gl_bundle)
		p_gl_bundle->onPassiveMouseMotion(n_x, n_y);
}

void onKeyPressed(unsigned char n_key_code, int n_x, int n_y)
{
	if(p_gl_bundle)
		p_gl_bundle->onKeyPressed(n_key_code, n_x, n_y);
}

#if 0
template <const int n_element_num, template <const int> CElementConstructor, const int n_first_element = 0>
class CTypelistIOTA {
public:
	typedef CTypelist<typename CElementConstructor<n_first_element>::_TyResult,
		typename CTypelistIOTA<n_element_num - 1, CElementConstructor, n_first_element + 1>::_TyResult> _TyResult;
};

template <template <const int> CElementConstructor, const int n_first_element>
class CTypelistIOTA<0, CElementConstructor, n_first_element> {
public:
	typedef CTypelistEnd _TyResult;
};

template <const int n_element_mask, class CElemType = double>
class CVectorSelector {
public:
	typedef CElemType _TyElemType;

public:
	template <const int n_vector_dimension>
	static inline Eigen::Vector<_TyElemType, n_SetBit_Num_Static(n_element_mask & n_Mask(n_vector_dimension)), 1>
		v_Select_MaskedElements(Eigen::Vector<_TyElemType, n_vector_dimension, 1> v)
	{
		enum {
			n_selected_elem_num = n_SetBit_Num_Static(n_element_mask & n_Mask(n_vector_dimension))
		};

		Eigen::Vector<_TyElemType, n_selected_elem_num, 1> result;
		for(int i = 0; i < n_vector_dimension; ++ i) {
			if(n_element_mask & (1 << i)) {
				int n_dest_index = n_SetBit_Num(n_element_mask & ((1 << i) - 1));
				result(n_dest_index) = v(i);
			}
		}

		return result;
	}
};

enum {
	vertex_cam_FocalLength = 0x..,
	vertex_cam_Skew = 0x..,
	vertex_cam_OpticalCenter = 0x..,
	// ...
};

template <const int optimized_param_mask = 0x3f> // default = the first 6 params
class CVertexCam : public CBaseSEVertexImpl<CVertexCam, n_Set_Bit_Num_Static(optimized_param_mask & 0x7ff)> {
public:
	enum {
		num_optimized = n_SetBit_Num_Static(optimized_param_mask & 0x7ff), // & 0x7ff -> high bits ignored
		num_fixed = 11 - num_optimized
	};

	eigen::Matrix<double, num_fixed, 1> params;

	CVertexCam(Eigen::Vector<double, 11, 1> state)
		:CBaseSEVertexImpl<CVertexCam, num_optimized>(CVectorSelector<optimized_param_mask>::v_Select_MaskedElements(state)),
		params(CVectorSelector<~optimized_param_mask>::v_Select_MaskedElements(state)) // bitwise inverted mask (higher bits will be ignored)
	{
		// ...
	}

	// ...
};

// used as e.g. CVertexCam<vertex_cam_FocalLength | vertex_cam_OpticalCenter>
#endif // 0

void PrintFlags()
{
#if defined(_M_X64) || defined(_M_AMD64) || defined(_M_IA64) || defined(__x86_64) || defined(__amd64) || defined(__ia64)
	printf("GraphViewer version x64 (compiled at %s)\nbuilt with the following flags:\n", __DATE__);
#else // _M_X64 || _M_AMD64 || _M_IA64 || __x86_64 || __amd64 || __ia64
	printf("GraphViewer version x86 (compiled at %s)\nbuilt with the following flags:\n", __DATE__);
#endif // _M_X64 || _M_AMD64 || _M_IA64 || __x86_64 || __amd64 || __ia64

#ifdef _DEBUG
	printf("%s\n", "_DEBUG");
#endif // _DEBUG
#ifdef _OPENMP
#pragma omp parallel
#pragma omp master
	{
		printf("%s (%d threads)\n", "_OPENMP", omp_get_num_threads());
	}
#endif // _OPENMP
	printf("\n");
}

int main(int n_arg_num, const char **p_arg_list)
{
	try {
		/*ul_bithacks::UnitTests<int8_t>();
		ul_bithacks::UnitTests<uint8_t>();
		ul_bithacks::UnitTests<int16_t>();
		ul_bithacks::UnitTests<uint16_t>();
		ul_bithacks::UnitTests<int32_t>();
		ul_bithacks::UnitTests<uint32_t>();
		ul_bithacks::UnitTests<int64_t>();
		ul_bithacks::UnitTests<uint64_t>();*/

		PrintFlags();

		TCommandlineArgs t_args;
		if(!t_args.Parse(n_arg_num, p_arg_list))
			return -1;
		// parse commandline

		if(t_args.b_fancy_color_points && !t_args.p_s_graph_file)
			fprintf(stderr, "warning: --project-camera-images specified without -g\n");
		// sanity check

		glutInit(&n_arg_num, (char**)p_arg_list);
		glutInitDisplayMode(GLUT_RGBA | GLUT_DEPTH |
#ifdef __APPLE__
			GLUT_3_2_CORE_PROFILE |
#endif // __APPLE__
			GLUT_DOUBLE);
		glutInitWindowSize(t_args.n_width, t_args.n_height);
		glutInitWindowPosition(200, 200);
		glutCreateWindow(t_args.p_s_window_title);
		// init OpenGL using GLUT

		if(CGLES20ExtensionHandler::n_GetGLES20FuncPointers() ||
		   CGLES20ExtensionHandler::n_GetARBVertexArrayObjectFuncPointers())
			fprintf(stderr, "warning: OpenGL ES 2.0 not (fully) supported\n");
		// get extensions

		//glGetString(123456); // fails here
		//glGetIntegerv(123456, 0); // fails here
		//glBindVertexArray(123456); // fails here
		//glShaderSource(123456, 0, 0, 0); // fails here
		// autocheck test

		if(glGetError() != GL_NO_ERROR)
			fprintf(stderr, "error: OpenGL error(s) occured during pre-initialization\n");

		glEnable(0x8642); // GL_PROGRAM_POINT_SIZE
#ifndef __APPLE__
		glEnable(0x8861); // GL_POINT_SPRITE // fails in core profile
#endif // __APPLE__
		//glEnable(0x8862); // GL_COORD_REPLACE // fails even in compatibility profile
		// enable point sprites (GLES doesn't have glPointSize())
		// todo - make a reusable function in the emulator

		printf("controls:\n");
		printf("wsad: move\n");
		printf("[left click]: select a vertex (a .graph must be loaded)\n");
		printf("[ctrl + left click]: modify selection (a .graph must be loaded)\n");
		printf("[shift + left click]: modify selection (a .graph must be loaded)\n");
		printf("[space]: toggle selection lock (a .graph must be loaded)\n");
		printf("[middle click]: assume a pose of a clicked camera (use \',\' to resume the animation)\n");
		printf("e: toggle edge display (a .graph must be loaded)\n");
		printf("q: toggle edge blending mode (a .graph must be loaded)\n");
		printf("tf: change edge alpha (a .graph must be loaded)\n");
		printf("+-: change point size\n");
		printf("/*: navigate between different camera poses\n");
		printf("r: reload input files\n");
		printf("u: toggle upside-down transform\n");
		printf("c: toggle false color display (covariances / vertex degrees)\n");
		printf("g: toggle grid display\n");
		printf("y: toggle camera frustum display\n");
		printf("b: change background color\n");
		printf("i: positions the camera so that 99%% of the points is visible\n");
		printf("3: toggle point cloud texture (see help on -prci)\n");
		printf("12: adjust transparency of camera inset images (see help on -ici)\n");
		printf("p: dump the current camera pose\n\n");
		CCameraAnimation::PrintHelp();
		printf("v: onRenderScreenshot()\n");
		printf("x: onRenderAnimation()\n\n");

		CGLBundle gl_bundle(t_args);
		p_gl_bundle = &gl_bundle;

		/*const char *p_font_list[] = {"Noto Sans CJK jp Regular", "Noto Sans CJK jp Light", "Noto Sans CJK jp Thin",
			"arial", "tahoma", "verdana", "trebuchet ms", "courier new", "lucida console",
			"NotoSans-Regular.ttf", "NotoSerif-Regular.ttf", "NotoMono-Regular.ttf"};
		const int p_size_list[] = {12, 16, 32, 48, 72};
		for(size_t n_font = 0; n_font < sizeof(p_font_list) / sizeof(p_font_list[0]); ++ n_font) {
			const char *p_s_font_face = p_font_list[n_font];
			for(size_t n_size = 0; n_size < sizeof(p_size_list) / sizeof(p_size_list[0]); ++ n_size) {
				int n_font_size = p_size_list[n_size];
				for(int b_advanched_chars = 0; b_advanched_chars < 2; ++ b_advanched_chars)  {
					CBitmapFont font_settings;
					font_settings.Set_FontAntialiasing(true);
					font_settings.Set_GlyphGeometryPadding(2);
					font_settings.Set_GlyphTexturePadding(2);
					font_settings.Set_UnicodeVersion("5.0.0");
					if(b_advanched_chars) {
						font_settings.Request_UnicodeBlock("Basic Latin");
						font_settings.Request_UnicodeBlock("Latin-1 Supplement");
						font_settings.Request_UnicodeBlock("Latin Extended-A");
						font_settings.Request_UnicodeBlock("Latin Extended-B");
						font_settings.Request_UnicodeBlock("Cyrillic");
						font_settings.Request_UnicodeBlock("Cyrillic Supplement");
						//font_settings.Request_UnicodeBlock("CJK Compatibility Ideographs"); // this is chinese in BMP
						//font_settings.Request_UnicodeBlock("CJK Unified Ideographs Extension B"); // this is chinese outside BMP, works on Win7 but only a few characters are there
						//font_settings.Request_UnicodeBlock("Musical Symbols"); // this is outside of BMP; does not seem to work at all (on XP x64), works on Win7 but just adds tofu
						//font_settings.Request_UnicodeBlock("Playing Cards"); // this is outside of BMP; does not seem to work at all (on XP x64), works on Win7 but just adds tofu
						//font_settings.Request_UnicodeBlock("Domino Tiles"); // this is outside of BMP; does not seem to work at all (on XP x64), works on Win7 but just adds tofu
						font_settings.Request_UnicodeBlock("IPA Extensions");
						font_settings.Request_UnicodeBlock("Mathematical Operators");
						font_settings.Request_UnicodeBlock("Combining Diacritical Marks");
						font_settings.Request_UnicodeBlock("Greek and Coptic");
					} else
						font_settings.Request_UnicodeBlock("Basic Latin");
					font_settings.Set_Max_PageSize(min(CGLTexture::n_Max_Size(), 1024));
					// this takes a long time in debug

					char p_s_name_pattern[256];
					stl_ut::Format(p_s_name_pattern, sizeof(p_s_name_pattern), "%s_%d%s_page-%%02d.tga",
						p_s_font_face, n_font_size, (b_advanched_chars)? "_ext" : "");

					CBitmapFont::CBitmapWriter writer(p_s_name_pattern);
					font_settings.Create(p_s_font_face, n_font_size, writer, false);
				}
			}
		}
		// this is how a font is created/**/

		if(glGetError() != GL_NO_ERROR)
			fprintf(stderr, "error: OpenGL error(s) occured during initialization\n");

		if(t_args.b_immed_animation) {
			onKeyPressed('l', 0, 0); // load animation
			onKeyPressed('x', 0, 0); // play & render animation
			printf("done, exiting ...\n");
			return 0;
		}
		if(t_args.b_immed_screenshot) {
			onKeyPressed('v', 0, 0);
			printf("done, exiting ...\n");
			return 0;
		}
		// render screenshot / animation on startup

		glutDisplayFunc(onDisplay);
		glutReshapeFunc(onResize);
		glutMouseFunc(onMouseClick);
		glutMotionFunc(onMouseMotion);
		glutPassiveMotionFunc(onPassiveMouseMotion);
		glutKeyboardFunc(onKeyPressed);
		// register our callbacks

		printf("entering main loop ...\n");

		onKeyPressed('j', 0, 0);
		// reset the animation so that the time zero is now

		glutMainLoop();
		// begin rendering
	} catch(std::bad_alloc&) {
		fprintf(stderr, "error: uncaught std::bad_alloc\n");
		return -1;
	} catch(std::exception &r_exc) {
		fprintf(stderr, "error: uncaught exception: \'%s\'\n", r_exc.what());
		return -1;
	}

	return 0;
}
