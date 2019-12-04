#include <iostream>
#include <cstring>
#include <opencv2/opencv.hpp>
#include <ctime>
#include <random>
#include "ray_tracing_tools_amp.h"
#include "crossable_amp.h"
#include "ray_tracing.h"
#include <amprt.h>
#include <ctime>
#include <mutex>
#include <amp.h>
#include <windows.h>
using namespace std;
using namespace ray_tracing;
using namespace concurrency;
using namespace concurrency::fast_math;
using namespace concurrency::graphics;
#define len(a) (sizeof(a) / sizeof(a[0]))

#define cons_inf (1e10f)
#define cons_pi (3.14159265f)
#define cons_2_pi (3.14159265f * 2.0f)
#define cons_pi_2 (3.14159265f / 2.0f)

#define cons_sample_phong_cnt 64
#define cons_pixel_sample_cnt 2
#define cons_soft_shadow_sample_cnt 64

//#define ONLY_PHONG
//#define ONLY_SAMPLE
//#define REVERSE_SHADOW

point_light point_lights[]{
	{{0, 0, 30}, {1.0f, 0.99f, 0.8f}, 1.0f, 0.09f, 0.032f}
};

material materials[]{
	{0.5f, 1.0f, 0.5f, 4.0f, init_color(ff, ff, ff) },	//white
	{0.5f, 1.0f, 0.5f, 4.0f, init_color(cc, 22, 08) },	//red
	{0.5f, 1.0f, 0.5f, 4.0f, init_color(ff, b7, 00) },	//yellow
	{0.5f, 1.0f, 0.5f, 4.0f, init_color(33, 80, 00) },	//green
	{0.5f, 1.0f, 0.5f, 4.0f, init_color(80, 3c, 00) },	//brown
	{0.5f, 1.0f, 0.5f, 4.0f, init_color(00, 36, cc) },	//blue
	{0.5f, 1.0f, 0.5f, 4.0f, init_color(33, 33, 33) },	//black
	{0.15f, 1.0f, 0.05f, 32.0f, {0.0f, 0.4f, 0.0f} }, //no reflection green
};

#define set_params_random arr_random, current_random_index
#define def_params_random const concurrency::array_view<float, 1>& arr_random, uint& current_random_index

#define set_params_crossables arr_triangles, arr_spheres
#define def_params_crossables const concurrency::array<triangle, 1>& arr_triangles, \
	const concurrency::array<sphere, 1>& arr_spheres

#define set_params_light_sources arr_point_lights
#define def_params_light_sources const concurrency::array<point_light, 1>& arr_point_lights

#define set_params_debug arr_debug
#define def_params_debug concurrency::array_view<float_3, 1>& arr_debug

inline float randf(def_params_random) restrict(amp) {
	//return xorshift(set_params_random) * 1.0f / 0xffffffff;
	current_random_index = (current_random_index + 1);
	return arr_random[current_random_index];
}

inline float_3 get_ambient_color() restrict(amp) {
	return float_3(0.0f, 0.0f, 0.0f);
}

inline uint make_id_triangle(uint id) restrict(amp) {
	return id | 0x00000000;
}

inline uint make_id_sphere(uint id) restrict(amp) {
	return id | 0x00010000;
}

inline bool try_parse_id_triangle(uint& id) restrict(amp) {
	return ((id & 0xffff0000) == 0x00000000);
}

inline bool try_parse_id_sphere(uint& id) restrict(amp) {
	if (((id & 0xffff0000) == 0x00010000)) {
		id &= 0x0000ffff;
		return true;
	}
	else return false;
}

inline float_3 get_normal(uint id, const float_3& at, def_params_crossables) restrict(amp) {
	if (try_parse_id_triangle(id)) {
		return arr_triangles[id].n;
	}
	else if (try_parse_id_sphere(id)) {
		return get_normal(arr_spheres[id], at);
	}
}

inline uint get_material_id(uint id, def_params_crossables) restrict(amp) {
	if (try_parse_id_triangle(id)) {
		return arr_triangles[id].material_id;
	}
	else if (try_parse_id_sphere(id)) {
		return arr_spheres[id].material_id;
	}
}

inline bool check_cross(const float_3& original_point, const float_3& dir, float& alpha, uint ignore,
	uint& crossable_index, def_params_crossables) restrict(amp) {
	float min_alpha = 1e10f;
	uint cid;
	for (uint i = 0; i < arr_triangles.extent.size(); ++i) {
		cid = make_id_triangle(i);
		if (cid == ignore) continue;
		if (arr_triangles[i].check_cross(original_point, dir, alpha)
			&& alpha < min_alpha) {
			min_alpha = alpha;
			crossable_index = cid;
		}
	}

	for (uint i = 0; i < arr_spheres.extent.size(); ++i) {
		cid = make_id_sphere(i);
		if (cid == ignore) continue;
		if (arr_spheres[i].check_cross(original_point, dir, alpha, min_alpha) && alpha < min_alpha) {
			min_alpha = alpha;
			crossable_index = cid;
		}
	}

	alpha = min_alpha;
	return min_alpha != 1e10f;
}

inline float_3 random_direction(const float_3& normal, const float_3& n_1, 
	const float_3& n_2, def_params_random) restrict(amp) {
	float radius = randf(set_params_random) * 0.8f;
	float height = sqrt(1.0f - radius * radius);
	float select = randf(set_params_random) * 4.0f;
	float select_1, select_2;
	if (select < 1.0f) {
		select_1 = select;
		select_2 = sqrt(1.0f - select_1 * select_1);
	}
	else if (select < 2.0f) {
		select_1 = - (select - 1.0f);
		select_2 = sqrt(1.0f - select_1 * select_1);
	}
	else if (select < 3.0f) {
		select_1 = -(select - 2.0f);
		select_2 = -sqrt(1.0f - select_1 * select_1);
	}
	else {
		select_1 = select - 3.0f;
		select_2 = -sqrt(1.0f - select_1 * select_1);
	}
	float_3 r = select_1 * n_1 + select_2 * n_2;
	return normal * height + r * radius;
}

inline float_3 render_phong(uint current_crossable, const float_3& current_point,
	const float_3& view_dir,
	const float_3& current_normal,
	const material& current_material,
	const bool soft_shadow,
	def_params_light_sources,
	def_params_crossables,
	def_params_random) restrict(amp) {
	float_3 result(0.0f, 0.0f, 0.0f);
#ifdef REVERSE_SHADOW
	float_3 all_lights(0.0f, 0.0f, 0.0f);
#endif
	float ignore_float;
	uint ignore_uint;
	for (uint i = 0; i < arr_point_lights.extent[0]; ++i) {
		float_3 dir_to_ls = arr_point_lights[i].loc - current_point;
		float far_to_ls = length(dir_to_ls);
		dir_to_ls /= far_to_ls;
#ifdef REVERSE_SHADOW
		float_3 intensity = arr_point_lights[i].get_intensity(current_point, current_normal,
			dir_to_ls, current_material);
#endif
		if (dot(current_normal, dir_to_ls) < 0) continue;
		if (soft_shadow) {
			float_3 delta, dir_to_ls_new;
			uint pass_cnt = 0;
			for (uint j = 0; j < cons_soft_shadow_sample_cnt; ++j) {
				delta = { randf(set_params_random) * 2.0f - 1.0f,
					randf(set_params_random) * 2.0f - 1.0f,
					randf(set_params_random) * 2.0f - 1.0f };
				dir_to_ls_new = normalize(arr_point_lights[i].loc + delta - current_point);
				if (!check_cross(current_point, dir_to_ls_new, ignore_float,
					current_crossable, ignore_uint, set_params_crossables) || 
					ignore_float > far_to_ls) {
					++pass_cnt;
				}
				continue;
			}
#ifdef REVERSE_SHADOW
			result -= (1.0f - pass_cnt * 1.0f / cons_soft_shadow_sample_cnt) *
				arr_point_lights[i].get_intensity(current_point, current_normal,
					dir_to_ls, current_material);
#else
			result = pass_cnt * 1.0f / cons_soft_shadow_sample_cnt * 
				arr_point_lights[i].get_intensity(current_point, current_normal,
				dir_to_ls, current_material);
#endif
		}
		else {
			if (!check_cross(current_point, dir_to_ls, ignore_float, 
				current_crossable, ignore_uint,
				set_params_crossables) || ignore_float >= far_to_ls) {
				result = result + arr_point_lights[i].get_intensity(current_point, 
					current_normal, dir_to_ls, current_material);
			}
		}
	}
	return result;
}

inline float_3 sample_phong(const float_3& current_point, const float_3& normal, const float_3& view_dir, uint sample_cnt,
	uint current_crossable, const concurrency::array<material, 1>& arr_materials,
	def_params_crossables,
	def_params_light_sources,
	def_params_random) restrict(amp) {
#ifdef ONLY_PHONG
	return 0.0f;
#endif
	float_3 new_dir, res(0.0f, 0.0f, 0.0f);
	float alpha, cos_theta;
	uint crossable_id;

	float_3 n_1, n_2;
	if (normal.z != 0.0f) n_1 = float_3(1.0f, 1.0f, -(normal.x + normal.y) / normal.z);
	else if (normal.y != 0.0f) n_1 = float_3(1.0f, -(normal.x + normal.z) / normal.y, 1.0f);
	else n_1 = float_3(-(normal.y + normal.z) / normal.x, 1.0f, 1.0f);
	n_2 = normalize(cross(n_1, normal));
	n_1 = normalize(n_1);

	material this_material = arr_materials[get_material_id(current_crossable, set_params_crossables)];
	material current_material;

	for (uint i = 0; i < sample_cnt; ++i) {
		new_dir = random_direction(normal, n_1, n_2, set_params_random);
		float brdf = this_material.brdf(-new_dir, -view_dir , normal);
		if (brdf < 1e-3f) continue;
		//render phong
		if (check_cross(current_point, new_dir, alpha, current_crossable, crossable_id, set_params_crossables)) {
			float_3 cross_point = current_point + new_dir * alpha;
			float_3 cross_norm = get_normal(crossable_id, cross_point, set_params_crossables);
			if (dot(cross_norm, new_dir) > 0) cross_norm = -cross_norm;
			current_material = arr_materials[get_material_id(crossable_id, set_params_crossables)];
			res = res + render_phong(crossable_id, cross_point, new_dir, cross_norm,
				current_material, false, set_params_light_sources, set_params_crossables, set_params_random) 
				* this_material.color * brdf;
		}
	}

	return res / (float)sample_cnt;
}

inline float_3 sample_all(const float_3& original_point, const float_3& view_dir,
	float_3& sample_phong_res, float_3& render_phong_res,
	const concurrency::array<material, 1>& arr_materials,
	def_params_crossables, def_params_light_sources, def_params_random) restrict(amp) {
	float alpha;
	uint crossable_index = 0;
	if (!check_cross(original_point, view_dir, alpha, -1, crossable_index, set_params_crossables))
		return sample_phong_res = render_phong_res = float_3(0.0f, 0.0f, 0.0f);
	float_3 cross_point = original_point + view_dir * alpha;
	float_3 normal = get_normal(crossable_index, cross_point, set_params_crossables);
	if (dot(view_dir, normal) > 0) normal = -normal;
	material material = arr_materials[get_material_id(crossable_index, set_params_crossables)];
	//set small offset and render phong
#ifndef ONLY_SAMPLE
	render_phong_res = render_phong(crossable_index, cross_point, view_dir, 
		normal, material, true, set_params_light_sources, set_params_crossables, set_params_random);
#else
	render_phong_res = 0.0f;
#endif
	if (crossable_index & 0xffff0000) {
		sample_phong_res = sample_phong(cross_point, normal, view_dir,
			cons_sample_phong_cnt * cons_pixel_sample_cnt,
			crossable_index, arr_materials, set_params_crossables,
			set_params_light_sources, set_params_random);
	}
	else sample_phong_res = 0.0f;
	return sample_phong_res + render_phong_res;
}

ray_tracing_controller::ray_tracing_controller(){
	srand(time(0));
	for (int i = 0; i < VIEW_ROW; ++i)
		for (int j = 0; j < VIEW_COL; ++j)
			self_arr_random_init[index<2>(i,j)] 
				= rand() % RANDOM_TABLE_SIZE;
	for (int i = 0; i < RANDOM_TABLE_SIZE; ++i)
		self_arr_random[i] = rand() * 1.0f / RAND_MAX;

	float width_d_height = VIEW_COL * 1.0f / VIEW_ROW;
	float real_width = 2.0f * MIN_VIEW * tan(FOV / 360.0f * cons_pi) * width_d_height;
	view_resolusion = real_width / VIEW_COL;
}

void ray_tracing_controller::render() {
	update_messages();
	const float_3 hori = normalize(cross(camera_up, look_at)) * -1.0;
	const float_3 vert = normalize(cross(look_at, hori));
	const float_3 origin = eye + normalize(look_at) * MIN_VIEW
		- hori * (VIEW_COL / 2 * view_resolusion)
		- vert * (VIEW_ROW / 2 * view_resolusion);
	const float this_view_resolusion = view_resolusion;
	const float_3 this_eye = eye;

	concurrency::array<sphere, 1> arr_spheres(len(spheres), spheres);
	concurrency::array<triangle, 1> arr_triangles(len(triangles), triangles);
	concurrency::array_view<float_3, 2> arr_sample_results((int)VIEW_ROW, (int)VIEW_COL);
	concurrency::array_view<float_3, 2> arr_render_results((int)VIEW_ROW, (int)VIEW_COL);
	concurrency::array<material, 1> arr_materials(len(materials), materials);
	concurrency::array<point_light, 1> arr_point_lights(len(point_lights), point_lights);
	concurrency::array_view<uint, 2> arr_random_init(self_arr_random_init);
	concurrency::array_view<float, 1> arr_random(self_arr_random);

	parallel_for_each(arr_sample_results.extent,
		[=, &arr_materials, &arr_point_lights,
		&arr_triangles, &arr_spheres](index<2> idx) restrict(amp) {
			uint r = idx[0], c = idx[1];
			float_3 o = origin + hori * c * this_view_resolusion +
				vert * r * this_view_resolusion;
			float_3 delta = normalize(o - this_eye);
			uint& current_random_index = arr_random_init[idx];
			sample_all(o, delta, arr_sample_results[idx],
				arr_render_results[idx], arr_materials,
				set_params_crossables, set_params_light_sources, set_params_random);
		});
	
	for(uint i = 0; i < VIEW_ROW; ++i)
		for (uint j = 0; j < VIEW_COL; ++j) {
			index<2> idx(i, j);
			float_3 res = arr_render_results[idx] + 
				arr_sample_results[idx];
			results[i][j][0] = res.r;
			results[i][j][1] = res.g;
			results[i][j][2] = res.b;
		}
	
	set_results();
}

float_3 seteye = float_3(0.0f, 0.0f, 5.0f);
float_3 setlook_at = float_3(1.0f, 0.0f, 0.0f);
float_3 setcamera_up = float_3(0.0f, 0.0f, 1.0f);

void ray_tracing_controller::update_messages() {
	eye = seteye;
	look_at = setlook_at;
	camera_up = setcamera_up;

	triangles[0] = init_triangle(125, 65, -3, 125, -65, -3, -125, -65, -3, 7);
	triangles[1] = init_triangle(-125, -65, -3, -125, 65, -3, 125, 65, -3, 7);

	spheres[0] = init_sphere(0, 0, 0, 3, 0);
	spheres[1] = init_sphere(12, 0, 0, 3, 1);
	spheres[2] = init_sphere(24, 0, 0, 3, 2);
	spheres[3] = init_sphere(36, 0, 0, 3, 3);
	spheres[4] = init_sphere(48, 0, 0, 3, 4);
	spheres[5] = init_sphere(60, 0, 0, 3, 5);
	spheres[6] = init_sphere(72, 0, 0, 3, 6);
	spheres[7] = init_sphere(84, 0, 0, 3, 1);
	spheres[8] = init_sphere(96, 0, 0, 3, 1);
	spheres[9] = init_sphere(108, 0, 0, 3, 1);
}

cv::Mat view;
void ray_tracing_controller::set_results() {
	uchar* view_ptr = view.ptr();
	for (uint i = 0; i < VIEW_ROW; ++i)
		for (uint j = 0; j < VIEW_COL; ++j) {
			*(view_ptr++) = (int)(min(results[i][j][2], 1.0f) * 255.0f);
			*(view_ptr++) = (int)(min(results[i][j][1], 1.0f) * 255.0f);
			*(view_ptr++) = (int)(min(results[i][j][0], 1.0f) * 255.0f);
		}
}

ray_tracing_controller rtc;

void onMouse(int event, int x, int y, int flags, void* ustc) {

}

int main() {
	srand(time(0));
	cv::namedWindow("wxnb", cv::WINDOW_AUTOSIZE);
	cv::setMouseCallback("wxnb", onMouse, 0);
	view = cv::Mat(VIEW_ROW, VIEW_COL, CV_8UC3);

	SYSTEMTIME time;

	for (uint t = 0;; ++t) {
		GetLocalTime(&time);
		rtc.render();
		cv::imshow("wxnb", view);
		SYSTEMTIME nowt;
		GetLocalTime(&nowt);
		double fps = 1000.0 / (nowt.wMilliseconds - time.wMilliseconds);
		if (fps > 0) cerr << "\rFPS: " << fps;
		int k = cv::waitKey(10);
		switch (k)
		{
		case 'w':
			seteye += 0.2f * setlook_at;
			break;
		case 's':
			seteye -= 0.2f * setlook_at;
			break;
		case 'a':
			seteye -= 0.2f * cross(setlook_at, setcamera_up);
			break;
		case 'd':
			seteye += 0.2f * cross(setlook_at, setcamera_up);
			break;
		case 'j':
			setlook_at = normalize(setlook_at - 0.01f * cross(setlook_at, setcamera_up));
			break;
		case 'l':
			setlook_at = normalize(setlook_at + 0.01f * cross(setlook_at, setcamera_up));
			break;
		case 'i':
			setlook_at = normalize(setlook_at - 0.01f * cross(setlook_at, cross(setlook_at, setcamera_up)));
			break;
		case 'k':
			setlook_at = normalize(setlook_at + 0.01f * cross(setlook_at, cross(setlook_at, setcamera_up)));
			break;
		default:
			break;
		}
	}

		

	return 0;
}
