#include <iostream>
#include <cstring>
#include <opencv2/opencv.hpp>
#include <ctime>
#include <random>
#include "wxrt_tools_amp.h"
#include "crossable_amp.h"
#include <amprt.h>
#include <ctime>
#include <mutex>
#include <amp.h>
#include <windows.h>
using namespace std;
using namespace wxrt;
using namespace concurrency;
using namespace concurrency::fast_math;
using namespace concurrency::graphics;
#define len(a) (sizeof(a) / sizeof(a[0]))

#define cons_inf (1e10f);
#define cons_pi (3.14159265f);
#define cons_2_pi (3.14159265f * 2.0f);
#define cons_pi_2 (3.14159265f / 2.0f);

#define cons_view_row 360u
#define cons_view_col 720u

#define cons_view_distance_to_eye 1.0f
#define cons_view_resolusion 0.0025f
#define cons_sample_phong_cnt 72
#define cons_pixel_sample_cnt 2
#define cons_soft_shadow_sample_cnt 128
#define cons_arr_random_size (10000000)

//#define ONLY_PHONG

#define init_triangle(ax, ay, az, bx, by, bz, cx, cy, cz, material_id)	\
	{{ax, ay, az}, {bx, by, bz}, {cx, cy, cz}, \
	normalize(cross(float_3(bx, by, bz) - float_3(ax, ay, az), \
	float_3(cx, cy, cz) - float_3(ax, ay, az))), material_id, \
	/*{max(max(ax, bx), cx), max(max(ay, by), cy), max(max(az, bz), cz)},	\
	{min(min(ax, bx), cx), min(min(ay, by), cy), min(min(az, bz), cz)}*/}

#define init_sphere(ox, oy, oz, r, material_id)	\
	{{ox, oy, oz}, r, material_id/*, {ox + r, oy + r, oz + r}, {ox - r, oy - r, oz - r}*/}

wxrt::triangle triangles[]{
	init_triangle(-10.0f, 10.0f, 0.0f,  -10.0f, -10.0f, 0.0f,  10.0f, -10.0f, 0.0f,  0),
	init_triangle(-10.0f, 10.0f, 0.0f,  10.0f, 10.0f, 0.0f,  10.0f, -10.0f, 0.0f,  0),
};

wxrt::sphere spheres[]{
	init_sphere(0.0, -0.3, 0.3,  0.3,  1),
	init_sphere(0.6, -0.3, 0.3,  0.3,  2),
	init_sphere(0.0, -0.9, 0.3,  0.3,  3),
	init_sphere(1.2, 0.0, 0.3,  0.3,  0),
	init_sphere(-0.8, 0.0, 0.3,  0.3,  0),
	init_sphere(0.0, 0.6, 0.3,  0.3,  4)
};

wxrt::point_light point_lights[]{
	{{1.0f, 3.0f, 2.0f}, {0.8f, 0.8f, 0.8f}, 1.0f, 0.09f, 0.032f}
};

wxrt::material materials[]{
	{{1.0f, 1.0f, 1.0f}, {0.6f, 0.6f, 0.6f}, 1.0f, 5.0f },	//white
	{{0.6f, 0.06f, 0.06f}, {0.6f, 0.06f, 0.06f}, 1.0f, 5.0f },	//red
	{{0.06f, 0.6f, 0.06f}, {0.06f, 0.6f, 0.06f}, 1.0f, 5.0f },	//green
	{{0.06f, 0.06f, 0.6f}, {0.06f, 0.06f, 0.6f}, 1.0f, 5.0f },	//blue
	{{0.06f, 0.8f, 0.8f}, {0.06f, 0.8f, 0.8f}, 1.0f, 5.0f },	//
};

#define set_params_random arr_random, current_random_index
#define def_params_random const concurrency::array<float, 1>& arr_random, uint& current_random_index

#define set_params_crossables arr_triangles, arr_spheres
#define def_params_crossables const concurrency::array<triangle, 1>& arr_triangles, \
	const concurrency::array<sphere, 1>& arr_spheres

#define set_params_light_sources arr_point_lights
#define def_params_light_sources const concurrency::array<point_light, 1>& arr_point_lights

#define set_params_debug arr_debug
#define def_params_debug concurrency::array_view<float_3, 1>& arr_debug

inline float randf(def_params_random) restrict(amp) {
	//return xorshift(set_params_random) * 1.0f / 0xffffffff;
	current_random_index = (current_random_index + 1) % cons_arr_random_size;
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
		if (check_cross(original_point, dir, alpha, arr_triangles[i])
			&& alpha < min_alpha) {
			min_alpha = alpha;
			crossable_index = cid;
		}
	}

	for (uint i = 0; i < arr_spheres.extent.size(); ++i) {
		cid = make_id_sphere(i);
		if (cid == ignore) continue;
		if (check_cross(original_point, dir, alpha, arr_spheres[i])
			&& alpha < min_alpha) {
			min_alpha = alpha;
			crossable_index = cid;
		}
	}

	alpha = min_alpha;
	return min_alpha != 1e10f;
}

inline float_3 random_direction(const float_3& normal, const float_3& n_1, 
	const float_3& n_2, def_params_random) restrict(amp) {
	float radius = randf(set_params_random);
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
	float ignore_float;
	uint ignore_uint;
	for (uint i = 0; i < arr_point_lights.extent[0]; ++i) {
		float_3 dir_to_ls = arr_point_lights[i].loc - current_point;
		float far_to_ls = length(dir_to_ls);
		dir_to_ls /= far_to_ls;
		if (dot(current_normal, dir_to_ls) < 0) continue;
		if (soft_shadow) {
			float_3 intensity(-1.0f, 0.0f, 0.0f);
			float_3 delta, dir_to_ls_new;
			uint id_triangle = 0xffffffff, id_sphere = 0xffffffff;
			uint pass_cnt = 0;
			for (uint j = 0; j < cons_soft_shadow_sample_cnt; ++j) {
				delta = { randf(set_params_random) * 2.0f - 1.0f,
					randf(set_params_random) * 2.0f - 1.0f,
					randf(set_params_random) * 2.0f - 1.0f };
				delta *= 1e-1f;
				dir_to_ls_new = normalize(arr_point_lights[i].loc + delta - current_point);
				if (!check_cross(current_point, dir_to_ls_new, ignore_float,
					current_crossable, ignore_uint, set_params_crossables) || ignore_float > far_to_ls) {
					++pass_cnt;
				}
				continue;
			}

			result = pass_cnt * 1.0f / cons_soft_shadow_sample_cnt * 
				get_intensity(current_point, current_normal,
				dir_to_ls, current_material, arr_point_lights[i]);
		}
		else {
			if (!check_cross(current_point, dir_to_ls, ignore_float, 
				current_crossable, ignore_uint,
				set_params_crossables) || ignore_float >= far_to_ls) {
				result = result + get_intensity(current_point, current_normal, 
					dir_to_ls, current_material, arr_point_lights[i]);
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
		if (brdf < 1e-3) continue;
		//render phong
		if (check_cross(current_point, new_dir, alpha, current_crossable, crossable_id, set_params_crossables)) {
			float_3 cross_point = current_point + new_dir * alpha;
			float_3 cross_norm = get_normal(crossable_id, cross_point, set_params_crossables);
			if (dot(cross_norm, new_dir) > 0) cross_norm = -cross_norm;
			current_material = arr_materials[get_material_id(crossable_id, set_params_crossables)];
			res = res + render_phong(crossable_id, cross_point, new_dir, cross_norm,
				current_material, false, set_params_light_sources, set_params_crossables, set_params_random) 
				* this_material._ka * brdf;
		}
	}
	return res / (float)sample_cnt;
}

inline float_3 sample_all(const float_3& original_point, const float_3& view_dir,
	const concurrency::array<material, 1>& arr_materials,
	def_params_crossables, def_params_light_sources, def_params_random) restrict(amp) {
	float alpha;
	uint crossable_index = 0;
	if (!check_cross(original_point, view_dir, alpha, -1, crossable_index, set_params_crossables))
		return float_3(0.0f, 0.0f, 0.0f);
	float_3 cross_point = original_point + view_dir * alpha;
	float_3 normal = get_normal(crossable_index, cross_point, set_params_crossables);
	if (dot(view_dir, normal) > 0) normal = -normal;
	material material = arr_materials[get_material_id(crossable_index, set_params_crossables)];
	//set small offset and render phong
	float_3 render_phong_res = render_phong(crossable_index, cross_point, view_dir, 
		normal, material, true, set_params_light_sources, set_params_crossables, set_params_random);
	float_3 sample_phong_res = sample_phong(cross_point, normal, view_dir,
		cons_sample_phong_cnt * cons_pixel_sample_cnt,
		crossable_index, arr_materials, set_params_crossables,
		set_params_light_sources, set_params_random);
	return sample_phong_res + render_phong_res;
}

cv::Mat view;

uint random_init_table[cons_view_row][cons_view_col];
float random_floats[cons_arr_random_size];

void init_random_table() {
	srand(time(0));
	for (int i = 0; i < cons_arr_random_size; ++i) {
		random_floats[i] = rand() * 1.0f / RAND_MAX;
	}

	for (int i = 0; i < cons_view_row; ++i)
		for (int j = 0; j < cons_view_col; ++j)
			random_init_table[i][j] = rand() % cons_arr_random_size;
}

int main() {
	srand(time(0));
	cv::namedWindow("wxnb", cv::WINDOW_AUTOSIZE);
	view = cv::Mat(cons_view_row, cons_view_col, CV_8UC3);

	concurrency::array_view<float_3, 2> arr_view_results(cons_view_row, cons_view_col);
	concurrency::array<triangle, 1> arr_triangles(len(triangles), triangles);
	concurrency::array<material, 1> arr_materials(len(materials), materials);
	concurrency::array<point_light, 1> arr_point_lights(len(point_lights), point_lights);

	init_random_table();
	concurrency::array<float, 1> arr_random(cons_arr_random_size, random_floats);
	concurrency::array<uint, 2> arr_random_init(cons_view_row, cons_view_col, (uint*)random_init_table);

	SYSTEMTIME time;

	for (uint t = 0;; ++t) {
		GetLocalTime(&time);
		float_3 eye(cos(t / 64.0) * 3.0f, sin(t / 64.0) * 3.0f, 0.5f);
		float_3 look_at = -eye;
		look_at.z = 0.0f;
		float_3 camera_up(0.0f, 0.0f, 1.0f);

		spheres[0].o.x = 1.5f * cos(t / 32.0);
		spheres[0].o.y = 1.5f * sin(t / 32.0);
		spheres[1].o.x = 0.3f * sin(t / 64.0);
		spheres[1].o.y = 0.3f * cos(t / 64.0);
		concurrency::array<sphere, 1> arr_spheres(len(spheres), spheres);
		uint rdbegin = rand() % cons_arr_random_size;

		parallel_for_each(arr_view_results.extent,
			[=, &arr_materials, &arr_point_lights, 
				&arr_triangles, &arr_spheres, &arr_random,
				&arr_random_init](index<2> idx) restrict(amp) {
				uint r = idx[0], c = idx[1];

				const float_3 hori = normalize(cross(camera_up, look_at)) * -1.0;
				const float_3 vert = normalize(camera_up) * -1.0;
				const float_3 origin = eye + normalize(look_at) * cons_view_distance_to_eye
					- hori * (cons_view_col / 2 * cons_view_resolusion)
					- vert * (cons_view_row / 2 * cons_view_resolusion);

				float_3 o = origin + hori * c * cons_view_resolusion +
					vert * r * cons_view_resolusion;
				float_3 delta = normalize(o - eye);

				uint& current_random_index = arr_random_init[idx];
				arr_view_results[idx] = sample_all(o, delta, arr_materials,
					set_params_crossables, set_params_light_sources, set_params_random);
			});

		for (int i = 0; i < cons_view_row; ++i)
			for (int j = 0; j < cons_view_col; ++j){
				cv::Vec3b& v = view.at<cv::Vec3b>(i, j);
				float_3 res = arr_view_results[index<2>(i, j)];
				v[0] = min(res.b, 1.0f) * 255;
				v[1] = min(res.g, 1.0f) * 255;
				v[2] = min(res.r, 1.0f) * 255;
			}
		cv::imshow("wxnb", view);
		SYSTEMTIME nowt;
		GetLocalTime(&nowt);
		double fps = 1000.0 / (nowt.wMilliseconds - time.wMilliseconds);
		if (fps > 0) cerr << "\rFPS: " << fps;
		cv::waitKey(1);
	}

		

	return 0;
}
