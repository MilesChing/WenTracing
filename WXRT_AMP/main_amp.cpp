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

#define cons_view_row 600u
#define cons_view_col 800u

#define cons_eye float_3(3.0f, 2.0f, 0.5f)
#define cons_look_at float_3(-3.0f, -2.0f, 0.0f)
#define cons_camera_up float_3(0.0f, 0.0f, 1.0f)

#define cons_view_distance_to_eye 1.0f
#define cons_view_resolusion 0.0010f
#define cons_sample_phong_cnt 8
#define cons_pixel_sample_cnt 4

uint xorshift(uint& x, uint& y, uint& z) restrict(amp) {
	uint t;
	x ^= x << 16;
	x ^= x >> 5;
	x ^= x << 1;
	t = x;
	x = y;
	y = z;
	return z = t ^ x ^ y;
}

uint xorshift(uint& x, uint& y, uint& z) {
	uint t;
	x ^= x << 16;
	x ^= x >> 5;
	x ^= x << 1;
	t = x;
	x = y;
	y = z;
	return z = t ^ x ^ y;
}

wxrt::triangle triangles[]{
	{{1.0f, 0.0f, 0.0f}, {0.0f, 0.0f, 1.0f}, {0.0f, 0.0f, 0.0f}, {0.0f, 1.0f, 0.0f}, 0},
	{{1.0f, 0.0f, 0.0f}, {0.0f, 0.0f, 1.0f}, {1.0f, 0.0f, 1.0f}, {0.0f, 1.0f, 0.0f}, 0},
	{{0.0f, 1.0f, 0.0f}, {0.0f, 0.0f, 1.0f}, {0.0f, 0.0f, 0.0f}, {1.0f, 0.0f, 0.0f}, 1},
	{{0.0f, 1.0f, 0.5f}, {0.5f, 1.0f, 0.0f}, {0.0f, 1.0f, 0.0f}, {0.0f, 1.0f, 0.0f}, 0},
	{{0.0f, 0.0f, 1.0f}, {0.0f, 1.0f, 0.0f}, {0.0f, 1.0f, 1.0f}, {1.0f, 0.0f, 0.0f}, 2},
	{{0.0f, 0.0f, 1.0f}, {1.0f, 0.0f, 1.0f}, {0.0f, 1.0f, 1.0f}, {0.0f, 0.0f, -1.0f}, 0},
	{{2.0f, 2.0f, 0.0f}, {2.0f, -2.0f, 0.0f}, {-2.0f, -2.0f, 0.0f}, {0.0f, 0.0f, 1.0f}, 2},
	{{2.0f, 2.0f, 0.0f}, {-2.0f, 2.0f, 0.0f}, {-2.0f, -2.0f, 0.0f}, {0.0f, 0.0f, 1.0f}, 2},
};

wxrt::point_light point_lights[]{
	{{1.0f, 3.0f, 2.0f}, {0.8f, 0.8f, 0.8f}, 1.0f, 0.09f, 0.032f},
	//{{1.0f, 2.0f, 1.0f}, {0.3f, 0.3f, 0.3f}, 1.0f, 0.4f, 0.032f}
};

wxrt::material materials[]{
	{{1.0f, 1.0f, 1.0f}, {0.6f, 0.6f, 0.6f}, {0.4f, 0.4f, 0.4f}, 32.0f },	//white triangle
	{{1.0f, 0.1f, 0.1f}, {0.6f, 0.06f, 0.06f}, {0.4f, 0.04f, 0.04f}, 32.0f },	//red triangle
	{{0.1f, 0.4f, 1.0f}, {0.06f, 0.6f, 0.06f}, {0.04f, 0.4f, 0.04f}, 32.0f },	//blue triangle
};

#define set_params_random x, y, z
#define def_params_random uint&x,uint&y,uint&z

#define set_params_crossables arr_triangles
#define def_params_crossables const concurrency::array<triangle, 1>& arr_triangles

#define set_params_light_sources arr_point_lights
#define def_params_light_sources const concurrency::array<point_light, 1>& arr_point_lights

#define set_params_debug arr_debug
#define def_params_debug concurrency::array_view<float_3, 1>& arr_debug

inline float randf(def_params_random) restrict(amp) {
	return xorshift(set_params_random) * 1.0f / 0xffffffff;
}

inline float_3 get_ambient_color() restrict(amp) {
	return float_3(0.0f, 0.0f, 0.0f);
}

inline uint make_id_triangle(uint id) restrict(amp) {
	return id | 0x00000000;
}

inline bool try_parse_id_triangle(uint& id) restrict(amp) {
	return ((id & 0xffff0000) == 0x00000000);
}

inline float_3 get_normal(uint id, 
	const concurrency::array<triangle, 1>& arr_triangles) restrict(amp) {
	if (try_parse_id_triangle(id)) {
		return arr_triangles[id].n;
	}
}

inline uint get_material_id(uint id,
	const concurrency::array<triangle, 1>& arr_triangles) restrict(amp) {
	if (try_parse_id_triangle(id)) {
		return arr_triangles[id].material_id;
	}
}

inline bool check_cross(const float_3& original_point, const float_3& dir, float& alpha, uint ignore,
	uint& crossable_index, def_params_crossables) restrict(amp) {
	float min_alpha = 1e10f;
	for (uint i = 0; i < arr_triangles.extent.size(); ++i) {
		if (i == ignore) continue;
		if (check_cross(original_point, dir, alpha, arr_triangles[i])
			&& alpha < min_alpha) {
			min_alpha = alpha;
			crossable_index = i;
		}
	}
	alpha = min_alpha;
	crossable_index = make_id_triangle(crossable_index);
	return min_alpha != 1e10f;
}

inline float_3 render_phong(uint current_crossable, const float_3& current_point, 
	const float_3& view_dir,
	const float_3& current_normal,
	const material& current_material,
	def_params_light_sources,
	def_params_crossables) restrict(amp) {
	float_3 result(0.0f, 0.0f, 0.0f);
	bool crossed = false;
	float ignore_float;
	uint ignore_uint;
	for (uint i = 0; i < arr_point_lights.extent[0]; ++i) {
		float_3 dir_to_ls = normalize(arr_point_lights[i].loc - current_point);
		if (dot(current_normal, dir_to_ls) < 0) continue;
		if (!check_cross(current_point, dir_to_ls, ignore_float, 
			current_crossable, ignore_uint,
			set_params_crossables)) {
			result = result + get_intensity(current_point, current_normal, 
				dir_to_ls, current_material, arr_point_lights[i]);
			crossed = true;
		}
	}
	if (!crossed) result = get_ambient_color() * current_material._ka;
	return result;
}

inline float_3 sample_phong(const float_3& current_point, const float_3& normal, uint sample_cnt,
	uint current_crossable, const concurrency::array<material, 1>& arr_materials,
	def_params_crossables,
	def_params_light_sources,
	def_params_random) restrict(amp) {
	float theta_zy = atan2(normal.y, normal.z);
	float theta_yx = atan2(normal.x, normal.y);
	if (normal.x == normal.y && normal.x == 0.0f) theta_yx = 0.0f;
	if (normal.z == normal.y && normal.z == 0.0f) theta_zy = 0.0f;
	float cos_theta_zy = cos(theta_zy);
	float sin_theta_zy = sin(theta_zy);
	float cos_theta_yx = cos(theta_yx);
	float sin_theta_yx = sin(theta_yx);
	float_3 new_dir, tmp1, tmp2, res(0.0f, 0.0f, 0.0f);
	float alpha;
	uint crossable_id;

	for (uint i = 0; i < sample_cnt; ++i) {
		float theta = randf(set_params_random) * cons_pi_2;
		float phi = randf(set_params_random) * cons_2_pi;
		float sin_theta = sin(theta);
		float cos_theta = cos(theta);
		new_dir.x = sin_theta * cos(phi);
		new_dir.y = sin_theta * sin(phi);
		new_dir.z = cos_theta;
		//rotate direction to normal
		tmp1.x = new_dir.x;
		tmp1.z = cos_theta_zy * new_dir.z - sin_theta_zy * new_dir.y;
		tmp1.y = sin_theta_zy * new_dir.z + cos_theta_zy * new_dir.y;
		tmp2.y = cos_theta_yx * tmp1.y - sin_theta_yx * tmp1.x;
		tmp2.x = sin_theta_yx * tmp1.y + cos_theta_yx * tmp1.x;
		new_dir.x = tmp2.x;
		new_dir.y = tmp2.y;
		new_dir.z = tmp1.z;
		//render phong
		const double ambert_a_0 = 1.0f, ambert_a_1 = 0.9f, ambert_a_2 = 0.032f;
		if (check_cross(current_point, new_dir, alpha, current_crossable, crossable_id, set_params_crossables)) {
			float_3 cross_point = current_point + new_dir * alpha;
			float_3 cross_norm = get_normal(crossable_id, set_params_crossables);
			if (dot(cross_norm, new_dir) > 0) cross_norm = -cross_norm;
			material current_material = arr_materials[get_material_id(crossable_id, set_params_crossables)];
			res = res + render_phong(crossable_id, cross_point, new_dir, cross_norm, 
				current_material, set_params_light_sources, set_params_crossables) * cos_theta / 
				(ambert_a_0 + alpha * ambert_a_1 + alpha * alpha * ambert_a_2);
		}
	}

	return res / (float)sample_cnt;
}

inline float_3 multi_sample_1(const float_3& current_point, const float_3& normal, uint sample_cnt,
	uint current_crossable, const concurrency::array<material, 1>& arr_materials,
	def_params_crossables,
	def_params_light_sources,
	def_params_random) restrict(amp) {

}

inline float_3 sample_all(const float_3& original_point, const float_3& view_dir, 
	const concurrency::array<material, 1>& arr_materials,
	def_params_crossables, def_params_light_sources, def_params_random) restrict(amp) {
	float alpha;
	uint crossable_index = 0;
	if (!check_cross(original_point, view_dir, alpha, -1, crossable_index, arr_triangles))
		return float_3(0.0f, 0.0f, 0.0f);
	float_3 cross_point = original_point + view_dir * alpha;
	float_3 normal = get_normal(crossable_index, arr_triangles);
	if (dot(view_dir, normal) > 0) normal = -normal;
	material material = arr_materials[get_material_id(crossable_index, arr_triangles)];
	//set small offset and render phong
	float_3 render_phong_res = render_phong(crossable_index, cross_point, view_dir, 
		normal, material, arr_point_lights, arr_triangles);
	return sample_phong(cross_point, normal, cons_sample_phong_cnt, crossable_index, arr_materials,
		set_params_crossables, set_params_light_sources, set_params_random) + render_phong_res;
}

cv::Mat view;
uint random_table[cons_view_row * cons_view_col * cons_pixel_sample_cnt];

void init_random_table() {
	uint x = rand(), y = rand(), z = rand();
	for (int i = 0; i < cons_view_row; ++i)
		for (int j = 0; j < cons_view_col; ++j)
			for (int k = 0; k < cons_pixel_sample_cnt; ++k)
				random_table[(i * cons_view_col + j) * cons_pixel_sample_cnt + k]
					= xorshift(x, y, z);
}

int main() {
	srand(time(0));
	cv::namedWindow("wxnb", cv::WINDOW_AUTOSIZE);
	view = cv::Mat(cons_view_row, cons_view_col, CV_8UC3);

	concurrency::array_view<float_3, 3> arr_view_results(cons_view_row, cons_view_col, cons_pixel_sample_cnt);
	concurrency::array<triangle, 1> arr_triangles(len(triangles), triangles);
	concurrency::array<material, 1> arr_materials(len(materials), materials);

	init_random_table();

	SYSTEMTIME time;

	for (uint t = 0;; ++t) {
		GetLocalTime(&time);
		point_lights[0].loc = float_3(cos(t / 100.0) * 2.0f, sin(t / 100.0) * 2.0f, 2.0f);

		concurrency::array<point_light, 1> arr_point_lights(len(point_lights), point_lights);
		concurrency::array<uint, 3> arr_random(cons_view_row, cons_view_col, 
			cons_pixel_sample_cnt, random_table);

		parallel_for_each(arr_view_results.extent,
			[=, &arr_materials, &arr_point_lights, 
				&arr_triangles, &arr_random](index<3> idx) restrict(amp) {
				uint r = idx[0], c = idx[1];

				const float_3 hori = normalize(cross(cons_camera_up, cons_look_at)) * -1.0;
				const float_3 vert = normalize(cons_camera_up) * -1.0;
				const float_3 origin = cons_eye + normalize(cons_look_at) * cons_view_distance_to_eye
					- hori * (cons_view_col / 2 * cons_view_resolusion)
					- vert * (cons_view_row / 2 * cons_view_resolusion);

				float_3 o = origin + hori * c * cons_view_resolusion +
					vert * r * cons_view_resolusion;
				float_3 delta = normalize(o - cons_eye);

				uint x = arr_random[idx + 1], y = arr_random[idx], z = arr_random[idx - 1];

				arr_view_results[idx] = sample_all(o, delta, arr_materials,
					set_params_crossables, set_params_light_sources, set_params_random);
			});

		init_random_table();

		for (int i = 0; i < cons_view_row; ++i)
			for (int j = 0; j < cons_view_col; ++j){
				cv::Vec3b& v = view.at<cv::Vec3b>(i, j);
				float_3 res(0.0f, 0.0f, 0.0f);
				for(int k = 0; k < cons_pixel_sample_cnt; ++k)
					res += arr_view_results[index<3>(i, j, k)];
				res /= cons_pixel_sample_cnt;
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
