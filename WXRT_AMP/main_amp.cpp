#include <iostream>
#include <cstring>
#include <opencv2/opencv.hpp>
#include <ctime>
#include <random>
#include "wxrt_tools_amp.h"
#include "crossable_amp.h"
#include <ctime>
#include <mutex>
#include <amp.h>
using namespace std;
using namespace wxrt;
using namespace concurrency;
using namespace concurrency::fast_math;
using namespace concurrency::graphics;

#define len(a) (sizeof(a) / sizeof(a[0]))

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
	{{1.0f, 3.0f, 2.0f}, {1.0f, 1.0f, 1.0f}, 1.0f, 0.09f, 0.032f},
	{{1.0f, 2.0f, 1.0f}, {0.3f, 0.3f, 0.3f}, 1.0f, 0.4f, 0.032f}
};

wxrt::material materials[]{
	{{1.0f, 1.0f, 1.0f}, {0.6f, 0.6f, 0.6f}, {0.4f, 0.4f, 0.4f}, 32.0f },	//white triangle
	{{1.0f, 0.1f, 0.1f}, {0.6f, 0.06f, 0.06f}, {0.4f, 0.04f, 0.04f}, 32.0f },	//red triangle
	{{0.1f, 0.4f, 1.0f}, {0.06f, 0.6f, 0.06f}, {0.04f, 0.4f, 0.04f}, 32.0f },	//blue triangle
};

inline float_3 get_ambient_color() restrict(amp) {
	return float_3(0.1f, 0.1f, 0.1f);
}

inline uint make_id_triangle(uint id) restrict(amp) {
	return id | 0x00000000;
};

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

inline float randf() {
	return rand() * 1.0f / RAND_MAX;
}

inline bool check_cross(const float_3& original_point, const float_3& dir, float& alpha,
	uint& crossable_index, const concurrency::array<triangle, 1>& arr_triangles) restrict(amp) {
	float min_alpha = 1e10f;
	for (uint i = 0; i < arr_triangles.extent.size(); ++i) {
		if (check_cross(original_point, dir, alpha, arr_triangles[i])
			&& alpha < min_alpha) {
			min_alpha = alpha;
			crossable_index = i;
		}
	}
	alpha = min_alpha;
	return min_alpha != 1e10f;
}

inline float_3 render_phong(uint current_crossable, const float_3& current_point, 
	const float_3& view_dir,
	const float_3& current_normal,
	const material& current_material,
	const concurrency::array<point_light, 1>& arr_point_lights,
	const concurrency::array<triangle, 1>& arr_triangles) restrict(amp) {
	float_3 result(0.0f, 0.0f, 0.0f);
	bool crossed = false;
	float ignore_float;
	uint ignore_uint;
	for (uint i = 0; i < arr_point_lights.extent[0]; ++i) {
		float_3 dir_to_ls = normalize(arr_point_lights[i].loc - current_point);
		if (!check_cross(current_point + dir_to_ls * 0.0001f, dir_to_ls, ignore_float, ignore_uint,
			arr_triangles)) {
			result = result + get_intensity(current_point, current_normal, 
				dir_to_ls, current_material, arr_point_lights[i]);
			crossed = true;
		}
	}
	if (!crossed) result = get_ambient_color() * current_material._ka;
	return result;
}

/*
inline float_3 sample_phong(const float_3& current_point, const float_3& normal, uint sample_cnt,
	uint current_crossable) {
	float theta_zy = atan2(normal.y, normal.z);
	float theta_yx = atan2(normal.x, normal.y);
	float cos_theta_zy = cos(theta_zy);
	float sin_theta_zy = sin(theta_zy);
	float cos_theta_yx = cos(theta_yx);
	float sin_theta_yx = sin(theta_yx);
	float_3 new_dir, tmp1, tmp2, res(0.0f, 0.0f, 0.0f);
	float alpha;
	uint crossable_id;
	for (uint i = 0; i < sample_cnt; ++i) {
		float theta = randf() * halfpi;
		float phi = randf() * doubpi;
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
		if (check_cross(current_point, new_dir, alpha, crossable_id)) {
			float_3 cross_point = current_point + new_dir * alpha;
			float_3 cross_norm = crossables[crossable_id]->get_normal(cross_point);
			cross_point = cross_point + cross_norm * 0.0001f;
			res = res + render_phong(crossable_id, cross_point, new_dir) * cos_theta;
		}
	}

	return res / (float)sample_cnt;
}
*/

inline float_3 sample_all(const float_3& original_point, const float_3& view_dir, 
	const concurrency::array<triangle, 1>& arr_triangles,
	const concurrency::array<material, 1>& arr_materials,
	const concurrency::array<point_light, 1>& arr_point_lights) restrict(amp) {
	float alpha;
	uint crossable_index;
	if (!check_cross(original_point, view_dir, alpha, crossable_index, arr_triangles))
		return float_3(0.0f, 0.0f, 0.0f);
	float_3 cross_point = original_point + view_dir * alpha;
	float_3 normal = get_normal(crossable_index, arr_triangles);
	material material = arr_materials[get_material_id(crossable_index, arr_triangles)];
	//set small offset and render phong
	cross_point = cross_point + normal * 0.0001f;
	float_3 render_phong_res = render_phong(crossable_index, cross_point, view_dir, 
		normal, material, arr_point_lights, arr_triangles);
	return /*sample_phong(cross_point, normal, sample_phong_cnt, crossable_index) + */render_phong_res;
}


cv::Mat view;
std::mutex view_mtx;

int main() {
	const float inf = 1e10f;
	const float pi = 3.14159265f;
	const float doubpi = pi * 2.0f;
	const float halfpi = pi / 2.0f;

	const int max_ray_cnt = 32;
	const int max_depth = 4;
	const int view_row = 400;
	const int view_col = 600;

	const float_3 eye(3.0f, 2.0f, 0.5f);
	const float_3 look_at(-3.0f, -2.0f, 0.0f);
	const float_3 up(0.0f, 0.0f, 1.0f);

	const float view_distance_to_eye = 1.0f;
	const float view_resolusion = 0.002f;

	const uint sample_phong_cnt = 16;
	const int pixel_sample_cnt = 8;

	cv::namedWindow("wxnb", cv::WINDOW_AUTOSIZE);
	view = cv::Mat(view_row, view_col, CV_8UC3);

	concurrency::array_view<float_3, 2> arr_view_results(view_row, view_col);
	concurrency::array<triangle, 1> arr_triangles(len(triangles), triangles);
	concurrency::array<material, 1> arr_materials(len(materials), materials);

	for (uint t = 0;; ++t) {
		point_lights[0].loc = float_3(cos(t / 100.0) * 2.0f, sin(t / 100.0) * 2.0f, 2.0f);

		concurrency::array<point_light, 1> arr_point_lights(len(point_lights), point_lights);

		parallel_for_each(arr_view_results.extent,
			[=, &arr_materials, &arr_point_lights, &arr_triangles](index<2> idx) restrict(amp) {
				uint r = idx[0], c = idx[1];

				const float_3 hori = normalize(cross(up, look_at)) * -1.0;
				const float_3 vert = normalize(up) * -1.0;
				const float_3 origin = eye + normalize(look_at) * view_distance_to_eye
					- hori * (view_col / 2 * view_resolusion)
					- vert * (view_row / 2 * view_resolusion);

				float_3 o = origin + hori * c * view_resolusion +
					vert * r * view_resolusion;
				float_3 delta = normalize(o - eye);
				arr_view_results[idx] = sample_all(o, delta,
					arr_triangles,
					arr_materials,
					arr_point_lights);
			});

		for (int i = 0; i < view_row; ++i)
			for (int j = 0; j < view_col; ++j) {
				cv::Vec3b& v = view.at<cv::Vec3b>(i, j);
				float_3 res = arr_view_results.operator[](index<2>(i, j));
				v[0] = min(res.b, 1.0f) * 255;
				v[1] = min(res.g, 1.0f) * 255;
				v[2] = min(res.r, 1.0f) * 255;
			}

		cv::imshow("wxnb", view);
		cv::waitKey(1);
	}

		

	return 0;
}
