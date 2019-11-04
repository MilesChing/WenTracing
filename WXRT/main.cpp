#include <iostream>
#include <cstring>
#include <opencv2/opencv.hpp>
#include <ctime>
#include <random>
#include "wxrt_tools.h"
#include "crossable.h"
#include <mutex>
#include <ppl.h>
using namespace std;
using namespace wxrt;
using namespace concurrency;
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
	{{1.0f, 3.0f, 2.0f}, {1.0f, 1.0f, 1.0f}},
	{{1.0f, 2.0f, 1.0f}, {0.5f, 0.5f, 0.5f}}
};

wxrt::crossable* crossables[]{
	triangles + 0,
	triangles + 1,
	triangles + 2,
	triangles + 3,
	triangles + 4,
	triangles + 5,
	triangles + 6,
	triangles + 7
};

wxrt::light_source* light_sources[]{
	point_lights + 0,
	//point_lights + 1,
};

wxrt::material materials[]{
	{{1.0f, 1.0f, 1.0f}, {0.6f, 0.6f, 0.6f}, {0.4f, 0.4f, 0.4f}, 32.0f },	//white triangle
	{{1.0f, 0.1f, 0.1f}, {0.6f, 0.06f, 0.06f}, {0.4f, 0.04f, 0.04f}, 32.0f },	//red triangle
	{{0.1f, 0.4f, 1.0f}, {0.06f, 0.6f, 0.06f}, {0.04f, 0.4f, 0.04f}, 32.0f },	//green triangle
};

const uint triangle_len = len(triangles);

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

const float_3 ambient_color = float_3(0.05f, 0.05f, 0.05f);

const uint sample_phong_cnt = 16;
const int pixel_sample_cnt = 8;

inline float randf() {
	return rand() * 1.0f / RAND_MAX;
}

inline bool check_cross(const float_3& original_point, const float_3& dir, float& alpha, 
	uint& crossable_index) {
	float min_alpha = 1e10;
	for (uint i = 0; i < len(crossables); ++i) {
		if (crossables[i]->check_cross(original_point, dir, alpha) && alpha < min_alpha) {
			min_alpha = alpha;
			crossable_index = i;
		}
	}
	alpha = min_alpha;
	if (min_alpha == 1e10) return false;
	else return true;
}

float ignore_float;
uint ignore_uint;

inline float_3 render_phong(uint current_crossable, const float_3& current_point, const float_3& view_dir) {
	float_3 normal = crossables[current_crossable]->get_normal(current_point);
	float_3 result(0.0f, 0.0f, 0.0f);
	bool crossed = false;
	for (uint i = 0; i < len(light_sources); ++i) {
		float_3 dir_to_ls = normalize(light_sources[i]->get_location() - current_point);
		if (!check_cross(current_point + dir_to_ls * 0.0001f, dir_to_ls, ignore_float, ignore_uint)) {
			result = result + light_sources[i]->get_intensity(current_point, normal, dir_to_ls,
				materials[crossables[current_crossable]->get_material_id()]);
			crossed = true;
		}
	}
	if (!crossed) result = ambient_color * materials[crossables[current_crossable]->get_material_id()]._ka;
	return result;
}

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

inline float_3 sample_all(const float_3& original_point, const float_3& view_dir) {
	float alpha;
	uint crossable_index;
	if (!check_cross(original_point, view_dir, alpha, crossable_index))
		return float_3(0.0, 0.0, 0.0);
	float_3 cross_point = original_point + view_dir * alpha;
	float_3 normal = crossables[crossable_index]->get_normal(cross_point);
	//set small offset and render phong
	cross_point = cross_point + normal * 0.0001f;
	float_3 render_phong_res = render_phong(crossable_index, cross_point, view_dir);
	return render_phong_res;
	return sample_phong(cross_point, normal, sample_phong_cnt, crossable_index) + render_phong_res;
}

float_3 cal(uint shape_id, const float_3& cross_point,
	const float_3& in_dir, int depth, int rcnt) {

	if (depth >= max_depth) return ambient_color;

	int ray_cnt;
	if (depth == 0) ray_cnt = 1;
	else ray_cnt = rcnt;

	if (ray_cnt <= 0) return ambient_color;

	float_3 n;
	float theta_zy;
	float theta_yx;
	float cos_theta_zy;
	float sin_theta_zy;
	float cos_theta_yx;
	float sin_theta_yx;

	//do cross checking when depth == 0
	if (depth != 0) {
		const material& mate = materials[crossables[shape_id]->get_material_id()];
		//load config
		n = crossables[shape_id]->get_normal(cross_point);
		theta_zy = atan2(n.y, n.z);
		theta_yx = atan2(n.x, n.y);
		cos_theta_zy = cos(theta_zy);
		sin_theta_zy = sin(theta_zy);
		cos_theta_yx = cos(theta_yx);
		sin_theta_yx = sin(theta_yx);
	}

	struct task_result {
		bool is_cross;
		float_3 cross_point;
		float_3 in_dir;
		uint shape_id;
		uint need_cnt;
	}task_results[max_ray_cnt];

	parallel_for(0, ray_cnt, [&](int task_id) {
		//generate direction
		float_3 new_direct;
		if (depth != 0) {
			float cos_theta = randf();
			float theta = acos(cos_theta);
			float phi = randf() * doubpi;
			float sin_theta = sin(theta);
			new_direct.x = sin_theta * cos(phi);
			new_direct.y = sin_theta * sin(phi);
			new_direct.z = cos_theta;
			float_3 tmp1, tmp2;
			//rotate direction to normal
			tmp1.x = new_direct.x;
			tmp1.z = cos_theta_zy * new_direct.z - sin_theta_zy * new_direct.y;
			tmp1.y = sin_theta_zy * new_direct.z + cos_theta_zy * new_direct.y;
			tmp2.y = cos_theta_yx * tmp1.y - sin_theta_yx * tmp1.x;
			tmp2.x = sin_theta_yx * tmp1.y + cos_theta_yx * tmp1.x;
			new_direct.x = tmp2.x;
			new_direct.y = tmp2.y;
			new_direct.z = tmp1.z;

			task_results[task_id].need_cnt = (uint)ceil(cos_theta * ray_cnt);
		}
		else new_direct = in_dir;
		//--------------check cross
		float min_alpha = 1e10;
		uint min_shape_id;
		float_3 start_point = cross_point + new_direct * 0.01;
		for (int i = 0; i < len(crossables); ++i) {
			float alpha;
			if (crossables[i]->check_cross(start_point, new_direct, alpha) && alpha < min_alpha) {
				min_alpha = alpha;
				min_shape_id = i;
			}
		}
		//calculate res
		if (task_results[task_id].is_cross = (min_alpha != 1e10)) {
			task_results[task_id].cross_point = cross_point + new_direct * min_alpha;
			task_results[task_id].in_dir = new_direct;
			task_results[task_id].shape_id = min_shape_id;

			if (depth == 0) task_results[task_id].need_cnt = rcnt;
		}
	});

	float_3 res_sub_cal, res(0.0f, 0.0f, 0.0f);
	for (int i = 0; i < ray_cnt; ++i) {
		if (!task_results[i].is_cross) continue;
		res_sub_cal = cal(task_results[i].shape_id, task_results[i].cross_point,
			task_results[i].in_dir, depth + 1, task_results[i].need_cnt);
		res = res + res_sub_cal;
	}
	return res / ray_cnt;
}

cv::Mat view;
std::mutex view_mtx;

int main() {
	cv::namedWindow("wxnb", cv::WINDOW_AUTOSIZE);
	view = cv::Mat(view_row, view_col, CV_8UC3);
	const float_3 hori = normalize(cross(up, look_at)) * -1.0;
	const float_3 vert = normalize(up) * -1.0;
	const float_3 origin = eye + normalize(look_at) * view_distance_to_eye
		- hori * (view_col / 2 * view_resolusion)
		- vert * (view_row / 2 * view_resolusion);

	for(uint t = 0;; ++t){
		point_lights[0] = point_light(float_3(cos(t / 100.0) * 2.0f, sin(t / 100.0) * 2.0f, 2.0f));
		for (int i = 0; i < view_row; ++i) {
			parallel_for(0, view_col, 1, [&](uint j) {
				float_3 o = origin + hori * j * view_resolusion +
					vert * i * view_resolusion;
				float_3 delta = normalize(o - eye);
				cv::Vec3b& v = view.at<cv::Vec3b>(i, j);
				float_3 res(0.0f, 0.0f, 0.0f);
				res = res + sample_all(o, delta);
				v[0] = (int)(min(res.z, 1.0f) * 255.0f);
				v[1] = (int)(min(res.y, 1.0f) * 255.0f);
				v[2] = (int)(min(res.x, 1.0f) * 255.0f);
				});
		}

		cv::imshow("wxnb", view);
		cv::waitKey(1);
	}
	
	return 0;
}
