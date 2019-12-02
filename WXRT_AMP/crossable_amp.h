#pragma once
#include <amp_graphics.h>
#include <amp_math.h>

using namespace concurrency;
using namespace concurrency::fast_math;
using namespace concurrency::graphics;

namespace wxrt {

	inline bool cant_cross_bbox(const float_3& b_max, const float_3& b_min, 
		const float_3& dir, const float_3& at) restrict(amp) {
		if (dir.x > 0) { if (at.x > b_max.x) return true; }
		else { if (at.x < b_min.x) return true; }
		if (dir.y > 0) { if (at.y > b_max.y) return true; }
		else { if (at.y < b_min.y) return true; }
		if (dir.z > 0) { if (at.z > b_max.z) return true; }
		else { if (at.z < b_min.z) return true; }
		return false;
	}

	struct material  {
		float_3 _ka;
		float_3 _kd;
		float _ks;
		float _shininess;
		inline float brdf(const float_3& in, const float_3& out, const float_3& normal) const restrict(amp) {
			float_3 h = normalize(out - in);
			return _ks * pow(fmaxf(dot(h, normal), 0.0f), _shininess);
		}
	};

	struct point_light {
		float_3 loc, intensity;
		float a_0, a_1, a_2;
	};

	inline float_3 get_intensity(const float_3& at, const float_3& normal,
		const float_3& view_dir, const material& material, const point_light& point_light) restrict(amp) {
		float_3 light = point_light.loc - at;
		float_3 light_dir = normalize(light);
		float_3 ambient = material._ka * point_light.intensity;
		float_3 diffuse = material._kd * fmaxf(dot(normal, light_dir), 0.0f) * point_light.intensity;
		float_3 h = normalize(view_dir + light_dir);
		float_3 specular = material._ka * material._ks * 
			powf(fmaxf(dot(h, normal), 0.0f), material._shininess) * point_light.intensity;
		float distance = length(light);
		float attentuation = 1.0f / (point_light.a_0 + point_light.a_1 * distance + 
			point_light.a_2 * distance * distance);
		return (ambient + diffuse + specular) * attentuation;
	}

	struct triangle {
		float_3 a, b, c;
		float_3 n;
		uint material_id;
		float_3 b_max;
		float_3 b_min;
	};

	inline bool check_cross(const float_3& original_point, const float_3& dir, float& alpha,
		const triangle& triangle) restrict(amp) {
		//if (cant_cross_bbox(triangle.b_max, triangle.b_min, dir, original_point)) return false;
		//https://www.cnblogs.com/graphics/archive/2010/08/09/1795348.html
		float_3 e1 = triangle.b - triangle.a;
		float_3 e2 = triangle.c - triangle.a;
		float_3 p = cross(dir, e2);
		float det = dot(e1, p);
		float_3 t;
		if (det > 0) t = original_point - triangle.a;
		else {
			t = triangle.a - original_point;
			det = -det;
		}
		if (det < 0.0f) return false;
		float u = dot(t, p);
		if (u < 0.0f || u > det) return false;
		float_3 q = cross(t, e1);
		float v = dot(dir, q);
		if (v < 0.0f || u + v > det) return false;
		alpha = dot(e2, q) / det;
		return alpha > 0;
	}

	struct sphere {
		float_3 o;
		float r;
		uint material_id;
		//float_3 b_max;
		//float_3 b_min;
	};

	inline bool check_cross(const float_3& original_point, const float_3& dir, float& alpha,
		const sphere& sphere) restrict(amp) {
		//if (cant_cross_bbox(sphere.b_max, sphere.b_min, dir, original_point)) return false;
		//https://www.cnblogs.com/yoyo-sincerely/p/8401861.html
		float_3 eo = sphere.o - original_point;
		float v = dot(eo, dir);
		float v_2 = v * v;
		alpha = sphere.r * sphere.r + v_2 - dot(eo, eo);
		if (alpha < 0.0f || v_2 < alpha) return false;
		alpha = v - sqrtf(alpha);
		return alpha >= 0.0f;
		//TODO: WENXIN
	}

	inline float_3 get_normal(const sphere& sphere, const float_3& at) restrict(amp) {
		return normalize(at - sphere.o);
		//TODO: WENXIN
	}
}
