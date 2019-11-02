#pragma once

#include "wxrt_tools.h"

namespace wxrt {

	inline float max(float a, float b) {
		return a > b ? a : b;
	}

	class material {
	public:
		material(const float_3& ka, const float_3& kd, const float_3& ks, float shininess)
			: _ka(ka), _kd(kd), _ks(ks), _shininess(shininess) {}
		float_3 _ka;
		float_3 _kd;
		float_3 _ks;
		float _shininess;
		inline virtual float_3 brdf(const float_3 in, const float_3 out) const {
			return float_3(1.0f, 1.0f, 1.0f);
		}
	};

	class light_source {
	public:
		virtual float_3 get_location() const = 0;
		virtual float_3 get_intensity(const float_3& at, const float_3& normal,
			const float_3& view_dir, const material& material) const = 0;
	};

	class point_light : public light_source {
	public:
		point_light(const float_3& loc, float_3 intensity = float_3(1.0f, 1.0f, 1.0f), 
			float a_0 = 1.0f, float a_1 = 0.4f, float a_2 = 0.036f)
			: _loc(loc), _intensity(intensity), _a_0(a_0), _a_1(a_1), _a_2(a_2){}
		inline float_3 get_location() const {
			return _loc;
		}

		inline float_3 get_intensity(const float_3& at, const float_3& normal, 
			const float_3& view_dir, const material& material) const {
			float_3 light = _loc - at;
			float_3 light_dir = normalize(light);
			float_3 ambient = material._ka * _intensity;
			float_3 diffuse = material._kd * max(dot(normal, light_dir), 0.0f) * _intensity;
			float_3 h = normalize(view_dir + light_dir);
			float_3 specular = material._ks * pow(max(dot(h, normal), 0.0f), material._shininess)
				* _intensity;
			float distance = length(light);
			float attentuation = 1.0f / (_a_0 + _a_1 * distance + _a_2 * distance * distance);
			return (ambient + diffuse + specular) * attentuation;
		}
	private:
		float_3 _loc, _intensity;
		float _a_0, _a_1, _a_2;
	};

	class crossable {
	public:
		virtual bool check_cross(const float_3& original_point, const float_3& dir, float& alpha) const = 0;
		virtual float_3 get_normal(const float_3& at) const = 0;
		virtual uint get_material_id() const = 0;
	};

	class triangle : public crossable {
	public:
		triangle(const float_3& a, const float_3& b, const float_3& c,
			const float_3& n, uint material_id) : 
			_a(a), _b(b), _c(c), _n(n), _material_id(material_id){}
		inline uint get_material_id() const {
			return _material_id;
		}

		inline bool check_cross(const float_3& original_point, const float_3& dir, float& alpha) const {
			//https://www.cnblogs.com/graphics/archive/2010/08/09/1795348.html
			float_3 e1 = _b - _a;
			float_3 e2 = _c - _a;
			float_3 p = cross(dir, e2);
			float det = dot(e1, p);
			float_3 t;
			if (det > 0) t = original_point - _a;
			else {
				t = _a - original_point;
				det = -det;
			}
			if (det < 0.0001f) return false;
			float u = dot(t, p);
			if (u < 0.0f || u > det) return false;
			float_3 q = cross(t, e1);
			float v = dot(dir, q);
			if (v < 0.0f || u + v > det) return false;
			alpha = dot(e2, q) / det;
			return alpha > 0;
		}

		inline float_3 get_normal(const float_3& at) const {
			return _n;
		}
	private:
		float_3 _a, _b, _c;
		float_3 _n;
		uint _material_id;
	};



}
