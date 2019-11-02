#pragma once
#include <cmath>
namespace wxrt {

	typedef unsigned int uint;

	struct float_3 {
		float_3() {}
		float_3(float _x, float _y, float _z) : x(_x), y(_y), z(_z) {}
		float x, y, z;
	};

	inline float_3 operator+(const float_3& a, const float_3& b) {
		return float_3(a.x + b.x, a.y + b.y, a.z + b.z);
	}

	inline float_3 operator-(const float_3& a, const float_3& b) {
		return float_3(a.x - b.x, a.y - b.y, a.z - b.z);
	}

	inline float_3 operator+(const float_3& a, float b) {
		return float_3(a.x + b, a.y + b, a.z + b);
	}

	inline float_3 operator-(const float_3& a, float b) {
		return float_3(a.x - b, a.y - b, a.z - b);
	}

	inline float_3 operator*(const float_3& a, float b) {
		return float_3(a.x * b, a.y * b, a.z * b);
	}

	inline float_3 operator/(const float_3& a, float b) {
		return float_3(a.x / b, a.y / b, a.z / b);
	}

	inline float_3 operator*(const float_3& a, const float_3& b) {
		return float_3(a.x * b.x, a.y * b.y, a.z * b.z);
	}

	inline float_3 operator/(const float_3& a, const float_3& b) {
		return float_3(a.x / b.x, a.y / b.y, a.z / b.z);
	}

	inline float_3 cross(const float_3& A, const float_3& B) {
		return float_3(A.y * B.z - A.z * B.y,
			A.z * B.x - A.x * B.z,
			A.x * B.y - A.y * B.x);
	}

	inline float dot(const float_3& A, const float_3& B) {
		return A.x * B.x + A.y * B.y + A.z * B.z;
	}

	inline float length(const float_3& A) {
		return sqrt(A.x * A.x + A.y * A.y + A.z * A.z);
	}

	inline float_3 normalize(const float_3& A) {
		return A / length(A);
	}
};
