#pragma once;
#include <math.h>

namespace wxcgal {

	struct Point3
	{
		float x, y, z;
		Point3(float x = 0.0f, float y = 0.0f, float z = 0.0f) : x(x), y(y), z(z) {}
	};

	typedef Point3 Vector3;

	Vector3 operator+(const Vector3& A, const Vector3& B) { return Vector3(A.x + B.x, A.y + B.y, A.z + B.z); }
	Vector3 operator-(const Vector3& A, const Vector3& B) { return Vector3(A.x - B.x, A.y - B.y, A.z - B.z); }
	Vector3 operator*(const Vector3& A, float p) { return Vector3(A.x * p, A.y * p, A.z * p); }
	Vector3 operator/(const Vector3& A, float p) { return Vector3(A.x / p, A.y / p, A.z / p); }
	bool operator==(const Vector3& A, const Vector3& B) { return A.x == B.x && A.y == B.y && A.z == B.z; }
	bool operator!=(const Vector3& A, const Vector3& B) { return A.x != B.x || A.y != B.y || A.z != B.z; }

	const float eps = 1e-8;
	int dcmp(float x)
	{
		if (fabs(x) < eps) return 0;
		else return x < 0 ? -1 : 1;
	}
	float Dot(const Vector3& A, const Vector3& B) { return A.x * B.x + A.y * B.y + A.z * B.z; }
	float Length(const Vector3& A) { return sqrt(Dot(A, A)); }
	Vector3 Norm(const Vector3& N) { return N / Length(N); }
	float Angle(const Vector3& A, const Vector3& B) { return acos(Dot(A, B) / Length(A) / Length(B)); }
	Vector3 Cross(const Vector3& A, const Vector3& B) { return Vector3(A.y * B.z - A.z * B.y, A.z * B.x - A.x * B.z, A.x * B.y - A.y * B.x); }
	
	float Area2(const Point3& A, const Point3& B, const Point3& C) { return Length(Cross(B - A, C - A)); }

	//��P��ֱ��AB�ľ���
	float DistanceToLine(Point3 P, Point3 A, Point3 B)
	{
		Vector3 v1 = B - A, v2 = P - A;
		return Length(Cross(v1, v2)) / Length(v1);
	}
	//��P���߶�AB�ľ���
	float DistanceToSegment(Point3 P, Point3 A, Point3 B)
	{
		if (A == B) return Length(P - A);
		Vector3 v1 = B - A, v2 = P - A, v3 = P - B;
		if (dcmp(Dot(v1, v2)) < 0) return Length(v2);
		else if (dcmp(Dot(v1, v3)) > 0) return Length(v3);
		else return Length(Cross(v1, v2)) / Length(v1);
	}
	//���� AB, AC, AD�Ļ�ϻ�����Ҳ����������ABCD�����������6��
	float Volume6(Point3 A, Point3 B, Point3 C, Point3 D)
	{
		return Dot(D - A, Cross(B - A, C - A));
	}

	//�ռ���������
	Point3 Centroid(const Point3& A, const Point3& B, const Point3& C, const Point3& D) { return (A + B + C + D) / 4.0; }
	// p1��p2�Ƿ����߶�a-b��ͬ��
	bool SameSide(const Point3& p1, const Point3& p2, const Point3& a, const Point3& b)
	{
		return dcmp(Dot(Cross(b - a, p1 - a), Cross(b - a, p2 - a))) >= 0;
	}
	// ����������P0, P1, P2��
	bool PointInTri(const Point3& P, const Point3& P0, const Point3& P1, const Point3& P2)
	{
		return SameSide(P, P0, P1, P2) && SameSide(P, P1, P0, P2) && SameSide(P, P2, P0, P1);
	}
	// ������P0P1P2�Ƿ���߶�AB�ཻ
	bool TriSegIntersection(const Point3& P0, const Point3& P1, const Point3& P2, const Point3& A, const Point3& B, Point3& P)
	{
		Vector3 n = Cross(P1 - P0, P2 - P0);
		if (dcmp(Dot(n, B - A)) == 0) return false; // �߶�A-B��ƽ��P0P1P2ƽ�л���
		else
		{ // ƽ��A��ֱ��P1-P2��Ωһ����
			float t = Dot(n, P0 - A) / Dot(n, B - A);
			if (dcmp(t) < 0 || dcmp(t - 1) > 0) return false;	// �����߶�AB��
			P = A + (B - A) * t; // ����
			return PointInTri(P, P0, P1, P2);
		}
	}
	//�жϿռ��������ཻ
	bool TriTriIntersection(Point3* T1, Point3* T2)
	{
		Point3 P;
		for (int i = 0; i < 3; i++)
		{
			if (TriSegIntersection(T1[0], T1[1], T1[2], T2[i], T2[(i + 1) % 3], P)) return true;
			if (TriSegIntersection(T2[0], T2[1], T2[2], T1[i], T1[(i + 1) % 3], P)) return true;
		}
		return false;
	}

	//�ռ�������ABC��ֱ��L1-L2����
	bool TriLineIntersection(const Point3& P0, const Point3& P1, const Point3& P2,
		const Point3& A, const Point3& B, Point3& P) {
		Vector3 n = Cross(P1 - P0, P2 - P0);
		if (dcmp(Dot(n, B - A)) == 0) return false;
		float t = Dot(n, P0 - A) / Dot(n, B - A);
		P = A + (B - A) * t;
		return PointInTri(P, P0, P1, P2);
	}

}

