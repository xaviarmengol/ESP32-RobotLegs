//Start of Vectors.h file
#pragma once

#include <cmath>
#define pi 3.14159

class Vector3;

class Vector2{
public:
	double x, y;
	Vector2() : x(0), y(0){};
	Vector2(double X, double Y) : x(X), y(Y){}

	double GetLength() const { return sqrt((x * x) + (y * y)); }
	void Normalize(){ *this /= GetLength(); }
	Vector2 Normalized(){ return *this /= GetLength(); }
	double Dot(Vector2 rightVec) const { return (x * rightVec.x) + (y * rightVec.y); }
	double Angle(Vector2);
	double GreaterAngle(Vector2 rightVec){ return double((2 * pi) - Angle(rightVec)); }
	void Vec3ToVec2(Vector3);

	Vector2 operator+ (Vector2) const;
	Vector2 operator- (Vector2) const;
	Vector2 operator* (double) const;
	Vector2 operator/ (double) const;
	Vector2& operator+= (Vector2);
	Vector2& operator-= (Vector2);
	Vector2& operator*= (double);
	Vector2& operator/= (double);
	Vector2& operator= (Vector2);
	bool operator== (const Vector2&) const;
	bool operator!= (const Vector2&) const;
};

class Vector3{
public:
	double x, y, z;
	Vector3() : x(0), y(0), z(0){};
	Vector3(double X, double Y, double Z) : x(X), y(Y), z(Z) {}

	double GetLength() const { return sqrt((x * x) + (y * y) + (z * z)); }
	void Normalize(){ *this /= GetLength(); }
	Vector3 Normalized(){ return *this /= GetLength(); }
	double Dot(Vector3 rightVec) const { return (x * rightVec.x) + (y * rightVec.y) + (z * rightVec.z); }
	Vector3 Cross(Vector3);
	double Angle(Vector3);
	double GreaterAngle(Vector3 rightVec){ return double((2 * pi) - Angle(rightVec)); }
	void Vec2ToVec3(Vector2 rightVec){ x = rightVec.x, y = rightVec.y, z = 0; }

	Vector3 operator+ (Vector3) const;
	Vector3 operator- (Vector3) const;
	Vector3 operator* (double) const;
	Vector3 operator/ (double) const;
	Vector3& operator+= (Vector3);
	Vector3& operator-= (Vector3);
	Vector3& operator*= (double);
	Vector3& operator/= (double);
	Vector3& operator= (Vector3);
	bool operator== (const Vector3&) const;
	bool operator!= (const Vector3&) const;
};
