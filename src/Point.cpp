
//Start of Vectors.cpp file

#include "Point.hpp"


double Vector2::Angle(Vector2 rightVec){
	Vector2 tempVec = Normalized();
	double temp = tempVec.Dot(rightVec.Normalized());
	return double(acos(temp));
}

void Vector2::Vec3ToVec2(Vector3 rightVec){
	x = rightVec.x;
	y = rightVec.y;
}

Vector2 Vector2::operator+(Vector2 rightVec)const{
	return Vector2(*this) += rightVec;
}

Vector2 Vector2::operator-(Vector2 rightVec) const{
	return Vector2(*this) -= rightVec;
}

Vector2 Vector2::operator*(double scalar) const{
	return Vector2(*this) *= scalar;
}

Vector2 Vector2::operator/(double scalar) const{
	return Vector2(*this) /= scalar;
}

Vector2& Vector2::operator+=(Vector2 rightVec){
	x += rightVec.x;
	y += rightVec.y;
	return *this;
}

Vector2& Vector2::operator-=(Vector2 rightVec){
	x -= rightVec.x;
	y -= rightVec.y;
	return *this;
}

Vector2& Vector2::operator*=(double scalar){
	x *= scalar;
	y *= scalar;
	return *this;
}

Vector2& Vector2::operator/=(double scalar){
	x /= scalar;
	y /= scalar;
	return *this;
}

Vector2& Vector2::operator=(Vector2 rightVec){
	x = rightVec.x;
	y = rightVec.y;
	return *this;
}

bool Vector2::operator==(const Vector2& rightVec) const{
	return (x == rightVec.x && y == rightVec.y);
}

bool Vector2::operator!=(const Vector2& rightVec) const{
	 return !operator==(rightVec);
}

Vector3 Vector3::Cross(Vector3 rightVec){
	return Vector3((y * rightVec.z) - (z * rightVec.y), 
		           (z * rightVec.x) - (x * rightVec.z), 
			   (x * rightVec.y) - (y * rightVec.x));
}

double Vector3::Angle(Vector3 rightVec){
	Vector3 tempVec = Normalized();
	double temp = tempVec.Dot(rightVec.Normalized());
	return double(acos(temp));
}

Vector3 Vector3::operator+(Vector3 rightVec)const{
	return Vector3(*this) += rightVec;
}

Vector3 Vector3::operator-(Vector3 rightVec) const{
	return Vector3(*this) -= rightVec;
}

Vector3 Vector3::operator*(double scalar) const{
	return Vector3(*this) *= scalar;
}

Vector3 Vector3::operator/(double scalar) const{
	return Vector3(*this) /= scalar;
}

Vector3& Vector3::operator+=(Vector3 rightVec){
	x += rightVec.x;
	y += rightVec.y;
	z += rightVec.z;
	return *this;
}

Vector3& Vector3::operator-=(Vector3 rightVec){
	x -= rightVec.x;
	y -= rightVec.y;
	z -= rightVec.z;
	return *this;
}

Vector3& Vector3::operator*=(double scalar){
	x *= scalar;
	y *= scalar;
	z *= scalar;
	return *this;
}

Vector3& Vector3::operator/=(double scalar){
	x /= scalar;
	y /= scalar;
	z /= scalar;
	return *this;
}

Vector3& Vector3::operator=(Vector3 rightVec){
	x = rightVec.x;
	y = rightVec.y;
	z = rightVec.z;
	return *this;
}

bool Vector3::operator==(const Vector3& rightVec) const{
	return (x == rightVec.x &&
		y == rightVec.y &&
		z == rightVec.z);
}

bool Vector3::operator!=(const Vector3& rightVec) const{
	return !operator==(rightVec);
}