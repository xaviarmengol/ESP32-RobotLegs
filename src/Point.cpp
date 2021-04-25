
//Start of Vectors.cpp file

#include "Point.hpp"


float FlynnVector2::Angle(FlynnVector2 rightVec){
	FlynnVector2 tempVec = Normalized();
	float temp = tempVec.Dot(rightVec.Normalized());
	return float(acos(temp));
}

void FlynnVector2::Vec3ToVec2(FlynnVector3 rightVec){
	x = rightVec.x;
	y = rightVec.y;
}

FlynnVector2 FlynnVector2::operator+(FlynnVector2 rightVec)const{
	return FlynnVector2(*this) += rightVec;
}

FlynnVector2 FlynnVector2::operator-(FlynnVector2 rightVec) const{
	return FlynnVector2(*this) -= rightVec;
}

FlynnVector2 FlynnVector2::operator*(float scalar) const{
	return FlynnVector2(*this) *= scalar;
}

FlynnVector2 FlynnVector2::operator/(float scalar) const{
	return FlynnVector2(*this) /= scalar;
}

FlynnVector2& FlynnVector2::operator+=(FlynnVector2 rightVec){
	x += rightVec.x;
	y += rightVec.y;
	return *this;
}

FlynnVector2& FlynnVector2::operator-=(FlynnVector2 rightVec){
	x -= rightVec.x;
	y -= rightVec.y;
	return *this;
}

FlynnVector2& FlynnVector2::operator*=(float scalar){
	x *= scalar;
	y *= scalar;
	return *this;
}

FlynnVector2& FlynnVector2::operator/=(float scalar){
	x /= scalar;
	y /= scalar;
	return *this;
}

FlynnVector2& FlynnVector2::operator=(FlynnVector2 rightVec){
	x = rightVec.x;
	y = rightVec.y;
	return *this;
}

bool FlynnVector2::operator==(const FlynnVector2& rightVec) const{
	return (x == rightVec.x && y == rightVec.y);
}

bool FlynnVector2::operator!=(const FlynnVector2& rightVec) const{
	 return !operator==(rightVec);
}

FlynnVector3 FlynnVector3::Cross(FlynnVector3 rightVec){
	return FlynnVector3((y * rightVec.z) - (z * rightVec.y), 
		           (z * rightVec.x) - (x * rightVec.z), 
			   (x * rightVec.y) - (y * rightVec.x));
}

float FlynnVector3::Angle(FlynnVector3 rightVec){
	FlynnVector3 tempVec = Normalized();
	float temp = tempVec.Dot(rightVec.Normalized());
	return float(acos(temp));
}

FlynnVector3 FlynnVector3::operator+(FlynnVector3 rightVec)const{
	return FlynnVector3(*this) += rightVec;
}

FlynnVector3 FlynnVector3::operator-(FlynnVector3 rightVec) const{
	return FlynnVector3(*this) -= rightVec;
}

FlynnVector3 FlynnVector3::operator*(float scalar) const{
	return FlynnVector3(*this) *= scalar;
}

FlynnVector3 FlynnVector3::operator/(float scalar) const{
	return FlynnVector3(*this) /= scalar;
}

FlynnVector3& FlynnVector3::operator+=(FlynnVector3 rightVec){
	x += rightVec.x;
	y += rightVec.y;
	z += rightVec.z;
	return *this;
}

FlynnVector3& FlynnVector3::operator-=(FlynnVector3 rightVec){
	x -= rightVec.x;
	y -= rightVec.y;
	z -= rightVec.z;
	return *this;
}

FlynnVector3& FlynnVector3::operator*=(float scalar){
	x *= scalar;
	y *= scalar;
	z *= scalar;
	return *this;
}

FlynnVector3& FlynnVector3::operator/=(float scalar){
	x /= scalar;
	y /= scalar;
	z /= scalar;
	return *this;
}

FlynnVector3& FlynnVector3::operator=(FlynnVector3 rightVec){
	x = rightVec.x;
	y = rightVec.y;
	z = rightVec.z;
	return *this;
}

bool FlynnVector3::operator==(const FlynnVector3& rightVec) const{
	return (x == rightVec.x &&
		y == rightVec.y &&
		z == rightVec.z);
}

bool FlynnVector3::operator!=(const FlynnVector3& rightVec) const{
	return !operator==(rightVec);
}