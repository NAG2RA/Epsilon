#pragma once
#include<cmath>
using namespace std;
class EpsilonVector
{
public:
	float x;
	float y;
	EpsilonVector(float x = 0,float y = 0);
	EpsilonVector operator+( EpsilonVector b);
	EpsilonVector& operator+=( EpsilonVector b);
	EpsilonVector operator-(EpsilonVector b);
	EpsilonVector& operator-=(EpsilonVector b);
	friend EpsilonVector operator-(EpsilonVector v);
	friend EpsilonVector operator*(EpsilonVector v,float s);
	friend EpsilonVector operator*(float s, EpsilonVector& v);
	friend EpsilonVector operator/(EpsilonVector v, float s);
	float LengthSquared();
	float Length();
	EpsilonVector Normalized();
	float Dot(EpsilonVector a);
	float Cross(EpsilonVector a);
	float Distance(EpsilonVector a);
	float DistanceSquared(EpsilonVector a);
};

