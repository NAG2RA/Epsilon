#include "AABB.h"

AABB::AABB(EpsilonVector Min, EpsilonVector Max)
{
	if (Min.x > Max.x) {
		float a = Min.x;
		Min.x = Max.x;
		Max.x = a;
	}
	if (Min.y > Max.y) {
		float a = Min.y;
		Min.y = Max.y;
		Max.y = a;
	}
	min = Min;
	max = Max;
}

AABB::AABB(float MinX, float MaxX, float MinY, float MaxY)
{
	if (MinX > MaxX) {
		float a = MinX;
		MinX = MaxX;
		MaxX = a;
	}
	if (MinY > MaxY) {
		float a = MinY;
		MinY = MaxY;
		MaxY = a;
	}
	min = EpsilonVector(MinX, MinY);
	max = EpsilonVector(MaxX, MaxY);
}
