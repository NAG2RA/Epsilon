#include "AABB.h"

AABB::AABB(EpsilonVector Min, EpsilonVector Max)
{
	min = Min;
	max = Max;
}

AABB::AABB(float MinX, float MaxX, float MinY, float MaxY)
{
	min = EpsilonVector(MinX, MinY);
	max = EpsilonVector(MaxX, MaxY);
}
