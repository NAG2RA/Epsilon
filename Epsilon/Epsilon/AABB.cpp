#include "AABB.h"

AABB::AABB(Vector2f Min, Vector2f Max)
{
	min = Min;
	max = Max;
}

AABB::AABB(float MinX, float MaxX, float MinY, float MaxY)
{
	min = Vector2f(MinX, MinY);
	max = Vector2f(MaxX, MaxY);
}
