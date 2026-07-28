#pragma once
#include"EpsilonVector.h"
#include<vector>
#include"AABB.h"
struct Water
{
public:
	EpsilonVector surfacePosition;
	float density;
	float width;
	float depth;
	AABB dimensions;
	Water(EpsilonVector surfacePosition, float density, float width, float depth);
	Water(AABB dimensions, float density);
};

