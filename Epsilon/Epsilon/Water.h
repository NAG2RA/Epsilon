#pragma once
#include"EpsilonVector.h"
#include<vector>
class Water
{
public:
	EpsilonVector surfacePosition;
	float density;
	float width;
	float depth;
	Water(EpsilonVector surfacePosition, float density, float width, float depth);
};

