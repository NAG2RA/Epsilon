#pragma once
#include<cmath>
#include"EpsilonVector.h"
using namespace std;
class AABB 
{
public:
	EpsilonVector min, max;
	AABB(EpsilonVector Min, EpsilonVector Max);
	AABB(float MinX = 0, float MaxX = 0, float MinY = 0, float MaxY = 0);
};

