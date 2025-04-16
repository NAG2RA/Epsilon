#pragma once
#include<SFML/Graphics.hpp>
#include<SFML/Window.hpp>
#include<SFML/System.hpp>
#include<iostream>
#include<vector>
#include<cmath>
#include"EpsilonVector.h"
using namespace sf;
using namespace std;
class AABB 
{
public:
	EpsilonVector min, max;
	AABB(EpsilonVector Min, EpsilonVector Max);
	AABB(float MinX, float MaxX, float MinY, float MaxY);
};

