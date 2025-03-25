#pragma once
#include<SFML/Graphics.hpp>
#include<SFML/Window.hpp>
#include<SFML/System.hpp>
#include<iostream>
#include<vector>
#include<cmath>
using namespace sf;
using namespace std;
class AABB 
{
public:
	Vector2f min, max;
	AABB(Vector2f Min, Vector2f Max);
	AABB(float MinX, float MaxX, float MinY, float MaxY);
};

