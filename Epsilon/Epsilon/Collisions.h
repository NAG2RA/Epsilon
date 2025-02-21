#pragma once
#include<SFML/Graphics.hpp>
#include<SFML/Window.hpp>
#include<SFML/System.hpp>
#include<iostream>
#include<vector>
#include<cmath>
#include"EpsilonBody.h"
using namespace sf;
using namespace std;
class Collisions
{
public:
	static float Distance(Vector2f a, Vector2f b);
	static bool IntersectCircles(float radiusA,float radiusB, Vector2f centerA, Vector2f centerB,Vector2f& normal,float& depth);
};

