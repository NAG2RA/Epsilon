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
class CollisionManifold
{
public:
	EpsilonBody& bodyA;
	EpsilonBody& bodyB;
	Vector2f contact1;
	Vector2f contact2;
	Vector2f normal;
	float depth;
	int contactCount;
	CollisionManifold(EpsilonBody& bA, EpsilonBody& bB, Vector2f c1, Vector2f c2, Vector2f& n, float& d, int cCount);
};

