#pragma once
#include<SFML/Graphics.hpp>
#include<SFML/Window.hpp>
#include<SFML/System.hpp>
#include<iostream>
#include<vector>
#include<cmath>
#include"EpsilonBody.h"
#include"EpsilonVector.h"
using namespace sf;
using namespace std;
class CollisionManifold
{
public:
	EpsilonBody& bodyA;
	EpsilonBody& bodyB;
	EpsilonVector contact1;
	EpsilonVector contact2;
	EpsilonVector normal;
	float depth;
	int contactCount;
	CollisionManifold(EpsilonBody& bA, EpsilonBody& bB, EpsilonVector c1, EpsilonVector c2, EpsilonVector& n, float& d, int cCount);
};

