#pragma once
#include<SFML/Graphics.hpp>
#include<SFML/Window.hpp>
#include<SFML/System.hpp>
#include<iostream>
#include<vector>
#include<cmath>
#include"EpsilonBody.h"
#include"Collisions.h"
class EpsilonWorld
{
public:
	Vector2f normal;
	float depth;
	EpsilonWorld();
	vector<EpsilonBody> bodyList;
	void AddBody(EpsilonBody body);
	bool GetBody(float index, EpsilonBody& body);
	void Update(float dt);
	bool Collide(EpsilonBody bodyA, EpsilonBody bodyB, Vector2f& normal, float& depth);
	void ResolveCollison(EpsilonBody& bodyA, EpsilonBody& bodyB, Vector2f normal, float depth);
};

