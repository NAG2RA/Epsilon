#pragma once
#include<SFML/Graphics.hpp>
#include<SFML/Window.hpp>
#include<SFML/System.hpp>
#include<iostream>
#include<vector>
#include<cmath>
using namespace sf;
using namespace std;

enum Shapetype {
	circle = 0,
	box = 1
};
class EpsilonBody
{
public:
	
	float rotation, rotationalVelocity;
	const float density, mass, restitution, area, radius, width, height;
	Vector2f position, linearVelocity;
	bool isStatic;
	Shapetype shapetype;
	EpsilonBody(Vector2f position, float density, float mass, float restitution, float area, float radius, float width,
		float height, bool isStatic, Shapetype shapetype);
	EpsilonBody CreateNewBody(EpsilonBody& body);
	static EpsilonBody CreateCircleBody(Vector2f position, float density, float restitution, float radius, bool isStatic);
	static EpsilonBody CreateBoxBody(Vector2f position, float density, float restitution, float width, float height, bool isStatic);
	Vector2f updateMovement(Vector2f& position, float& dt, Vector2f& velocity, Vector2f& acceleration, float& drag);
};

