#pragma once
#include<SFML/Graphics.hpp>
#include<SFML/Window.hpp>
#include<SFML/System.hpp>
#include<iostream>
#include<vector>
#include<cmath>
#include"AABB.h"
#include<limits>
using namespace sf;
using namespace std;

enum Shapetype {
	circle = 0,
	box = 1
};
class EpsilonBody
{
public:
	float angle, angularVelocity, inverseMass;
	float density, mass, restitution, area, radius, width, height, inertia, inverseInertia, dynamicFriction, staticFriction;
	Vector2f position, linearVelocity, force;
	AABB aabb;
	bool isStatic;
	Shapetype shapetype;
	vector<Vector2f> vertices;
	vector<Vector2f> transformedVertices;
	bool isTransformUpdated;
	bool isAABBUpdated;
	EpsilonBody(Vector2f position, float density, float mass, float inertia, float restitution, float area, float radius, float width,
		float height,vector<Vector2f> vertices, bool isStatic, Shapetype shapetype);
	static EpsilonBody CreateNewBody(EpsilonBody body);
	static EpsilonBody CreateCircleBody(Vector2f position, float density, float restitution, float radius, bool isStatic);
	static EpsilonBody CreateBoxBody(Vector2f position, float density, float restitution, float width, float height, bool isStatic);
	void updateMovement(float dt, Vector2f gravity, int iterations);
	void Move(Vector2f amount);
	static vector<Vector2f> GetBoxVertices(float width, float height);
	Vector2f Transform(Vector2f position,Vector2f endposition, float angle);
	void MoveTo(Vector2f& pos);
	void UpdateRotation(float angle);
	void AddForce(Vector2f amount);
	vector<Vector2f> GetTransformedVertices();
	AABB GetAABB();
};

