#pragma once
#include<SFML/Graphics.hpp>
#include<SFML/Window.hpp>
#include<SFML/System.hpp>
#include<iostream>
#include<vector>
#include<cmath>
#include"AABB.h"
#include<limits>
#include"EpsilonVector.h"
using namespace sf;
using namespace std;

enum Shapetype {
	circle = 0,
	box = 1,
	triangle = 2
};
class EpsilonBody
{
public:
	float angle, angularVelocity, inverseMass;
	float density, mass, restitution, area, radius, width, height, inertia, inverseInertia, dynamicFriction, staticFriction;
	EpsilonVector position, linearVelocity, force;
	AABB aabb;
	bool isStatic;
	Shapetype shapetype;
	vector<EpsilonVector> vertices;
	vector<EpsilonVector> transformedVertices;
	bool isTransformUpdated;
	bool isAABBUpdated;
	EpsilonBody(EpsilonVector position, float density, float mass, float inertia, float restitution, float area, float radius, float width,
		float height,vector<EpsilonVector> vertices, bool isStatic, Shapetype shapetype);
	static EpsilonBody CreateNewBody(EpsilonBody body);
	static EpsilonBody CreateCircleBody(EpsilonVector position, float density, float restitution, float radius, bool isStatic);
	static EpsilonBody CreateBoxBody(EpsilonVector position, float density, float restitution, float width, float height, bool isStatic);
	static EpsilonBody CreateTriangleBody(EpsilonVector position, float density, float restitution, float side, bool isStatic);
	void updateMovement(float dt, EpsilonVector gravity, int iterations);
	void Move(EpsilonVector amount);
	static vector<EpsilonVector> GetBoxVertices(float width, float height);
	static vector<EpsilonVector> GetTriangleVertices(float side);
	EpsilonVector Transform(EpsilonVector position, EpsilonVector endposition, float angle);
	void MoveTo(EpsilonVector& pos);
	void UpdateRotation(float angle);
	void AddForce(EpsilonVector amount);
	vector<EpsilonVector> GetTransformedVertices();
	AABB GetAABB();
};

