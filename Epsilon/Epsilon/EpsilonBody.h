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
	
	float rotation, rotationalVelocity, drag;
	const float density, mass, restitution, area, radius, width, height;
	Vector2f position, linearVelocity, acceleration;
	bool isStatic;
	Shapetype shapetype;
	vector<Vector2f> vertices;
	vector<Vector2f> transformedVertices;
	vector<int> boxtriangles;
	bool isTransformUpdated;
	EpsilonBody(Vector2f position, float density, float mass, float restitution, float area, float radius, float width,
		float height, bool isStatic, Shapetype shapetype);
	static EpsilonBody CreateNewBody(EpsilonBody body);
	static EpsilonBody CreateCircleBody(Vector2f position, float density, float restitution, float radius, bool isStatic);
	static EpsilonBody CreateBoxBody(Vector2f position, float density, float restitution, float width, float height, bool isStatic);
	void updateMovement(float& dt);
	void Move(Vector2f amount);
	vector<Vector2f> GetBoxVertices(float width, float height);
	vector<int> GetBoxTriangles();
	Vector2f Transform(Vector2f position,Vector2f endposition, float angle);
	void MoveTo(Vector2f& pos);
	void UpdateRotation(float angle);
	vector<Vector2f> GetTransformedVertices();
};

