#pragma once
#include<iostream>
#include<vector>
#include<mutex>
#include"AABB.h"
#include"EpsilonVector.h"
using namespace std;

enum Shapetype {
	circle = 0,
	box = 1,
	triangle = 2
};
enum Connectiontype {
	none = 0,
	thr = 1,
	spring = 2
};
class EpsilonBody
{
public:
	bool isLocked;
	float angle, angularVelocity, inverseMass, connectionDistance;
	float density, mass, restitution, area, radius, width, height, inertia, inverseInertia, dynamicFriction, staticFriction;
	bool isStatic;
	EpsilonVector position, linearVelocity, force, originPosition, connectionPosition;
	AABB aabb;
	Shapetype shapetype;
	Connectiontype connectiontype;
	bool isTransformUpdated;
	bool isAABBUpdated;
	EpsilonBody(EpsilonVector position, float density, float mass, float inertia, float restitution, float area, float radius, float width,
		float height,vector<EpsilonVector> vertices, bool isStatic, Shapetype shapetype, Connectiontype connectiontype);
	static EpsilonBody CreateNewBody(EpsilonBody body);
	static EpsilonBody CreateCircleBody(EpsilonVector position, float density, float restitution, float radius, bool isStatic, Connectiontype connectiontype);
	static EpsilonBody CreateBoxBody(EpsilonVector position, float density, float restitution, float width, float height, bool isStatic, Connectiontype connectiontype);
	static EpsilonBody CreateTriangleBody(EpsilonVector position, float density, float restitution, float side, bool isStatic, Connectiontype connectiontype);
	void CreateConnection(EpsilonVector origin);
	void updateMovement(float dt, EpsilonVector gravity,int iterations);
	void Move(EpsilonVector amount);
	vector<EpsilonVector> GetTransformedVertices();
	void MoveTo(EpsilonVector& pos);
	void AddForce(EpsilonVector amount);
	AABB GetAABB();
	EpsilonVector Transform(EpsilonVector position, EpsilonVector endposition, float angle);
private:
	static vector<EpsilonVector> GetBoxVertices(float width, float height);
	static vector<EpsilonVector> GetTriangleVertices(float side);
	void UpdateRotation(float angle);
	vector<EpsilonVector> transformedVertices;
	vector<EpsilonVector> vertices;

};

