#pragma once
#include<vector>
#include"EpsilonBody.h"
#include"Collisions.h"
#include"CollisionManifold.h"
#include"EpsilonVector.h"

class EpsilonWorld
{
public:
	EpsilonWorld();
	void AddBody(EpsilonBody body);
	void RemoveBody(int index);
	EpsilonBody GetBody(float index);
	int GetBodyCount();
	void Explosion(EpsilonVector position, float radius, float magnitude);
	void CreateWater(EpsilonVector surfacePosition, float width, float depth, float density);
	void Update(float dt, int iterations);
private:
	EpsilonVector gravity;
	vector<vector<AABB>> grid;
	vector<vector<EpsilonBody>> cellBodyList;
	vector<int> display;
	float airResistanceConstant;
	float rotationalAirResistanceConstant;
	float springConstant, damperConstant, damperThreadConstant;
	float waterDensity, waterWidth, waterDepth;
	EpsilonVector waterSurfacePosition;
	EpsilonVector normal;
	float depth;
	vector<EpsilonBody> bodyList;
	vector<tuple<size_t, size_t>> contactPairs;
	void UpdateMovement(float dt, int iterations);
	void SeperateBodies(EpsilonBody& bodyA, EpsilonBody& bodyB, EpsilonVector mtv);
	void BroadPhase();
	void NarrowPhase();
	void ResolveCollisonBasic(CollisionManifold& manifold);
	void ResolveCollisonWithRotation(CollisionManifold& manifold);
	void ResolveCollisonWithRotationAndFriction(CollisionManifold& manifold);
	void ResolveThreadConnection();
	void ResolveSpringConnection(float dt);
	void Buoyancy(EpsilonVector surfacePosition, float width, float depth, float density);
	void AirResistance(float dt);

};

