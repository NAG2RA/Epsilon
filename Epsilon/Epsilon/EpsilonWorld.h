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
	vector<vector<vector<EpsilonBody*>>> grid;
	float airResistanceConstant;
	float rotationalAirResistanceConstant;
	float springConstant, damperConstant, damperThreadConstant;
	float gridCellSizeX;
	float gridCellSizeY;
	float waterDensity, waterWidth, waterDepth;
	EpsilonVector waterSurfacePosition;
	EpsilonVector normal;
	float depth;
	vector<EpsilonBody> bodyList;
	vector<vector<int>> contactPairs;
	void UpdateMovement(float dt, int iterations);
	void SeperateBodies(EpsilonBody& bodyA, EpsilonBody& bodyB, EpsilonVector mtv);
	void BroadPhase(float windowWidth = 1280, float windowHeight = 720, int cellcount = 64, float zoom = 1);
	void NarrowPhase();
	void ResolveCollisonBasic(CollisionManifold& manifold);
	void ResolveCollisonWithRotation(CollisionManifold& manifold);
	void ResolveCollisonWithRotationAndFriction(CollisionManifold& manifold);
	void ZoZoResolveCollisonBasic(CollisionManifold& manifold);
	void ResolveThreadConnection();
	void ResolveSpringConnection(float dt);
	void Buoyancy(EpsilonVector surfacePosition, float width, float depth, float density);
	void AirResistance(float dt);

};

