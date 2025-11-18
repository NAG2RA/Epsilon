#pragma once
#include<vector>
#include"EpsilonBody.h"
#include"Collisions.h"
#include"CollisionManifold.h"
#include"EpsilonVector.h"
#include"Water.h"
class EpsilonWorld
{
public:
	EpsilonWorld();
	int GetBodyCount();
	void AddBody(EpsilonBody body);
	void RemoveBody(int index);
	int GetDynamicBodyCount();
	void Explosion(EpsilonVector position, float radius, float magnitude);
	void CreateWater(EpsilonVector surfacePosition, float width, float depth, float density);
	void DeleteWater(int index);
	void Update(float dt, int iterations);
	EpsilonBody GetBody(float index);
	EpsilonBody* GetDynamicBody(float index);
private:
	float airResistanceConstant;
	float rotationalAirResistanceConstant;
	float springConstant, damperConstant, damperThreadConstant, damperWaterConstant;
	float depth;
	EpsilonVector gravity;
	EpsilonVector normal;
	vector<Water> waterList;
	vector<EpsilonBody> bodyList;
	vector<EpsilonBody*> staticBodyList;
	vector<EpsilonBody*> dynamicBodyList;
	vector<vector<EpsilonBody*>> contactPairs;
	vector<EpsilonBody*> potentialColliders;
	vector<EpsilonBody*> highPriorityObjects;
	vector<EpsilonBody*> lowPriorityObjects;
	vector<vector<EpsilonBody*>> highPriorityPairs;
	vector<vector<EpsilonBody*>> lowPriorityPairs;
	void PreFiltering();
	void UpdateMovement(float dt, int iterations);
	void UpdateHighPriorityMovement(float dt, int iterations);
	void SeperateBodies(EpsilonBody& bodyA, EpsilonBody& bodyB, EpsilonVector mtv);
	void BroadPhase(int windowWidth = 1280, int windowHeight = 720, float zoom = 1.f);
	void NarrowPhase();
	void ResolveCollisonBasic(CollisionManifold& manifold);
	void ResolveCollisonWithRotation(CollisionManifold& manifold);
	void ResolveCollisonWithRotationAndFriction(CollisionManifold& manifold);
	void ZoZoResolveCollisonBasic(CollisionManifold& manifold);
	void ResolveThreadConnection();
	void ResolveSpringConnection(float dt, int iterations);
	void ResolveHighPriorityThreadConnection();
	void ResolveHighPrioritySpringConnection(float dt, int iterations);
	void Buoyancy();
	void AirResistance(float dt, int iterations);

};

