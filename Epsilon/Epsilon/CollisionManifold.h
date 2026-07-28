#pragma once
#include"EpsilonBody.h"
#include"EpsilonVector.h"
struct CollisionManifold
{
public:
	EpsilonBody& bodyA;
	EpsilonBody& bodyB;
	vector<EpsilonVector> tangentList;
	vector<float> accumulatedNormalImpulse;
	vector<float> accumulatedTangentImpulse;
	EpsilonVector contact1;
	EpsilonVector contact2;
	EpsilonVector normal;
	float depth;
	float deltaTime;
	int contactCount;
	CollisionManifold(EpsilonBody& bA, EpsilonBody& bB, EpsilonVector c1, EpsilonVector c2, EpsilonVector& n, float& d, int cCount);
};

struct CollisionManifoldDOD
{
public:
	int entityA_ID;
	int entityB_ID;
	uint32_t posIdxA, posIdxB;
	uint32_t angleIdxA, angleIdxB;
	uint32_t velIdxA, velIdxB;
	uint32_t invIdxA, invIdxB;
	uint32_t fricIdxA, fricIdxB;
	EpsilonVector tangentList[2];
	float accumulatedNormalImpulse[2];
	float accumulatedTangentImpulse[2];
	EpsilonVector contact1;
	EpsilonVector contact2;
	EpsilonVector normal;
	float depth;
	float deltaTime;
	int contactCount;
	CollisionManifoldDOD(int& entA_id, int& entB_id, EpsilonVector c1, EpsilonVector c2, EpsilonVector& n, float& d, int cCount);
};
