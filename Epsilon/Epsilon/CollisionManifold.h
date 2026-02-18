#pragma once
#include"EpsilonBody.h"
#include"EpsilonVector.h"
class CollisionManifold
{
public:
	EpsilonBody& bodyA;
	EpsilonBody& bodyB;
	EpsilonVector contact1;
	EpsilonVector contact2;
	EpsilonVector normal;
	float depth;
	int contactCount;
	vector<float> accumulatedNormalImpulse;
	vector<float> accumulatedTangentImpulse;
	float deltaTime;
	vector<EpsilonVector> tangentList;
	CollisionManifold(EpsilonBody& bA, EpsilonBody& bB, EpsilonVector c1, EpsilonVector c2, EpsilonVector& n, float& d, int cCount);
};

