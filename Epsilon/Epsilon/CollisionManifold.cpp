#include "CollisionManifold.h"

CollisionManifold::CollisionManifold(EpsilonBody& bA, EpsilonBody& bB, EpsilonVector c1, EpsilonVector c2, EpsilonVector& n, float& d, int cCount)
	:bodyA(bA),
	bodyB(bB),
	contact1(c1),
	contact2(c2),
	normal(n),
	depth(d),
	contactCount(cCount),
	accumulatedNormalImpulse(2),
	accumulatedTangentImpulse(2),
	tangentList(2),
	deltaTime(0)
{
}

CollisionManifoldDOD::CollisionManifoldDOD(int& entA_id, int& entB_id, EpsilonVector c1, EpsilonVector c2, EpsilonVector& n, float& d, int cCount)
	:entityA_ID(entA_id),
	entityB_ID(entB_id),
	contact1(c1),
	contact2(c2),
	normal(n),
	depth(d),
	contactCount(cCount),
	deltaTime(0)

{
	accumulatedNormalImpulse[0] = 0.0f;
	accumulatedNormalImpulse[1] = 0.0f;
	accumulatedTangentImpulse[0] = 0.0f;
	accumulatedTangentImpulse[1] = 0.0f;
	tangentList[0] = EpsilonVector(0, 0);
	tangentList[1] = EpsilonVector(0, 0);
}
