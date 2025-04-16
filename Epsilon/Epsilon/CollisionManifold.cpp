#include "CollisionManifold.h"

CollisionManifold::CollisionManifold(EpsilonBody& bA, EpsilonBody& bB, EpsilonVector c1, EpsilonVector c2, EpsilonVector& n, float& d, int cCount)
	:bodyA(bA),
	bodyB(bB),
	contact1(c1),
	contact2(c2),
	normal(n),
	depth(d),
	contactCount(cCount)
{
}
