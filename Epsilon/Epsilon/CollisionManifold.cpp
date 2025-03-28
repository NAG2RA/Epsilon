#include "CollisionManifold.h"

CollisionManifold::CollisionManifold(EpsilonBody& bA, EpsilonBody& bB, Vector2f c1, Vector2f c2, Vector2f& n, float& d, int cCount)
	:bodyA(bA),
	bodyB(bB),
	contact1(c1),
	contact2(c2),
	normal(n),
	depth(d),
	contactCount(cCount)
{
}
