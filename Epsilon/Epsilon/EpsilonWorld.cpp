#include "EpsilonWorld.h"

EpsilonWorld::EpsilonWorld()
	:depth(0),
	normal(0,0)
{
}

void EpsilonWorld::AddBody(EpsilonBody body)
{
	bodyList.push_back(body);
}

bool EpsilonWorld::GetBody(float index,EpsilonBody& body)
{
	if (index < bodyList.size()) {
		return true;
	}
	else {
		cout << "Index was out of bounds";
		return false;
	}
}

void EpsilonWorld::Update(float dt)
{
	for (size_t i = 0; i < bodyList.size(); i++) {
		for (size_t j = i + 1; j < bodyList.size(); j++) {
			if (Collide(bodyList[i], bodyList[j], normal, depth)) {
				Vector2f VelocityA = -normal * (float)(depth / 2), acceleration;
				Vector2f VelocityB = -VelocityA;
				bodyList[i].Move(VelocityA);
				bodyList[j].Move(VelocityB);
				ResolveCollison(bodyList[i], bodyList[j], normal, depth);	
			}
			
		}
	}
}

bool EpsilonWorld::Collide(EpsilonBody bodyA, EpsilonBody bodyB, Vector2f& normal, float& depth)
{
	if (bodyA.shapetype == box) {
		if (bodyB.shapetype == box) {
			if (Collisions::IntersectPolygons(bodyA.GetTransformedVertices(), bodyB.GetTransformedVertices(), normal, depth)) {
				return true;
			}
		}
		else {
			if (Collisions::IntersectPolygonAndCircle(bodyB.position, bodyB.radius, bodyA.GetTransformedVertices(), normal, depth)) {
				normal = -normal;
				return true;
			}
		}
	}
	else {
		if (bodyB.shapetype == box) {
			if (Collisions::IntersectPolygonAndCircle(bodyA.position, bodyA.radius, bodyB.GetTransformedVertices(), normal, depth)) {
				return true;
			}
		}
		else {
			if (Collisions::IntersectCircles(bodyA.radius, bodyB.radius, bodyA.position, bodyB.position, normal, depth)) {
				return true;
			}
		}
	}
	return false;
}

void EpsilonWorld::ResolveCollison(EpsilonBody& bodyA, EpsilonBody& bodyB, Vector2f normal, float depth)
{
	Vector2f relativeVelocity = bodyB.linearVelocity - bodyA.linearVelocity;
	float e = min(bodyA.restitution, bodyB.restitution);
	float j = -(1 + e) * relativeVelocity.dot(normal);
	j /= ((float) 1 / bodyA.mass) + ((float) 1 / bodyB.mass);
	bodyA.linearVelocity -= (j / bodyA.mass) * normal;
	bodyB.linearVelocity += (j / bodyB.mass) * normal;
	
}
