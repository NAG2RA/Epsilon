#include "EpsilonWorld.h"

EpsilonWorld::EpsilonWorld()
	:depth(0),
	normal(0,0),
	gravity(0,9.81f)
{
}

void EpsilonWorld::AddBody(EpsilonBody body)
{
	bodyList.push_back(body);
}

void EpsilonWorld::RemoveBody(int index)
{
	bodyList.erase(bodyList.begin() + index);
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
		bodyList[i].updateMovement(dt, gravity);
	}
	for (size_t i = 0; i < bodyList.size(); i++) {
		for (size_t j = i + 1; j < bodyList.size(); j++) {
			if (Collide(bodyList[i], bodyList[j], normal, depth)) {
				Vector2f VelocityA = -normal * (float)(depth/2);
				Vector2f VelocityB = -VelocityA;
				if (bodyList[i].isStatic && bodyList[j].isStatic) {
					continue;
				}
				if(!bodyList[i].isStatic && !bodyList[j].isStatic) {
					bodyList[i].Move(VelocityA);
					bodyList[j].Move(VelocityB);
				}
				else if (bodyList[i].isStatic && !bodyList[j].isStatic) {
					bodyList[j].Move(VelocityB * 2.f);
				}
				else if (!bodyList[i].isStatic && bodyList[j].isStatic) {
					bodyList[i].Move(VelocityA * 2.f);
				}
				ResolveCollison(bodyList[i], bodyList[j], normal, depth);	
			}
		}
	}
}

bool EpsilonWorld::Collide(EpsilonBody bodyA, EpsilonBody bodyB, Vector2f& normal, float& depth)
{
	if (bodyA.shapetype == box) {
		if (bodyB.shapetype == box) {
			if (Collisions::IntersectPolygons(bodyA.position, bodyA.GetTransformedVertices(), bodyB.position, bodyB.GetTransformedVertices(), normal, depth)) {
				return true;
			}
		}
		else {
			if (Collisions::IntersectPolygonAndCircle(bodyB.position, bodyA.position, bodyB.radius, bodyA.GetTransformedVertices(), normal, depth)) {
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

void EpsilonWorld::ResolveCollison(EpsilonBody& bodyA, EpsilonBody& bodyB, Vector2f normal, float& depth)
{
	Vector2f relativeVelocity = bodyB.linearVelocity - bodyA.linearVelocity;
	if (relativeVelocity.dot(normal) > 0.f) {
		return;
	}
	float e = min(bodyA.restitution, bodyB.restitution);
	float j = -(1 + e) * relativeVelocity.dot(normal);
	j /= bodyA.InverseMass+bodyB.InverseMass;
	Vector2f impulse = j * normal;
	bodyA.linearVelocity -= impulse * bodyA.InverseMass;
	bodyB.linearVelocity += impulse * bodyB.InverseMass;	
}
