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

void EpsilonWorld::Update(float dt, int iterations)
{
	if (iterations < 1) {
		iterations = 1;
	}
	else if (iterations > 128) {
		iterations = 128;
	}
	for (int it = 0; it < iterations; it++) {
		
		contactPairs.clear();
		BroadPhase();
		NarrowPhase();
		UpdateMovement(dt,iterations);
		
	}
}

void EpsilonWorld::SeperateBodies(EpsilonBody& bodyA, EpsilonBody& bodyB, Vector2f mtv) {
	if (!bodyA.isStatic && !bodyB.isStatic) {
		bodyA.Move(-mtv / 2.f);
		bodyB.Move(mtv / 2.f);
	}
	else if (bodyA.isStatic) {
		bodyB.Move(mtv);
	}
	else if (bodyB.isStatic) {
		bodyA.Move(-mtv);
	}
}

void EpsilonWorld::BroadPhase() {
	for (size_t i = 0; i < bodyList.size() - 1; i++) {
		for (size_t j = i + 1; j < bodyList.size(); j++) {
			if (bodyList[i].isStatic && bodyList[j].isStatic) {
				continue;
			}
			if (!Collisions::IntersectAABB(bodyList[i].GetAABB(), bodyList[j].GetAABB())) {
				continue;
			}
			
				contactPairs.push_back(make_tuple(i, j));
				
			
		}
	}
}

void EpsilonWorld::NarrowPhase() {
	for (size_t i = 0; i < contactPairs.size(); i++) {
		tuple<size_t, size_t> t = contactPairs[i];
		EpsilonBody& bodyA = bodyList[get<0>(t)];
		EpsilonBody& bodyB = bodyList[get<1>(t)];
		if (Collisions::Collide(bodyA, bodyB, normal, depth)) {
			
			SeperateBodies(bodyA, bodyB, normal * depth);
			Vector2f contact1(0, 0);
			Vector2f contact2(0, 0);
			int contactCount = 0;
			Collisions::FindContactPoints(bodyA, bodyB, contact1, contact2, contactCount);
			CollisionManifold contact(bodyA, bodyB, contact1, contact2, normal, depth, contactCount);
			ResolveCollisonWithRotationAndFriction(contact);
		}


		
	}
}

void EpsilonWorld::UpdateMovement(float dt, int iterations) {
	for (size_t i = 0; i < bodyList.size(); i++) {
		bodyList[i].updateMovement(dt, gravity, iterations);
	}
}

void EpsilonWorld::ResolveCollisonBasic(CollisionManifold& manifold)
{
	EpsilonBody& bodyA = manifold.bodyA;
	EpsilonBody& bodyB = manifold.bodyB;
	Vector2f normal = manifold.normal;
	float depth = manifold.depth;
	Vector2f relativeVelocity = bodyB.linearVelocity - bodyA.linearVelocity;
	if (relativeVelocity.dot(normal) > 0.f) {
		return;
	}
	float e = min(bodyA.restitution, bodyB.restitution);
	float j = -(1 + e) * relativeVelocity.dot(normal);
	j /= bodyA.inverseMass+bodyB.inverseMass;
	Vector2f impulse = j * normal;
	bodyA.linearVelocity -= impulse * bodyA.inverseMass;
	bodyB.linearVelocity += impulse * bodyB.inverseMass;	
}

void EpsilonWorld::ResolveCollisonWithRotation(CollisionManifold& manifold)
{
	EpsilonBody& bodyA = manifold.bodyA;
	EpsilonBody& bodyB = manifold.bodyB;
	Vector2f normal = manifold.normal;
	Vector2f contact1 = manifold.contact1;
	Vector2f contact2 = manifold.contact2;
	int contactCount = manifold.contactCount;
	float e = min(bodyA.restitution, bodyB.restitution);
	vector<Vector2f> contactList = { contact1,contact2 };
	vector<Vector2f> impulseList(2);
	vector<Vector2f> raList(2);
	vector<Vector2f> rbList(2);
	for (size_t i = 0; i < contactCount; i++) {
		Vector2f ra = contactList[i] - bodyA.position;
		Vector2f rb = contactList[i] - bodyB.position;
		raList[i] = ra;
		rbList[i] = rb;
		Vector2f raPerp(-ra.y, ra.x);
		Vector2f rbPerp(-rb.y, rb.x);
		Vector2f angularLinearVelocityA = raPerp * bodyA.angularVelocity;
		Vector2f angularLinearVelocityB = rbPerp * bodyB.angularVelocity;
		Vector2f relativeVelocity = (bodyB.linearVelocity+angularLinearVelocityB) - (bodyA.linearVelocity+angularLinearVelocityA);
		float contactVelocityMag = relativeVelocity.dot(normal);
		if (contactVelocityMag > 0.f) {
			continue;
		}
		float j = -(1 + e) * contactVelocityMag;
		float raPerpDotN = raPerp.dot(normal);
		float rbPerpDotN = rbPerp.dot(normal);
		float denom = bodyA.inverseMass + bodyB.inverseMass + 
			(raPerpDotN*raPerpDotN)*bodyA.inverseInertia + 
			(rbPerpDotN * rbPerpDotN) * bodyB.inverseInertia;
		j /= denom;
		j /= (float)contactCount;
		Vector2f impulse = j * normal;
		impulseList[i] = impulse;
	}
	for (size_t i = 0; i < contactCount; i++) {
		Vector2f ra = raList[i];
		Vector2f rb = rbList[i];
		Vector2f impulse = impulseList[i];
		bodyA.linearVelocity += -impulse * bodyA.inverseMass;
		bodyA.angularVelocity += -ra.cross(impulse) * bodyA.inverseInertia;
		bodyB.linearVelocity += impulse * bodyB.inverseMass;
		bodyB.angularVelocity += rb.cross(impulse) * bodyB.inverseInertia;
	}
}

void EpsilonWorld::ResolveCollisonWithRotationAndFriction(CollisionManifold& manifold)
{
	EpsilonBody& bodyA = manifold.bodyA;
	EpsilonBody& bodyB = manifold.bodyB;
	Vector2f normal = manifold.normal;
	Vector2f contact1 = manifold.contact1;
	Vector2f contact2 = manifold.contact2;
	int contactCount = manifold.contactCount;
	float e = min(bodyA.restitution, bodyB.restitution);
	vector<Vector2f> contactList = { contact1,contact2 };
	vector<Vector2f> impulseList(2);
	vector<Vector2f> raList(2);
	vector<Vector2f> rbList(2);
	vector<Vector2f> frictionImpulseList(2);
	vector<float> jList(2);
	float sf = (bodyA.staticFriction+bodyB.staticFriction)/2.f;
	float df = (bodyA.dynamicFriction + bodyB.dynamicFriction) / 2.f;
	for (size_t i = 0; i < contactCount; i++) {
		Vector2f ra = contactList[i] - bodyA.position;
		Vector2f rb = contactList[i] - bodyB.position;
		raList[i] = ra;
		rbList[i] = rb;
		Vector2f raPerp(-ra.y, ra.x);
		Vector2f rbPerp(-rb.y, rb.x);
		Vector2f angularLinearVelocityA = raPerp * bodyA.angularVelocity;
		Vector2f angularLinearVelocityB = rbPerp * bodyB.angularVelocity;
		Vector2f relativeVelocity = (bodyB.linearVelocity + angularLinearVelocityB) - (bodyA.linearVelocity + angularLinearVelocityA);
		float contactVelocityMag = relativeVelocity.dot(normal);
		if (contactVelocityMag > 0.f) {
			continue;
		}
		float j = -(1 + e) * contactVelocityMag;
		float raPerpDotN = raPerp.dot(normal);
		float rbPerpDotN = rbPerp.dot(normal);
		float denom = bodyA.inverseMass + bodyB.inverseMass +
			(raPerpDotN * raPerpDotN) * bodyA.inverseInertia +
			(rbPerpDotN * rbPerpDotN) * bodyB.inverseInertia;
		j /= denom;
		j /= (float)contactCount;
		jList[i] = j;
		Vector2f impulse = j * normal;
		impulseList[i] = impulse;
	}
	for (size_t i = 0; i < contactCount; i++) {
		Vector2f ra = raList[i];
		Vector2f rb = rbList[i];
		Vector2f impulse = impulseList[i];
		bodyA.linearVelocity += -impulse * bodyA.inverseMass;
		bodyA.angularVelocity += -ra.cross(impulse) * bodyA.inverseInertia;
		bodyB.linearVelocity += impulse * bodyB.inverseMass;
		bodyB.angularVelocity += rb.cross(impulse) * bodyB.inverseInertia;
	}

	for (size_t i = 0; i < contactCount; i++) {
		Vector2f ra = contactList[i] - bodyA.position;
		Vector2f rb = contactList[i] - bodyB.position;
		raList[i] = ra;
		rbList[i] = rb;
		Vector2f raPerp(-ra.y, ra.x);
		Vector2f rbPerp(-rb.y, rb.x);
		Vector2f angularLinearVelocityA = raPerp * bodyA.angularVelocity;
		Vector2f angularLinearVelocityB = rbPerp * bodyB.angularVelocity;
		Vector2f relativeVelocity = (bodyB.linearVelocity + angularLinearVelocityB) - (bodyA.linearVelocity + angularLinearVelocityA);
		Vector2f tangent = relativeVelocity - relativeVelocity.dot(normal) * normal;
		if (Collisions::NearlyEqual(tangent, Vector2f(0, 0))) {
			continue;
		}
		else {
			tangent = tangent.normalized();
		}
		float jt = -relativeVelocity.dot(tangent);
		float raPerpDotT = raPerp.dot(tangent);
		float rbPerpDotT = rbPerp.dot(tangent);
		float denom = bodyA.inverseMass + bodyB.inverseMass +
			(raPerpDotT * raPerpDotT) * bodyA.inverseInertia +
			(rbPerpDotT * rbPerpDotT) * bodyB.inverseInertia;
		jt /= denom;
		jt /= (float)contactCount;
		Vector2f frictionImpulse;
		float j = jList[i];
		if (abs(jt) <= j * sf) {
			frictionImpulse = jt * tangent;
		}
		else {
			frictionImpulse = -j * tangent * df;
		}
		frictionImpulseList[i] = frictionImpulse;
	}
	for (size_t i = 0; i < contactCount; i++) {
		Vector2f ra = raList[i];
		Vector2f rb = rbList[i];
		Vector2f frictionImpulse = frictionImpulseList[i];
		bodyA.linearVelocity += -frictionImpulse * bodyA.inverseMass;
		bodyA.angularVelocity += -ra.cross(frictionImpulse) * bodyA.inverseInertia;
		bodyB.linearVelocity += frictionImpulse * bodyB.inverseMass;
		bodyB.angularVelocity += rb.cross(frictionImpulse) * bodyB.inverseInertia;
	}
}