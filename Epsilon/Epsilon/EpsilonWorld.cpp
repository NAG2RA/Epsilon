#include "EpsilonWorld.h"
class QuadTree {
public:
	int nodeCapacity;
	AABB aabb;
	vector<EpsilonBody*> bodies;
	std::unique_ptr<QuadTree> nw, ne, sw, se;

	QuadTree(AABB ab, int capacity)
		:
		aabb(ab.min.x, ab.max.x, ab.min.y, ab.max.y),
		nodeCapacity(capacity)
	{
	}

	void query(AABB area, vector<EpsilonBody*>& found) {
		if (!Collisions::IntersectAABB(area, aabb)) {
			return;
		}

		if (divided) {
			nw->query(area, found);
			ne->query(area, found);
			sw->query(area, found);
			se->query(area, found);
		}
		for (int i = 0; i < bodies.size(); i++) {
			if (Collisions::IntersectAABB(area, bodies[i]->GetAABB())) {
				found.push_back(bodies[i]);
			}
		}
	}

	bool insert(EpsilonBody* body) {
		if (!Collisions::IntersectAABB(aabb, body->GetAABB())) {
			return false;
		}

		if (!divided) {
			if (bodies.size() < nodeCapacity) {
				bodies.push_back(body);
				return true;
			}
			subdivide();
		}

		if (Collisions::ContainsAABB(body->GetAABB(), nw->aabb)) return nw->insert(body);
		if (Collisions::ContainsAABB(body->GetAABB(), ne->aabb)) return ne->insert(body);
		if (Collisions::ContainsAABB(body->GetAABB(), sw->aabb)) return sw->insert(body);
		if (Collisions::ContainsAABB(body->GetAABB(), se->aabb)) return se->insert(body);
		bodies.push_back(body);
		return true;
	}

private:
	bool divided = false;
	void subdivide() {
		float midX = (aabb.min.x + aabb.max.x) / 2.f;
		float midY = (aabb.min.y + aabb.max.y) / 2.f;

		nw = make_unique<QuadTree>(AABB(aabb.min.x, midX, aabb.min.y, midY), nodeCapacity);
		ne = make_unique<QuadTree>(AABB(midX, aabb.max.x, aabb.min.y, midY), nodeCapacity);
		sw = make_unique<QuadTree>(AABB(aabb.min.x, midX, midY, aabb.max.y), nodeCapacity);
		se = make_unique<QuadTree>(AABB(midX, aabb.max.x, midY, aabb.max.y), nodeCapacity);

		divided = true;
		vector<EpsilonBody*> parentBodies = move(bodies);
		bodies.clear();

		for (EpsilonBody* bd : parentBodies) {
			insert(bd);
		}
	}
};
EpsilonWorld::EpsilonWorld(int windowWidth, int windowHeight, float zoom)
	:depth(0),
	normal(0,0),
	gravity(0,9.81f),
	springConstant(20),
	damperConstant(1),
	damperThreadConstant(2),
	damperWaterConstant(1),
	airResistanceConstant(0.01f),
	rotationalAirResistanceConstant(0.01f),
	windowWidth(windowWidth),
	windowHeight(windowHeight),
	zoom(zoom)
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

int EpsilonWorld::GetDynamicBodyCount()
{
	return dynamicBodyList.size();
}

EpsilonBody EpsilonWorld::GetBody(float index)
{
	return bodyList[index];
}

Water EpsilonWorld::GetWater(int index)
{
	return waterList[index];
}

EpsilonBody* EpsilonWorld::GetDynamicBody(float index)
{
	return dynamicBodyList[index];
}

int EpsilonWorld::GetBodyCount()
{
	return bodyList.size();
}

int EpsilonWorld::GetWaterCount()
{
	return waterList.size();
}

void EpsilonWorld::Update(float dt, int iterations)
{
	if (iterations < 1) {
		iterations = 1;
	}
	else if (iterations > 32) {
		iterations = 32;
	}
	dynamicBodyList.clear();
	highPriorityObjects.clear();
	lowPriorityObjects.clear();
	PreFiltering();
	UpdateMovement(dt, 1);
	ResolveSpringConnection(dt, 1);
	ResolveThreadConnection();
	for (int it = 0; it < iterations / 2; it++) {
		ResolveHighPrioritySpringConnection(dt,iterations/2.f);
		UpdateHighPriorityMovement(dt, iterations / 2.f);
		ResolveHighPriorityThreadConnection();
		
	}
	for (int it = 0; it < iterations; it++) {
		contactPairs.clear();
		BroadPhase(windowWidth, windowHeight, zoom);
		NarrowPhase();
	}
	
	AirResistance(dt,1);
	Buoyancy();
}


void EpsilonWorld::SeperateBodies(EpsilonBody& bodyA, EpsilonBody& bodyB, EpsilonVector mtv) {
	bodyA.Move((-mtv / 2.f) * !bodyA.isStatic);
	bodyB.Move((mtv/2.f) * !bodyB.isStatic);
}

void EpsilonWorld::PreFiltering()
{
	
	for (int i = 0; i < bodyList.size(); i++) {
		if(!bodyList[i].isStatic) {
			dynamicBodyList.push_back(&bodyList[i]);
			if (bodyList[i].linearVelocity.LengthSquared() > 100.f) {
				highPriorityObjects.push_back(&bodyList[i]);
			}
			else {
				lowPriorityObjects.push_back(&bodyList[i]);
			}
		}
	}
}

void EpsilonWorld::BroadPhase(int windowWidth, int windowHeight, float zoom) {
		if (dynamicBodyList.size() == 0) {
			return;
		}
		float offsetX((windowWidth - windowWidth * zoom) / 2.f);
		float offsetY((windowHeight - windowHeight * zoom) / 2.f);
		QuadTree qtree(AABB(offsetX, windowWidth * zoom + offsetX, offsetY, windowHeight * zoom + offsetY), 50);
		for (int i = 0; i < bodyList.size(); i++) {
			qtree.insert(&bodyList[i]);
		}
		for (int i = 0; i < dynamicBodyList.size(); i++) {
			potentialColliders.clear();
			qtree.query(dynamicBodyList[i]->GetAABB(), potentialColliders);
			for (int j = 0; j < potentialColliders.size(); j++) {
				if (dynamicBodyList[i] == potentialColliders[j]) {
					continue;
				}

				vector<EpsilonBody*> t = { dynamicBodyList[i], potentialColliders[j] };
				contactPairs.push_back(t);

			}
		}
	
}

void EpsilonWorld::NarrowPhase() {
	for (int i = 0; i < contactPairs.size(); i++) {
		vector<EpsilonBody*> t = contactPairs[i];
		EpsilonBody& bodyA = *t[0];
		EpsilonBody& bodyB = *t[1];
		if (Collisions::Collide(bodyA, bodyB, normal, depth)) {
			SeperateBodies(bodyA, bodyB, normal * depth);
			EpsilonVector contact1(0, 0);
			EpsilonVector contact2(0, 0);
			int contactCount = 0;
			Collisions::FindContactPoints(bodyA, bodyB, contact1, contact2, contactCount);
			CollisionManifold contact(bodyA, bodyB, contact1, contact2, normal, depth, contactCount);
			ResolveCollisonWithRotationAndFriction(contact);
		}
	}
}

void EpsilonWorld::UpdateMovement(float dt,int iterations) {
	for (int i = 0; i < lowPriorityObjects.size(); i++) {
		lowPriorityObjects[i]->updateMovement(dt, gravity, iterations);
	}
	
}

void EpsilonWorld::UpdateHighPriorityMovement(float dt, int iterations)
{
	for (int i = 0; i < highPriorityObjects.size(); i++) {
		highPriorityObjects[i]->updateMovement(dt, gravity, iterations);
	}
}

void EpsilonWorld::ResolveCollisonBasic(CollisionManifold& manifold)
{
	EpsilonBody& bodyA = manifold.bodyA;
	EpsilonBody& bodyB = manifold.bodyB;
	EpsilonVector normal = manifold.normal;
	float depth = manifold.depth;
	EpsilonVector relativeVelocity = bodyB.linearVelocity - bodyA.linearVelocity;
	if (relativeVelocity.Dot(normal) > 0.f) {
		return;
	}
	float e = max(bodyA.restitution, bodyB.restitution);
	float j = -(1 + e) * relativeVelocity.Dot(normal);
	j /= bodyA.inverseMass+bodyB.inverseMass;
	EpsilonVector impulse = j * normal;
	bodyA.linearVelocity -= impulse * bodyA.inverseMass;
	bodyB.linearVelocity += impulse * bodyB.inverseMass;	
}

void EpsilonWorld::ResolveCollisonWithRotation(CollisionManifold& manifold)
{
	EpsilonBody& bodyA = manifold.bodyA;
	EpsilonBody& bodyB = manifold.bodyB;
	EpsilonVector normal = manifold.normal;
	EpsilonVector contact1 = manifold.contact1;
	EpsilonVector contact2 = manifold.contact2;
	int contactCount = manifold.contactCount;
	float e = max(bodyA.restitution, bodyB.restitution);
	vector<EpsilonVector> contactList = { contact1,contact2 };
	vector<EpsilonVector> impulseList(2);
	vector<EpsilonVector> raList(2);
	vector<EpsilonVector> rbList(2);
	for (size_t i = 0; i < contactCount; i++) {
		EpsilonVector ra = contactList[i] - bodyA.position;
		EpsilonVector rb = contactList[i] - bodyB.position;
		raList[i] = ra;
		rbList[i] = rb;
		EpsilonVector raPerp(-ra.y, ra.x);
		EpsilonVector rbPerp(-rb.y, rb.x);
		EpsilonVector angularLinearVelocityA = raPerp * bodyA.angularVelocity;
		EpsilonVector angularLinearVelocityB = rbPerp * bodyB.angularVelocity;
		EpsilonVector relativeVelocity = (bodyB.linearVelocity+angularLinearVelocityB) - (bodyA.linearVelocity+angularLinearVelocityA);
		float contactVelocityMag = relativeVelocity.Dot(normal);
		if (contactVelocityMag > 0.f) {
			continue;
		}
		float j = -(1 + e) * contactVelocityMag;
		float raPerpDotN = raPerp.Dot(normal);
		float rbPerpDotN = rbPerp.Dot(normal);
		float denom = bodyA.inverseMass + bodyB.inverseMass + 
			(raPerpDotN*raPerpDotN)*bodyA.inverseInertia + 
			(rbPerpDotN * rbPerpDotN) * bodyB.inverseInertia;
		j /= denom;
		j /= (float)contactCount;
		EpsilonVector impulse = j * normal;
		impulseList[i] = impulse;
	}
	for (size_t i = 0; i < contactCount; i++) {
		EpsilonVector ra = raList[i];
		EpsilonVector rb = rbList[i];
		EpsilonVector impulse = impulseList[i];
		bodyA.linearVelocity += -impulse * bodyA.inverseMass;
		bodyA.angularVelocity += -ra.Cross(impulse) * bodyA.inverseInertia;
		bodyB.linearVelocity += impulse * bodyB.inverseMass;
		bodyB.angularVelocity += rb.Cross(impulse) * bodyB.inverseInertia;
	}
}

void EpsilonWorld::ResolveCollisonWithRotationAndFriction(CollisionManifold& manifold)
{
	EpsilonBody& bodyA = manifold.bodyA;
	EpsilonBody& bodyB = manifold.bodyB;
	EpsilonVector normal = manifold.normal;
	EpsilonVector contact1 = manifold.contact1;
	EpsilonVector contact2 = manifold.contact2;
	int contactCount = manifold.contactCount;
	float e = max(bodyA.restitution, bodyB.restitution);
	vector<EpsilonVector> contactList = { contact1,contact2 };
	vector<EpsilonVector> impulseList(2);
	vector<EpsilonVector> raList(2);
	vector<EpsilonVector> rbList(2);
	vector<EpsilonVector> frictionImpulseList(2);
	vector<float> jList(2);
	float sf = (bodyA.staticFriction + bodyB.staticFriction) / 2.f;
	float df = (bodyA.dynamicFriction + bodyB.dynamicFriction) / 2.f;
	for (size_t i = 0; i < contactCount; i++) {
		EpsilonVector ra = contactList[i] - bodyA.position;
		EpsilonVector rb = contactList[i] - bodyB.position;
		raList[i] = ra;
		rbList[i] = rb;
		EpsilonVector raPerp(-ra.y, ra.x);
		EpsilonVector rbPerp(-rb.y, rb.x);
		EpsilonVector angularLinearVelocityA = raPerp * bodyA.angularVelocity;
		EpsilonVector angularLinearVelocityB = rbPerp * bodyB.angularVelocity;
		EpsilonVector relativeVelocity = (bodyB.linearVelocity + angularLinearVelocityB) - (bodyA.linearVelocity + angularLinearVelocityA);
		float contactVelocityMag = relativeVelocity.Dot(normal);
		if (contactVelocityMag > 0.f) {
			continue;
		}
		float j = -(1 + e) * contactVelocityMag;
		float raPerpDotN = raPerp.Dot(normal);
		float rbPerpDotN = rbPerp.Dot(normal);
		float denom = bodyA.inverseMass + bodyB.inverseMass +
			(raPerpDotN * raPerpDotN * bodyA.inverseInertia) +
			(rbPerpDotN * rbPerpDotN * bodyB.inverseInertia);
		j /= denom;
		j /= (float)contactCount;
		jList[i] = j;
		EpsilonVector impulse = j * normal;
		impulseList[i] = impulse;
	}
	for (size_t i = 0; i < contactCount; i++) {
		EpsilonVector ra = raList[i];
		EpsilonVector rb = rbList[i];
		EpsilonVector impulse = impulseList[i];
		bodyA.linearVelocity += -impulse * bodyA.inverseMass;
		bodyA.angularVelocity += -ra.Cross(impulse) * bodyA.inverseInertia;
		bodyB.linearVelocity += impulse * bodyB.inverseMass;
		bodyB.angularVelocity += rb.Cross(impulse) * bodyB.inverseInertia;
	}

	for (size_t i = 0; i < contactCount; i++) {
		EpsilonVector ra = contactList[i] - bodyA.position;
		EpsilonVector rb = contactList[i] - bodyB.position;
		raList[i] = ra;
		rbList[i] = rb;
		EpsilonVector raPerp(-ra.y, ra.x);
		EpsilonVector rbPerp(-rb.y, rb.x);
		EpsilonVector angularLinearVelocityA = raPerp * bodyA.angularVelocity;
		EpsilonVector angularLinearVelocityB = rbPerp * bodyB.angularVelocity;
		EpsilonVector relativeVelocity = (bodyB.linearVelocity + angularLinearVelocityB) - (bodyA.linearVelocity + angularLinearVelocityA);
		EpsilonVector tangent = relativeVelocity - relativeVelocity.Dot(normal) * normal;
		if (Collisions::NearlyEqual(tangent, EpsilonVector(0, 0))) {
			continue;
		}
		else {
			tangent = tangent.Normalized();
		}
		float jt = -relativeVelocity.Dot(tangent);
		float raPerpDotT = raPerp.Dot(tangent);
		float rbPerpDotT = rbPerp.Dot(tangent);
		float denom = (bodyA.inverseMass + bodyB.inverseMass) +
			((raPerpDotT * raPerpDotT) * bodyA.inverseInertia) +
			((rbPerpDotT * rbPerpDotT) * bodyB.inverseInertia);
		jt /= denom;
		jt /= (float)contactCount;
		EpsilonVector frictionImpulse;
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
		EpsilonVector ra = raList[i];
		EpsilonVector rb = rbList[i];
		EpsilonVector frictionImpulse = frictionImpulseList[i];
		bodyA.linearVelocity += -frictionImpulse * bodyA.inverseMass;
		bodyA.angularVelocity += -ra.Cross(frictionImpulse) * bodyA.inverseInertia;
		bodyB.linearVelocity += frictionImpulse * bodyB.inverseMass;
		bodyB.angularVelocity += rb.Cross(frictionImpulse) * bodyB.inverseInertia;
	}
}
void EpsilonWorld::ZoZoResolveCollisonBasic(CollisionManifold& manifold)
{
	EpsilonBody& bodyA = manifold.bodyA;
	EpsilonBody& bodyB = manifold.bodyB;
	EpsilonVector normal = manifold.normal;
	float g0 = 0.001f;
	EpsilonVector n = manifold.contact2 - manifold.contact1;
	n = n.Normalized();
	float g = n.Dot(manifold.contact2 - manifold.contact1);
	g = max(g, g0 * 0.1f);
	float diff = (g - g0) / g0;
	float mass = 1.0f / (bodyA.inverseMass + bodyB.inverseMass);
	float k = mass / (g * g);
	float e = min(bodyA.restitution, bodyB.restitution);
	if (g < g0) {
		float f = -(1+e) * k * diff * diff;
	EpsilonVector impulse = f*normal;
	bodyA.linearVelocity -= impulse;
	bodyB.linearVelocity += impulse;

	}
	
}
void EpsilonWorld::ResolveThreadConnection() {
	for (int i = 0; i < lowPriorityObjects.size(); i++) {
		if (lowPriorityObjects[i]->connectiontype == none|| lowPriorityObjects[i]->connectiontype == spring) {
			continue;
		}
		else if (lowPriorityObjects[i]->originPosition.Distance(lowPriorityObjects[i]->connectionPosition) > lowPriorityObjects[i]->connectionDistance) {
			EpsilonVector dir = lowPriorityObjects[i]->connectionPosition - lowPriorityObjects[i]->originPosition;
			float dist = dir.Length();
			float restDist = dist - lowPriorityObjects[i]->connectionDistance;
			if (restDist > 0.01f) {
				restDist = 0.01f;
			}
			float damperForce = -(damperThreadConstant * lowPriorityObjects[i]->linearVelocity.Dot(dir)) / dist;
			lowPriorityObjects[i]->linearVelocity += -dir * (lowPriorityObjects[i]->inverseMass*restDist);
			lowPriorityObjects[i]->AddForce(damperForce * dir);
			EpsilonVector offset = lowPriorityObjects[i]->connectionPosition - lowPriorityObjects[i]->position;
			lowPriorityObjects[i]->angularVelocity += offset.Cross(-dir * (lowPriorityObjects[i]->inverseMass * restDist));
		}
	}
}
void EpsilonWorld::ResolveSpringConnection(float dt, int iterations) {
	dt = dt/iterations;
	for (int i = 0; i < lowPriorityObjects.size(); i++) {
		if (lowPriorityObjects[i]->connectiontype == none || lowPriorityObjects[i]->connectiontype == thr) {
			continue;
		}
		
		EpsilonVector dir = lowPriorityObjects[i]->connectionPosition - lowPriorityObjects[i]->originPosition;
		float dist = dir.Length();
		float restDist = dist - lowPriorityObjects[i]->connectionDistance;
		float springForce = restDist * springConstant;
		
		float damperForce = (damperConstant * (lowPriorityObjects[i]->linearVelocity.Dot(dir))/dist);
		EpsilonVector force = -(springForce+damperForce)*dir/dist;
		EpsilonVector offset = lowPriorityObjects[i]->connectionPosition - lowPriorityObjects[i]->position;
		float angularResistance = offset.Cross(force*lowPriorityObjects[i]->inverseInertia);
		lowPriorityObjects[i]->angularVelocity += angularResistance* lowPriorityObjects[i]->inverseMass* dt;
		lowPriorityObjects[i]->AddForce(force);
	}
}
void EpsilonWorld::ResolveHighPriorityThreadConnection()
{
	for (int i = 0; i < highPriorityObjects.size(); i++) {
		if (highPriorityObjects[i]->connectiontype == none || highPriorityObjects[i]->connectiontype == spring) {
			continue;
		}
		else if (highPriorityObjects[i]->originPosition.Distance(highPriorityObjects[i]->connectionPosition) > highPriorityObjects[i]->connectionDistance) {
			EpsilonVector dir = highPriorityObjects[i]->connectionPosition - highPriorityObjects[i]->originPosition;
			float dist = dir.Length();
			float restDist = dist - highPriorityObjects[i]->connectionDistance;
			if (restDist > 0.01f) {
				restDist = 0.01f;
			}
			float damperForce = -(damperThreadConstant * highPriorityObjects[i]->linearVelocity.Dot(dir)) / dist;
			highPriorityObjects[i]->linearVelocity += -dir * (highPriorityObjects[i]->inverseMass * restDist);
			highPriorityObjects[i]->AddForce(damperForce * dir);
			EpsilonVector offset = highPriorityObjects[i]->connectionPosition - highPriorityObjects[i]->position;
			highPriorityObjects[i]->angularVelocity += offset.Cross(-dir * (highPriorityObjects[i]->inverseMass * restDist));
		}
	}
}
void EpsilonWorld::ResolveHighPrioritySpringConnection(float dt, int iterations)
{
	dt = dt / iterations;
	for (int i = 0; i < highPriorityObjects.size(); i++) {
		if (highPriorityObjects[i]->connectiontype == none || highPriorityObjects[i]->connectiontype == thr) {
			continue;
		}

		EpsilonVector dir = highPriorityObjects[i]->connectionPosition - highPriorityObjects[i]->originPosition;
		float dist = dir.Length();
		float restDist = dist - highPriorityObjects[i]->connectionDistance;
		float springForce = restDist * springConstant;
		float damperForce = (damperConstant * (highPriorityObjects[i]->linearVelocity.Dot(dir)) / dist);
		EpsilonVector force = -(springForce + damperForce) * dir / dist;
		EpsilonVector offset = highPriorityObjects[i]->connectionPosition - highPriorityObjects[i]->position;
		float angularResistance = offset.Cross(force * highPriorityObjects[i]->inverseInertia);
		highPriorityObjects[i]->angularVelocity += angularResistance* highPriorityObjects[i]->inverseMass* dt;
		highPriorityObjects[i]->AddForce(force);
	}
}
void EpsilonWorld::Explosion(EpsilonVector position, float radius, float magnitude)
{
	for (int i = 0; i < dynamicBodyList.size(); i++) {
		if (dynamicBodyList[i]->position.Distance(position) > radius) {
			continue;
		}
		EpsilonVector dir = dynamicBodyList[i]->position - position;
		float dist = dir.Length();
		EpsilonVector impulse = (dir * magnitude) / (dist*dist);
		EpsilonVector vertical(0, 1.f);
		float mag = -vertical.Cross(impulse);
		dynamicBodyList[i]->linearVelocity += impulse * dynamicBodyList[i]->inverseMass;
		dynamicBodyList[i]->angularVelocity += mag * dynamicBodyList[i]->inverseInertia;
	}
}
void EpsilonWorld::Buoyancy() {
	for (int j = 0; j < waterList.size(); j++) {
		for (int i = 0; i < dynamicBodyList.size(); i++) {
			Water& w = waterList[j];
			if (dynamicBodyList[i]->shapetype == box) {
				if (dynamicBodyList[i]->position.y + dynamicBodyList[i]->height / 2.f > w.surfacePosition.y && dynamicBodyList[i]->position.x < w.surfacePosition.x + (w.width / 2.f) && dynamicBodyList[i]->position.x > w.surfacePosition.x - (w.width / 2.f) && dynamicBodyList[i]->position.y < w.surfacePosition.y + w.depth) {
					EpsilonVector dir(0, 1.f);
					float h = dynamicBodyList[i]->position.y + (dynamicBodyList[i]->height / 2.f) - w.surfacePosition.y;
					if (h > dynamicBodyList[i]->height) {
						h = dynamicBodyList[i]->height;
					}
					float damperForce = (damperWaterConstant * dynamicBodyList[i]->linearVelocity.Dot(dir)) * h;
					EpsilonVector force = -dir * ((dynamicBodyList[i]->width * h * 9.81f * w.density) + damperForce);
					dynamicBodyList[i]->AddForce(force);
				}
			}
			else if (dynamicBodyList[i]->shapetype == circle) {
				if (dynamicBodyList[i]->position.y + dynamicBodyList[i]->radius > w.surfacePosition.y && dynamicBodyList[i]->position.x < w.surfacePosition.x + w.width / 2.f && dynamicBodyList[i]->position.x > w.surfacePosition.x - w.width / 2.f && dynamicBodyList[i]->position.y < w.surfacePosition.y + w.depth) {
					EpsilonVector dir(0, 1.f);
					float h = dynamicBodyList[i]->position.y + dynamicBodyList[i]->radius - w.surfacePosition.y;
					if (h > dynamicBodyList[i]->radius * 2.f) {
						h = dynamicBodyList[i]->radius * 2.f;
					}
					float r = dynamicBodyList[i]->radius;
					float area = r * r * acos(1 - (h / r)) - (r - h) * sqrt(r * r - (r - h) * (r - h));
					float damperForce = (damperWaterConstant * dynamicBodyList[i]->linearVelocity.Dot(dir)) * h;
					EpsilonVector force = -dir * ((area * 9.81f * w.density) + damperForce);
					dynamicBodyList[i]->AddForce(force);
				}
			}
			else if (dynamicBodyList[i]->shapetype == triangle) {
				if (dynamicBodyList[i]->position.y + dynamicBodyList[i]->height / 3.f > w.surfacePosition.y && dynamicBodyList[i]->position.x < w.surfacePosition.x + w.width / 2.f && dynamicBodyList[i]->position.x > w.surfacePosition.x - w.width / 2.f && dynamicBodyList[i]->position.y < w.surfacePosition.y + w.depth) {
					EpsilonVector dir(0, 1.f);
					float h = dynamicBodyList[i]->position.y + dynamicBodyList[i]->height / 3.f - w.surfacePosition.y;
					if (h > dynamicBodyList[i]->height) {
						h = dynamicBodyList[i]->height;
					}
					float hTriangle = dynamicBodyList[i]->height - h;
					float sideTriangle = (hTriangle * dynamicBodyList[i]->width) / dynamicBodyList[i]->height;
					float area = (sideTriangle + dynamicBodyList[i]->width) * h / 2;
					float damperForce = (damperWaterConstant * dynamicBodyList[i]->linearVelocity.Dot(dir)) * h;
					EpsilonVector force = -dir * ((area * 9.81f * w.density) + damperForce);
					dynamicBodyList[i]->AddForce(force);
				}
			}
		}
	}
}

void EpsilonWorld::CreateWater(EpsilonVector surfacePosition, float width, float depth, float density)
{
	waterList.push_back(Water(surfacePosition, density, width, depth));
}

void EpsilonWorld::DeleteWater(int index)
{
	waterList.erase(waterList.begin() + index);
}

void EpsilonWorld::AirResistance(float dt, int iterations)
{
	dt/=iterations;
	for (int i = 0; i < dynamicBodyList.size(); i++) {
		EpsilonVector resistance(dynamicBodyList[i]->linearVelocity.x * dynamicBodyList[i]->linearVelocity.x, dynamicBodyList[i]->linearVelocity.y * dynamicBodyList[i]->linearVelocity.y);
		resistance = -resistance * airResistanceConstant;
		float angularResistance = -abs(dynamicBodyList[i]->angularVelocity) * dynamicBodyList[i]->angularVelocity * rotationalAirResistanceConstant;
		if (angularResistance != 0) {
			dynamicBodyList[i]->angularVelocity += (angularResistance *dynamicBodyList[i]->inverseInertia) * dt;
		}
		
		dynamicBodyList[i]->AddForce(resistance);
	}
}
