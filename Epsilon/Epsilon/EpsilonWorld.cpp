#include "EpsilonWorld.h"
class QuadTree {
public:
	int nodeCapacity;
	AABB aabb;
	vector<EpsilonBody*> bodies;
	std::unique_ptr<QuadTree> nw, ne, sw, se;

	QuadTree(AABB ab, int capacity = 5)
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
		else {
			for (int i = 0; i < bodies.size(); i++) {
				if (Collisions::IntersectAABB(area, bodies[i]->GetAABB())) {
					found.push_back(bodies[i]);
				}
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

		bool inserted_nw = nw->insert(body);
		bool inserted_ne = ne->insert(body);
		bool inserted_sw = sw->insert(body);
		bool inserted_se = se->insert(body);

		return inserted_nw || inserted_ne || inserted_sw || inserted_se;
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

		for (EpsilonBody* bd : bodies) {
			nw->insert(bd);
			ne->insert(bd);
			sw->insert(bd);
			se->insert(bd);
		}
		bodies.clear();
	}
};
EpsilonWorld::EpsilonWorld()
	:depth(0),
	normal(0,0),
	gravity(0,9.81f),
	springConstant(50),
	damperConstant(5),
	damperThreadConstant(25),
	airResistanceConstant(0.01f),
	rotationalAirResistanceConstant(0.01f),
	waterDensity(0),
	waterDepth(0),
	waterWidth(0),
	waterSurfacePosition(0,0)
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

EpsilonBody EpsilonWorld::GetBody(float index)
{
	return bodyList[index];
}

int EpsilonWorld::GetBodyCount()
{
	return bodyList.size();
}

void EpsilonWorld::Update(float dt, int iterations)
{
	if (iterations < 1) {
		iterations = 1;
	}
	else if (iterations > 32) {
		iterations = 32;
	}
	PreFiltering();
	for (int it = 0; it < iterations / 2; it++) {
		UpdateMovement(dt, iterations/2);
	}
	for (int it = 0; it < iterations; it++) {
		contactPairs.clear();
		BroadPhase(1280, 720, 0.2f);
		NarrowPhase();
	}
	ResolveThreadConnection();
	ResolveSpringConnection(dt);
	AirResistance(dt);
	Buoyancy(waterSurfacePosition, waterWidth, waterDepth, waterDensity);
}


void EpsilonWorld::SeperateBodies(EpsilonBody& bodyA, EpsilonBody& bodyB, EpsilonVector mtv) {
	bodyA.Move((-mtv / 2.f) * !bodyA.isStatic);
	bodyB.Move((mtv/2.f) * !bodyB.isStatic);
}

void EpsilonWorld::PreFiltering()
{
	dynamicBodyList.clear();
	for (int i = 0; i < bodyList.size(); i++) {
		if(!bodyList[i].isStatic) {
			dynamicBodyList.push_back(&bodyList[i]);
		}
	}
}

void EpsilonWorld::BroadPhase(int windowWidth, int windowHeight, float zoom) {
		if (dynamicBodyList.size() == 0) {
			return;
		}
		float offsetX((windowWidth - windowWidth * zoom) / 2.f);
		float offsetY((windowHeight - windowHeight * zoom) / 2.f);
		unique_ptr<QuadTree> qtree = make_unique<QuadTree>(AABB(offsetX, windowWidth * zoom + offsetX, offsetY, windowHeight * zoom + offsetY), 30);
		for (int i = 0; i < bodyList.size(); i++) {
			qtree->insert(&bodyList[i]);
		}
		for (int i = 0; i < dynamicBodyList.size(); i++) {
			potentialColliders.clear();
			qtree->query(dynamicBodyList[i]->GetAABB(), potentialColliders);
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
	for (int i = 0; i < dynamicBodyList.size(); i++) {
		dynamicBodyList[i]->updateMovement(dt, gravity, iterations);
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
	for (size_t i = 0; i < bodyList.size(); i++) {
		if (bodyList[i].connectiontype == none|| bodyList[i].connectiontype == spring) {
			continue;
		}
		else if (bodyList[i].originPosition.Distance(bodyList[i].connectionPosition) > bodyList[i].connectionDistance) {
			EpsilonVector dir = bodyList[i].connectionPosition - bodyList[i].originPosition;
			float dist = dir.Length();
			float restDist = dist - bodyList[i].connectionDistance;
			float damperForce = -(damperThreadConstant * bodyList[i].linearVelocity.Dot(dir)) / dist;
			bodyList[i].linearVelocity += -dir * (bodyList[i].inverseMass*restDist);
			bodyList[i].AddForce(damperForce * dir);
			EpsilonVector offset = bodyList[i].connectionPosition - bodyList[i].position;
			bodyList[i].angularVelocity += offset.Cross(-dir * (bodyList[i].inverseMass * restDist));
		}
	}
}
void EpsilonWorld::ResolveSpringConnection(float dt) {
	for (size_t i = 0; i < bodyList.size(); i++) {
		if (bodyList[i].connectiontype == none || bodyList[i].connectiontype == thr) {
			continue;
		}
		
		EpsilonVector dir = bodyList[i].connectionPosition - bodyList[i].originPosition;
		float dist = dir.Length();
		float restDist = dist - bodyList[i].connectionDistance;
		float springForce = -restDist * springConstant;
		float damperForce = -(damperConstant * bodyList[i].linearVelocity.Dot(dir))/dist;
		EpsilonVector force = (springForce+damperForce)*dir/dist;
		bodyList[i].AddForce(force);
		EpsilonVector offset = bodyList[i].connectionPosition - bodyList[i].position;
		springForce = -abs(restDist) * springConstant;
		force = (springForce + damperForce) * dir / dist;
		float angularResistance = offset.Cross(force);
		bodyList[i].angularVelocity += (angularResistance * bodyList[i].inverseInertia*damperConstant/springConstant) * dt;
	}
}
void EpsilonWorld::Explosion(EpsilonVector position, float radius, float magnitude)
{
	for (size_t i = 0; i < bodyList.size(); i++) {
		if (bodyList[i].position.Distance(position) > radius) {
			continue;
		}
		EpsilonVector dir = bodyList[i].position - position;
		float dist = dir.Length();
		EpsilonVector impulse = (dir * magnitude) / (dist*dist);
		EpsilonVector vertical(0, 1.f);
		float mag = -vertical.Cross(impulse);
		bodyList[i].linearVelocity += impulse * bodyList[i].inverseMass;
		bodyList[i].angularVelocity += mag * bodyList[i].inverseInertia;
	}
}
void EpsilonWorld::Buoyancy(EpsilonVector surfacePosition,float width, float depth, float density) {
	for (size_t i = 0; i < bodyList.size(); i++) {
		if (bodyList[i].shapetype == box) {
			if (bodyList[i].position.y + bodyList[i].height / 2.f > surfacePosition.y && bodyList[i].position.x < surfacePosition.x+(width/2.f) && bodyList[i].position.x > surfacePosition.x - (width / 2.f) && bodyList[i].position.y < surfacePosition.y + depth) {
				EpsilonVector dir(0, 1.f);
				float h = bodyList[i].position.y + (bodyList[i].height/2.f) - surfacePosition.y;
				if (h > bodyList[i].height) {
					h = bodyList[i].height;
				}
				float damperForce = (damperConstant*bodyList[i].linearVelocity.Dot(dir))*h;
				EpsilonVector force = -dir * ((bodyList[i].width * h * 9.81f * density)+damperForce);
				bodyList[i].AddForce(force);
			}
		}
		else if(bodyList[i].shapetype == circle){
			if (bodyList[i].position.y + bodyList[i].radius > surfacePosition.y && bodyList[i].position.x < surfacePosition.x + width / 2.f && bodyList[i].position.x > surfacePosition.x - width / 2.f && bodyList[i].position.y < surfacePosition.y + depth) {
				EpsilonVector dir(0, 1.f);
				float h = bodyList[i].position.y + bodyList[i].radius - surfacePosition.y;
				if (h > bodyList[i].radius*2.f) {
					h = bodyList[i].radius*2.f;
				}
				float r = bodyList[i].radius;
				float area = r * r * acos(1 - (h / r)) - (r - h) * sqrt(r * r - (r - h) * (r - h));
				float damperForce = (damperConstant * bodyList[i].linearVelocity.Dot(dir)) * h;
				EpsilonVector force = -dir * ((area * 9.81f * density) + damperForce);
				bodyList[i].AddForce(force);
			}
		}
		else if (bodyList[i].shapetype == triangle) {
			if (bodyList[i].position.y + bodyList[i].height / 3.f > surfacePosition.y && bodyList[i].position.x < surfacePosition.x + width / 2.f && bodyList[i].position.x > surfacePosition.x - width / 2.f && bodyList[i].position.y < surfacePosition.y + depth) {
				EpsilonVector dir(0, 1.f);
				float h = bodyList[i].position.y + bodyList[i].height / 3.f - surfacePosition.y;
				if (h > bodyList[i].height) {
					h = bodyList[i].height;
				}
				float hTriangle = bodyList[i].height - h;
				float sideTriangle = (hTriangle * bodyList[i].width) / bodyList[i].height;
				float area = (sideTriangle + bodyList[i].width) * h / 2;
				float damperForce = (damperConstant * bodyList[i].linearVelocity.Dot(dir)) * h;
				EpsilonVector force = -dir * ((area * 9.81f * density) + damperForce);
				bodyList[i].AddForce(force);
			}
		}
	}
}

void EpsilonWorld::CreateWater(EpsilonVector surfacePosition, float width, float depth, float density)
{
	waterSurfacePosition = surfacePosition;
	waterDensity = density;
	waterWidth = width;
	waterDepth = depth;
}

void EpsilonWorld::AirResistance(float dt)
{
	for (size_t i = 0; i < bodyList.size(); i++) {
		EpsilonVector resistance(bodyList[i].linearVelocity.x * bodyList[i].linearVelocity.x, bodyList[i].linearVelocity.y * bodyList[i].linearVelocity.y);
		resistance = -resistance * airResistanceConstant;
		float angularResistance = -abs(bodyList[i].angularVelocity) * bodyList[i].angularVelocity * rotationalAirResistanceConstant;
		if (angularResistance != 0) {
			bodyList[i].angularVelocity += (angularResistance *bodyList[i].inverseInertia) * dt;
		}
		
		bodyList[i].AddForce(resistance);
	}
}
