#include "EpsilonWorld.h"

struct Task : enki::ITaskSet {
	std::function<void(uint32_t, uint32_t, uint32_t)> m_Function;

	Task(uint32_t setSize, std::function<void(uint32_t, uint32_t, uint32_t)> func)
		: enki::ITaskSet(setSize), m_Function(func) {}

	void ExecuteRange(enki::TaskSetPartition range, uint32_t threadnum) override {
		if (m_Function) m_Function(range.start, range.end, threadnum);
	}
};
void EpsilonWorld::RunTask(uint32_t count, std::function<void(uint32_t, uint32_t, uint32_t)> func) {
	Task task(count, func);
	scheduler.AddTaskSetToPipe(&task);
	scheduler.WaitforTask(&task);
}
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
	scheduler.Initialize();
}

void EpsilonWorld::AddBody(EpsilonBody body)
{
	bodyList.emplace_back(body);
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

EpsilonBody EpsilonWorld::GetDynamicBody(float index)
{
	return bodyList[dynamicBodyList[index]];
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
	//ZoneScoped;
	if (iterations < 1) {
		iterations = 1;
	}
	else if (iterations > 32) {
		iterations = 32;
	}
	dynamicBodyList.clear();
	lowPriorityObjects.clear();
	highPriorityObjects.clear();
	PreFiltering();
	RunTask(lowPriorityObjects.size(), [this,&dt](uint32_t start, uint32_t end, uint32_t threadNum) {
		UpdateMovement(start, end, dt, 1);
		});
	
		
	//UpdateMovement(0, lowPriorityObjects.size(), dt, 1);
	RunTask(lowPriorityObjects.size(), [this,&dt](uint32_t start, uint32_t end, uint32_t threadNum) {
		ResolveSpringConnection(start,end, dt, 1);
		});
	RunTask(lowPriorityObjects.size(), [this](uint32_t start, uint32_t end, uint32_t threadNum) {
		ResolveThreadConnection(start,end);
		});
	for (int it = 0; it < iterations / 2; it++) {
		RunTask(highPriorityObjects.size(), [this,&dt,iterations](uint32_t start, uint32_t end, uint32_t threadNum) {	
			ResolveHighPrioritySpringConnection(start,end,dt,iterations/2.f);
			});
		
		RunTask(highPriorityObjects.size(), [this, &dt,iterations](uint32_t start, uint32_t end, uint32_t threadNum) {
			UpdateHighPriorityMovement(start, end, dt, iterations / 2.f);
			});
		RunTask(highPriorityObjects.size(), [this](uint32_t start, uint32_t end, uint32_t threadNum) {
			ResolveHighPriorityThreadConnection(start,end);
			});
		
	}
	for (int it = 0; it < iterations; it++) {
		
		contactPairs.clear();
		BroadPhase(windowWidth,windowHeight,zoom);
		RunTask(contactPairs.size(), [this](uint32_t start, uint32_t end, uint32_t threadNum) {
			NarrowPhase(start, end);
			});			
	}
	RunTask(dynamicBodyList.size(), [this, &dt](uint32_t start, uint32_t end, uint32_t threadNum) {
		AirResistance(start,end, dt, 1);
		});
	RunTask(dynamicBodyList.size(), [this](uint32_t start, uint32_t end, uint32_t threadNum) {
		Buoyancy(start,end);
		});
}


void EpsilonWorld::SeperateBodies(EpsilonBody& bodyA, EpsilonBody& bodyB, EpsilonVector mtv) {
	bodyA.Move((-mtv / 2.f) * !bodyA.isStatic);
	bodyB.Move((mtv/2.f) * !bodyB.isStatic);
}

void EpsilonWorld::PreFiltering()
{
	//ZoneScoped;
	for (int i = 0; i < bodyList.size(); i++) {
		if(!bodyList[i].isStatic) {
			dynamicBodyList.push_back(i);
			if (bodyList[i].linearVelocity.LengthSquared() > 100.f) {
				highPriorityObjects.push_back(i);
			}
			else {
				lowPriorityObjects.push_back(i);
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
		QuadTree qtree(AABB(offsetX, windowWidth * zoom + offsetX, offsetY, windowHeight * zoom + offsetY), 15);
		for (int i = 0; i < bodyList.size(); i++) {
			qtree.insert(bodyList[i],bodyList,i);
		}

		const uint32_t numTasks = scheduler.GetNumTaskThreads() + 1;
		std::vector<std::vector<std::pair<int, int>>> localPairs(numTasks);

		enki::TaskSet task(
			dynamicBodyList.size(),
			[&](enki::TaskSetPartition range, uint32_t threadnum)
			{
				std::vector<int> localPotential;

				for (uint32_t i = range.start; i < range.end; ++i)
				{
					int dynIdx = dynamicBodyList[i];

					localPotential.clear();
					qtree.query(
						bodyList[dynIdx].GetAABB(),
						localPotential,
						bodyList
					);

					for (int other : localPotential)
					{
						if (dynIdx == other) continue;

						localPairs[threadnum].emplace_back(dynIdx, other);
					}
				}
			}
		);

		scheduler.AddTaskSetToPipe(&task);
		scheduler.WaitforTask(&task);

		// Merge results (single-threaded)
		for (auto& v : localPairs) {
			for (auto& p : v) {
				contactPairs.push_back({ p.first, p.second });
			}
		}
	
}

void EpsilonWorld::NarrowPhase(int start, int end) {
	//ZoneScoped;
	for (int i = start; i < end; i++) {
		vector<int> t = contactPairs[i];
		EpsilonBody& bodyA = bodyList[t[0]];
		EpsilonBody& bodyB = bodyList[t[1]];
		float depth = 0;
		EpsilonVector normal(0, 0);
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

void EpsilonWorld::UpdateMovement(uint32_t start,uint32_t end, float dt,int iterations) {
	//ZoneScoped;
	for (int i = start; i < end; i++) {
		bodyList[lowPriorityObjects[i]].updateMovement(dt, gravity, iterations);
	}
	
}

void EpsilonWorld::UpdateHighPriorityMovement(uint32_t start, uint32_t end, float dt, int iterations)
{
	//ZoneScoped;
	for (int i = start; i < end; i++) {
		bodyList[highPriorityObjects[i]].updateMovement(dt, gravity, iterations);
	}
}

void EpsilonWorld::ResolveCollisonBasic(CollisionManifold& manifold)
{
	//ZoneScoped;
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
	//ZoneScoped;
	
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
	//ZoneScoped;
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
void EpsilonWorld::ResolveThreadConnection(int start, int end) {
	
	for (int i = start; i < end; i++) {
		if (bodyList[lowPriorityObjects[i]].connectiontype == none|| bodyList[lowPriorityObjects[i]].connectiontype == spring) {
			continue;
		}
		else if (bodyList[lowPriorityObjects[i]].originPosition.Distance(bodyList[lowPriorityObjects[i]].connectionPosition) > bodyList[lowPriorityObjects[i]].connectionDistance) {
			EpsilonVector dir = bodyList[lowPriorityObjects[i]].connectionPosition - bodyList[lowPriorityObjects[i]].originPosition;
			float dist = dir.Length();
			float restDist = dist - bodyList[lowPriorityObjects[i]].connectionDistance;
			if (restDist > 0.01f) {
				restDist = 0.01f;
			}
			float damperForce = -(damperThreadConstant * bodyList[lowPriorityObjects[i]].linearVelocity.Dot(dir)) / dist;
			bodyList[lowPriorityObjects[i]].linearVelocity += -dir * (bodyList[lowPriorityObjects[i]].inverseMass*restDist);
			bodyList[lowPriorityObjects[i]].AddForce(damperForce * dir);
			EpsilonVector offset = bodyList[lowPriorityObjects[i]].connectionPosition - bodyList[lowPriorityObjects[i]].position;
			bodyList[lowPriorityObjects[i]].angularVelocity += offset.Cross(-dir * (bodyList[lowPriorityObjects[i]].inverseMass * restDist));
		}
	}
}
void EpsilonWorld::ResolveSpringConnection(int start, int end,float dt, int iterations) {
	
	dt = dt/iterations;
	for (int i = start; i < end; i++) {
		if (bodyList[lowPriorityObjects[i]].connectiontype == none || bodyList[lowPriorityObjects[i]].connectiontype == thr) {
			continue;
		}
		
		EpsilonVector dir = bodyList[lowPriorityObjects[i]].connectionPosition - bodyList[lowPriorityObjects[i]].originPosition;
		float dist = dir.Length();
		float restDist = dist - bodyList[lowPriorityObjects[i]].connectionDistance;
		float springForce = restDist * springConstant;
		
		float damperForce = (damperConstant * (bodyList[lowPriorityObjects[i]].linearVelocity.Dot(dir))/dist);
		EpsilonVector force = -(springForce+damperForce)*dir/dist;
		EpsilonVector offset = bodyList[lowPriorityObjects[i]].connectionPosition - bodyList[lowPriorityObjects[i]].position;
		float angularResistance = offset.Cross(force*bodyList[lowPriorityObjects[i]].inverseInertia);
		bodyList[lowPriorityObjects[i]].angularVelocity += angularResistance* bodyList[lowPriorityObjects[i]].inverseMass* dt;
		bodyList[lowPriorityObjects[i]].AddForce(force);
	}
}
void EpsilonWorld::ResolveHighPriorityThreadConnection(int start, int end)
{
	
	for (int i = start; i < end; i++) {
		if (bodyList[highPriorityObjects[i]].connectiontype == none || bodyList[highPriorityObjects[i]].connectiontype == spring) {
			continue;
		}
		else if (bodyList[highPriorityObjects[i]].originPosition.Distance(bodyList[highPriorityObjects[i]].connectionPosition) > bodyList[highPriorityObjects[i]].connectionDistance) {
			EpsilonVector dir = bodyList[highPriorityObjects[i]].connectionPosition - bodyList[highPriorityObjects[i]].originPosition;
			float dist = dir.Length();
			float restDist = dist - bodyList[highPriorityObjects[i]].connectionDistance;
			if (restDist > 0.01f) {
				restDist = 0.01f;
			}
			float damperForce = -(damperThreadConstant * bodyList[highPriorityObjects[i]].linearVelocity.Dot(dir)) / dist;
			bodyList[highPriorityObjects[i]].linearVelocity += -dir * (bodyList[highPriorityObjects[i]].inverseMass * restDist);
			bodyList[highPriorityObjects[i]].AddForce(damperForce * dir);
			EpsilonVector offset = bodyList[highPriorityObjects[i]].connectionPosition - bodyList[highPriorityObjects[i]].position;
			bodyList[highPriorityObjects[i]].angularVelocity += offset.Cross(-dir * (bodyList[highPriorityObjects[i]].inverseMass * restDist));
		}
	}
}
void EpsilonWorld::ResolveHighPrioritySpringConnection(int start, int end,float dt, int iterations)
{
	
	dt = dt / iterations;
	for (int i = start; i < end; i++) {
		if (bodyList[highPriorityObjects[i]].connectiontype == none || bodyList[highPriorityObjects[i]].connectiontype == thr) {
			continue;
		}

		EpsilonVector dir = bodyList[highPriorityObjects[i]].connectionPosition - bodyList[highPriorityObjects[i]].originPosition;
		float dist = dir.Length();
		float restDist = dist - bodyList[highPriorityObjects[i]].connectionDistance;
		float springForce = restDist * springConstant;
		float damperForce = (damperConstant * (bodyList[highPriorityObjects[i]].linearVelocity.Dot(dir)) / dist);
		EpsilonVector force = -(springForce + damperForce) * dir / dist;
		EpsilonVector offset = bodyList[highPriorityObjects[i]].connectionPosition - bodyList[highPriorityObjects[i]].position;
		float angularResistance = offset.Cross(force * bodyList[highPriorityObjects[i]].inverseInertia);
		bodyList[highPriorityObjects[i]].angularVelocity += angularResistance* bodyList[highPriorityObjects[i]].inverseMass* dt;
		bodyList[highPriorityObjects[i]].AddForce(force);
	}
}
void EpsilonWorld::Explosion(EpsilonVector position, float radius, float magnitude)
{
	
	for (int i = 0; i < dynamicBodyList.size(); i++) {
		if (bodyList[dynamicBodyList[i]].position.Distance(position) > radius) {
			continue;
		}
		EpsilonVector dir = bodyList[dynamicBodyList[i]].position - position;
		float dist = dir.Length();
		EpsilonVector impulse = (dir * magnitude) / (dist*dist);
		EpsilonVector vertical(0, 1.f);
		float mag = -vertical.Cross(impulse);
		bodyList[dynamicBodyList[i]].linearVelocity += impulse * bodyList[dynamicBodyList[i]].inverseMass;
		bodyList[dynamicBodyList[i]].angularVelocity += mag * bodyList[dynamicBodyList[i]].inverseInertia;
	}
}
void EpsilonWorld::Buoyancy(int start, int end) {
	//ZoneScoped;
	for (int j = 0; j < waterList.size(); j++) {
		for (int i = start; i < end; i++) {
			Water& w = waterList[j];
			if (bodyList[dynamicBodyList[i]].shapetype == box) {
				if (bodyList[dynamicBodyList[i]].position.y + bodyList[dynamicBodyList[i]].height / 2.f > w.surfacePosition.y && bodyList[dynamicBodyList[i]].position.x < w.surfacePosition.x + (w.width / 2.f) && bodyList[dynamicBodyList[i]].position.x > w.surfacePosition.x - (w.width / 2.f) && bodyList[dynamicBodyList[i]].position.y < w.surfacePosition.y + w.depth) {
					EpsilonVector dir(0, 1.f);
					float h = bodyList[dynamicBodyList[i]].position.y + (bodyList[dynamicBodyList[i]].height / 2.f) - w.surfacePosition.y;
					if (h > bodyList[dynamicBodyList[i]].height) {
						h = bodyList[dynamicBodyList[i]].height;
					}
					float damperForce = (damperWaterConstant * bodyList[dynamicBodyList[i]].linearVelocity.Dot(dir)) * h;
					EpsilonVector force = -dir * ((bodyList[dynamicBodyList[i]].width * h * 9.81f * w.density) + damperForce);
					bodyList[dynamicBodyList[i]].AddForce(force);
				}
			}
			else if (bodyList[dynamicBodyList[i]].shapetype == circle) {
				if (bodyList[dynamicBodyList[i]].position.y + bodyList[dynamicBodyList[i]].radius > w.surfacePosition.y && bodyList[dynamicBodyList[i]].position.x < w.surfacePosition.x + w.width / 2.f && bodyList[dynamicBodyList[i]].position.x > w.surfacePosition.x - w.width / 2.f && bodyList[dynamicBodyList[i]].position.y < w.surfacePosition.y + w.depth) {
					EpsilonVector dir(0, 1.f);
					float h = bodyList[dynamicBodyList[i]].position.y + bodyList[dynamicBodyList[i]].radius - w.surfacePosition.y;
					if (h > bodyList[dynamicBodyList[i]].radius * 2.f) {
						h = bodyList[dynamicBodyList[i]].radius * 2.f;
					}
					float r = bodyList[dynamicBodyList[i]].radius;
					float area = r * r * acos(1 - (h / r)) - (r - h) * sqrt(r * r - (r - h) * (r - h));
					float damperForce = (damperWaterConstant * bodyList[dynamicBodyList[i]].linearVelocity.Dot(dir)) * h;
					EpsilonVector force = -dir * ((area * 9.81f * w.density) + damperForce);
					bodyList[dynamicBodyList[i]].AddForce(force);
				}
			}
			else if (bodyList[dynamicBodyList[i]].shapetype == triangle) {
				if (bodyList[dynamicBodyList[i]].position.y + bodyList[dynamicBodyList[i]].height / 3.f > w.surfacePosition.y && bodyList[dynamicBodyList[i]].position.x < w.surfacePosition.x + w.width / 2.f && bodyList[dynamicBodyList[i]].position.x > w.surfacePosition.x - w.width / 2.f && bodyList[dynamicBodyList[i]].position.y < w.surfacePosition.y + w.depth) {
					EpsilonVector dir(0, 1.f);
					float h = bodyList[dynamicBodyList[i]].position.y + bodyList[dynamicBodyList[i]].height / 3.f - w.surfacePosition.y;
					if (h > bodyList[dynamicBodyList[i]].height) {
						h = bodyList[dynamicBodyList[i]].height;
					}
					float hTriangle = bodyList[dynamicBodyList[i]].height - h;
					float sideTriangle = (hTriangle * bodyList[dynamicBodyList[i]].width) / bodyList[dynamicBodyList[i]].height;
					float area = (sideTriangle + bodyList[dynamicBodyList[i]].width) * h / 2;
					float damperForce = (damperWaterConstant * bodyList[dynamicBodyList[i]].linearVelocity.Dot(dir)) * h;
					EpsilonVector force = -dir * ((area * 9.81f * w.density) + damperForce);
					bodyList[dynamicBodyList[i]].AddForce(force);
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

void EpsilonWorld::AirResistance(int start,int end, float dt, int iterations)
{
	
	dt/=iterations;
	for (int i = start; i < end; i++) {
		EpsilonVector resistance(bodyList[dynamicBodyList[i]].linearVelocity.x * bodyList[dynamicBodyList[i]].linearVelocity.x, bodyList[dynamicBodyList[i]].linearVelocity.y * bodyList[dynamicBodyList[i]].linearVelocity.y);
		resistance = -resistance * airResistanceConstant;
		float angularResistance = -abs(bodyList[dynamicBodyList[i]].angularVelocity) * bodyList[dynamicBodyList[i]].angularVelocity * rotationalAirResistanceConstant;
		if (angularResistance != 0) {
			bodyList[dynamicBodyList[i]].angularVelocity += (angularResistance *bodyList[dynamicBodyList[i]].inverseInertia) * dt;
		}
		
		bodyList[dynamicBodyList[i]].AddForce(resistance);
	}
}
