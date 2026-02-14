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
	linearVelocityThreshold(0.00000001f),
	angularVelocityThreshold(0.00000001f),
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
	PreFiltering(dt);
	for (int it = 0; it < iterations; it++) {
		contactPairs.clear();
		allManifolds.clear();
		islands.clear();
		BroadPhase(windowWidth, windowHeight, zoom);
		
			//WarmStart(0, 0);
			
			NarrowPhase(0,0);
			
		BuildIslands();
		RunTask(islands.size(), [this, &dt,iterations](uint32_t start, uint32_t end, uint32_t threadNum) {
			SolveIslands(start, end, dt,iterations);
			});
	}
	
	RunTask(islands.size(), [this, &dt](uint32_t start, uint32_t end, uint32_t threadNum) {
		UpdateMovement(start, end, dt, 1);
		});
	RunTask(islands.size(), [this, &dt](uint32_t start, uint32_t end, uint32_t threadNum) {
		ResolveSpringConnection(start, end, dt, 1);
		});
	RunTask(islands.size(), [this](uint32_t start, uint32_t end, uint32_t threadNum) {
		ResolveThreadConnection(start, end);
		});
	RunTask(islands.size(), [this, &dt](uint32_t start, uint32_t end, uint32_t threadNum) {
		AirResistance(start,end, dt, 1);
		});
	RunTask(islands.size(), [this](uint32_t start, uint32_t end, uint32_t threadNum) {
		Buoyancy(start,end);
		});
}


void EpsilonWorld::SeperateBodies(EpsilonBody& bodyA, EpsilonBody& bodyB, EpsilonVector mtv, float depth) {
	float slop = 0.01f;
	float percent = 0.2f;
	float totalInvMass = bodyA.inverseMass + bodyB.inverseMass;
	EpsilonVector correction = (mtv.Normalized() * (max(depth - slop, 0.0f)) * percent) /totalInvMass ;
	bodyA.Move((-correction)*bodyA.inverseMass);
	bodyB.Move((correction) *bodyB.inverseMass);
}

void EpsilonWorld::PreFiltering(float dt)
{
	//ZoneScoped;
	for (int i = 0; i < bodyList.size(); i++) {
		if (!bodyList[i].isStatic) {
			dynamicBodyList.push_back(i);
		}
		/*if (bodyList[i].sleepTimer > 1.0f) {

			bodyList[i].sleepTimer = 0.0f;
			bodyList[i].isSleeping = true;
		}*/
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
						if (dynIdx <= other) continue;
						else {

							localPairs[threadnum].emplace_back(dynIdx, other);
						}
					}
				}
			}
		);

		scheduler.AddTaskSetToPipe(&task);
		scheduler.WaitforTask(&task);

		for (auto& v : localPairs) {
			for (auto& p : v) {
				contactPairs.push_back({ p.first, p.second });
			}
		}
}

void EpsilonWorld::WarmStart(int start, int end)
{
	for (int i = 0; i < allManifolds.size(); i++) {
		CollisionManifold& manifold = allManifolds[i];
			for (int j = 0; j < manifold.prevImp.size(); j++) {
				manifold.bodyA.linearVelocity += -manifold.prevImp[j] * manifold.bodyA.inverseMass;
				manifold.bodyB.linearVelocity += manifold.prevImp[j] * manifold.bodyB.inverseMass;
			}
			for (int j = 0; j < manifold.prevAngImp.size(); j++) {

				manifold.bodyA.angularVelocity += -manifold.prevAngImp[j] * manifold.bodyA.inverseInertia;
				manifold.bodyB.angularVelocity += manifold.prevAngImp[j] * manifold.bodyB.inverseInertia;
			}
		
	}
	allManifolds.clear();
}

void EpsilonWorld::NarrowPhase(int start, int end) {
	//ZoneScoped;
	const uint32_t numTasks = scheduler.GetNumTaskThreads() + 1;
	vector<vector<CollisionManifold>> manifolds(numTasks);
	enki::TaskSet task(
		contactPairs.size(),
		[&](enki::TaskSetPartition range, uint32_t threadnum)
		{
			for (uint32_t i = range.start; i < range.end; i++) {
				vector<int> t = contactPairs[i];
				EpsilonBody& bodyA = bodyList[t[0]];
				EpsilonBody& bodyB = bodyList[t[1]];
				float depth = 0;
				EpsilonVector normal(0, 0);
				if (bodyB.isSleeping && bodyA.isSleeping || bodyB.isStatic && bodyA.isSleeping || bodyB.isSleeping && bodyA.isStatic) continue;
				if (Collisions::Collide(bodyA, bodyB, normal, depth)) {
					bodyB.isSleeping = false;
					bodyA.isSleeping = false;
					for (int i = 0; i < 3; i++) {
						SeperateBodies(bodyA, bodyB, normal * depth, depth);
					}
					EpsilonVector contact1(0, 0);
					EpsilonVector contact2(0, 0);
					int contactCount = 0;
					Collisions::FindContactPoints(bodyA, bodyB, contact1, contact2, contactCount);
					CollisionManifold contact(bodyA, bodyB, contact1, contact2, normal, depth, contactCount);
					
					for (int i = 0; i < 15; i++) {
						ResolveCollisonWithRotationAndFriction(contact);
					}
					manifolds[threadnum].emplace_back(contact);
				}


			}
		});
	scheduler.AddTaskSetToPipe(&task);
	scheduler.WaitforTask(&task);
	for (auto& v : manifolds) {
		for (auto& p : v) {
			allManifolds.emplace_back(p);
		}
	}
}
void EpsilonWorld::BuildIslands()
{
	int n = bodyList.size();
	DSU dsu(n);

	// 1. Collect all manifolds from threads
	for (int i = 0; i < contactPairs.size();i++) {
		if (bodyList[contactPairs[i][0]].isStatic || bodyList[contactPairs[i][1]].isStatic) continue;
			dsu.unite(contactPairs[i][0], contactPairs[i][1]);
	}

	// 2. Map roots to Island objects
	std::unordered_map<int, int> rootToIslandIdx;
	islands.clear();

	for (int i = 0; i < n; i++) {
		if (bodyList[i].isStatic) continue;

		int root = dsu.find(i);
		if (rootToIslandIdx.find(root) == rootToIslandIdx.end()) {
			rootToIslandIdx[root] = islands.size();
			islands.emplace_back();
		}
		islands[rootToIslandIdx[root]].bodyIndices.push_back(i);
	}

	// 3. Distribute manifolds to islands
}

void EpsilonWorld::SolveIslands(int start, int end, float dt,int iterations)
{
	
	for (int i = start; i < end; ++i) {
		Island& island = islands[i];
		
		// Check if every body in the island is sleeping
		// --- INTEGRATE & SLEEP CHECK ---
		float maxEnergy = 0.0f;
		for (int bIdx : island.bodyIndices) {
			EpsilonBody& b = bodyList[bIdx];
			float energy = b.linearVelocity.LengthSquared()+(b.angularVelocity*b.angularVelocity);
			if (energy > maxEnergy) maxEnergy = energy;
		}
		// 1. Start by assuming the island CAN sleep
		int sleep = 1;
		float sleepDelay = 0.75f; // The object must be still for this many seconds

		if (maxEnergy < 1.f) {
			for (int bIdx : island.bodyIndices) {
				bodyList[bIdx].sleepTimer += dt/iterations;
				// VETO: If any body has a timer lower than the delay, 
				// it's too "new" or "active" to sleep.
				if (bodyList[bIdx].sleepTimer < sleepDelay) {
					sleep = -1;
				}
			}
		}
		else {
			// Energy is too high; nobody sleeps.
			sleep = -1;
			for (int bIdx : island.bodyIndices) {
				bodyList[bIdx].sleepTimer = 0.0f;
			}
		}

		// 2. Final State Application
		if (sleep == 1) {
			island.isAsleep = true;
			for (int bIdx : island.bodyIndices) {
				bodyList[bIdx].isSleeping = true;
			}
		}
		else if (sleep == -1) {
			island.isAsleep = false;

			// Only wake them up if energy is actually high
			// This prevents "flickering" while they are settling			
				for (int bIdx : island.bodyIndices) {
					bodyList[bIdx].isSleeping = false;
				}
		}
	}
}

void EpsilonWorld::UpdateMovement(uint32_t start,uint32_t end, float dt,int iterations) {
	//ZoneScoped;
	for (int i = start; i < end; i++) {
		if (islands[i].isAsleep) {
			continue;
		}
		for (int j = 0; j < islands[i].bodyIndices.size(); j++) {
			
				bodyList[islands[i].bodyIndices[j]].updateMovement(dt, gravity, iterations);
			
		}
		
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
	float e = min(bodyA.restitution, bodyB.restitution);
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
	float e = min(bodyA.restitution, bodyB.restitution);
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
	float e = min(bodyA.restitution, bodyB.restitution);
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
		if (contactVelocityMag > -0.5f) {
			e = 0.0f;
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
		manifold.prevImp.push_back(impulse);
		manifold.prevAngImp.push_back(ra.Cross(impulse));
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
		//jt /= (float)contactCount;
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
		if (islands[i].isAsleep) {
			continue;
		}
		for (int j = 0; j < islands[i].bodyIndices.size(); j++) {
			if (bodyList[islands[i].bodyIndices[j]].connectiontype == none || bodyList[islands[i].bodyIndices[j]].connectiontype == spring) {
				continue;
			}
			if (bodyList[islands[i].bodyIndices[j]].originPosition.Distance(bodyList[islands[i].bodyIndices[j]].connectionPosition) > bodyList[islands[i].bodyIndices[j]].connectionDistance) {
				EpsilonVector dir = bodyList[islands[i].bodyIndices[j]].connectionPosition - bodyList[islands[i].bodyIndices[j]].originPosition;
				float dist = dir.Length();
				float restDist = dist - bodyList[islands[i].bodyIndices[j]].connectionDistance;
				if (restDist > 0.01f) {
					restDist = 0.01f;
				}
				float damperForce = -(damperThreadConstant * bodyList[islands[i].bodyIndices[j]].linearVelocity.Dot(dir)) / dist;
				bodyList[islands[i].bodyIndices[j]].linearVelocity += -dir * (bodyList[islands[i].bodyIndices[j]].inverseMass * restDist);
				bodyList[islands[i].bodyIndices[j]].AddForce(damperForce * dir);
				EpsilonVector offset = bodyList[islands[i].bodyIndices[j]].connectionPosition - bodyList[islands[i].bodyIndices[j]].position;
				bodyList[islands[i].bodyIndices[j]].angularVelocity += offset.Cross(-dir * (bodyList[islands[i].bodyIndices[j]].inverseMass * restDist));
			}
		}
	}
}
void EpsilonWorld::ResolveSpringConnection(int start, int end,float dt, int iterations) {
	
	dt = dt/iterations;
	for (int i = start; i < end; i++) {
		if (islands[i].isAsleep) {
			continue;
		}
		for (int j = 0; j < islands[i].bodyIndices.size(); j++) {
			if (bodyList[islands[i].bodyIndices[j]].connectiontype == none || bodyList[islands[i].bodyIndices[j]].connectiontype == thr) {
				continue;
			}
			EpsilonVector dir = bodyList[islands[i].bodyIndices[j]].connectionPosition - bodyList[islands[i].bodyIndices[j]].originPosition;
			float dist = dir.Length();
			float restDist = dist - bodyList[islands[i].bodyIndices[j]].connectionDistance;
			float springForce = restDist * springConstant;

			float damperForce = (damperConstant * (bodyList[islands[i].bodyIndices[j]].linearVelocity.Dot(dir)) / dist);
			EpsilonVector force = -(springForce + damperForce) * dir / dist;
			EpsilonVector offset = bodyList[islands[i].bodyIndices[j]].connectionPosition - bodyList[islands[i].bodyIndices[j]].position;
			float angularResistance = offset.Cross(force * bodyList[islands[i].bodyIndices[j]].inverseInertia);
			bodyList[islands[i].bodyIndices[j]].angularVelocity += angularResistance * bodyList[islands[i].bodyIndices[j]].inverseMass * dt;
			bodyList[islands[i].bodyIndices[j]].AddForce(force);
		}
	}
}


void EpsilonWorld::Explosion(EpsilonVector position, float radius, float magnitude)
{
	
	for (int i = 0; i < dynamicBodyList.size(); i++) {
		bodyList[dynamicBodyList[i]].isSleeping = false;
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
			if (islands[i].isAsleep) {
				continue;
			}
			for (int k = 0; k < islands[i].bodyIndices.size(); k++) {
				Water& w = waterList[j];
				if (bodyList[islands[i].bodyIndices[k]].shapetype == box) {
					if (bodyList[islands[i].bodyIndices[k]].position.y + bodyList[islands[i].bodyIndices[k]].height / 2.f > w.surfacePosition.y && bodyList[islands[i].bodyIndices[k]].position.x < w.surfacePosition.x + (w.width / 2.f) && bodyList[islands[i].bodyIndices[k]].position.x > w.surfacePosition.x - (w.width / 2.f) && bodyList[islands[i].bodyIndices[k]].position.y < w.surfacePosition.y + w.depth) {
						EpsilonVector dir(0, 1.f);
						float h = bodyList[islands[i].bodyIndices[k]].position.y + (bodyList[islands[i].bodyIndices[k]].height / 2.f) - w.surfacePosition.y;
						if (h > bodyList[islands[i].bodyIndices[k]].height) {
							h = bodyList[islands[i].bodyIndices[k]].height;
						}
						float damperForce = (damperWaterConstant * bodyList[islands[i].bodyIndices[k]].linearVelocity.Dot(dir)) * h;
						EpsilonVector force = -dir * ((bodyList[islands[i].bodyIndices[k]].width * h * 9.81f * w.density) + damperForce);
						bodyList[islands[i].bodyIndices[k]].AddForce(force);
					}
				}
				else if (bodyList[islands[i].bodyIndices[k]].shapetype == circle) {
					if (bodyList[islands[i].bodyIndices[k]].position.y + bodyList[islands[i].bodyIndices[k]].radius > w.surfacePosition.y && bodyList[islands[i].bodyIndices[k]].position.x < w.surfacePosition.x + w.width / 2.f && bodyList[islands[i].bodyIndices[k]].position.x > w.surfacePosition.x - w.width / 2.f && bodyList[islands[i].bodyIndices[k]].position.y < w.surfacePosition.y + w.depth) {
						EpsilonVector dir(0, 1.f);
						float h = bodyList[islands[i].bodyIndices[k]].position.y + bodyList[islands[i].bodyIndices[k]].radius - w.surfacePosition.y;
						if (h > bodyList[islands[i].bodyIndices[k]].radius * 2.f) {
							h = bodyList[islands[i].bodyIndices[k]].radius * 2.f;
						}
						float r = bodyList[islands[i].bodyIndices[k]].radius;
						float area = r * r * acos(1 - (h / r)) - (r - h) * sqrt(r * r - (r - h) * (r - h));
						float damperForce = (damperWaterConstant * bodyList[islands[i].bodyIndices[k]].linearVelocity.Dot(dir)) * h;
						EpsilonVector force = -dir * ((area * 9.81f * w.density) + damperForce);
						bodyList[islands[i].bodyIndices[k]].AddForce(force);
					}
				}
				else if (bodyList[islands[i].bodyIndices[k]].shapetype == triangle) {
					if (bodyList[islands[i].bodyIndices[k]].position.y + bodyList[islands[i].bodyIndices[k]].height / 3.f > w.surfacePosition.y && bodyList[islands[i].bodyIndices[k]].position.x < w.surfacePosition.x + w.width / 2.f && bodyList[islands[i].bodyIndices[k]].position.x > w.surfacePosition.x - w.width / 2.f && bodyList[islands[i].bodyIndices[k]].position.y < w.surfacePosition.y + w.depth) {
						EpsilonVector dir(0, 1.f);
						float h = bodyList[islands[i].bodyIndices[k]].position.y + bodyList[islands[i].bodyIndices[k]].height / 3.f - w.surfacePosition.y;
						if (h > bodyList[islands[i].bodyIndices[k]].height) {
							h = bodyList[islands[i].bodyIndices[k]].height;
						}
						float hTriangle = bodyList[islands[i].bodyIndices[k]].height - h;
						float sideTriangle = (hTriangle * bodyList[islands[i].bodyIndices[k]].width) / bodyList[islands[i].bodyIndices[k]].height;
						float area = (sideTriangle + bodyList[islands[i].bodyIndices[k]].width) * h / 2;
						float damperForce = (damperWaterConstant * bodyList[islands[i].bodyIndices[k]].linearVelocity.Dot(dir)) * h;
						EpsilonVector force = -dir * ((area * 9.81f * w.density) + damperForce);
						bodyList[islands[i].bodyIndices[k]].AddForce(force);
					}
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
		if (islands[i].isAsleep) {
			continue;
		}
		for (int j = 0; j < islands[i].bodyIndices.size(); j++) {
				EpsilonVector resistance(bodyList[islands[i].bodyIndices[j]].linearVelocity.x * bodyList[islands[i].bodyIndices[j]].linearVelocity.x, bodyList[islands[i].bodyIndices[j]].linearVelocity.y * bodyList[islands[i].bodyIndices[j]].linearVelocity.y);
				resistance = -resistance * airResistanceConstant;
				float angularResistance = -abs(bodyList[islands[i].bodyIndices[j]].angularVelocity) * bodyList[islands[i].bodyIndices[j]].angularVelocity * rotationalAirResistanceConstant;
				if (angularResistance != 0) {
					bodyList[islands[i].bodyIndices[j]].angularVelocity += (angularResistance * bodyList[islands[i].bodyIndices[j]].inverseInertia) * dt;
				}
				bodyList[islands[i].bodyIndices[j]].AddForce(resistance);
		}
		
	}
}
