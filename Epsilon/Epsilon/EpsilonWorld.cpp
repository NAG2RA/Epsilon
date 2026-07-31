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
EpsilonWorld::EpsilonWorld(int windowWidth, int windowHeight, int worldWidth, int worldHeight, float zoom)
	:depth(0),
	normal(0, 0),
	gravity(0, 9.8f),
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
	worldHeight(worldHeight),
	worldWidth(worldWidth),
	zoom(zoom)
{
	scheduler.Initialize();
	transforms.Init();
	angVelocities.Init();
	pseudoAngVels.Init();
	pseudoVels.Init();
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
	ZoneScoped;
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
		islands.clear();
		BroadPhase(windowWidth, windowHeight, zoom);
		NarrowPhase(0, 0, dt);
		BuildIslands();
		RunTask(islands.size(), [this, &dt, iterations](uint32_t start, uint32_t end, uint32_t threadNum) {
			SolveIslands(start, end, dt, iterations);
			});
	}
	resolveCCDCollisions(dt, iterations);
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
		AirResistance(start, end, dt, 1);
		});
	RunTask(islands.size(), [this](uint32_t start, uint32_t end, uint32_t threadNum) {
		Buoyancy(start, end);
		});

}


void EpsilonWorld::SeparateBodies(EpsilonBody& bodyA, EpsilonBody& bodyB, EpsilonVector mtv, float depth) {
	float slop = 0.01f;
	float percent = 0.2f;
	float totalInvMass = bodyA.inverseMass + bodyB.inverseMass;
	EpsilonVector correction = (mtv.Normalized() * (max(depth - slop, 0.0f)) * percent) / totalInvMass;
	bodyA.Move((-correction) * bodyA.inverseMass);
	bodyB.Move((correction)*bodyB.inverseMass);
}

void EpsilonWorld::PreFiltering(float dt)
{
	//ZoneScoped;
	for (int i = 0; i < bodyList.size(); i++) {
		if (!bodyList[i].isStatic) {
			dynamicBodyList.push_back(i);
		}
		if (bodyList[i].usingCCD) {
			ccdBodies.push_back(i);
		}
	}
}
void EpsilonWorld::BroadPhase(int windowWidth, int windowHeight, float zoom) {
	ZoneScoped;
	if (dynamicBodyList.size() == 0) {
		return;
	}
	float offsetX((windowWidth - windowWidth * zoom) / 2.f);
	float offsetY((windowHeight - windowHeight * zoom) / 2.f);
	QuadTree qtree(AABB(offsetX, windowWidth * zoom + offsetX, offsetY, windowHeight * zoom + offsetY), 15);
	for (int i = 0; i < bodyList.size(); i++) {
		qtree.insert(bodyList[i], bodyList, i);
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
					bodyList[dynIdx].GetAABB(bodyList[dynIdx].usingCCD),
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

float EpsilonWorld::TimeOfImpact(float dt, EpsilonBody& A, EpsilonBody& B, float& depth, EpsilonVector& normal)
{
	if (!A.usingCCD && !B.usingCCD)
		return dt;

	EpsilonVector originalPosA = A.position;
	EpsilonVector originalPosB = B.position;

	EpsilonVector vRel = A.linearVelocity - B.linearVelocity;
	EpsilonVector aRel = A.acceleration - B.acceleration;
	float tMin = 0.0f;
	float tMax = dt;

	A.position = originalPosA + vRel * tMax + aRel * tMax * tMax * 0.5f;
	B.position = originalPosB;


	for (int i = 0; i < 15; ++i)
	{
		float tMid = 0.5f * (tMin + tMax);

		A.position = originalPosA + vRel * tMid + aRel * tMid * tMid * 0.5f;
		B.position = originalPosB;

		if (Collide(A, B, normal, depth))
			tMax = tMid;
		else
			tMin = tMid;
	}

	A.position = originalPosA;
	B.position = originalPosB;

	return tMax;

}

void EpsilonWorld::resolveCCDCollisions(float& dt, int iterations)
{
	float remaining = dt;
	int maxSubSteps = 10;
	for (int step = 0; step < maxSubSteps && remaining > 1e-4f; ++step) {
		float minTOI = remaining;
		vector<int> bestPair;
		int indx = 0;
		float finalDepth = 0;
		EpsilonVector finalNormal(0, 0);
		for (int i = 0; i < contactPairs.size(); i++) {
			float tempDepth = 0;
			EpsilonVector tempNormal(0, 0);
			vector<int> t = contactPairs[i];
			float toi = TimeOfImpact(remaining, bodyList[t[0]], bodyList[t[1]], tempDepth, tempNormal);
			if (toi < minTOI) {

				minTOI = toi;
				bestPair = t;
				indx = i;
				finalDepth = tempDepth;
				finalNormal = tempNormal;
			}
		}
		if (bestPair.empty()) break;

		for (int idx : dynamicBodyList) {
			bodyList[idx].updateMovement(minTOI, gravity, iterations);
		}
		EpsilonBody& bodyA = bodyList[bestPair[0]];
		EpsilonBody& bodyB = bodyList[bestPair[1]];

		EpsilonVector c1, c2;
		int count = 0;
		FindContactPoints(bodyA, bodyB, c1, c2, count);

		CollisionManifold manifold(bodyA, bodyB, c1, c2, finalNormal, finalDepth, count);
		ResolveCollisonWithRotationAndFriction(manifold);

		bodyA.isSleeping = false;
		bodyB.isSleeping = false;

		remaining -= minTOI;
		contactPairs.erase(contactPairs.begin() + indx);
	}
	dt = remaining;
}

void EpsilonWorld::NarrowPhase(int start, int end, float dt) {
	/*ZoneScoped;
	const uint32_t numTasks = scheduler.GetNumTaskThreads() + 1;
	vector<vector<pair<vector<int>, CollisionManifold>>> pairs(numTasks);
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
				if (Collide(bodyA, bodyB, normal, depth)) {
					bodyB.isSleeping = false;
					bodyA.isSleeping = false;

					SeparateBodies(bodyA, bodyB, normal * depth, depth);

					EpsilonVector contact1(0, 0);
					EpsilonVector contact2(0, 0);
					int contactCount = 0;
					FindContactPoints(bodyA, bodyB, contact1, contact2, contactCount);
					CollisionManifold contact(bodyA, bodyB, contact1, contact2, normal, depth, contactCount);

					auto it = prevManifolds.find(t);
					vector<EpsilonVector> contactList = { contact1,contact2 };
					if (it != prevManifolds.end()) {
						CollisionManifold& oldManifold = it->second;
						if (NearlyEqual(oldManifold.contact1, contact.contact1) && NearlyEqual(oldManifold.contact2, contact.contact2)) {
							for (int i = 0; i < contactList.size(); i++) {
								contact.accumulatedNormalImpulse[i] = oldManifold.accumulatedNormalImpulse[i];
								contact.accumulatedTangentImpulse[i] = oldManifold.accumulatedTangentImpulse[i];
								contact.tangentList[i] = oldManifold.tangentList[i];
								contact.bodyA.linearVelocity += -oldManifold.accumulatedNormalImpulse[i] * contact.bodyA.inverseMass * contact.normal;
								contact.bodyB.linearVelocity += oldManifold.accumulatedNormalImpulse[i] * contact.bodyB.inverseMass * contact.normal;
								contact.bodyA.linearVelocity += -oldManifold.accumulatedTangentImpulse[i] * contact.bodyA.inverseMass * contact.tangentList[i];
								contact.bodyB.linearVelocity += oldManifold.accumulatedTangentImpulse[i] * contact.bodyB.inverseMass * contact.tangentList[i];
								EpsilonVector ra = contactList[i] - bodyA.position;
								EpsilonVector rb = contactList[i] - bodyB.position;
								contact.bodyA.angularVelocity += -ra.Cross(oldManifold.accumulatedNormalImpulse[i] * contact.normal) * contact.bodyA.inverseInertia;
								contact.bodyB.angularVelocity += rb.Cross(oldManifold.accumulatedNormalImpulse[i] * contact.normal) * contact.bodyB.inverseInertia;
								contact.bodyA.angularVelocity += -ra.Cross(oldManifold.accumulatedTangentImpulse[i] * contact.tangentList[i]) * contact.bodyA.inverseInertia;
								contact.bodyB.angularVelocity += rb.Cross(oldManifold.accumulatedTangentImpulse[i] * contact.tangentList[i]) * contact.bodyB.inverseInertia;
							}



						}

					}

					pairs[threadnum].emplace_back(make_pair(t, contact));




					ResolveCollisonWithRotationAndFriction(contact);
				}


			}
		});
	scheduler.AddTaskSetToPipe(&task);
	scheduler.WaitforTask(&task);
	prevManifolds.clear();
	for (auto& v : pairs) {
		for (auto& p : v) {
			prevManifolds.insert(p);
		}
	}*/
}
void EpsilonWorld::BuildIslands()
{
	int n = bodyList.size();
	DSU dsu(n);

	for (int i = 0; i < contactPairs.size(); i++) {
		if (bodyList[contactPairs[i][0]].isStatic || bodyList[contactPairs[i][1]].isStatic) continue;
		dsu.unite(contactPairs[i][0], contactPairs[i][1]);
	}

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

}

void EpsilonWorld::SolveIslands(int start, int end, float dt, int iterations)
{

	for (int i = start; i < end; ++i) {
		Island& island = islands[i];

		float maxEnergy = 0.0f;
		for (int bIdx : island.bodyIndices) {
			EpsilonBody& b = bodyList[bIdx];
			float energy = b.linearVelocity.LengthSquared() + (b.angularVelocity * b.angularVelocity);
			if (energy > maxEnergy) maxEnergy = energy;
		}
		int sleep = 1;
		float sleepDelay = 2.f;

		if (maxEnergy < 2.f) {
			for (int bIdx : island.bodyIndices) {
				bodyList[bIdx].sleepTimer += dt / iterations;
				if (bodyList[bIdx].sleepTimer < sleepDelay) {
					sleep = -1;
				}
			}
		}
		else {
			sleep = -1;
			for (int bIdx : island.bodyIndices) {
				bodyList[bIdx].sleepTimer = 0.0f;
			}
		}

		if (sleep == 1) {
			island.isAsleep = true;
			for (int bIdx : island.bodyIndices) {
				bodyList[bIdx].isSleeping = true;
			}
		}
		else if (sleep == -1) {
			island.isAsleep = false;
			for (int bIdx : island.bodyIndices) {
				bodyList[bIdx].isSleeping = false;
			}
		}
	}
}

void EpsilonWorld::UpdateMovement(uint32_t start, uint32_t end, float dt, int iterations) {
	ZoneScoped;
	for (int i = start; i < end; i++) {
		if (islands[i].isAsleep) {
			continue;
		}
		for (int j = 0; j < islands[i].bodyIndices.size(); j++) {
			EpsilonBody& bd = bodyList[islands[i].bodyIndices[j]];
			//bodyList[islands[i].bodyIndices[j]].updateMovement(dt, gravity, iterations);
			dt /= iterations;

			bd.deltaTime = dt;
			bd.position += bd.linearVelocity * dt;
			bd.acceleration = bd.force * bd.inverseMass;
			bd.acceleration += gravity;
			bd.linearVelocity += bd.acceleration * dt;
			bd.angle += bd.angularVelocity * dt;
			bd.position += (bd.acceleration * dt * dt) / 2.f;
			if (bd.connectiontype != none) {
				if (bd.shapetype == box) {
					EpsilonVector offset(0, -bd.height / 2.f);
					EpsilonVector rotOffset = bd.Transform(offset, bd.position, bd.angle);
					bd.connectionPosition = rotOffset;
				}
				else if (bd.shapetype == triangle) {
					EpsilonVector offset(0, -bd.height * 2.f / 3.f);
					EpsilonVector rotOffset = bd.Transform(offset, bd.position, bd.angle);
					bd.connectionPosition = rotOffset;
				}
				else {
					EpsilonVector offset(0, -bd.radius);
					EpsilonVector rotOffset = bd.Transform(offset, bd.position, bd.angle);
					bd.connectionPosition = rotOffset;
				}
			}

			bd.force = EpsilonVector({ 0,0 });

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
	j /= bodyA.inverseMass + bodyB.inverseMass;
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
		EpsilonVector relativeVelocity = (bodyB.linearVelocity + angularLinearVelocityB) - (bodyA.linearVelocity + angularLinearVelocityA);
		float contactVelocityMag = relativeVelocity.Dot(normal);
		if (contactVelocityMag > 0.f) {
			continue;
		}
		float j = -(1 + e) * contactVelocityMag;
		float raPerpDotN = raPerp.Dot(normal);
		float rbPerpDotN = rbPerp.Dot(normal);
		float denom = bodyA.inverseMass + bodyB.inverseMass +
			(raPerpDotN * raPerpDotN) * bodyA.inverseInertia +
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

		if (abs(contactVelocityMag) < 0.5f) {
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
		float oldImp = manifold.accumulatedNormalImpulse[i];
		manifold.accumulatedNormalImpulse[i] = max(0.0f, oldImp + j);
		EpsilonVector impulse = (manifold.accumulatedNormalImpulse[i] - oldImp) * normal;
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

		tangent = tangent.Normalized();

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
		jt = clamp(jt, -abs(j * sf), abs(j * sf));
		float oldTangImp = manifold.accumulatedTangentImpulse[i];
		float maxF = manifold.accumulatedNormalImpulse[i] * sf;

		manifold.accumulatedTangentImpulse[i] = clamp(oldTangImp + jt, -maxF, maxF);
		frictionImpulse = (manifold.accumulatedTangentImpulse[i] - oldTangImp) * tangent;
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
		float f = -(1 + e) * k * diff * diff;
		EpsilonVector impulse = f * normal;
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
void EpsilonWorld::ResolveSpringConnection(int start, int end, float dt, int iterations) {

	dt = dt / iterations;
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
		EpsilonVector impulse = (dir * magnitude) / (dist * dist);
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

void EpsilonWorld::AirResistance(int start, int end, float dt, int iterations)
{

	dt /= iterations;
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

































void RunTask(EpsilonWorld& world,uint32_t count, std::function<void(uint32_t, uint32_t, uint32_t)> func) {
	Task task(count, func);
	world.scheduler.AddTaskSetToPipe(&task);
	world.scheduler.WaitforTask(&task);
}

int AddStaticBox(EpsilonWorld& world, Entity& ent, Position& pos, Angle& angle, float width, float height, float density, float dynamicFriction, float staticFriction, float restitution) {
	ent.id = world.entityList.size();
	ent.shapetype = box;
	world.entityList.emplace_back(ent);
	world.idToIndex.emplace_back(ent.id);
	world.positions.Add(pos, world.entityList.size() - 1, world.entityList.size());
	Box b{ width,height };
	Vertices v;
	v.vertices = GetBoxVertices(b);
	world.verts.Add(v, world.entityList.size() - 1, world.entityList.size());
	AABB aabb = GetPolyAABB(pos, v, angle);
	world.angles.Add(angle, world.entityList.size() - 1, world.entityList.size());
	world.aabbs.Add(aabb, world.entityList.size() - 1, world.entityList.size());
	FrictionAndRestitution r{ dynamicFriction,staticFriction,restitution };
	world.frictionsAndResitutions.Add(r, world.entityList.size() - 1, world.entityList.size());
	inverseSim s{ 0,0 };
	world.invSims.Add(s, world.entityList.size() - 1, world.entityList.size());
	return ent.id;
}
int CreateStaticBox(EpsilonWorld& world, Position pos, Angle angle, float width, float height, float dynamicFriction, float staticFriction, float restitution) {
	Entity ent;
	ent.id = world.entityList.size();
	ent.shapetype = box;
	world.entityList.emplace_back(ent);
	world.idToIndex.emplace_back(ent.id);
	world.positions.Add(pos, world.entityList.size() - 1, world.entityList.size());
	Box b{ width,height };
	Vertices v;
	v.vertices = GetBoxVertices(b);
	world.verts.Add(v, world.entityList.size() - 1, world.entityList.size());
	AABB aabb = GetPolyAABB(pos, v, angle);
	world.angles.Add(angle, world.entityList.size() - 1, world.entityList.size());
	world.aabbs.Add(aabb, world.entityList.size() - 1, world.entityList.size());
	FrictionAndRestitution r{ dynamicFriction,staticFriction,restitution };
	world.frictionsAndResitutions.Add(r, world.entityList.size() - 1, world.entityList.size());
	inverseSim s{ 0,0 };
	world.invSims.Add(s, world.entityList.size() - 1, world.entityList.size());
	return ent.id;
}
int AddDynamicBox(EpsilonWorld& world, Entity& ent, Position& pos, Angle& angle, float width, float height, float density, float dynamicFriction, float staticFriction, float restitution) {
	ent.id = world.entityList.size();
	ent.shapetype = box;
	world.entityList.emplace_back(ent);
	world.idToIndex.emplace_back(ent.id);
	world.dynamicEntities.emplace_back(ent);
	world.positions.Add(pos, world.entityList.size() - 1, world.entityList.size());
	world.activeObjectCount++;
	Box b{ width,height };
	Vertices v;
	v.vertices = GetBoxVertices(b);
	AABB aabb = GetPolyAABB(pos, v, angle);
	world.verts.Add(v, world.entityList.size() - 1, world.entityList.size());
	Transform t{ 0,0 };
	world.transforms.Add(t, world.entityList.size() - 1, world.entityList.size());
	AngularVelocity vel{ 0 };
	world.angVelocities.Add(vel, world.entityList.size() - 1, world.entityList.size());
	world.pseudoAngVels.Add(vel, world.entityList.size() - 1, world.entityList.size());
	world.angles.Add(angle, world.entityList.size() - 1, world.entityList.size());
	world.aabbs.Add(aabb, world.entityList.size() - 1, world.entityList.size());
	FrictionAndRestitution r{ dynamicFriction,staticFriction,restitution };
	world.frictionsAndResitutions.Add(r, world.entityList.size() - 1, world.entityList.size());
	float mass = width*height * (float)density;
	float inertia = (1.f / 12.f) * mass * (width * width + height * height);
	inverseSim s{ 1.f/mass,1.f/inertia };
	world.invSims.Add(s, world.entityList.size() - 1, world.entityList.size());
	return ent.id;
}
int CreateDynamicBox(EpsilonWorld& world, Position& pos, Angle& angle, float width, float height, float density, float dynamicFriction, float staticFriction, float restitution) {
	Entity ent;
	ent.id = world.entityList.size();
	ent.shapetype = box;
	world.entityList.emplace_back(ent);
	world.idToIndex.emplace_back(ent.id);
	world.dynamicEntities.emplace_back(ent);
	world.activeObjectCount++;
	world.positions.Add(pos, world.entityList.size() - 1, world.entityList.size());
	Box b{ width,height };
	Vertices v;
	v.vertices = GetBoxVertices(b);
	AABB aabb = GetPolyAABB(pos, v, angle);
	world.verts.Add(v, world.entityList.size() - 1, world.entityList.size());
	Transform t{ 0,0 };
	world.transforms.Add(t, world.entityList.size() - 1, world.entityList.size());
	world.pseudoVels.Add(t, world.entityList.size() - 1, world.entityList.size());
	AngularVelocity vel{ 0 };
	world.angVelocities.Add(vel, world.entityList.size() - 1, world.entityList.size());
	world.pseudoAngVels.Add(vel, world.entityList.size() - 1, world.entityList.size());
	world.angles.Add(angle, world.entityList.size() - 1, world.entityList.size());
	world.aabbs.Add(aabb, world.entityList.size() - 1, world.entityList.size());
	FrictionAndRestitution r{ dynamicFriction,staticFriction,restitution };
	world.frictionsAndResitutions.Add(r, world.entityList.size() - 1, world.entityList.size());
	float mass = width * height * (float)density;
	float inertia = (1.f / 12.f) * mass * (width * width + height * height);
	inverseSim s{ 1.f / mass,1.f / inertia };
	world.invSims.Add(s, world.entityList.size() - 1, world.entityList.size());
	return ent.id;
}
int CreateDynamicCircle(EpsilonWorld& world, Position& pos, Angle& angle, float radius, float density, float dynamicFriction, float staticFriction, float restitution) {
	Entity ent;
	ent.id = world.entityList.size();
	ent.shapetype = circle;
	world.entityList.emplace_back(ent);
	world.idToIndex.emplace_back(ent.id);
	world.dynamicEntities.emplace_back(ent);
	world.activeObjectCount++;
	world.positions.Add(pos, world.entityList.size() - 1, world.entityList.size());
	Circle c{ radius };
	world.circles.Add(c, world.entityList.size() - 1, world.entityList.size());
	AABB aabb = GetCircleAABB(pos, c);
	Transform t{ 0,0 };
	world.transforms.Add(t, world.entityList.size() - 1, world.entityList.size());
	world.pseudoVels.Add(t, world.entityList.size() - 1, world.entityList.size());
	AngularVelocity vel{ 0 };
	world.angVelocities.Add(vel, world.entityList.size() - 1, world.entityList.size());
	world.pseudoAngVels.Add(vel, world.entityList.size() - 1, world.entityList.size());
	world.angles.Add(angle, world.entityList.size() - 1, world.entityList.size());
	world.aabbs.Add(aabb, world.entityList.size() - 1, world.entityList.size());
	FrictionAndRestitution r{ dynamicFriction,staticFriction,restitution };
	world.frictionsAndResitutions.Add(r, world.entityList.size() - 1, world.entityList.size());
	float mass = 3.14f * radius * radius * (float)density;
	float inertia = (1.f / 2.f) * mass * radius * radius;
	inverseSim s{ 1.f / mass,1.f / inertia };
	world.invSims.Add(s, world.entityList.size() - 1, world.entityList.size());
	return ent.id;
}
void SeparateBodies(Position& positionA, Position& positionB, const inverseSim& mA, const inverseSim& mB, EpsilonVector mtv, const float& depth) {
	float slop = 0.01f;
	float percent = 0.2f;
	float totalInvMass = mA.inverseMass+mB.inverseMass;
	EpsilonVector correction = mtv * (( (max(depth - slop, 0.0f)) * percent) / totalInvMass);
	positionA.value += (-correction) * mA.inverseMass;
	positionB.value += (correction) *mB.inverseMass;
}

struct QuadTreeDOD {
public:
	int nodeCapacity;
	AABB aabb;
	vector<int> bodies;
	std::unique_ptr<QuadTreeDOD> nw, ne, sw, se;

	QuadTreeDOD(AABB ab, int capacity)
		:
		aabb(ab.min.x, ab.max.x, ab.min.y, ab.max.y),
		nodeCapacity(capacity)
	{
	}
	void query(AABB area, vector<int>& found, vector<AABB>& list) {

		if (!IntersectAABB(area, aabb)) {
			return;
		}

		if (divided) {
			nw->query(area, found, list);
			ne->query(area, found, list);
			sw->query(area, found, list);
			se->query(area, found, list);
		}
		for (int i = 0; i < bodies.size(); i++) {
			if (IntersectAABB(area, list[bodies[i]])) {
				found.push_back(bodies[i]);
			}
		}
	}

	bool insert(AABB& body, vector<AABB>& list, int index) {

		if (!IntersectAABB(aabb, body)) {
			return false;
		}

		if (!divided) {
			if (bodies.size() < nodeCapacity) {
				bodies.push_back(index);
				return true;
			}
			subdivide(list);
		}

		if (ContainsAABB(body, nw->aabb)) return nw->insert(body, list, index);
		if (ContainsAABB(body, ne->aabb)) return ne->insert(body, list, index);
		if (ContainsAABB(body, sw->aabb)) return sw->insert(body, list, index);
		if (ContainsAABB(body, se->aabb)) return se->insert(body, list, index);
		bodies.push_back(index);
		return true;
	}

private:
	bool divided = false;
	void subdivide(vector<AABB>& list) {
		float midX = (aabb.min.x + aabb.max.x) / 2.f;
		float midY = (aabb.min.y + aabb.max.y) / 2.f;

		nw = make_unique<QuadTreeDOD>(AABB(aabb.min.x, midX, aabb.min.y, midY), nodeCapacity);
		ne = make_unique<QuadTreeDOD>(AABB(midX, aabb.max.x, aabb.min.y, midY), nodeCapacity);
		sw = make_unique<QuadTreeDOD>(AABB(aabb.min.x, midX, midY, aabb.max.y), nodeCapacity);
		se = make_unique<QuadTreeDOD>(AABB(midX, aabb.max.x, midY, aabb.max.y), nodeCapacity);

		divided = true;
		vector<int> parentBodies = move(bodies);
		bodies.clear();

		for (int bd : parentBodies) {
			insert(list[bd], list, bd);
		}
	}
};


void BroadPhase(EpsilonWorld& world) {
	ZoneScoped;
	float offsetX((world.windowWidth - world.windowWidth * world.zoom) / 2.f);
	float offsetY((world.windowHeight - world.windowHeight * world.zoom) / 2.f);
	QuadTreeDOD qtree(AABB(-world.worldWidth, world.worldWidth, -world.worldHeight,world.worldHeight), 15);
	for (int i = 0; i < world.aabbs.Size(); i++) {
		qtree.insert(world.aabbs.get(i), world.aabbs.dense, world.aabbs.getEntity(i));
	}

	const uint32_t numTasks = world.scheduler.GetNumTaskThreads() + 1;
	std::vector<std::vector<std::pair<int, int>>> localPairs(numTasks);

	enki::TaskSet task(
		world.dynamicEntities.size(),
		[&](enki::TaskSetPartition range, uint32_t threadnum)
		{
			std::vector<int> localPotential;

			for (uint32_t i = range.start; i < range.end; ++i)
			{
				int dynIdx = world.dynamicEntities[i].id;

				localPotential.clear();
				qtree.query(
					world.aabbs.get(dynIdx),
					localPotential,
					world.aabbs.dense
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

	world.scheduler.AddTaskSetToPipe(&task);
	world.scheduler.WaitforTask(&task);

	for (auto& v : localPairs) {
		for (auto& p : v) {
			
			world.contactPairs.push_back({ p.first, p.second });
			
		}
	}
}

//float TimeOfImpact(float dt, Position& posA, Position& posB, Velocity velA, Velocity velB, Acceleration accA, Acceleration accB, Vertices vertA, Vertices VertB, float& depth, EpsilonVector& normal)
//{
//
//	EpsilonVector originalPosA = posA.value;
//	EpsilonVector originalPosB = posB.value;
//
//	EpsilonVector vRel = velA.value - velB.value;
//	EpsilonVector aRel = accA.value - accB.value;
//	float tMin = 0.0f;
//	float tMax = dt;
//
//	posA.value = originalPosA + vRel * tMax + aRel * tMax * tMax * 0.5f;
//	posB.value = originalPosB;
//
//
//	for (int i = 0; i < 15; ++i)
//	{
//		float tMid = 0.5f * (tMin + tMax);
//
//		posA.value = originalPosA + vRel * tMid + aRel * tMid * tMid * 0.5f;
//		posB.value = originalPosB;
//
//		if (Collide(vertA,vertB,posA,posB, normal, depth))
//			tMax = tMid;
//		else
//			tMin = tMid;
//	}
//
//	posA.value = originalPosA;
//	posB.value = originalPosB;
//
//	return tMax;
//
//}
//
//
//void resolveCCDCollisions(float& dt, int iterations, vector<vector<int>> contactPairs, Position posA, Position posB, Velocity velA, Velocity velB, Acceleration accA, Acceleration accB, Vertices vertA, Vertices vertB)
//{
//	float remaining = dt;
//	int maxSubSteps = 10;
//	for (int step = 0; step < maxSubSteps && remaining > 1e-4f; ++step) {
//		float minTOI = remaining;
//		vector<int> bestPair;
//		int indx = 0;
//		float finalDepth = 0;
//		EpsilonVector finalNormal(0, 0);
//		for (int i = 0; i < contactPairs.size(); i++) {
//			float tempDepth = 0;
//			EpsilonVector tempNormal(0, 0);
//			vector<int> t = contactPairs[i];
//			float toi = TimeOfImpact(remaining, posA,posB,velA,velB,accA,accB,vertA,vertB, tempDepth, tempNormal);
//			if (toi < minTOI) {
//
//				minTOI = toi;
//				bestPair = t;
//				indx = i;
//				finalDepth = tempDepth;
//				finalNormal = tempNormal;
//			}
//		}
//		if (bestPair.empty()) break;
//
//		for (int idx : dynamicBodyList) {
//			bodyList[idx].updateMovement(minTOI, gravity, iterations);
//		}
//
//		EpsilonVector c1, c2;
//		int count = 0;
//		FindContactPoints(vertA, vertB, posA, posB, c1, c2, count);
//
//		CollisionManifold manifold(posA, posB, c1, c2, finalNormal, finalDepth, count);
//		ResolveCollisonWithRotationAndFriction(manifold);
//
//		bodyA.isSleeping = false;
//		bodyB.isSleeping = false;
//
//		remaining -= minTOI;
//		contactPairs.erase(contactPairs.begin() + indx);
//	}
//	dt = remaining;
//}
//
//
static uint64_t g_frameCounter = 0;
static int g_debugEntityA = 1, g_debugEntityB = 2;
void ResolveCollisonWithRotationAndFriction(EpsilonWorld& world, CollisionManifoldDOD& manifold,float dt);
void NarrowPhase(EpsilonWorld& world, float dt) {
	ZoneScoped;
	const uint32_t numTasks = world.scheduler.GetNumTaskThreads() + 1;
	vector<vector<CollisionManifoldDOD>> manifs(numTasks);
	enki::TaskSet task(
		world.contactPairs.size(),
		[&](enki::TaskSetPartition range, uint32_t threadnum)
		{
			ZoneScoped;
			for (uint32_t i = range.start; i < range.end; i++) {
				vector<int> t = world.contactPairs[i];
				Entity entityA = world.entityList[world.idToIndex[t[0]]];
				Entity entityB = world.entityList[world.idToIndex[t[1]]];
				
				Position* pBuf = world.positions.dense.data();
				Transform* vBuf = world.transforms.dense.data();
				AngularVelocity* avBuf = world.angVelocities.dense.data();
				inverseSim* iBuf = world.invSims.dense.data();

				if (entityA.shapetype > entityB.shapetype) {
					swap(entityA, entityB);
				}
				Position& pA = pBuf[world.positions.getInternalIndex(entityA.id)];
				Position& pB = pBuf[world.positions.getInternalIndex(entityB.id)];

				inverseSim& invsimA = iBuf[world.invSims.getInternalIndex(entityA.id)];
				inverseSim& invsimB = iBuf[world.invSims.getInternalIndex(entityB.id)];
				float depth = 0;
				EpsilonVector normal(0, 0);
				if (entityA.sleepTimer > world.sleepThreshold && entityB.sleepTimer > world.sleepThreshold || world.transforms.sparse[entityB.id] == -1
					&& entityA.sleepTimer > world.sleepThreshold || entityB.sleepTimer > world.sleepThreshold && world.transforms.sparse[entityA.id] == -1) continue;
				
				bool collided = false;
				int refPoly = 0, refEdgeIndex = 0;
				if (entityA.shapetype == circle && entityB.shapetype == circle) {
					Circle* cBuf = world.circles.dense.data();
					Circle& cA = cBuf[world.circles.getInternalIndex(entityA.id)];
					Circle& cB = cBuf[world.circles.getInternalIndex(entityB.id)];
					collided = IntersectCircles(cA.radius, cB.radius, pA.value, pB.value, normal, depth);
				}
				else if (entityA.shapetype == circle && (entityB.shapetype == box || entityB.shapetype == triangle)) {
					
					Circle* cBuf = world.circles.dense.data();
					Angle* aBuf = world.angles.dense.data();
					Vertices* vertBuf = world.verts.dense.data();
					Circle& cA = cBuf[world.circles.getInternalIndex(entityA.id)];
					Angle& angB = aBuf[world.angles.getInternalIndex(entityB.id)];
					Vertices& vertB = vertBuf[world.verts.getInternalIndex(entityB.id)];
					collided = IntersectPolygonAndCircle(pA.value, cA.radius, 
						GetTransformedVertices(pB,vertB,angB), normal, depth);
				}
				else {
					Angle* aBuf = world.angles.dense.data();
					Vertices* vertBuf = world.verts.dense.data();
					Angle& angB = aBuf[world.angles.getInternalIndex(entityB.id)];
					Vertices& vertB = vertBuf[world.verts.getInternalIndex(entityB.id)];
					Angle& angA = aBuf[world.angles.getInternalIndex(entityA.id)];
					Vertices& vertA = vertBuf[world.verts.getInternalIndex(entityA.id)];
					collided = IntersectPolygons(pA.value, GetTransformedVertices(pA, vertA, angA),
						pB.value, GetTransformedVertices(pB, vertB, angB), normal, depth,refPoly,refEdgeIndex);
				}
				static bool seenFirstContact = false; // per-entity would be better, but quick global test is fine
				if (!seenFirstContact) {
					printf("[FIRST CONTACT] depth=%.5f\n", depth);
					seenFirstContact = true;
				}
				if (collided) {
					entityA.sleepTimer = 0;
					entityB.sleepTimer = 0;
					//SeparateBodies(pA, pB, invsimA, invsimB, normal, depth);

					EpsilonVector contact1(0, 0);
					EpsilonVector contact2(0, 0);
					int contactCount = 0;
					//FindContactPoints(bodyA, bodyB, contact1, contact2, contactCount);
					if (entityA.shapetype == circle && entityB.shapetype == circle) {
						Circle* cBuf = world.circles.dense.data();
						Circle& cA = cBuf[world.circles.getInternalIndex(entityA.id)];
						FindCirclesContactPoint(pA.value, pB.value, cA.radius, contact1);
						contactCount = 1;
					}
					else if (entityA.shapetype == circle && (entityB.shapetype == box || entityB.shapetype == triangle)) {
						Circle* cBuf = world.circles.dense.data();
						Angle* aBuf = world.angles.dense.data();
						Vertices* vertBuf = world.verts.dense.data();
						Circle& cA = cBuf[world.circles.getInternalIndex(entityA.id)];
						Angle& angB = aBuf[world.angles.getInternalIndex(entityB.id)];
						Vertices& vertB = vertBuf[world.verts.getInternalIndex(entityB.id)];
						FindCirclePolygonContactPoint(pA.value, cA.radius, pB.value,
							GetTransformedVertices(pB, vertB, angB), contact1);
						contactCount = 1;
					}
					else {
						Angle* aBuf = world.angles.dense.data();
						Vertices* vertBuf = world.verts.dense.data();
						Angle& angB = aBuf[world.angles.getInternalIndex(entityB.id)];
						Vertices& vertB = vertBuf[world.verts.getInternalIndex(entityB.id)];
						Angle& angA = aBuf[world.angles.getInternalIndex(entityA.id)];
						Vertices& vertA = vertBuf[world.verts.getInternalIndex(entityA.id)];
						FindPolygonsContactPointsClipped(GetTransformedVertices(pA, vertA, angA),
							GetTransformedVertices(pB, vertB, angB),normal,refPoly,refEdgeIndex, contact1, contact2, contactCount);
					}
					//if (contactCount == 0) printf("refPoly=%d contactCount=0 (dropped!)\n", refPoly);
					CollisionManifoldDOD contact(entityA.id,entityB.id, contact1, contact2, normal, depth, contactCount);
					contact.posIdxA = world.positions.getInternalIndex(entityA.id);
					contact.posIdxB = world.positions.getInternalIndex(entityB.id);

					contact.angleIdxA = world.angles.getInternalIndex(entityA.id);
					contact.angleIdxB = world.angles.getInternalIndex(entityB.id);

					contact.velIdxA = world.transforms.getInternalIndexForStatic(entityA.id);
					contact.velIdxB = world.transforms.getInternalIndexForStatic(entityB.id);

					contact.invIdxA = world.invSims.getInternalIndex(entityA.id);
					contact.invIdxB = world.invSims.getInternalIndex(entityB.id);

					contact.fricIdxA = world.frictionsAndResitutions.getInternalIndex(entityA.id);
					contact.fricIdxB = world.frictionsAndResitutions.getInternalIndex(entityB.id);
					uint64_t key = (uint64_t)min(entityA.id, entityB.id) << 32 | (uint32_t)max(entityA.id, entityB.id);
					auto it = lower_bound(world.prevManifoldsDOD.begin(), world.prevManifoldsDOD.end(), ImpulseCache{key});
					vector<EpsilonVector> contactList = { contact1,contact2 };
					//printf("[NP] A=%d B=%d normal=(%.5f,%.5f) refPoly=%d refEdge=%d depth=%.5f c1=(%.4f,%.4f) c2=(%.4f,%.4f) cc=%d\n",
						//entityA.id, entityB.id, normal.x, normal.y, refPoly, refEdgeIndex, depth,
						//contact1.x, contact1.y, contact2.x, contact2.y, contactCount);
					if ((entityA.id == g_debugEntityA && entityB.id == g_debugEntityB) ||
						(entityA.id == g_debugEntityB && entityB.id == g_debugEntityA)) {
						printf("[F%llu][NP] normal=(%.5f,%.5f) depth=%.5f cc=%d c1=(%.4f,%.4f) c2=(%.4f,%.4f)\n",
							g_frameCounter, normal.x, normal.y, depth, contactCount,
							contact1.x, contact1.y, contact2.x, contact2.y);
					}
					if (it != world.prevManifoldsDOD.end() && it->key == key) {
						
						
						
							for (int i = 0; i < contactCount; i++) {
								//ZoneScoped;
								Position& pA = pBuf[contact.posIdxA];
								Position& pB = pBuf[contact.posIdxB];

								Transform& tA = vBuf[contact.velIdxA];
								Transform& tB = vBuf[contact.velIdxB];

								AngularVelocity& angvelA = avBuf[contact.velIdxA];
								AngularVelocity& angvelB = avBuf[contact.velIdxB];

								inverseSim& invsimA = iBuf[contact.invIdxA];
								inverseSim& invsimB = iBuf[contact.invIdxB];
								float perc = 1.0f;
								contact.accumulatedNormalImpulse[i] = it->accumulatedNormalImpulse[i];
								
								contact.accumulatedTangentImpulse[i] = it->accumulatedTangentImpulse[i];
								contact.tangentList[i] = it->tangentList[i];
								tA.velocity += -it->accumulatedNormalImpulse[i] * invsimA.inverseMass * contact.normal * perc;
								tB.velocity += it->accumulatedNormalImpulse[i] * invsimB.inverseMass * contact.normal * perc;
								tA.velocity += -it->accumulatedTangentImpulse[i] * invsimA.inverseMass * contact.tangentList[i] * perc;
								tB.velocity += it->accumulatedTangentImpulse[i] * invsimB.inverseMass * contact.tangentList[i] * perc;
								EpsilonVector ra = contactList[i] - pA.value;
								EpsilonVector rb = contactList[i] - pB.value;
								angvelA.value += -ra.Cross(it->accumulatedNormalImpulse[i] * contact.normal) * invsimA.inverseInertia * perc;
								angvelB.value += rb.Cross(it->accumulatedNormalImpulse[i] * contact.normal) * invsimB.inverseInertia * perc;
								angvelA.value += -ra.Cross(it->accumulatedTangentImpulse[i] * contact.tangentList[i]) * invsimA.inverseInertia * perc;
								angvelB.value += rb.Cross(it->accumulatedTangentImpulse[i] * contact.tangentList[i]) * invsimB.inverseInertia * perc;
							}



						

					}

					manifs[threadnum].emplace_back(contact);




					//ResolveCollisonWithRotationAndFriction(world,contact,dt);
				}


			}
		});
	world.scheduler.AddTaskSetToPipe(&task);
	world.scheduler.WaitforTask(&task);
	{
		ZoneScopedN("prevmanifolds");
		world.prevManifoldsDOD.clear();

		for (auto& res : manifs) {
			for (auto& m : res) {
				world.manifolds.push_back(m);
			}
		}
	}
}



void Solver(EpsilonWorld& world, int start, int end, float dt) {
	ZoneScoped;
	
	for (int i = 0; i < 20; i++) {
		for (int i = start; i < end; i++) {

			ResolveCollisonWithRotationAndFriction(world, world.manifolds[i], dt);
		}
	}
	world.prevManifoldsDOD.clear();
	for (int i = start; i < end; i++) {
		auto& m = world.manifolds[i];
		uint64_t key = (uint64_t)min(m.entityA_ID, m.entityB_ID) << 32 | (uint32_t)max(m.entityA_ID, m.entityB_ID);

		world.prevManifoldsDOD.push_back({ key,
			{m.tangentList[0], m.tangentList[1]},
			{m.accumulatedNormalImpulse[0], m.accumulatedNormalImpulse[1]},
			{m.accumulatedTangentImpulse[0], m.accumulatedTangentImpulse[1]}
			});
	}

	std::sort(world.prevManifoldsDOD.begin(), world.prevManifoldsDOD.end());
	/*for (int j = 0; j < 2; j++) {
		for (int i = 0; i < world.manifolds.size(); i++) {
			CollisionManifoldDOD& manifold = world.manifolds[i];
			Position* pBuf = world.positions.dense.data();
			inverseSim* iBuf = world.invSims.dense.data();
			Position& pA = pBuf[manifold.posIdxA];
			Position& pB = pBuf[manifold.posIdxB];
			inverseSim& invsimA = iBuf[manifold.invIdxA];
			inverseSim& invsimB = iBuf[manifold.invIdxB];
			SeparateBodies(pA, pB, invsimA, invsimB, world.manifolds[i].normal, world.manifolds[i].depth);
		}
	}*/
}


void BuildIslands(EpsilonWorld& world)
{
	int n = world.entityList.size();
	DSU dsu(n);

	for (int i = 0; i < world.contactPairs.size(); i++) {
		Entity entityA = world.entityList[world.idToIndex[world.contactPairs[i][0]]];
		Entity entityB = world.entityList[world.idToIndex[world.contactPairs[i][1]]];
		if(world.transforms.sparse[entityB.id] == -1 || world.transforms.sparse[entityA.id] == -1)
			continue;
		dsu.unite(world.entityList[world.idToIndex[world.contactPairs[i][0]]].id, world.entityList[world.idToIndex[world.contactPairs[i][1]]].id);
	}

	std::unordered_map<int, int> rootToIslandIdx;
	world.dodislands.clear();

	for (int i = 0; i < n; i++) {
		Entity entity = world.entityList[i];
		if (world.transforms.sparse[entity.id] == -1) continue;
		int root = dsu.find(i);
		if (rootToIslandIdx.find(root) == rootToIslandIdx.end()) {
			rootToIslandIdx[root] = world.dodislands.size();
			world.dodislands.emplace_back();
		}
		world.dodislands[rootToIslandIdx[root]].EntityIds.emplace_back(i);
	}

}
void PutEntityToSleep(EpsilonWorld& world, const int& entityId) {
	int idx = world.transforms.getInternalIndex(entityId);
	if (idx >= world.activeObjectCount) return; // already asleep

	int lastActive = world.activeObjectCount - 1;

	// swap this entity with the last active slot across every array
	// that's iterated in parallel (synchronized) fashion
	world.transforms.Swap(idx, lastActive);
	world.angVelocities.Swap(idx, lastActive);
	world.pseudoVels.Swap(idx, lastActive);
	world.pseudoAngVels.Swap(idx, lastActive);
	// add any other array that UpdateMovement/gravity/etc. index by the same `i`

	world.activeObjectCount--;
}

void WakeEntity(EpsilonWorld& world, const int& entityId) {
	int idx = world.transforms.getInternalIndex(entityId);
	if (idx < world.activeObjectCount) return; // already awake

	int firstSleeping = world.activeObjectCount;

	world.transforms.Swap(idx, firstSleeping);
	world.angVelocities.Swap(idx, firstSleeping);
	world.pseudoVels.Swap(idx, firstSleeping);
	world.pseudoAngVels.Swap(idx, firstSleeping);

	world.activeObjectCount++;
}
void SolveIslands(EpsilonWorld& world, int start, int end, float dt, int iterations)
{

	for (int i = start; i < end; ++i) {
		IslandDOD& island = world.dodislands[i];

		float maxEnergy = 0.0f;
		for (int bIdx : island.EntityIds) {
			float energy = world.transforms.get(bIdx).velocity.LengthSquared() + (world.angVelocities.get(bIdx).value * world.angVelocities.get(bIdx).value);
			if (energy > maxEnergy) maxEnergy = energy;
		}
		int sleep = 1;
		

		if (maxEnergy < 0.8f) {
			for (int bIdx : island.EntityIds) {
				world.entityList[bIdx].sleepTimer += dt / iterations;
				if (world.entityList[bIdx].sleepTimer < world.sleepThreshold) {
					sleep = -1;
				}
			}
		}
		else {
			sleep = -1;
			for (int bIdx : island.EntityIds) {
				world.entityList[bIdx].sleepTimer = 0.0f;
			}
		}

		if (sleep == 1) {
			for (int bIdx : island.EntityIds) {
				PutEntityToSleep(world, world.entityList[bIdx].id);
			}
		}
		else if (sleep == -1) {
			for (int bIdx : island.EntityIds) {
				WakeEntity(world, world.entityList[bIdx].id);
			}
		}
	}
}


void UpdateMovement(uint32_t start, uint32_t end, EpsilonWorld& world, float dt, int iterations) {
	ZoneScoped;
	dt /= iterations;
	for (int i = start; i < end; i++) {
		
			//bodyList[islands[i].bodyIndices[j]].updateMovement(dt, gravity, iterations);
			int entId = world.transforms.getEntity(i);
			Entity ent = world.entityList[world.idToIndex[entId]];
			AABB& aabb = world.aabbs.get(entId);
			Position& pos = world.positions.get(world.transforms.getEntity(i));
			Angle& ang = world.angles.get(world.angVelocities.getEntity(i));
			world.transforms.dense[i].acceleration += world.gravity;
			world.transforms.dense[i].velocity += world.transforms.dense[i].acceleration * dt;
			ang.value += (world.angVelocities.dense[i].value+world.pseudoAngVels.dense[i].value) * dt;
			pos.value += (world.transforms.dense[i].velocity+ world.pseudoVels.dense[i].velocity) * dt;
			world.transforms.dense[i].acceleration = EpsilonVector(0,0);
			world.pseudoAngVels.dense[i].value = 0.0f;
			world.pseudoVels.dense[i].velocity = EpsilonVector(0, 0);
			if (ent.shapetype == circle) {
				Circle c = world.circles.get(entId);
				UpdateCircleAABB(aabb, pos, c);
			}
			else {
				Vertices& vert = world.verts.get(entId);
				Angle& ang = world.angles.get(entId);
				UpdatePolyAABB(aabb, pos, vert, ang);
			}
			if (entId == g_debugEntityA || entId == g_debugEntityB) {
				printf("[F%llu][INTEGRATE] id=%d pos=(%.5f,%.5f) angle=%.5f vel=(%.5f,%.5f) angvel=%.5f\n",
					g_frameCounter, entId, pos.value.x, pos.value.y, ang.value,
					world.transforms.dense[i].velocity.x, world.transforms.dense[i].velocity.y,
					world.angVelocities.dense[i].value);
			}
	}

}
//
//
void ResolveCollisonWithRotationAndFriction(EpsilonWorld& world, CollisionManifoldDOD& manifold, float dt)
{
	ZoneScoped;
	Position* pBuf = world.positions.dense.data();
	Angle* aBuf = world.angles.dense.data();
	Transform* vBuf = world.transforms.dense.data();
	AngularVelocity* avBuf = world.angVelocities.dense.data();
	AngularVelocity* pseudoavBuf = world.pseudoAngVels.dense.data();
	inverseSim* iBuf = world.invSims.dense.data();
	FrictionAndRestitution* fBuf = world.frictionsAndResitutions.dense.data();
	Transform* pseudovBuf = world.pseudoVels.dense.data();

	Position& pA = pBuf[manifold.posIdxA];
	Position& pB = pBuf[manifold.posIdxB];
	Transform& tA = vBuf[manifold.velIdxA];
	Transform& tB = vBuf[manifold.velIdxB];
	Transform& psA = pseudovBuf[manifold.velIdxA];
	Transform& psB = pseudovBuf[manifold.velIdxB];
	AngularVelocity& angvelA = avBuf[manifold.velIdxA];
	AngularVelocity& angvelB = avBuf[manifold.velIdxB];
	AngularVelocity& psangvelA = pseudoavBuf[manifold.velIdxA];
	AngularVelocity& psangvelB = pseudoavBuf[manifold.velIdxB];
	FrictionAndRestitution& fA = fBuf[manifold.fricIdxA];
	FrictionAndRestitution& fB = fBuf[manifold.fricIdxB];
	inverseSim& invsimA = iBuf[manifold.invIdxA];
	inverseSim& invsimB = iBuf[manifold.invIdxB];

	EpsilonVector normal = manifold.normal;
	EpsilonVector contactList[2] = { manifold.contact1, manifold.contact2 };
	int contactCount = manifold.contactCount;

	float e = min(fA.restitution, fB.restitution);
	float sf = (fA.staticFriction + fB.staticFriction) * 0.5f;
	float df = (fA.dynamicFriction + fB.dynamicFriction) * 0.5f;
	float slop = 0.001f;
	float beta = 0.3f;
	//float maxBiasSpeed = 2.0f; // Limit position correction speed (m/s)
	float bias = (beta / dt) * max(0.0f, manifold.depth - slop);
	float linearDamping = 1.0f;
	float angularDamping = 1.0f;
	// ==========================================
	// 1. SOLVE NORMAL IMPULSES & PENETRATION
	// ==========================================
	bool dbg = (manifold.entityA_ID == g_debugEntityA && manifold.entityB_ID == g_debugEntityB) ||
		(manifold.entityA_ID == g_debugEntityB && manifold.entityB_ID == g_debugEntityA);
	EpsilonVector avA_before = tA.velocity, avB_before = tB.velocity;
	float angA_before = angvelA.value, angB_before = angvelB.value;

	for (size_t i = 0; i < contactCount; i++) {
		EpsilonVector ra = contactList[i] - pA.value;
		EpsilonVector rb = contactList[i] - pB.value;
		EpsilonVector raPerp(-ra.y, ra.x);
		EpsilonVector rbPerp(-rb.y, rb.x);

		EpsilonVector angularLinearVelocityA = raPerp * angvelA.value;
		EpsilonVector angularLinearVelocityB = rbPerp * angvelB.value;
		EpsilonVector relativeVelocity = (tB.velocity + angularLinearVelocityB) - (tA.velocity + angularLinearVelocityA);

		float contactVelocityMag = relativeVelocity.Dot(normal);

		// FIX 3: Restitution threshold to prevent micro-jitter when stacked
		float restitutionBias = 0.0f;
		if (contactVelocityMag < -1.0f) { // Only bounce if closing fast enough
			restitutionBias = -e * contactVelocityMag;
		}
		restitutionBias = 0.0f;
		float j = -(contactVelocityMag)+restitutionBias;

		float raPerpDotN = raPerp.Dot(normal);
		float rbPerpDotN = rbPerp.Dot(normal);
		float denom = invsimA.inverseMass + invsimB.inverseMass +
			(raPerpDotN * raPerpDotN * invsimA.inverseInertia) +
			(rbPerpDotN * rbPerpDotN * invsimB.inverseInertia);

		j /= denom;
		//j /= contactCount;
		float oldImp = manifold.accumulatedNormalImpulse[i];
		manifold.accumulatedNormalImpulse[i] = max(0.0f, oldImp + j);

		// Calculate the actual delta applied this frame
		float j_delta = manifold.accumulatedNormalImpulse[i] - oldImp;
		EpsilonVector impulse = j_delta * normal;

		tA.velocity += -impulse * invsimA.inverseMass;
		angvelA.value += -ra.Cross(impulse) * invsimA.inverseInertia;
		tB.velocity += impulse * invsimB.inverseMass;
		angvelB.value += rb.Cross(impulse) * invsimB.inverseInertia;
		
		// --- PSEUDO VELOCITY (Position Correction) ---
		EpsilonVector psRelVel = (psB.velocity + rbPerp * psangvelB.value) -
			(psA.velocity + raPerp * psangvelA.value);
		float psRelVelMag = psRelVel.Dot(normal);

		float jsplit = (bias - psRelVelMag) / denom;
		if (jsplit < 0.0f) jsplit = 0.0f;
		//jsplit = min(jsplit, maxBiasSpeed * 50.0f);
		EpsilonVector splitImp = jsplit * normal;
		psA.velocity += -splitImp * invsimA.inverseMass;
		psB.velocity += splitImp * invsimB.inverseMass;

		// MISSING LINES TO ADD: 
		// You MUST apply angular position correction, or boxes will slide sideways!
		//psangvelA.value += -ra.Cross(splitImp) * invsimA.inverseInertia;
		//psangvelB.value += rb.Cross(splitImp) * invsimB.inverseInertia;
		if (dbg) {
			printf("[F%llu][PSEUDO] i=%zu bias=%.5f jsplit=%.5f psangvelA=%.5f psangvelB=%.5f\n",
				g_frameCounter, i, bias, jsplit, psangvelA.value, psangvelB.value);
		}
	}

	// ==========================================
	// 2. SOLVE FRICTION (TANGENT) IMPULSES
	// ==========================================
	for (size_t i = 0; i < contactCount; i++) {
		EpsilonVector ra = contactList[i] - pA.value;
		EpsilonVector rb = contactList[i] - pB.value;
		EpsilonVector raPerp(-ra.y, ra.x);
		EpsilonVector rbPerp(-rb.y, rb.x);

		EpsilonVector angularLinearVelocityA = raPerp * angvelA.value;
		EpsilonVector angularLinearVelocityB = rbPerp * angvelB.value;
		EpsilonVector relativeVelocity = (tB.velocity + angularLinearVelocityB) - (tA.velocity + angularLinearVelocityA);

		// FIX 1: Tangent is strictly perpendicular to normal. Never derive it from velocity!
		EpsilonVector tangent = EpsilonVector(-normal.y, normal.x);
		tangent = tangent.Normalized(); // just to be safe, since normal itself isn't always perfectly unit length
		manifold.tangentList[i] = tangent; // <-- add this line, write the real tangent back to the manifold

		float jt = -relativeVelocity.Dot(tangent);

		float raPerpDotT = raPerp.Dot(tangent);
		float rbPerpDotT = rbPerp.Dot(tangent);
		float denom = invsimA.inverseMass + invsimB.inverseMass +
			(raPerpDotT * raPerpDotT * invsimA.inverseInertia) +
			(rbPerpDotT * rbPerpDotT * invsimB.inverseInertia);

		jt /= denom;
		//jt /= contactCount;
		// FIX 2: Correct Friction Accumulation against the TOTAL normal impulse
		
		float oldTangImp = manifold.accumulatedTangentImpulse[i];

		// Use static friction limit based on the total accumulated weight
		float maxFriction = manifold.accumulatedNormalImpulse[i] * sf;

		manifold.accumulatedTangentImpulse[i] = clamp(oldTangImp + jt, -maxFriction, maxFriction);

		// Calculate the actual delta applied this frame
		float jf_delta = manifold.accumulatedTangentImpulse[i] - oldTangImp;
		EpsilonVector frictionImpulse = jf_delta * tangent;
		
			tA.velocity += -frictionImpulse * invsimA.inverseMass;
			tB.velocity += frictionImpulse * invsimB.inverseMass;
		
		angvelA.value += -ra.Cross(frictionImpulse) * invsimA.inverseInertia;
		angvelB.value += rb.Cross(frictionImpulse) * invsimB.inverseInertia;
	}
	if (dbg) {
		printf("[F%llu][SOLVE] cc=%d normJ=(%.5f,%.5f) tanJ=(%.5f,%.5f) angvelA=%.5f->%.5f angvelB=%.5f->%.5f velA=(%.4f,%.4f)->(%.4f,%.4f) velB=(%.4f,%.4f)->(%.4f,%.4f)\n",
			g_frameCounter, contactCount,
			manifold.accumulatedNormalImpulse[0], manifold.accumulatedNormalImpulse[1],
			manifold.accumulatedTangentImpulse[0], manifold.accumulatedTangentImpulse[1],
			angA_before, angvelA.value, angB_before, angvelB.value,
			avA_before.x, avA_before.y, tA.velocity.x, tA.velocity.y,
			avB_before.x, avB_before.y, tB.velocity.x, tB.velocity.y);
	}
	tA.velocity = tA.velocity * linearDamping;
	tB.velocity = tB.velocity * linearDamping;
	angvelA.value = angvelA.value * angularDamping;
	angvelB.value = angvelB.value * angularDamping;
}
//void ResolveThreadConnection(int start, int end, EpsilonWorld& world) {
//
//	for (int i = start; i < end; i++) {
//		if (islands[i].isAsleep) {
//			continue;
//		}
//		for (int j = 0; j < islands[i].bodyIndices.size(); j++) {
//			if (bodyList[islands[i].bodyIndices[j]].originPosition.Distance(bodyList[islands[i].bodyIndices[j]].connectionPosition) > bodyList[islands[i].bodyIndices[j]].connectionDistance) {
//				EpsilonVector dir = bodyList[islands[i].bodyIndices[j]].connectionPosition - bodyList[islands[i].bodyIndices[j]].originPosition;
//				float dist = dir.Length();
//				float restDist = dist - bodyList[islands[i].bodyIndices[j]].connectionDistance;
//				if (restDist > 0.01f) {
//					restDist = 0.01f;
//				}
//				float damperForce = -(damperThreadConstant * bodyList[islands[i].bodyIndices[j]].linearVelocity.Dot(dir)) / dist;
//				bodyList[islands[i].bodyIndices[j]].linearVelocity += -dir * (bodyList[islands[i].bodyIndices[j]].inverseMass * restDist);
//				bodyList[islands[i].bodyIndices[j]].AddForce(damperForce * dir);
//				EpsilonVector offset = bodyList[islands[i].bodyIndices[j]].connectionPosition - bodyList[islands[i].bodyIndices[j]].position;
//				bodyList[islands[i].bodyIndices[j]].angularVelocity += offset.Cross(-dir * (bodyList[islands[i].bodyIndices[j]].inverseMass * restDist));
//			}
//		}
//	}
//}
//void ResolveSpringConnection(int start, int end, float dt, int iterations, EpsilonWorld& world) {
//
//	dt = dt / iterations;
//	for (int i = start; i < end; i++) {
//		if (islands[i].isAsleep) {
//			continue;
//		}
//		for (int j = 0; j < islands[i].bodyIndices.size(); j++) {
//			EpsilonVector dir = bodyList[islands[i].bodyIndices[j]].connectionPosition - bodyList[islands[i].bodyIndices[j]].originPosition;
//			float dist = dir.Length();
//			float restDist = dist - bodyList[islands[i].bodyIndices[j]].connectionDistance;
//			float springForce = restDist * springConstant;
//
//			float damperForce = (damperConstant * (bodyList[islands[i].bodyIndices[j]].linearVelocity.Dot(dir)) / dist);
//			EpsilonVector force = -(springForce + damperForce) * dir / dist;
//			EpsilonVector offset = bodyList[islands[i].bodyIndices[j]].connectionPosition - bodyList[islands[i].bodyIndices[j]].position;
//			float angularResistance = offset.Cross(force * bodyList[islands[i].bodyIndices[j]].inverseInertia);
//			bodyList[islands[i].bodyIndices[j]].angularVelocity += angularResistance * bodyList[islands[i].bodyIndices[j]].inverseMass * dt;
//			bodyList[islands[i].bodyIndices[j]].AddForce(force);
//		}
//	}
//}
//
//// the correct version (kinda)
//void Explosion(EpsilonVector position, float magnitude, EpsilonWorld& world)
//{
//
//	for (int i = 0; i < velocities.dense.size(); i++) {
//		//bodyList[dynamicBodyList[i]].isSleeping = false;
//		int id = velocities.getEntity(i);
//		Position& pos = positions.get(id);
//		EpsilonVector dir = pos.value - position;
//		float dist = dir.Length();
//		EpsilonVector impulse = (dir * magnitude) / (dist * dist);
//		EpsilonVector vertical(0, 1.f);
//		float mag = -vertical.Cross(impulse);
//		velocities.dense[i].value += impulse * invMasses.dense[i].value;
//		angularVels.dense[i].value += mag * invInertias.dense[i].value;
//	}
//}
//void Buoyancy(int start, int end, EpsilonWorld& world) {
//	//ZoneScoped;
//	for (int j = 0; j < waterList.size(); j++) {
//		for (int i = start; i < end; i++) {
//			if (islands[i].isAsleep) {
//				continue;
//			}
//			for (int k = 0; k < islands[i].bodyIndices.size(); k++) {
//				Water& w = waterList[j];
//				int32_t id = islands[i].bodyIndices[k];
//				if (IntersectAABB(aabbs[id],waterList[j].dimensions)) {
//					EpsilonVector dir(0, 1.f);
//					float hRatio = aabbs[id].max.y / waterList[j].dimensions.max.y;
//					hRatio = clamp(hRatio, 0.f, 1.f);
//					float damperForce = (damperWaterConstant * velocities[id].value.Dot(dir)) * hRatio;
//					EpsilonVector force = -dir * ((area * 9.81f * w.density) + damperForce);
//					accelerations[id].value += force / invMasses[id].value;
//				}
//				
//			}
//		}
//	}
//}
//
//void CreateWater(EpsilonWorld& world,AABB dimensions, float density)
//{
//	waterList.emplace_back(dimensions,density);
//}
//
//void DeleteWater(vector<Water>& waterList, int index)
//{
//	waterList.erase(waterList.begin() + index);
//}
//
void AirResistance(int start, int end, float dt, int iterations, EpsilonWorld& world)
{

	dt /= iterations;
	for (int i = start; i < end; i++) {
		inverseSim& s = world.invSims.get(world.transforms.getEntity(i));
		EpsilonVector resistance(world.transforms.dense[i].velocity.x * world.transforms.dense[i].velocity.x,
		world.transforms.dense[i].velocity.y * world.transforms.dense[i].velocity.y);
		resistance = -resistance * world.airResistanceConstant;
		float angularResistance = -abs(world.angVelocities.dense[i].value) * world.angVelocities.dense[i].value * world.rotationalAirResistanceConstant;
		world.angVelocities.dense[i].value += (angularResistance * s.inverseInertia) * dt;
		world.transforms.dense[i].acceleration += resistance * s.inverseMass;
		

	}
}
void ApplyGravity(uint32_t start, uint32_t end, EpsilonWorld& world, float dt) {
	for (int i = start; i < end; i++) {
		world.transforms.dense[i].acceleration += world.gravity;
		world.transforms.dense[i].velocity += world.transforms.dense[i].acceleration * dt;
		world.transforms.dense[i].acceleration = EpsilonVector(0, 0);
	}
}

void IntegratePositions(uint32_t start, uint32_t end, EpsilonWorld& world, float dt) {
	for (int i = start; i < end; i++) {
		int entId = world.transforms.getEntity(i);
		Entity ent = world.entityList[world.idToIndex[entId]];
		AABB& aabb = world.aabbs.get(entId);
		Position& pos = world.positions.get(entId);
		Angle& ang = world.angles.get(world.angVelocities.getEntity(i));

		ang.value += (world.angVelocities.dense[i].value + world.pseudoAngVels.dense[i].value) * dt;
		pos.value += (world.transforms.dense[i].velocity + world.pseudoVels.dense[i].velocity) * dt;

		world.pseudoAngVels.dense[i].value = 0.0f;
		world.pseudoVels.dense[i].velocity = EpsilonVector(0, 0);

		if (ent.shapetype == circle) {
			Circle c = world.circles.get(entId);	
			UpdateCircleAABB(aabb, pos,c);
		}
		else {
			Vertices& vert = world.verts.get(entId);
			UpdatePolyAABB(aabb, pos, vert, ang);
		}
	}
}
void WorldStep(EpsilonWorld& world, float dt, float iterations) {
	ZoneScoped;
	g_frameCounter++;
	world.contactPairs.clear();
	world.manifolds.clear();
	world.dodislands.clear();
	BroadPhase(world);
	BuildIslands(world);
	SolveIslands(world, 0, world.dodislands.size(), dt, iterations);
	NarrowPhase(world, dt);
	
	
	
		
			Solver(world, 0, world.manifolds.size(), dt);
			
	
	/*for (int i = 0; i < 3; i++) {
		RunTask(world, world.manifolds.size(), [&world, &dt](uint32_t start, uint32_t end, uint32_t threadNum) {
			PositionSolver(world, start, end, dt);
			});
	}*/
	
			UpdateMovement(1, world.activeObjectCount, world, dt, 1);
		

	/*RunTask(world, world.transforms.Size() - 1, [&world, &dt](uint32_t start, uint32_t end, uint32_t threadNum) {
		AirResistance(start + 1, end + 1, dt, 1, world);
		});*/
}