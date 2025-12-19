#pragma once
#include<vector>
#include<mutex>
#include<condition_variable>
#include<Tracy.hpp>
#include<enkiTS/TaskScheduler.h>
#include"EpsilonBody.h"
#include"Collisions.h"
#include"CollisionManifold.h"
#include"EpsilonVector.h"
#include"Water.h"
class QuadTree {
public:
	int nodeCapacity;
	AABB aabb;
	vector<int> bodies;
	std::unique_ptr<QuadTree> nw, ne, sw, se;

	QuadTree(AABB ab, int capacity)
		:
		aabb(ab.min.x, ab.max.x, ab.min.y, ab.max.y),
		nodeCapacity(capacity)
	{
	}
	void query(AABB area, vector<int>& found, vector<EpsilonBody>& list) {

		if (!Collisions::IntersectAABB(area, aabb)) {
			return;
		}

		if (divided) {
			nw->query(area, found, list);
			ne->query(area, found, list);
			sw->query(area, found, list);
			se->query(area, found, list);
		}
		for (int i = 0; i < bodies.size(); i++) {
			if (Collisions::IntersectAABB(area, list[bodies[i]].GetAABB())) {
				found.push_back(bodies[i]);
			}
		}
	}

	bool insert(EpsilonBody& body, vector<EpsilonBody>& list, int index) {

		if (!Collisions::IntersectAABB(aabb, body.GetAABB())) {
			return false;
		}

		if (!divided) {
			if (bodies.size() < nodeCapacity) {
				bodies.push_back(index);
				return true;
			}
			subdivide(list);
		}

		if (Collisions::ContainsAABB(body.GetAABB(), nw->aabb)) return nw->insert(body, list, index);
		if (Collisions::ContainsAABB(body.GetAABB(), ne->aabb)) return ne->insert(body, list, index);
		if (Collisions::ContainsAABB(body.GetAABB(), sw->aabb)) return sw->insert(body, list, index);
		if (Collisions::ContainsAABB(body.GetAABB(), se->aabb)) return se->insert(body, list, index);
		bodies.push_back(index);
		return true;
	}

private:
	bool divided = false;
	void subdivide(vector<EpsilonBody>& list) {
		float midX = (aabb.min.x + aabb.max.x) / 2.f;
		float midY = (aabb.min.y + aabb.max.y) / 2.f;

		nw = make_unique<QuadTree>(AABB(aabb.min.x, midX, aabb.min.y, midY), nodeCapacity);
		ne = make_unique<QuadTree>(AABB(midX, aabb.max.x, aabb.min.y, midY), nodeCapacity);
		sw = make_unique<QuadTree>(AABB(aabb.min.x, midX, midY, aabb.max.y), nodeCapacity);
		se = make_unique<QuadTree>(AABB(midX, aabb.max.x, midY, aabb.max.y), nodeCapacity);

		divided = true;
		vector<int> parentBodies = move(bodies);
		bodies.clear();

		for (int bd : parentBodies) {
			insert(list[bd], list, bd);
		}
	}
};
class EpsilonWorld
{
public:
	enki::TaskScheduler scheduler;
	mutex mtx;
	condition_variable cv;
	void RunTask(uint32_t count, std::function<void(uint32_t, uint32_t, uint32_t)> func);
	EpsilonWorld(int windowWidth,int windowHeight, float zoom);
	int GetBodyCount();
	int GetWaterCount();
	void AddBody(EpsilonBody body);
	void RemoveBody(int index);
	int GetDynamicBodyCount();
	void Explosion(EpsilonVector position, float radius, float magnitude);
	void CreateWater(EpsilonVector surfacePosition, float width, float depth, float density);
	void DeleteWater(int index);
	void Update(float dt, int iterations);
	EpsilonBody GetBody(float index);
	Water GetWater(int index);
	EpsilonBody GetDynamicBody(float index);
private:
	float airResistanceConstant;
	float rotationalAirResistanceConstant;
	float springConstant, damperConstant, damperThreadConstant, damperWaterConstant;
	float depth;
	float zoom;
	int windowWidth, windowHeight;
	EpsilonVector gravity;
	EpsilonVector normal;
	vector<Water> waterList;
	vector<EpsilonBody> bodyList;
	vector<int> dynamicBodyList;
	vector<vector<int>> contactPairs;
	vector<int> potentialColliders;
	vector<int> highPriorityObjects;
	vector<int> lowPriorityObjects;
	void PreFiltering();
	void UpdateMovement(uint32_t start, uint32_t end, float dt, int iterations);
	void UpdateHighPriorityMovement(uint32_t start, uint32_t end, float dt, int iterations);
	void SeperateBodies(EpsilonBody& bodyA, EpsilonBody& bodyB, EpsilonVector mtv);
	void BroadPhase(int windowWidth = 1280, int windowHeight = 720, float zoom = 1.f);
	void NarrowPhase(int start, int end);
	void ResolveCollisonBasic(CollisionManifold& manifold);
	void ResolveCollisonWithRotation(CollisionManifold& manifold);
	void ResolveCollisonWithRotationAndFriction(CollisionManifold& manifold);
	void ZoZoResolveCollisonBasic(CollisionManifold& manifold);
	void ResolveThreadConnection(int start, int end);
	void ResolveSpringConnection(int start, int end,float dt, int iterations);
	void ResolveHighPriorityThreadConnection(int start, int end);
	void ResolveHighPrioritySpringConnection(int start, int end, float dt, int iterations);
	void Buoyancy(int start, int end);
	void AirResistance(int start, int end, float dt, int iterations);

};

