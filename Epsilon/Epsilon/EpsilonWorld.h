#pragma once
#include<vector>
#include<mutex>
#include<condition_variable>
#include<Tracy.hpp>
#include<enkiTS/TaskScheduler.h>
#include<algorithm>
#include<map>
#include"EpsilonBody.h"
#include"Collisions.h"
#include"CollisionManifold.h"
#include"EpsilonVector.h"
#include"Water.h"



struct QuadTree {
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
			if (IntersectAABB(area, list[bodies[i]].GetAABB(list[bodies[i]].usingCCD))) {
				found.push_back(bodies[i]);
			}
		}
	}

	bool insert(EpsilonBody& body, vector<EpsilonBody>& list, int index) {

		if (!IntersectAABB(aabb, body.GetAABB(body.usingCCD))) {
			return false;
		}

		if (!divided) {
			if (bodies.size() < nodeCapacity) {
				bodies.push_back(index);
				return true;
			}
			subdivide(list);
		}

		if (ContainsAABB(body.GetAABB(body.usingCCD), nw->aabb)) return nw->insert(body, list, index);
		if (ContainsAABB(body.GetAABB(body.usingCCD), ne->aabb)) return ne->insert(body, list, index);
		if (ContainsAABB(body.GetAABB(body.usingCCD), sw->aabb)) return sw->insert(body, list, index);
		if (ContainsAABB(body.GetAABB(body.usingCCD), se->aabb)) return se->insert(body, list, index);
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



struct Island {
public:
	vector<int> bodyIndices;
	int sleepTimer;
	bool isAsleep = false;
	vector<CollisionManifold> manifolds;
};



struct DSU {
public:
	vector<int> parent;
	DSU(int n) {
		parent.resize(n);
		for (int i = 0; i < n; i++) parent[i] = i;
	}
	int find(int i) {
		if (parent[i] == i) return i;
		return parent[i] = find(parent[i]);
	}
	void unite(int i, int j) {
		int root_i = find(i);
		int root_j = find(j);
		if (root_i != root_j) parent[root_i] = root_j;
	}
};



struct EpsilonWorld
{
public:
	enki::TaskScheduler scheduler;
	void RunTask(uint32_t count, std::function<void(uint32_t, uint32_t, uint32_t)> func);
	EpsilonWorld(int windowWidth, int windowHeight, int worldWidth, int worldHeight, float zoom);
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
	vector<Entity> entityList;
	vector<Entity> dynamicEntities;
	vector<Entity> staticEntities;
	vector<EpsilonVector> debugpos;
	SparseSet<Transform> transforms;
	SparseSet<Position> positions;
	SparseSet<AABB> aabbs;
	SparseSet<Angle> angles;
	SparseSet<AngularVelocity> angVelocities;
	SparseSet<inverseSim> invSims;
	SparseSet<FrictionAndRestitution> frictionsAndResitutions;
	SparseSet<Vertices> verts;
	//vector<IslandDOD> dodislands;
	float sleepThreshold = 2.f;
	int windowWidth, windowHeight;
	int worldWidth, worldHeight;
	float zoom;
	vector<vector<int>> contactPairs;
	vector<vector<Entity>> contactPairsDOD;
	vector<int> idToIndex;
	SparseSet<Circle> circles;
	map<vector<int>, CollisionManifold> prevManifolds;
	vector<ImpulseCache> prevManifoldsDOD;
	vector<CollisionManifoldDOD> manifolds;
	SparseSet<Transform> pseudoVels;
	SparseSet<AngularVelocity> pseudoAngVels;
	EpsilonVector gravity;
	float airResistanceConstant;
	float rotationalAirResistanceConstant;
private:
	float springConstant, damperConstant, damperThreadConstant, damperWaterConstant;
	float depth;
	float angularVelocityThreshold;
	float linearVelocityThreshold;
	EpsilonVector normal;
	vector<Water> waterList;
	vector<EpsilonBody> bodyList;
	vector<int> dynamicBodyList;
	vector<int> nonStaticBodies;
	vector<int> potentialColliders;
	vector<int> ccdBodies;
	vector<Island> islands;
	void PreFiltering(float dt);
	void UpdateMovement(uint32_t start, uint32_t end, float dt, int iterations);
	void SeparateBodies(EpsilonBody& bodyA, EpsilonBody& bodyB, EpsilonVector mtv, float depth);
	void BroadPhase(int windowWidth = 1280, int windowHeight = 720, float zoom = 1.f);
	float TimeOfImpact(float dt, EpsilonBody& A, EpsilonBody& B, float& depth, EpsilonVector& normal);
	void resolveCCDCollisions(float& dt, int iterations);
	void NarrowPhase(int start, int end, float dt);
	void BuildIslands();
	void SolveIslands(int start, int end, float dt, int iterations);
	void ResolveCollisonBasic(CollisionManifold& manifold);
	void ResolveCollisonWithRotation(CollisionManifold& manifold);
	void ResolveCollisonWithRotationAndFriction(CollisionManifold& manifold);
	void ZoZoResolveCollisonBasic(CollisionManifold& manifold);
	void ResolveThreadConnection(int start, int end);
	void ResolveSpringConnection(int start, int end, float dt, int iterations);
	void Buoyancy(int start, int end);

	void AirResistance(int start, int end, float dt, int iterations);

};


void WorldStep(EpsilonWorld& world, float dt, float iterations);
int AddStaticBox(EpsilonWorld& world, Entity& ent, Position& pos, Angle& angle, float width, float height, float density, float dynamicFriction, float staticFriction, float restitution);
int AddDynamicBox(EpsilonWorld& world, Entity& ent, Position& pos, Angle& angle, float width, float height, float density, float dynamicFriction, float staticFriction, float restitution);
int CreateDynamicBox(EpsilonWorld& world, Position& pos, Angle& angle, float width, float height, float density, float dynamicFriction, float staticFriction, float restitution);

int CreateDynamicCircle(EpsilonWorld& world, Position& pos, Angle& angle, float radius, float density, float dynamicFriction, float staticFriction, float restitution);
