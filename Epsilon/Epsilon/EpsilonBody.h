#pragma once
#include<iostream>
#include<vector>
#include<mutex>
#include"AABB.h"
#include"EpsilonVector.h"
using namespace std;

enum Shapetype {
	circle = 0,
	box = 1,
	triangle = 2
};
enum Connectiontype {
	none = 0,
	thr = 1,
	spring = 2
};
struct EpsilonBody
{
public:
	bool usingCCD;
	float angle, angularVelocity, inverseMass, connectionDistance;
	float density, mass, restitution, area, radius, width, height, inertia, inverseInertia, dynamicFriction, staticFriction;
	bool isStatic;
	bool isSleeping;
	float sleepTimer;
	float deltaTime;
	EpsilonVector grav;
	EpsilonVector acceleration;
	vector<int> collisions;
	EpsilonVector position, linearVelocity, force, originPosition, connectionPosition;
	AABB aabb;
	AABB ccdAABB;
	Shapetype shapetype;
	Connectiontype connectiontype;




	EpsilonBody(EpsilonVector position, float density, float mass, float inertia, float restitution, float area, float radius, float width,
		float height, vector<EpsilonVector> vertices, bool isStatic, bool usingCCD, Shapetype shapetype, Connectiontype connectiontype);
	static EpsilonBody CreateNewBody(EpsilonBody body);
	static EpsilonBody CreateCircleBody(EpsilonVector position, float density, float restitution, float radius, bool isStatic, bool usingCCD, Connectiontype connectiontype);
	static EpsilonBody CreateBoxBody(EpsilonVector position, float density, float restitution, float width, float height, bool isStatic, bool usingCCD, Connectiontype connectiontype);
	static EpsilonBody CreateTriangleBody(EpsilonVector position, float density, float restitution, float side, bool isStatic, bool usingCCD, Connectiontype connectiontype);
	void CreateConnection(EpsilonVector origin);
	void updateMovement(float dt, EpsilonVector gravity, int iterations);
	void Move(EpsilonVector amount);
	vector<EpsilonVector> GetTransformedVertices();
	void MoveTo(EpsilonVector& pos);
	void AddForce(EpsilonVector amount);
	AABB GetAABB(bool isCCD = 0);
	EpsilonVector Transform(EpsilonVector position, EpsilonVector endposition, float angle);
private:
	static vector<EpsilonVector> GetBoxVertices(float width, float height);
	static vector<EpsilonVector> GetTriangleVertices(float side);
	void UpdateRotation(float angle);
	vector<EpsilonVector> transformedVertices;
	vector<EpsilonVector> vertices;

};

struct Entity {
	int32_t id;
	float sleepTimer = 0;
	Shapetype shapetype;
};
struct Position {
	EpsilonVector value;
};
struct Transform {
	EpsilonVector velocity;
	EpsilonVector acceleration;
};
struct Vertices {
	vector<EpsilonVector> transformedVertices;
	vector<EpsilonVector> vertices;
};
struct FrictionAndRestitution {
	float dynamicFriction, staticFriction, restitution;
};
struct Angle {
	float value;
};
struct AngularVelocity {
	float value;
};
struct inverseSim {
	float inverseMass;
	float inverseInertia;
};
struct Box {
	float width;
	float height;
};
struct Triangle {
	float side;
};
struct Circle {
	float radius;
};
struct SpringJoint {
	vector<Entity> jointIds;
};
struct ThreadJoint {
	vector<Entity> jointIds;
};
struct ImpulseCache {
	uint64_t key;
	EpsilonVector tangentList[2] = {EpsilonVector(0,0),EpsilonVector(0,0)};
	float accumulatedNormalImpulse[2] = { 0.0f,0.0f };
	float accumulatedTangentImpulse[2] = { 0.0f,0.0f };
	bool operator<(const ImpulseCache& other) const { return key < other.key; }
};
template <typename T>
struct SparseSet {
	vector<int> sparse;
	vector<T> dense;
	vector<int> entities;
	T def{};
	void Init() {
		sparse.emplace_back(-1);
		entities.emplace_back(-1);
		dense.emplace_back(T{});
	}
	void Add(T& x, int index, int size) {
		sparse.resize(size, -1);
		dense.emplace_back(x);
		sparse[index] = dense.size() - 1;
		entities.emplace_back(index);
	}
	void Swap(int denseIndexA, int denseIndexB) {
		std::swap(dense[denseIndexA], dense[denseIndexB]);
		std::swap(entities[denseIndexA], entities[denseIndexB]);
		sparse[entities[denseIndexA]] = denseIndexA;
		sparse[entities[denseIndexB]] = denseIndexB;
	}
	void remove(int index) {
		int target_dense_idx = sparse[index];
		if (target_dense_idx == -1) return; // Guard against bad index

		int last_entity = entities.back();

		// 1. Swap target element with last element in both vectors
		dense[target_dense_idx] = std::move(dense.back());
		entities[target_dense_idx] = last_entity;

		// 2. Update sparse map for the element that got moved
		sparse[last_entity] = target_dense_idx;

		// 3. Mark the removed entity as invalid
		sparse[index] = -1;

		// 4. Pop last elements in O(1)
		dense.pop_back();
		entities.pop_back();
	}
	T& get(int id) {return dense[sparse[id]]; }
	T& getForStatic(int id) { 
		if (sparse[id] == -1) {
			return def;
		}
		else {
			return dense[sparse[id]];
		}
	}
	int getEntity(int index) { return entities[index]; }
	uint32_t getInternalIndex(int entityID) {
		
		return sparse[entityID];
		
	}
	uint32_t getInternalIndexForStatic(int entityID) {
		if (entityID < 0 || entityID >= (int)sparse.size()) return 0;
		int idx = sparse[entityID];
		return (idx == -1) ? 0 : (uint32_t)idx;
	}
	int Size() { return dense.size(); }
};

vector<EpsilonVector> GetBoxVertices(Box& b);
AABB GetPolyAABB(Position pos, Vertices vert, Angle ang);
vector<EpsilonVector> GetTransformedVertices(Position position, Vertices vertices, Angle angle);
AABB GetCircleAABB(Position pos, Circle c);
void UpdatePolyAABB(AABB& aabb, Position pos, Vertices vert, Angle ang);

void UpdateCircleAABB(AABB& aabb, Position pos, Circle c);
