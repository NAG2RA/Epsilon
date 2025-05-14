#pragma once
#include<SFML/Graphics.hpp>
#include<SFML/Window.hpp>
#include<SFML/System.hpp>
#include<iostream>
#include<vector>
#include<cmath>
#include<tuple>
#include"EpsilonBody.h"
#include"Collisions.h"
#include"CollisionManifold.h"
#include"EpsilonVector.h"
class EpsilonWorld
{
public:
	EpsilonVector gravity;
	float forceConstant;
	float springConstant, damperConstant, damperThreadConstant;
	EpsilonVector normal;
	float depth;
	EpsilonWorld();
	vector<EpsilonBody> bodyList;
	vector<tuple<size_t,size_t>> contactPairs;
	void AddBody(EpsilonBody body);
	void RemoveBody(int index);
	bool GetBody(float index, EpsilonBody& body);
	void Update(float dt, int iterations);
	void SeperateBodies(EpsilonBody& bodyA, EpsilonBody& bodyB, EpsilonVector mtv);
	void BroadPhase();
	void NarrowPhase();
	void UpdateMovement(float dt, int iterations);
	void ResolveCollisonBasic(CollisionManifold& manifold);
	void ResolveCollisonWithRotation(CollisionManifold& manifold);
	void ResolveCollisonWithRotationAndFriction(CollisionManifold& manifold);
	void ResolveThreadConnection();
	void ResolveSpringConnection();
	void Explosion(EpsilonVector position, float radius, float magnitude);
	void Buoyancy(EpsilonVector surfacePosition, float density);
	void AirResistance(float dt);
};

