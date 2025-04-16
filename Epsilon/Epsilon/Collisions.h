#pragma once
#include<SFML/Graphics.hpp>
#include<SFML/Window.hpp>
#include<SFML/System.hpp>
#include<iostream>
#include<vector>
#include<cmath>
#include<limits>
#include"EpsilonBody.h"
#include"EpsilonVector.h"
using namespace sf;
using namespace std;
class Collisions
{
public:
	static float Distance(EpsilonVector a, EpsilonVector b);
	static float DistanceSquared(EpsilonVector a, EpsilonVector b);
	static bool NearlyEqual(float a, float b);
	static bool NearlyEqual(EpsilonVector a, EpsilonVector b);
	static bool IntersectCircles(float radiusA,float radiusB, EpsilonVector centerA, EpsilonVector centerB, EpsilonVector& normal,float& depth);
	static bool IntersectPolygons(vector<EpsilonVector> verticesA, vector<EpsilonVector> verticesB, EpsilonVector& normal, float& depth);
	static bool IntersectPolygons(EpsilonVector centerA, vector<EpsilonVector> verticesA, EpsilonVector centerB, vector<EpsilonVector> verticesB, EpsilonVector& normal, float& depth);
	static bool IntersectPolygonAndCircle(EpsilonVector circleCenter, float radius, vector<EpsilonVector> vertices, EpsilonVector& normal, float& depth);
	static bool IntersectPolygonAndCircle(EpsilonVector circleCenter, EpsilonVector polygonCenter, float circleRadius, vector<EpsilonVector> vertices, EpsilonVector& normal, float& depth);
	static void ProjectCircle(EpsilonVector center, float radius, EpsilonVector axis, float& min, float& max);
	static void ProjectVertices(vector<EpsilonVector> vertices, EpsilonVector axis, float& min, float& max);
	static int FindClosestPointOnPolygon(EpsilonVector Center, vector<EpsilonVector> vertices);
	static bool Collide(EpsilonBody bodyA, EpsilonBody bodyB, EpsilonVector& normal, float& depth);
	static bool IntersectAABB(AABB a, AABB b);
	static void PointSegmentDistance(EpsilonVector p, EpsilonVector a, EpsilonVector b, float& distanceSquared, EpsilonVector& cp);
	static void FindContactPoints(EpsilonBody bodyA, EpsilonBody bodyB, EpsilonVector& contact1, EpsilonVector& contact2, int& contactCount);
	static void FindPolygonsContactPoints(vector<EpsilonVector> VerticesA, vector<EpsilonVector> VerticesB, EpsilonVector& contact1, EpsilonVector& contact2, int& contactCount);
	static void FindCirclePolygonContactPoint(EpsilonVector circleCenter, float circleRadius, EpsilonVector polygonCenter, vector<EpsilonVector> polygonVertices, EpsilonVector& cp);
	static void FindCirclesContactPoint(EpsilonVector centerA, EpsilonVector centerB, float radiusA, EpsilonVector& cp);
	static EpsilonVector FindArithmeticMean(vector<EpsilonVector> vertices);
};

