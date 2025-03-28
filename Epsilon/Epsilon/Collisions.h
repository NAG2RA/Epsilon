#pragma once
#include<SFML/Graphics.hpp>
#include<SFML/Window.hpp>
#include<SFML/System.hpp>
#include<iostream>
#include<vector>
#include<cmath>
#include<limits>
#include"EpsilonBody.h"
using namespace sf;
using namespace std;
class Collisions
{
public:
	static float Distance(Vector2f a, Vector2f b);
	static float DistanceSquared(Vector2f a, Vector2f b);
	static bool NearlyEqual(float a, float b);
	static bool NearlyEqual(Vector2f a, Vector2f b);
	static bool IntersectCircles(float radiusA,float radiusB, Vector2f centerA, Vector2f centerB,Vector2f& normal,float& depth);
	static bool IntersectPolygons(vector<Vector2f> verticesA, vector<Vector2f> verticesB, Vector2f& normal, float& depth);
	static bool IntersectPolygons(Vector2f centerA, vector<Vector2f> verticesA, Vector2f centerB, vector<Vector2f> verticesB, Vector2f& normal, float& depth);
	static bool IntersectPolygonAndCircle(Vector2f circleCenter, float radius, vector<Vector2f> vertices, Vector2f& normal, float& depth);
	static bool IntersectPolygonAndCircle(Vector2f circleCenter, Vector2f polygonCenter, float circleRadius, vector<Vector2f> vertices, Vector2f& normal, float& depth);
	static void ProjectCircle(Vector2f center, float radius, Vector2f axis, float& min, float& max);
	static void ProjectVertices(vector<Vector2f> vertices, Vector2f axis, float& min, float& max);
	static int FindClosestPointOnPolygon(Vector2f Center, vector<Vector2f> vertices);
	static bool Collide(EpsilonBody bodyA, EpsilonBody bodyB, Vector2f& normal, float& depth);
	static bool IntersectAABB(AABB a, AABB b);
	static void PointSegmentDistance(Vector2f p, Vector2f a, Vector2f b, float& distanceSquared, Vector2f& cp);
	static void FindContactPoints(EpsilonBody bodyA, EpsilonBody bodyB, Vector2f& contact1, Vector2f& contact2, int& contactCount);
	static void FindPolygonsContactPoints(vector<Vector2f> VerticesA, vector<Vector2f> VerticesB, Vector2f& contact1, Vector2f& contact2, int& contactCount);
	static void FindCirclePolygonContactPoint(Vector2f circleCenter, float circleRadius, Vector2f polygonCenter, vector<Vector2f> polygonVertices, Vector2f& cp);
	static void FindCirclesContactPoint(Vector2f centerA, Vector2f centerB, float radiusA, Vector2f& cp);
	static Vector2f FindArithmeticMean(vector<Vector2f> vertices);
};

