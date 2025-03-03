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
	static bool IntersectCircles(float radiusA,float radiusB, Vector2f centerA, Vector2f centerB,Vector2f& normal,float& depth);
	static bool IntersectPolygons(vector<Vector2f> verticesA, vector<Vector2f> verticesB, Vector2f& normal, float& depth);
	static bool IntersectPolygonAndCircle(Vector2f circleCenter, float radius, vector<Vector2f> vertices, Vector2f& normal, float& depth);
	static void ProjectCircle(Vector2f center, float radius, Vector2f axis, float& min, float& max);
	static void ProjectVertices(vector<Vector2f> vertices, Vector2f axis, float& min, float& max);
	static int FindClosestPointOnPolygon(Vector2f Center, vector<Vector2f> vertices);
	static Vector2f FindArithmeticMean(vector<Vector2f> vertices);
};

