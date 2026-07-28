#pragma once
#include<vector>
#include<immintrin.h>
#include"EpsilonBody.h"
#include"EpsilonVector.h"

	bool NearlyEqual(float a, float b);
	bool NearlyEqual(EpsilonVector a, EpsilonVector b);
	bool IntersectCircles(float radiusA,float radiusB, EpsilonVector centerA, EpsilonVector centerB, EpsilonVector& normal,float& depth);
	bool IntersectPolygons(vector<EpsilonVector> verticesA, vector<EpsilonVector> verticesB, EpsilonVector& normal, float& depth);
	bool IntersectPolygons(EpsilonVector centerA, vector<EpsilonVector> verticesA, EpsilonVector centerB, vector<EpsilonVector> verticesB, EpsilonVector& normal, float& depth);
	void FindPolygonsContactPointsClipped(vector<EpsilonVector> verticesA, vector<EpsilonVector> verticesB, EpsilonVector normal, int refPoly, int refEdgeIndex, EpsilonVector& contact1, EpsilonVector& contact2, int& contactCount);
	bool IntersectPolygons(EpsilonVector centerA, vector<EpsilonVector> verticesA, EpsilonVector centerB, vector<EpsilonVector> verticesB, EpsilonVector& normal, float& depth, int& refPoly, int& refEdgeIndex);
	bool IntersectPolygonAndCircle(EpsilonVector circleCenter, float radius, vector<EpsilonVector> vertices, EpsilonVector& normal, float& depth);
	bool IntersectPolygonAndCircle(EpsilonVector circleCenter, EpsilonVector polygonCenter, float circleRadius, vector<EpsilonVector> vertices, EpsilonVector& normal, float& depth);
	void ProjectCircle(EpsilonVector center, float radius, EpsilonVector axis, float& min, float& max);
	void ProjectVertices(vector<EpsilonVector> vertices, EpsilonVector axis, float& min, float& max);
	int FindClosestPointOnPolygon(EpsilonVector Center, vector<EpsilonVector> vertices);
	bool Collide(EpsilonBody bodyA, EpsilonBody bodyB, EpsilonVector& normal, float& depth);
	bool IntersectAABB(AABB a, AABB b);
	bool ContainsAABB(AABB a, AABB b);
	void PointSegmentDistance(EpsilonVector p, EpsilonVector a, EpsilonVector b, float& distanceSquared, EpsilonVector& cp);
	void FindContactPoints(EpsilonBody bodyA, EpsilonBody bodyB, EpsilonVector& contact1, EpsilonVector& contact2, int& contactCount);
	void FindPolygonsContactPoints(vector<EpsilonVector> VerticesA, vector<EpsilonVector> VerticesB, EpsilonVector& contact1, EpsilonVector& contact2, int& contactCount);
	void FindCirclePolygonContactPoint(EpsilonVector circleCenter, float circleRadius, EpsilonVector polygonCenter, vector<EpsilonVector> polygonVertices, EpsilonVector& cp);
	void FindCirclesContactPoint(EpsilonVector centerA, EpsilonVector centerB, float radiusA, EpsilonVector& cp);
	EpsilonVector FindArithmeticMean(vector<EpsilonVector> vertices);


