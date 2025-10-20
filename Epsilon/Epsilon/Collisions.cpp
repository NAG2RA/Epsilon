#include "Collisions.h"

bool Collisions::NearlyEqual(float a, float b) {
    float nearlyEqual = 0.0000005f;
    return abs(a - b) < nearlyEqual;
}
bool Collisions::NearlyEqual(EpsilonVector a, EpsilonVector b) {
    float nearlyEqual = 0.0005f;
    return a.DistanceSquared(b)<nearlyEqual*nearlyEqual;
}
bool Collisions::IntersectCircles(float radiusA, float radiusB, EpsilonVector centerA, EpsilonVector centerB, EpsilonVector& normal, float& depth)
{
    float dist = centerA.Distance(centerB);
    float radii = radiusA + radiusB;
    if (dist >= radii) 
    {
        return false;
    }
    normal = centerB - centerA;
    normal = normal.Normalized();
    depth = radii - dist;
    return true;
}

bool Collisions::IntersectPolygons(vector<EpsilonVector> verticesA, vector<EpsilonVector> verticesB, EpsilonVector& normal, float& depth)
{
    depth = FLT_MAX;
    for (size_t i = 0; i < verticesA.size(); i++) {
        EpsilonVector va = verticesA[i];
        EpsilonVector vb = verticesA[(i + 1) % verticesA.size()];
        EpsilonVector edge = vb - va;
        EpsilonVector axis = EpsilonVector(-edge.y, edge.x);
        axis = axis.Normalized();
        float minA = 0;
        float maxA = 0;
        float minB = 0;
        float maxB = 0;
        ProjectVertices(verticesA, axis, minA, maxA);
        ProjectVertices(verticesB, axis, minB, maxB);
        if (minA >= maxB || minB >= maxA) {
            return false;
        }
        float axisDepth = min(maxB - minA, maxA - minB);
        if (axisDepth < depth) {
            depth = axisDepth;
            normal = axis;
        }
    }
    for (size_t i = 0; i < verticesB.size(); i++) {
        EpsilonVector va = verticesB[i];
        EpsilonVector vb = verticesB[(i + 1) % verticesB.size()];
        EpsilonVector edge = vb - va;
        EpsilonVector axis = EpsilonVector(-edge.y, edge.x);
        axis = axis.Normalized();
        float minA = 0;
        float maxA = 0;
        float minB = 0;
        float maxB = 0;
        ProjectVertices(verticesA, axis, minA, maxA);
        ProjectVertices(verticesB, axis, minB, maxB);
        if (minA >= maxB || minB >= maxA) {
          
            return false;
        }
        float axisDepth = min(maxB - minA, maxA - minB);
        if (axisDepth < depth) {
            depth = axisDepth;
            normal = axis;
        }
    }
    EpsilonVector centerA = FindArithmeticMean(verticesA);
    EpsilonVector centerB = FindArithmeticMean(verticesB);
    EpsilonVector direction = centerB - centerA;
    if (direction.Dot(normal) < 0.f) {
        normal = -normal;
    }
    return true;
}

bool Collisions::IntersectPolygons(EpsilonVector centerA, vector<EpsilonVector> verticesA, EpsilonVector centerB, vector<EpsilonVector> verticesB, EpsilonVector& normal, float& depth)
{
    depth = FLT_MAX;
    for (size_t i = 0; i < verticesA.size(); i++) {
        EpsilonVector va = verticesA[i];
        EpsilonVector vb = verticesA[(i + 1) % verticesA.size()];
        EpsilonVector edge = vb - va;
        EpsilonVector axis = EpsilonVector(-edge.y, edge.x);
        axis = axis.Normalized();
        float minA = 0;
        float maxA = 0;
        float minB = 0;
        float maxB = 0;
        ProjectVertices(verticesA, axis, minA, maxA);
        ProjectVertices(verticesB, axis, minB, maxB);
        if (minA >= maxB || minB >= maxA) {
            return false;
        }
        float axisDepth = min(maxB - minA, maxA - minB);
        if (axisDepth < depth) {
            depth = axisDepth;
            normal = axis;
        }
    }
    for (size_t i = 0; i < verticesB.size(); i++) {
        EpsilonVector va = verticesB[i];
        EpsilonVector vb = verticesB[(i + 1) % verticesB.size()];
        EpsilonVector edge = vb - va;
        EpsilonVector axis = EpsilonVector(-edge.y, edge.x);
        axis = axis.Normalized();
        float minA = 0;
        float maxA = 0;
        float minB = 0;
        float maxB = 0;
        ProjectVertices(verticesA, axis, minA, maxA);
        ProjectVertices(verticesB, axis, minB, maxB);
        if (minA >= maxB || minB >= maxA) {

            return false;
        }
        float axisDepth = min(maxB - minA, maxA - minB);
        if (axisDepth < depth) {
            depth = axisDepth;
            normal = axis;
        }
    }
    EpsilonVector direction = centerB - centerA;
    if (direction.Dot(normal) < 0.f) {
        normal = -normal;
    }
    return true;
}

bool Collisions::IntersectPolygonAndCircle(EpsilonVector circleCenter, float circleRadius, vector<EpsilonVector> vertices, EpsilonVector& normal, float& depth)
{
    depth = FLT_MAX;
    float minA = 0;
    float maxA = 0;
    float minB = 0;
    float maxB = 0;
    float axisDepth = 0;
    EpsilonVector axis;
    for (size_t i = 0; i < vertices.size(); i++) {
        EpsilonVector va = vertices[i];
        EpsilonVector vb = vertices[(i + 1) % vertices.size()];
        EpsilonVector edge = vb - va;
        axis = EpsilonVector(-edge.y, edge.x);
        axis = axis.Normalized();
        ProjectVertices(vertices, axis, minA, maxA);
        ProjectCircle(circleCenter,circleRadius, axis, minB, maxB);
        if (minA >= maxB || minB >= maxA) {
            return false;
        }
        axisDepth = min(maxB - minA, maxA - minB);
        if (axisDepth < depth) {
            depth = axisDepth;
            normal = axis;
        }
    }
    int cpindex = FindClosestPointOnPolygon(circleCenter, vertices);
    EpsilonVector cp = vertices[cpindex];
    axis = cp - circleCenter;
    axis = axis.Normalized();
    ProjectVertices(vertices, axis, minA, maxA);
    ProjectCircle(circleCenter, circleRadius, axis, minB, maxB);
    if (minA >= maxB || minB >= maxA) {
        return false;
    }
    axisDepth = min(maxB - minA, maxA - minB);
    if (axisDepth < depth) {
        depth = axisDepth;
        normal = axis;
    }
    EpsilonVector centerPolygon = FindArithmeticMean(vertices);
    EpsilonVector direction = centerPolygon - circleCenter;
    if (direction.Dot(normal) < 0.f) {
        normal = -normal;
    }
    return true;
}

bool Collisions::IntersectPolygonAndCircle(EpsilonVector circleCenter, EpsilonVector centerPolygon, float circleRadius, vector<EpsilonVector> vertices, EpsilonVector& normal, float& depth)
{
    depth = FLT_MAX;
    float minA = 0;
    float maxA = 0;
    float minB = 0;
    float maxB = 0;
    float axisDepth = 0;
    EpsilonVector axis;
    for (size_t i = 0; i < vertices.size(); i++) {
        EpsilonVector va = vertices[i];
        EpsilonVector vb = vertices[(i + 1) % vertices.size()];
        EpsilonVector edge = vb - va;
        axis = EpsilonVector(-edge.y, edge.x);
        axis = axis.Normalized();
        ProjectVertices(vertices, axis, minA, maxA);
        ProjectCircle(circleCenter, circleRadius, axis, minB, maxB);
        if (minA >= maxB || minB >= maxA) {
            return false;
        }
        axisDepth = min(maxB - minA, maxA - minB);
        if (axisDepth < depth) {
            depth = axisDepth;
            normal = axis;
        }
    }
    int cpindex = FindClosestPointOnPolygon(circleCenter, vertices);
    EpsilonVector cp = vertices[cpindex];
    axis = cp - circleCenter;
    axis = axis.Normalized();
    ProjectVertices(vertices, axis, minA, maxA);
    ProjectCircle(circleCenter, circleRadius, axis, minB, maxB);
    if (minA >= maxB || minB >= maxA) {
        return false;
    }
    axisDepth = min(maxB - minA, maxA - minB);
    if (axisDepth < depth) {
        depth = axisDepth;
        normal = axis;
    }
    EpsilonVector direction = centerPolygon - circleCenter;
    if (direction.Dot(normal) < 0.f) {
        normal = -normal;
    }
    return true;
}

void Collisions::ProjectCircle(EpsilonVector center, float radius, EpsilonVector axis, float& min, float& max)
{
    EpsilonVector dir = axis.Normalized();
    EpsilonVector dirAndRad = dir * radius;
    EpsilonVector p1 = center + dirAndRad;
    EpsilonVector p2 = center - dirAndRad;
    min = axis.Dot(p1);
    max = axis.Dot(p2);
    if (min >= max) {
        float t = min;
        min = max;
        max = t;
    }
}

void Collisions::ProjectVertices(vector<EpsilonVector> vertices, EpsilonVector axis, float& min, float& max)
{
    min = FLT_MAX;
    max = -FLT_MAX;
    for (size_t i = 0; i < vertices.size(); i++) {
        EpsilonVector v = vertices[i];
        float proj = axis.Dot(v);
        if (proj < min) {
            min = proj;
        }
        if (proj > max) {
            max = proj;
        }
    }
}

int Collisions::FindClosestPointOnPolygon(EpsilonVector Center, vector<EpsilonVector> vertices)
{
    int result = 0;
    float minDistance = FLT_MAX;
    for (size_t i = 0; i < vertices.size(); i++) {
        EpsilonVector v = vertices[i];
        float distance = v.Distance(Center);
        if (distance < minDistance) {
            minDistance = distance;
            result = i;
        }
    }
    return result;
}
bool Collisions::Collide(EpsilonBody bodyA, EpsilonBody bodyB, EpsilonVector& normal, float& depth)
{
    if (bodyA.shapetype == box || bodyA.shapetype == triangle) {
        if (bodyB.shapetype == box || bodyB.shapetype == triangle) {
            if (Collisions::IntersectPolygons(bodyA.position, bodyA.GetTransformedVertices(), bodyB.position, bodyB.GetTransformedVertices(), normal, depth)) {
                return true;
            }
        }
        else if (bodyB.shapetype == circle) {

            if (Collisions::IntersectPolygonAndCircle(bodyB.position, bodyA.position, bodyB.radius, bodyA.GetTransformedVertices(), normal, depth)) {
                normal = -normal;
                return true;
            }
        }
    }
    else if(bodyA.shapetype == circle) {
        if (bodyB.shapetype == box || bodyB.shapetype == triangle) {
            if (Collisions::IntersectPolygonAndCircle(bodyA.position, bodyA.radius, bodyB.GetTransformedVertices(), normal, depth)) {
                return true;
            }
        }
        else if (bodyB.shapetype == circle) {
            if (Collisions::IntersectCircles(bodyA.radius, bodyB.radius, bodyA.position, bodyB.position, normal, depth)) {
                return true;
            }
        }
    }
    return false;
}
bool Collisions::IntersectAABB(AABB a, AABB b)
{
    if (a.max.x <= b.min.x || a.min.x >= b.max.x || a.max.y<=b.min.y || a.min.y >= b.max.y) {
        return false;
    }
    return true;
}
void Collisions::PointSegmentDistance(EpsilonVector p, EpsilonVector a, EpsilonVector b, float& distanceSquared, EpsilonVector& cp)
{
    EpsilonVector ab = b - a;
    EpsilonVector ap = p - a;
    float proj = ap.Dot(ab);
    float ablensq = ab.LengthSquared();
    float d = proj / ablensq;
    if (d <= 0) {
        cp = a;
    }
    else if (d >= 1) {
        cp = b;
    }
    else {
        cp = a + ab * d;
    }
    distanceSquared = p.DistanceSquared(cp);
}
void Collisions::FindContactPoints(EpsilonBody bodyA, EpsilonBody bodyB, EpsilonVector& contact1, EpsilonVector& contact2, int& contactCount)
{
    contact1 = EpsilonVector(0, 0);
    contact2 = EpsilonVector(0, 0);
    contactCount = 0;
    if (bodyA.shapetype == box||bodyA.shapetype == triangle) {
        if (bodyB.shapetype == box||bodyB.shapetype == triangle) {
            FindPolygonsContactPoints(bodyA.GetTransformedVertices(), bodyB.GetTransformedVertices(), contact1, contact2, contactCount);
        }
        else {
            FindCirclePolygonContactPoint(bodyB.position, bodyB.radius, bodyA.position, bodyA.GetTransformedVertices(), contact1);
            contactCount = 1;
        }
    }
    else {
        if (bodyB.shapetype == box || bodyB.shapetype == triangle) {
            FindCirclePolygonContactPoint(bodyA.position, bodyA.radius, bodyB.position, bodyB.GetTransformedVertices(), contact1);
            contactCount = 1;
        }
        else {
            FindCirclesContactPoint(bodyA.position, bodyB.position, bodyA.radius, contact1);
            contactCount = 1;
        }
    }
}
void Collisions::FindPolygonsContactPoints(vector<EpsilonVector> verticesA, vector<EpsilonVector> verticesB, EpsilonVector& contact1, EpsilonVector& contact2, int& contactCount) {
    float mindistsq = FLT_MAX;
    float distsq = 0;
    EpsilonVector cp;
    for (size_t i = 0; i < verticesA.size(); i++) {
        EpsilonVector p = verticesA[i];
        for (size_t j = 0; j < verticesB.size(); j++) {
            EpsilonVector a = verticesB[j];
            EpsilonVector b = verticesB[(j + 1) % verticesB.size()];
            PointSegmentDistance(p, a, b, distsq, cp);
            if (NearlyEqual(distsq, mindistsq)) {
                if (!NearlyEqual(cp, contact1)&& !NearlyEqual(cp, contact2)) {
                    contact2 = cp;
                    contactCount = 2;
                }
            }
            else if (distsq < mindistsq) {
                mindistsq = distsq;
                contactCount = 1;
                contact1 = cp;
            }
        }
    }
    for (size_t i = 0; i < verticesB.size(); i++) {
        EpsilonVector p = verticesB[i];
        for (size_t j = 0; j < verticesA.size(); j++) {
            EpsilonVector a = verticesA[j];
            EpsilonVector b = verticesA[(j + 1) % verticesA.size()];
            PointSegmentDistance(p, a, b, distsq, cp);
            if (NearlyEqual(distsq, mindistsq)) {
                if (!NearlyEqual(cp, contact1)&& !NearlyEqual(cp, contact2)) {
                    contact2 = cp;
                    contactCount = 2;
                }
            }
            else if (distsq < mindistsq) {
                mindistsq = distsq;
                contactCount = 1;
                contact1 = cp;
            }
        }
    }
}
void Collisions::FindCirclePolygonContactPoint(EpsilonVector circleCenter, float circleRadius, EpsilonVector polygonCenter, vector<EpsilonVector> polygonVertices, EpsilonVector& cp)
{
    float mindistsq = FLT_MAX;
    float distsq = 0;
    EpsilonVector contact;
    for (size_t i = 0; i < polygonVertices.size(); i++) {
        EpsilonVector a = polygonVertices[i];
        EpsilonVector b = polygonVertices[(i+1)%polygonVertices.size()];
        PointSegmentDistance(circleCenter, a, b, distsq, contact);
        if (distsq < mindistsq) {
            mindistsq = distsq;
            cp = contact;
        }
    }
}
void Collisions::FindCirclesContactPoint(EpsilonVector centerA, EpsilonVector centerB, float radiusA, EpsilonVector& cp)
{
    EpsilonVector dir = centerB - centerA;
    dir = dir.Normalized();
    cp = centerA + dir * radiusA;
}

EpsilonVector Collisions::FindArithmeticMean(vector<EpsilonVector> vertices)
{
    float sumX = 0.f;
    float sumY = 0.f;
    for (size_t i = 0; i < vertices.size(); i++) {
        EpsilonVector v = vertices[i];
       sumX += v.x;
       sumY += v.y;
    }
    return EpsilonVector(sumX / (float)vertices.size(), sumY / (float)vertices.size());
}

