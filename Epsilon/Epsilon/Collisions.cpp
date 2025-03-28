#include "Collisions.h"

float Collisions::Distance(Vector2f a, Vector2f b)
{
    float dx = a.x - b.x;
    float dy = a.y - b.y;
    return sqrt(dx*dx+dy*dy);
}
float Collisions::DistanceSquared(Vector2f a, Vector2f b)
{
    float dx = a.x - b.x;
    float dy = a.y - b.y;
    return (dx * dx + dy * dy);
}
bool Collisions::NearlyEqual(float a, float b) {
    float nearlyEqual = 0.0005f;
    return abs(a - b) < nearlyEqual;
}
bool Collisions::NearlyEqual(Vector2f a, Vector2f b) {
    float nearlyEqual = 0.0005f;
    return DistanceSquared(a,b)<nearlyEqual*nearlyEqual;
}
bool Collisions::IntersectCircles(float radiusA, float radiusB, Vector2f centerA, Vector2f centerB, Vector2f& normal, float& depth)
{
    float dist = Distance(centerA, centerB);
    float radii = radiusA + radiusB;
    if (dist >= radii) 
    {
        return false;
    }
    normal = centerB - centerA;
    normal = normal.normalized();
    depth = radii - dist;
    return true;
}

bool Collisions::IntersectPolygons(vector<Vector2f> verticesA, vector<Vector2f> verticesB, Vector2f& normal, float& depth)
{
    depth = FLT_MAX;
    for (size_t i = 0; i < verticesA.size(); i++) {
        Vector2f va = verticesA[i];
        Vector2f vb = verticesA[(i + 1) % verticesA.size()];
        Vector2f edge = vb - va;
        Vector2f axis = Vector2f(-edge.y, edge.x);
        axis = axis.normalized();
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
        Vector2f va = verticesB[i];
        Vector2f vb = verticesB[(i + 1) % verticesB.size()];
        Vector2f edge = vb - va;
        Vector2f axis = Vector2f(-edge.y, edge.x);
        axis = axis.normalized();
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
    Vector2f centerA = FindArithmeticMean(verticesA);
    Vector2f centerB = FindArithmeticMean(verticesB);
    Vector2f direction = centerB - centerA;
    if (direction.dot(normal) < 0.f) {
        normal = -normal;
    }
    return true;
}

bool Collisions::IntersectPolygons(Vector2f centerA, vector<Vector2f> verticesA, Vector2f centerB, vector<Vector2f> verticesB, Vector2f& normal, float& depth)
{
    depth = FLT_MAX;
    for (size_t i = 0; i < verticesA.size(); i++) {
        Vector2f va = verticesA[i];
        Vector2f vb = verticesA[(i + 1) % verticesA.size()];
        Vector2f edge = vb - va;
        Vector2f axis = Vector2f(-edge.y, edge.x);
        axis = axis.normalized();
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
        Vector2f va = verticesB[i];
        Vector2f vb = verticesB[(i + 1) % verticesB.size()];
        Vector2f edge = vb - va;
        Vector2f axis = Vector2f(-edge.y, edge.x);
        axis = axis.normalized();
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
    Vector2f direction = centerB - centerA;
    if (direction.dot(normal) < 0.f) {
        normal = -normal;
    }
    return true;
}

bool Collisions::IntersectPolygonAndCircle(Vector2f circleCenter, float circleRadius, vector<Vector2f> vertices, Vector2f& normal, float& depth)
{
    depth = FLT_MAX;
    float minA = 0;
    float maxA = 0;
    float minB = 0;
    float maxB = 0;
    float axisDepth = 0;
    Vector2f axis;
    for (size_t i = 0; i < vertices.size(); i++) {
        Vector2f va = vertices[i];
        Vector2f vb = vertices[(i + 1) % vertices.size()];
        Vector2f edge = vb - va;
        axis = Vector2f(-edge.y, edge.x);
        axis = axis.normalized();
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
    Vector2f cp = vertices[cpindex];
    axis = cp - circleCenter;
    axis = axis.normalized();
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
    Vector2f centerPolygon = FindArithmeticMean(vertices);
    Vector2f direction = centerPolygon - circleCenter;
    if (direction.dot(normal) < 0.f) {
        normal = -normal;
    }
    return true;
}

bool Collisions::IntersectPolygonAndCircle(Vector2f circleCenter, Vector2f centerPolygon, float circleRadius, vector<Vector2f> vertices, Vector2f& normal, float& depth)
{
    depth = FLT_MAX;
    float minA = 0;
    float maxA = 0;
    float minB = 0;
    float maxB = 0;
    float axisDepth = 0;
    Vector2f axis;
    for (size_t i = 0; i < vertices.size(); i++) {
        Vector2f va = vertices[i];
        Vector2f vb = vertices[(i + 1) % vertices.size()];
        Vector2f edge = vb - va;
        axis = Vector2f(-edge.y, edge.x);
        axis = axis.normalized();
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
    Vector2f cp = vertices[cpindex];
    axis = cp - circleCenter;
    axis = axis.normalized();
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
    Vector2f direction = centerPolygon - circleCenter;
    if (direction.dot(normal) < 0.f) {
        normal = -normal;
    }
    return true;
}

void Collisions::ProjectCircle(Vector2f center, float radius, Vector2f axis, float& min, float& max)
{
    Vector2f dir = axis.normalized();
    Vector2f dirAndRad = dir * radius;
    Vector2f p1 = center + dirAndRad;
    Vector2f p2 = center - dirAndRad;
    min = axis.dot(p1);
    max = axis.dot(p2);
    if (min >= max) {
        float t = min;
        min = max;
        max = t;
    }
}

void Collisions::ProjectVertices(vector<Vector2f> vertices, Vector2f axis, float& min, float& max)
{
    min = FLT_MAX;
    max = -FLT_MAX;
    for (size_t i = 0; i < vertices.size(); i++) {
        Vector2f v = vertices[i];
        float proj = axis.dot(v);
        if (proj < min) {
            min = proj;
        }
        if (proj > max) {
            max = proj;
        }
    }
}

int Collisions::FindClosestPointOnPolygon(Vector2f Center, vector<Vector2f> vertices)
{
    int result = -1;
    float minDistance = FLT_MAX;
    for (size_t i = 0; i < vertices.size(); i++) {
        Vector2f v = vertices[i];
        float distance = Distance(v, Center);   
        if (distance < minDistance) {
            minDistance = distance;
            result = i;
        }
    }
    return result;
}
bool Collisions::Collide(EpsilonBody bodyA, EpsilonBody bodyB, Vector2f& normal, float& depth)
{
    if (bodyA.shapetype == box) {
        if (bodyB.shapetype == box) {
            if (Collisions::IntersectPolygons(bodyA.position, bodyA.GetTransformedVertices(), bodyB.position, bodyB.GetTransformedVertices(), normal, depth)) {
                return true;
            }
        }
        else {
            if (Collisions::IntersectPolygonAndCircle(bodyB.position, bodyA.position, bodyB.radius, bodyA.GetTransformedVertices(), normal, depth)) {
                normal = -normal;
                return true;
            }
        }
    }
    else {
        if (bodyB.shapetype == box) {
            if (Collisions::IntersectPolygonAndCircle(bodyA.position, bodyA.radius, bodyB.GetTransformedVertices(), normal, depth)) {
                return true;
            }
        }
        else {
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
void Collisions::PointSegmentDistance(Vector2f p, Vector2f a, Vector2f b, float& distanceSquared, Vector2f& cp)
{
    Vector2f ab = b - a;
    Vector2f ap = p - a;
    float proj = ap.dot(ab);
    float ablensq = ab.lengthSquared();
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
    distanceSquared = DistanceSquared(p, cp);
}
void Collisions::FindContactPoints(EpsilonBody bodyA, EpsilonBody bodyB, Vector2f& contact1, Vector2f& contact2, int& contactCount)
{
    contact1 = Vector2f(0, 0);
    contact2 = Vector2f(0, 0);
    contactCount = 0;
    if (bodyA.shapetype == box) {
        if (bodyB.shapetype == box) {
            FindPolygonsContactPoints(bodyA.GetTransformedVertices(), bodyB.GetTransformedVertices(), contact1, contact2, contactCount);
        }
        else {
            FindCirclePolygonContactPoint(bodyB.position, bodyB.radius, bodyA.position, bodyA.GetTransformedVertices(), contact1);
            contactCount = 1;
        }
    }
    else {
        if (bodyB.shapetype == box) {
            FindCirclePolygonContactPoint(bodyA.position, bodyA.radius, bodyB.position, bodyB.GetTransformedVertices(), contact1);
            contactCount = 1;
        }
        else {
            FindCirclesContactPoint(bodyA.position, bodyB.position, bodyA.radius, contact1);
            contactCount = 1;
        }
    }
}
void Collisions::FindPolygonsContactPoints(vector<Vector2f> verticesA, vector<Vector2f> verticesB, Vector2f& contact1, Vector2f& contact2, int& contactCount) {
    float mindistsq = FLT_MAX;
    float distsq = 0;
    Vector2f cp;
    for (size_t i = 0; i < verticesA.size(); i++) {
        Vector2f p = verticesA[i];
        for (size_t j = 0; j < verticesB.size(); j++) {
            Vector2f a = verticesB[j];
            Vector2f b = verticesB[(j + 1) % verticesB.size()];
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
        Vector2f p = verticesB[i];
        for (size_t j = 0; j < verticesA.size(); j++) {
            Vector2f a = verticesA[j];
            Vector2f b = verticesA[(j + 1) % verticesA.size()];
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
void Collisions::FindCirclePolygonContactPoint(Vector2f circleCenter, float circleRadius, Vector2f polygonCenter, vector<Vector2f> polygonVertices, Vector2f& cp)
{
    float mindistsq = FLT_MAX;
    float distsq = 0;
    Vector2f contact;
    for (size_t i = 0; i < polygonVertices.size(); i++) {
        Vector2f a = polygonVertices[i];
        Vector2f b = polygonVertices[(i+1)%polygonVertices.size()];
        PointSegmentDistance(circleCenter, a, b, distsq, contact);
        if (distsq < mindistsq) {
            mindistsq = distsq;
            cp = contact;
        }
    }
}
void Collisions::FindCirclesContactPoint(Vector2f centerA, Vector2f centerB, float radiusA, Vector2f& cp)
{
    Vector2f dir = centerB - centerA;
    dir = dir.normalized();
    cp = centerA + dir * radiusA;
}

Vector2f Collisions::FindArithmeticMean(vector<Vector2f> vertices)
{
    float sumX = 0.f;
    float sumY = 0.f;
    for (size_t i = 0; i < vertices.size(); i++) {
       Vector2f v = vertices[i];
       sumX += v.x;
       sumY += v.y;
    }
    return Vector2f(sumX / (float)vertices.size(), sumY / (float)vertices.size());
}

