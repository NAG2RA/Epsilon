#include "Collisions.h"

float Collisions::Distance(Vector2f a, Vector2f b)
{
    float dx = a.x - b.x;
    float dy = a.y - b.y;
    return sqrt(dx*dx+dy*dy);
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
    depth /= normal.length();
    normal = normal.normalized();
    Vector2f centerA = FindArithmeticMean(verticesA);
    Vector2f centerB = FindArithmeticMean(verticesB);
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

