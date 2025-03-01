#include "Collisions.h"

float Collisions::Distance(Vector2f a, Vector2f b)
{
    float dx = b.x - a.x;
    float dy = b.y - a.y;
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

bool Collisions::IntersectPolygons(vector<Vector2f> verticesA, vector<Vector2f> verticesB)
{
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
    }
    
    return true;
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

