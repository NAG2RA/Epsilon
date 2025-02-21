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
