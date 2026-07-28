#include "Collisions.h"

bool NearlyEqual(float a, float b) {
    float nearlyEqual = 0.005f;
    return abs(a - b) < nearlyEqual;
}
bool NearlyEqual(EpsilonVector a, EpsilonVector b) {
    float nearlyEqual = 0.05f;
    return a.DistanceSquared(b)<nearlyEqual*nearlyEqual;
}
bool IntersectCircles(float radiusA, float radiusB, EpsilonVector centerA, EpsilonVector centerB, EpsilonVector& normal, float& depth)
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

bool IntersectPolygons(vector<EpsilonVector> verticesA, vector<EpsilonVector> verticesB, EpsilonVector& normal, float& depth)
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

bool IntersectPolygons(EpsilonVector centerA, vector<EpsilonVector> verticesA, EpsilonVector centerB, vector<EpsilonVector> verticesB, EpsilonVector& normal, float& depth)
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
vector<EpsilonVector> ClipEdge(const vector<EpsilonVector>& path, EpsilonVector n, float offset)
{
    vector<EpsilonVector> out;
    if (path.size() < 2) return out;

    // Loop to size - 1 (do NOT loop back to the beginning)
    for (size_t i = 0; i < path.size() - 1; ++i) {
        EpsilonVector a = path[i];
        EpsilonVector b = path[i + 1];

        float da = n.Dot(a) - offset;
        float db = n.Dot(b) - offset;

        // If 'a' is inside
        if (da <= 0.0f) out.push_back(a);

        // If edge crosses the plane, add the intersection
        if (da * db < 0.0f) {
            float t = da / (da - db);
            out.push_back(a + (b - a) * t);
        }
    }

    // Check the final point in the path
    if (n.Dot(path.back()) - offset <= 0.0f) {
        out.push_back(path.back());
    }

    return out;
}

void FindPolygonsContactPointsClipped(
    vector<EpsilonVector> verticesA, vector<EpsilonVector> verticesB,
    EpsilonVector normal, int refPoly, int refEdgeIndex,
    EpsilonVector& contact1, EpsilonVector& contact2, int& contactCount)
{
   

    vector<EpsilonVector>& refVerts = (refPoly == 0) ? verticesA : verticesB;
    vector<EpsilonVector>& incVerts = (refPoly == 0) ? verticesB : verticesA;

    EpsilonVector refA = refVerts[refEdgeIndex];
    EpsilonVector refB = refVerts[(refEdgeIndex + 1) % refVerts.size()];
    EpsilonVector refEdgeDir = (refB - refA).Normalized();

    // Derive the TRUE outward normal of the reference edge directly from its
    // own polygon's centroid — don't rely on the two-body `normal` for this.
    EpsilonVector refNormal = EpsilonVector(-refEdgeDir.y, refEdgeDir.x).Normalized();
    EpsilonVector refCenter = FindArithmeticMean(refVerts);
    EpsilonVector edgeMid = (refA + refB) * 0.5f;
   
        refNormal = refNormal * -1.0f;
    

    int incEdgeIndex = 0;
    float minDot = FLT_MAX;
	EpsilonVector incCenter = FindArithmeticMean(incVerts);
    for (size_t i = 0; i < incVerts.size(); i++) {
        EpsilonVector ea = incVerts[i];
        EpsilonVector eb = incVerts[(i + 1) % incVerts.size()];
        EpsilonVector edgeDir = (eb - ea).Normalized();

        // Compute outward normal relative to incident polygon's center
        EpsilonVector incFaceNormal = EpsilonVector(-edgeDir.y, edgeDir.x);
        
        
            incFaceNormal = -incFaceNormal;
        

        float d = incFaceNormal.Dot(refNormal);
        if (d < minDot) {
            minDot = d;
            incEdgeIndex = (int)i;
        }
    }
    EpsilonVector incA = incVerts[incEdgeIndex];
    EpsilonVector incB = incVerts[(incEdgeIndex + 1) % incVerts.size()];

    EpsilonVector sidePlaneNormal = refEdgeDir;
    float negSide = -sidePlaneNormal.Dot(refA);
    float posSide = sidePlaneNormal.Dot(refB);

    vector<EpsilonVector> clipped = { incA, incB };

    // Clip against side plane 1
    clipped = ClipEdge(clipped, -sidePlaneNormal, negSide);
    if (clipped.size() < 2) return;

    // Clip against side plane 2
    clipped = ClipEdge(clipped, sidePlaneNormal, posSide);
    if (clipped.size() < 2) return;

    if (clipped.size() == 2) {
        // order deterministically along the reference edge direction
        float t0 = refEdgeDir.Dot(clipped[0]);
        float t1 = refEdgeDir.Dot(clipped[1]);
        if (t0 > t1) std::swap(clipped[0], clipped[1]);
    }

    float maxAllowedSeparation = 0.01f;

    float refFaceOffset = refNormal.Dot(refA);
    float sep1;
    float sep2;
    for (auto& p : clipped) {
        float sep = refNormal.Dot(p) - refFaceOffset;
        if (sep <= maxAllowedSeparation) {
            if (contactCount == 0) {
                contact1 = p;
                contactCount = 1;
                sep1 = sep;
            }
            else if (contactCount == 1) {
                contact2 = p;
                contactCount = 2;
                sep2 = sep;
                break;
            }
        }
    }
}
bool IntersectPolygons(EpsilonVector centerA, vector<EpsilonVector> verticesA,
    EpsilonVector centerB, vector<EpsilonVector> verticesB,
    EpsilonVector& normal, float& depth,
    int& refPoly, int& refEdgeIndex)
{
    depth = FLT_MAX;

    // Test A's axes
    for (size_t i = 0; i < verticesA.size(); i++) {
        EpsilonVector va = verticesA[i];
        EpsilonVector vb = verticesA[(i + 1) % verticesA.size()];
        EpsilonVector edge = vb - va;
        EpsilonVector axis = EpsilonVector(-edge.y, edge.x).Normalized();

        // Ensure axis points outward from Polygon A
        EpsilonVector faceMid = (va + vb) * 0.5f;
        if (axis.Dot(faceMid - centerA) < 0.0f) {
            axis = -axis;
        }

        float minA, maxA, minB, maxB;
        ProjectVertices(verticesA, axis, minA, maxA);
        ProjectVertices(verticesB, axis, minB, maxB);

        // DIRECTED OVERLAP: Penetration of B into A along A's outward normal
        float axisDepth = maxA - minB;
        if (axisDepth <= 0.0f) return false; // Separating axis found, no collision

        if (axisDepth < depth) {
            depth = axisDepth;
            normal = axis; // Normal points from A to B
            refPoly = 0;
            refEdgeIndex = (int)i;
        }
    }

    // Test B's axes
    for (size_t i = 0; i < verticesB.size(); i++) {
        EpsilonVector va = verticesB[i];
        EpsilonVector vb = verticesB[(i + 1) % verticesB.size()];
        EpsilonVector edge = vb - va;
        EpsilonVector axis = EpsilonVector(-edge.y, edge.x).Normalized();

        // Ensure axis points outward from Polygon B
        EpsilonVector faceMid = (va + vb) * 0.5f;
        if (axis.Dot(faceMid - centerB) < 0.0f) {
            axis = -axis;
        }

        float minA, maxA, minB, maxB;
        ProjectVertices(verticesA, axis, minA, maxA);
        ProjectVertices(verticesB, axis, minB, maxB);

        // DIRECTED OVERLAP: Penetration of A into B along B's outward normal
        float axisDepth = maxB - minA;
        if (axisDepth <= 0.0f) return false; // Separating axis found, no collision

        if (axisDepth < depth) {
            depth = axisDepth;
            // Since axis is outward from B (pointing B to A), 
            // we invert it so the collision normal consistently points A to B
            normal = -axis;
            refPoly = 1;
            refEdgeIndex = (int)i;
        }
    }

    // We no longer need the normal.Dot(...) check at the end because 
    // the directed overlaps guarantee the normal points from A -> B.
    return true;
}

bool IntersectPolygonAndCircle(EpsilonVector circleCenter, float circleRadius, vector<EpsilonVector> vertices, EpsilonVector& normal, float& depth)
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

bool IntersectPolygonAndCircle(EpsilonVector circleCenter, EpsilonVector centerPolygon, float circleRadius, vector<EpsilonVector> vertices, EpsilonVector& normal, float& depth)
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

void ProjectCircle(EpsilonVector center, float radius, EpsilonVector axis, float& min, float& max)
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

void ProjectVertices(vector<EpsilonVector> vertices, EpsilonVector axis, float& min, float& max)
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

int FindClosestPointOnPolygon(EpsilonVector Center, vector<EpsilonVector> vertices)
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
bool Collide(EpsilonBody bodyA, EpsilonBody bodyB, EpsilonVector& normal, float& depth)
{
    if (bodyA.shapetype == box || bodyA.shapetype == triangle) {
        if (bodyB.shapetype == box || bodyB.shapetype == triangle) {
            if (IntersectPolygons(bodyA.position, bodyA.GetTransformedVertices(), bodyB.position, bodyB.GetTransformedVertices(), normal, depth)) {
                return true;
            }
        }
        else if (bodyB.shapetype == circle) {

            if (IntersectPolygonAndCircle(bodyB.position, bodyA.position, bodyB.radius, bodyA.GetTransformedVertices(), normal, depth)) {
                normal = -normal;
                return true;
            }
        }
    }
    else if(bodyA.shapetype == circle) {
        if (bodyB.shapetype == box || bodyB.shapetype == triangle) {
            if (IntersectPolygonAndCircle(bodyA.position, bodyA.radius, bodyB.GetTransformedVertices(), normal, depth)) {
                return true;
            }
        }
        else if (bodyB.shapetype == circle) {
            if (IntersectCircles(bodyA.radius, bodyB.radius, bodyA.position, bodyB.position, normal, depth)) {
                return true;
            }
        }
    }
    return false;
}
bool IntersectAABB(AABB a, AABB b)
{
    if (a.max.x < b.min.x || a.min.x > b.max.x || a.max.y<b.min.y || a.min.y > b.max.y) {
        return false;
    }
    return true;
}
bool ContainsAABB(AABB a, AABB b)
{
    if (a.min.x<=b.min.x && a.max.x>=b.max.x && a.min.y<=b.min.y && a.max.y>=b.max.y || b.min.x<=a.min.x && b.max.x>=a.max.x && b.min.y<=a.min.y && b.max.y>=a.max.y) {
        return true;
    }
    return false;
}
void PointSegmentDistance(EpsilonVector p, EpsilonVector a, EpsilonVector b, float& distanceSquared, EpsilonVector& cp)
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
void FindContactPoints(EpsilonBody bodyA, EpsilonBody bodyB, EpsilonVector& contact1, EpsilonVector& contact2, int& contactCount)
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
void FindPolygonsContactPoints(vector<EpsilonVector> verticesA, vector<EpsilonVector> verticesB, EpsilonVector& contact1, EpsilonVector& contact2, int& contactCount) {
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
void FindCirclePolygonContactPoint(EpsilonVector circleCenter, float circleRadius, EpsilonVector polygonCenter, vector<EpsilonVector> polygonVertices, EpsilonVector& cp)
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
void FindCirclesContactPoint(EpsilonVector centerA, EpsilonVector centerB, float radiusA, EpsilonVector& cp)
{
    EpsilonVector dir = centerB - centerA;
    dir = dir.Normalized();
    cp = centerA + dir * radiusA;
}

EpsilonVector FindArithmeticMean(vector<EpsilonVector> vertices)
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


