#include "EpsilonBody.h"
float minArea = 0.01f * 0.01f;
float maxArea = 64 * 64;
float minDensity = 0.5f;
float maxDensity = 21.4f;
EpsilonBody::EpsilonBody(Vector2f position, float density, float mass, float restitution, float area, float radius, float width,
	float height, bool isStatic, Shapetype shapetype)
	:position(position),
	density(density),
	mass(mass),
	restitution(restitution),
	area(area),
	radius(radius),
	width(width),
	height(height),
	isStatic(isStatic),
	shapetype(shapetype),
	rotation(0.f),
	rotationalVelocity(0.f),
	linearVelocity(0, 0),
	drag(0.99f),
	force(0,0),
	aabb(0,0,0,0)
{
	if (!isStatic) {
		InverseMass = (float)1 / mass;
	}
	else {
		InverseMass = 0;
	}
	if (shapetype == Shapetype::box) 
	{
		vertices = GetBoxVertices(width, height);
		transformedVertices.resize(vertices.size());
		boxtriangles = GetBoxTriangles();
	}
	else
	{
		boxtriangles = {};
		vertices = {};
		transformedVertices = {};
	}
	isTransformUpdated = false;
	isAABBUpdated = false;
}

EpsilonBody EpsilonBody::CreateNewBody(EpsilonBody body)
{
	return EpsilonBody(body.position,body.density,body.mass,body.restitution,body.area,body.radius,body.width,body.height,body.isStatic,body.shapetype);
}

EpsilonBody EpsilonBody::CreateCircleBody(Vector2f position, float density, float restitution, float radius, bool isStatic)
{
	float area = radius * radius * 3.14f;
	if (area < minArea) 
	{
		cout << "Size is too small, setting size to minimum value";
		area = minArea;
	}
	else if (area > maxArea) 
	{
		cout << "Size is too big, setting size to maximum value";
		area = maxArea;
	}
	if (density < minDensity)
	{
		cout << "Density is too small, setting density to minimum value";
		density = minDensity;
	}
	else if (density > maxDensity)
	{
		cout << "Density is too big, setting density to maximum value";
		density = maxDensity;
	}
	if (restitution > 1) 
	{
		restitution = 1;
	}
	if(restitution <0)
	{
		restitution = 0;
	}
	float mass = area * (float)density / 10.f;
	EpsilonBody bd(position, density, mass, restitution, area, radius, 0.f, 0.f, isStatic, Shapetype::circle);
	return bd;
}

EpsilonBody EpsilonBody::CreateBoxBody(Vector2f position, float density, float restitution, float width, float height, bool isStatic)
{
	float area = width * height;
	if (area < minArea)
	{
		cout << "Size is too small, setting size to minimum value";
		area = minArea;
	}
	else if (area > maxArea)
	{
		cout << "Size is too big, setting size to maximum value";
		area = maxArea;
	}
	if (density < minDensity)
	{
		cout << "Density is too small, setting density to minimum value";
		density = minDensity;
	}
	else if (density > maxDensity)
	{
		cout << "Density is too big, setting density to maximum value";
		density = maxDensity;
	}
	if (restitution > 1)
	{
		restitution = 1;
	}
	if (restitution < 0)
	{
		restitution = 0;
	}
	float mass = area * (float)density/10.f;
	EpsilonBody bd(position, density, mass, restitution, area, 0.f, width, height, isStatic, Shapetype::box);
	return bd;
}
void EpsilonBody::updateMovement(float& dt, Vector2f gravity)
{
	if (isStatic) {
		return;
	}
	AddForce(gravity);
	Vector2f acceleration = force / mass;
	isTransformUpdated = false;
	isAABBUpdated = false;
	float converter = (float)1 / (float)10000;
	linearVelocity += acceleration * dt * converter;
	linearVelocity *= drag;
	position += linearVelocity * dt * converter;
	force = Vector2f({ 0,0 });
}

void EpsilonBody::Move(Vector2f amount)
{
	isTransformUpdated = false;
	isAABBUpdated = false;
	position += amount;
}

vector<Vector2f> EpsilonBody::GetBoxVertices(float width, float height)
{
	vector<Vector2f> vertices(4);
	float left = -width/2.f;
	float right = left+width;
	float bottom = height/2.f;
	float top = bottom-height;
	vertices[0] = Vector2f(left,top);
	vertices[1] = Vector2f(right, top);
	vertices[2] = Vector2f(right, bottom);
	vertices[3] = Vector2f(left, bottom);
	return vertices;
}

vector<int> EpsilonBody::GetBoxTriangles()
{
	vector<int> triangles(6);
	triangles[0] = 0;
	triangles[1] = 1;
	triangles[2] = 2;
	triangles[3] = 0;
	triangles[4] = 2;
	triangles[5] = 3;
	return triangles;
}

Vector2f EpsilonBody::Transform(Vector2f position,Vector2f endposition, float angle)
{
	float rx = cos(angle) * position.x - sin(angle) * position.y;
	float ry = sin(angle) * position.x + cos(angle) * position.y;
	float tx = rx + endposition.x;
	float ty =  ry+endposition.y;
	return Vector2f(tx,ty);
}

void EpsilonBody::MoveTo(Vector2f& pos)
{
	position = pos;
	isTransformUpdated = false;
	isAABBUpdated = false;
}

void EpsilonBody::UpdateRotation(float angle)
{
	rotation += angle;
	isTransformUpdated = false;
	isAABBUpdated = false;
}

void EpsilonBody::AddForce(Vector2f amount)
{
	force += amount;
}

vector<Vector2f> EpsilonBody::GetTransformedVertices()
{
	if (!isTransformUpdated) {
		Vector2f endposition = position;
		for (size_t i = 0; i < vertices.size(); i++) {
			Vector2f v = vertices[i];
			transformedVertices[i] = Transform(v, endposition, rotation);
		}
	}
	return transformedVertices;
}

AABB EpsilonBody::GetAABB()
{
	if (!isAABBUpdated) {
		float minX = FLT_MAX;
		float minY = FLT_MAX;
		float maxX = FLT_MIN;
		float maxY = FLT_MIN;
		if (shapetype == box) {
			vector<Vector2f> vertices = GetTransformedVertices();
			for (size_t i = 0; i < vertices.size(); i++) {
				Vector2f v = vertices[i];
				if (v.x < minX) { minX = v.x; }
				if (v.x > maxX) { maxX = v.x; }
				if (v.y < minY) { minY = v.y; }
				if (v.y > maxY) { maxY = v.y; }

			}
		}
		else if (shapetype == circle) {
			minX = position.x - radius;
			maxX = position.x + radius;
			minY = position.y - radius;
			maxY = position.y + radius;
		}
		aabb = AABB(minX, maxX, minY, maxY);
	}
	isAABBUpdated = true;
	return aabb;
}
