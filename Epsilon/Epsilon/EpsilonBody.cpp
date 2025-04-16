#include "EpsilonBody.h"
float minArea = 0.01f * 0.01f;
float maxArea = 64 * 64;
float minDensity = 0.5f;
float maxDensity = 21.4f;
EpsilonBody::EpsilonBody(EpsilonVector position, float density, float mass, float inertia, float restitution, float area, float radius, float width,
	float height, vector<EpsilonVector> vertices, bool isStatic, Shapetype shapetype)
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
	angle(0.f),
	angularVelocity(0.f),
	linearVelocity(0, 0),
	force(0, 0),
	aabb(0, 0, 0, 0),
	inertia(inertia),
	inverseMass(mass > 0 ? (float)1 / mass : 0),
	inverseInertia(inertia > 0 ? (float)1 / inertia : 0),
	vertices(vertices),
	dynamicFriction(0.4f),
	staticFriction(0.6f)
{
	if (shapetype == Shapetype::box) 
	{
		vertices = GetBoxVertices(width, height);
		transformedVertices.resize(vertices.size());
	}
	else if (shapetype == Shapetype::triangle) {
		vertices = GetTriangleVertices(width);
		transformedVertices.resize(vertices.size());
	}
	else
	{
		vertices = {};
		transformedVertices = {};
	}
	isTransformUpdated = false;
	isAABBUpdated = false;
}

EpsilonBody EpsilonBody::CreateNewBody(EpsilonBody body)
{
	return EpsilonBody(body.position,body.density,body.mass, body.inertia, body.restitution,body.area,body.radius,body.width,body.height, body.vertices, body.isStatic,body.shapetype);
}

EpsilonBody EpsilonBody::CreateCircleBody(EpsilonVector position, float density, float restitution, float radius, bool isStatic)
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
	float mass;
	float inertia;
	if (isStatic) {
		mass = 0;
		inertia = 0;
	}
	else {
		mass = area * (float)density;
		inertia = (1.f / 2.f) * mass * radius * radius;
	}
	EpsilonBody bd(position, density, mass, inertia, restitution, area, radius, 0.f, 0.f, {}, isStatic, Shapetype::circle);
	return bd;
}

EpsilonBody EpsilonBody::CreateBoxBody(EpsilonVector position, float density, float restitution, float width, float height, bool isStatic)
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
	float mass;
	float inertia;
	if (isStatic) {
		mass = 0;
		inertia = 0;
	}
	else {
		mass = area * (float)density;
		inertia = (1.f / 12.f) * mass * (width * width + height * height);
	}
	vector<EpsilonVector> vertices = GetBoxVertices(width,height);
	EpsilonBody bd(position, density, mass, inertia, restitution, area, 0.f, width, height, vertices, isStatic, Shapetype::box);
	return bd;
}
EpsilonBody EpsilonBody::CreateTriangleBody(EpsilonVector position, float density, float restitution, float side, bool isStatic)
{
	float area = (side*side*sqrt(3))/4;
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
	float mass;
	float inertia;
	float height = (side * sqrt(3)) / 2;
	if (isStatic) {
		mass = 0;
		inertia = 0;
	}
	else {
		mass = area * (float)density;
		inertia = (1.f / 36.f)*side*height*height*height;
	}
	vector<EpsilonVector> vertices = GetTriangleVertices(side);
	EpsilonBody bd(position, density, mass, inertia, restitution, area, 0.f, side, height, vertices, isStatic, Shapetype::triangle);
	return bd;
}
void EpsilonBody::updateMovement(float dt, EpsilonVector gravity,int iterations)
{
	if (isStatic) {
		return;
	}
	dt = dt / (float)iterations;
	EpsilonVector acceleration = force / mass;
	isTransformUpdated = false;
	isAABBUpdated = false;
	linearVelocity += gravity * dt;
	linearVelocity += acceleration*dt;
	angle += angularVelocity * dt;
	position += linearVelocity * dt;
	force = EpsilonVector({ 0,0 });
}

void EpsilonBody::Move(EpsilonVector amount)
{
	isTransformUpdated = false;
	isAABBUpdated = false;
	position += amount;
}

vector<EpsilonVector> EpsilonBody::GetBoxVertices(float width, float height)
{
	vector<EpsilonVector> vertices(4);
	float left = -width/2.f;
	float right = left+width;
	float bottom = height/2.f;
	float top = bottom-height;
	vertices[0] = EpsilonVector(left,top);
	vertices[1] = EpsilonVector(right, top);
	vertices[2] = EpsilonVector(right, bottom);
	vertices[3] = EpsilonVector(left, bottom);
	return vertices;
}
vector<EpsilonVector> EpsilonBody::GetTriangleVertices(float side)
{
	float height = (side * sqrt(3)) / 2;
	vector<EpsilonVector> vertices(3);
	float left = -side / 2.f;
	float right = left + side;
	float bottom = height/2.f;
	float top = bottom - height;
	vertices[0] = EpsilonVector(0.f, top);
	vertices[1] = EpsilonVector(left, bottom);
	vertices[2] = EpsilonVector(right, bottom);
	return vertices;
}


EpsilonVector EpsilonBody::Transform(EpsilonVector position, EpsilonVector endposition, float angle)
{
	float rx = cos(angle) * position.x - sin(angle) * position.y;
	float ry = sin(angle) * position.x + cos(angle) * position.y;
	float tx = rx + endposition.x;
	float ty =  ry+endposition.y;
	return EpsilonVector(tx,ty);
}

void EpsilonBody::MoveTo(EpsilonVector& pos)
{
	position = pos;
	isTransformUpdated = false;
	isAABBUpdated = false;
}

void EpsilonBody::UpdateRotation(float angle)
{
	angle += angle;
	isTransformUpdated = false;
	isAABBUpdated = false;
}

void EpsilonBody::AddForce(EpsilonVector amount)
{
	force += amount;
}

vector<EpsilonVector> EpsilonBody::GetTransformedVertices()
{
	if (!isTransformUpdated) {
		EpsilonVector endposition = position;
		for (size_t i = 0; i < vertices.size(); i++) {
			EpsilonVector v = vertices[i];
			transformedVertices[i] = Transform(v, endposition, angle);
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
		if (shapetype == box||shapetype == triangle) {
			vector<EpsilonVector> vertices = GetTransformedVertices();
			for (size_t i = 0; i < vertices.size(); i++) {
				EpsilonVector v = vertices[i];
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
