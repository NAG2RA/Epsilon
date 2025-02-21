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
	linearVelocity(0, 0)
{
}

EpsilonBody EpsilonBody::CreateNewBody(EpsilonBody& body)
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
	
	float mass = area * density;
	EpsilonBody bd(position, density, mass, restitution, area, radius, 0.f, 0.f, false, Shapetype::circle);
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

	float mass = area * density;
	EpsilonBody bd(position, density, mass, restitution, area, 0.f, width, height, false, Shapetype::box);
	return bd;
}
Vector2f EpsilonBody::updateMovement(Vector2f& position, float& dt, Vector2f& velocity, Vector2f& acceleration, float& drag)
{
	float converter = (float)1 / (float)10000;

	velocity += acceleration * dt * converter;

	velocity *= drag;
	
	position += velocity * dt * converter;
	return position;
}
