#include "Circle.h"
#include "Gizmos.h"
#include <glm/ext.hpp>

#include <stdio.h>

Circle::Circle(const glm::vec2 Location, const glm::vec2 Velocity, const float Radius, const float Mass, const glm::vec4 Color)
	: Collider(CIRCLE)
{
	this->Location = Location;
	this->Velocity = Velocity;
	this->Radius = Radius;
	this->Mass = Mass;
	
	if (this->Mass == 0)
		this->InverseMass = 0;
	else
		this->InverseMass = 1.0f / Mass;
	
	this->Normal = normalize(Velocity);

	this->Color = Color;

	Shape = Geometry::CIRCLE;
}

Circle::~Circle() = default;

void Circle::Debug()
{
	printf("X: %f, Y: %f\n", Location.x, Location.y);
	printf("Radius: %f\n", Radius);
}

void Circle::MakeGizmo()
{
	aie::Gizmos::add2DCircle(Location, Radius, 30, Color);

	//aie::Gizmos::add2DCircle(Location, 0.5f, 30, {0.0f, 1.0f, 0.0f, 1.0f});
}
