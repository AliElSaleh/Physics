#include "Circle.h"
#include "Gizmos.h"
#include <glm/ext.hpp>

#include <stdio.h>

Circle::Circle(const glm::vec2 Location, const glm::vec2 Velocity, const float Radius, const float Mass, const glm::vec4 Color)
{
	this->Location = Location;
	this->Velocity = Velocity;
	this->Radius = Radius;
	this->Mass = Mass;
	this->LinearDrag = 0.3f;
	this->AngularDrag = 0.3f;
	this->Torque = 1.0f;

	if (this->Mass == 0)
		this->InverseMass = 0;
	else
		this->InverseMass = 1.0f / Mass;
	
//	this->Normal = normalize(Velocity);

	this->AngularVelocity = 0.0f;
	this->Moment = 0.5f * Mass * Radius * Radius;

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
	// Circle
	aie::Gizmos::add2DCircle(Location, Radius, 30, Color);

	const glm::vec2 End = glm::vec2(cosf(Rotation), sinf(Rotation)) * Radius;
	aie::Gizmos::add2DLine(Location, Location + End, { 1.0f, 1.0f, 1.0f, 1.0f });
}
