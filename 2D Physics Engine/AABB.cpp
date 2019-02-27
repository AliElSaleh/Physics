#include "AABB.h"
#include "Gizmos.h"

#include <glm/ext.hpp>
#include <stdio.h>

AABB::AABB(const glm::vec2 Location, const glm::vec2 Velocity, const float Width, const float Height, const float Mass, const glm::vec4 Color)
	: Collider(Type::AABB)
{
	this->Location = Location;
	this->Velocity = Velocity;
	this->Extent = { Width / 2, Height / 2 };
	this->Width = Extent.x * 2;
	this->Height = Extent.y * 2;
	this->Mass = Mass;
	this->LinearDrag = 0.3f;
	this->AngularDrag = 0.3f;

	if (this->Mass == 0)
		this->InverseMass = 0;
	else
		this->InverseMass = 1.0f / Mass;

	this->Min = Location - Extent;
	this->Max = Location + Extent;

	this->AngularVelocity = 0.0f;
	this->Moment = 1.0f;

	this->Normal = normalize(Velocity);

	this->Color = Color;

	Shape = Geometry::AABB;
}

AABB::~AABB() = default;

void AABB::FixedUpdate(const glm::vec2 Gravity, const float TimeStep)
{
	Object::FixedUpdate(Gravity, TimeStep);

	Min = Location - Extent;
	Max = Location + Extent;
}

void AABB::Debug()
{
	printf("X: %f, Y: %f\n", Location.x, Location.y);
	printf("Size X: %f, Y:%f\n", Width, Height);
	printf("Extent X: %f, Y:%f\n", Extent.x, Extent.y);
}

void AABB::MakeGizmo()
{
	// The AABB
	aie::Gizmos::add2DAABBFilled(Location, Extent, Color);

	// The extent locations
	//aie::Gizmos::add2DCircle(Min, 1.0f, 30, { 0.0f, 1.0f, 0.0f, 1.0f });
	//aie::Gizmos::add2DCircle(Max, 1.0f, 30, { 0.0f, 1.0f, 0.0f, 1.0f });
	//
	//// Location
	//aie::Gizmos::add2DCircle(Location, 0.5f, 30, {1.0f, 0.0f, 0.0f, 1.0f});

	// Lines at x and y
	// aie::Gizmos::add2DLine(Location - Extent, Location + Extent, {1.0f, 1.0f, 1.0f, 1.0f});
}

