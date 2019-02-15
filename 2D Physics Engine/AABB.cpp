#include "AABB.h"
#include "Gizmos.h"
#include <stdio.h>

AABB::AABB(const glm::vec2 Location, glm::vec2 Velocity, const float Width, const float Height, const float Mass, const glm::vec4 Color)
{
	this->Location = Location;
	this->Velocity = Velocity;
	this->Extent = { Width / 2, Height / 2 };
	this->Width = Extent.x * 2;
	this->Height = Extent.y * 2;
	this->Mass = Mass;

	if (this->Mass == 0)
		this->InverseMass = 0;
	else
		this->InverseMass = 1.0f / Mass;
	
	this->Normal = { 1.0f, 0.0f };

	this->Color = Color;

	Shape = SQUARE;
}

AABB::~AABB() = default;

void AABB::FixedUpdate(const glm::vec2 Gravity, const float TimeStep)
{
	Object::FixedUpdate(Gravity, TimeStep);
}

void AABB::Debug()
{
	printf("X: %f, Y: %f\n", Location.x, Location.y);
	printf("Size X: %f, Y:%f\n", Width, Height);
	printf("Extent X: %f, Y:%f\n", Extent.x, Extent.y);
}

void AABB::MakeGizmo()
{
	aie::Gizmos::add2DAABBFilled(Location, Extent, Color);
}

