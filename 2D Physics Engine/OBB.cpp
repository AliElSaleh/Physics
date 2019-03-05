#include "OBB.h"
#include "Gizmos.h"

#include <glm/ext.hpp>
#include <cstdio>

OBB::OBB()
{
	this->Location = {0.0f, 0.0f};
	this->HalfExtent = {1.0f, 1.0f};
	this->Rotation = 0.0f;
	this->Mass = 1;
	this->InverseMass = 1;
	this->LinearDrag = 0.0f;
	this->AngularDrag = 0.0f;
	this->Color = {1.0f, 0.0f, 0.0f, 1.0f};

	Shape = Geometry::OBB;
}

OBB::OBB(glm::vec2 Location, glm::vec2 Velocity, glm::vec2 Extent, float Rotation, float Mass, glm::vec4 Color)
{
	this->Location = Location;
	this->Velocity = Velocity;
	this->HalfExtent = Extent;
	this->Rotation = Rotation;
	this->Mass = Mass;
	this->Color = Color;

	this->LinearDrag = 0.3f;
	this->AngularDrag = 0.3f;

	if (this->Mass == 0)
		this->InverseMass = 0;
	else
		this->InverseMass = 1.0f / Mass;
	
	this->AngularVelocity = 0.0f;
	this->Moment = 1.0f;

	Shape = Geometry::OBB;
}

OBB::~OBB() = default;

void OBB::FixedUpdate(const glm::vec2 Gravity, const float TimeStep)
{
	Object::FixedUpdate(Gravity, TimeStep);

	// Store the local axes
	const float CS = cosf(DEG2RAD(Rotation));
	const float SN = sinf(DEG2RAD(Rotation));

	Transform = {CS, SN,0, 0,
				 -SN, CS, 0, 0,
				 0,  0,  1.0f, 0,
				 0,  0,  0, 1.0f};
}

void OBB::Debug()
{
	printf("Location X: %f, Y: %f\n", Location.x, Location.y);
	printf("Rotation: %f\n", Rotation);
	//printf("AngVelocity: %f\n", AngularVelocity);
}

void OBB::MakeGizmo()
{
	// Box
	aie::Gizmos::add2DAABBFilled(Location, HalfExtent, Color, &Transform);
	
	// Location
	aie::Gizmos::add2DCircle(Location, 0.5f, 30, {1.0f, 1.0f, 1.0f, 1.0f});
}
