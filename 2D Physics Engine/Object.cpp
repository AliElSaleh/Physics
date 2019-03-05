#include "Object.h"

#include <glm/ext.hpp>

Object::Object() = default;

void Object::ApplyForce(const glm::vec2 Force)
{
	Velocity += Force / Mass; // A = F / M formula
	AngularVelocity += (Force.y * Location.x - Force.x * Location.y) / Moment;
}

void Object::FixedUpdate(const glm::vec2 Gravity, const float TimeStep)
{
	if (bIsKinematic)
	{
		Velocity = { 0.0f, 0.0f };
		AngularVelocity = 0.0f;
		LinearDrag = 0.0f;
		AngularDrag = 0.0f;
		return;
	}

	ApplyForce(Gravity * Mass * TimeStep);
	Location += Velocity * TimeStep;
	Velocity -= Velocity * Friction * LinearDrag * TimeStep;

	Rotation += AngularVelocity * TimeStep;
	AngularVelocity -= AngularDrag * AngularDrag * TimeStep;

	if (length(Velocity) < MIN_LINEAR_THRESHOLD)
		Velocity = glm::vec2(0.0f, 0.0f);
	
	if (fabs(AngularVelocity) > MIN_ROTATION_THRESHOLD)
		AngularVelocity = 0.0f;
}
