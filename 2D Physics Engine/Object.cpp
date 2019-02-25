#include "Object.h"

#include <glm/ext.hpp>

Object::Object() = default;

void Object::ApplyForce(const glm::vec2 Force)
{
	Velocity += Force / Mass; // A = F / M formula
}

void Object::FixedUpdate(const glm::vec2 Gravity, const float TimeStep)
{
	if (bIsKinematic)
	{
		Velocity = { 0.0f, 0.0f };
		LinearDrag = 0.0f;
		return;
	}

	ApplyForce(Gravity * Mass * TimeStep);
	Location += Velocity * TimeStep;

	Velocity -= Velocity * LinearDrag * TimeStep;

	if (length(Velocity) < MIN_LINEAR_THRESHOLD)
		Velocity = glm::vec2(0.0f, 0.0f);
}
