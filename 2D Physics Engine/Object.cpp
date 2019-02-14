#include "Object.h"

Object::Object() = default;

void Object::ApplyForce(const glm::vec2 Force)
{
	Velocity += Force / Mass; // A = F / M formula
}

void Object::FixedUpdate(const glm::vec2 Gravity, const float TimeStep)
{
	ApplyForce(Gravity * Mass * TimeStep);
	Location += Velocity * TimeStep;
}
