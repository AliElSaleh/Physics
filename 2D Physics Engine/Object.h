#pragma once
#include <glm/vec2.hpp>
#include <glm/vec4.hpp>

enum Geometry
{
	SQUARE, CIRCLE, LAST
};

class Object
{
public:
	Object();
	virtual ~Object() = default;

	void ApplyForce(glm::vec2 Force);

	virtual void FixedUpdate(glm::vec2 Gravity, float TimeStep);
	virtual void Debug() = 0;
	virtual void MakeGizmo() = 0;

	glm::vec2 GetLocation() const { return Location; }
	glm::vec2 GetVelocity() const { return  Velocity; }

	Geometry GetShape() const { return Shape; }

protected:
	float Mass{1.0f};

	glm::vec2 Location{};
	glm::vec2 Velocity{1.0f, 1.0f};

	glm::vec4 Color{1.0f, 1.0f, 1.0f, 1.0f};

	Geometry Shape{};
};

