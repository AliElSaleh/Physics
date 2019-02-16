#pragma once
#include "Object.h"

#include <glm/vec2.hpp>
#include "Collider.h"

class Circle final : public Object, public Collider
{
public:
	Circle(glm::vec2 Location, glm::vec2 Velocity, float Radius, float Mass, glm::vec4 Color);
	~Circle();

	void Debug() override;
	void MakeGizmo() override;

	float GetRadius() const { return Radius; }

protected:
	float Radius{};
};

