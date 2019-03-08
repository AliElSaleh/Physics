#pragma once
#include "Object.h"

#include <glm/vec2.hpp>

class Circle final : public Object
{
public:
	Circle(glm::vec2 Location, glm::vec2 Velocity, float Radius, float Mass, glm::vec4 Color);
	~Circle();

	void Debug() override;
	void MakeGizmo() override;

	float GetRadius() const { return Radius; }

	bool Collided{false};
private:
	float Radius{};
};

