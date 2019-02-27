#pragma once
#include "Object.h"
#include <glm/mat4x4.hpp>

class Box final : public Object
{
public:
	Box(glm::vec2 Location, glm::vec2 Velocity, float Width, float Height, float Mass, glm::vec4 Color);
	~Box();

	void FixedUpdate(glm::vec2 Gravity, float TimeStep) override;
	void Debug() override;
	void MakeGizmo() override;

private:
	glm::vec2 Extent{};
	glm::vec2 ExtentOutline{};
	glm::vec2 Min{}, Max{};	// Lower bounds of x and y axis (Top left)
							// Higher bounds of x and y axis (Bottom right)

	glm::mat4 Transform{};

	float Width{}, Height{};
};

