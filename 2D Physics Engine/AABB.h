#pragma once
#include "Object.h"

#include <glm/vec2.hpp>
#include "Collider.h"

class AABB final : public Object, public Collider
{
public:
	AABB(glm::vec2 Location, glm::vec2 Velocity, float Width, float Height, float Mass, glm::vec4 Color);
	~AABB();

	void FixedUpdate(glm::vec2 Gravity, float TimeStep) override;
	void Debug() override;
	void MakeGizmo() override;

	glm::vec2 GetExtent() const { return Extent; }

	float GetWidth() const { return Extent.x * 2; }
	float GetHeight() const { return Extent.y * 2; }

	glm::vec2 GetMin() const { return Min; }
	glm::vec2 GetMax() const { return Max; }

protected:
	glm::vec2 Extent{};
	

	float Width{}, Height{};

	glm::vec2 Min{}; // Lower bounds of x and y axis (Top left)
	glm::vec2 Max{}; // Higher bounds of x and y axis (Bottom right)
};

