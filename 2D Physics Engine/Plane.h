#pragma once
#include "Object.h"
#include "Manifold.h"

class Plane final : public Object
{
public:
	Plane();
	Plane(glm::vec2 Normal, float Distance);
	~Plane();

	void Debug() override;
	void MakeGizmo() override;

	void ResolveCollision(Manifold* M);
	void PositionalCorrection(Manifold* M);

	float GetDistance() const { return DistanceToOrigin; }
private:
	float DistanceToOrigin{};
};

