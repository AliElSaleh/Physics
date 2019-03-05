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

	glm::vec2 GetStart() const { return Start; }
	glm::vec2 GetEnd() const { return End; }
private:
	float DistanceToOrigin{};

	glm::vec2 Start{}, End{};
};

