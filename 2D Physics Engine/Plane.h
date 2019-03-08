#pragma once
#include "Object.h"
#include "Manifold.h"

class Plane final : public Object
{
public:
	Plane();
	Plane(glm::vec2 Normal, float Distance, float LineLength);
	~Plane();

	void Debug() override;
	void MakeGizmo() override;

	void ResolveCollision(Manifold* M);
	void PositionalCorrection(Manifold* M);

	float GetDistance() const { return DistanceToOrigin; }

	glm::vec2 GetStart() const { return Start; }
	glm::vec2 GetEnd() const { return End; }

	void SetSegmentLength(const float Length) { LineSegment = Length; }
	void SetStart(const glm::vec2 Start) { this->Start = Start; }
	void SetEnd(const glm::vec2 End) { this->End = End; }

	void SetDistance(const float Distance) { this->DistanceToOrigin = Distance; }
private:
	float DistanceToOrigin{};
	float LineSegment = 300;

	glm::vec2 Start{}, End{};
};

