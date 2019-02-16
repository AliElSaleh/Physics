#pragma once
#include <glm/vec2.hpp>
#include <glm/vec4.hpp>

enum Geometry
{
	SQUARE, CIRCLE, LAST
};

static const float MIN_LINEAR_THRESHOLD = 0.1f;

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
	glm::vec2 GetNormal() const { return Normal; }
	

	void SetKinematic(const bool State) { bIsKinematic = State; }
	bool IsKinematic() const { return bIsKinematic; }

	Geometry GetShape() const { return Shape; }

	float GetMass() const { return Mass; }
	float GetInverseMass() const { return InverseMass; }
	float GetRestitution() const { return Restitution; }

	glm::vec2 ContactLocation{};
protected:
	float Mass{1.0f};
	float InverseMass{ 1.0f / Mass };
	float Restitution{0.6f};
	float PenetrationDepth{0.0f};
	float LinearDrag{0.8f};

	glm::vec2 Location{};
	glm::vec2 Velocity{1.0f, 1.0f};
	glm::vec2 Normal{ 0.0f, 1.0f };

	glm::vec4 Color{1.0f, 1.0f, 1.0f, 1.0f};

	Geometry Shape{};

	bool bIsKinematic{false};
};

