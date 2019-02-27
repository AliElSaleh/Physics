#pragma once
#include <glm/vec2.hpp>
#include <glm/vec4.hpp>

enum Geometry
{
	AABB, SQUARE, CIRCLE, PLANE, LAST
};

static const float MIN_LINEAR_THRESHOLD = 0.1f;
static const float MIN_ROTATION_THRESHOLD = 0.01f;

class Object
{
public:
	Object();
	virtual ~Object() = default;

	void ApplyForce(glm::vec2 Force);

	float Cross(glm::vec2 A, glm::vec2 B);
	glm::vec2 Cross(float S, glm::vec2 A);

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
	float GetAngularVelocity() const { return AngularVelocity; }
	float GetMoment() const { return Moment; }

	bool bCollided{false};
protected:
	float Rotation{0.0f};
	float Mass{1.0f};
	float InverseMass{ 1.0f / Mass };
	float Restitution{1.0f};
	float LinearDrag{0.3f};
	float AngularDrag{0.3f};
	float AngularVelocity{0.0f};
	float Moment{1.0f};
	float InverseMoment{ 1.0f / Moment };
	float Torque{0.0f};

	glm::vec2 Location{};
	glm::vec2 Velocity{1.0f, 1.0f};
	glm::vec2 Normal{};

	glm::vec4 Color{1.0f, 1.0f, 1.0f, 1.0f};

	Geometry Shape{};

	bool bIsKinematic{false};
};

