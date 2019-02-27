#include "Plane.h"
#include "Gizmos.h"
#include <glm/ext.hpp>

Plane::Plane()
{
	DistanceToOrigin = 0;
	Normal = { 0.0f, 1.0f };

	Shape = PLANE;
}

Plane::Plane(const glm::vec2 Normal, const float Distance)
{
	DistanceToOrigin = Distance;
	this->Normal = Normal;

	Shape = PLANE;
}

Plane::~Plane() = default;

void Plane::Debug()
{
}

void Plane::MakeGizmo()
{
	const float LineSegmentLength = 300;
	const glm::vec2 CenterPoint = Normal * DistanceToOrigin;

	const glm::vec2 Parallel = { Normal.y, -Normal.x };
	const glm::vec4 Color = { 1.0f, 1.0f, 1.0f, 1.0f };
	const glm::vec2 Start = CenterPoint + Parallel * LineSegmentLength;
	const glm::vec2 End = CenterPoint - Parallel * LineSegmentLength;

	aie::Gizmos::add2DLine(Start, End, Color);
}

void Plane::ResolveCollision(Manifold* M)
{
	const glm::vec2 RelativeVelocity = M->B->GetVelocity();

	// Calculate restitution
	const float e = M->B->GetRestitution();

	// Impulse scalar calculation
	const float j = dot(-(1 + e) * RelativeVelocity, M->Normal) / (M->B->GetInverseMass());

	const glm::vec2 Force = M->Normal * j;

	M->B->ApplyForce(Force);
	PositionalCorrection(M);

	M->B->bCollided = true;
}

void Plane::PositionalCorrection(Manifold* M)
{
	auto A = dynamic_cast<Object*>(M->A);
	auto B = dynamic_cast<Object*>(M->B);

	if (A != nullptr && B != nullptr)
	{
		const float PenetrationDepthAllowance = 0.03f;
		const float PenetrationCorrection = 3.0f;

		const glm::vec2 Correction = glm::max(M->Penetration - PenetrationDepthAllowance, 0.0f) / (A->GetInverseMass() + B->GetInverseMass()) * M->Normal * PenetrationCorrection;

		if (!A->IsKinematic())
			A->ApplyForce(-Correction * A->GetInverseMass());

		if (!B->IsKinematic())
			B->ApplyForce(Correction*B->GetInverseMass());
	}
}