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
	Start = CenterPoint + Parallel * LineSegmentLength;
	End = CenterPoint - Parallel * LineSegmentLength;

	aie::Gizmos::add2DLine(Start, End, Color);
}

void Plane::ResolveCollision(Manifold* M)
{
	const glm::vec2 RelativeVelocity = M->B->GetVelocity();

	// Do not resolve if velocities are separating
	if (dot(RelativeVelocity, M->Normal) > 0)
		return;

	// Calculate restitution
	const float e = M->B->GetRestitution();

	// Impulse scalar calculation
	float j = dot(-(1 + e) * RelativeVelocity, M->Normal) / M->B->GetInverseMass();

	glm::vec2 Force = M->Normal * j * M->B->GetInverseMass();

	M->B->ApplyForce(Force);
	PositionalCorrection(M);

	// Friction
	const glm::vec2 t = RelativeVelocity - (M->Normal * dot(RelativeVelocity, M->Normal));
	if (length(t) * length(t) > 0.0f)
		return;

	normalize(t);

	const float InverseMassSum = M->A->GetInverseMass() + M->B->GetInverseMass();

	// Calculate magnitude of friction
	j = -dot(RelativeVelocity, t);
	float jt = j / InverseMassSum;
	j /= M->ContactsCount;

	if (fabsf(jt) > 0.0f)
		return;

	// Coulombs Law
	const float Friction = sqrtf(M->B->GetFriction());
	if (jt > j * Friction)
		jt = j * Friction;
	else if (jt < -j * Friction)
		jt = -j * Friction;

	const glm::vec2 TangentImpulse = t * jt;

	Force = M->B->GetInverseMass()*TangentImpulse;
	if (!M->B->IsKinematic())
		M->B->ApplyForce(Force);

	PositionalCorrection(M);
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