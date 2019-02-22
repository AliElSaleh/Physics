#include "World.h"
#include "AABB.h"
#include "Circle.h"

#include <cstdio>
#include <glm/ext.hpp>
#include <glm/mat2x2.hpp>
#include <stdio.h>
#include "Gizmos.h"
World::World() = default;
World::~World() = default;

typedef bool(*CollisionFn)(Manifold*);

static CollisionFn CollisionFunctionArray[] =
{
	World::AABBToAABB, World::CircleToAABB, World::CircleToCircle, World::AABBToCircle
};

void World::AddActor(Object* Actor)
{
	Actors.emplace_back(Actor);
}

void World::RemoveActor(Object* Actor)
{
	const auto FoundActor = std::find(Actors.begin(), Actors.end(), Actor);

	if (FoundActor < Actors.end())
		Actors.erase(FoundActor);
}

void World::Update(const float DeltaTime)
{
	static float AccumulatedTime = 0.0f;
	AccumulatedTime += DeltaTime;
	
	if (AccumulatedTime >= 0.2f)
		AccumulatedTime = 0.2f;
	
	while (AccumulatedTime >= DeltaTime)
	{
		for (auto Actor : Actors)
			Actor->FixedUpdate(Gravity, TimeStep);
	
		CheckForCollisions();
	
		AccumulatedTime -= TimeStep;
	}
}

void World::UpdateGizmos()
{
	for (auto Actor : Actors)
		Actor->MakeGizmo();
}

void World::CheckForCollisions()
{
	const int ActorCount = Actors.size();

	for (int Outer = 0; Outer < ActorCount - 1; Outer++)
	{
		for (int Inner = Outer + 1; Inner < ActorCount; Inner++)
		{
			Object* Object1 = Actors[Outer];
			Object* Object2 = Actors[Inner];
			auto* M = new Manifold();
			M->A = Object1;
			M->B = Object2;
			const int Shape1 = Object1->GetShape();
			const int Shape2 = Object2->GetShape();

			const unsigned short FunctionIndex = Shape1 + Shape2;
			const CollisionFn CollisionFunctionPtr = CollisionFunctionArray[FunctionIndex];
			
			if (CollisionFunctionPtr != nullptr)
				CollisionFunctionPtr(M);
		}
	}
}

bool World::AABBToAABB(Manifold* M)
{
	const auto Rec1 = dynamic_cast<AABB*>(M->A);
	const auto Rec2 = dynamic_cast<AABB*>(M->B);

	if (Rec1 != nullptr && Rec2 != nullptr)
	{
		if (Rec1->GetLocation().x + Rec1->GetExtent().x > Rec2->GetLocation().x - Rec2->GetExtent().x &&
			Rec1->GetLocation().x - Rec1->GetExtent().x < Rec2->GetLocation().x + Rec2->GetExtent().x &&
			Rec1->GetLocation().y + Rec1->GetExtent().y > Rec2->GetLocation().y - Rec2->GetExtent().y &&
			Rec1->GetLocation().y - Rec1->GetExtent().y < Rec2->GetLocation().y + Rec2->GetExtent().y)
		{
			glm::vec2 CollisionNormal1;

			const glm::vec2 Size = {fabsf(Rec1->GetWidth()), fabsf(Rec1->GetHeight()) };

			const glm::vec2 A = { fabsf(Size.x), fabsf(Size.y) };

			const glm::vec2 S = { Rec1->GetLocation().x < Rec2->GetLocation().x ? -1.0f : 1.0f,
							Rec1->GetLocation().y < Rec2->GetLocation().y ? -1.0f : 1.0f};

			if (A.x <= A.y)
				CollisionNormal1 = glm::vec2(S.x, 0.0f);
			else
				CollisionNormal1 = glm::vec2(0.f, S.y);

			const glm::vec2 CollisionNormal2 = -CollisionNormal1;

			M->ContactsCount = 1;

			M->Normal = CollisionNormal2;

			M->Penetration = length(Size) * Rec1->GetInverseMass()/Rec2->GetInverseMass();
			
			ResolveCollision(M);
			
			if (Rec1->IsKinematic())
				printf("AABBToAABB (Kinematic): Collided!\n");
			
			if (Rec2->IsKinematic())
				printf("AABBToAABB (Kinematic): Collided!\n");
			
			if (!Rec1->IsKinematic() && !Rec2->IsKinematic())
				printf("AABBToAABB: Collided!\n");
			
			return true;
		}
	}

	// Error checks
	if (Rec1 == nullptr && Rec2 != nullptr)
		printf("AABBToAABB: Rec1 is null\n");
	
	if (Rec1 != nullptr && Rec2 == nullptr)
		printf("AABBToAABB: Rec2 is null\n");
	
	if (Rec1 == nullptr && Rec2 == nullptr)
		printf("AABBToAABB: Both of the objects were null\n");
	
	return false;
}

bool World::CircleToAABB(Manifold* M)
{
	const auto Rec = dynamic_cast<AABB*>(M->A);
	const auto Circle = dynamic_cast<::Circle*>(M->B);

	if (Rec != nullptr && Circle != nullptr)
	{
		M->ContactsCount = 0;
		
		glm::vec2 PotentialCollision;
		PotentialCollision.x = glm::clamp(Circle->GetLocation().x, Rec->GetMin().x, Rec->GetMax().x);
		PotentialCollision.y = glm::clamp(Circle->GetLocation().y, Rec->GetMin().y, Rec->GetMax().y);
		
		// Closest Point of Rec to center of Circle
		const glm::vec2 Closest = Circle->GetLocation() - PotentialCollision;
		
		const float Distance = length(Closest);

		if (Distance < Circle->GetRadius())
		{
			M->ContactsCount = 1;
			M->Penetration = Distance * Rec->GetMass() + Circle->GetMass();
			M->Normal = normalize(Closest);
		
			ResolveCollision(M);
		
			if (Rec->IsKinematic())
				printf("AABBToCircle (Kinematic): Collided!\n");
		
			if (Circle->IsKinematic())
				printf("AABBToCircle (Kinematic): Collided!\n");
		
			if (!Rec->IsKinematic() && !Circle->IsKinematic())
				printf("AABBToCircle: Collided!\n");
		
			return true;
		}
	}

	if (Circle == nullptr && Rec != nullptr)
		printf("CircleToAABB: Circle is null\n");

	if (Circle != nullptr && Rec == nullptr)
		printf("CircleToAABB: AABB is null\n");

	if (Circle == nullptr && Rec == nullptr)
		printf("CircleToAABB: Both of the objects were null\n");

	return false;
}

bool World::CircleToCircle(Manifold* M)
{
	const auto C1 = dynamic_cast<Circle*>(M->A);
	const auto C2 = dynamic_cast<Circle*>(M->B);

	if (C1 != nullptr && C2 != nullptr)
	{
		// Calculate the normal
		const glm::vec2 Normal = C2->GetLocation() - C1->GetLocation();

		const float DistanceSquared = LengthSquared(Normal);

		float Radius = C1->GetRadius() + C2->GetRadius();
		Radius *= Radius;

		// Check if circles are not contacting each other
		if (DistanceSquared >= Radius)
		{
			M->ContactsCount = 0;
			return false;
		}

		// We are in contact, calculate the distance and resolve collision
		const float Distance = sqrtf(DistanceSquared);

		if (Distance == 0.0f)
		{
			M->Penetration = C1->GetRadius();
			M->Normal = normalize(Normal);
		}
		else
		{
			M->Penetration = Radius - Distance;
			M->Normal = normalize(Normal);
		}
		
		M->ContactsCount = 1;

		ResolveCollision(M);

		if (C1->IsKinematic())
			printf("Circle (Kinematic): Collided!\n");

		if (C2->IsKinematic())
			printf("Circle (Kinematic): Collided!\n");

		if (!C1->IsKinematic() && !C2->IsKinematic())
			printf("Circle: Collided!\n");

		return true;
	}

	if (C1 == nullptr && C2 != nullptr)
		printf("CircleToCircle: C1 is null\n");

	if (C1 != nullptr && C2 == nullptr)
		printf("CircleToCircle: C2 is null\n");
	
	if (C1 == nullptr && C2 == nullptr)
		printf("CircleToCircle: Both of the objects were null\n");

	return false;
}

bool World::AABBToCircle(Manifold* M)
{
	const auto Circle = dynamic_cast<::Circle*>(M->A);
	const auto Rec = dynamic_cast<AABB*>(M->B);

	if (Circle != nullptr && Rec != nullptr)
	{
		M->A = Rec;
		M->B = Circle;

		CircleToAABB(M);

		M->Normal *= -1.0f;

		return true;
	}

	if (Circle == nullptr && Rec != nullptr)
		printf("AABBToCircle: Circle is null\n");

	if (Circle != nullptr && Rec == nullptr)
		printf("AABBToCircle: AABB is null\n");
	
	if (Circle == nullptr && Rec == nullptr)
		printf("AABBToCircle: Both of the objects were null\n");

	return false;
}

float World::Distance(const glm::vec2 A, const glm::vec2 B)
{
	return sqrtf((A.x - B.x) * (A.x - B.x) + (A.y - B.y) * (A.y - B.y));
}

float World::LengthSquared(const glm::vec2 Vector)
{
	return (Vector.x*Vector.x + Vector.y*Vector.y);
}

void World::ResolveCollision(Manifold* M)
{
	const auto A = dynamic_cast<Object*>(M->A);
	const auto B = dynamic_cast<Object*>(M->B);

	if (A == nullptr && B == nullptr)
		return;

	for (unsigned int i = 0; i < M->ContactsCount; i++)
	{
		const glm::vec2 RelativeVelocity = B->GetVelocity() - A->GetVelocity();

		// Velocity along the normal
		const float ContactVelocity = dot(RelativeVelocity, M->Normal);

		// Do not resolve if velocities are separating
		if (ContactVelocity > 0)
			return;

		// Calculate restitution
		const float e = glm::min(A->GetRestitution(), B->GetRestitution())/2.0f;

		const float InverseMassSum = A->GetInverseMass() + B->GetInverseMass();

		// Calculate the impulse scalar
		float j = -(1 + e) * ContactVelocity;
		j /= InverseMassSum;
		j /= M->ContactsCount;

		// Calculate the amount of force to apply depending on the object's mass
		const glm::vec2 ImpulseVector = j * M->Normal;

		glm::vec2 Force = A->GetInverseMass()*-ImpulseVector;
		if (!A->IsKinematic())
			A->ApplyForce(Force);

		Force = B->GetInverseMass()*ImpulseVector;
		if (!B->IsKinematic())
			B->ApplyForce(Force);
		
		PositionalCorrection(M);
	}
}

void World::PositionalCorrection(Manifold* M)
{
	auto A = dynamic_cast<Object*>(M->A);
	auto B = dynamic_cast<Object*>(M->B);

	if (A != nullptr && B != nullptr)
	{
		const float PenetrationDepthAllowance = 0.03f;
		const float PenetrationCorrection = 3.0f;

		const glm::vec2 Correction = glm::max(M->Penetration - PenetrationDepthAllowance, 0.0f)/(A->GetInverseMass() + B->GetInverseMass()) * M->Normal * PenetrationCorrection;

		if (!A->IsKinematic())
			A->ApplyForce(-Correction*A->GetInverseMass());

		if (!B->IsKinematic())
			B->ApplyForce(Correction*B->GetInverseMass());
	}
}
