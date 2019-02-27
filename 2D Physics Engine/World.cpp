#include "World.h"
#include "AABB.h"
#include "Circle.h"
#include "Gizmos.h"

#include <glm/ext.hpp>
#include "Plane.h"
#include <stdio.h>
#include "Box.h"
#include "Input.h"

World::World() = default;
World::~World() = default;

typedef bool(*CollisionFn)(Manifold*);

static CollisionFn CollisionFunctionArray[] =
{
	World::AABBToAABB, World::AABBToBox, World::CircleToAABB, World::AABBToPlane, World::CircleToCircle, World::CircleToPlane,
	World::PlaneToPlane, World::AABBToCircle, World::PlaneToAABB, World::PlaneToCircle, World::BoxToAABB
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
	const auto Rec1 = dynamic_cast<class AABB*>(M->A);
	const auto Rec2 = dynamic_cast<class AABB*>(M->B);

	if (Rec1 != nullptr && Rec2 != nullptr)
	{
		if (Rec1->GetLocation().x + Rec1->GetExtent().x > Rec2->GetLocation().x - Rec2->GetExtent().x &&
			Rec1->GetLocation().x - Rec1->GetExtent().x < Rec2->GetLocation().x + Rec2->GetExtent().x &&
			Rec1->GetLocation().y + Rec1->GetExtent().y > Rec2->GetLocation().y - Rec2->GetExtent().y &&
			Rec1->GetLocation().y - Rec1->GetExtent().y < Rec2->GetLocation().y + Rec2->GetExtent().y)
		{
			M->ContactsCount = 0;

			glm::vec2 CollisionNormal;

			const glm::vec2 Size = {fabsf(Rec1->GetWidth()), fabsf(Rec1->GetHeight()) };

			const glm::vec2 A = { fabsf(Size.x), fabsf(Size.y) };

			const glm::vec2 S = { Rec1->GetLocation().x < Rec2->GetLocation().x ? 1.0f : -1.0f,
							Rec1->GetLocation().y < Rec2->GetLocation().y ? 1.0f : -1.0f};

			if (A.x < A.y)
				CollisionNormal = glm::vec2(S.x, 0.0f);
			else
				CollisionNormal = glm::vec2(0.f, S.y);

			M->ContactsCount++;

			M->Normal = CollisionNormal;

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
	const auto Rec = dynamic_cast<class AABB*>(M->A);
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
	else
	{
		AABBToCircle(M);
		return true;
	}

	if (Circle == nullptr && Rec != nullptr)
		printf("CircleToAABB: Circle is null\n");

	if (Circle != nullptr && Rec == nullptr)
		printf("CircleToAABB: AABB is null\n");

	if (Circle == nullptr && Rec == nullptr)
		printf("CircleToAABB: Both of the objects were null\n");

	return false;
}

bool World::AABBToPlane(Manifold* M)
{
	const auto Plane = dynamic_cast<::Plane*>(M->A);
	const auto Rec = dynamic_cast<class AABB*>(M->B);

	if (Plane != nullptr && Rec != nullptr)
	{
		M->ContactsCount = 0;

		glm::vec2 CollisionNormal = Plane->GetNormal();

		float AABBToPlane = dot(Rec->GetLocation(), CollisionNormal) - Plane->GetDistance();

		// If we are behind the plane, then flip the normal
		if (AABBToPlane < 0)
		{
			CollisionNormal *= -1;
			AABBToPlane *= -1;
		}

		const float Intersection = Rec->GetExtent().x - AABBToPlane;
		if (Intersection > 0)
		{
			M->ContactsCount = 1;
			M->Penetration = 50.0f;//Rec->GetMass();
			M->Normal = CollisionNormal;

			Plane->ResolveCollision(M);

			printf("AABBToPlane: Collided!\n");

			return true;
		}
	}
	else
	{
		PlaneToAABB(M);
		return true;
	}

	if (Rec == nullptr && Plane != nullptr)
		printf("AABBToPlane: Rec is null\n");

	if (Rec != nullptr && Plane == nullptr)
		printf("AABBToPlane: Plane is null\n");

	if (Rec == nullptr && Plane == nullptr)
		printf("AABBToPlane: Both of the objects were null\n");

	return false;
}

bool World::AABBToBox(Manifold* M)
{
	auto* Rec = dynamic_cast<class AABB*>(M->A);
	auto* Box = dynamic_cast<::Box*>(M->B);

	if (Box != nullptr && Rec != nullptr)
	{
		M->A = Box;
		M->B = Rec;

		BoxToAABB(M);

		M->Normal *= -1.0f;

		return true;
	}

	// Error checks
	if (Box == nullptr && Rec != nullptr)
		printf("AABBToBox: Box is null\n");

	if (Box != nullptr && Rec == nullptr)
		printf("AABBToBox: Rec is null\n");

	if (Box == nullptr && Rec == nullptr)
		printf("AABBToBox: Both of the objects were null\n");

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

bool World::CircleToPlane(Manifold* M)
{
	auto *P = dynamic_cast<Plane*>(M->A);
	auto *C = dynamic_cast<Circle*>(M->B);

	// If we are successful, then test for collision
	if (C != nullptr && P != nullptr)
	{
		M->ContactsCount = 0;

		glm::vec2 CollisionNormal = P->GetNormal();
		float CircleToPlane = dot(C->GetLocation(), CollisionNormal) - P->GetDistance();
		
		// If we are behind the plane, then flip the normal
		if (CircleToPlane < 0)
		{
			CollisionNormal *= -1;
			CircleToPlane *= -1;
		}
		
		const float Intersection = C->GetRadius() - CircleToPlane;
		if (Intersection > 0)
		{
			M->ContactsCount = 1;
			M->Penetration = C->GetMass();
			M->Normal = CollisionNormal;
		
			P->ResolveCollision(M);
		
			printf("CircleToPlane: Collided!\n");
		
			return true;
		}
	}
	else
	{
		PlaneToCircle(M);
		return true;
	}

	// Error checks
	if (C == nullptr && P != nullptr)
		printf("CircleToPlane: Circle is null\n");

	if (C != nullptr && P == nullptr)
		printf("CircleToPlane: Plane is null\n");

	if (C == nullptr && P == nullptr)
		printf("CircleToPlane: Both of the objects were null\n");

	return false;
}

bool World::PlaneToCircle(Manifold* M)
{
	auto *C = dynamic_cast<Circle*>(M->A);
	auto *P = dynamic_cast<Plane*>(M->B);

	if (C != nullptr && P != nullptr)
	{
		M->A = P;
		M->B = C;

		CircleToPlane(M);

		return true;
	}

	// Error checks
	if (C == nullptr && P != nullptr)
		printf("CircleToPlane: Circle is null\n");

	if (C != nullptr && P == nullptr)
		printf("CircleToPlane: Plane is null\n");

	if (C == nullptr && P == nullptr)
		printf("CircleToPlane: Both of the objects were null\n");

	return false;
}

bool World::PlaneToAABB(Manifold* M)
{
	auto *R = dynamic_cast<class AABB*>(M->A);
	auto *P = dynamic_cast<Plane*>(M->B);

	if (R != nullptr && P != nullptr)
	{
		M->A = P;
		M->B = R;

		AABBToPlane(M);

		M->Normal *= -1.0f;

		return true;
	}

	// Error checks
	if (R == nullptr && P != nullptr)
		printf("PlaneToAABB: R is null\n");

	if (R != nullptr && P == nullptr)
		printf("PlaneToAABB: P is null\n");

	if (R == nullptr && P == nullptr)
		printf("PlaneToAABB: Both of the objects were null\n");

	return false;
}

bool World::PlaneToPlane(Manifold* M)
{
	return false;
}

bool World::BoxToAABB(Manifold* M)
{
	auto* Box = dynamic_cast<::Box*>(M->A);
	auto* Rec = dynamic_cast<class AABB*>(M->B);

	if (Box != nullptr && Rec != nullptr)
	{
		glm::vec2 RecLocation = Rec->GetLocation() - Box->GetLocation();

		glm::vec2 Normal = glm::vec2(0.0f, 0.0f);
		glm::vec2 Contact = glm::vec2(0.0f, 0.0f);

		int NumOfContacts = 0;


		//if ()
		//{
		//	Normal = -Normal;
		//}

		if (NumOfContacts > 0)
		{
			//const glm::vec2 ContactForce = 0.5f * (ContactForce1 - ContactForce2);

			ResolveCollision(M);

			printf("BoxToAABB: Collided!\n");

			return true;
		}
	}
	else
	{
		AABBToBox(M);
		return true;
	}

	// Error checks
	if (Box == nullptr && Rec != nullptr)
		printf("BoxToAABB: Box is null\n");

	if (Box != nullptr && Rec == nullptr)
		printf("BoxToAABB: Rec is null\n");

	if (Box == nullptr && Rec == nullptr)
		printf("BoxToAABB: Both of the objects were null\n");

	return false;
}

bool World::AABBToCircle(Manifold* M)
{
	const auto Circle = dynamic_cast<::Circle*>(M->A);
	const auto Rec = dynamic_cast<class AABB*>(M->B);

	if (Circle != nullptr && Rec != nullptr)
	{
		M->A = Rec;
		M->B = Circle;

		CircleToAABB(M);

		M->Normal *= -1.0f;

		return true;
	}

	// Error checks
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
