#include "World.h"
#include "AABB.h"
#include "Circle.h"
#include "Gizmos.h"

#include <glm/ext.hpp>
#include "Plane.h"
#include <cstdio>
#include "Box.h"
#include "Input.h"

World::World() = default;
World::~World() = default;

typedef bool(*CollisionFn)(Manifold*);

static CollisionFn CollisionFunctionArray[] =
{
	World::AABBToAABB, World::AABBToBox, World::CircleToAABB, World::BoxToCircle, World::BoxToPlane, World::CircleToPlane, World::PlaneToPlane,
	World::PlaneToCircle, World::CircleToCircle, World::AABBToPlane, World::AABBToCircle, World::BoxToBox, World::PlaneToAABB, World::BoxToAABB
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

		const glm::vec2 Min = Rec->GetMin();
		const glm::vec2 Max = Rec->GetMax();

		// Find the closest point on the rectangle
		glm::vec2 ClosestPoint;

		ClosestPoint.x = glm::clamp(Circle->GetLocation().x, Min.x, Max.x);
		ClosestPoint.y = glm::clamp(Circle->GetLocation().y, Min.y, Max.y);

		const glm::vec2 Distance = Circle->GetLocation() - ClosestPoint;

		if (LengthSquared(Distance) < Circle->GetRadius() * Circle->GetRadius())
		{
			M->ContactsCount = 1;
			M->Penetration = Circle->GetRadius() * Circle->GetRadius();
			M->Normal = normalize(Distance);

			ResolveCollision(M);

			if (Rec->IsKinematic())
				printf("CircleToAABB (Kinematic): Collided!\n");

			if (Circle->IsKinematic())
				printf("CircleToAABB (Kinematic): Collided!\n");

			if (!Rec->IsKinematic() && !Circle->IsKinematic())
				printf("CircleToAABB: Collided!\n");

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

		float RadiiSum = C1->GetRadius() + C2->GetRadius();
		RadiiSum *= RadiiSum;

		if (DistanceSquared > RadiiSum)
			return false;

		if (DistanceSquared <= RadiiSum)
		{
			M->ContactsCount = 1;
			M->Penetration = RadiiSum - DistanceSquared;
			M->Normal = normalize(Normal);

			ResolveCollision(M);

			if (C1->IsKinematic())
				printf("Circle (Kinematic): Collided!\n");

			if (C2->IsKinematic())
				printf("Circle (Kinematic): Collided!\n");

			if (!C1->IsKinematic() && !C2->IsKinematic())
				printf("Circle: Collided!\n");

			return true;
		}
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
			M->Penetration = CircleToPlane;
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

bool World::CircleToBox(Manifold * M)
{
	auto *Circle = dynamic_cast<::Circle*>(M->A);
	auto *Box = dynamic_cast<::Box*>(M->B);

	if (Circle != nullptr && Box != nullptr)
	{
		M->A = Box;
		M->B = Circle;

		BoxToCircle(M);

		M->Normal *= -1.0f;

		return true;
	}

	// Error checks
	if (Box == nullptr && Circle != nullptr)
		printf("AABBToBox: Box is null\n");

	if (Box != nullptr && Circle == nullptr)
		printf("AABBToBox: Circle is null\n");

	if (Box == nullptr && Circle == nullptr)
		printf("AABBToBox: Both of the objects were null\n");

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
		printf("PlaneToAABB: Rec is null\n");

	if (R != nullptr && P == nullptr)
		printf("PlaneToAABB: Plane is null\n");

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
		glm::vec2 AxisToTest[] = {glm::vec2(1.0f, 0.0f), glm::vec2(0.0f, 1.0f),
								  glm::vec2(0.0f, 0.0f), glm::vec2(0.0f, 0.0f)};

		// Create rotation matrix
		const float t = DEG2RAD(Box->GetRotation());
		float ZRotation[] = {cosf(t), sinf(t),
							 -sinf(t), cosf(t)};

		// Create separating axis number 3
		glm::vec2 Axis = Normalize(glm::vec2(Box->GetExtent().x, 0.0f));

		Multiply(AxisToTest[2].AsArray, Axis.AsArray, 1, 2, ZRotation, 2, 2);

		// Create separating axis number 4
		Axis = Normalize(glm::vec2(0.0f, Box->GetExtent().y));

		Multiply(AxisToTest[3].AsArray, Axis.AsArray, 1, 2, ZRotation, 2, 2);

		// Check every axis for overlap
		for (int i = 0; i < 4; ++i)
		{
			if (!OverlapOnAxis(*Rec, *Box, AxisToTest[i]))
				return false;
		}

		M->ContactsCount++;
		M->Normal = Box->GetLocation() - Rec->GetLocation();

		ResolveCollision(M);

		if (Rec->IsKinematic())
			printf("BoxToAABB (Kinematic): Collided\n");

		if (Box->IsKinematic())
			printf("BoxToAABB (Kinematic): Collided\n");

		if (!Rec->IsKinematic() && !Box->IsKinematic())
			printf("BoxToAABB: Collided\n");

		return true;
	}

	AABBToBox(M);
	return false;
}

bool World::BoxToCircle(Manifold * M)
{
	auto* Box = dynamic_cast<::Box*>(M->A);
	auto* Circle = dynamic_cast<::Circle*>(M->B);

	if (Box != nullptr && Circle != nullptr)
	{
		glm::vec2 Distance = Circle->GetLocation() - Box->GetLocation();

		// Make a rotation matrix
		const float Theta = -DEG2RAD(Box->GetRotation());
		float ZRotation[] = {cosf(Theta), sinf(Theta),
							-sinf(Theta), cosf(Theta)};

		// Rotate the line by the negative rotation matrix above. This transforms the line into local space of box
		Multiply(Distance.AsArray, glm::vec2(Distance.x, Distance.y).AsArray, 1, 2, ZRotation, 2, 2);

		// Create a new circle in the local space of the box
		::Circle LocalCircle(Distance + Box->GetExtent(), Circle->GetVelocity(), Circle->GetRadius(), Circle->GetMass(), Circle->GetColor());

		// Create an AABB to represent the local space of the box
		class AABB LocalAABB(Box->GetLocation(), Box->GetVelocity(), Box->GetExtent().x * 2.0f, Box->GetExtent().y * 2.0f, Box->GetMass(), glm::vec4());

		M->A = &LocalCircle;
		M->A = &LocalAABB;

		return CircleToAABB(M);
	}

	CircleToBox(M);
	return false;
}

bool World::BoxToBox(Manifold * M)
{
	auto* Box1 = dynamic_cast<Box*>(M->A);
	auto* Box2 = dynamic_cast<Box*>(M->B);

	if (Box1 != nullptr && Box2 != nullptr)
	{
		glm::vec2 AxisToTest[] = {glm::vec2(1.0f, 0.0f), glm::vec2(0.0f, 1.0f),
								  glm::vec2(0.0f, 0.0f), glm::vec2(0.0f, 0.0f),
								  glm::vec2(0.0f, 0.0f), glm::vec2(0.0f, 0.0f)};

		// Create rotation matrix
		const float t = DEG2RAD(Box2->GetRotation());
		float ZRotation[] = {cosf(t), sinf(t),
							-sinf(t), cosf(t)};

		// Create separating axis number 3
		glm::vec2 Axis = Normalize(glm::vec2(Box2->GetExtent().x, 0.0f));

		Multiply(AxisToTest[2].AsArray, Axis.AsArray, 1, 2, ZRotation, 2, 2);

		// Create separating axis number 4
		Axis = Normalize(glm::vec2(0.0f, Box2->GetExtent().y));

		Multiply(AxisToTest[3].AsArray, Axis.AsArray, 1, 2, ZRotation, 2, 2);

		// Create separating axis number 5
		Axis = Normalize(glm::vec2(Box2->GetExtent().x, 0.0f));

		Multiply(AxisToTest[4].AsArray, Axis.AsArray, 1, 2, ZRotation, 2, 2);

		// Create separating axis number 6
		Axis = Normalize(glm::vec2(0.0f, Box2->GetExtent().y));

		Multiply(AxisToTest[5].AsArray, Axis.AsArray, 1, 2, ZRotation, 2, 2);

		// Check every axis for overlap
		for (int i = 0; i < 6; ++i)
		{
			if (!OverlapOnAxis(*Box1, *Box2, AxisToTest[i]))
				return false;
		}

		M->ContactsCount++;
		M->Penetration = 5.0f;
		M->Normal = Normalize(Box2->GetLocation() - Box1->GetLocation());
		
		ResolveCollision(M);

		if (Box1->IsKinematic())
			printf("BoxToBox (Kinematic): Collided\n");

		if (Box2->IsKinematic())
			printf("BoxToBox (Kinematic): Collided\n");

		if (!Box1->IsKinematic() && !Box2->IsKinematic())
			printf("BoxToBox: Collided\n");

		return true;
	}

	// Error checks
	if (Box1 == nullptr && Box2 != nullptr)
		printf("BoxToBox: Box1 is null\n");

	if (Box1 != nullptr && Box2 == nullptr)
		printf("BoxToBox: Box2 is null\n");
	
	if (Box1 == nullptr && Box2 == nullptr)
		printf("BoxToBox: Both of the objects were null\n");

	return false;
}

bool World::BoxToPlane(Manifold * M)
{
	const auto Box = dynamic_cast<::Box*>(M->A);
	const auto Plane = dynamic_cast<::Plane*>(M->B);

	if (Box != nullptr && Plane != nullptr)
	{
		
	}
	else
		CircleToCircle(M);

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

	BoxToBox(M);
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

Interval World::GetInterval(const class AABB& Rec, const glm::vec2& Axis)
{
	// Get the min and max of this AABB
	const glm::vec2 Min = Rec.GetMin();
	const glm::vec2 Max = Rec.GetMax();

	// Get all vertices of the AABB
	const glm::vec2 Vertices[] = {glm::vec2(Min.x, Min.y), glm::vec2(Min.x, Max.y),
								  glm::vec2(Max.x, Max.y), glm::vec2(Max.x, Min.y)};

	// Set interval to first projected vertex
	Interval Result{0.0f, 0.0f};
	Result.Min = dot(Axis, Vertices[0]);
	Result.Max = Result.Min;

	// Project each vertex onto the axis
	for (int i = 0; i < 4; i++)
	{
		const float Projection = dot(Axis, Vertices[i]);
		if (Projection < Result.Min)
			Result.Min = Projection;

		if (Projection > Result.Max)
			Result.Max = Projection;
	}
	
	return Result;
}

Interval World::GetInterval(const Box& Box, const glm::vec2& Axis)
{
	// Make a non oriented version of this box
	class AABB Rec(Box.GetLocation(), Box.GetVelocity(), Box.GetExtent().x * 2.0f, Box.GetExtent().y * 2.0f, Box.GetMass(), Box.GetColor());

	// Find the 4 vertices of the AABB
	const glm::vec2 Min = Rec.GetMin();
	const glm::vec2 Max = Rec.GetMax();
	glm::vec2 Vertices[] = {Min, Max, glm::vec2(Min.x, Max.y), glm::vec2(Max.x, Min.y)}; // Bottom left - Top right - Top left - Bottom right

	// Create a rotation matrix from the orientation on the AABB
	const float t = DEG2RAD(Box.GetRotation());
	float ZRotation[] = {cosf(t), sinf(t),
						-sinf(t), cosf(t)};

	// Rotate every vertex of the AABB by the matrix above 
	for (int i = 0; i < 4; i++)
	{
		glm::vec2 R = Vertices[i] - Box.GetLocation();

		Multiply(R.AsArray, glm::vec2(R.x, R.y).AsArray, 1, 2, ZRotation, 2, 2);

		Vertices[i] = R + Box.GetLocation();
	}

	// Store the min and max points of every projected vertex
	Interval Result{0.0f, 0.0f};
	Result.Min = Result.Max = dot(Axis, Vertices[0]);
	for (int i = 0; i < 4; i++)
	{
		float Projection = dot(Axis, Vertices[i]);
		Result.Min = (Projection < Result.Min) ? Projection : Result.Min;
		Result.Max = (Projection > Result.Max) ? Projection : Result.Max;
	}

	return Result;
}

bool World::OverlapOnAxis(const class AABB& Rec, const Box& Box, const glm::vec2& Axis)
{
	const Interval A = GetInterval(Rec, Axis);
	const Interval B = GetInterval(Box, Axis);
	return (B.Min <= A.Max) && (A.Min <= B.Max);
}

bool World::OverlapOnAxis(const class Box& Box1, const Box& Box2, const glm::vec2& Axis)
{
	const Interval A = GetInterval(Box1, Axis);
	const Interval B = GetInterval(Box2, Axis);
	return (B.Min <= A.Max) && (A.Min <= B.Max);
}

bool World::Multiply(float * Out, const float * MatA, int ARows, int ACols, const float * MatB, int BRows, int BCols)
{
	if (ACols != BRows)
		return false;

	for (int i = 0; i < ARows; ++i)
	{
		for (int j = 0; j < BCols; ++j)
		{
			Out[BCols * i + j] = 0.0f;

			for (int k = 0; k < BRows; ++k)
			{
				const int A = ACols * i + k;
				const int B = BCols * k + j;
				Out[BCols * i + j] += MatA[A] * MatB[B];
			}
		}
	}

	return true;
}

glm::vec2 World::Normalize(const glm::vec2 & Vector)
{
	return Vector * (1.0f/length(Vector));
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

		// Friction
		const glm::vec2 t = RelativeVelocity - (M->Normal * ContactVelocity);
		if (LengthSquared(t) > 0.0f)
			return;

		Normalize(t);

		// Calculate magnitude of friction
		j = -dot(RelativeVelocity, t);
		float jt = j / InverseMassSum;
		j /= M->ContactsCount;

		if (fabsf(jt) > 0.0f)
			return;

		// Coulombs Law
		const float Friction = sqrtf(M->A->GetFriction() * M->B->GetFriction());
		if (jt > j * Friction)
			jt = j * Friction;
		else if (jt < -j * Friction)
			jt = -j * Friction;

		const glm::vec2 TangentImpulse = t * jt;

		Force = A->GetInverseMass()*-TangentImpulse;
		if (!A->IsKinematic())
			A->ApplyForce(Force);

		Force = B->GetInverseMass()*TangentImpulse;
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
