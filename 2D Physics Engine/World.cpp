#include "World.h"
#include "AABB.h"
#include "Circle.h"
#include "Gizmos.h"

#include <glm/ext.hpp>
#include "Plane.h"
#include <stdio.h>
#include "OBB.h"
#include "Input.h"

World::World() = default;
World::~World() = default;

typedef bool(*CollisionFn)(Manifold*);

static CollisionFn CollisionFunctionArray[] =
{
	World::AABBToAABB, World::AABBToOBB, World::CircleToAABB, World::OBBToCircle, World::OBBToPlane, World::CircleToPlane, World::PlaneToPlane,
	World::PlaneToCircle, World::CircleToCircle, World::AABBToPlane, World::AABBToCircle, World::OBBToOBB, World::PlaneToAABB, World::OBBToAABB
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
		{
			Actor->FixedUpdate(Gravity, TimeStep);

			if (Actor->IsOutsideWindow() && !Actor->IsKinematic() && Actor->GetShape() != PLANE)
				RemoveActor(Actor);
		}
	
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

			delete M;
		}
	}
}

bool World::AABBToAABB(Manifold* M)
{
	const auto Rec1 = dynamic_cast<class AABB*>(M->A);
	const auto Rec2 = dynamic_cast<class AABB*>(M->B);

	if (Rec1 != nullptr && Rec2 != nullptr)
	{
		// Are both axes overlapped
		if (Rec1->GetLocation().x + Rec1->GetExtent().x > Rec2->GetLocation().x - Rec2->GetExtent().x &&
			Rec1->GetLocation().x - Rec1->GetExtent().x < Rec2->GetLocation().x + Rec2->GetExtent().x &&
			Rec1->GetLocation().y + Rec1->GetExtent().y > Rec2->GetLocation().y - Rec2->GetExtent().y &&
			Rec1->GetLocation().y - Rec1->GetExtent().y < Rec2->GetLocation().y + Rec2->GetExtent().y)
		{
			glm::vec2 CollisionNormal;

			const glm::vec2 Size = {fabsf(Rec1->GetWidth()), fabsf(Rec1->GetHeight()) };

			const glm::vec2 A = { fabsf(Size.x), fabsf(Size.y) };

			const glm::vec2 S = { Rec1->GetLocation().x < Rec2->GetLocation().x ? -1.0f : 1.0f,
								  Rec1->GetLocation().y < Rec2->GetLocation().y ? -1.0f : 1.0f};

			if (A.x < A.y)
				CollisionNormal = glm::vec2(S.x, S.y);
			else
				CollisionNormal = glm::vec2(-S.x, -S.y);

			M->ContactsCount = 1;
			M->Penetration = LengthSquared(CollisionNormal);
			M->Normal = CollisionNormal;
			
			ResolveCollision(M);
			return true;
		}
	}

	PrintError(Rec1, Rec2, AABB, AABB);
	
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
			M->Penetration = Circle->GetRadius();
			M->Normal = normalize(Distance);

			ResolveCollision(M);
			return true;
		}
	}
	else
	{
		AABBToCircle(M);
		return true;
	}

	PrintError(Circle, Rec, CIRCLE, AABB);

	return false;
}

bool World::AABBToPlane(Manifold* M)
{
	const auto Plane = dynamic_cast<::Plane*>(M->A);
	const auto Rec = dynamic_cast<class AABB*>(M->B);

	if (Plane != nullptr && Rec != nullptr)
	{
		M->ContactsCount = 0;

		const glm::vec2 CollisionNormal = Plane->GetNormal();

		// Check if the start or the end points are inside the AABB
		if (PointOnAABB(Plane->GetStart(), *Rec) || PointOnAABB(Plane->GetEnd(), *Rec))
			return true;

		// Do raycast against the AABB
		glm::vec2 Normal = Normalize(Plane->GetEnd() - Plane->GetStart());
		Normal.x = Normal.x != 0 ? 1.0f/Normal.x : 0;
		Normal.y = Normal.y != 0 ? 1.0f/Normal.y : 0;

		const glm::vec2 Min = (Rec->GetMin() - Plane->GetStart()) * Normal;
		const glm::vec2 Max = (Rec->GetMax() - Plane->GetStart()) * Normal;

		const float tmin = fmaxf(fminf(Min.x, Max.x), fminf(Min.y, Max.y));
		const float tmax = fminf(fmaxf(Min.x, Max.x), fmaxf(Min.y, Max.y));

		// if tmax < 0, the ray is intersecting the AABB, but the AABB is behind us.
		// OR if tmin > tmax the ray doesn't intersect the AABB
		if (tmax < 0 || tmin > tmax)
			return false;

		// Ray intersects the AABB
		const float t = tmin < 0.0f ? tmax : tmin;

		const float PenetrationDepth = dot(Rec->GetLocation(), CollisionNormal) - Plane->GetDistance();

		// If ray hits and the length of the ray is less than the length of the line, we have a collision
		if (t > 0.0f && t * t < MagnitudeSquared(Plane->GetEnd() - Plane->GetStart()))
		{
			M->ContactsCount = 1;
			M->Penetration = 5.0f;
			M->Normal = CollisionNormal;

			Plane->ResolveCollision(M);
			return true;
		}
	}
	else
	{
		PlaneToAABB(M);
		return true;
	}

	PrintError(Rec, Plane, AABB, PLANE);

	return false;
}

bool World::AABBToOBB(Manifold* M)
{
	auto* Rec = dynamic_cast<class AABB*>(M->A);
	auto* Box = dynamic_cast<class OBB*>(M->B);

	if (Box != nullptr && Rec != nullptr)
	{
		M->A = Box;
		M->B = Rec;

		OBBToAABB(M);

		M->Normal *= -1;

		return true;
	}
	
	OBBToAABB(M);
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

			return true;
		}
	}

	PrintError(C1, C2, CIRCLE, CIRCLE);

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

		const glm::vec2 AB = P->GetEnd() - P->GetStart();
		const float t = dot(C->GetLocation() - P->GetStart(), AB) / dot(AB, AB);

		if (t < 0.0f || t > 1.0f)
			return false;

		const glm::vec2 ClosestPoint = P->GetStart() + AB * t;

		const glm::vec2 CircleToClosest[] = {C->GetLocation(), ClosestPoint};

		glm::vec2 CollisionNormal = P->GetNormal();
		const float CircleToPlane = dot(C->GetLocation(), CollisionNormal) - P->GetDistance();
		
		// If we are behind the plane, then flip the normal
		if (CircleToPlane < 0)
			CollisionNormal *= -1;

		const float Intersection = MagnitudeSquared(CircleToClosest[1] - CircleToClosest[0]);
		if (Intersection < C->GetRadius() * C->GetRadius() * 1.1f) // 1.1 - offset
		{
			M->ContactsCount = 1;
			M->Penetration = Intersection * Intersection;
			M->Normal = CollisionNormal;
		
			P->ResolveCollision(M);

			return true;
		}
	}
	else
	{
		PlaneToCircle(M);
		return true;
	}

	PrintError(C, P, CIRCLE, PLANE);

	return false;
}

bool World::CircleToOBB(Manifold * M)
{
	auto *Circle = dynamic_cast<::Circle*>(M->A);
	auto *Box = dynamic_cast<class OBB*>(M->B);

	if (Circle != nullptr && Box != nullptr)
	{
		M->A = Box;
		M->B = Circle;

		OBBToCircle(M);

		M->Normal *= -1.0f;

		return true;
	}
	
	PlaneToAABB(M);
	return true;
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

	PrintError(P, C, PLANE, CIRCLE);

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

	AABBToPlane(M);
	return true;
}

bool World::PlaneToPlane(Manifold* M)
{
	return false;
}

bool World::OBBToAABB(Manifold* M)
{
	auto* Box = dynamic_cast<class OBB*>(M->A);
	auto* Rec = dynamic_cast<class AABB*>(M->B);

	if (Box != nullptr && Rec != nullptr)
	{
		glm::vec2 AxisToTest[] = {glm::vec2(1.0f, 0.0f), glm::vec2(0.0f, 1.0f),
								  glm::vec2(0.0f, 0.0f), glm::vec2(0.0f, 0.0f)};

		// Create rotation matrix
		const float r = DEG2RAD(Box->GetRotation());
		float ZRotation[] = {cosf(r), sinf(r),
							-sinf(r), cosf(r)};

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
		M->Penetration = 5.0f;
		M->Normal = Axis;

		ResolveCollision(M);
		return true;
	}

	AABBToOBB(M);
	return false;
}

bool World::OBBToCircle(Manifold * M)
{
	auto* Box = dynamic_cast<class OBB*>(M->A);
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

	CircleToOBB(M);
	return false;
}

bool World::OBBToOBB(Manifold * M)
{
	auto* Box1 = dynamic_cast<class OBB*>(M->A);
	auto* Box2 = dynamic_cast<class OBB*>(M->B);

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
		M->Penetration = 2.0f;
		M->Normal = Normalize(Box2->GetLocation() - Box1->GetLocation());
		
		ResolveCollision(M);
		return true;
	}

	PrintError(Box1, Box2, OBB, OBB);

	return false;
}

bool World::OBBToPlane(Manifold* M)
{
	const auto Plane = dynamic_cast<::Plane*>(M->A);
	const auto Box = dynamic_cast<class OBB*>(M->B);

	if (Box != nullptr && Plane != nullptr)
	{
		// Create a rotation matrix 
		float Theta = -DEG2RAD(Box->GetRotation());
		float ZRotation[] ={cosf(Theta), sinf(Theta),
						   -sinf(Theta), cosf(Theta)};

		// Create a new plane in the local space of the OBB
		::Plane LocalPlane;
		LocalPlane.SetNormal(Plane->GetNormal());

		glm::vec2 RotationVector = Plane->GetStart() - Box->GetLocation();
		Multiply(RotationVector.AsArray, glm::vec2(RotationVector.x, RotationVector.y).AsArray, 1, 2, ZRotation, 2, 2);
		LocalPlane.SetStart(RotationVector + 0.1f);

		RotationVector = Plane->GetEnd() - Box->GetLocation();
		Multiply(RotationVector.AsArray, glm::vec2(RotationVector.x, RotationVector.y).AsArray, 1, 2, ZRotation, 2, 2);
		LocalPlane.SetEnd(RotationVector + 0.1f);

		class AABB LocalAABB(glm::vec2(), Box->GetVelocity(), Box->GetExtent().x * 2, Box->GetExtent().y * 2, Box->GetMass(), Box->GetColor());

		M->A = &LocalPlane;
		M->B = &LocalAABB;

		// The OBB in local space is an AABB, use our existing function to test for collision
		if (AABBToPlane(M))
		{
			M->A = Plane;
			M->B = Box;

			M->ContactsCount = 1;
			M->Penetration = 5.0f;
			M->Normal = Plane->GetNormal();

			Plane->ResolveCollision(M);
			return true;
		}

		return false;
	}

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

	OBBToOBB(M);
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

Interval World::GetInterval(const class OBB& Box, const glm::vec2& Axis)
{
	// Make a non oriented version of this box
	class AABB Rec(Box.GetLocation(), Box.GetVelocity(), Box.GetExtent().x * 2.0f, Box.GetExtent().y * 2.0f, Box.GetMass(), Box.GetColor());

	// Find the 4 vertices of the AABB
	const glm::vec2 Min = Rec.GetMin();
	const glm::vec2 Max = Rec.GetMax();
	glm::vec2 Vertices[] = {Min, Max, glm::vec2(Min.x, Max.y), glm::vec2(Max.x, Min.y)}; // Bottom left - Top right - Top left - Bottom right

	// Create a rotation matrix from the orientation on the AABB
	const float r = DEG2RAD(Box.GetRotation());
	float ZRotation[] = {cosf(r), sinf(r),
						-sinf(r), cosf(r)};

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
		float Projected = dot(Axis, Vertices[i]);
		Result.Min = (Projected < Result.Min) ? Projected : Result.Min;
		Result.Max = (Projected > Result.Max) ? Projected : Result.Max;
	}

	return Result;
}

bool World::OverlapOnAxis(const class AABB& Rec, const class OBB& Box, const glm::vec2& Axis)
{
	const Interval A = GetInterval(Rec, Axis);
	const Interval B = GetInterval(Box, Axis);
	return (B.Min <= A.Max) && (A.Min <= B.Max);
}

bool World::OverlapOnAxis(const class OBB& Box1, const class OBB& Box2, const glm::vec2& Axis)
{
	const Interval A = GetInterval(Box1, Axis);
	const Interval B = GetInterval(Box2, Axis);
	return (B.Min <= A.Max) && (A.Min <= B.Max);
}

bool World::PointOnAABB(const glm::vec2& Point, const class AABB& Rec)
{
	const glm::vec2 Min = Rec.GetMin();
	const glm::vec2 Max = Rec.GetMax();

	return Min.x <= Point.x && Min.y <= Point.y && Point.x <= Max.x && Point.y <= Max.y;
}

void World::PrintCollided(Manifold* M, const Geometry Type1, const Geometry Type2)
{
	const char* CollisionTest{};

	switch (Type1)
	{
	case AABB:
		switch (Type2)
		{
		case AABB:
			CollisionTest = "AABBToAABB";
			break;

		case OBB:
			CollisionTest = "AABBToOBB";
			break;

		case CIRCLE:
			CollisionTest = "AABBToCircle";
			break;

		case PLANE:
			CollisionTest = "AABBToPlane";
			break;

		default:
			break;
		}
		break;

	case OBB:
		switch (Type2)
		{
		case AABB:
			CollisionTest = "OBBToAABB";
			break;

		case OBB:
			CollisionTest = "OBBToOBB";
			break;

		case CIRCLE:
			CollisionTest = "OBBToCircle";
			break;

		case PLANE:
			CollisionTest = "OBBToPlane";
			break;

		default:
			break;
		}
		break;

	case CIRCLE:
		switch (Type2)
		{
		case AABB:
			CollisionTest = "CircleToAABB";
			break;

		case OBB:
			CollisionTest = "CircleToOBB";
			break;

		case CIRCLE:
			CollisionTest = "CircleToCircle";
			break;

		case PLANE:
			CollisionTest = "CircleToPlane";
			break;

		default:
			break;
		}
		break;

	case PLANE:
		switch (Type2)
		{
		case AABB:
			CollisionTest = "PlaneToAABB";
			break;

		case OBB:
			CollisionTest = "PlaneToOBB";
			break;

		case CIRCLE:
			CollisionTest = "PlaneToCircle";
			break;

		case PLANE:
			CollisionTest = "PlaneToPlane";
			break;

		default:
			break;
		}
		break;

	default:
		break;
	}

	if (M->A->IsKinematic())
		printf("%s (Kinematic): Collided!\n", CollisionTest);

	if (M->B->IsKinematic())
		printf("%s (Kinematic): Collided!\n", CollisionTest);

	if (!M->A->IsKinematic() && !M->B->IsKinematic())
		printf("%s: Collided!\n", CollisionTest);

}

void World::PrintError(Object* A, Object* B, const Geometry Type1, const Geometry Type2)
{
	const char* CollisionTest{};
	const char* Shape1{}, *Shape2{};

	switch (Type1)
	{
	case AABB:
		switch (Type2)
		{
		case AABB:
			CollisionTest = "AABBToAABB";
			Shape1 = "AABB";
			Shape2 = "AABB";
			break;

		case OBB:
			CollisionTest = "AABBToOBB";
			Shape1 = "AABB";
			Shape2 = "OBB";
			break;

		case CIRCLE:
			CollisionTest = "AABBToCircle";
			Shape1 = "AABB";
			Shape2 = "Circle";
			break;

		case PLANE:
			CollisionTest = "AABBToPlane";
			Shape1 = "AABB";
			Shape2 = "Plane";
			break;

		default:
			break;
		}
		break;

	case OBB:
		switch (Type2)
		{
		case AABB:
			CollisionTest = "OBBToAABB";
			Shape1 = "OBB";
			Shape2 = "AABB";
			break;

		case OBB:
			CollisionTest = "OBBToOBB";
			Shape1 = "OBB";
			Shape2 = "OBB";
			break;

		case CIRCLE:
			CollisionTest = "OBBToCircle";
			Shape1 = "OBB";
			Shape2 = "Circle";
			break;

		case PLANE:
			CollisionTest = "OBBToPlane";
			Shape1 = "OBB";
			Shape2 = "Plane";
			break;

		default:
			break;
		}
		break;

	case CIRCLE:
		switch (Type2)
		{
		case AABB:
			CollisionTest = "CircleToAABB";
			Shape1 = "Circle";
			Shape2 = "AABB";
			break;

		case OBB:
			CollisionTest = "CircleToOBB";
			Shape1 = "Circle";
			Shape2 = "OBB";
			break;

		case CIRCLE:
			CollisionTest = "CircleToCircle";
			Shape1 = "Circle";
			Shape2 = "Circle";
			break;

		case PLANE:
			CollisionTest = "CircleToPlane";
			Shape1 = "Circle";
			Shape2 = "Plane";
			break;

		default:
			break;
		}
		break;

	case PLANE:
		switch (Type2)
		{
		case AABB:
			CollisionTest = "PlaneToAABB";
			Shape1 = "Plane";
			Shape2 = "AABB";
			break;

		case OBB:
			CollisionTest = "PlaneToOBB";
			Shape1 = "Plane";
			Shape2 = "OBB";
			break;

		case CIRCLE:
			CollisionTest = "PlaneToCircle";
			Shape1 = "Plane";
			Shape2 = "Circle";
			break;

		case PLANE:
			CollisionTest = "PlaneToPlane";
			Shape1 = "Plane";
			Shape2 = "Plane";
			break;

		default:
			break;
		}
		break;

	default:
		break;
	}

	if (A == nullptr && B != nullptr)
		printf("%s: %s is null\n", CollisionTest, Shape1);

	if (A != nullptr && B == nullptr)
		printf("%s: %s is null\n", CollisionTest, Shape2);

	if (A == nullptr && B == nullptr)
		printf("%s: Both of the objects were null\n", CollisionTest);
}

bool World::Multiply(float * Out, const float * MatA, const int ARows, const int ACols, const float * MatB, const int BRows, const int BCols)
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

float World::MagnitudeSquared(const glm::vec2 & Vector)
{
	return dot(Vector, Vector);
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
		const float PenetrationDepthAllowance = 0.1f;
		const float PenetrationCorrection = 3.0f;

		const glm::vec2 Correction = glm::max(M->Penetration - PenetrationDepthAllowance, 0.0f)/(A->GetInverseMass() + B->GetInverseMass()) * M->Normal * PenetrationCorrection;

		if (!A->IsKinematic())
			A->ApplyForce(-Correction*A->GetInverseMass());

		if (!B->IsKinematic())
			B->ApplyForce(Correction*B->GetInverseMass());
	}
}
