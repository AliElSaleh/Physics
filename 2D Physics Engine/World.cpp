#include "World.h"
#include "AABB.h"
#include "Circle.h"

#include <stdio.h>

World::World() = default;
World::~World() = default;

typedef bool(*CollisionFn)(Object*, Object*);

static CollisionFn CollisionFunctionArray[] =
{
	World::AABBToAABB, World::CircleToCircle
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
			const int Shape1 = Object1->GetShape();
			const int Shape2 = Object2->GetShape();

			const unsigned short FunctionIndex = Shape1 * 1;
			const CollisionFn CollisionFunctionPtr = CollisionFunctionArray[FunctionIndex];

			if (CollisionFunctionPtr != nullptr)
				CollisionFunctionPtr(Object1, Object2);

			//AABBToAABB(dynamic_cast<AABB*>(Object1), dynamic_cast<AABB*>(Object2));
		}
	}
}

bool World::AABBToAABB(Object* A, Object* B)
{
	const auto Rec1 = dynamic_cast<AABB*>(A);
	const auto Rec2 = dynamic_cast<AABB*>(B);

	if (Rec1 != nullptr && Rec2 != nullptr)
	{
		// Exit with no intersection if found sepRec1rRec1ted Rec1long Rec1n Rec1xis
		if (Rec1->GetMax().x < Rec2->GetMin().x || Rec1->GetMin().x > Rec2->GetMax().x)
		{
			printf("Not Colliding!\n");
			return false;
		}
		if (Rec1->GetMax().y < Rec2->GetMin().y || Rec1->GetMin().y > Rec2->GetMax().y)
		{
			printf("Not Colliding!\n");
			return false;
		}

		// No separating axis found, at least one overlapping axis is intersecting
		return true ? printf("Collided!\n") : false;
	}

	printf("AABB: One of the objects were null\n");

	return false;
}

bool World::CircleToCircle(Object* A, Object* B)
{
	const auto C1 = dynamic_cast<Circle*>(A);
	const auto C2 = dynamic_cast<Circle*>(B);

	if (C1 != nullptr && C2 != nullptr)
	{
		const float Radius = C1->GetRadius() + C2->GetRadius();

		return Radius > Distance(C1->GetLocation(), C2->GetLocation()) ? printf("Collided!\n") : printf("Not colliding\n");
	}

	printf("Circle: One of the objects were null\n");

	return false;
}

float World::Distance(const glm::vec2 A, const glm::vec2 B)
{
	return sqrtf((A.x - B.x) * (A.x - B.x) + (A.y - B.y) * (A.y - B.y));
}
