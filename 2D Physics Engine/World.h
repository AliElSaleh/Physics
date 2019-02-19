#pragma once
#include "Object.h"

#include <vector>
#include "Manifold.h"
class AABB;
class Circle;

class World
{
public:
	World();
	~World();

	void AddActor(Object* Actor);
	void RemoveActor(Object* Actor);

	void Update(float DeltaTime);
	void UpdateGizmos();

	void CheckForCollisions();
	static void ResolveCollision(Manifold* M);
	static void PositionalCorrection(Manifold* M);

//	static bool AABBToAABB(Object* A, Object* B);
	static bool AABBToAABB(Manifold* M);
//	static bool CircleToCircle(Object* A, Object* B);
	static bool CircleToAABB(Manifold* M);
	static bool CircleToCircle(Manifold* M);
//	static bool AABBToCircle(Object* A, Object* B);
	static bool AABBToCircle(Manifold* M);

	static float Distance(glm::vec2 A, glm::vec2 B);

	glm::vec2 Gravity{};
	float TimeStep{};

private:
	std::vector<Object*> Actors;

	static float LengthSquared(glm::vec2 Vector);
};

