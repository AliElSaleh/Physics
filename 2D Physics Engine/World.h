#pragma once
#include "Object.h"

#include <vector>
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

	static bool AABBToAABB(Object* A, Object* B);
	static bool CircleToCircle(Object* A, Object* B);

	static float Distance(glm::vec2 A, glm::vec2 B);

	glm::vec2 Gravity{};
	float TimeStep{};

private:
	std::vector<Object*> Actors;
};

