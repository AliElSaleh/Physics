#pragma once
#include "Object.h"

#include <vector>
#include "Manifold.h"

#define WHITE {1.0f, 1.0f, 1.0f, 1.0f}
#define RED {1.0f, 0.0f, 0.0f, 1.0f}
#define GREEN {0.0f, 1.0f, 0.0f, 1.0f}
#define BLUE {0.0f, 0.0f, 1.0f, 1.0f}
#define LIGHT_BLUE {0.0f, 1.0f, 1.0f, 1.0f}
#define YELLOW {1.0f, 1.0f, 0.0f, 1.0f}

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

	static bool AABBToAABB(Manifold* M);
	static bool AABBToCircle(Manifold* M);
	static bool AABBToPlane(Manifold* M);
	static bool AABBToOBB(Manifold* M);
	static bool CircleToAABB(Manifold* M);
	static bool CircleToCircle(Manifold* M);
	static bool CircleToPlane(Manifold* M);
	static bool CircleToOBB(Manifold* M);
	static bool PlaneToCircle(Manifold* M);
	static bool PlaneToAABB(Manifold* M);
	static bool PlaneToPlane(Manifold* M);
	static bool OBBToAABB(Manifold* M);
	static bool OBBToCircle(Manifold* M);
	static bool OBBToOBB(Manifold* M);
	static bool OBBToPlane(Manifold* M);

	static float Distance(glm::vec2 A, glm::vec2 B);

	static Interval GetInterval(const class AABB& Rec, const glm::vec2& Axis);
	static Interval GetInterval(const class OBB& Box, const glm::vec2& Axis);
	static bool OverlapOnAxis(const class AABB& Rec, const class OBB& Box, const glm::vec2& Axis);
	static bool OverlapOnAxis(const class OBB& Box1, const class OBB& Box2, const glm::vec2& Axis);

	static bool PointOnAABB(const glm::vec2& Point, const class AABB& Rec);

	glm::vec2 Gravity{};
	float TimeStep{};

private:
	std::vector<Object*> Actors;

	static void PrintCollided(Manifold* M, Geometry Type1, Geometry Type2);
	static void PrintError(Object* A, Object* B, Geometry Type1, Geometry Type2);

	static bool Multiply(float* Out, const float* MatA, int ARows, int ACols, const float* MatB, int BRows, int BCols);

	static glm::vec2 Normalize(const glm::vec2& Vector);
	static float MagnitudeSquared(const glm::vec2& Vector);
	static float LengthSquared(glm::vec2 Vector);
};

