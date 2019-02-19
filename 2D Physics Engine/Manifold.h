#pragma once
#include "Object.h"

class Manifold
{
public:
	Manifold();
	~Manifold();

	Object* A{};
	Object* B{};

	float Penetration{ 0.05f };

	glm::vec2 Normal{};

	unsigned int ContactsCount{};
};

