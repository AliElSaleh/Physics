#pragma once

#include "Application.h"
#include "Renderer2D.h"
#include "World.h"

class Physics2DEngine final : public aie::Application
{
public:

	Physics2DEngine();
	virtual ~Physics2DEngine();

	bool Startup() override;
	void Shutdown() override;

	void Update(float DeltaTime) override;
	void Draw() override;

protected:

	aie::Renderer2D* Renderer{};
	aie::Font* Font{};
	aie::Font* FontSmall{};

	World* PhysicsWorld{};

	Circle* Ball{};

	glm::vec2 MouseLocation{};
private:
	float XSpacing = 10.0f;
	float YSpacing = 10.0f;

	bool CanShoot = true;

	void DrawText();
};