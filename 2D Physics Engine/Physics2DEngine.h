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

private:
	float XSpacing = 10.0f;
	float YSpacing = 10.0f;

	bool CanShoot = true;

	glm::vec2 SliderLocation{};
	float SliderLength = 1;
	float IncrementRate = 10.0f;

	bool ReachedMax{false};
	bool ReachedMin{false};

	void DrawText();
};