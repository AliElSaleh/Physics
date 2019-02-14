#pragma once

#include "Application.h"
#include "Renderer2D.h"

class Physics2DEngine final : public aie::Application
{
public:

	Physics2DEngine();
	virtual ~Physics2DEngine();

	bool Startup() override;
	void Shutdown() override;

	void Update(float deltaTime) override;
	void Draw() override;

protected:

	aie::Renderer2D* Renderer{};
	aie::Font* Font{};

private:

	void DrawText();
};