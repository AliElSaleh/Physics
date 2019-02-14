#include "Physics2DEngine.h"
#include "Texture.h"
#include "Font.h"
#include "Input.h"

Physics2DEngine::Physics2DEngine() = default;

Physics2DEngine::~Physics2DEngine() = default;

bool Physics2DEngine::Startup() {
	
	Renderer = new aie::Renderer2D();
	Font = new aie::Font("../bin/font/consolas.ttf", 32);

	return true;
}

void Physics2DEngine::Shutdown() {

	delete Font;
	delete Renderer;
}

void Physics2DEngine::Update(float deltaTime) {

	aie::Input* input = aie::Input::getInstance();



	if (input->isKeyDown(aie::INPUT_KEY_ESCAPE))
		Quit();
}

void Physics2DEngine::Draw() {

	ClearScreen();

	Renderer->begin();


	// UI
	DrawText();

	Renderer->end();
}

void Physics2DEngine::DrawText()
{
	Renderer->drawText(Font, "2D Physics Engine", 10, 10);
}
