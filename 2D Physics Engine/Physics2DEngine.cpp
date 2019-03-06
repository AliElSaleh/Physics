#include "Physics2DEngine.h"
#include "Font.h"
#include "Input.h"
#include "AABB.h"
#include "Circle.h"
#include "Gizmos.h"
#include "Plane.h"
#include "OBB.h"
#include "../dependencies/glfw/include/GLFW/glfw3.h"

#include <glm/gtc/matrix_transform.inl>
#include <string.h>
#include <string>
#include <stdio.h>

Physics2DEngine::Physics2DEngine() = default;
Physics2DEngine::~Physics2DEngine() = default;

bool Physics2DEngine::Startup()
{
	Renderer = new aie::Renderer2D();
	Font = new aie::Font("../bin/font/consolas.ttf", 32);
	FontSmall = new aie::Font("../bin/font/consolas.ttf", 24);

	aie::Gizmos::create(255U, 255U, 65535U, 65535U);

	PhysicsWorld = new World();
	PhysicsWorld->Gravity = {0.0f, -19.81f};
	PhysicsWorld->TimeStep = 0.01f;
	
	// Borders
	// Left
	//auto B = new class AABB({ -95.0f, 0.0f }, { 0.0f, 0.0f }, 5.0f, 100.0f, 10.0f, { 0.780f, 0.403f, 0.0f, 1.0f });
	//B->SetKinematic(true);
	//PhysicsWorld->AddActor(B);
	//
	//// Right
	//B = new class AABB({ 95.0f, 0.0f }, { 0.0f, 0.0f }, 5.0f, 100.0f, 10.0f, { 0.780f, 0.403f, 0.0f, 1.0f });
	//B->SetKinematic(true);
	//PhysicsWorld->AddActor(B);
	
	//// Top
	//B = new class AABB({ 0.0f, 50.0f }, { 0.0f, 0.0f }, 180.0f, 5.0f, 10.0f, { 0.780f, 0.403f, 0.0f, 1.0f });
	//B->SetKinematic(true);
	//PhysicsWorld->AddActor(B);
	
	// Bottom
	//B = new class AABB({ 0.0f, -50.0f }, { 0.0f, 0.0f }, 180.0f, 5.0f, 10.0f, { 0.780f, 0.403f, 0.0f, 1.0f });
	//B->SetKinematic(true);
	//PhysicsWorld->AddActor(B);

	//auto R = new Box({ 20.0f, 20.0f }, { -20.0f, -10.0f }, {10.0f, 5.0f}, 0.0f, 10.0f, { 1.0f, 0.0f, 0.0f, 1.0f });
	//PhysicsWorld->AddActor(R);

	//auto R = new AABB({ 0.0f, 0.0f }, { 0.0f, -20.0f }, 3, 3, 10.0f, { 1.0f, 1.0f, 0.0f, 1.0f });
	//PhysicsWorld->AddActor(R);
	//
	//R = new AABB({ 0.0f, 40.0f }, { 0.0f, -10.0f }, 3, 3, 10.0f, { 1.0f, 1.0f, 0.0f, 1.0f });
	//PhysicsWorld->AddActor(R);

	//const auto C = new Circle({ 0.0f, 0.0f }, { 10.0f, -10.0f }, 6.0f, 10.0f, { 0.0f, 1.0f, 1.0f, 1.0f });
	//PhysicsWorld->AddActor(C);

	// Plane
	glm::vec2 Normal = { -0.65f, 0.75f };
	PhysicsWorld->AddActor(new Plane(Normal, -30.0f));
	
	// Plane
	Normal = { 0.65f, 0.75f };
	PhysicsWorld->AddActor(new Plane(Normal, -30.0f));

	// Pool setup
	//float XOffset = -50;
	//float YOffset = 6;
	//
	//// First row
	//for (int i = 0; i < 4; i++)
	//{
	//	const auto C = new Circle({ XOffset, YOffset }, { 0.0f, 0.0f }, 3.0f, 2.0f, { 1, 0.992, 0.658, 1.0f });
	//	YOffset -= 6;
	//	PhysicsWorld->AddActor(C);
	//}
	//
	//XOffset = -44;
	//YOffset = 3;
	//
	//// Second row
	//for (int i = 0; i < 3; i++)
	//{
	//	const auto C = new Circle({ XOffset, YOffset }, { 0.0f, 0.0f }, 3.0f, 2.0f, { 1, 0.992, 0.658, 1.0f });
	//	YOffset -= 6;
	//	PhysicsWorld->AddActor(C);
	//}
	//
	//XOffset = -38;
	//YOffset = 0;
	//
	//// Third row
	//for (int i = 0; i < 2; i++)
	//{
	//	const auto C = new Circle({ XOffset, YOffset }, { 0.0f, 0.0f }, 3.0f, 2.0f, { 1, 0.992, 0.658, 1.0f });
	//	YOffset -= 6;
	//	PhysicsWorld->AddActor(C);
	//}
	//
	//XOffset = -32;
	//YOffset = -3;
	//
	//// Final row
	//auto C = new Circle({ XOffset, YOffset }, { 0.0f, 0.0f }, 3.0f, 2.0f, { 1, 0.992, 0.658, 1.0f });
	//PhysicsWorld->AddActor(C);
	//
	//// Cue
	//Cue = new Circle({ 50.0f, -3.0f }, { 0.0f, 0.0f }, 3.0f, 5.0f, { 1.0f, 1.0f, 1.0f, 1.0f });
	//PhysicsWorld->AddActor(Cue);
	//

	// AABBs
	//for (int i = 0; i < 10; i++)
	//{
	//	const auto Rec = new class AABB({ rand() % 30 - 20, rand() % 80 - 30}, { 0.0f, 0.0f }, 3.0f, 3.0f, 2.0f, { 1, 0.992, 0.658, 1.0f });
	//	PhysicsWorld->AddActor(Rec);
	//}
	
	// OBBs
	//for (int i = 0; i < 10; i++)
	//{
	//	const auto Box = new class OBB({ rand() % 30 - 20, rand() % 80 - 30}, { 0.0f, 0.0f }, {1.5f, 1.5f}, 45, 2.0f, { 1, 0.992, 0.658, 1.0f });
	//	PhysicsWorld->AddActor(Box);
	//}

	// Circles
	//for (int i = 0; i < 10; i++)
	//{
	//	const auto C = new Circle({ rand() % 30 - 20, rand() % 80 - 30}, { 0.0f, 0.0f }, 2.0f, 2.0f, { 1, 0.992, 0.658, 1.0f });
	//	PhysicsWorld->AddActor(C);
	//}

	return true;
}

void Physics2DEngine::Shutdown()
{
	delete Font;
	delete Renderer;
	delete PhysicsWorld;
}

void Physics2DEngine::Update(const float DeltaTime)
{
	aie::Input* Input = aie::Input::getInstance();

	char Result[10];

	strcpy_s(Result, "FPS: ");
	strcat_s(Result, std::to_string(GetFPS()).c_str());
	
	glfwSetWindowTitle(GetWindowPtr(), Result);

	// Clear gizmos
	aie::Gizmos::clear();

	PhysicsWorld->Update(DeltaTime);

	if (Input->wasKeyPressed(aie::INPUT_KEY_C))
	{
		const auto C = new Circle({0.0f, 40.0f}, { 0.0f, -10.0f }, 2.0f, 2.0f, { 1, 0.992, 0.658, 1.0f });
		PhysicsWorld->AddActor(C);
	}

	if (Input->wasKeyPressed(aie::INPUT_KEY_A))
	{
		const auto Rec = new class AABB({0.0f, 40.0f}, { 0.0f, 0.0f }, 3.0f, 3.0f, 2.0f, { 1, 0.992, 0.658, 1.0f });
		PhysicsWorld->AddActor(Rec);
	}

	if (Input->wasKeyPressed(aie::INPUT_KEY_B))
	{
		const auto Box = new class OBB({0.0f, 40.0f}, { 0.0f, 0.0f }, {1.5f, 1.5f}, rand() % 360, 2.0f, { 1, 0.992, 0.658, 1.0f });
		PhysicsWorld->AddActor(Box);
	}
	// Check if mouse is intersecting the cue ball
	//if (Input->isKeyDown(aie::INPUT_KEY_A))
	//	Cue->ApplyForce({ -10.0f, 0.0f });
	//
	//if (Input->isKeyDown(aie::INPUT_KEY_D))
	//	Cue->ApplyForce({ 10.0f, 0.0f });
	//
	//if (Input->isKeyDown(aie::INPUT_KEY_W))
	//	Cue->ApplyForce({ 0.0f, 10.0f });
	//
	//if (Input->isKeyDown(aie::INPUT_KEY_S))
	//	Cue->ApplyForce({ 0.0f, -10.0f });
	//
	//int x, y;
	//Input->getMouseXY(&x, &y);
	//
	//MouseLocation = glm::vec2( x, y );

	PhysicsWorld->UpdateGizmos();

	if (Input->isKeyDown(aie::INPUT_KEY_ESCAPE))
		Quit();
}

void Physics2DEngine::Draw()
{
	ClearScreen();

	Renderer->begin();

	// UI
	DrawText();

	// Gizmos
	static float AspectRatio = 16.0f / 9.0f;
	aie::Gizmos::draw2D(glm::ortho<float>(-100, 100, -100 / AspectRatio, 100 / AspectRatio, -1.0f, 1.0f));

	Renderer->end();
}

void Physics2DEngine::DrawText()
{
	Renderer->drawText(Font, "2D Physics Engine", 10, 12);

	Renderer->drawText(Font, "Popcorn physics engine", float( GetWindowWidth()) / 2 - 200, float(GetWindowHeight()) / 2);
	Renderer->drawText(FontSmall, "*World's first", float(GetWindowWidth()) / 2 - 110, float(GetWindowHeight()) / 2 - 30);

	Renderer->drawText(FontSmall, "Pop!", float( GetWindowWidth()) / 2 - 100, float(GetWindowHeight()) / 2 + 50);
	Renderer->drawText(FontSmall, "Pop!", float( GetWindowWidth()) / 2 + 200, float(GetWindowHeight()) / 2 + 100);
	Renderer->drawText(FontSmall, "Pop!", float( GetWindowWidth()) / 2 - 250, float(GetWindowHeight()) / 2 - 100);
}
