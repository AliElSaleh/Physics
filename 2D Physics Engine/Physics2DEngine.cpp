#include "Physics2DEngine.h"
#include "Font.h"
#include "Input.h"
#include "AABB.h"
#include "Circle.h"
#include "Gizmos.h"

#include <glm/gtc/matrix_transform.inl>
#include "Plane.h"
#include "../dependencies/glfw/include/GLFW/glfw3.h"
#include <string>
#include <string.h>
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
	auto B = new AABB({ -90.0f, 0.0f }, { 0.0f, 0.0f }, 5.0f, 100.0f, 10.0f, { 1.0f, 1.0f, 1.0f, 1.0f });
	B->SetKinematic(true);
	PhysicsWorld->AddActor(B);
	
	// Right
	B = new AABB({ 90.0f, 0.0f }, { 0.0f, 0.0f }, 5.0f, 100.0f, 10.0f, { 1.0f, 1.0f, 1.0f, 1.0f });
	B->SetKinematic(true);
	PhysicsWorld->AddActor(B);
	
	// Top
	B = new AABB({ 0.0f, 45.0f }, { 0.0f, 0.0f }, 150.0f, 5.0f, 10.0f, { 1.0f, 1.0f, 1.0f, 1.0f });
	B->SetKinematic(true);
	PhysicsWorld->AddActor(B);
	
	// Bottom
	B = new AABB({ 0.0f, -45.0f }, { 0.0f, 0.0f }, 150.0f, 5.0f, 10.0f, { 1.0f, 1.0f, 1.0f, 1.0f });
	B->SetKinematic(true);
	PhysicsWorld->AddActor(B);

	//const auto R = new AABB({ 0.0f, 0.0f }, { 10.0f, 0.0f }, 10, 20, 10.0f, { 1.0f, 0.0f, 0.0f, 1.0f });
	//PhysicsWorld->AddActor(R);

	//const auto C = new Circle({ 0.0f, 0.0f }, { 70.0f, 0.0f }, 6.0f, 10.0f, { 0.0f, 1.0f, 1.0f, 1.0f });
	//PhysicsWorld->AddActor(C);

	// Plane
	glm::vec2 Normal = { -0.65f, 0.75f };
	PhysicsWorld->AddActor(new Plane(Normal, -30.0f));

	// Plane
	Normal = { 0.65f, 0.75f };
	PhysicsWorld->AddActor(new Plane(Normal, -30.0f));

	// AABBs
	for (int i = 0; i < 5; i++)
	{
		const glm::vec2 Location = { rand() % 50, rand() % 50 };
		const glm::vec2 Velocity = { rand() % 20 - 20, rand() % 20 - 20 };
		const float Mass = rand() % 10;
	
		auto* R = new AABB(Location, Velocity, 3, 3, Mass, {1.0f, 1.0f, 0.0f, 1.0f});
		PhysicsWorld->AddActor(R);
	}

	// Circles
	//for (int i = 0; i < 20; i++)
	//{
	//	const auto C = new Circle({ rand() % 60 - 60, rand() % 80 - 80}, { rand() % 10 + 1, rand() % 10 + 1 }, rand() % 6 + 1, rand() % 10 + 1, { 1, 0, 0, 1.0f });
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
	aie::Input* input = aie::Input::getInstance();

	char Result[10];

	strcpy_s(Result, "FPS: ");
	strcat_s(Result, std::to_string(GetFPS()).c_str());

	
	glfwSetWindowTitle(GetWindowPtr(), Result);

	// Clear gizmos
	aie::Gizmos::clear();

	PhysicsWorld->Update(DeltaTime);

	PhysicsWorld->UpdateGizmos();

	if (input->isKeyDown(aie::INPUT_KEY_ESCAPE))
		Quit();
}

void Physics2DEngine::Draw()
{
	ClearScreen();

	Renderer->begin();

	// Gizmos
	static float AspectRatio = 16.0f / 9.0f;
	aie::Gizmos::draw2D(glm::ortho<float>(-100, 100, -100 / AspectRatio, 100 / AspectRatio, -1.0f, 1.0f));

	// UI
	DrawText();

	Renderer->end();
}

void Physics2DEngine::DrawText()
{
	Renderer->drawText(Font, "2D Physics Engine", 10, 10);
	Renderer->drawText(Font, "Popcorn physics engine", float( GetWindowWidth()) / 2 - 200, float(GetWindowHeight()) / 2);
	Renderer->drawText(FontSmall, "*World's first", float( GetWindowWidth()) / 2 - 110, float(GetWindowHeight()) / 2 - 30);
}
