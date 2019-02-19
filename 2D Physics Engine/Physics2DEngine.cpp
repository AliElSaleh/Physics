#include "Physics2DEngine.h"
#include "Font.h"
#include "Input.h"
#include "AABB.h"
#include "Circle.h"
#include "Gizmos.h"

#include <glm/gtc/matrix_transform.inl>

Physics2DEngine::Physics2DEngine() = default;

Physics2DEngine::~Physics2DEngine() = default;

bool Physics2DEngine::Startup()
{
	Renderer = new aie::Renderer2D();
	Font = new aie::Font("../bin/font/consolas.ttf", 32);

	aie::Gizmos::create(255U, 255U, 65535U, 65535U);

	PhysicsWorld = new World();
	PhysicsWorld->Gravity = {0.0f, -9.81f};
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
	B = new AABB({ 0.0f, 45.0f }, { 0.0f, 0.0f }, 150.0f, 5.0f, 1.0f, { 1.0f, 1.0f, 1.0f, 1.0f });
	B->SetKinematic(true);
	PhysicsWorld->AddActor(B);
	
	// Bottom
	B = new AABB({ 0.0f, -45.0f }, { 0.0f, 0.0f }, 150.0f, 5.0f, 1.0f, { 1.0f, 1.0f, 1.0f, 1.0f });
	B->SetKinematic(true);
	PhysicsWorld->AddActor(B);

	// AABB vs AABB
	//auto* R = new AABB({ -30.0f, -30.0f }, { 20.0f, 0.0f }, 10.0f, 20.0f, 8.0f, { 1.0f, 0.0f, 0.0f, 1.0f });
	//PhysicsWorld->AddActor(R);
	//
	//R = new AABB({ 30.0f, -30.0f }, { -20.0f, 0.0f }, 5.0f, 30.0f, 6.0f, { 0.0f, 1.0f, 0.0f, 1.0f });
	//R->SetKinematic(true);
	//PhysicsWorld->AddActor(R);

	// Circle vs Circle
	auto* C = new Circle({ -40.0f, 0.0f }, { 30.0f, 0.0f }, 8.0f, 15.0f, { 1.0f, 0.0f, 0.0f, 1.0f });
	PhysicsWorld->AddActor(C);

	// AABB vs Circle
	//const auto R = new AABB({ -50.0f, 0.0f }, { 20.0f, 0.0f }, 10.0f, 20.0f, 6.0f, { 1.0f, 0.0f, 0.0f, 1.0f });
	//R->SetKinematic(false);
	//PhysicsWorld->AddActor(R);
	
	C = new Circle({ 30.0f, 0.0f }, { -30.0f, 0.0f }, 6.0f, 15.0f, { 0.0f, 1.0f, 1.0f, 1.0f });
	PhysicsWorld->AddActor(C);

	// Circles
	//for (int i = 0; i < 20; i++)
	//{
	//	const auto C = new Circle({ rand() % 60 - 60, rand() % 80 - 80}, { rand() % 10 + 1, rand() % 10 + 1 }, rand() % 6 + 1, rand() % 10 + 1, { rand() % 1 + 1, rand() % 1 + 1, rand() % 1 + 1, 1.0f });
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
}
