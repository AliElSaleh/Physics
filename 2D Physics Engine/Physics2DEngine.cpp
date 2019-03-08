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
	
	Ball = new Circle({ 95.0f, -55.0f }, { 0.0f, -10.0f }, 3.0f, 1.5f, { 1, 0.992, 0.658, 1.0f });
	Ball->SetKinematic(true);
	PhysicsWorld->AddActor(Ball);

	// Borders
	// Plane
	// Left
	glm::vec2 Normal = { 1.0f, 0.0f };
	PhysicsWorld->AddActor(new Plane(Normal, -99.5f, 300));

	// Right
	Normal = { 1.0f, 0.0f };
	PhysicsWorld->AddActor(new Plane(Normal, 99.7f, 300));

	// Barrier
	auto B = new class AABB({ 90.0f, -20.0f }, { 0.0f, 0.0f }, 1.0f, 80.0f, 2.0f, { 1.0f, 1.0f, 1.0f, 1.0f });
	B->SetKinematic(true);
	PhysicsWorld->AddActor(B);

	// Top
	Normal = { 0.0f, 1.0f };
	PhysicsWorld->AddActor(new Plane(Normal, 72.0f, 300));

	// Bottom
	Normal = { 0.0f, 1.0f };
	const auto BottomPlane = new Plane(Normal, -60.0f, 300);
	BottomPlane->SetKinematic(true);
	PhysicsWorld->AddActor(BottomPlane);

	// Right diagonal
	Normal = { 0.5f, 0.5f };
	PhysicsWorld->AddActor(new Plane(Normal, 150.0f, 300));

	// Left diagonal
	Normal = { 0.5f, -0.5f };
	PhysicsWorld->AddActor(new Plane(Normal, -150.0f, 300));

	// Kinematic circles
	for (int i = 0; i < 4; i++)
	{
		for (int j = 0; j < 10; j++)
		{
			auto C = new Circle({ -80.0f + XSpacing, 30.0f + YSpacing }, { 0.0f, 0.0f }, 2.0f, 1.0f, { 1.0f, 1.0f, 1.0f, 1.0f });
			C->SetKinematic(true);
			PhysicsWorld->AddActor(C);
			XSpacing += 15;
		}

		XSpacing = 10;
		YSpacing -= 15;
	}

	XSpacing = 17;
	YSpacing = 3;

	// Kinematic AABBs
	for (int i = 0; i < 4; i++)
	{
		for (int j = 0; j < 9; j++)
		{
			const auto O = new class OBB({ -80.0f + XSpacing, 30.0f + YSpacing }, { 0.0f, 0.0f }, { 1.0f, 1.0f }, 45.0f, 2.0f, { 1.0f, 1.0f, 1.0f, 1.0f });
			O->SetKinematic(true);
			PhysicsWorld->AddActor(O);
			XSpacing += 15;
		}

		XSpacing = 17;
		YSpacing -= 15;
	}

	XSpacing = 15.0f;

	// Kinematic AABBs
	for (int i = 0; i < 12; i++)
	{
		B = new class AABB({ -105.0f + XSpacing, -45.0f }, { 0.0f, 0.0f }, 1.0f, 30.0f, 2.0f, { 1.0f, 1.0f, 1.0f, 1.0f });
		B->SetKinematic(true);
		PhysicsWorld->AddActor(B);
		XSpacing += 15.0f;
	}

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

	if (Input->wasKeyPressed(aie::INPUT_KEY_SPACE) && CanShoot)
	{
		Ball->SetKinematic(false);
		Ball->ApplyForce({ 0.0f, 200.0f });
		CanShoot = false;
	}

	// Reset when collided with plane
	if (Ball->Collided)
	{
		CanShoot = true;
		Ball->SetLocation({ 95.0f, -55.0f });
		Ball->SetKinematic(true);
		Ball->Collided = false;
	}

	if (Input->wasKeyPressed(aie::INPUT_KEY_C))
	{
		const auto C = new Circle({ rand() % -20 - rand() % 20, 70.0f }, { 0.0f, -10.0f }, 3.0f, 1.5f, { 1, 0.992, 0.658, 1.0f });
		PhysicsWorld->AddActor(C);
	}

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
	static float AspectRatio = 4.0f / 3.0f;
	aie::Gizmos::draw2D(glm::ortho<float>(-100, 100, -100 / AspectRatio, 100 / AspectRatio, -1.0f, 1.0f));

	Renderer->end();
}

void Physics2DEngine::DrawText()
{
	Renderer->drawText(Font, "Pachinko", 10, 12);
}
