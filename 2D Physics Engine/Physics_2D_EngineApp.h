#pragma once

#include "Application.h"
#include "Renderer2D.h"

class Physics2DEngine : public aie::Application {
public:

	Physics2DEngine();
	virtual ~Physics2DEngine();

	virtual bool startup();
	virtual void shutdown();

	virtual void update(float deltaTime);
	virtual void draw();

protected:

	aie::Renderer2D*	m_2dRenderer;
	aie::Font*			m_font;
};