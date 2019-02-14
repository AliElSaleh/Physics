#include "Physics2DEngine.h"

int main() {
	
	// allocation
	auto app = new Physics2DEngine();

	// initialise and loop
	app->run("AIE", 1280, 720, false);

	// deallocation
	delete app;

	return 0;
}