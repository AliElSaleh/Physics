#include "_2D_Physics_EngineApp.h"

int main() {
	
	// allocation
	auto app = new _2D_Physics_EngineApp();

	// initialise and loop
	app->run("AIE", 1280, 720, false);

	// deallocation
	delete app;

	return 0;
}