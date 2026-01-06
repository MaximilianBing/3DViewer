#include <iostream>
#include <SDL2/SDL.h>
#include <spdlog/spdlog.h>
#include <3DViewer.hpp>
#include <Eigen/Core>

int main(int argc, char** argv){
	spdlog::set_level(spdlog::level::debug);

	const int width = 400;
	const int height = 400;

	std::vector<std::pair<Vec3D, Color>> buffer;

	createDepthBufferExample(buffer, width, height);
	ImageViewer viewer(width, height, 1, buffer, "Testing Phase");
	viewer.init();

	while (viewer.isRunning()) {
		viewer.handleEvents();
		viewer.update();
		viewer.render();
	}
	return EXIT_SUCCESS;
}
