#include <iostream>
#include <SDL2/SDL.h>
#include <spdlog/spdlog.h>
#include <3DViewer.hpp>
#include <Eigen/Core>

int main(int argc, char** argv){
	const size_t width = 40;
	const size_t height = 40;
	std::vector<std::pair<Vec3D, Color>> buffer;//(width * height * 1, std::pair<Vec3D, Color>({0,0,0}, {0,0,0,0}));
	
	createCircleBuffer(buffer, width, height);
	ImageViewer viewer(width, height, 1, buffer, "Testing Phase");
	viewer.init();

	while (viewer.isRunning()) {
		viewer.handleEvents();
		viewer.update();
		viewer.render();
	}
	return EXIT_SUCCESS;
}
