#include <iostream>
#include <SDL2/SDL.h>
#include <spdlog/spdlog.h>
#include <3DViewer.hpp>
#include <Eigen/Core>

int main(int argc, char** argv){
	spdlog::set_level(spdlog::level::debug);

	const int width = 400;
	const int height = 400;
	std::pair<Vec3D, Color> test1, test2, test3, test4, test5;
	test1.first = Vec3D{width/2.0, height/ 2.0, 0};
	test1.second = Color{0, 255, 0, 255};
	test2.first = Vec3D{width, height, 0};
	test2.second = Color{255, 0, 255, 255};
	test3.first = Vec3D{0, 0, 0};
	test3.second = Color{0, 0, 255, 255};
	test4.first = Vec3D{width, 0, 0};
	test4.second = Color{255, 0, 0, 255};

	// std::vector<std::pair<Vec3D, Color>> buffer = {test1, test2, test3, test4, test5};//(width * height * 1, std::pair<Vec3D, Color>({0,0,0}, {0,0,0,0}));
	std::vector<std::pair<Vec3D, Color>> buffer;

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
