#include <iostream>
#include <SDL2/SDL.h>
#include <spdlog/spdlog.h>
#include <3DViewer.hpp>
#include <Eigen/Core>

int main(int argc, char** argv){
	const size_t width = 800;
	const size_t height = 800;
	std::vector<std::pair<Vec3D, Color>> buffer;//(width * height * 1, std::pair<Vec3D, Color>({0,0,0}, {0,0,0,0}));
	
	createRandomBuffer(buffer, width, height);
	ImageViewer viewer(width, height, 1, buffer, "Testing Phase");
	while(viewer.displayViewer()){
		viewer.updateBuffer();
		SDL_Delay(50);
	}

	spdlog::info("Pixel to World");
	auto obj = viewer.pixelToWorld(0,0);
	spdlog::info("Input: ({}, {}) Output: ({},{},{})", 0, 0, obj.x(), obj.y(), obj.z());
	obj = viewer.pixelToWorld(width-1, height-1);
	spdlog::info("Input: ({}, {}) Output: ({},{},{})", width - 1, height - 1, obj.x(), obj.y(), obj.z());
	obj = viewer.pixelToWorld(static_cast<double>((width-1)/2.0), static_cast<double>((height-1)/2.0));
	spdlog::info("Input: ({}, {}) Output: ({},{},{})", (width -1)/2.0 , (height-1)/2.0, obj.x(), obj.y(), obj.z());

	spdlog::info("Object to World");
	auto obj2 = viewer.objectToWorld(0,0,0);
	spdlog::info("Input: ({}, {}, {}) Output: ({},{}, {})", 0, 0, 0, obj2.x(), obj2.y(), obj2.z());
	obj2 = viewer.objectToWorld(width - 1, height -1 ,0);
	spdlog::info("Input: ({}, {}, {}) Output: ({},{}, {})", width -1, height -1 , 0, obj2.x(), obj2.y(), obj2.z());
	obj2 = viewer.objectToWorld(static_cast<double>((width-1)/2.0), static_cast<double>((height-1)/2.0),0);
	spdlog::info("Input: ({}, {}, {}) Output: ({},{}, {})", (width -1)/2.0, (height -1)/2.0 , 0, obj2.x(), obj2.y(), obj2.z());
	spdlog::info("Ray Intersection: ");
	viewer.calculateRayIntersection(0,799);
	return EXIT_SUCCESS;
}
