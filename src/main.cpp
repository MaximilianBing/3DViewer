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
	ImageViewer viewer(width, height, 1, buffer);
	while(viewer.displayViewer()){
		viewer.updateBuffer();
		SDL_Delay(50);
	}

	spdlog::info("Pixel to World");
	viewer.pixelToWorld(0,0);
	viewer.pixelToWorld(width-1, height-1);
	viewer.pixelToWorld(static_cast<double>((width-1)/2.0), static_cast<double>((height-1)/2.0));

	spdlog::info("Object to World");
	viewer.objectToWorld(0,0,0);
	viewer.objectToWorld(width - 1, height -1 ,0);
	viewer.objectToWorld(static_cast<double>((width-1)/2.0), static_cast<double>((height-1)/2.0),0);
	return 0;
}
