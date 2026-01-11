#pragma once

#include <SDL2/SDL.h>
#include <SDL2/SDL_events.h>
#include <SDL2/SDL_keyboard.h>
#include <SDL2/SDL_image.h>

#include <Eigen/Core>
#include <Eigen/Dense>

#include <spdlog/spdlog.h>
#include <iostream>
#include <string>
#include <numeric>

typedef SDL_Color Color;

struct Vec3D{
    double x = 0;
    double y = 0;
    double z = 0;
};

class ImageViewer{
    private:
    struct Camera{
        Vec3D position;
        Vec3D orientation;
        Vec3D up;
    };

    enum class INTERACTION{TRANSLATE_HORIZONTAL, TRANSLATE_VERTICAL, ROTATE_HORIZONTAL, ROTATE_VERTICAL, SCALE_IMAGE, SCALE_WINDOW, QUIT, NON_TYPE};

    size_t width = 500;
    size_t height = 500;
    size_t depth = 0;

    std::vector<std::pair<Vec3D, Color>> rawBuffer;
    std::vector<std::pair<Vec3D, Color>> mappedBuffer;
    std::vector<Color> displayBuffer;
    std::vector<double> depthBuffer;

    std::string windowTitle;
    SDL_Window *p_window;
    SDL_Renderer* p_renderer;
    SDL_Texture* p_texture;
    SDL_Event *p_event;
    bool running = false;
    bool cameraChanged = false;

    Camera cam{0,0,1,0,0,-1, 0, 1,0};
    double cx;
    double cy;
    double f = 1.0;
    double s;


    double vTranslate = 0.01;
    double vScale = 0.99;
    double vRotate = 0.01;

    double min_x, min_y, min_z;
    double max_x, max_y, max_z;
    
    public:
    ImageViewer(const size_t width_, const size_t height_, const size_t depth_, std::vector<std::pair<Vec3D, Color>>& buffer_, std::string windowTitle_ = "Window Title"){
        this->depth = depth_;
        this->height = height_;
        this->width = width_;
        this->s = this->f / this->width;
        this->cx = (this->width - 1) / 2.0;
        this->cy = (this->height - 1) / 2.0;
        
        this->rawBuffer = buffer_;
        this->displayBuffer.resize(width_ * height_ * depth_);
        this->depthBuffer = std::vector<double>(this->width * this->height, +INFINITY);
        this->windowTitle = windowTitle_;

        this->min_x = std::numeric_limits<double>::max();
        this->max_x = - std::numeric_limits<double>::max();
        this->min_y = std::numeric_limits<double>::max();
        this->max_y = - std::numeric_limits<double>::max();
        this->min_z = std::numeric_limits<double>::max();
        this->max_z = - std::numeric_limits<double>::max();
    }

    template<typename VectorType>
    int transform3Dto1D(VectorType const& vec){
        return static_cast<int>(vec.z()) * this->width * this->height + static_cast<int>(vec.y()) * this->width + static_cast<int>(vec.x());
    };

    Eigen::Vector3i transform1Dto3D(int const& idx){
        int z = idx / (width * height);
        int y = (idx - z * width * height) / width;
        int x = (idx - z * width * height) % width;
        return Eigen::Vector3i(x,y,z);
    };

    bool isRunning();
    bool init();
    void handleEvents();
    void update();
    void render();
    void addToBuffer(const Vec3D pos, Color c);

    Eigen::Matrix4d pixelToWorld();
    Eigen::Vector3d pixelToWorld(const Eigen::Vector2d& point);
    Eigen::Matrix4d worldToPixel();
    Eigen::Vector3d worldToPixel(const Eigen::Vector3d& point);
    Eigen::Matrix4d objectToWorld();
    Eigen::Vector3d objectToWorld(const Eigen::Vector3d& point);

    std::vector<std::pair<Vec3D, Color>> calculateRayIntersection(int px, int py);
    
    private:
    void shutdown();
    void detectInteraction(INTERACTION* inter, double* value);
    void updateCamera(INTERACTION* inter, double* value);
    void updateDisplayBuffer();
    void initBuffer();
    void sortBuffer();

    Eigen::Matrix4d cameraToWorld();
    Eigen::Vector3d cameraToWorld(const Eigen::Vector3d& point);
    Eigen::Matrix4d worldToCamera();
    Eigen::Vector3d worldToCamera(const Eigen::Vector3d& point);
    Eigen::Matrix4d imagePlaneToCamera();
    Eigen::Vector3d imagePlaneToCamera(const Eigen::Vector3d& point);
    Eigen::Matrix4d cameraToImagePlane();
    Eigen::Vector3d cameraToImagePlane(const Eigen::Vector3d& point);
    Eigen::Matrix4d pixelToImagePlane();
    Eigen::Vector3d pixelToImagePlane(const Eigen::Vector3d& point);
    Eigen::Matrix4d imagePlaneToPixel();
    Eigen::Vector3d imagePlaneToPixel(const Eigen::Vector3d& point);
};

inline void createRandomBuffer(std::vector<std::pair<Vec3D, Color>>& buffer, const size_t width, const size_t height){
    for(int idx = 0; idx < (width * height); idx++){
        double x_coord = idx % width;
        double y_coord = idx / width;
        Uint8 randR = rand() % 255;
        Uint8 randG = rand() % 255;
        Uint8 randB = rand() % 255;
        Color c{randR, randG, randB, 255};
        buffer.push_back(std::pair<Vec3D, Color>({x_coord, y_coord, 0}, c));
    }
};

inline void createCircleBuffer(std::vector<std::pair<Vec3D, Color>>& buffer, const size_t width, const size_t height, int xCenter = -1, int yCenter = -1, double circleSize = -1){
    if(xCenter == -1){
        xCenter = width / 2.0;
    }
    if(yCenter == -1){
        yCenter = height / 2.0;
    }
    if(circleSize == -1){
        circleSize = std::min(width, height) / 2.0;
    }

    for(int idx = 0; idx < (width * height); idx++){
        double xCoord = idx % width;
        double yCoord = idx / width;

        double xDist = xCoord - xCenter;
        double yDist = yCoord - yCenter;
        double r = xDist * xDist + yDist * yDist;

        if (std::sqrt(r) < circleSize){
            Uint8 randR = rand() % 255;
            Uint8 randG = rand() % 255;
            Uint8 randB = rand() % 255;
            buffer.push_back(std::pair<Vec3D, Color>({xCoord, yCoord, 0}, Color{randR, randG, randB, 255}));
        }
    }
};

inline void createSimpleBuffer(std::vector<std::pair<Vec3D, Color>>& buffer, const size_t width, const size_t height){
    double d_height = static_cast<double>(height);
    double d_width = static_cast<double>(width);
    std::pair<Vec3D, Color> test1, test2, test3, test4, test5;
	test1.first = Vec3D{d_width/2.0, d_height/ 2.0, 0};
	test1.second = Color{0, 255, 0, 255};

	test2.first = Vec3D{d_width, d_height, 0};
	test2.second = Color{255, 0, 255, 255};

	test3.first = Vec3D{0, 0, 0};
	test3.second = Color{0, 0, 255, 255};

	test4.first = Vec3D{d_width, 0, 0};
	test4.second = Color{255, 0, 0, 255};

    test5.first = Vec3D{0, d_height, 0};
    test5.second = Color{0, 255, 0, 255};

    buffer.push_back(test1);
    buffer.push_back(test2);
    buffer.push_back(test3);
    buffer.push_back(test4);
    buffer.push_back(test5);
}

// bug detected when creating an image with points not at the width and height borders
inline void createDepthBufferExample(std::vector<std::pair<Vec3D, Color>>& buffer, const size_t width, const size_t height){
    double d_height = static_cast<double>(height);
    double d_width = static_cast<double>(width);

    double margin = 0.25;

    Color cFront{255, 0, 0, 255};
    Color cBack{0, 255, 0, 255};

    for(int idx = 0; idx < (width * height); idx++){
        double xCoord = idx % width;
        double yCoord = idx / width;
        double zBack = -1.0;
        double zFront = 0.0;

        Vec3D pFront{xCoord, yCoord, zFront};
        Vec3D pBack{xCoord, yCoord, zBack};

        if(xCoord < 0.75 * d_width || yCoord < 0.75 * d_height)
            buffer.push_back(std::pair<Vec3D,Color>(pFront, cFront));
        buffer.push_back(std::pair<Vec3D,Color>(pBack, cBack));
    }
}