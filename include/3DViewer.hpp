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
    double vScale = 0.01;
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
    Eigen::Matrix4d worldToPixel();
    Eigen::Matrix4d objectToWorld();

    std::vector<std::pair<Vec3D, Color>> calculateRayIntersection(int px, int py);
    
    private:
    void shutdown();
    void detectInteraction(INTERACTION* inter, double* value);
    void updateCamera(INTERACTION* inter, double* value);
    void updateDisplayBuffer();
    void initBuffer();
    void sortBuffer();

    Eigen::Matrix4d cameraToWorld();
    Eigen::Matrix4d worldToCamera();
    Eigen::Matrix4d imagePlaneToCamera();
    Eigen::Matrix4d cameraToImagePlane();
    Eigen::Matrix4d pixelToImagePlane();
    Eigen::Matrix4d imagePlaneToPixel();
};

inline void createRandomBuffer(std::vector<std::pair<Vec3D, Color>>& buffer, const size_t width, const size_t height){
    buffer = std::vector<std::pair<Vec3D, Color>>(width * height * 1, std::pair<Vec3D, Color>({0,0,0}, {0,0,0,0}));
    for(int idx = 0; idx < buffer.size(); idx++){
        double x_coord = idx % width;
        double y_coord = idx / width;
        Uint8 randR = rand() % 255;
        Uint8 randG = rand() % 255;
        Uint8 randB = rand() % 255;
        Color c{randR, randG, randB, 255};
        buffer[idx] = std::pair<Vec3D, Color>({x_coord, y_coord, 0}, c);
    }
};

inline void createCircleBuffer(std::vector<std::pair<Vec3D, Color>>& buffer, const size_t width, const size_t height, int xCenter = -1, int yCenter = -1, double circleSize = -1){
    buffer = std::vector<std::pair<Vec3D, Color>>(width * height * 1, std::pair<Vec3D, Color>({0,0,0}, {0,0,0,255}));
    if(xCenter == -1){
        xCenter = width / 2.0;
    }
    if(yCenter == -1){
        yCenter = height / 2.0;
    }
    if(circleSize == -1){
        circleSize = std::min(width, height) / 2.0;
    }

    for(int idx = 0; idx < buffer.size(); idx++){
        double xCoord = idx % width;
        double yCoord = idx / width;

        double xDist = xCoord - xCenter;
        double yDist = yCoord - yCenter;
        double r = xDist * xDist + yDist * yDist;
        Uint8 randR = 0;
        Uint8 randG = 0;
        Uint8 randB = 0;
        if (std::sqrt(r) < circleSize){
            randR = rand() % 255;
            randG = rand() % 255;
            randB = rand() % 255;
        }
        Color c{randR, randG, randB, 255};
        buffer[idx] = std::pair<Vec3D, Color>({xCoord, yCoord, 0}, c);
    }
};
