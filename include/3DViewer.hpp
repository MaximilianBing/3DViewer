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

    std::vector<std::pair<Vec3D, Color>> buffer;
    std::vector<Color> displayBuffer;

    std::string windowTitle;
    SDL_Window *p_window;
    SDL_Event *p_event;
    bool running = false;

    Camera cam{0,0,1,0,0,-1, 0, 1,0};
    double cx;
    double cy;
    double f = 0.5;
    double s;


    double vTranslate = 1.0;
    double vScale = 0.9;
    double vRotate = 1.0;

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
        this->buffer.resize(width_ * height_ * depth_);
        this->windowTitle = windowTitle_;

        this->min_x = std::numeric_limits<double>::max();
        this->max_x = -std::numeric_limits<double>::max();
        this->min_y = std::numeric_limits<double>::max();
        this->max_y = -std::numeric_limits<double>::max();
        this->min_z = std::numeric_limits<double>::max();
        this->max_z = -std::numeric_limits<double>::max();

        if(buffer_.size() == this->buffer.size()){
            this->buffer = buffer_;
            this->displayBuffer = std::vector<Color>(this->width * this->height * this->depth, {0,0,0,0});
        }
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

    bool displayViewer();
    bool updateBuffer();
    void addToBuffer(const Vec3D pos, Color c);
    Eigen::Vector3f pixelToWorld(const double u, const double v);
    Eigen::Vector3f objectToWorld(const double u, const double v, const double w);
    
    private:
    void renderBuffer();
    void detectInteraction(INTERACTION* inter, double* value);
    void updateCamera(INTERACTION* inter, double* value);
    void updateDisplayBuffer();
    void initBuffer();

};

inline void createRandomBuffer(std::vector<std::pair<Vec3D, Color>>& buffer, const size_t width, const size_t height, const Color defaultColor = {0,0,0,0}){
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
