#include <SDL2/SDL.h>
#include <SDL2/SDL_events.h>
#include <SDL2/SDL_keyboard.h>
#include <3DViewer.hpp>
#include <spdlog/spdlog.h>
#include <iostream>
#include <algorithm>

bool ImageViewer::init(){
  SDL_Init(SDL_INIT_VIDEO);
  this->p_window = SDL_CreateWindow(
      this->windowTitle.c_str(), SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED,
      this->width, this->height, SDL_WINDOW_SHOWN);
  this->p_event = new SDL_Event();
  this->p_renderer =
      SDL_CreateRenderer(this->p_window, -1, SDL_RENDERER_ACCELERATED);
  this->p_texture =
      SDL_CreateTexture(this->p_renderer, SDL_PIXELFORMAT_RGBA8888,
                        SDL_TEXTUREACCESS_STREAMING, this->width, this->height);
  this->initBuffer();
  this->running = true;
  this->cameraChanged = false;
  return true;
}

void ImageViewer::initBuffer(){
  void* pixels;
  int pitch;

  SDL_LockTexture(this->p_texture, nullptr, &pixels, &pitch);
  Uint32* dst = static_cast<Uint32*>(pixels);
  int stride = pitch / sizeof(Uint32);

  std::fill(dst, dst + stride * height, 0);

  SDL_PixelFormat* fmt = SDL_AllocFormat(SDL_PIXELFORMAT_RGBA8888);

  for (const auto& entry : buffer) {
    const Vec3D& pos = entry.first;
    const SDL_Color& c = entry.second;

    if(pos.x > this->max_x) this->max_x = pos.x;    
    if(pos.x < this->min_x) this->min_x = pos.x;    
    if(pos.y > this->max_y) this->max_y = pos.y;    
    if(pos.y < this->min_y) this->min_y = pos.y;    
    if(pos.z > this->max_z) this->max_z = pos.z;    
    if(pos.z < this->min_z) this->min_z = pos.z;  

    int x = static_cast<int>(pos.x);
    int y = static_cast<int>(pos.y);

    if (x < 0 || x >= width || y < 0 || y >= height)
        continue;

    Uint32 pixel = SDL_MapRGBA(fmt, c.r, c.g, c.b, c.a);
    dst[y * stride + x] = pixel;
  }

  SDL_FreeFormat(fmt);
  SDL_UnlockTexture(this->p_texture);

  spdlog::info("x: min={} max={}, y: min={} max={}, z: min={} max={}", this->min_x, this->max_x, this->min_y, this->max_y, this->min_z, this->max_z);
}

bool ImageViewer::isRunning(){
  return this->running;
}

void ImageViewer::handleEvents(){
   while (SDL_PollEvent(this->p_event) && this->running) {
    INTERACTION inter;
    double value;
    this->detectInteraction(&inter, &value);
    if(inter != INTERACTION::NON_TYPE && inter != INTERACTION::QUIT){
      this->updateCamera(&inter, &value);
      this->cameraChanged = true;
    }
    else if(inter == INTERACTION::QUIT){
      this->shutdown();
    }
  }
}

void ImageViewer::update(){
  if (this->cameraChanged) {
    spdlog::info("Changing camera...");
    updateDisplayBuffer();
    this->cameraChanged = false;
    spdlog::info("Camera changed");
  }
}

void ImageViewer::render(){    
    SDL_RenderClear(p_renderer);
    SDL_RenderCopy(p_renderer, p_texture, nullptr, nullptr);
    SDL_RenderPresent(p_renderer);
}

void ImageViewer::shutdown() {
    SDL_DestroyTexture(p_texture);
    SDL_DestroyRenderer(p_renderer);
    SDL_DestroyWindow(p_window);
    delete p_event;
    SDL_Quit();
    this->running = false;
    this->cameraChanged = false;
}

void ImageViewer::detectInteraction(INTERACTION* inter, double* value){
  *inter = INTERACTION::NON_TYPE;
  // translation 
  if (this->p_event->type == SDL_KEYDOWN) {
    switch (this->p_event->key.keysym.sym){
      case SDLK_UP: 
        *inter = INTERACTION::TRANSLATE_VERTICAL;
        *value = vTranslate;
        break;
      case SDLK_DOWN: 
        *inter = INTERACTION::TRANSLATE_VERTICAL;
        *value = -vTranslate;
        break;
      case SDLK_LEFT: 
        *inter = INTERACTION::TRANSLATE_HORIZONTAL;
        *value = -vTranslate;
        break;
      case SDLK_RIGHT: 
        *inter = INTERACTION::TRANSLATE_HORIZONTAL;
        *value = vTranslate;
        break;
      case SDLK_SPACE: 
        *inter = INTERACTION::QUIT;
        *value = -1;
        break;
      default:
        break;
    }
  }
  // Scaling
  else if (this->p_event->type == SDL_MOUSEWHEEL) {
    *inter = INTERACTION::SCALE_IMAGE;
    *value = (this->p_event->wheel.preciseY > 0) ? 1.0 / vScale : vScale;
  } 
  // Rotation
  // if (this->p_event->type == SDL_MOUSEMOTION) {
  //   spdlog::info("Current mouse position is: ({}, {}), state:{}",
  //                 this->p_event->motion.x, this->p_event->motion.y, this->p_event->motion.state);
  // } 
  // else if (this->p_event->type == SDL_MOUSEBUTTONDOWN) {
  //   spdlog::info("Mouse pressed at x:{}, y:{}, button: {}", this->p_event->motion.x,
  //                 this->p_event->motion.y, this->p_event->button.button);
  // } 
  // else if (this->p_event->type == SDL_TEXTINPUT) {
  //   spdlog::info("Keyboard this->p_Event-> {}", this->p_event->text.text);
  // }
  // Quit
  else if (this->p_event->type == SDL_QUIT) {
    spdlog::info("QUIT");
    *inter = INTERACTION::QUIT;
    *value = -1.0;
  }
}

void ImageViewer::updateCamera(INTERACTION* inter, double* value){
  switch (*inter){
    case INTERACTION::TRANSLATE_HORIZONTAL:
      this->cam.position.x += *value;
      // this->sortBuffer();
      break;
    case INTERACTION::TRANSLATE_VERTICAL:
      this->cam.position.y += *value;
      // this->sortBuffer();
      break;
    case INTERACTION::SCALE_IMAGE:
      this->f *= *value;
      break;
    default:
      break;
  }
  spdlog::info("Camera Position: x = {}, y = {}, z = {}, focal: {}", this->cam.position.x, this->cam.position.y, this->cam.position.z, this->f);
}

Eigen::Vector3f ImageViewer::objectToWorld(const double u, const double v, const double w){
  Eigen::Vector4f objectVec(u,v,w,1);
  Eigen::Matrix4f objectToWorldMatrix = Eigen::Matrix4f::Identity();

  double cx = (this->max_x - this->min_x) / 2.0;
  double cy = (this->max_y - this->min_y) / 2.0;
  double cz = (this->max_z - this->min_z) / 2.0;
  double m = -1.0 / (this->min_x - cx);

  objectToWorldMatrix(0,0) = m;
  objectToWorldMatrix(0,3) = - m * cx;
  objectToWorldMatrix(1,1) = m;
  objectToWorldMatrix(1,3) = - m * cy;
  objectToWorldMatrix(2,2) = m;
  objectToWorldMatrix(2,3) = - m * cz;

  return (objectToWorldMatrix * objectVec).block<3, 1>(0, 0);
}

Eigen::Vector3f ImageViewer::pixelToWorld(const double u, const double v){
  Eigen::Vector4f pixelVec(u, v, 0, 1);
  Eigen::Matrix4f pixelToImagePlane = Eigen::Matrix4f::Identity();
  Eigen::Matrix4f imagePlaneToCamera = Eigen::Matrix4f::Identity();
  Eigen::Matrix4f cameraToWorld = Eigen::Matrix4f::Identity();

  pixelToImagePlane(0,0) = 2 * f / (this->width - 1);
  pixelToImagePlane(0,3) = -f;
  pixelToImagePlane(1,1) = - 2 * f / (this->height - 1);
  pixelToImagePlane(1,3) = f;

  imagePlaneToCamera(2,3) = this->f;

  Eigen::Vector3f up(this->cam.up.x, this->cam.up.y, this->cam.up.z);
  Eigen::Vector3f dir(this->cam.orientation.x, this->cam.orientation.y, this->cam.orientation.z);
  Eigen::Vector3f right = dir.cross(up);
  cameraToWorld(0,0) = right.x();
  cameraToWorld(0,1) = right.y();
  cameraToWorld(0,2) = right.z();
  cameraToWorld(1,1) = up.x();
  cameraToWorld(1,1) = up.y();
  cameraToWorld(1,2) = up.z();
  cameraToWorld(2,0) = dir.x();
  cameraToWorld(2,1) = dir.y();
  cameraToWorld(2,2) = dir.z();
  cameraToWorld(0,3) = this->cam.position.x;
  cameraToWorld(1,3) = this->cam.position.y;
  cameraToWorld(2,3) = this->cam.position.z;

  Eigen::Vector4f worldVec = cameraToWorld * imagePlaneToCamera * pixelToImagePlane * pixelVec;

  return (cameraToWorld * imagePlaneToCamera * pixelToImagePlane * pixelVec).block<3,1>(0,0);
  }

std::vector<std::pair<Vec3D, Color>> ImageViewer::calculateRayIntersection(int px, int py){
  Eigen::Vector3d o = this->pixelToWorld(px, py).cast<double>();
  Eigen::Vector3d d(o.x() - this->cam.position.x, o.y() - this->cam.position.y, o.z() - this->cam.position.z);
  d.normalize();

  
  double eps = 0.01;
  
  std::vector<std::pair<Vec3D, Color>> intersectors;
  
  for(auto& pair : this->buffer){
    Eigen::Vector3d p = this->objectToWorld(pair.first.x, pair.first.y, pair.first.z).cast<double>();
    
    Eigen::Vector3d op = o - p;
    Eigen::Vector3d po = p - o;
    double r = (op + d.dot(po) / d.dot(d) * d).norm();
    
    if(r < eps){
      // spdlog::info("Intersector: ({}, {}, {})", pair.first.x,  pair.first.y, pair.first.z);
      intersectors.push_back(pair);
    }
  }

  return intersectors;
}

void ImageViewer::sortBuffer(){
  const auto& cam = this->cam;
  std::sort(buffer.begin(), buffer.end(), [&cam](const std::pair<Vec3D, Color>& a, const std::pair<Vec3D, Color>& b){
    double a_dx = a.first.x - cam.position.x;
    double a_dy = a.first.y - cam.position.y;
    double a_dz = a.first.z - cam.position.z;
    
    double b_dx = b.first.x - cam.position.x;
    double b_dy = b.first.y - cam.position.y;
    double b_dz = b.first.z - cam.position.z;
    
    double d1  = a_dx * a_dx + a_dy * a_dy + a_dz * a_dz;
    double d2  = b_dx * b_dx + b_dy * b_dy + b_dz * b_dz;
    return d1 < d2;
  });
}

void ImageViewer::updateDisplayBuffer(){
  void* pixels = nullptr;
  int pitch = 0;

  if (SDL_LockTexture(p_texture, nullptr, &pixels, &pitch) != 0) {
      return;
  }

  Uint32* dst = static_cast<Uint32*>(pixels);
  int stride = pitch / sizeof(Uint32);

  SDL_PixelFormat* fmt = SDL_AllocFormat(SDL_PIXELFORMAT_RGBA8888);

  // Clear framebuffer
  std::fill(dst, dst + stride * height, 0);

  const int pixelCount = width * height;

  for (int idx = 0; idx < pixelCount; ++idx) {
      int x = idx % width;
      int y = idx / width;

      // 1. Build ray from camera through this pixel
      Eigen::Vector3i v = this->transform1Dto3D(idx);
      auto intersectors = calculateRayIntersection(v.x(), v.y());

      if (intersectors.size() == 0) {
          continue; // background remains black
      }

      // 3. Write pixel
      dst[y * stride + x] = SDL_MapRGBA(
          fmt,
          intersectors[0].second.r,
          intersectors[0].second.g,
          intersectors[0].second.b,
          intersectors[0].second.a
      );
  }

  SDL_FreeFormat(fmt);
  SDL_UnlockTexture(p_texture);
}
