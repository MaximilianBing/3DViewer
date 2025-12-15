#include <SDL2/SDL.h>
#include <SDL2/SDL_events.h>
#include <SDL2/SDL_keyboard.h>
#include <3DViewer.hpp>
#include <spdlog/spdlog.h>
#include <iostream>


void ImageViewer::initBuffer(){
  for (int i = 0; i < this->buffer.size(); i++){
    Vec3D coord = this->buffer[i].first;
    if(coord.x > this->max_x) this->max_x = coord.x;    
    if(coord.x < this->min_x) this->min_x = coord.x;    
    if(coord.y > this->max_y) this->max_y = coord.y;    
    if(coord.y < this->min_y) this->min_y = coord.y;    
    if(coord.z > this->max_z) this->max_z = coord.z;    
    if(coord.z < this->min_z) this->min_z = coord.z;        

    int xCoord = static_cast<int>(coord.x);
    int yCoord = static_cast<int>(coord.y);
    int zCoord = static_cast<int>(coord.z);
    int idx = zCoord * this->width * this->height + yCoord * this->width + xCoord;
    if(xCoord >= 0 && xCoord < this->width && yCoord >= 0 && yCoord < this->height){
      Color c = this->buffer[i].second;
      this->displayBuffer[idx] = c;
    }
  }

  spdlog::info("x: min={} max={}, y: min={} max={}, z: min={} max={}", this->min_x, this->max_x, this->min_y, this->max_y, this->min_z, this->max_z);
}

bool ImageViewer::updateBuffer(){
  this->renderBuffer();
  return true;
}

void ImageViewer::renderBuffer(){
    // Setup renderer
    SDL_Renderer* renderer = NULL;
    renderer =  SDL_CreateRenderer( this->p_window, -1, SDL_RENDERER_ACCELERATED);
    
    // background color
    // // Set render color to red ( background will be rendered in this color )
    SDL_SetRenderDrawColor( renderer, 0, 0, 0, 255 );

    // // Clear winow
    SDL_RenderClear( renderer );

    for(int idx = 0; idx < this->displayBuffer.size(); idx++){
        SDL_Color color = this->displayBuffer[idx];
        Eigen::Vector3i v = this->transform1Dto3D(idx);
        SDL_Rect r;
        r.x = v.x();
        r.y = v.y();
        r.w = r.h = 1;

        SDL_SetRenderDrawColor(renderer, color.r, color.g, color.b, color.a);
        SDL_RenderFillRect(renderer, &r);
    }


    // // Creat a rect at pos ( 50, 50 ) that's 50 pixels wide and 50 pixels high.
    // SDL_Rect r;
    // r.x = 50;
    // r.y = 50;
    // r.w = 50;
    // r.h = 50;

    // // Set render color to blue ( rect will be rendered in this color )
    // SDL_SetRenderDrawColor( renderer, 0, 0, 255, 255 );

    // // Render rect
    // SDL_RenderFillRect( renderer, &r );

    // Render the rect to the screen
    SDL_RenderPresent(renderer);
    SDL_DestroyRenderer(renderer);
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
      break;
    case INTERACTION::TRANSLATE_VERTICAL:
      this->cam.position.y += *value;
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
  double m = -0.5 / (this->min_x - cx);

  objectToWorldMatrix(0,0) = m;
  objectToWorldMatrix(0,3) = - m * cx;
  objectToWorldMatrix(1,1) = m;
  objectToWorldMatrix(1,3) = - m * cy;
  objectToWorldMatrix(2,2) = m;
  objectToWorldMatrix(2,3) = - m * cz;

  Eigen::Vector4f worldVec = objectToWorldMatrix * objectVec;
  spdlog::info("Input: ({}, {}, {}), Output: ({},{}, {})", u, v, w, worldVec.x(), worldVec.y(), worldVec.z());
  
  return worldVec.block<3, 1>(0, 0);
}

Eigen::Vector3f ImageViewer::pixelToWorld(const double u, const double v){
  Eigen::Vector4f pixelVec(u, v, 1, 1);
  Eigen::Matrix4f pixelToImagePlane = Eigen::Matrix4f::Identity();
  Eigen::Matrix4f imagePlaneToCamera = Eigen::Matrix4f::Identity();
  Eigen::Matrix4f cameraToWorld = Eigen::Matrix4f::Identity();

  pixelToImagePlane(0,0) = this->s / this->f;
  pixelToImagePlane(0,2) = - this->s / this->f * this->cx;
  pixelToImagePlane(1,1) = - this->s / this->f;
  pixelToImagePlane(1,2) = this->s / this->f * this->cy;
  pixelToImagePlane(2,2) = 0;

  Eigen::Vector4f test = pixelToImagePlane * pixelVec;

  spdlog::info("Test Input: ({}, {}), Output: ({},{}, {})", u, v, test.x(), test.y(), test.z());


  imagePlaneToCamera(2,3) = this->f;
  
  // Eigen::Vector3f up(this->cam.up.x, this->cam.up.y, this->cam.up.z);
  // Eigen::Vector3f dir(this->cam.orientation.x, this->cam.orientation.y, this->cam.orientation.z);
  // Eigen::Vector3f right = up.cross(dir);
  // cameraToWorld(0,0) = right.x();
  // cameraToWorld(0,1) = right.y();
  // cameraToWorld(0,2) = right.z();
  cameraToWorld(0,3) = this->cam.position.x;
  // cameraToWorld(1,0) = up.x();
  // cameraToWorld(1,1) = up.y();
  // cameraToWorld(1,2) = up.z();
  cameraToWorld(1,3) = this->cam.position.y;
  // cameraToWorld(2,0) = dir.x();
  // cameraToWorld(2,1) = dir.y();
  // cameraToWorld(2,2) = dir.z();
  cameraToWorld(2,3) = -this->cam.position.z;
  
  // spdlog::info("Camera Position: ({}, {}, {}) Orientation: ({}, {}, {})", this->cam.position.x, this->cam.position.y, this->cam.position.z, this->cam.orientation.x, this->cam.orientation.y, this->cam.orientation.z);


  Eigen::Vector4f worldVec = cameraToWorld * imagePlaneToCamera * pixelToImagePlane * pixelVec;
  spdlog::info("Input: ({}, {}), Output: ({},{}, {})", u, v, worldVec.x(), worldVec.y(), worldVec.z());

  return worldVec.block<3,1>(0,0);
  }

void ImageViewer::updateDisplayBuffer(){
  return;
}

bool ImageViewer::displayViewer(){
  if(this->running == false){
    this->initBuffer();
    SDL_Init(SDL_INIT_VIDEO);
    this->p_window = SDL_CreateWindow(this->windowTitle.c_str(), SDL_WINDOWPOS_CENTERED,
                                  SDL_WINDOWPOS_CENTERED, this->width,
                                  this->height, SDL_WINDOW_SHOWN);
    this->p_event = new SDL_Event();
    this->running = true;
  }  

  while (SDL_PollEvent(this->p_event) && this->running) {
    INTERACTION inter;
    double value;
    this->detectInteraction(&inter, &value);
    if(inter != INTERACTION::NON_TYPE && inter != INTERACTION::QUIT){
      this->updateCamera(&inter, &value);
      this->updateDisplayBuffer();
    }
    else if(inter == INTERACTION::QUIT){
      SDL_DestroyWindow(this->p_window);
      SDL_Quit();
      delete this->p_event;
      this->running = false;
      return false;
    }
  }
    return true;
}
