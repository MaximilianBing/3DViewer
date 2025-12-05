#include <SDL2/SDL.h>
#include <SDL2/SDL_events.h>
#include <SDL2/SDL_keyboard.h>
#include <3DViewer.hpp>
#include <spdlog/spdlog.h>
#include <iostream>

void detectMouseClick(){
    SDL_Init(SDL_INIT_VIDEO);
    SDL_Window* window = SDL_CreateWindow(
        "Test",
        SDL_WINDOWPOS_CENTERED,
        SDL_WINDOWPOS_CENTERED,
        800,
        600,
        SDL_WINDOW_SHOWN
    );

    SDL_Event event;
    bool running = true;
    while(running){
        while(SDL_PollEvent(&event)){
            if(event.type == SDL_MOUSEMOTION){
                spdlog::info("Current mouse position is: ({}, {}), state:{}", event.motion.x, event.motion.y, event.motion.state);
            }
            else if(event.type == SDL_MOUSEBUTTONDOWN){
                spdlog::info("Mouse pressed at x:{}, y:{}, button: {}", event.motion.x, event.motion.y, event.button.button);
            }
            else if(event.type == SDL_MOUSEWHEEL){
                spdlog::info("Mouse wheel motion detected: up: {} x:{}, y:{}", event.wheel.direction, event.wheel.preciseY, event.wheel.preciseX);
            }
            else if(event.type == SDL_TEXTINPUT){
                spdlog::info("Keyboard Event: {}", event.text.text);
            }
            else if(event.type == SDL_KEYDOWN){
                if(event.key.keysym.sym == SDLK_UP){
                    spdlog::info("Up pressed");
                }
                else if(event.key.keysym.sym == SDLK_DOWN){
                    spdlog::info("Down pressed");
                }
                else if(event.key.keysym.sym == SDLK_LEFT){
                    spdlog::info("Left pressed");
                }
                else if(event.key.keysym.sym == SDLK_RIGHT){
                    spdlog::info("Right pressed");
                }
            }
            
            else if(event.type == SDL_QUIT){
                running = false;
            }
        }
        SDL_Delay(50);
    }
    SDL_DestroyWindow(window);
    SDL_Quit();
}
