#ifndef CONSTANTS_H
#define CONSTANTS_H
#include <SDL2/SDL.h>

const int SCREEN_WIDTH = 640;
const int SCREEN_HEIGHT = 480;

extern float gravity;

const float restitution = 0.5;
const float friction_coefficient = 0.5;

const float VELOCITY_THRESHOLD = 5.f;
const float ANGULAR_THRESHOLD = 0.1f;
const float SLEEP_THRESHOLD = 1.f;

extern SDL_Window* gWindow;
extern SDL_Renderer* gRenderer;


#endif