#ifndef RENDER_H
#define RENDER_H
#include <SDL2/SDL.h>

bool init(SDL_Renderer*& gRenderer, SDL_Window*& gWindow);

bool loadMedia();

void close(SDL_Renderer*& gRenderer, SDL_Window*& gWindow);

#endif