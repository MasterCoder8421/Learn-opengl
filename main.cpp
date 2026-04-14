#include <SDL2/SDL.h>
#include "RigidBody.h"
#include "Utils.h"
#include "Render.h"
#include "Collision.h"
#include <stdio.h>
#include <iostream>
#include "constants.h"
#include <time.h>


class PhysicsEngine{
    public:
    std::vector<RigidBody*> bodies;
    std::vector<Vector2> collisionPoints;

    void addBody(RigidBody* body){
        bodies.push_back(body);
    }

    void updateVelocity(float dt){
        for (RigidBody* body: bodies){
            if (!body->isSleeping){
                body->updateVelocity(dt);
            }
        }
    }

    void updatePosition(float dt){
        for (RigidBody* body: bodies){
            if (!body->isSleeping){
                body->updatePosition(dt);
            }
        }
    }

    void checkSleeping(float dt){
        for (RigidBody* body: bodies){
            body->checkSleeping(dt);
        }
    }

    void show(SDL_Renderer* gRenderer){
        for (RigidBody* body: bodies){
            body->show(gRenderer);
        }
    }

    void collisionHandling(bool& update){
        for (int i = 0; i < bodies.size(); i++){
            for (int j = 0; j < i; j++){
                RigidBody* body1 = bodies[i];
                RigidBody* body2 = bodies[j];
                if (body1->shape.invmass == 0 and body2->shape.invmass == 0) continue;
                CollisionData* colData = new CollisionData;
                bool isCollding = isCollidingSat(*body1, *body2, colData);
                if (isCollding){
                    body1->isSleeping = false;
                    body2->isSleeping = false;
                    std::vector<Contact> manifold;
                    getCollisionManifold(*body1, *body2, colData, manifold);
                    // printf("Before Collision: %f %f\n", TotalKineticEnergy(), body2->linearVelocity.y);
                    // printf("%f\n", body2->linearVelocity.x);
                    handleCollision(*body1, *body2, manifold);
                    // printf("%f\n", body2->linearVelocity.x);
                    // update = false;
                }
                delete colData;
            }
        }
    }

    void drawCollisionPoints(SDL_Renderer* renderer, const std::vector<Vector2>& points) {
        SDL_SetRenderDrawColor(renderer, 0, 255, 0, 255);
        for (const Vector2& p : points) {
            // Simple way: draw a 5x5 filled square centered at collision point
            SDL_Rect rect;
            rect.x = (int)(p.x - 10);
            rect.y = (int)(p.y - 10);
            rect.w = 20;
            rect.h = 20;
            SDL_RenderFillRect(renderer, &rect);
        }
    }

    void clearForceAndTorque(){
        for (RigidBody* body: bodies){
            body->clearForceAndTorque();
        }
    }

    float TotalKineticEnergy(){
        float ans = 0;
        for (RigidBody* body: bodies){
            ans += kineticEnergy(*body);
        }
        return ans;
    }

    void clearSleeping(){
        for (RigidBody* body: bodies){
            body->isSleeping = false;
        }
    }
};

int main(int argc, char* args[])
{
    srand( (unsigned)time(NULL) );
    if( !init(gRenderer, gWindow) )
    {
        printf( "Failed to initialize!\n" );
    }
    else
    {
        if( !loadMedia() )
        {
            printf( "Failed to load media!\n" );
        }
        else
        {
            SDL_Event e; 
            bool quit = false; 
            PhysicsEngine physicsEngine;
            RigidBody body1 = {200, 300, 50, 50, 0.04};
            RigidBody body2 = {200, 200, 50, 50, 0.04};
            RigidBody floor = {320, 430, 640, 100, 0};
            RigidBody ceil = {320, 0, 640, 100, 0};
            RigidBody wall1 = {600, 240, 100, 480, 0};
            RigidBody wall2 = {0, 240, 100, 480, 0};

            body1.name = "body1";
            body2.name = "body2";

            int gridCols = 4;
            int gridRows = 5;
            float cellWidth = 80;
            float cellHeight = 80;


            // for (int row = 0; row < gridRows; row++) {
            //     for (int col = 0; col < gridCols; col++) {
            //         float x = col * cellWidth + cellWidth / 2 + 100 + row * 10;
            //         float y = row * cellHeight + cellHeight / 2;

            //         RigidBody* gridBody = new RigidBody{x, y, cellWidth*0.4f, cellHeight*0.4f, 0.04};

            //         physicsEngine.addBody(gridBody);
            //     }
            // }
            physicsEngine.addBody(&body1);
            physicsEngine.addBody(&body2);
            physicsEngine.addBody(&floor);
            physicsEngine.addBody(&ceil);
            physicsEngine.addBody(&wall1);
            physicsEngine.addBody(&wall2);


            const double targetFPS = 200.0;
            const double targetFrameTime = 1.0 / targetFPS; // ~0.01667 sec
            const Uint64 perfFreq = SDL_GetPerformanceFrequency();

            Uint64 lastCounter = SDL_GetPerformanceCounter();
            double accumulator = 0.0;

            double avgFrameTime, printAccumulator;
            double fps;

            bool update = true;
            const float deltaTime = 0.01;


            while( quit == false ){ 
                while( SDL_PollEvent( &e ) != 0){
                    if( e.type == SDL_QUIT ) {
                        quit = true; 
                    }
                    if (e.type == SDL_KEYDOWN){
                        gravity *= -1;
                        physicsEngine.clearSleeping();
                    }
                }

                Uint64 currentCounter = SDL_GetPerformanceCounter();
                double change = (double)(currentCounter - lastCounter) / perfFreq;
                lastCounter = currentCounter;

                accumulator += change;
                

                if (update){
                    SDL_SetRenderDrawColor( gRenderer, 255, 255, 255, 255 );
                    SDL_RenderClear(gRenderer);
                    int subSteps = 1;
                    physicsEngine.show(gRenderer);
                    physicsEngine.drawCollisionPoints(gRenderer, physicsEngine.collisionPoints);
                    accumulator = std::min(accumulator, 1.);
                    while (accumulator>0){
                        for (int i = 0; i<subSteps; i++){
                            for (RigidBody* body: physicsEngine.bodies){
                                if (body->shape.invmass > 0){
                                    body->applyForceAndTorque({0, gravity*1/body->shape.invmass}, body->position);
                                }
                            }
                            physicsEngine.updateVelocity(deltaTime/subSteps);
                            physicsEngine.updatePosition(deltaTime/subSteps);
                            for (int i = 0; i<5; i++){
                                physicsEngine.collisionHandling(update);
                            }
                            // physicsEngine.checkSleeping(deltaTime/subSteps);
                            physicsEngine.clearForceAndTorque();
                        }
                        accumulator-=deltaTime;
                    }
                    SDL_RenderPresent( gRenderer );
                }
                avgFrameTime = avgFrameTime * 0.9 + change * 0.1;
                fps = 1.0 / avgFrameTime;

                printAccumulator += deltaTime;
                if (printAccumulator >= 1.0) {
                    printf("FPS: %.2f\n", fps);
                    printAccumulator = 0.0;
                }

                Uint64 frameEnd = SDL_GetPerformanceCounter();
                double frameDuration = (double)(frameEnd - currentCounter) / perfFreq;
                if (frameDuration < targetFrameTime) {
                    double remaining = targetFrameTime - frameDuration;
                    SDL_Delay((Uint32)(remaining * 1000.0)); // convert sec → ms
                }
                
            }
            printf("Energy After Collision %f\n", physicsEngine.TotalKineticEnergy());
        }
    }
    
    close(gRenderer, gWindow);
    
    return 0;
}