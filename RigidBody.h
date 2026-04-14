#ifndef RIGIDBODY_H
#define RIGIDBODY_H
#include <vector>
#include "Utils.h"
#include <SDL2/SDL.h>


class RigidBody{
    public:
    Vector2 position;
    Vector2 linearVelocity;
    float angle;
    float angularVelocity;
    Vector2 force;
    float torque;
    BoxShape shape;
    float sleepTime;
    bool isSleeping;
    Vector2 linearVelocityExponential;
    float angularVelocityExponential;
    std::string name;

    RigidBody(float x, float y, float w, float h, float invmass);

    void applyForceAndTorque(Vector2 appliedForce, Vector2 worldLocation);

    void clearForceAndTorque();

    void updateVelocity(float dt);

    void updatePosition(float dt);

    void generateVertices(float vx[4], float vy[4]);

    void show(SDL_Renderer* gRenderer);

    void getAxes(std::vector<Vector2> &axes);

    void getMinMaxVerticesOnAxis(Vector2 axis, Vector2& min, Vector2& max);

    Vector2 getClosestPoint(Vector2 point);

    void GetIncidentReferencePolygon(Vector2 collision_normal, std::vector<Vector2>& polygon, Vector2& normal, std::vector<Plane>& adjPlanes);

    void checkSleeping(float dt);
};

#endif