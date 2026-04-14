#include "Utils.h"
#include "RigidBody.h"
#include <vector>
#include <SDL2/SDL.h>
#include <SDL2/SDL2_gfxPrimitives.h>
#include <stdio.h>
#include "constants.h"


RigidBody::RigidBody(float x, float y, float w, float h, float invmass){
    position = (Vector2){x, y};
    shape = {w, h, invmass, (12.f * invmass) / ((w * w + h * h)) };
    angle = 0;
    angularVelocity = 0;
    force = {0, 0};
    torque = 0;
    linearVelocity = (Vector2){0, 0};
    sleepTime = 0.f;
    isSleeping = false;
    linearVelocityExponential = (Vector2){0, 0};
    angularVelocityExponential = 0;
    name = "";
}
void RigidBody::applyForceAndTorque(Vector2 appliedForce, Vector2 worldLocation){
    if (isSleeping) {
        return;
    }
    force.x += appliedForce.x;
    force.y += appliedForce.y;

    Vector2 localLocation = {worldLocation.x - position.x, worldLocation.y - position.y};
    torque += localLocation.x * appliedForce.y - localLocation.y * appliedForce.x;
}

void RigidBody::clearForceAndTorque(){
    force.x = 0;
    force.y = 0;
    torque = 0;
}

void RigidBody::updateVelocity(float dt){
    Vector2 linearAcceleration = (Vector2){force.x * shape.invmass, force.y * shape.invmass};
    linearVelocity.x += linearAcceleration.x * dt;
    linearVelocity.y += linearAcceleration.y * dt;
    float angularAcceleration = torque * shape.invMomentOfInertia;
    angularVelocity += angularAcceleration * dt;
    // linearVelocity = snap(linearVelocity, 0.01f);
    // angularVelocity = snap(angularVelocity, 0.01f);
}

void RigidBody::updatePosition(float dt){
    angularVelocityExponential = angularVelocityExponential * 0.9 + angularVelocity * 0.1;
    linearVelocityExponential = linearVelocityExponential * 0.9 + linearVelocity * 0.1;
    position.x += linearVelocity.x * dt;
    position.y += linearVelocity.y * dt;
    angle += angularVelocity * dt;
    position = snap(position, 0.01f);
    angle = snap(angle, 0.01f);
}

void RigidBody::generateVertices(float vx[4], float vy[4]){
    float halfW = shape.width / 2;
    float halfH = shape.height / 2;

    float vertices[4][2] = {
        {-halfW, -halfH},
        { halfW, -halfH}, 
        { halfW,  halfH},
        {-halfW,  halfH}
    };
    for (int i = 0; i < 4; i++) {
        float x = vertices[i][0];
        float y = vertices[i][1];
        float xRot = x * cos(angle) - y * sin(angle);
        float yRot = x * sin(angle) + y * cos(angle);
        vx[i] = position.x + xRot;
        vy[i] = position.y + yRot;
    }
}



void RigidBody::show(SDL_Renderer* gRenderer){
    float fx[4];
    float fy[4];
    generateVertices(fx, fy);
    Sint16 ix[4];
    Sint16 iy[4];
    for (int i = 0; i < 4; i++) {
        ix[i] = (Sint16)(fx[i] + 0.5f);
        iy[i] = (Sint16)(fy[i] + 0.5f);
    }
    filledPolygonRGBA(gRenderer, ix, iy, 4, 255, isSleeping ? 255 : 0, 0, 255);
    for (int i = 0; i<4; i++){
        int ni = (i+1) %4;
        for (int j=0; j<2; j++){
            aalineRGBA(gRenderer, ix[i]+j, iy[i]+j, ix[ni]+j, iy[ni]+j, 0, 0, 0, 255);
        }
    }
}

void RigidBody::getAxes(std::vector<Vector2> &axes){
    float vx[4];
    float vy[4];
    generateVertices(vx, vy);
    for (int i = 0; i<4; i++){
        float x1 = vx[i];
        float x2 = vx[(i+1)%4];
        float y1 = vy[i];
        float y2 = vy[(i+1)%4];

        Vector2 axis = {(float)y1-y2, (float)x2-x1};
        float length =  sqrt(axis.x * axis.x + axis.y * axis.y);
        axes.push_back({axis.x/length, axis.y/length});
    }

};

void RigidBody::getMinMaxVerticesOnAxis(Vector2 axis, Vector2& min, Vector2& max){
    float vx[4];
    float vy[4];
    generateVertices(vx, vy);
    float minimum = FLT_MAX;
    float maximum = -FLT_MAX;
    for (int i = 0; i<4; i++){
        float proj = project({vx[i], vy[i]}, axis);
        if (proj <= minimum){
            minimum = proj;
            min = {vx[i], vy[i]};
        }
        if (proj >= maximum){
            maximum = proj;
            max = {vx[i], vy[i]};
        }
    }
}

void RigidBody::GetIncidentReferencePolygon(Vector2 collision_normal, std::vector<Vector2>& polygon, Vector2& normal, std::vector<Plane>& adjPlanes){
    float vx[4];
    float vy[4];
    float best_alignment = -FLT_MAX;
    int best_vertex = -1;
    generateVertices(vx, vy);
    for (int i = 0; i<4; i++){
        float dist = dot((Vector2){vx[i], vy[i]}, collision_normal);
        if (dist > best_alignment){
            best_alignment = dist;
            best_vertex = i;
        }
    }



    float best_match = -FLT_MAX;
    std::pair<int, int> edge;

    for (int i = 0; i<4; i++){
        int next = (i + 1) % 4;
        if (i == best_vertex or (next == best_vertex)){
            Vector2 normal_temp = (Vector2{vx[i], vy[i]} - Vector2{vx[next], vy[next]}).perpendicular().normalized();
            float match = dot(normal_temp, collision_normal);
            if (match > best_match){
                best_match = match;
                edge = {i, next};
                normal = normal_temp;
            }
        }
    }
    polygon.push_back((Vector2){vx[edge.first], vy[edge.first]});
    polygon.push_back((Vector2){vx[edge.second], vy[edge.second]});

    for (int i = 0; i<4; i++){
        int next = (i + 1) % 4;
        if (i == edge.second or next == edge.first){
            Vector2 normal_temp = -(Vector2{vx[i], vy[i]} - Vector2{vx[next], vy[next]}).perpendicular().normalized();

            Plane clippingPlane;
            clippingPlane.normal = normal_temp;
            clippingPlane.distance = dot(Vector2{vx[i], vy[i]}, clippingPlane.normal);

            adjPlanes.push_back(clippingPlane);
        }
    }
}

void RigidBody::checkSleeping(float dt){
    if (linearVelocity.length()<VELOCITY_THRESHOLD
    && abs(angularVelocityExponential) < ANGULAR_THRESHOLD){
        sleepTime+=dt;
        if (sleepTime>SLEEP_THRESHOLD){
            isSleeping = true;
            linearVelocity = {0, 0};
            angularVelocity = 0;
        }
    } else{
        isSleeping = false;
        sleepTime = 0.0f;
    }
}