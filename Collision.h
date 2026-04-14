#ifndef COLLISION_H
#define COLLISION_H
#include "Utils.h"
#include "RigidBody.h"


bool isCollidingSat(RigidBody shape1, RigidBody shape2, CollisionData *coldata);

float calculateImpulse(RigidBody shape1, RigidBody shape2, Contact contact);

void handleCollision(RigidBody &shape1, RigidBody &shape2, std::vector<Contact>& contacts);

void getCollisionManifold(RigidBody &shape1, RigidBody &shape2, CollisionData *coldata, std::vector<Contact>& out_manifold);

#endif