#include "Utils.h"
#include "RigidBody.h"
#include <vector>
#include "constants.h"
#include <iostream>
#include <algorithm>
#include <time.h>

float calculateEffectiveMass(const RigidBody& shape1, const RigidBody& shape2, const Contact& contact){
     Vector2 n = contact.normal;
    Vector2 ra = contact.contactPoint - shape1.position;
    Vector2 rb = contact.contactPoint - shape2.position;

    float effectiveMass = (shape1.shape.invmass + shape2.shape.invmass) + (cross(ra, n)*cross(ra, n)*shape1.shape.invMomentOfInertia) + (cross(rb, n)*cross(rb, n)*shape2.shape.invMomentOfInertia);
    return effectiveMass;
}

Vector2 calculateRelativeVelocity(const RigidBody& shape1, const RigidBody& shape2, const Contact& contact){
    Vector2 ra = contact.contactPoint - shape1.position;
    Vector2 rb = contact.contactPoint - shape2.position;
    Vector2 velA = shape1.linearVelocity + cross(shape1.angularVelocity, ra);
    Vector2 velB = shape2.linearVelocity + cross(shape2.angularVelocity, rb);
    Vector2 rel_v = velB - velA;
    return rel_v;
}

float calculateImpulseRestitutionBias(const RigidBody& shape1, const RigidBody& shape2, const Contact& contact, float effectiveMass){
    Vector2 n = contact.normal;
    Vector2 rel_v = calculateRelativeVelocity(shape1, shape2, contact);
    float numerator = - restitution * dot(rel_v, n);
    return numerator;
}

float calculateImpulseCollision(const RigidBody& shape1, const RigidBody& shape2, const Contact& contact, float effectiveMass, float restitutionBias){
    Vector2 n = contact.normal;
    Vector2 rel_v = calculateRelativeVelocity(shape1, shape2, contact);
    float numerator = -dot(rel_v, n) + restitutionBias;
    return numerator / effectiveMass;
}

float calculateImpulseFriction(const RigidBody& shape1, const RigidBody& shape2, const Contact& contact, Vector2& tangent, float effectiveMass){
    Vector2 n = contact.normal;
    Vector2 rel_v = calculateRelativeVelocity(shape1, shape2, contact);
    tangent = rel_v - n * dot(n, rel_v);
    tangent = tangent.normalized();

    float numerator = - (friction_coefficient) * dot(rel_v, tangent);
    return numerator / effectiveMass;
}

void handleImpulseAddition(RigidBody& shape1, RigidBody& shape2, Contact contact, float impulse, Vector2 normal){
    Vector2 ra = contact.contactPoint - shape1.position;
    Vector2 rb = contact.contactPoint - shape2.position;
    shape1.linearVelocity = shape1.linearVelocity - normal * impulse * shape1.shape.invmass;
    // printf("%f %f %f\n", normal.x, normal.y, impulse);
    shape2.linearVelocity = shape2.linearVelocity + normal * impulse * shape2.shape.invmass;
    shape1.angularVelocity -= impulse * cross(ra, normal) * shape1.shape.invMomentOfInertia;
    shape2.angularVelocity += impulse * cross(rb, normal) * shape2.shape.invMomentOfInertia;
}

void handleCollision(RigidBody &shape1, RigidBody &shape2, std::vector<Contact>& contacts){
    if (contacts.size()==2){
        if (rand()%2==0){
            std::swap(contacts[0], contacts[1]);
        }
    }
    if (shape1.shape.invmass==0 and shape2.shape.invmass==0){
        return;
    }
    std::vector<float> accumulatedImpulse(contacts.size(), 0);
    std::vector<float> accumulatedFrictionImpulse(contacts.size(), 0);
    std::vector<float> effectiveMass(contacts.size(), 0);
    std::vector<float> restitutionBias(contacts.size(), 0);
    for (int i = 0; i < contacts.size(); ++i) {
        Contact& c = contacts[i];
        effectiveMass[i] = calculateEffectiveMass(shape1, shape2, c);
        restitutionBias[i] = calculateImpulseRestitutionBias(shape1, shape2, c, effectiveMass[i]);
    }
    for (int iter =0; iter<10; iter++){
        for (int i = 0; i < contacts.size(); ++i) {
            Contact& contact = contacts[i];

            float desiredImpulse = calculateImpulseCollision(shape1, shape2, contact, effectiveMass[i], restitutionBias[i]);
            float newImpulse = accumulatedImpulse[i] + desiredImpulse;
            newImpulse = std::max(newImpulse, 0.f);
            float impulse = newImpulse - accumulatedImpulse[i];
            accumulatedImpulse[i] = newImpulse;
            handleImpulseAddition(shape1, shape2, contact, impulse, contact.normal);
        }
    }

    for (int iter =0; iter<10; iter++){
        for (int i = 0; i < contacts.size(); ++i) {
            Contact& contact = contacts[i];

            Vector2 tangent;
            float frictional_impulse = calculateImpulseFriction(shape1, shape2, contact, tangent, effectiveMass[i]);
            float maximum_friction = accumulatedImpulse[i] * friction_coefficient;

            float newFrictionImpulse = accumulatedFrictionImpulse[i] + frictional_impulse;
            newFrictionImpulse = std::clamp(newFrictionImpulse, -maximum_friction, maximum_friction);
            float frictionImpulse = newFrictionImpulse - accumulatedFrictionImpulse[i];
            accumulatedFrictionImpulse[i] = newFrictionImpulse;

            handleImpulseAddition(shape1, shape2, contact, frictionImpulse, tangent);
        }
    }

    for (int i = 0; i < contacts.size(); ++i) {
        Contact contact = contacts[i];
        float correctionMag = std::max(-contact.penetration + 0.01f, 0.0f) 
                    / (shape1.shape.invmass + shape2.shape.invmass);
        Vector2 correction = contact.normal * correctionMag * 0.8;
        shape1.position = shape1.position - correction * shape1.shape.invmass;
        shape2.position = shape2.position + correction * shape2.shape.invmass;
        shape1.position = snap(shape1.position, 0.01f);
        shape2.position = snap(shape2.position, 0.01f);
    }
}

bool CheckCollisionAxes(RigidBody shape1, RigidBody shape2, Vector2 axis, CollisionData &coldata){
    Vector2 min1, min2, max1, max2;
    shape1.getMinMaxVerticesOnAxis(axis, min1, max1);
    shape2.getMinMaxVerticesOnAxis(axis, min2, max2);

    float A = project(min1, axis);
    float B = project(max1, axis);
    float C = project(min2, axis);
    float D = project(max2, axis);
    if (A <= C && B>= C){
        coldata.normal = axis;
        coldata.penetration = C - B;
        coldata.pointOnLine = max1 + coldata.normal * coldata.penetration;

        return true;
    }

    if (C <= A && D >= A){
        coldata.normal = -axis;
        coldata.penetration = A - D;
        coldata.pointOnLine = min1 + coldata.normal * coldata.penetration;
        return true;
    }

    return false;
}

bool isCollidingSat(RigidBody shape1, RigidBody shape2, CollisionData* coldata){
    std::vector<Vector2> possibleCollisionAxes;
    
    std::vector<Vector2> axes1;
    shape1.getAxes(axes1);
    for (const Vector2& axes : axes1){
        possibleCollisionAxes.push_back(axes);
    }

    std::vector<Vector2> axes2;
    shape2.getAxes(axes2);
    for (const Vector2& axes : axes2){
        possibleCollisionAxes.push_back(axes);
    }


    CollisionData cur_colData;
    CollisionData best_colData;

    best_colData.penetration = -FLT_MAX;
    for (const Vector2& axis : possibleCollisionAxes){
        if (!CheckCollisionAxes (shape1, shape2, axis , cur_colData))
           return false;
        if (cur_colData.penetration >= best_colData.penetration){
            best_colData = cur_colData;
        }
    }

    *coldata = best_colData;
    return true;
}

std::vector<Vector2> ClipPolygonAgainstPlane(const std::vector<Vector2>& input, const Plane& plane) {
    std::vector<Vector2> output;
    for (int i = 0; i<input.size(); i++){
        Vector2 current = input[i];
        Vector2 prev    = input[(i + input.size() - 1) % input.size()];

        bool currInside = dot(plane.normal, current) >= plane.distance;
        bool prevInside = dot(plane.normal, prev)    >= plane.distance;
        if (prevInside && currInside) {
            output.push_back(current);
        }
        else if (prevInside && !currInside) {
            float t = (plane.distance - dot(plane.normal, prev)) / dot(plane.normal, current - prev);
            output.push_back(prev + (current - prev) * t);
        }
        else if (!prevInside && currInside) {
            float t = (plane.distance - dot(plane.normal, prev)) / dot(plane.normal, current - prev);
            output.push_back(prev + (current - prev) * t);
            output.push_back(current);
        }
    }
    return output;
}

void getCollisionManifold(RigidBody &shape1, RigidBody &shape2, CollisionData *coldata, std::vector<Contact>& out_manifold){

    if (coldata->penetration >= 0.f){
        return;
    }
    std::vector<Vector2> polygon1, polygon2;
    std::vector<Plane> adjPlanes1, adjPlanes2;
    Vector2 normal1, normal2;

    shape1.GetIncidentReferencePolygon(coldata->normal, polygon1, normal1, adjPlanes1);
    shape2.GetIncidentReferencePolygon(-coldata->normal, polygon2, normal2, adjPlanes2);

    if (polygon1.size() == 0 || polygon2.size() == 0){
        return;
    } else if (polygon1.size() == 1)
    {
        out_manifold.push_back({
            polygon1[0],
            coldata->normal,
            coldata->penetration,
        });
    } else if (polygon2.size() == 1){
        out_manifold.push_back({
            polygon2[0],
            coldata->normal,
            coldata->penetration,
        });
    } else{
        bool flipped = fabs(dot(coldata->normal , normal1)) < fabs(dot(coldata->normal, normal2));
        if (flipped){
            std::swap(polygon1, polygon2);
            std::swap(normal1, normal2);
            std::swap(adjPlanes1, adjPlanes2);
        }

        if (adjPlanes1.size() > 0){
            for (const Plane& plane : adjPlanes1){
                polygon2 = ClipPolygonAgainstPlane(polygon2, plane);
            }
        }

        Plane RefPlane = {normal1, dot(normal1, polygon1[0])};
        std::vector<Vector2> output;
        for (const auto& p : polygon2) {
            bool currInside = dot(RefPlane.normal, p) <= RefPlane.distance+1e-2f;
            if(currInside){
                output.push_back(p);
            }
        }
        swap(output, polygon2);

        std::vector<Vector2> unique_points;
        for (const auto& p : polygon2) {
            bool exists = false;
            for (const auto& up : unique_points) {
                if (std::fabs(p.x - up.x) < 0.01 && std::fabs(p.y - up.y) < 0.01) {
                    exists = true;
                    break;
                }
            }
            if (!exists) unique_points.push_back(p);
        }
        polygon2.swap(unique_points);
        for (const Vector2 & point : polygon2 ){
            Vector2 pointDiff = point - getClosestPoint(polygon1, point);
            float penetrationDepth = dot(pointDiff, coldata->normal);
            if (flipped){
                penetrationDepth = -penetrationDepth;
            }
            if (penetrationDepth<0.f){
                out_manifold.push_back({
                    point,
                    coldata->normal,
                    penetrationDepth
                });
            }
        }
    }
}