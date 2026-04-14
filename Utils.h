#ifndef UTILS_H
#define UTILS_H
#include <vector>

class RigidBody;

struct Vector2 {
    float x;
    float y;

    Vector2 operator+(const Vector2& other) const;
    Vector2 operator-(const Vector2& other) const;
    Vector2 operator-() const;
    Vector2 operator*(const float other) const;
    Vector2 perpendicular() const;
    float length() const;
    Vector2 normalized() const;
};

typedef struct {
    float width;
    float height;
    float invmass;
    float invMomentOfInertia;
} BoxShape;

struct CollisionData {
    Vector2 normal;
    Vector2 pointOnLine;
    float penetration;
};

struct Plane {
    Vector2 normal;
    float distance;
};

struct Contact{
    Vector2 contactPoint;
    Vector2 normal;
    float penetration;
};

float dot(Vector2 a, Vector2 b);
float cross(Vector2 a, Vector2 b);
Vector2 cross(float omega, Vector2 r);
float project(Vector2 a, Vector2 b);
float kineticEnergy(const RigidBody& body);
void drawPoint(int x, int y);
Vector2 closestPointOnSegment(const Vector2& p, const Vector2& a, const Vector2& b);
Vector2 getClosestPoint(std::vector<Vector2> polygon, Vector2 point);
float snap(float value, float step);
Vector2 snap(Vector2 value, float step);

#endif