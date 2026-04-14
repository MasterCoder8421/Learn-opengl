#include "RigidBody.h"
#include "Utils.h"
#include "constants.h"

Vector2 Vector2::operator+(const Vector2& other) const {
    return Vector2{x + other.x, y + other.y};
}

Vector2 Vector2::operator-(const Vector2& other) const {
    return Vector2{x - other.x, y - other.y};
}

Vector2 Vector2::operator*(const float other) const {
    return Vector2{x * other, y * other};
}

Vector2 Vector2::operator-() const{
    return Vector2{-x, -y};
}

Vector2 Vector2::perpendicular() const {
    return {-y, x};
}

float Vector2::length() const {
    return std::sqrt(x*x + y*y);
}

Vector2 Vector2::normalized() const {
    float len = length();
    return len > 0 ? Vector2{x / len, y / len} : Vector2{0, 0};
}

float dot(Vector2 a, Vector2 b){
    return a.x * b.x + a.y * b.y;
}

float cross(Vector2 a, Vector2 b){
    return a.x * b.y - a.y * b.x;
}

Vector2 cross(float omega, Vector2 r) {
    return (Vector2){-omega * r.y, omega * r.x};
}

float project(Vector2 a, Vector2 b){
    return dot(a, b) / dot(b, b);
}

float kineticEnergy(const RigidBody& body) {
    float linearSpeedSquared = body.linearVelocity.x * body.linearVelocity.x +
                               body.linearVelocity.y * body.linearVelocity.y;
    if (body.shape.invmass==0){
        return 0;
    }
    float KE_trans = 0.5f * 1/body.shape.invmass * linearSpeedSquared;
    float KE_rot = 0.5f * 1/body.shape.invMomentOfInertia * body.angularVelocity * body.angularVelocity;
    float PE_grav = 1/body.shape.invmass * gravity * -body.position.y;
    return KE_trans + KE_rot + PE_grav;
}

void drawPoint(int x, int y){
    SDL_SetRenderDrawColor(gRenderer, 0, 0, 0, 255);
    SDL_Rect rect;
    rect.x = (int)(x - 10);
    rect.y = (int)(y - 10);
    rect.w = 20;
    rect.h = 20;
    SDL_RenderFillRect(gRenderer, &rect);
};

Vector2 closestPointOnSegment(const Vector2& p, const Vector2& a, const Vector2& b) {
    Vector2 ab = b - a;
    float t = dot(p - a, ab) / (ab.length() * ab.length());
    t = std::fmax(0.0f, std::fmin(1.0f, t));
    Vector2 closest = a + ab * t;
    return closest;
}

Vector2 getClosestPoint(std::vector<Vector2> polygon, Vector2 point){
    float min_dist = FLT_MAX;
    Vector2 ans = {0, 0};
    for (int i = 0; i<polygon.size(); i++){
        Vector2 curr = polygon[i];
        Vector2 next = polygon[(i+1)%polygon.size()];
        Vector2 cp = closestPointOnSegment(point, curr, next);
        float dist = (cp - point).length();
        if (dist <= min_dist) ans = cp;
        min_dist = std::min(min_dist, dist);
    }
    return ans;
}

float snap(float value, float step) {
    return round(value / step) * step;
}

Vector2 snap(Vector2 value, float step) {
    return Vector2{snap(value.x, step), snap(value.y, step)};
}

