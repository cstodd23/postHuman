#include "verlet_object.hpp"
#include "raymath.h"
#include <algorithm>
#include <cmath>

// Free Object Constructor
VerletObject::VerletObject(Vector2 pos, float radius, bool fixed)
    : currPosition(pos), lastPosition(pos), radius(radius), fixed(fixed) {
        color = defaultColor;
    }

// Body Object Constructor
VerletObject::VerletObject(Vector2 pos, float radius, bool fixed, Color default_color, int32_t body_id)
    : currPosition(pos), lastPosition(pos), radius(radius), fixed(fixed), defaultColor(default_color), bodyID(body_id) {
        color = default_color;
    }

// Control Point Constructor
VerletObject::VerletObject(Vector2 pos, float radius, Color default_color, int32_t body_id, int32_t control_point_id)
    : currPosition(pos), lastPosition(pos), radius(radius), fixed(false), defaultColor(default_color), 
    bodyID(body_id), controlPoint(true), controlPointID(control_point_id), inputAllowed(true) {
        color = default_color;
    }

// Core Physics Methods
void VerletObject::UpdatePosition(float dt) {
    if (fixed && !inputAllowed) {
        StopObject();
        return;
    }

    if (fixed && inputAllowed) {
        const Vector2 velocity = Vector2Subtract(currPosition, lastPosition);

        const float speed = Vector2Length(velocity);
        const float speedNormalized = std::min(speed / 100.0f, 9.0f);
        const float logFactor = std::log(speedNormalized + 1.0f) / std::log(9.0f + 1.0f);
        const float dynamicDamping = MIN_DAMPING_FACTOR + logFactor * (MAX_DAMPING_FACTOR - MIN_DAMPING_FACTOR);

        Vector2 dynamicVelocity = Vector2Scale(velocity, dynamicDamping);

        lastPosition = currPosition;
        currPosition = currPosition + dynamicVelocity + acceleration * dt * dt;
        acceleration = {0.0f, 0.0f};
        return;
    }

    const Vector2 velocity = Vector2Subtract(currPosition, lastPosition);
    const float speed = Vector2Length(velocity);

    if (isSleeping) {
        float displacement = Vector2Distance(currPosition, lastPosition);
        if (displacement > radius * 0.1f || speed > WAKE_VELOCITY_THRESHOLD || wasInCollisionThisUpdate) {
            Wake();
        } else {
            acceleration = {0.0f, 0.0f};
            return;
        }
    }

    const float speedNormalized = std::min(speed / 100.0f, 9.0f);
    const float logFactor = std::log(speedNormalized + 1.0f) / std::log(9.0f + 1.0f);
    const float dynamicDamping = MIN_DAMPING_FACTOR + logFactor * (MAX_DAMPING_FACTOR - MIN_DAMPING_FACTOR);

    // Vector2 temp = currPosition;
    const Vector2 dynamicVelocity = Vector2Scale(velocity, dynamicDamping);
    lastPosition = currPosition;
    currPosition = currPosition + dynamicVelocity + acceleration * dt * dt;

    if (bodyID == -1) {
        UpdateSleepState(speed, dt);
    }

    acceleration = {0.0f, 0.0f};
    UpdateColor();
    wasInCollisionThisUpdate = false;
}

void VerletObject::StopObject() {
    acceleration = {0.0f, 0.0f};
    lastPosition = currPosition;
}

void VerletObject::Sleep() {
    if (!isSleeping) {
        isSleeping = true;
        StopObject();
        defaultColor = GRAY;
    }
}

void VerletObject::Wake() {
    if (isSleeping) {
        sleepTimer = 0.0f;
        isSleeping = false;
        defaultColor = WHITE;
    }
}

void VerletObject::NotifyCollision() {
    wasInCollisionThisUpdate = true;
    if (isSleeping) {
        Wake();
    }
}

void VerletObject::UpdateVelocity(Vector2 v, float dt) {
    lastPosition = Vector2Subtract(lastPosition, Vector2Scale(v, dt));
    Wake();
}

// Setter Methods
void VerletObject::SetPosition(Vector2 pos) {
    currPosition = pos;
}

void VerletObject::SetCurrLastPositions(Vector2 pos) {
    currPosition = pos;
    lastPosition = pos;
}

void VerletObject::SetPositionAndStop(Vector2 pos) {
    currPosition = pos;
    lastPosition = pos;
    acceleration = {0.0f, 0.0f};
}

void VerletObject::SetAcceleration(Vector2 accel) {
    acceleration = accel;
}

void VerletObject::SetDefaultColor(Color newDefaultColor) {
    defaultColor = newDefaultColor;
    color = newDefaultColor;
}

void VerletObject::SetColor(Color newColor) {
    color = newColor;
}

void VerletObject::SetHidden(bool isHidden) {
    hidden = isHidden;
}

// Private Helper Methods
void VerletObject::UpdateColor() {
    if (wasInCollisionThisUpdate) {
        color = PINK;
    } else {
        color = defaultColor;
    }
}

void VerletObject::UpdateSleepState(float speed, float dt) {
    if (speed < SLEEP_VELOCITY_THRESHOLD) {
        sleepTimer += dt;
        if (sleepTimer > SLEEP_TIME_REQUIRED) {
            Sleep();
        }
    } else {
        sleepTimer = 0.0f;
    }
}