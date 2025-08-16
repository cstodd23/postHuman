#include "verlet_components.hpp"
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

VerletConstraint::VerletConstraint(int32_t obj_1_index, int32_t obj_2_index, 
    float max_target_distance, float min_target_distance)
    : obj1Index(obj_1_index), obj2Index(obj_2_index), maxTargetDistance(max_target_distance), 
    minTargetDistance(min_target_distance) {}

void VerletConstraint::Apply(std::vector<VerletObject>& objects) {
    if(!active) { return; }

    int32_t objectsSize = static_cast<int32_t>(objects.size());
    if (obj1Index >= objectsSize || obj2Index >= objectsSize || obj1Index < 0 || obj2Index < 0) {
        return;
    }

    VerletObject& obj1 = objects[obj1Index];
    VerletObject& obj2 = objects[obj2Index];
    
    if (obj1.IsFixed() && obj2.IsFixed()) {return;}

    const Vector2 displacement = Vector2Subtract(obj1.GetPosition(), obj2.GetPosition());
    float distance = Vector2Length(displacement);

    const float MIN_DISTANCE = 0.001f;
    if (distance < MIN_DISTANCE) {distance = MIN_DISTANCE;}
    const Vector2 normal = Vector2Scale(displacement, 1.0f / distance);

    float delta = 0.0f;
    if (distance > maxTargetDistance) {
        delta = maxTargetDistance - distance;
    } else if (distance < minTargetDistance) {
        delta = minTargetDistance - distance;
    } else {
        return; 
    }

    delta *= damping;

    ApplyConstraintForce(obj1, obj2, normal, delta);

    // if (!obj1.IsFixed() && obj2.IsFixed()) {
    //     obj1.SetPosition(Vector2Add(obj1.GetPosition(), Vector2Scale(normal, delta)));
    // } else if (obj1.IsFixed() && !obj2.IsFixed()) {
    //     obj2.SetPosition(Vector2Subtract(obj2.GetPosition(), Vector2Scale(normal, delta)));
    // } else if (!obj1.IsFixed() && !obj2.IsFixed()) {
    //     const Vector2 halfDelta = Vector2Scale(normal, delta * 0.5f);
    //     obj1.SetPosition(Vector2Add(obj1.GetPosition(), halfDelta));
    //     obj2.SetPosition(Vector2Subtract(obj2.GetPosition(), halfDelta));
    // }
}

bool VerletConstraint::IsValidConstraint(const std::vector<VerletObject>& objects) const {
    return ValidateIndices(static_cast<int32_t>(objects.size()));
}

float VerletConstraint::GetCurrentDistance(const std::vector<VerletObject>& objects) const {
    if (!ValidateIndices(static_cast<int32_t>(objects.size()))) {
        return -1.0f;
    }

    const VerletObject& obj1 = objects[obj1Index];
    const VerletObject& obj2 = objects[obj2Index];

    return Vector2Distance(obj1.GetPosition(), obj2.GetPosition());
}

// Helper Methods
bool VerletConstraint::ValidateIndices(int32_t objectsSize) const {
    return (obj1Index >= 0 && obj1Index < objectsSize &&
            obj2Index >= 0 && obj2Index < objectsSize &&
            obj1Index != obj2Index);
}

void VerletConstraint::ApplyConstraintForce(VerletObject& obj1, VerletObject& obj2, 
                                            const Vector2& normal, float delta) const {

    if (!obj1.IsFixed() && obj2.IsFixed()) {
        obj1.SetPosition(Vector2Add(obj1.GetPosition(), Vector2Scale(normal, delta)));

    } else if (obj1.IsFixed() && !obj2.IsFixed()) {
        obj2.SetPosition(Vector2Subtract(obj2.GetPosition(), Vector2Scale(normal, delta)));

    } else if (!obj1.IsFixed() && !obj2.IsFixed()) {
        const Vector2 halfDelta = Vector2Scale(normal, delta * 0.5f);
        obj1.SetPosition(Vector2Add(obj1.GetPosition(), halfDelta));
        obj2.SetPosition(Vector2Subtract(obj2.GetPosition(), halfDelta));
    }
}