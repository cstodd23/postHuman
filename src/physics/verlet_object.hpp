#pragma once
#include <cmath>            // for std::log, std::min
#include <cstdint>          // for int32_t
#include "raylib.h"         // main raylib library

constexpr float DEFAULT_RADIUS = 10.0f;
constexpr float MAX_DAMPING_FACTOR = 0.9995f;
constexpr float MIN_DAMPING_FACTOR = 0.999f;


struct VerletObject {
public:
    static constexpr float SLEEP_VELOCITY_THRESHOLD = 0.1f;
    static constexpr float WAKE_VELOCITY_THRESHOLD = 1.0F;
    static constexpr float SLEEP_TIME_REQUIRED = 5.0f;

    VerletObject() = default;
    VerletObject(Vector2 pos, float radius, bool fixed);
    VerletObject(Vector2 pos, float radius, bool fixed, Color default_color, int32_t body_id);
    VerletObject(Vector2 pos, float radius, Color default_color, int32_t body_id, int32_t control_point_id);

    // Core Physics Methods
    void UpdatePosition(float dt);
    void UpdateVelocity(Vector2 v, float dt);
    void NotifyCollision();

    // State Management
    void Sleep();
    void Wake();
    void StopObject();

    // Getters
    Vector2     GetPosition()       const { return currPosition; }
    Vector2     GetLastPosition()   const { return lastPosition; }
    Vector2     GetAcceleration()   const { return acceleration; }
    float       GetRadius()         const { return radius; }
    Color       GetColor()          const { return color; }
    Color       GetDefaultColor()   const { return defaultColor; }
    int32_t     GetBodyID()         const { return bodyID; }
    int32_t     GetControlPointID() const { return controlPointID; }
    bool        IsFixed()           const { return fixed; }
    bool        IsControlPoint()    const { return controlPoint; }
    bool        IsInputAllowed()    const { return inputAllowed; }
    bool        IsSleeping()        const { return isSleeping; }
    bool        IsHidden()          const { return hidden; }

    // Setters
    void SetPosition(Vector2 pos);
    void SetCurrLastPositions(Vector2 pos);
    void SetPositionAndStop(Vector2 pos);
    void SetAcceleration(Vector2 accel);
    void SetDefaultColor(Color newDefaultColor);
    void SetColor(Color newColor);
    void SetHidden(bool isHidden);

private:
    // Core Physics Setup
    int32_t bodyID = -1;
    Vector2 currPosition = {0.0f, 0.0f};
    Vector2 lastPosition = {0.0f, 0.0f};
    Vector2 acceleration = {0.0f, 0.0f};
    float radius = DEFAULT_RADIUS;

    // Visual Properties
    Color defaultColor;
    Color color;
    bool hidden = false;

    // Object Properties
    bool fixed;
    bool inputAllowed = false;
    bool controlPoint = false;
    int32_t controlPointID = -1;

    // Sleep System
    bool isSleeping = false;
    float sleepTimer = 0.0f;
    bool wasInCollisionThisUpdate = false;

    // Private Helper Methods
    void UpdateColor();
    void UpdateSleepState(float speed, float dt);

    friend class Solver;
};