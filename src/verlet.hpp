#pragma once
#include "raylib.h"
#include "raymath.h"
#include <vector>
#include <cmath>
#include <cstdint>

constexpr int DEFAULT_SUBSTEPS = 8;
constexpr float DEFAULT_RADIUS = 10.0f;
constexpr float DEFAULT_DAMPING_FACTOR = 0.9999f;
constexpr float DEFAULT_GRAVITY = 980.7f;



struct VerletObject {
    Vector2 currPosition = {0.0f, 0.0f};
    Vector2 lastPosition = {0.0f, 0.0f};
    Vector2 acceleration = {0.0f, 0.0f};
    float radius = DEFAULT_RADIUS;
    Color color = WHITE;
    bool hidden = false;
    bool fixed = false;

    VerletObject() = default;
    VerletObject(Vector2 pos, float radius, bool fixed)
        : currPosition(pos), lastPosition(pos), radius(radius), fixed(fixed) {}

    void UpdatePosition(float dt) {
        Vector2 temp = currPosition;
        const Vector2 displacement = Vector2Subtract(currPosition, lastPosition) * DEFAULT_DAMPING_FACTOR;
        lastPosition = currPosition;
        currPosition = currPosition + displacement + acceleration * dt * dt;
        acceleration = {};
    }

    void UpdateVelocity(Vector2 v, float dt) {lastPosition -= v * dt;}
};

struct VerletConstraint {

};



struct Solver { 
private:
    float targetPhysicsHz = 240.0f;
    float physicsDeltatime = 1.0 / targetPhysicsHz;
    float time = 0.0f;
    int physicsSteps = 0;
    int stepsRequired = 1;
    // float currentPhysicsStepDeltaTime = 0;
    int32_t substeps;
    Vector2 gravity = {0.0f, -DEFAULT_GRAVITY};
    
public:
    std::vector<VerletObject> objects;
    std::vector<VerletConstraint> constraints;
    Solver()
        : substeps(DEFAULT_SUBSTEPS) {}

    void UpdateTiming(float dt) {
        time += dt;
        int totalSteps = static_cast<int>(time * targetPhysicsHz);
        stepsRequired = std::min(15, totalSteps - physicsSteps);
    }

    void UpdateNaive() {
        for (int i(stepsRequired); i--;) {
            UpdatePhysicsStep();
        }
    }

    void UpdatePhysicsStep() {
        physicsSteps++;
        /* Verlet Update Steps
        1. Collisions
        2. Constraints
        3. Soft bodies
        4. Objects
            4a. Gravity
            4b. Object Update Position
            4c. Apply */
        SolveCollisionsNaive();
        //UpdateConstraints
        //UpdateSoftBodies
        UpdateObjects();
    }

    void SolveCollisionsNaive() {
        
    }

    void ApplyBorders(VerletObject& obj) {
        
    }

    VerletObject& AddObject(Vector2 pos, float radius, bool fixed = false) {
        return objects.emplace_back(pos, radius, fixed);
    }

    void UpdateObjects() {
        for (VerletObject& obj : objects) {
            UpdateObject(obj);
        }
    }

    void UpdateObject(VerletObject& obj) {
        if (!obj.fixed) {
            obj.acceleration.y -= DEFAULT_GRAVITY;
        }
        obj.UpdatePosition(physicsDeltatime);
        ApplyBorders(obj);
    }

    void SetObjectVelocity(VerletObject& obj, Vector2 velocity) {
        obj.UpdateVelocity(velocity, physicsDeltatime);
    }
};



struct Renderer {
    void Render(int width, int height, Solver& solver) {
        BeginDrawing();
        ClearBackground(RAYWHITE);
        RenderBackground(width, height);
        RenderObjects(solver);
    }

    void RenderObjects(const Solver& solver) const {
        const auto& objects = solver.objects;
        for (auto& obj : objects) {
            DrawCircle(obj.currPosition.x, obj.currPosition.y, obj.radius, obj.color);
        }
    }

    void RenderBackground(int width, int height) {
        DrawRectangle(0, 0, width, height, BLACK);
    }
};



struct Game {
public:
    Game(int worldW, int worldH)
        : worldWidth(worldW), worldHeight(worldH), solver(Solver()), renderer(Renderer()) {}

    void MainLoop() {
        SetTraceLogLevel(LOG_WARNING);
        InitWindow(worldWidth, worldHeight, "postHuman");
        while (!WindowShouldClose()) {
            solver.UpdateTiming(GetFrameTime());
            solver.UpdateNaive();
            renderer.Render(worldWidth, worldHeight, solver);
        }
        CloseWindow();
    }

    void SpwanFree(int32_t count, Vector2 spawnPos, float spawnSpeed, float spawnDelay, float spawnAngle) {
        int total = 0;
        const Vector2 spawnAngleVector = {cos(spawnAngle), sin(spawnAngle)};

        SpawnFreeObject(spawnPos, spawnAngleVector, DEFAULT_RADIUS, spawnSpeed);
    }

    void SpawnFreeObject(Vector2 pos, Vector2 angle, float radius, float speed) {
        VerletObject& obj = solver.AddObject(pos, radius);
        Vector2 velocity = Vector2Scale(angle, speed);
        solver.SetObjectVelocity(obj, velocity);
    }


private:
    Solver solver;
    Renderer renderer;
    int worldWidth;
    int worldHeight;
};

