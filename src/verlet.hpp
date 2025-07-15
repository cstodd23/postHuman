#pragma once
#include "raylib.h"
#include "raymath.h"
#include <vector>
#include <queue>
#include <cmath>
#include <cstdint>

constexpr int DEFAULT_SUBSTEPS = 8;
constexpr float DEFAULT_RADIUS = 10.0f;
constexpr float DEFAULT_DAMPING_FACTOR = 0.9999f;
constexpr float DEFAULT_GRAVITY = -980.7f;
constexpr float MARGIN_WIDTH = 10.0f;
constexpr float RESPONSE_COEF = 0.5f;



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

struct SpawnCommand {
    Vector2 pos;
    float spawnAngle;
    float radius;
    float speed;
    float spawnDelay;

    SpawnCommand(Vector2 p, float a, float r, float s, float sD) 
        : pos(p), spawnAngle(a), radius(r), speed(s), spawnDelay(sD) {}
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
    Vector2 stageSize;
    
public:
    std::vector<VerletObject> objects;
    std::vector<VerletConstraint> constraints;
    Solver(Vector2 size)
        : substeps(DEFAULT_SUBSTEPS), stageSize(size) {}

    void UpdateTiming(float dt) {
        time += dt;
        int totalSteps = static_cast<int>(time * targetPhysicsHz);
        stepsRequired = std::min(15, totalSteps - physicsSteps);
    }

    void UpdateNaive() {
        UpdateTiming(GetFrameTime());
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
            4c. Apply Boarders */
        SolveCollisionsNaive();
        //UpdateConstraints
        //UpdateSoftBodies
        UpdateObjects();
    }

    void SolveCollisionsNaive() {
        
    }

    void ApplyBorders(VerletObject& obj) {
        const float margin = MARGIN_WIDTH + obj.radius;
        Vector2 collisionNormal = {0.0f, 0.0f};
        if (obj.currPosition.x > stageSize.x - margin) {
            collisionNormal = Vector2Add(collisionNormal, {obj.currPosition.x - stageSize.x + margin, 0.0f});
        } else if (obj.currPosition.x < margin) {
            collisionNormal = Vector2Subtract(collisionNormal, {margin - obj.currPosition.x, 0.0f});
        }
        
        if (obj.currPosition.y > stageSize.y - margin) {
            collisionNormal = Vector2Add(collisionNormal, {0.0f, obj.currPosition.y - stageSize.y + margin});
        } else if (obj.currPosition.y < margin) {
            collisionNormal = Vector2Subtract(collisionNormal, {margin - obj.currPosition.y, 0.0f});
        }
        obj.currPosition = Vector2Subtract(obj.currPosition, Vector2Scale(collisionNormal, 0.2f * RESPONSE_COEF));
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
        EndDrawing();
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
        : worldWidth(worldW), worldHeight(worldH), solver(Solver({float(worldW), float(worldH)})), renderer(Renderer()) {}

    void MainLoop() {
        SetTraceLogLevel(LOG_WARNING);
        InitWindow(worldWidth, worldHeight, "postHuman");
        for (int i = 0; i < 10; i++) {
            SpawnCommand command = SpawnCommand({50.0f, 50.0f}, 2.0f * PI, 10.0f, 100.0f, 0.1f);
            AddSpawnCommand(command);
        }
        for (int i = 0; i < 10; i++) {
            SpawnCommand command = SpawnCommand({50.0f, 50.0f}, 2.0f * PI, 10.0f, 500.0f, 0.8f);
            AddSpawnCommand(command);
        }
        while (!WindowShouldClose()) {
            ProcessSpawnQueue();
            solver.UpdateNaive();
            renderer.Render(worldWidth, worldHeight, solver);
        }
        CloseWindow();
    }

    void AddSpawnCommand(SpawnCommand command) {
        spawnQueue.push(command);
    }

    void ProcessSpawnQueue() {
        if (spawnQueue.empty()) {return;}
        SpawnCommand& command = spawnQueue.front();
        if (GetTime() - lastSpawnedTime < command.spawnDelay) {return;}
        
        SpawnFreeObject(command.pos, command.spawnAngle, command.radius, command.speed);
        spawnQueue.pop();
        lastSpawnedTime = GetTime();
    }

    void SpawnFreeObject(Vector2 pos, float spawnAngle, float radius, float speed) {
        Vector2 angle = {cos(spawnAngle), sin(spawnAngle)};
        VerletObject& obj = solver.AddObject(pos, radius);
        Vector2 velocity = Vector2Scale(angle, speed);
        solver.SetObjectVelocity(obj, velocity);
    }


private:
    Solver solver;
    Renderer renderer;
    int worldWidth;
    int worldHeight;
    std::queue<SpawnCommand> spawnQueue;
    float lastSpawnedTime = 0.0f;
};

