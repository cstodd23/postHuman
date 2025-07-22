#pragma once
#include "raylib.h"
#include "raymath.h"
#include <vector>
#include <queue>
#include <unordered_map>
#include <cmath>
#include <cstdint>

constexpr int DEFAULT_SUBSTEPS = 8;
constexpr float DEFAULT_RADIUS = 10.0f;
constexpr float MAX_DAMPING_FACTOR = 0.9995f;
constexpr float MIN_DAMPING_FACTOR = 0.999f;
constexpr float DEFAULT_GRAVITY = -980.7f;
constexpr float MARGIN_WIDTH = 10.0f;
constexpr float RESPONSE_COEF = 0.5f;



struct VerletObject {
    static constexpr float SLEEP_VELOCITY_THRESHOLD = 0.1f;
    static constexpr float WAKE_VELOCITY_THRESHOLD = 1.0F;
    static constexpr float SLEEP_TIME_REQUIRED = 5.0f;
    static constexpr float MAX_DAMPING_FACTOR = 0.9995f;
    static constexpr float MIN_DAMPING_FACTOR = 0.999f;
    Vector2 currPosition = {0.0f, 0.0f};
    Vector2 lastPosition = {0.0f, 0.0f};
    Vector2 acceleration = {0.0f, 0.0f};
    float radius = DEFAULT_RADIUS;
    Color color = WHITE;
    bool hidden = false;
    bool fixed = false;
    bool isSleeping = false;
    float sleepTimer = 0.0f;
    bool wasInCollisionThisUpdate = false;

    VerletObject() = default;
    VerletObject(Vector2 pos, float radius, bool fixed)
        : currPosition(pos), lastPosition(pos), radius(radius), fixed(fixed) {}

    void UpdatePosition(float dt) {
        if (fixed) {acceleration = {0.0f, 0.0f}; return;}

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

        const float distance = Vector2Distance(currPosition, lastPosition);

        const float speedNormalized = std::min(speed / 100.0f, 9.0f);
        const float logFactor = std::log(speedNormalized + 1.0f) / std::log(9.0f + 1.0f);
        const float dynamicDamping = MIN_DAMPING_FACTOR + logFactor * (MAX_DAMPING_FACTOR - MIN_DAMPING_FACTOR);

        // Vector2 temp = currPosition;
        const Vector2 displacement = Vector2Subtract(currPosition, lastPosition) * dynamicDamping;
        lastPosition = currPosition; // ADD SLEEPING TO THIS FUNCTION DUMMY
        currPosition = currPosition + displacement + acceleration * dt * dt;

        UpdateSleepState(speed, dt);

        acceleration = {0.0f, 0.0f};
        wasInCollisionThisUpdate = false;
    }

    void UpdateSleepState(float speed, float dt) {
        if (speed < SLEEP_VELOCITY_THRESHOLD) {
            sleepTimer += dt;
            if (sleepTimer > SLEEP_TIME_REQUIRED) {
                Sleep();
            }
        } else {
            sleepTimer = 0.0f;
        }
    }

    void Sleep() {
        if (!isSleeping) {
            isSleeping = true;
            lastPosition = currPosition;
            color = GRAY;
        }
    }

    void Wake() {
        if (isSleeping) {
            sleepTimer = 0.0f;
            isSleeping = false;
            color = WHITE;
        }
    }

    void NotifyCollision() {
        wasInCollisionThisUpdate = true;
        if (isSleeping) {
            Wake();
        }
    }

    void UpdateVelocity(Vector2 v, float dt) {
        lastPosition = Vector2Subtract(lastPosition, Vector2Scale(v, dt));
        Wake();
    }
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
    std::unordered_map<int32_t, int32_t> object_body_mapping;
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
        for (size_t i = 0; i < objects.size(); i++) {
            for (size_t j = i + 1; j < objects.size(); j++) {
                SolveCollision(i, j);
            }
        }
    }

    void SolveCollision(int32_t obj1_id, int32_t obj2_id) {
        if (object_body_mapping.count(obj1_id) && object_body_mapping.count(obj2_id)) {
            if (object_body_mapping[obj1_id] == object_body_mapping[obj2_id]) {return;}
        }
        VerletObject& obj1 = objects[obj1_id];
        VerletObject& obj2 = objects[obj2_id];

        if (obj1.fixed && obj2.fixed) {return;}

        float distance = Vector2Distance(obj1.currPosition, obj2.currPosition);
        const Vector2 displacement = Vector2Subtract(obj1.currPosition, obj2.currPosition);
        const float min_distance = obj1.radius + obj2.radius;
        const float overlap = min_distance - distance;
        if (distance < 0.0001f) {distance = 0.001f;}

        constexpr float esp = 0.0001f;

        if (distance < min_distance && overlap > esp) {
            obj1.NotifyCollision();
            obj2.NotifyCollision();

            float radius1 = obj1.radius;
            float radius2 = obj2.radius;
            const float massProportion1 = radius1 * radius1 * radius1;
            const float massProportion2 = radius2 * radius2 * radius2;
            const float totalMassProportion = massProportion1 + massProportion2;
            const Vector2 collisionNormal = Vector2Normalize(displacement);
            const float collisionRatio1 = massProportion2 / totalMassProportion;
            const float collisionRatio2 = massProportion1 / totalMassProportion;
            const float delta = RESPONSE_COEF * (distance - min_distance);
            
            if (!obj1.fixed && !obj2.fixed) {
                Vector2 collisionVector1 = Vector2Scale(collisionNormal, (0.5f * collisionRatio1 * delta));
                Vector2 collisionVector2 = Vector2Scale(collisionNormal, (0.5f * collisionRatio2 * delta));
                obj1.currPosition = Vector2Subtract(obj1.currPosition, collisionVector1);
                obj2.currPosition = Vector2Add(obj2.currPosition, collisionVector2);
            } else if (!obj1.fixed && obj2.fixed) {
                obj1.currPosition = Vector2Subtract(obj1.currPosition, Vector2Scale(collisionNormal, delta));
            } else if (obj1.fixed && !obj2.fixed) {
                obj2.currPosition = Vector2Add(obj2.currPosition, Vector2Scale(collisionNormal, delta));
            }
        }
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
            SpawnCommand command = SpawnCommand({50.0f, 50.0f}, 2.0f * PI, 10.0f, 700.0f, 0.15f);
            AddSpawnCommand(command);
        }
        for (int i = 0; i < 10; i++) {
            SpawnCommand command = SpawnCommand({50.0f, 50.0f}, 2.0f * PI, 20.0f, 100.0f, 0.8f);
            AddSpawnCommand(command);
        }
        // SpawnCommand smallBall = SpawnCommand({150.0f, 150.0f}, 0.0f, 10.0f, 1000.0f, 0.0f);
        // SpawnCommand bigBall = SpawnCommand({600.0f, 150.0f}, PI, 20.0f, 1000.0f, 0.0f);
        // AddSpawnCommand(smallBall);
        // AddSpawnCommand(bigBall);
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

