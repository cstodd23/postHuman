#pragma once
#include "raylib.h"
#include "raymath.h"
#include <vector>
#include <queue>
#include <unordered_map>
#include <cmath>
#include <cstdint>

constexpr int DEFAULT_SUBSTEPS = 8;
constexpr int JAKOBSEN_ITERATIONS = 10;
constexpr float DEFAULT_RADIUS = 10.0f;
constexpr float MAX_DAMPING_FACTOR = 0.9995f;
constexpr float MIN_DAMPING_FACTOR = 0.999f;
constexpr float DEFAULT_GRAVITY = 980.7f;
constexpr float MARGIN_WIDTH = 10.0f;
constexpr float RESPONSE_COEF = 0.5f;



struct VerletObject {
    static constexpr float SLEEP_VELOCITY_THRESHOLD = 0.1f;
    static constexpr float WAKE_VELOCITY_THRESHOLD = 1.0F;
    static constexpr float SLEEP_TIME_REQUIRED = 5.0f;
    static constexpr float MAX_DAMPING_FACTOR = 0.9995f;
    static constexpr float MIN_DAMPING_FACTOR = 0.999f;
    int32_t bodyID = -1;
    Vector2 currPosition = {0.0f, 0.0f};
    Vector2 lastPosition = {0.0f, 0.0f};
    Vector2 acceleration = {0.0f, 0.0f};
    float radius = DEFAULT_RADIUS;
    Color defaultColor;
    Color color;
    bool hidden = false;
    bool fixed;
    bool isSleeping = false;
    float sleepTimer = 0.0f;
    bool wasInCollisionThisUpdate = false;

    VerletObject() = default;
    VerletObject(Vector2 pos, float radius, bool fixed)
        : currPosition(pos), lastPosition(pos), radius(radius), fixed(fixed) {}

    VerletObject(Vector2 pos, float radius, bool fixed, Color default_color, int32_t body_id)
        : currPosition(pos), lastPosition(pos), radius(radius), fixed(fixed), defaultColor(default_color), bodyID(body_id) {
            color = default_color;
        }

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
        lastPosition = currPosition;
        currPosition = currPosition + displacement + acceleration * dt * dt;

        if (bodyID == -1) {
            UpdateSleepState(speed, dt);
        }

        acceleration = {0.0f, 0.0f};
        UpdateColor();
        wasInCollisionThisUpdate = false;
    }

    void UpdateColor() {
        if (wasInCollisionThisUpdate) {
            color = PINK;
        } else {
            color = defaultColor;
        }
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
            defaultColor = GRAY;
        }
    }

    void Wake() {
        if (isSleeping) {
            sleepTimer = 0.0f;
            isSleeping = false;
            defaultColor = WHITE;
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
    int32_t obj1Index;
    int32_t obj2Index;
    const float maxTargetDistance;
    const float minTargetDistance;
    const float targetAngle = 2.0f * PI;
    bool inBody;

    VerletConstraint(int32_t obj_1_index, int32_t obj_2_index, float max_target_distance, float min_target_distance)
        : obj1Index(obj_1_index), obj2Index(obj_2_index), maxTargetDistance(max_target_distance), minTargetDistance(min_target_distance) {}

    void apply(std::vector<VerletObject>& objects) {
        if (obj1Index >= objects.size() || obj2Index >= objects.size() || obj1Index < 0 || obj2Index < 0) {
            return;
        }

        VerletObject& obj1 = objects[obj1Index];
        VerletObject& obj2 = objects[obj2Index];
        
        if (obj1.fixed && obj2.fixed) {return;}

        const Vector2 displacement = Vector2Subtract(obj1.currPosition, obj2.currPosition);
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

        const float CONSTRAINT_DAMPING = 0.5f;
        delta *= CONSTRAINT_DAMPING;

        if (!obj1.fixed && obj2.fixed) {
            obj1.currPosition = Vector2Add(obj1.currPosition, Vector2Scale(normal, delta));
        } else if (obj1.fixed && !obj2.fixed) {
            obj2.currPosition = Vector2Subtract(obj2.currPosition, Vector2Scale(normal, delta));
        } else if (!obj1.fixed && !obj2.fixed) {
            const Vector2 halfDelta = Vector2Scale(normal, delta * 0.5f);
            obj1.currPosition = Vector2Add(obj1.currPosition, halfDelta);
            obj2.currPosition = Vector2Subtract(obj2.currPosition, halfDelta);
        }
    }
};

struct SpawnCommand {
    enum Type { FREE_OBJECT, ROPE_SEGMENT };
    Type type = FREE_OBJECT;

    Vector2 pos;
    float radius;
    Color defaultColor;
    float spawnAngle;
    float speed;
    float spawnDelay;

    int32_t bodyID = -1;
    int32_t segmentIndex = -1;
    bool isFixed = false;

    // Constructor for single objects
    SpawnCommand(Vector2 position_,  float radius_, Color default_color, float spawn_angle, float speed_, float spawn_delay) 
        : type (FREE_OBJECT), pos(position_), radius(radius_), defaultColor(default_color),
        spawnAngle(spawn_angle), speed(speed_), spawnDelay(spawn_delay) {}

    // Constructor for Rope Segments
    SpawnCommand(Type type_, Vector2 position, float radius_, Color default_color, float spawn_delay, 
        int32_t body_id, int32_t segment_index, bool fixed_ = false)
        : type(type_), pos(position), spawnAngle(0), radius(radius_), defaultColor(default_color), speed(0),
        spawnDelay(spawn_delay), bodyID(body_id), segmentIndex(segment_index), isFixed(fixed_) {}
};



struct Solver { 
private:
    // float targetPhysicsHz = 240.0f;
    float targetPhysicsHz = 960.0f;
    // float targetPhysicsHz = 480.0f;
    float physicsDeltatime = 1.0 / targetPhysicsHz;
    float time = 0.0f;
    int physicsSteps = 0;
    int stepsRequired = 1;
    // float currentPhysicsStepDeltaTime = 0;
    int32_t substeps;
    Vector2 gravity = {0.0f, DEFAULT_GRAVITY};
    Vector2 stageSize;
    
public:
    std::vector<VerletObject> objects;
    std::vector<VerletConstraint> constraints;
    int32_t bodyCount = 0;
    // std::unordered_map<int32_t, int32_t> objectBodyMapping;
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
        UpdateConstraints();
        SolveCollisionsNaive();
        //UpdateSoftBodies
        UpdateObjects();
    }

    void SolveCollisionsNaive() {
        for (int32_t i = 0; i < static_cast<int32_t>(objects.size()); i++) {
            for (int32_t j = i + 1; j < static_cast<int32_t>(objects.size()); j++) {
                SolveCollision(i, j);
            }
        }
    }

    void SolveCollision(int32_t obj1_id, int32_t obj2_id) {
        VerletObject& obj1 = objects[obj1_id];
        VerletObject& obj2 = objects[obj2_id];

        if (obj1.bodyID != -1 && obj2.bodyID != -1 && obj1.bodyID == obj2.bodyID) {return;}

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
            collisionNormal = Vector2Subtract(collisionNormal, {0.0f, margin - obj.currPosition.y});
        }
        obj.currPosition = Vector2Subtract(obj.currPosition, Vector2Scale(collisionNormal, 0.2f * RESPONSE_COEF));
    }

    VerletObject& AddObject(Vector2 pos, float radius, Color color, bool fixed = false, int32_t bodyID = -1) {
        return objects.emplace_back(pos, radius, fixed, color, bodyID);
    }

    VerletConstraint& AddConstraint(int32_t obj1Index, int32_t obj2Index, float maxLength, float minLength) {
        return constraints.emplace_back(obj1Index, obj2Index, maxLength, minLength);
    }

    void UpdateObjects() {
        for (VerletObject& obj : objects) {
            UpdateObject(obj);
        }
    }

    void UpdateConstraints() {
        if (constraints.empty()) {return;}

        for (int32_t i = 0; i < JAKOBSEN_ITERATIONS; i++) {
            for (VerletConstraint& constraint : constraints) {
                constraint.apply(objects);
            }
        }
    }

    void UpdateObject(VerletObject& obj) {
        if (!obj.fixed) {
            obj.acceleration.y += DEFAULT_GRAVITY;
        }
        obj.UpdatePosition(physicsDeltatime);
        ApplyBorders(obj);
    }

    void SetObjectVelocity(VerletObject& obj, Vector2 velocity) {
        obj.UpdateVelocity(velocity, physicsDeltatime);
    }
};



struct Renderer {
public:
    Renderer(int game_width, int game_height) 
        : gameWidth(game_width), gameHeight(game_height) {
            startButton = {gameWidth/2.0f - 100.0f, gameHeight/2.0f - 25, 200, 50};
        }

    void Render(Solver& solver) {
        BeginDrawing();
        ClearBackground(RAYWHITE);
        RenderBackground();

        if (!gameStarted) {RenderStartButton(gameWidth, gameHeight);}

        RenderObjects(solver);
        EndDrawing();
    }

    bool StartButtonContained(Vector2 mousePos) {
        return CheckCollisionPointRec(mousePos, startButton);
    }

    void StartGame() {
        gameStarted = true;
    }

private:
    int gameWidth;
    int gameHeight;
    Rectangle startButton;
    bool gameStarted = false;

    void RenderObjects(const Solver& solver) const {
        const auto& objects = solver.objects;
        for (auto& obj : objects) {
            DrawCircle(obj.currPosition.x, obj.currPosition.y, obj.radius, obj.color);
        }
    }

    void RenderBackground() {
        DrawRectangle(0, 0, gameWidth, gameHeight, BLACK);
    }

    void RenderStartButton(int width, int height) {
        DrawRectangleRec(startButton, GRAY);
        DrawRectangleLinesEx(startButton, 2, WHITE);

        const char* text = "START GAME";
        int textWidth = MeasureText(text, 20);
        DrawText(text, startButton.x + (startButton.width - textWidth)/2, startButton.y + 15, 20, BLACK);
    }
};



struct Game {
private:
    Solver solver;
    Renderer renderer;
    int worldWidth;
    int worldHeight;
    std::queue<SpawnCommand> spawnQueue;
    float lastSpawnedTime = 0.0f;

    int32_t lastRopeSegmentIndex = -1;

    int32_t selectedObjectIndex = -1;
    Vector2 dragOffset = {0.0f, 0.0f};
    bool isDragging = false;

public:
    Game(int world_width, int world_height)
        : worldWidth(world_width), worldHeight(world_height), solver(Solver({float(world_width), float(world_height)})), renderer(Renderer(world_width, world_height)) {}

    void QueueRope(int32_t length, Vector2 startPos, float radius, float spawnDelay) {
        int32_t ropeBodyID = solver.bodyCount++;
        const float segmentSpacing = radius * 2.2f;

        for (int i = 0; i < length; i++) {
            Vector2 segmentPos = {startPos.x, startPos.y + i * segmentSpacing};
            bool isFixed = (i == 0);
            Color defaultColor = isFixed ? RED : BLUE;

            SpawnCommand cmd(SpawnCommand::ROPE_SEGMENT, segmentPos, radius, defaultColor, spawnDelay, ropeBodyID, i, isFixed);
            AddSpawnCommand(cmd);
        }
    }

    void AddSpawnCommand(SpawnCommand command) {
        spawnQueue.push(command);
    }
    
    void ProcessSpawnQueue() {
        if (spawnQueue.empty()) {return;}
        SpawnCommand& cmd = spawnQueue.front();

        if (GetTime() - lastSpawnedTime < cmd.spawnDelay) {return;}

        if (cmd.type == SpawnCommand::FREE_OBJECT) {
            SpawnFreeObject(cmd.pos, cmd.spawnAngle, cmd.radius, cmd.defaultColor, cmd.speed);
        } else if (cmd.type == SpawnCommand::ROPE_SEGMENT) {
            ProcessRopeSegment(cmd);
        }

        spawnQueue.pop();
        lastSpawnedTime = GetTime();
    }
    
    void SpawnFreeObject(Vector2 pos, float spawnAngle, float radius, Color defaultColor, float speed) {
        Vector2 angle = {cos(spawnAngle), sin(spawnAngle)};
        VerletObject& obj = solver.AddObject(pos, radius, defaultColor);
        Vector2 velocity = Vector2Scale(angle, speed);
        solver.SetObjectVelocity(obj, velocity);
    }

    void ProcessRopeSegment(SpawnCommand cmd) {
        VerletObject& obj = solver.AddObject(cmd.pos, cmd.radius, cmd.defaultColor, cmd.isFixed, cmd.bodyID);

        int32_t currentIndex = solver.objects.size() - 1;

        if (cmd.segmentIndex > 0 && lastRopeSegmentIndex != -1) {
            float maxLength = cmd.radius * 2.0f;
            float minLength = cmd.radius * 1.5f;
            solver.AddConstraint(lastRopeSegmentIndex, currentIndex, maxLength, minLength);
        }
        lastRopeSegmentIndex = currentIndex;
    }

    void HandleMouseInput() {
        Vector2 mousePos = GetMousePosition();
        
        if (IsMouseButtonPressed(MOUSE_BUTTON_LEFT)) {
            selectedObjectIndex = FindObjectAtPosition(mousePos);
            if (selectedObjectIndex == -1) {return;}
            VerletObject& obj = solver.objects[selectedObjectIndex];
            if (obj.fixed) {
                isDragging = true;
                dragOffset = Vector2Subtract(mousePos, obj.currPosition);
                obj.defaultColor = GREEN;
            }
        }

        if (IsMouseButtonDown(MOUSE_BUTTON_LEFT) && isDragging && selectedObjectIndex != -1) {
            VerletObject& obj = solver.objects[selectedObjectIndex];
            Vector2 newPos = Vector2Subtract(mousePos, dragOffset);
            obj.currPosition = newPos;
            obj.lastPosition = newPos;
        }

        if (IsMouseButtonReleased(MOUSE_BUTTON_LEFT)) {
            if (isDragging && selectedObjectIndex != 1) {
                solver.objects[selectedObjectIndex].defaultColor = RED;
            }
            isDragging = false;
            selectedObjectIndex = -1;
        }
    }

    int32_t FindObjectAtPosition(Vector2 pos) {
        for (int32_t i = 0; i < static_cast<int32_t>(solver.objects.size()); i++) {
            const VerletObject& obj = solver.objects[i];
            float distance = Vector2Distance(pos, obj.currPosition);
            if (distance <=obj.radius) {
                return i;
            }
        }
        return -1;
    }

    void MainLoop() {
        SetTraceLogLevel(LOG_WARNING);
        InitWindow(worldWidth, worldHeight, "postHuman");
        
        bool gameStarted = false;
        
        QueueRope(20, {500.0f, 200.0f}, 10.0f, 0.01f);

        for (int i = 0; i < 10; i++) {
            SpawnCommand cmd = SpawnCommand({50.0f, 50.0f}, 10.0f, WHITE, 2.0f * PI, 700.0f, 0.15f);
            AddSpawnCommand(cmd);
        }
        for (int i = 0; i < 10; i++) {
            SpawnCommand cmd = SpawnCommand({50.0f, 50.0f}, 20.0f, WHITE, 2.0f * PI, 300.0f, 0.8f);
            AddSpawnCommand(cmd);
        }
        // SpawnCommand leftCmd = SpawnCommand({50.0f, 150.0f}, 0.0f, 20.0f, 9000.0f, 0.001f);
        // AddSpawnCommand(leftCmd);
        // SpawnCommand rightCmd = SpawnCommand({1000.0f, 150.0f}, PI, 20.0f, 9000.0f, 0.001f);
        // AddSpawnCommand(rightCmd);

        while (!WindowShouldClose() && !gameStarted) {
            renderer.Render(solver);

            if (renderer.StartButtonContained(GetMousePosition()) && IsMouseButtonPressed(MOUSE_BUTTON_LEFT)) {
                gameStarted = true;
                renderer.StartGame();
            }
        }

        while (!WindowShouldClose()) {
            HandleMouseInput();
            ProcessSpawnQueue();
            solver.UpdateNaive();
            renderer.Render(solver);
        }
        CloseWindow();
        // printf("Size of Constraints: %u\n", solver.constraints.size());
    }
};
