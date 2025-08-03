#pragma once
#include "raylib.h"
#include "raymath.h"
#include <vector>
#include <queue>
#include <memory>
#include <unordered_map>
#include <cmath>
#include <cstdint>
#include <stdexcept>
#include <cstdio>

constexpr int DEFAULT_SUBSTEPS = 8;
constexpr int JAKOBSEN_ITERATIONS = 10;
constexpr float DEFAULT_RADIUS = 10.0f;
constexpr float MAX_DAMPING_FACTOR = 0.9995f;
constexpr float MIN_DAMPING_FACTOR = 0.999f;
constexpr float DEFAULT_GRAVITY = 980.7f;
constexpr float MARGIN_WIDTH = 10.0f;
constexpr float RESPONSE_COEF = 0.5f;

struct Solver;

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
        int32_t objectsSize = static_cast<int32_t>(objects.size());
        if (obj1Index >= objectsSize || obj2Index >= objectsSize || obj1Index < 0 || obj2Index < 0) {
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

struct SpawnInstruction {
    virtual ~SpawnInstruction() = default;
    virtual void Execute(Solver& solver, Vector2 startPos) = 0;
};

struct SpawnCommand {
    enum BodyType { FREE_OBJECT, ROPE, TENTACLE };
    BodyType bodyType;
    Vector2 startPos;
    float spawnDelay;
    std::unique_ptr<SpawnInstruction> instruction; // polymorphic

    SpawnCommand(BodyType body_type, Vector2 start_position, float spawn_delay, std::unique_ptr<SpawnInstruction> instructions_)
        : bodyType(body_type), startPos(start_position), spawnDelay(spawn_delay), instruction(std::move(instructions_)) {}

    SpawnCommand(SpawnCommand&& other) noexcept
        : bodyType(other.bodyType), startPos(other.startPos), 
        spawnDelay(other.spawnDelay), instruction(std::move(other.instruction)) {}

    SpawnCommand& operator=(SpawnCommand&& other) noexcept {
        if (this != & other) {
            bodyType = other.bodyType;
            startPos = other.startPos;
            spawnDelay = other.spawnDelay;
            instruction = std::move(other.instruction);
        }
        return *this;
    }

    SpawnCommand(const SpawnCommand&) = delete;
    SpawnCommand& operator=(const SpawnCommand&) = delete;
};

struct FreeObjectInstruction : public SpawnInstruction {
    float radius;
    Color color;
    float spawnAngle;
    float speed;

    FreeObjectInstruction(float radius_, Color color_, float angle_, float speed_)
        : radius(radius_), color(color_), spawnAngle(angle_), speed(speed_) {}

    std::unique_ptr<FreeObjectInstruction> Clone() const {
        return std::make_unique<FreeObjectInstruction>(radius, color, spawnAngle, speed);
    }

    void Execute(Solver& solver, Vector2 startPos) override;
};

struct RopeInstruction : public SpawnInstruction {
    int32_t bodyID;
    int32_t length;
    float radius;
    Color fixedColor;
    Color defaultColor;
    float segmentSpacing;

    RopeInstruction(int32_t body_id, int32_t length_, float radius_, Color fixed_color, Color default_color, float segment_spacing)
        : bodyID(body_id), length(length_), radius(radius_), fixedColor(fixed_color), defaultColor(default_color), segmentSpacing(segment_spacing) {}

    void Execute(Solver& solver, Vector2 startPos) override;
};

struct TentacleInstruction : public SpawnInstruction {
    int32_t bodyID;             // The tentacle body's ID
    int32_t length;             // Number of objects per side (1 additional object acting as the tip)
    float radius;               // Radius of the individual Objects
    Color anchorColor;          // Color of the anchored obejcts, usually attached to the main body or acting as a base.
    Color controlColor;         // Color of the tentacle controlling objects
    Color defaultColor;         // Default color of the objects
    float overlapScalar;        // Scalar for how much overlap each object has. >= 2 means no overlap, 1 means object center is on edge of next objects
    float baseWidth;            // Distance the two objects acting as a base will be apart
    int32_t controlPoints;      // Number of control points that each tentacle will get.

    TentacleInstruction(int32_t body_id, int32_t length_, float radius_, Color anchor_color, Color default_color, Color control_color,
        float overlap_scalar, float base_width, int32_t control_points)
        : bodyID(body_id), length(length_), radius(radius_), anchorColor(anchor_color), defaultColor(default_color), controlColor(control_color),
        overlapScalar(overlap_scalar), baseWidth(base_width), controlPoints(control_points) {
            ValidateParameters(length, radius, overlapScalar, baseWidth);
            if (controlPoints > length - 1) {controlPoints = length - 1;}
        }
    
    void Execute(Solver& solver, Vector2 startPos) override;

    static void ValidateParameters(int32_t length, float radius, float overlapScalar, float baseWidth);

    void GenerateSupportConstraints(Solver& solver, std::vector<int32_t>& rightObjIndices, std::vector<int32_t>& leftObjIndices, 
    Vector2 rightSideSegmentPos, Vector2 leftSideSegmentPos, int32_t rightObjIndex, int32_t leftObjIndex, int32_t relativeAnchorPosition);
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

    // Returns the index of created object
    int32_t AddObject(Vector2 pos, float radius, Color color, bool fixed = false, int32_t bodyID = -1) {
        objects.emplace_back(pos, radius, fixed, color, bodyID);
        return static_cast<int32_t>(objects.size() - 1);
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

    void SetObjectVelocity(int32_t objIndex, Vector2 velocity) {
        VerletObject& obj = objects[objIndex];
        obj.UpdateVelocity(velocity, physicsDeltatime);
    }
};

// Implementation of Execute Methods after solver has been declared
void FreeObjectInstruction::Execute(Solver& solver, Vector2 startPos) {
    Vector2 angle = {cos(spawnAngle), sin(spawnAngle)};
    int32_t objIndex = solver.AddObject(startPos, radius, color);
    Vector2 velocity = Vector2Scale(angle, speed);
    solver.SetObjectVelocity(objIndex, velocity);
}

void RopeInstruction::Execute(Solver& solver, Vector2 startPos) {
    int32_t lastRopeSegmentIndex = -1;
    
    for (int i = 0; i < length; i++) {
        Vector2 segmentPos = {startPos.x, startPos.y + i * segmentSpacing};
        bool isFixed = (i == 0);
        Color color = isFixed ? fixedColor : defaultColor;

        int32_t objIndex = solver.AddObject(segmentPos, radius, color, isFixed, bodyID);

        int32_t currentIndex = static_cast<int32_t>(solver.objects.size()) - 1;

        if (i > 0 && lastRopeSegmentIndex != -1) {
            float maxLength = radius * 2.0f;
            float minLength = radius * 1.5f;
            solver.AddConstraint(lastRopeSegmentIndex, currentIndex, maxLength, minLength);
        }
        lastRopeSegmentIndex = currentIndex;
    }
}

void TentacleInstruction::ValidateParameters(int32_t length, float radius, float overlapScalar, float baseWidth) {
    if (length <= 0) {
        throw std::invalid_argument("Tentacle length must be positive");
    }
    if (radius <= 0) {
        throw std::invalid_argument("Tentacle radius must be positive");
    }
    if (baseWidth <= 0) {
        throw std::invalid_argument("Tentacle baseWidth must be positive");
    }
    if (overlapScalar <= 0) {
        throw std::invalid_argument("Tentacle overlapScalar must be positive");
    }

    float halfBaseWidth = baseWidth / 2;
    float sideLength = length * radius * overlapScalar;

    if (sideLength <= halfBaseWidth) {
        char errorMsg[256];
        snprintf(errorMsg, sizeof(errorMsg),
                "Invalid tentacle geometry: sideLength (%.2f) must be greater than halfBaseWidth (%.2f). "
                "Current parameters: length=%d, radius=%.2f, overlapScalar=%.2f, baseWidth=%.2f",
                sideLength, halfBaseWidth, length, radius, overlapScalar, baseWidth);
        throw std::invalid_argument(errorMsg);
    }
}

void TentacleInstruction::Execute(Solver& solver, Vector2 startPos) {

    std::vector<int32_t> leftObjIndices;
    std::vector<int32_t> rightObjIndices;

    float maxLength = radius * overlapScalar * 1.05f;
    float minLength = radius * overlapScalar * 0.95f;
    
    float halfBaseWidth = baseWidth / 2;
    float sideLength = length * radius * overlapScalar;

    float tipOffsetY = sqrt(sideLength * sideLength - halfBaseWidth * halfBaseWidth);

    Vector2 leftSideStart = {startPos.x - halfBaseWidth, startPos.y};
    Vector2 rightSideStart = {startPos.x + halfBaseWidth, startPos.y};
    Vector2 tentacleTipPos = {startPos.x, startPos.y - tipOffsetY};

    Vector2 leftSideNormal = Vector2Normalize(Vector2Subtract(tentacleTipPos, leftSideStart));
    Vector2 rightSideNormal = Vector2Normalize(Vector2Subtract(tentacleTipPos, rightSideStart));

    for (int i = 0; i < length; i++) {
        Color color = defaultColor;
        bool isFixed = false;

        if (i == 0) {
            color = anchorColor;
            isFixed = true;
        }

        float segmentScale = sideLength * (float(i) / float(length));
        Vector2 leftSideSegmentPos = Vector2Add(leftSideStart, Vector2Scale(leftSideNormal, segmentScale));
        Vector2 rightSideSegmentPos = Vector2Add(rightSideStart, Vector2Scale(rightSideNormal, segmentScale));

        int32_t leftObjIndex = solver.AddObject(leftSideSegmentPos, radius, color, isFixed, bodyID);
        int32_t rightObjIndex = solver.AddObject(rightSideSegmentPos, radius, color, isFixed, bodyID);

        if (i > 0) {
            solver.AddConstraint(leftObjIndices[leftObjIndices.size() - 1], leftObjIndex, maxLength, minLength);
            solver.AddConstraint(rightObjIndices[rightObjIndices.size() - 1], rightObjIndex, maxLength, minLength);
        }

        for (int32_t j = 1; j < 4; j++) {
            if (leftObjIndices.size() > j && rightObjIndices.size() > j) {
                GenerateSupportConstraints(solver, rightObjIndices, leftObjIndices, rightSideSegmentPos, leftSideSegmentPos,
                                           rightObjIndex, leftObjIndex, j);
            }
        }

        float distance = Vector2Distance(leftSideSegmentPos, rightSideSegmentPos);
        solver.AddConstraint(leftObjIndex, rightObjIndex, distance * 1.1f, distance * 0.9f);

        leftObjIndices.push_back(leftObjIndex);
        rightObjIndices.push_back(rightObjIndex);

    }
}

void TentacleInstruction::GenerateSupportConstraints(Solver& solver, std::vector<int32_t>& rightObjIndices, std::vector<int32_t>& leftObjIndices, 
    Vector2 rightSideSegmentPos, Vector2 leftSideSegmentPos, int32_t rightObjIndex, int32_t leftObjIndex, int32_t relativeAnchorPosition) {        

        int32_t rightAnchorIndex = rightObjIndices[rightObjIndices.size() - (relativeAnchorPosition + 1)];
        float constraintDistanceRL = Vector2Distance(solver.objects[rightAnchorIndex].currPosition, leftSideSegmentPos);
        float maxDistanceRL = constraintDistanceRL * 1.05f;
        float minDistanceRL = constraintDistanceRL * 0.95f;
        solver.AddConstraint(rightAnchorIndex, leftObjIndex, maxDistanceRL, minDistanceRL);
        
        int32_t leftAnchorIndex = leftObjIndices[leftObjIndices.size() - (relativeAnchorPosition + 1)];
        float constraintDistanceLR = Vector2Distance(solver.objects[leftAnchorIndex].currPosition, rightSideSegmentPos);
        float maxDistanceLR = constraintDistanceLR * 1.05f;
        float minDistanceLR = constraintDistanceLR * 0.95f;
        solver.AddConstraint(leftAnchorIndex, rightObjIndex, maxDistanceLR, minDistanceLR);
}

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

struct Spawner {
    std::queue<SpawnCommand> spawnQueue;
    float lastSpawnedTime = 0.0f;

    void AddSpawnCommand(SpawnCommand cmd) {
        spawnQueue.push(std::move(cmd));
    }

    void ProcessSpawnQueue(Solver& solver) {
        if (spawnQueue.empty()) {return;}
        SpawnCommand& cmd = spawnQueue.front();

        if (GetTime() - lastSpawnedTime < cmd.spawnDelay) {return;}

        cmd.instruction->Execute(solver, cmd.startPos);

        spawnQueue.pop();
        lastSpawnedTime = GetTime();
    }
};

struct Game {
private:
    Solver solver;
    Spawner spawner;
    Renderer renderer;
    int worldWidth;
    int worldHeight;

    int32_t selectedObjectIndex = -1;
    Vector2 dragOffset = {0.0f, 0.0f};
    bool isDragging = false;

    std::unique_ptr<FreeObjectInstruction> streamInstruction;
    int32_t streamCount = 0;
    bool fpsLimitReached = false;

public:
    Game(int world_width, int world_height)
        : worldWidth(world_width), worldHeight(world_height), solver(Solver({float(world_width), float(world_height)})), 
        renderer(Renderer(world_width, world_height)), spawner(Spawner()) {
            streamInstruction = std::make_unique<FreeObjectInstruction>(
                10.0f,
                WHITE,
                2.0 * PI,
                700.0f
            );
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

            const float DRAG_SMOOTHING = 0.05f;
            obj.currPosition = Vector2Lerp(obj.currPosition, newPos, DRAG_SMOOTHING);
            // obj.lastPosition = obj.currPosition;
        }

        if (IsMouseButtonReleased(MOUSE_BUTTON_LEFT)) {
            if (isDragging && selectedObjectIndex != -1) {
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

    void HandleObjectStream() {
        if (GetFPS() < 60) { fpsLimitReached = true; }
        if (fpsLimitReached) { return; }
        if (spawner.spawnQueue.size() > 10) { return; }
        if (solver.objects.size() >= 100) { return; }
        SpawnObjectStream();
    }

    void SpawnObjectStream() {
        SpawnCommand streamCmd(
            SpawnCommand::FREE_OBJECT,
            {200.0f, 200.0f},
            0.05f,
            streamInstruction->Clone()
        );

        spawner.AddSpawnCommand(std::move(streamCmd));
        streamCount++;

    }

    void MainLoop() {
        SetTraceLogLevel(LOG_WARNING);
        InitWindow(worldWidth, worldHeight, "postHuman");
        
        bool gameStarted = false;

        auto ropeInstruction = std::make_unique<RopeInstruction>(
            solver.bodyCount++,
            20,
            10.0f,
            RED,
            BLUE,
            10.0f * 2.5f
        );
        SpawnCommand ropeCmd(
            SpawnCommand::ROPE,
            {500.0f, 200.0f},
            0.5f,
            std::move(ropeInstruction)
        );
        spawner.AddSpawnCommand(std::move(ropeCmd));

        auto rope2Instruction = std::make_unique<RopeInstruction>(
            solver.bodyCount++,
            1,
            15.0f,
            RED,
            BLUE,
            10.0f * 2.5f
        );
        SpawnCommand rope2Cmd(
            SpawnCommand::ROPE,
            {700.0f, 200.0f},
            0.5f,
            std::move(rope2Instruction)
        );
        spawner.AddSpawnCommand(std::move(rope2Cmd));

        auto tentacleInstruction = std::make_unique<TentacleInstruction>(
            solver.bodyCount++,
            23,
            10.0f,
            RED,
            GREEN,
            ORANGE,
            1.8f,
            100.0f,
            2
        );
        SpawnCommand tentacleCmd(
            SpawnCommand::TENTACLE,
            {700.0f, 700.0f},
            0.5f,
            std::move(tentacleInstruction)
        );
        spawner.AddSpawnCommand(std::move(tentacleCmd));

        while (!WindowShouldClose() && !gameStarted) {
            renderer.Render(solver);

            if (renderer.StartButtonContained(GetMousePosition()) && IsMouseButtonPressed(MOUSE_BUTTON_LEFT)) {
                gameStarted = true;
                renderer.StartGame();
            }
        }

        while (!WindowShouldClose()) {
            HandleMouseInput();
            HandleObjectStream();
            spawner.ProcessSpawnQueue(solver);
            solver.UpdateNaive();
            renderer.Render(solver);
        }
        CloseWindow();
    }
};
