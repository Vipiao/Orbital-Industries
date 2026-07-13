// GameBase.h
#pragma once

// GraphicsEngineBase.h is needed for the Mode enum in the constructor signature.
#include "graphics/GraphicsEngineBase.h"
#include "utils/IHashable.h"
#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>
#include <string>
#include <vector>
#include <memory>
#include <chrono>

// Subsystems are held by pointer — include their headers where you use them.
class GraphicsEngine;
class PhysicsEngine;
class JobManager;
class Job;
class GridSubsystem;
class CharacterSubsystem;
class CockpitDockingCoordinator;
class Grid;
class TimeHandler;
class DebugRenderer;
class Digibot;

class GameBase : public IHashable {
public:
    GameBase(int screenWidth = 800, int screenHeight = 600,
             const std::string& windowTitle = "Game", 
             TimeHandler* timeHandler = nullptr,
             GraphicsEngineBase::Mode controlMode = GraphicsEngineBase::Mode::NONE);
    virtual ~GameBase();
    
    std::weak_ptr<Grid> createGrid(const glm::dvec3& position, const glm::dquat& orientation = glm::dquat(1.0, 0.0, 0.0, 0.0));
    void removeGrid(std::weak_ptr<Grid> grid);
    std::weak_ptr<Digibot> createDigibot();

    // Resumable frame advance. Call in a loop until FrameDone; the other
    // statuses yield to the caller at a control point, exactly once each per
    // frame/step. Run the control, then call again. While yielded the world is
    // at a clean boundary and may be mutated.
    enum class FrameStatus {
        AwaitingFrameControl,  // input is polled; run per-frame control
        AwaitingStepControl,   // a physics step is at its control point,
                               // before integration; run per-step control
        FrameDone,
    };
    FrameStatus advanceFrame();

    void scheduleGridSplitCheck(std::weak_ptr<Grid> sourceGrid, const std::vector<glm::ivec3>& edgeCoords);
    
    // Shader reloading
    std::pair<bool, std::string> reloadShaders();

    // Count of completed physics ticks; increments only at clean step
    // boundaries. Frame-level consumers (network sync) compare it across
    // frames to detect completed ticks.
    uint64_t getPhysicsTick() const;
    // True while a physics step is parked mid-step across frames. World state
    // must not be mutated externally while this holds.
    bool isPhysicsStepInProgress() const { return m_physicsUpdateInProgress; }
    // True while advanceFrame is yielded at AwaitingStepControl. Per-step
    // world mutation (mode control, network state apply) is legal only then.
    bool isAtStepControlPoint() const {
        return m_physicsUpdateInProgress &&
               m_physicsUpdateState == PhysicsUpdateState::STEP_CONTROL;
    }

    std::unique_ptr<GraphicsEngine> m_graphicsEngine;
    std::unique_ptr<PhysicsEngine> m_physicsEngine;
    std::unique_ptr<JobManager> m_jobManager;
    std::unique_ptr<GridSubsystem> m_gridSubsystem;
    std::unique_ptr<CharacterSubsystem> m_characterSubsystem;
    // Cockpit docking is world physics: it runs every step regardless of mode
    std::unique_ptr<CockpitDockingCoordinator> m_cockpitDockingCoordinator;
    TimeHandler* m_timeHandler;

    // Debug support
    void setDebugRenderer(DebugRenderer* debugRenderer);
    DebugRenderer* getDebugRenderer() const { return m_debugRenderer; }

    // Subsystem access
    GridSubsystem* getGridSubsystem() const { return m_gridSubsystem.get(); }
    
    // IHashable interface
    virtual size_t computeHash() const override;
    
protected:
    void beginFrame();
    void render();
    void prepareFrame();

    enum class StepResult {
        OUT_OF_TIME,        // frame budget exhausted; call again to continue
        AWAITING_CONTROL,   // at the step's control point; issued once per step
        DONE,               // step completed at a clean boundary
    };
    virtual StepResult updatePhysics(
        std::chrono::time_point<std::chrono::high_resolution_clock> endTime);

private:
    // Sets up the frame's physics budget (lag discard, step counter).
    void beginPhysicsWindow();

    // Runs due physics steps within the frame budget.
    enum class PhysicsWindowResult {
        YIELDED_FOR_CONTROL,  // a step is at its control point; the caller
                              // must surface AwaitingStepControl and re-enter
        WINDOW_DONE,          // no more physics this frame: all due steps ran,
                              // none were due, or a step parked on the budget
    };
    PhysicsWindowResult advancePhysicsWindow();

    static constexpr int s_maxStepsPerFrame{4};

    std::chrono::time_point<std::chrono::high_resolution_clock> m_lastFrameTime;
    std::chrono::time_point<std::chrono::high_resolution_clock> m_nextPhysicsTime;
    std::chrono::time_point<std::chrono::high_resolution_clock> m_currentFrameStartTime;
    std::chrono::time_point<std::chrono::high_resolution_clock> m_targetFrameEnd;
    std::chrono::time_point<std::chrono::high_resolution_clock> m_workEndTime;
    double m_physicsTimeError{0.0}; // Track scheduling error for interpolation
    double m_physicsTimeStep{}; // Is set in constructor.
    int m_stepsThisFrame{0};

    // Resumable state for advanceFrame across calls within one frame.
    enum class FramePhase {
        FRAME_BEGIN,    // next call polls input and yields for frame control
        FRAME_CONTROL,  // frame control ran; next call renders and runs physics
        PHYSICS         // physics catch-up in progress (may span yields)
    };
    FramePhase m_framePhase{FramePhase::FRAME_BEGIN};

    // Flag to track if physics update is in progress
    bool m_physicsUpdateInProgress{false};

    // Resumable state for updatePhysics. Grid splits and control code can both
    // destroy grids/colliders and apply forces, so they must run exactly once
    // per physics step and only at a clean step boundary -- never re-run when
    // the physics engine has parked mid-step (which would free a collider
    // still referenced by this step's collision records, or double-apply
    // forces depending on frame budget).
    enum class PhysicsUpdateState {
        SPLITS,        // draining m_gridSubsystem->handlePendingSplits
        STEP_CONTROL,  // yielded to the caller's control; world control next
        PHYSICS        // running m_physicsEngine->runUntil, then graphics publish
    };
    PhysicsUpdateState m_physicsUpdateState{PhysicsUpdateState::SPLITS};

    DebugRenderer* m_debugRenderer = nullptr;
};

