// GameBase.h
#pragma once

// GraphicsEngineBase.h is needed for the Mode enum in the constructor signature.
#include "graphics/GraphicsEngineBase.h"
#include "GridSplitPiece.h"
#include "StructuralCommand.h"
#include "utils/IHashable.h"
#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>
#include <array>
#include <string>
#include <vector>
#include <memory>
#include <chrono>
#include <utility>

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
    // Remove a cell and return the coords actually removed. Does not look for splits
    // or despawn an emptied grid.
    std::vector<glm::ivec3> removeCell(uint64_t gridId, const glm::ivec3& coord);
    // Schedule a split check seeded from the given coords and their neighbours.
    void scheduleSplitCheck(uint64_t gridId, const std::vector<glm::ivec3>& seedCoords);
    // Nudge one corner of a cell by a direction if the grid permits; returns whether
    // it did. Composes on the grid's current shape. Does not look for splits.
    bool modifyCell(uint64_t gridId, const glm::ivec3& coord, int cornerIndex,
                    const glm::ivec3& direction);
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

    // Realise a split from a decided piece list (e.g. a decoded command).
    void applySplit(uint64_t sourceGridId, const std::vector<GridSplitPiece>& pieces);
    // Splits this peer originated since the last drain, for the caller to relay.
    std::vector<GridSplitResult> drainSplitResults();
    
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

    // Shift the wall-clock instant the next physics step is due by this many ticks
    // of phase (fractional allowed), leaving the fixed timestep untouched. A client
    // uses it to run microscopically fast or slow and hold its tick phase aligned
    // with the server's. Positive delays the next step (runs slower).
    void nudgePhysicsSchedule(double ticks);

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

    // Realise a decoded structural command against the world exactly as given.
    // Missing targets are ignored (a command can outlive its grid in transit).
    // For replaying an already-decided stream; an originated edit goes through
    // resolveStructuralEdit.
    void applyStructural(const StructuralCommand& command);

    // Apply an originated edit and decide its consequences: a spawn gets a fresh
    // grid id (resolved into the command), removals and modifications seed split
    // checks, and a removal that empties a grid despawns it. Returns the follow-up
    // commands decided here (despawns) for the caller to relay; realised splits
    // arrive later through drainSplitResults.
    std::vector<StructuralCommand> resolveStructuralEdit(StructuralCommand& command);

    // Structural edits queued by tools this frame, drained and applied at a
    // controlled point rather than the instant a tool runs, so edit application
    // lives in one place.
    void requestStructuralEdit(const StructuralCommand& command) {
        m_pendingEdits.push_back(command);
    }
    std::vector<StructuralCommand> drainStructuralEdits() {
        return std::exchange(m_pendingEdits, {});
    }

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
    std::vector<StructuralCommand> m_pendingEdits;

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

