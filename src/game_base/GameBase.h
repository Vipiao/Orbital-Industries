// GameBase.h
#pragma once

#include "../graphics/GraphicsEngine.h"
#include "../graphics/GraphicsEngineBase.h"
#include "../physics/RigidBody.h"
#include "../physics/PhysicsEngine.h"
#include "../utils/IHashable.h"
#include "GridSubsystem.h"
#include "../characters/CharacterSubsystem.h"
#include "../utils/Generator.h"
#include <vector>
#include <memory>
#include <chrono>

// Forward declaration
class TimeHandler;
class DebugRenderer;
class Digibot;

class GameBase : public IHashable {
public:
    class Callback {
    public:
        virtual ~Callback() = default;
        virtual void onPhysicsUpdateComplete() = 0;
    };
    GameBase(int screenWidth = 800, int screenHeight = 600, 
             const std::string& windowTitle = "Game", 
             TimeHandler* timeHandler = nullptr,
             GraphicsEngineBase::Mode controlMode = GraphicsEngineBase::Mode::NONE);
    virtual ~GameBase();
    
    std::weak_ptr<Grid> createGrid(const glm::dvec3& position, const glm::dquat& orientation = glm::dquat(1.0, 0.0, 0.0, 0.0));
    void removeGrid(std::weak_ptr<Grid> grid);
    std::weak_ptr<Digibot> createDigibot();

    void beginFrame();
    void render();
    void endFrame();

    void addPhysicsCallback(Callback* callback);

    void scheduleGridSplitCheck(std::weak_ptr<Grid> sourceGrid, const std::vector<glm::ivec3>& edgeCoords);
    
    // Shader reloading
    std::pair<bool, std::string> reloadShaders();

    std::unique_ptr<GraphicsEngine> m_graphicsEngine;
    std::unique_ptr<PhysicsEngine> m_physicsEngine;
    std::unique_ptr<JobManager> m_jobManager;
    std::unique_ptr<GridSubsystem> m_gridSubsystem;
    std::unique_ptr<CharacterSubsystem> m_characterSubsystem;
    TimeHandler* m_timeHandler;
    std::vector<Callback*> m_callbacks;

    // Track pending jobs for cleanup
    std::vector<std::weak_ptr<Job>> m_pendingJobs;

    // Debug support
    void setDebugRenderer(DebugRenderer* debugRenderer);
    DebugRenderer* getDebugRenderer() const { return m_debugRenderer; }

    // Subsystem access
    GridSubsystem* getGridSubsystem() const { return m_gridSubsystem.get(); }
    
    // IHashable interface
    virtual size_t computeHash() const override;
    
protected:
    void prepareFrame();
    void finalizeFrame();

    virtual bool updatePhysics(std::chrono::time_point<std::chrono::high_resolution_clock> endTime);

    // Helper to track job handles
    void trackJob(std::weak_ptr<Job> jobHandle);

private:
    
    std::chrono::time_point<std::chrono::high_resolution_clock> m_lastFrameTime;
    std::chrono::time_point<std::chrono::high_resolution_clock> m_nextPhysicsTime;
    std::chrono::time_point<std::chrono::high_resolution_clock> m_currentFrameStartTime;
    double m_physicsTimeError{0.0}; // Track scheduling error for interpolation
    double m_physicsTimeStep{}; // Is set in constructor.

    // Flag to track if physics update is in progress
    bool m_physicsUpdateInProgress{false};

    // Resumable state for updatePhysics. Grid splits and physics callbacks can
    // both destroy grids/colliders, so they must run exactly once per physics
    // step and only at a clean step boundary -- never re-run when the physics
    // engine has parked mid-step (which would free a collider still referenced
    // by this step's collision records).
    enum class PhysicsUpdateState {
        SPLITS,     // draining m_gridSubsystem->handlePendingSplits
        CALLBACKS,  // running onPhysicsUpdateComplete callbacks (once)
        PHYSICS     // running m_physicsEngine->runUntil
    };
    PhysicsUpdateState m_physicsUpdateState{PhysicsUpdateState::SPLITS};

    DebugRenderer* m_debugRenderer = nullptr;
};

