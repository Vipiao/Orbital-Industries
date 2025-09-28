// GameBase.h
#pragma once

#include "../graphics/GraphicsEngine.h"
#include "../graphics/GraphicsEngineBase.h"
#include "../graphics/GraphicsCallbacks.h"
#include "../graphics/CallbackManager.h"
#include "../physics/RigidBody.h"
#include "../physics/PhysicsEngine.h"
#include "../utils/IHashable.h"
#include "../utils/HashFunctions.h"
#include "Grid.h"
#include "../utils/Generator.h"
#include <vector>
#include <memory>
#include <chrono>

// Forward declaration
class TimeHandler;
class DebugRenderer;

class GameBase : public IGraphicsCallbacks, public CallbackManager, public IHashable {
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
    void run();
    void addPhysicsCallback(Callback* callback);

    // Grid partitioning/splitting - now deferred
    void scheduleGridSplitCheck(std::weak_ptr<Grid> sourceGrid, const std::vector<glm::ivec3>& edgeCoords);
    
    // Shader reloading
    std::pair<bool, std::string> reloadShaders();

    std::unique_ptr<GraphicsEngine> m_graphicsEngine;
    std::unique_ptr<PhysicsEngine> m_physicsEngine;
    std::unique_ptr<JobManager> m_jobManager;
    std::vector<std::shared_ptr<Grid>> m_grids;
    TimeHandler* m_timeHandler;
    std::vector<Callback*> m_callbacks;

    // Track pending jobs for cleanup
    std::vector<std::weak_ptr<Job>> m_pendingJobs;

    // Debug support
    void setDebugRenderer(DebugRenderer* debugRenderer);
    DebugRenderer* getDebugRenderer() const { return m_debugRenderer; }
    
    // IGraphicsCallbacks implementation
    virtual void preRenderCallback(uint64_t frameNum) override;
    virtual void renderCallback(glm::dmat4 viewMatrix, glm::dmat4 projectionMatrix) override;
    virtual void postRenderCallback(uint64_t frameNum) override;
    virtual void framebufferSizeCallback(int width, int height) override;
    virtual void windowPosCallback(int xpos, int ypos) override;
    
    // IHashable interface
    virtual size_t computeHash() const override;
    
protected:
    virtual void processInput();
    virtual bool updatePhysics(std::chrono::time_point<std::chrono::high_resolution_clock> endTime);
    virtual void update(double deltaTime);

    // Helper to track job handles
    void trackJob(std::weak_ptr<Job> jobHandle);

private:
    // Deferred grid splitting
    std::unordered_map<uint64_t, std::unordered_set<glm::ivec3, Hash::IVec3Hash>> m_pendingGridSplits;
    bool handlePendingSplits(std::chrono::time_point<std::chrono::high_resolution_clock> endTime);
    Generator<bool> handlePendingSplitsAsync();
    Generator<bool> performGridSplitAsync(Grid* sourceGrid, const std::vector<glm::ivec3>& edgeCoords);

    // Async state management
    std::unique_ptr<Generator<bool>> m_pendingSplitsGenerator;
    
    std::chrono::time_point<std::chrono::high_resolution_clock> m_lastFrameTime;
    std::chrono::time_point<std::chrono::high_resolution_clock> m_nextPhysicsTime;
    std::chrono::time_point<std::chrono::high_resolution_clock> m_currentFrameStartTime;
    double m_physicsTimeStep{}; // Is set in constructor.

    DebugRenderer* m_debugRenderer = nullptr;
};

