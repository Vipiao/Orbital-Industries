// GameBase.h
#pragma once

#include "../graphics/GraphicsEngine.h"
#include "../utils/JobManager.h"
#include "../physics/PhysicsEngine.h"
#include "../utils/TimeHandler.h"
#include "Grid.h"
#include <vector>
#include <memory>
#include <chrono>

// Forward declaration
class DebugRenderer;

class GameBase : public GraphicsEngine::CallBack {
public:
    GameBase(int screenWidth = 800, int screenHeight = 600, 
             const std::string& windowTitle = "Game", 
             TimeHandler* timeHandler = nullptr,
             GraphicsEngineBase::Mode controlMode = GraphicsEngineBase::Mode::NONE);
    virtual ~GameBase();
    
    Grid* createGrid(const glm::dvec3& position, const glm::dquat& orientation = glm::dquat(1.0, 0.0, 0.0, 0.0));
    void removeGrid(Grid* grid);
    void run();

    // Grid partitioning/splitting
    std::vector<Grid*> splitGrid(Grid* sourceGrid, const std::vector<glm::ivec3>& edgeCoords);
    
    std::unique_ptr<GraphicsEngine> m_graphicsEngine;
    std::unique_ptr<PhysicsEngine> m_physicsEngine;
    std::unique_ptr<JobManager> m_jobManager;
    std::vector<std::unique_ptr<Grid>> m_grids;
    TimeHandler* m_timeHandler;

    // Debug support
    void setDebugRenderer(DebugRenderer* debugRenderer);
    DebugRenderer* getDebugRenderer() const { return m_debugRenderer; }
    
    virtual void preRenderCallback(uint64_t frameNum) override;
    virtual void renderCallback(glm::dmat4 viewMatrix, glm::dmat4 projectionMatrix) override;
    virtual void framebufferSizeCallback(int width, int height) override;
    virtual void windowPosCallback(int xpos, int ypos) override;
    
protected:
    virtual void processInput();
    virtual bool updatePhysics(std::chrono::time_point<std::chrono::high_resolution_clock> endTime);
    virtual void update(double deltaTime);
    virtual void processGridGraphicsUpdates();
    
    std::chrono::time_point<std::chrono::high_resolution_clock> m_lastFrameTime;
    std::chrono::time_point<std::chrono::high_resolution_clock> m_nextPhysicsTime;
    double m_physicsTimeStep{}; // Is set in constructor.

private:
    DebugRenderer* m_debugRenderer = nullptr;
};

