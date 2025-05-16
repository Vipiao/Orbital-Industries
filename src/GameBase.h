// GameBase.h
#pragma once

#include "GraphicsEngine.h"
#include "PhysicsEngine.h"
#include "TimeHandler.h"
#include "Grid.h"
#include <vector>
#include <memory>
#include <chrono>

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
    
    std::unique_ptr<GraphicsEngine> graphicsEngine;
    std::unique_ptr<PhysicsEngine> physicsEngine;
    std::vector<std::unique_ptr<Grid>> grids;
    TimeHandler* m_timeHandler; // New
    
    virtual void preRenderCallback(uint64_t frameNum) override;
    virtual void renderCallback(glm::dmat4 viewMatrix, glm::dmat4 projectionMatrix) override;
    virtual void framebufferSizeCallback(int width, int height) override;
    virtual void windowPosCallback(int xpos, int ypos) override;
    
protected:
    virtual void processInput();
    virtual void updatePhysics();
    virtual void update(double deltaTime);
    virtual void processGridGraphicsUpdates();
    
    std::chrono::time_point<std::chrono::high_resolution_clock> m_lastFrameTime;
    std::chrono::time_point<std::chrono::high_resolution_clock> m_nextPhysicsTime;
    double m_physicsTimeStep{}; // Is set in constructor.
};

