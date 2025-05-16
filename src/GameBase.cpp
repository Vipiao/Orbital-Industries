// GameBase.cpp
#include "GameBase.h"
#include <iostream>
#include <algorithm>

GameBase::GameBase(
    int screenWidth, int screenHeight, const std::string& windowTitle,
    TimeHandler* timeHandler,
    GraphicsEngineBase::Mode controlMode)
    : m_timeHandler(timeHandler)
{
    graphicsEngine = std::make_unique<GraphicsEngine>(
        screenWidth, 
        screenHeight, 
        windowTitle,
        10000,        // maxTriangles
        100,          // maxMeshes
        controlMode
    );
    
    physicsEngine = std::make_unique<PhysicsEngine>();
    
    graphicsEngine->addCallbackObject(this);
    
    m_lastFrameTime = m_timeHandler ? m_timeHandler->now() : std::chrono::high_resolution_clock::now();
    m_nextPhysicsTime = m_lastFrameTime + 
        std::chrono::duration_cast<std::chrono::high_resolution_clock::duration>(
            std::chrono::duration<double>(m_physicsTimeStep));
    
    int refreshRate = graphicsEngine->m_frameRate;
    
    m_physicsTimeStep = 1.0 / static_cast<double>(16);
    
    std::cout << "Display refresh rate: " << refreshRate << " Hz" << std::endl;
    std::cout << "Physics update rate: " << 16 << " Hz" << std::endl;
}

GameBase::~GameBase() {
    grids.clear();
    
    if (graphicsEngine) {
        graphicsEngine->removeCallbackObject(this);
    }
}

Grid* GameBase::createGrid(const glm::dvec3& position, const glm::dquat& orientation) {
    auto grid = std::make_unique<Grid>(physicsEngine.get(), graphicsEngine.get(), position, orientation);
    Grid* gridPtr = grid.get();
    grids.push_back(std::move(grid));
    return gridPtr;
}

void GameBase::removeGrid(Grid* grid) {
    auto it = std::find_if(grids.begin(), grids.end(),
        [grid](const std::unique_ptr<Grid>& item) {
            return item.get() == grid;
        });
    
    if (it != grids.end()) {
        grids.erase(it);
    }
}

void GameBase::run() {
    graphicsEngine->startRenderLoop();
}

void GameBase::preRenderCallback(uint64_t frameNum) {
    auto currentTime = m_timeHandler ? m_timeHandler->now() : std::chrono::high_resolution_clock::now();
    auto deltaTime = std::chrono::duration<double>(currentTime - m_lastFrameTime).count();
    m_lastFrameTime = currentTime;
    
    processInput();
    
    if (currentTime >= m_nextPhysicsTime) {
        updatePhysics();
        m_nextPhysicsTime += std::chrono::duration_cast<std::chrono::high_resolution_clock::duration>(
            std::chrono::duration<double>(m_physicsTimeStep));
    }
    
    update(deltaTime);

    processGridGraphicsUpdates();
    
    for (auto& grid : grids) {
        grid->updateGraphics();
    }
}

void GameBase::renderCallback(glm::dmat4 viewMatrix, glm::dmat4 projectionMatrix) {
    // Convert double precision matrices to float precision
    glm::mat4 view = glm::mat4(viewMatrix);
    glm::mat4 projection = glm::mat4(projectionMatrix);

    // Calculate time remainder directly here where it's needed
    auto currentTime = m_timeHandler ? m_timeHandler->now() : std::chrono::high_resolution_clock::now();
    auto timeToNextPhysics = std::chrono::duration<double>(m_nextPhysicsTime - currentTime).count();
    double physicsTimeRemainder = 1.0 - (timeToNextPhysics / m_physicsTimeStep);
    
    //std::cout << "physics Time step " << physicsEngine->getCurrentPhysicsTimeStep() << std::endl;
    //std::cout << "reminder " << physicsTimeRemainder << std::endl;
    //std::cout << std::endl;

    // Render using MeshHandler's single-pass render method
    graphicsEngine->meshHandler->render(
        view, projection, 
        graphicsEngine->m_frameNum,     // frame number
        physicsEngine->getCurrentPhysicsTimeStep(),  // time in milliseconds
        physicsTimeRemainder,  // time remainder (fractional part)
        glm::dvec3(4.0, 4.0, 4.0),      // light position (fixed value or you can make this a member)
        graphicsEngine->m_camPos        // camera position
    );
}

void GameBase::framebufferSizeCallback(int width, int height) {
    // Nothing specific needed
}

void GameBase::windowPosCallback(int xpos, int ypos) {
    // Nothing specific needed
}

void GameBase::processInput() {
    // Base implementation does nothing
    // Override in derived classes for game-specific input handling
}

void GameBase::updatePhysics() {
    physicsEngine->run();
}

void GameBase::update(double deltaTime) {
    // Base implementation does nothing
    // Override in derived classes for game-specific logic
}

void GameBase::processGridGraphicsUpdates() {
    for (auto& grid : grids) {
        if (grid->hasGraphicsUpdates()) {
            grid->processGraphicsQueue();
        }
    }
}