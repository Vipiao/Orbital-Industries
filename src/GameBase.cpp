// GameBase.cpp
#include "GameBase.h"
#include <iostream>
#include <algorithm>

Grid::Grid(GameBase* gameBase, const glm::dvec3& position, const glm::dquat& orientation)
    : m_gameBase(gameBase)
{
    // Load cube mesh and store texture units
    m_meshId = m_gameBase->graphicsEngine->loadModel(
        "../media/blender/00_cube.obj",
        "../media/color_512x512_occluded.png",
        "../media/normal_combined_512x512.png",
        false,  // ignoreTextureCoordinates
        &m_colorTextureUnit,  // Pass the address to store color texture unit
        &m_normalTextureUnit  // Pass the address to store normal texture unit
    );
    
    if (m_meshId < 0) {
        std::cerr << "Failed to create mesh for grid" << std::endl;
        return;
    }
    
    // Create a rigid body in the physics engine
    m_rigidBodyId = m_gameBase->physicsEngine->addRigidBody(
        position,
        orientation,
        1.0,  // Mass
        0.4   // Moment of inertia (approximated for a cube)
    );
    
    // Set initial graphics state
    updateGraphics();
}

Grid::~Grid() {
    if (m_meshId >= 0) {
        m_gameBase->graphicsEngine->removeMesh(m_meshId);
    }
    
    if (m_rigidBodyId >= 0) {
        m_gameBase->physicsEngine->removeRigidBody(m_rigidBodyId);
    }
}

void Grid::updateGraphics() {
    if (m_meshId < 0 || m_rigidBodyId < 0) {
        return;
    }
    
    PhysicsEngine::RigidBody* body = m_gameBase->physicsEngine->getRigidBody(m_rigidBodyId);
    if (!body) {
        return;
    }

    glm::dvec3 angVelAxis = body->angularVelocity;
    double angVelMagnitude = glm::length(angVelAxis);
    if (angVelMagnitude > 0.00001) {
        angVelAxis = angVelAxis / angVelMagnitude;
    } else {
        // If angular velocity is effectively zero, use a safe default axis
        angVelAxis = glm::dvec3(0.0, 0.0, 1.0);
        angVelMagnitude = 0.0;
    }
    
    m_gameBase->graphicsEngine->updateMeshTransform(
        m_meshId,
        body->position,
        body->velocity,
        body->orientation,
        angVelAxis,
        angVelMagnitude,
        glm::dvec3(0.0, 0.0, 0.0),
        m_colorTextureUnit,
        m_normalTextureUnit,
        m_gameBase->physicsEngine->getCurrentPhysicsTimeStep()
    );
}

GameBase::GameBase(int screenWidth, int screenHeight, const std::string& windowTitle) {
    graphicsEngine = std::make_unique<GraphicsEngine>(
        screenWidth, 
        screenHeight, 
        windowTitle
    );
    
    physicsEngine = std::make_unique<PhysicsEngine>();
    
    graphicsEngine->addCallbackObject(this);
    
    m_lastFrameTime = std::chrono::high_resolution_clock::now();
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
    auto grid = std::make_unique<Grid>(this, position, orientation);
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
    auto currentTime = std::chrono::high_resolution_clock::now();
    auto deltaTime = std::chrono::duration<double>(currentTime - m_lastFrameTime).count();
    m_lastFrameTime = currentTime;
    
    processInput();
    
    if (currentTime >= m_nextPhysicsTime) {
        updatePhysics();
        m_nextPhysicsTime += std::chrono::duration_cast<std::chrono::high_resolution_clock::duration>(
            std::chrono::duration<double>(m_physicsTimeStep));
    }
    
    update(deltaTime);
    
    for (auto& grid : grids) {
        grid->updateGraphics();
    }
}

void GameBase::renderCallback(glm::dmat4 viewMatrix, glm::dmat4 projectionMatrix) {
    // Convert double precision matrices to float precision
    glm::mat4 view = glm::mat4(viewMatrix);
    glm::mat4 projection = glm::mat4(projectionMatrix);

    // Calculate time remainder directly here where it's needed
    auto currentTime = std::chrono::high_resolution_clock::now();
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