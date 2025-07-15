// GameBase.cpp
#include "GameBase.h"
#include "../debug/DebugRenderer.h"
#include <iostream>
#include <algorithm>
#include "../utils/PartitionCalculator.h"

GameBase::GameBase(
    int screenWidth, int screenHeight, const std::string& windowTitle,
    TimeHandler* timeHandler,
    GraphicsEngineBase::Mode controlMode)
    : m_timeHandler(timeHandler)
{
    m_graphicsEngine = std::make_unique<GraphicsEngine>(
        screenWidth, 
        screenHeight, 
        windowTitle,
        10000000,        // maxTriangles
        10000,          // maxMeshes
        controlMode
    );

    m_physicsEngine = std::make_unique<PhysicsEngine>(m_timeHandler);

    // Create job manager
    m_jobManager = std::make_unique<JobManager>(m_timeHandler);

    if (!m_timeHandler) {
        throw std::runtime_error("TimeHandler cannot be null");
    }
    
    m_graphicsEngine->addCallbackObject(this);
    
    m_lastFrameTime = m_timeHandler->now();
    m_nextPhysicsTime = m_lastFrameTime + 
        std::chrono::duration_cast<std::chrono::high_resolution_clock::duration>(
            std::chrono::duration<double>(m_physicsTimeStep));
    
    int refreshRate = m_graphicsEngine->m_frameRate;
    
    m_physicsTimeStep = 1.0 / static_cast<double>(64);
    
    std::cout << "Display refresh rate: " << refreshRate << " Hz" << std::endl;
    std::cout << "Physics update rate: " << 64 << " Hz" << std::endl;
}

GameBase::~GameBase() {
    m_grids.clear();
    
    if (m_graphicsEngine) {
        m_graphicsEngine->removeCallbackObject(this);
    }
}

void GameBase::setDebugRenderer(DebugRenderer* debugRenderer) {
    m_debugRenderer = debugRenderer;
    
    // Pass debug pointer down to subsystems
    if (m_physicsEngine) {
        m_physicsEngine->setDebugRenderer(debugRenderer);
    }
}

Grid* GameBase::createGrid(const glm::dvec3& position, const glm::dquat& orientation) {
    auto grid = std::make_unique<Grid>(m_physicsEngine.get(), m_graphicsEngine.get(), m_jobManager.get(), m_timeHandler, position, orientation);
    Grid* gridPtr = grid.get();
    m_grids.push_back(std::move(grid));
    return gridPtr;
}

void GameBase::removeGrid(Grid* grid) {
    auto it = std::find_if(m_grids.begin(), m_grids.end(),
        [grid](const std::unique_ptr<Grid>& item) {
            return item.get() == grid;
        });
    
    if (it != m_grids.end()) {
        m_grids.erase(it);
    }
}

std::vector<Grid*> GameBase::checkAndSplitGrid(Grid* sourceGrid, const std::vector<glm::ivec3>& edgeCoords) {
    std::vector<Grid*> newGrids;

    struct PartitionPhysics {
        glm::dvec3 centerOfMass;
        glm::dvec3 velocity;
        double mass;
    };
    
    if (!sourceGrid || edgeCoords.empty()) {
        return newGrids;
    }
    
    // Step 1: Analyze partitions using PartitionCalculator
    auto result = PartitionCalculator<GridCell>::analyzePartitions(
        &sourceGrid->getCells(),
        edgeCoords,
        [](const GridCell& cell) {
            return cell.getConnectedNeighbors();
        }
    );
    
    // Step 2: If no split detected, return empty vector
    if (!result.hasSplit || result.partitions.size() <= 1) {
        return newGrids;
    }
    
    std::cout << "Grid split detected! " << result.partitions.size() << " partitions found." << std::endl;
    
    // Step 3: Pre-calculate physics properties for each partition
    PhysicsEngine::RigidBody* sourceBody = sourceGrid->getRigidBody();
    glm::dvec3 originalCenterOfMass = sourceBody->m_position;
    glm::dvec3 originalVelocity = sourceBody->m_velocity;
    glm::dvec3 originalAngularVelocity = sourceBody->m_angularVelocity;
    glm::dquat originalOrientation = sourceBody->m_orientation;
    
    std::vector<PartitionPhysics> partitionPhysics(result.partitions.size());
    const double blockMass = 60.0; // Match Grid.cpp value
    
    for (size_t i = 0; i < result.partitions.size(); ++i) {
        const std::vector<glm::ivec3>& partition = result.partitions[i];
        
        // Calculate center of mass for this partition
        glm::dvec3 weightedSum(0.0);
        double totalMass = 0.0;
        
        for (const glm::ivec3& coord : partition) {
            glm::dvec3 blockPosition = sourceGrid->gridToWorld(glm::dvec3(coord) + glm::dvec3(0.5)); // Block center in world space
            weightedSum += blockPosition * blockMass;
            totalMass += blockMass;
        }
        
        glm::dvec3 partitionCenterOfMass = weightedSum / totalMass;
        
        // Calculate velocity using rigid body kinematics: v_p = v_t + ω × (cm_p - cm_t)
        glm::dvec3 relativePosition = partitionCenterOfMass - originalCenterOfMass;
        glm::dvec3 partitionVelocity = originalVelocity + glm::cross(originalAngularVelocity, relativePosition);
        
        partitionPhysics[i] = {partitionCenterOfMass, partitionVelocity, totalMass};
        
        std::cout << "Partition " << i << ": " << partition.size() << " cells, mass=" << totalMass 
                  << ", CM=(" << partitionCenterOfMass.x << "," << partitionCenterOfMass.y << "," << partitionCenterOfMass.z << ")"
                  << ", vel=(" << partitionVelocity.x << "," << partitionVelocity.y << "," << partitionVelocity.z << ")" << std::endl;
    }
    
    // Step 4: Find the largest partition (this stays with the original grid)
    size_t largestPartitionIndex = 0;
    size_t largestSize = result.partitions[0].size();
    
    for (size_t i = 1; i < result.partitions.size(); ++i) {
        if (result.partitions[i].size() > largestSize) {
            largestSize = result.partitions[i].size();
            largestPartitionIndex = i;
        }
    }
    
    std::cout << "Largest partition (index " << largestPartitionIndex << ") has " << largestSize << " cells" << std::endl;
    
    // Step 5: Create new grids for all partitions except the largest
    
    for (size_t i = 0; i < result.partitions.size(); ++i) {
        if (i == largestPartitionIndex) {
            continue; // Skip largest partition - it stays with original grid
        }
        
        const std::vector<glm::ivec3>& partition = result.partitions[i];
        
        // Create new grid - position will be set after adding cells
        Grid* newGrid = createGrid(glm::dvec3(0.0), originalOrientation);
        
        std::cout << "Moving " << partition.size() << " cells to new grid" << std::endl;
        
        // Move cells from source grid to new grid
        for (const glm::ivec3& cellCoord : partition) {
            // Get cell type before removing (assume ARMOR for now, could be extended)
            CellType cellType = CellType::ARMOR;
            
            // Remove from source grid and add to new grid
            sourceGrid->removeCell(cellCoord);
            newGrid->addCell(cellCoord, cellType);
        }

        // Set physics properties using pre-calculated values
        PhysicsEngine::RigidBody* newBody = newGrid->getRigidBody();
        if (newBody) {
            // Transform center of mass to world space and set position
            glm::dvec3 worldCenterOfMass = partitionPhysics[i].centerOfMass;
            newBody->m_position = worldCenterOfMass;
            newBody->m_velocity = partitionPhysics[i].velocity;
            newBody->m_angularVelocity = originalAngularVelocity;
            newBody->m_orientation = originalOrientation;
        }
        
        newGrids.push_back(newGrid);
    }
    
    std::cout << "Created " << newGrids.size() << " new grids from split" << std::endl;
    
    return newGrids;
}

void GameBase::run() {
    m_graphicsEngine->startRenderLoop();
}

void GameBase::preRenderCallback(uint64_t frameNum) {
    if (!m_timeHandler) {
        throw std::runtime_error("TimeHandler cannot be null");
    }
    auto currentTime = m_timeHandler->now();
    auto deltaTime = std::chrono::duration<double>(currentTime - m_lastFrameTime).count();
    m_lastFrameTime = currentTime;
    
    processInput();
    
    // Schedule physics job if needed
    if (currentTime >= m_nextPhysicsTime) {
        // Schedule physics as a high-priority job
        m_jobManager->schedule([this](std::chrono::time_point<std::chrono::high_resolution_clock> endTime) -> bool {
            return updatePhysics(endTime);
        }, 0); // Priority 0 (high priority)
        //updatePhysics(currentTime + std::chrono::duration_cast<std::chrono::high_resolution_clock::duration>(
        //    std::chrono::duration<double>(9999.9)));
        
        m_nextPhysicsTime += std::chrono::duration_cast<std::chrono::high_resolution_clock::duration>(
            std::chrono::duration<double>(m_physicsTimeStep));
    }
    
    update(deltaTime);

    processGridGraphicsUpdates();
    
    for (auto& grid : m_grids) {
        grid->updateGraphics(m_graphicsEngine->m_camPos);
    }

    // Process jobs with remaining frame time
    double targetFrameDuration = 1.0 / static_cast<double>(m_graphicsEngine->m_frameRate);
    auto targetFrameEnd = currentTime  + 
        std::chrono::duration_cast<std::chrono::high_resolution_clock::duration>(
            std::chrono::duration<double>(targetFrameDuration));
    
    // Calculate end time with 1ms safety margin
    auto jobEndTime = targetFrameEnd - std::chrono::milliseconds(2);
    m_jobManager->work(jobEndTime);
}

void GameBase::renderCallback(glm::dmat4 viewMatrix, glm::dmat4 projectionMatrix) {
    // Convert double precision matrices to float precision
    glm::mat4 view = glm::mat4(viewMatrix);
    glm::mat4 projection = glm::mat4(projectionMatrix);

    // Calculate time remainder directly here where it's needed
    if (!m_timeHandler) {
        throw std::runtime_error("TimeHandler cannot be null");
    }
    auto currentTime = m_timeHandler->now();
    auto timeToNextPhysics = std::chrono::duration<double>(m_nextPhysicsTime - currentTime).count();
    double physicsTimeRemainder = 1.0 - (timeToNextPhysics / m_physicsTimeStep);
    
    //std::cout << "physics Time step " << physicsEngine->getCurrentPhysicsTimeStep() << std::endl;
    //std::cout << "reminder " << physicsTimeRemainder << std::endl;
    //std::cout << std::endl;

    // Render using MeshHandler's single-pass render method
    m_graphicsEngine->m_meshHandler->render(
        view, projection, 
        m_graphicsEngine->m_frameNum,     // frame number
        m_physicsEngine->getCurrentPhysicsTimeStep(),  // time in milliseconds
        physicsTimeRemainder,  // time remainder (fractional part)
        glm::dvec3(4.0, 4.0, 4.0),      // light position (fixed value or you can make this a member)
        m_graphicsEngine->m_camPos        // camera position
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

bool GameBase::updatePhysics(std::chrono::time_point<std::chrono::high_resolution_clock> endTime) {
    // Apply drag to all objects before running physics
    // (move existing drag code from MyGame here if you want)
    
    // Run physics engine
    return m_physicsEngine->runUntil(endTime);
}

void GameBase::update(double deltaTime) {
    // Base implementation does nothing
    // Override in derived classes for game-specific logic
}

void GameBase::processGridGraphicsUpdates() {
    for (auto& grid : m_grids) {
        if (grid->hasGraphicsUpdates()) {
            grid->processGraphicsQueue();
        }
    }
}