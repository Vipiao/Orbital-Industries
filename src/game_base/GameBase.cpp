// GameBase.cpp
#include "GameBase.h"
#include "../debug/DebugRenderer.h"
#include <iostream>
#include <algorithm>
#include "../utils/PartitionCalculator.h"
#include "../game_base/JobPriorities.h"

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
    // Cancel all pending jobs
    for (auto& jobHandle : m_pendingJobs) {
        if (!jobHandle.expired()) {
            m_jobManager->cancel(jobHandle);
        }
    }

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
        // Remove any pending split operations for this grid
        auto pendingIt = m_pendingGridSplits.find(grid);
        if (pendingIt != m_pendingGridSplits.end()) {
            m_pendingGridSplits.erase(pendingIt);
        }
        m_grids.erase(it);
    }
}

void GameBase::scheduleGridSplitCheck(Grid* sourceGrid, const std::vector<glm::ivec3>& edgeCoords) {
    if (!sourceGrid || edgeCoords.empty()) {
        return;
    }
    
    // Add edge coordinates to pending splits, automatically deduplicating
    auto& pendingEdges = m_pendingGridSplits[sourceGrid];
    pendingEdges.insert(edgeCoords.begin(), edgeCoords.end());
}

bool GameBase::handlePendingSplits(std::chrono::time_point<std::chrono::high_resolution_clock> endTime) {
    if (!m_pendingSplitsGenerator) {
        if (m_pendingGridSplits.empty()) {
            return false; // No work to do
        }
        m_pendingSplitsGenerator = std::make_unique<Generator<bool>>(handlePendingSplitsAsync());
        ++(*m_pendingSplitsGenerator); // Start the generator
    }
    
    while (*m_pendingSplitsGenerator && m_timeHandler->now() < endTime) {
        ++(*m_pendingSplitsGenerator);
    }
    
    if (!*m_pendingSplitsGenerator) {
        // Generator finished
        m_pendingSplitsGenerator.reset();
        return false; // Work complete
    }
    
    return true; // More work needed
}

Generator<bool> GameBase::handlePendingSplitsAsync() {
    if (m_pendingGridSplits.empty()) {
        co_return;
    }
    
    // Snapshot pending splits to avoid race conditions
    auto pendingSplitsSnapshot = std::move(m_pendingGridSplits);
    // m_pendingGridSplits is now empty and ready for new entries

    // Process snapshotted splits
    for (const auto& pair : pendingSplitsSnapshot) {
        Grid* sourceGrid = pair.first;
        const auto& edgeCoords = pair.second;
        
        // Convert unordered_set back to vector for the analysis function
        std::vector<glm::ivec3> edgeVector(edgeCoords.begin(), edgeCoords.end());
        
        // Use async grid split generator
        auto splitGenerator = performGridSplitAsync(sourceGrid, edgeVector);
        ++splitGenerator; // Start the generator
        
        while (splitGenerator) {
            co_yield true; // Yield control for time checking
            ++splitGenerator;
        }
    }
}

Generator<bool> GameBase::performGridSplitAsync(Grid* sourceGrid, const std::vector<glm::ivec3>& edgeCoords) {
    std::vector<Grid*> newGrids;

    struct PartitionPhysics {
        glm::dvec3 centerOfMass;
        glm::dvec3 velocity;
        double mass;
    };
    
    if (!sourceGrid || edgeCoords.empty()) {
        co_return;
    }

    co_yield true; // Allow time check before expensive operation
    
    // Step 1: Analyze partitions using PartitionCalculator
    auto result = PartitionCalculator<StructuralBlock>::analyzePartitions(
        &sourceGrid->getCells(),
        edgeCoords,
        [](const StructuralBlock& cell) -> std::vector<glm::ivec3> {
            std::vector<glm::ivec3> neighbors;
            cell.forEachConnectedNeighbor([&](const glm::ivec3& neighbor) {
                neighbors.push_back(neighbor);
            });
            return neighbors;
        }
    );

    co_yield true; // Allow time check after expensive analysis
    
    // Step 2: If no split detected, return empty vector
    if (!result.hasSplit || result.partitions.size() <= 1) {
        co_return;
    }
    
    std::cout << "Grid split detected! " << result.partitions.size() << " partitions found." << std::endl;
    
    // Step 3: Pre-calculate physics properties for each partition
    RigidBody* sourceBody = sourceGrid->getRigidBody();
    glm::dvec3 originalCenterOfMass = sourceBody->m_position;
    glm::dvec3 originalVelocity = sourceBody->m_velocity;
    
    // Calculate angular velocity from angular momentum
    glm::dvec3 originalAngularVelocity = sourceBody->getAngularVelocityWorld();
    glm::dquat originalOrientation = sourceBody->m_orientation;
    
    std::vector<PartitionPhysics> partitionPhysics(result.partitions.size());
    const double blockMass = 60.0; // Match Grid.cpp value

    co_yield true; // Allow time check before physics calculations
    
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

    co_yield true; // Allow time check before grid operations
    
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

    co_yield true; // Allow time check before creating new grids
    
    for (size_t i = 0; i < result.partitions.size(); ++i) {
        if (i == largestPartitionIndex) {
            continue; // Skip largest partition - it stays with original grid
        }
        
        const std::vector<glm::ivec3>& partition = result.partitions[i];
        
        // Create new grid - position will be set after adding cells
        Grid* newGrid = createGrid(glm::dvec3(0.0), originalOrientation);
        
        std::cout << "Moving " << partition.size() << " cells to new grid" << std::endl;
        
        co_yield true; // Allow time check before moving cells

        // Move cells from source grid to new grid
        size_t cellsProcessed = 0;
        for (const glm::ivec3& cellCoord : partition) {
            // Get cell type before removing (assume ARMOR for now, could be extended)
            CellType cellType = CellType::ARMOR;
            
            // Remove from source grid and add to new grid
            sourceGrid->removeCell(cellCoord);
            newGrid->addCell(cellCoord, cellType);

            // Yield every 5 cells to avoid blocking too long
            if (++cellsProcessed % 5 == 0) {
                co_yield true;
            }
        }

        // Set physics properties using pre-calculated values
        RigidBody* newBody = newGrid->getRigidBody();
        if (newBody) {
            // Transform center of mass to world space and set position
            glm::dvec3 worldCenterOfMass = partitionPhysics[i].centerOfMass;
            newBody->m_position = worldCenterOfMass;
            newBody->m_velocity = partitionPhysics[i].velocity;
            
            // Set angular momentum instead of angular velocity
            glm::dvec3 angularVelocityBody = glm::conjugate(sourceBody->m_orientation) * originalAngularVelocity;
            newBody->setAngularVelocityBody(angularVelocityBody);
            newBody->m_orientation = originalOrientation;
        }
        
        newGrids.push_back(newGrid);
    }

    // Recalculate original partition local angular momentum as its mass has now changed.
    glm::dvec3 angularVelocityBody = glm::conjugate(sourceBody->m_orientation) * originalAngularVelocity;
    sourceBody->setAngularVelocityBody(angularVelocityBody);
    
    std::cout << "Created " << newGrids.size() << " new grids from split" << std::endl;
}

void GameBase::addCallback(Callback* callback) {
    m_callbacks.push_back(callback);
}

void GameBase::run() {
    m_graphicsEngine->startRenderLoop();
}

int hit_count = 0;

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
        auto jobHandle = m_jobManager->schedule([this](std::chrono::time_point<std::chrono::high_resolution_clock> endTime) -> bool {
            return updatePhysics(endTime);
        }, JobPriorities::PHYSICS_UPDATE);
        trackJob(jobHandle);
        //updatePhysics(currentTime + std::chrono::duration_cast<std::chrono::high_resolution_clock::duration>(
        //    std::chrono::duration<double>(9999.9)));
        
        m_nextPhysicsTime += std::chrono::duration_cast<std::chrono::high_resolution_clock::duration>(
            std::chrono::duration<double>(m_physicsTimeStep));
    }
    
    update(deltaTime);

    // Process jobs with remaining frame time
    double targetFrameDuration = 1.0 / static_cast<double>(m_graphicsEngine->m_frameRate);
    auto targetFrameEnd = currentTime  + 
        std::chrono::duration_cast<std::chrono::high_resolution_clock::duration>(
            std::chrono::duration<double>(targetFrameDuration));

    int hh = hit_count++;
    
    // Calculate end time with 1ms safety margin
    auto jobEndTime = targetFrameEnd - std::chrono::milliseconds(2);
    m_jobManager->work(jobEndTime);

    if(m_timeHandler->now() >= targetFrameEnd) {
        std::cout << "Frame drop" << std::endl;
    }
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
    // Handle any pending grid splits before running physics
    extern int hit_count;
    int hh = hit_count;
    if (handlePendingSplits(endTime)) {
        return true; // Grid splitting needs more time
    }

    // Apply drag to all objects before running physics
    // (move existing drag code from MyGame here if you want)
    
    // Run physics engine
    bool needsMoreTime = m_physicsEngine->runUntil(endTime);
    if (needsMoreTime)
    {
        return true;
    }
    
    // Update graphics only when physics step is complete
    for (auto& grid : m_grids) {
        grid->updateGraphics(m_graphicsEngine->m_camPos);
    }

    // Call physics update callbacks
    for (auto* callback : m_callbacks) {
        callback->onPhysicsUpdateComplete();
    }
    
    return false;
}

void GameBase::update(double deltaTime) {
    // Base implementation does nothing
    // Override in derived classes for game-specific logic
}

void GameBase::trackJob(std::weak_ptr<Job> jobHandle) {
    for (size_t ii = 0; ii < m_pendingJobs.size(); ii++)
    {
        if (!m_pendingJobs[ii].expired())
        {
            std::cout << "Physics frame drop" << std::endl;
            //extern int hit_count;
            //std::cout << "hit_count" << hit_count << std::endl;
        }
        
    }
    
    // Clean up expired handles periodically to prevent unbounded growth
    if (m_pendingJobs.size() % 20 == 0) {
        m_pendingJobs.erase(std::remove_if(m_pendingJobs.begin(), m_pendingJobs.end(),
            [](const std::weak_ptr<Job>& handle) { return handle.expired(); }), m_pendingJobs.end());
    }
    
    m_pendingJobs.push_back(jobHandle);
}
