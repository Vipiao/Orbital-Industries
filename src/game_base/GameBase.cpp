// GameBase.cpp
#include "GameBase.h"
#include "GridSubsystem.h"
#include "../utils/TimeHandler.h"
#include "../debug/DebugRenderer.h"
#include <iostream>
#include <algorithm>
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

    // Create grid subsystem
    m_gridSubsystem = std::make_unique<GridSubsystem>(
        m_physicsEngine.get(),
        m_graphicsEngine.get(),
        m_jobManager.get(),
        m_timeHandler
    );

    // Create character subsystem
    m_characterSubsystem = std::make_unique<CharacterSubsystem>(
        m_physicsEngine.get(),
        m_graphicsEngine.get(),
        m_jobManager.get(),
        m_timeHandler,
        JobPriorities::GRID_CELL_CLASSIFICATION
    );

    if (!m_timeHandler) {
        throw std::runtime_error("TimeHandler cannot be null");
    }
    
    m_lastFrameTime = m_timeHandler->now();
    m_nextPhysicsTime = m_lastFrameTime + 
        std::chrono::duration_cast<std::chrono::high_resolution_clock::duration>(
            std::chrono::duration<double>(m_physicsTimeStep));
    
    int refreshRate = m_graphicsEngine->getFrameRate();
    
    m_physicsTimeStep = 1.0 / static_cast<double>(m_physicsEngine->getPhysicsHz());

    std::cout << "Display refresh rate: " << refreshRate << " Hz" << std::endl;
    std::cout << "Physics update rate: " << m_physicsEngine->getPhysicsHz() << " Hz" << std::endl;
}

GameBase::~GameBase() {
    // Cancel all pending jobs
    for (auto& jobHandle : m_pendingJobs) {
        if (!jobHandle.expired()) {
            m_jobManager->cancel(jobHandle);
        }
    }
}

void GameBase::setDebugRenderer(DebugRenderer* debugRenderer) {
    m_debugRenderer = debugRenderer;
    
    // Pass debug pointer down to subsystems
    if (m_physicsEngine) {
        m_physicsEngine->setDebugRenderer(debugRenderer);
    }
}

std::weak_ptr<Grid> GameBase::createGrid(const glm::dvec3& position, const glm::dquat& orientation) {
    return m_gridSubsystem->createGrid(position, orientation);
}

std::weak_ptr<Digibot> GameBase::createDigibot() {
    return m_characterSubsystem->createDigibot();
}

void GameBase::removeGrid(std::weak_ptr<Grid> gridWeak) {
    m_gridSubsystem->removeGrid(gridWeak);
}

void GameBase::scheduleGridSplitCheck(std::weak_ptr<Grid> sourceGridWeak, const std::vector<glm::ivec3>& edgeCoords) {
    m_gridSubsystem->scheduleGridSplitCheck(sourceGridWeak, edgeCoords);
}

void GameBase::addPhysicsCallback(Callback* callback) {
    m_callbacks.push_back(callback);
}

int hit_count = 0;

void GameBase::beginFrame() {
    m_graphicsEngine->beginFrame();
    prepareFrame();
}

void GameBase::render() {
    m_graphicsEngine->render();
}

void GameBase::endFrame() {
    finalizeFrame();
    m_graphicsEngine->endFrame();
}

void GameBase::prepareFrame() {
    if (!m_timeHandler) {
        throw std::runtime_error("TimeHandler cannot be null");
    }
    m_currentFrameStartTime = m_timeHandler->now();
    m_lastFrameTime = m_currentFrameStartTime;

    auto timeSinceLastPhysics = std::chrono::duration<double>(
        m_currentFrameStartTime - m_physicsEngine->getLastPhysicsStepTime()).count();
    
    // Adjust time remainder based on scheduling error
    double adjustedTimeSincePhysics = timeSinceLastPhysics + m_physicsTimeError;
    //double timeRemainder = std::clamp(adjustedTimeSincePhysics / m_physicsTimeStep, 0.0, 1.0);
    double timeRemainder = adjustedTimeSincePhysics / m_physicsTimeStep;

    // Set render parameters in graphics engine
    uint64_t currentTimeStep = m_physicsEngine->getCurrentPhysicsTimeStep();
    m_graphicsEngine->setRenderParameters(
        currentTimeStep,
        timeRemainder
    );

    // Update characters pre-render
    m_characterSubsystem->updateAllPreRender(m_graphicsEngine->getFrameNum(), timeRemainder);
}

void GameBase::finalizeFrame() {
    if (!m_timeHandler) {
        throw std::runtime_error("TimeHandler cannot be null");
    }

    double targetFrameDuration = 1.0 / static_cast<double>(m_graphicsEngine->getFrameRate());
    auto targetFrameEnd = m_currentFrameStartTime +
        std::chrono::duration_cast<std::chrono::high_resolution_clock::duration>(
            std::chrono::duration<double>(targetFrameDuration));
    auto jobEndTime = targetFrameEnd - std::chrono::milliseconds(2);

    // Discard physics debt beyond maxStepsPerFrame steps to prevent catch-up after lag spikes.
    constexpr int maxStepsPerFrame = 4;
    {
        auto lagCap = std::chrono::duration_cast<std::chrono::high_resolution_clock::duration>(
            std::chrono::duration<double>(m_physicsTimeStep * maxStepsPerFrame));
        if (m_timeHandler->now() > m_nextPhysicsTime + lagCap) {
            m_nextPhysicsTime = m_timeHandler->now() - lagCap;
        }
    }

    // Schedule and run up to maxStepsPerFrame physics steps per frame.
    // This keeps simulation speed correct on displays below physicsHz.
    for (int step = 0; step < maxStepsPerFrame; ++step) {
        auto currentTime = m_timeHandler->now();

        if (currentTime >= m_nextPhysicsTime && !m_physicsUpdateInProgress) {
            m_physicsTimeError = std::chrono::duration<double>(currentTime - m_nextPhysicsTime).count();
            m_physicsUpdateInProgress = true;
            auto jobHandle = m_jobManager->schedule(
                [this](std::chrono::time_point<std::chrono::high_resolution_clock> endTime) -> bool {
                    return updatePhysics(endTime);
                }, JobPriorities::PHYSICS_UPDATE);
            trackJob(jobHandle);
            m_nextPhysicsTime += std::chrono::duration_cast<std::chrono::high_resolution_clock::duration>(
                std::chrono::duration<double>(m_physicsTimeStep));
        }

        m_jobManager->work(jobEndTime);

        if (m_physicsUpdateInProgress) break;  // step didn't finish in time budget
        if (m_timeHandler->now() < m_nextPhysicsTime) break;  // no more steps due
    }

    if (m_timeHandler->now() >= targetFrameEnd) {
        std::cout << "Frame drop" << std::endl;
    }
}

bool GameBase::updatePhysics(std::chrono::time_point<std::chrono::high_resolution_clock> endTime) {
    // Resumable state machine
    switch (m_physicsUpdateState) {
        case PhysicsUpdateState::SPLITS:
            // Drain pending grid splits
            if (m_gridSubsystem->handlePendingSplits(endTime)) {
                return true; // Grid splitting needs more time -- stay in SPLITS
            }
            m_physicsUpdateState = PhysicsUpdateState::CALLBACKS;
            [[fallthrough]];

        case PhysicsUpdateState::CALLBACKS:
            // Runs exactly once per physics step
            for (auto* callback : m_callbacks) {
                callback->onPhysicsUpdateComplete();
            }

            m_physicsUpdateState = PhysicsUpdateState::PHYSICS;
            [[fallthrough]];

        case PhysicsUpdateState::PHYSICS:
            // Run physics engine
            if (m_physicsEngine->runUntil(endTime)) {
                return true; // Physics step needs more time -- stay in PHYSICS
            }

            m_gridSubsystem->updateAllGraphics(m_graphicsEngine->getCamPos());
            m_characterSubsystem->updateAllPhysicsComplete();

            // Clear the in-progress flag and reset for the next step.
            m_physicsUpdateInProgress = false;
            m_physicsUpdateState = PhysicsUpdateState::SPLITS;
            return false;
    }

    return false; // Unreachable; keeps the compiler happy.
}

void GameBase::trackJob(std::weak_ptr<Job> jobHandle) {
    bool didPrint = false;
    for (size_t ii = 0; ii < m_pendingJobs.size(); ii++)
    {
        if (!m_pendingJobs[ii].expired())
        {
            if (!didPrint) {
                didPrint = true;
                std::cout << "Physics frame drop" << std::endl;
            }
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

size_t GameBase::computeHash() const {
    size_t hash = 0;
    
    // Hash physics timestep
    hash = Hash::combineHashes(hash, std::hash<uint64_t>{}(m_physicsEngine->getCurrentPhysicsTimeStep()));
    
    hash = Hash::combineHashes(hash, m_gridSubsystem->computeHash());
    
    return hash;
}

std::pair<bool, std::string> GameBase::reloadShaders() {
    auto [success, message] = m_graphicsEngine->reloadShaders();
    
    return {success, success ? "GameBase: " + message : "GameBase: " + message};
}