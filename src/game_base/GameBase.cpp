// GameBase.cpp
#include "GameBase.h"
#include "graphics/GraphicsEngine.h"
#include "../physics/PhysicsEngine.h"
#include "../characters/CharacterSubsystem.h"
#include "utils/JobManager.h"
#include "utils/HashFunctions.h"
#include "Grid.h"
#include "GridSubsystem.h"
#include "cockpit/CockpitDockingCoordinator.h"
#include "thruster/ThrusterControl.h"
#include "../characters/digibot/Digibot.h"
#include "../characters/digibot/DigibotController.h"
#include "utils/TimeHandler.h"
#include "debug/DebugRenderer.h"
#include <iostream>
#include <algorithm>
#include "../game_base/JobPriorities.h"
#include "Mode.h"

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

    // Create cockpit docking coordinator (world-level, mode-independent)
    m_cockpitDockingCoordinator = std::make_unique<CockpitDockingCoordinator>();

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
    m_characterSubsystem->framePreRenderAll(m_graphicsEngine->getFrameNum(), timeRemainder);
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
            m_physicsUpdateState = PhysicsUpdateState::CONTROL;
            [[fallthrough]];

        case PhysicsUpdateState::CONTROL:
            // Runs exactly once per physics step. All input -> force/command
            // decisions happen here so the runUntil below integrates them,
            // minimizing input latency.
            if (m_mode) {
                m_mode->stepControl();
            }

            // Cockpit docking runs on the last completed step's collisions,
            // right before the character controllers consume their docking
            // targets.
            m_cockpitDockingCoordinator->stepControl(
                m_characterSubsystem.get(), m_physicsEngine.get(),
                m_gridSubsystem.get());

            // Seated pilots command their grid's thrusters, then the stored
            // throttles burn on every grid -- a burn persists with no one in
            // the seat until the next command overwrites it.
            m_cockpitDockingCoordinator->forEachSeatedPilot(
                [](Digibot& digibot, Grid& grid, const CockpitBlock& cockpit) {
                    DigibotController* controller{digibot.getController()};
                    if (controller) {
                        ThrusterControl::setPilotCommand(
                            grid, cockpit, controller->getMovementDirection());
                    }
                });
            for (const std::shared_ptr<Grid>& grid : m_gridSubsystem->getGrids()) {
                ThrusterControl::applyThrustForces(m_physicsEngine.get(), *grid);
            }

            m_characterSubsystem->stepControlAll();

            m_physicsUpdateState = PhysicsUpdateState::PHYSICS;
            [[fallthrough]];

        case PhysicsUpdateState::PHYSICS:
            // Run physics engine
            if (m_physicsEngine->runUntil(endTime)) {
                return true; // Physics step needs more time -- stay in PHYSICS
            }

            // Publish the new step's state to graphics so rendering
            // interpolates toward it.
            m_gridSubsystem->stepUpdateGraphicsAll(m_graphicsEngine->getCamPos());
            m_characterSubsystem->stepUpdateGraphicsAll();

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