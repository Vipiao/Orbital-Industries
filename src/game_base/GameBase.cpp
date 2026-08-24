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
#include "reaction_wheel/ReactionWheelControl.h"
#include "../characters/digibot/Digibot.h"
#include "../characters/digibot/DigibotController.h"
#include "utils/TimeHandler.h"
#include "debug/DebugRenderer.h"
#include "../debug/FrameProfiler.h"
#include <algorithm>
#include <cassert>
#include <fstream>
#include <iomanip>
#include <iostream>
#include "../game_base/JobPriorities.h"

// Per-tick world-state journal for replay equivalence checks: every body's
// world-space center of mass, orientation, velocity and angular momentum, in
// engine order, at full precision so two runs can be diffed numerically.
static constexpr bool k_stateLogEnabled{false};

static void logWorldState(const PhysicsEngine& physics) {
    static std::ofstream s_file{"state_log.txt", std::ios::trunc};
    s_file << std::setprecision(17);
    s_file << "tick " << physics.getCurrentPhysicsTimeStep() << '\n';
    for (const std::weak_ptr<RigidBody>& bodyWeak : physics.getRigidBodies()) {
        std::shared_ptr<RigidBody> body{bodyWeak.lock()};
        if (!body) {
            continue;
        }
        const glm::dvec3 com{body->getWorldCenterOfMass()};
        const glm::dquat& ori{body->getOrientation()};
        const glm::dvec3& vel{body->m_velocity};
        const glm::dvec3& ang{body->getAngularMomentumBody()};
        s_file << "com " << com.x << ' ' << com.y << ' ' << com.z
               << " ori " << ori.w << ' ' << ori.x << ' ' << ori.y << ' ' << ori.z
               << " vel " << vel.x << ' ' << vel.y << ' ' << vel.z
               << " L " << ang.x << ' ' << ang.y << ' ' << ang.z
               << " m " << body->getMass() << '\n';
    }
}

GameBase::GameBase(
    int screenWidth, int screenHeight, const std::string& windowTitle,
    TimeHandler* timeHandler,
    GraphicsEngineBase::Mode controlMode,
    const std::filesystem::path& controlRecordingDir)
    : m_timeHandler(timeHandler)
{
    if (!m_timeHandler) {
        throw std::runtime_error("TimeHandler cannot be null");
    }

    m_graphicsEngine = std::make_unique<GraphicsEngine>(
        m_timeHandler,
        screenWidth, 
        screenHeight, 
        windowTitle,
        10000000,        // maxTriangles
        10000,          // maxMeshes
        controlMode,
        controlRecordingDir
    );

    // After the engine, which is what owns the GL context its queries live in.
    m_frameProfiler = std::make_unique<FrameProfiler>();
    if (m_frameProfiler->isEnabled()) {
        // A throttled swap would put the display's pacing inside the timings.
        m_graphicsEngine->getGraphicsEngineBase()->setSwapInterval(0);
    }

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

    m_nextPhysicsTime = m_timeHandler->now() +
        std::chrono::duration_cast<std::chrono::high_resolution_clock::duration>(
            std::chrono::duration<double>(m_physicsTimeStep));

    int refreshRate = m_graphicsEngine->getMonitorRefreshRate();
    
    m_physicsTimeStep = 1.0 / static_cast<double>(m_physicsEngine->getPhysicsHz());

    std::cout << "Display refresh rate: " << refreshRate << " Hz" << std::endl;
    std::cout << "Physics update rate: " << m_physicsEngine->getPhysicsHz() << " Hz" << std::endl;
}

GameBase::~GameBase() = default;

void GameBase::setDebugRenderer(DebugRenderer* debugRenderer) {
    m_debugRenderer = debugRenderer;
    
    // Pass debug pointer down to subsystems
    if (m_physicsEngine) {
        m_physicsEngine->setDebugRenderer(debugRenderer);
    }
}

uint64_t GameBase::getPhysicsTick() const {
    return m_physicsEngine->getCurrentPhysicsTimeStep();
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

void GameBase::applySplit(uint64_t sourceGridId, const std::vector<GridSplitPiece>& pieces) {
    m_gridSubsystem->applySplit(sourceGridId, pieces);
}

void GameBase::applyStructural(const StructuralCommand& command) {
    switch (command.m_op) {
        case StructuralOp::SetColor: {
            std::shared_ptr<Grid> grid{
                m_gridSubsystem->getGridById(command.m_gridId).lock()};
            if (grid) {
                grid->setColor(command.m_coord, command.m_color);
            }
            break;
        }
        case StructuralOp::AddCell: {
            std::shared_ptr<Grid> grid{
                m_gridSubsystem->getGridById(command.m_gridId).lock()};
            if (grid) {
                grid->addCell(command.m_coord, command.m_vertices, command.m_color);
            }
            break;
        }
        case StructuralOp::RemoveCell:
            removeCell(command.m_gridId, command.m_coord);
            break;
        case StructuralOp::ModifyCell:
            modifyCell(command.m_gridId, command.m_coord, command.m_cornerIndex,
                       command.m_direction);
            break;
        case StructuralOp::AddThruster: {
            std::shared_ptr<Grid> grid{
                m_gridSubsystem->getGridById(command.m_gridId).lock()};
            if (grid) {
                grid->addThruster(command.m_coord, command.m_orientation);
            }
            break;
        }
        case StructuralOp::AddCockpit: {
            std::shared_ptr<Grid> grid{
                m_gridSubsystem->getGridById(command.m_gridId).lock()};
            if (grid) {
                grid->addCockpit(command.m_coord, command.m_orientation);
            }
            break;
        }
        case StructuralOp::AddReactionWheel: {
            std::shared_ptr<Grid> grid{
                m_gridSubsystem->getGridById(command.m_gridId).lock()};
            if (grid) {
                grid->addReactionWheel(command.m_coord, command.m_orientation);
            }
            break;
        }
        case StructuralOp::SplitGrid:
            applySplit(command.m_gridId, command.m_pieces);
            break;
        case StructuralOp::SpawnGrid: {
            std::shared_ptr<Grid> grid{
                m_gridSubsystem->createGrid(command.m_gridId, command.m_position).lock()};
            if (grid) {
                switch (command.m_cellType) {
                    case CellType::THRUSTER:
                        grid->addThruster(glm::ivec3{0, 0, 0}, command.m_orientation);
                        break;
                    case CellType::COCKPIT:
                        grid->addCockpit(glm::ivec3{0, 0, 0}, command.m_orientation);
                        break;
                    case CellType::REACTION_WHEEL:
                        grid->addReactionWheel(glm::ivec3{0, 0, 0}, command.m_orientation);
                        break;
                    default:
                        grid->addCell(glm::ivec3{0, 0, 0}, command.m_vertices,
                                      command.m_color);
                        break;
                }
            }
            break;
        }
        case StructuralOp::DespawnGrid:
            m_gridSubsystem->despawnGrid(command.m_gridId);
            break;
    }
}

std::vector<StructuralCommand> GameBase::resolveStructuralEdit(StructuralCommand& command) {
    std::vector<StructuralCommand> followUps{};
    switch (command.m_op) {
        case StructuralOp::RemoveCell: {
            std::vector<glm::ivec3> removed{removeCell(command.m_gridId, command.m_coord)};
            std::shared_ptr<Grid> grid{
                m_gridSubsystem->getGridById(command.m_gridId).lock()};
            if (grid && grid->isEmpty()) {
                m_gridSubsystem->despawnGrid(command.m_gridId);
                followUps.push_back(StructuralCommand::despawnGrid(command.m_gridId));
            } else if (grid) {
                scheduleSplitCheck(command.m_gridId, removed);
            }
            break;
        }
        case StructuralOp::ModifyCell:
            if (modifyCell(command.m_gridId, command.m_coord, command.m_cornerIndex,
                           command.m_direction)) {
                scheduleSplitCheck(command.m_gridId, {command.m_coord});
            }
            break;
        case StructuralOp::SpawnGrid:
            command.m_gridId = m_gridSubsystem->allocateGridId();
            applyStructural(command);
            break;
        default:
            applyStructural(command);
            break;
    }
    return followUps;
}

std::vector<GridSplitResult> GameBase::drainSplitResults() {
    return m_gridSubsystem->drainCompletedSplits();
}

std::vector<glm::ivec3> GameBase::removeCell(uint64_t gridId, const glm::ivec3& coord) {
    return m_gridSubsystem->removeCell(gridId, coord);
}

void GameBase::scheduleSplitCheck(uint64_t gridId, const std::vector<glm::ivec3>& seedCoords) {
    m_gridSubsystem->scheduleSplitCheck(gridId, seedCoords);
}

bool GameBase::modifyCell(uint64_t gridId, const glm::ivec3& coord, int cornerIndex,
                          const glm::ivec3& direction) {
    return m_gridSubsystem->modifyCell(gridId, coord, cornerIndex, direction);
}

GameBase::FrameStatus GameBase::advanceFrame() {
    switch (m_framePhase) {
    case FramePhase::FRAME_BEGIN:
        beginFrame();
        m_framePhase = FramePhase::FRAME_CONTROL;
        return FrameStatus::AwaitingFrameControl;

    case FramePhase::FRAME_CONTROL:
        render();
        beginPhysicsWindow();
        m_framePhase = FramePhase::PHYSICS;
        [[fallthrough]];

    case FramePhase::PHYSICS:
        if (advancePhysicsWindow() == PhysicsWindowResult::YIELDED_FOR_CONTROL) {
            return FrameStatus::AwaitingStepControl;
        }

        // Background jobs get whatever frame budget remains.
        m_jobManager->work(m_workEndTime);

        if (m_timeHandler->now() >= m_targetFrameEnd) {
            std::cout << "Frame drop" << std::endl;
        }
        m_graphicsEngine->endFrame();
        m_framePhase = FramePhase::FRAME_BEGIN;
        return FrameStatus::FrameDone;
    }

    return FrameStatus::FrameDone; // Unreachable; keeps the compiler happy.
}

void GameBase::beginFrame() {
    m_graphicsEngine->beginFrame();
    prepareFrame();
}

void GameBase::render() {
    // Around the scene only: the swap sits in endFrame, and a query spanning it
    // would time the wait for the display rather than the drawing.
    m_frameProfiler->beginFrame();
    m_graphicsEngine->render();
    m_frameProfiler->endFrame();
}

void GameBase::prepareFrame() {
    if (!m_timeHandler) {
        throw std::runtime_error("TimeHandler cannot be null");
    }
    // The engine opened the frame and stamped its start; physics schedules
    // against that same instant rather than sampling the clock a second time.
    m_currentFrameStartTime = m_graphicsEngine->getFrameStartTime();

    auto timeSinceLastPhysics = std::chrono::duration<double>(
        m_currentFrameStartTime - m_physicsEngine->getLastPhysicsStepTime()).count();
    
    // Adjust time remainder based on scheduling error. Left unclamped, so a frame
    // that runs long extrapolates past the last physics step rather than freezing
    // at it.
    double adjustedTimeSincePhysics = timeSinceLastPhysics + m_physicsTimeError;
    double timeRemainder = adjustedTimeSincePhysics / m_physicsTimeStep;

    uint64_t currentTimeStep = m_physicsEngine->getCurrentPhysicsTimeStep();
    m_graphicsEngine->setRenderParameters(
        currentTimeStep,
        timeRemainder
    );

    // Update characters pre-render
    m_characterSubsystem->framePreRenderAll(m_graphicsEngine->getFrameNum(), timeRemainder);
}

void GameBase::nudgePhysicsSchedule(double ticks) {
    m_nextPhysicsTime += std::chrono::duration_cast<
        std::chrono::high_resolution_clock::duration>(
        std::chrono::duration<double>(ticks * m_physicsTimeStep));
}

void GameBase::beginPhysicsWindow() {
    if (!m_timeHandler) {
        throw std::runtime_error("TimeHandler cannot be null");
    }

    // Paces work against the display, so it wants the mode's rate: deriving the
    // budget from the measured rate would feed back on itself, a slow frame
    // granting a larger budget that makes the next frame slower still.
    double targetFrameDuration =
        1.0 / static_cast<double>(m_graphicsEngine->getMonitorRefreshRate());
    m_targetFrameEnd = m_currentFrameStartTime +
        std::chrono::duration_cast<std::chrono::high_resolution_clock::duration>(
            std::chrono::duration<double>(targetFrameDuration));
    m_workEndTime = m_targetFrameEnd - std::chrono::milliseconds(2);
    m_stepsThisFrame = 0;

    // Discard physics debt beyond s_maxStepsPerFrame steps to prevent catch-up
    // after lag spikes.
    auto lagCap = std::chrono::duration_cast<std::chrono::high_resolution_clock::duration>(
        std::chrono::duration<double>(m_physicsTimeStep * s_maxStepsPerFrame));
    if (m_timeHandler->now() > m_nextPhysicsTime + lagCap) {
        m_nextPhysicsTime = m_timeHandler->now() - lagCap;
    }
}

GameBase::PhysicsWindowResult GameBase::advancePhysicsWindow() {
    // Runs up to s_maxStepsPerFrame physics steps per frame as direct,
    // budgeted, resumable calls; a step that exhausts the budget parks and
    // resumes next frame. This keeps simulation speed correct on displays
    // below physicsHz.
    while (m_stepsThisFrame < s_maxStepsPerFrame) {
        auto currentTime = m_timeHandler->now();

        if (currentTime >= m_nextPhysicsTime && !m_physicsUpdateInProgress) {
            m_physicsTimeError = std::chrono::duration<double>(currentTime - m_nextPhysicsTime).count();
            m_physicsUpdateInProgress = true;
            m_nextPhysicsTime += std::chrono::duration_cast<std::chrono::high_resolution_clock::duration>(
                std::chrono::duration<double>(m_physicsTimeStep));
        }
        if (!m_physicsUpdateInProgress) {
            return PhysicsWindowResult::WINDOW_DONE;  // no step due
        }

        while (m_physicsUpdateInProgress && m_timeHandler->now() < m_workEndTime) {
            if (updatePhysics(m_workEndTime) == StepResult::AWAITING_CONTROL) {
                return PhysicsWindowResult::YIELDED_FOR_CONTROL;
            }
        }

        if (m_physicsUpdateInProgress) {
            // Step didn't finish in the budget; parked until next frame.
            return PhysicsWindowResult::WINDOW_DONE;
        }
        m_stepsThisFrame++;
        if (m_timeHandler->now() < m_nextPhysicsTime) {
            return PhysicsWindowResult::WINDOW_DONE;  // no more steps due
        }
    }
    return PhysicsWindowResult::WINDOW_DONE;  // step cap reached
}

GameBase::StepResult GameBase::updatePhysics(
    std::chrono::time_point<std::chrono::high_resolution_clock> endTime) {
    // Resumable state machine
    switch (m_physicsUpdateState) {
        case PhysicsUpdateState::SPLITS:
            // Drain pending grid splits
            if (m_gridSubsystem->handlePendingSplits(endTime)) {
                return StepResult::OUT_OF_TIME; // Splitting needs more time
            }
            // Yield exactly once per step: the caller injects its control
            // (mode, network) here, at a clean boundary -- splits done,
            // integration not started.
            m_physicsUpdateState = PhysicsUpdateState::STEP_CONTROL;
            return StepResult::AWAITING_CONTROL;

        case PhysicsUpdateState::STEP_CONTROL:
            // The caller's control ran; world control follows, exactly once
            // per step, so the runUntil below integrates it with minimal
            // input latency.
            assert(m_physicsUpdateInProgress);

            // Cockpit docking runs on the last completed step's collisions,
            // right before the character controllers consume their docking
            // targets.
            m_cockpitDockingCoordinator->stepControl(
                m_characterSubsystem.get(), m_physicsEngine.get(),
                m_gridSubsystem.get());

            // Seated pilots command their grid's thrusters and reaction wheels.
            // A burn persists with no one in the seat until the next command
            // overwrites it; the wheels instead recompute every step on every
            // grid, so an empty seat reads as no rotation asked for and they
            // bring the ship to rest.
            m_cockpitDockingCoordinator->forEachSeatedPilot(
                [](Digibot& digibot, Grid& grid, const CockpitBlock& cockpit) {
                    DigibotController* controller{digibot.getController()};
                    if (controller) {
                        ThrusterControl::setPilotCommand(
                            grid, cockpit, controller->getMovementDirection());
                        ReactionWheelControl::setPilotCommand(
                            grid, cockpit, controller->getRotationCommand());
                    }
                });
            for (const std::shared_ptr<Grid>& grid : m_gridSubsystem->getGrids()) {
                ThrusterControl::applyThrustForces(m_physicsEngine.get(), *grid);
                ReactionWheelControl::stepControl(m_physicsEngine.get(), *grid);
            }

            m_characterSubsystem->stepControlAll();

            // Docking transitions the controllers' physics asked for this step,
            // applied before integration; the coordinator owns the state machine.
            m_cockpitDockingCoordinator->applyDesiredTransitions();

            m_physicsUpdateState = PhysicsUpdateState::PHYSICS;
            [[fallthrough]];

        case PhysicsUpdateState::PHYSICS:
            // Run physics engine
            if (m_physicsEngine->runUntil(endTime)) {
                return StepResult::OUT_OF_TIME; // Step needs more time
            }

            if (k_stateLogEnabled) {
                logWorldState(*m_physicsEngine);
            }

            // Publish the new step's state to graphics so rendering
            // interpolates toward it.
            m_gridSubsystem->stepUpdateGraphicsAll(m_graphicsEngine->getCamPos());
            m_characterSubsystem->stepUpdateGraphicsAll();

            // Clear the in-progress flag and reset for the next step.
            m_physicsUpdateInProgress = false;
            m_physicsUpdateState = PhysicsUpdateState::SPLITS;
            return StepResult::DONE;
    }

    return StepResult::DONE; // Unreachable; keeps the compiler happy.
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