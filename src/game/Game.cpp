#include "Game.h"

#include "GameNetwork.h"
#include "../debug/DebugVisualization.h"
#include "../game_base/Creative.h"
#include "../game_base/GameBase.h"
#include "../game_base/Grid.h"
#include "../network/INetworkTransport.h"
#include "../physics/RigidBody.h"
#include "graphics/GraphicsEngine.h"
#include "debug/DebugRenderer.h"

#include <cassert>
#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>
#include <iostream>

Game::Game(TimeHandler* timeHandler, GraphicsEngineBase::Mode controlMode,
           std::unique_ptr<INetworkTransport> transport)
    : m_transport{std::move(transport)} {
    assert(m_transport != nullptr && timeHandler != nullptr);

    m_gameBase = std::make_unique<GameBase>(800, 600, "3D Grid Demo", timeHandler,
                                            controlMode);
    m_gameNetwork = std::make_unique<GameNetwork>(m_transport.get(), m_gameBase.get());

    setupDebugVisualization();

    // Set global debug renderer with RAII guard
    m_debugGuard = DebugGlobals::setDebugRenderer(m_debugViz.get());

    // Set global GameBase for debugging
    DebugGlobals::g_gameBase = m_gameBase.get();

    // Create creative mode; Game drives it at advanceFrame's control points
    m_mode = std::make_unique<Creative>(m_gameBase.get());

    // Set up initial camera position and orientation
    m_gameBase->m_graphicsEngine->getCamPos() = glm::dvec3(0, 0, 0);
    m_gameBase->m_graphicsEngine->getCamOri() =
        glm::angleAxis(glm::radians(0.0), glm::dvec3(1, 0, 0));
    m_gameBase->m_graphicsEngine->getFieldOfView() = glm::radians(120.0);
    // Panini projection: 0 = standard rectilinear (off), 1 = max distortion.
    m_gameBase->m_graphicsEngine->getPaniniHorizontal() = 0.5;
    m_gameBase->m_graphicsEngine->getPaniniVertical() = 0.1;
    // Blue-noise dither defaults to one 8-bit quantization step (1/255);
    // override via getDitherStrength() (0 = off, more = film grain).

    // Enable mouse lock for camera control
    m_gameBase->m_graphicsEngine->getMouseHandler()->setMouseLock(true);

    setupWorld();

    // Print instructions
    std::cout << "3D Grid Block Demo" << std::endl;
    std::cout << "Controls:" << std::endl;
    std::cout << "  WASD: Move camera" << std::endl;
    std::cout << "  Mouse: Look around" << std::endl;
    std::cout << "  Space/Shift: Move up/down" << std::endl;
    std::cout << "  M: Toggle mouse lock" << std::endl;
    std::cout << "  F: Apply force to grid" << std::endl;
    std::cout << "  R: Configure block (select corners)" << std::endl;
    std::cout << "  Q: Remove block at (1,1,1)" << std::endl;
}

Game::~Game() {
    // The guard member clears the debug-renderer global; this one is ours.
    DebugGlobals::g_gameBase = nullptr;
}

void Game::setupWorld() {
    // Create a center grid that will be our player object
    auto initialGridWeak = m_gameBase->createGrid(glm::dvec3(0, 0, 0));
    auto initialGrid = initialGridWeak.lock();
    auto rigidBodyWeak = initialGrid->getRigidBody();
    auto bb = rigidBodyWeak.lock();
    if (bb) {
        bb->m_position = {0, 0, 0};
    }

    // Create a Digibot character at origin
    m_gameBase->createDigibot();

    // Ground.
    int size{10};
    for (int ii = -size; ii < size; ii++) {
        for (int jj = -size; jj < size; jj++) {
            for (int kk = -3; kk < -2; kk++) {
                initialGrid->addCell(glm::ivec3(ii, jj, kk));
            }
        }
    }
}

void Game::onFrame() {
    // Poll the network first, apply its state at tick boundaries, send after:
    // the frame is a receive -> simulate -> send pipeline.
    m_gameNetwork->framePoll();

    // Drive the world's resumable frame, injecting control at each yield.
    // Remote state lands before local control so control acts on fresh state.
    GameBase::FrameStatus status{};
    do {
        status = m_gameBase->advanceFrame();
        if (status == GameBase::FrameStatus::AwaitingFrameControl) {
            m_mode->frameProcessInputs();
        } else if (status == GameBase::FrameStatus::AwaitingStepControl) {
            m_gameNetwork->stepApply();
            m_mode->stepControl();
        }
    } while (status != GameBase::FrameStatus::FrameDone);

    // Dev tooling, not world simulation: shader reload on the N key. Input was
    // polled inside the frame, so justPressed still reads this frame's state.
    if (m_gameBase->m_graphicsEngine->getKeyboardHandler()->m_n.justPressed()) {
        auto [success, message] = m_gameBase->reloadShaders();
        std::cout << "Shader Reload " << (success ? "SUCCESS" : "FAILED") << ": "
                  << message << std::endl;
    }

    // Sends a snapshot once per completed physics tick, not per frame.
    m_gameNetwork->frameSend();
}

void Game::run() {
    while (!m_gameBase->m_graphicsEngine->getGraphicsEngineBase()->shouldClose()) {
        onFrame();
    }
}

void Game::setupDebugVisualization() {
    m_debugViz = std::make_unique<DebugVisualization>(
        m_gameBase->m_graphicsEngine.get(),
        m_gameBase->m_graphicsEngine->m_ssboManager.get());
    m_gameBase->setDebugRenderer(m_debugViz.get());
}
