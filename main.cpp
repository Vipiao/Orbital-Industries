// main.cpp
#include "src/game/Game.h"
#include "src/game_base/GameBase.h"
#include "src/game_base/Grid.h"
#include "src/characters/digibot/Digibot.h"
#include "src/characters/digibot/DigibotController.h"
#include "src/network/DelayedTransport.h"
#include "src/network/GnsTransport.h"
#include "src/network/INetworkTransport.h"
#include "src/network/ReplayTransport.h"
#include "src/network/StartupPrompt.h"
#include "src/physics/PhysicsUnits.h"
#include "src/physics/RigidBody.h"
#include "utils/TimeHandler.h"
#include "debug/DebugRenderer.h"
#include "debug/DebugGlobals.h"
#include "graphics/GraphicsEngine.h"
#include <glm/glm.hpp>
#include <filesystem>
#include <iostream>
#include <memory>

int debug1 = 0;
int debug2 = 0;

// Define the global debug renderer (must be in exactly one .cpp file)
DebugRenderer* DebugGlobals::g_debugRenderer = nullptr;
IHashable* DebugGlobals::g_gameBase = nullptr;

// Test fixture: the hardcoded demo world. This is content, not game machinery, so
// it lives with the entry point. Server and client build the identical world in
// the same order, so every object id matches deterministically and the network
// layer can sync purely by id.
static void buildTestWorld(GameBase* gameBase) {
    // The platform and both characters share a fast downward velocity, so
    // prediction and anchoring are exercised at speed from the first tick.
    glm::dvec3 worldVelocity{0.0, 0.0, -PhysicsUnits::metersPerSecond(2000.0)};

    std::shared_ptr<Grid> ground{gameBase->createGrid(glm::dvec3{0.0, 0.0, 0.0}).lock()};
    if (ground) {
        int size{10};
        for (int ii = -size; ii < size; ii++) {
            for (int jj = -size; jj < size; jj++) {
                ground->addCell(glm::ivec3{ii, jj, -3});
            }
        }
        if (std::shared_ptr<RigidBody> body{ground->getRigidBody().lock()}) {
            body->m_velocity = worldVelocity;
        }
    }

    // Two controllable characters, side by side above the platform. Peers acquire
    // one through the control-arbitration flow when the player takes control.
    for (int ii = 0; ii < 2; ii++) {
        std::shared_ptr<Digibot> digibot{gameBase->createDigibot().lock()};
        if (digibot) {
            std::shared_ptr<RigidBody> body{digibot->getRigidBody().lock()};
            if (body) {
                // Close above the platform surface, so walking mode engages
                // right away instead of a long flying approach. Spaced wide
                // enough that neither stands on the other's sensor collider.
                body->m_position = glm::dvec3{ii * 5.0, 0.0, -1.0};
                body->m_velocity = worldVelocity;
            }
            // Start in walking mode (jetpack off); both peers build the same
            // state, and the input sync carries the flag from there.
            digibot->getController()->setJetpackEnabled(false);
        }
    }
}

// One switch drives record/playback for the whole session. The time, control and
// network handlers each take their own Mode enum; each is selected by name below.
// They must move together: a faithful replay needs all three fed from the same
// run's recordings.
enum class SessionMode { NONE, RECORD, PLAY };
static constexpr SessionMode s_sessionMode{SessionMode::RECORD};

int main() {
    try {
        // Choose network role before any engine/window init.
        NetworkStartupConfig netConfig{startupPrompt::prompt()};
        bool isServerRole{netConfig.m_role == NetworkStartupConfig::Role::Server};

        // Each role journals its own file, so one multiplayer session yields two
        // recordings, each replayable on its own.
        ReplayTransport::Mode replayMode{
            s_sessionMode == SessionMode::RECORD ? ReplayTransport::Mode::RECORD
            : s_sessionMode == SessionMode::PLAY ? ReplayTransport::Mode::PLAY
                                                 : ReplayTransport::Mode::NONE};
        std::filesystem::path replayFile{isServerRole ? "recording_network/server.bin"
                                                      : "recording_network/client.bin"};

        std::unique_ptr<INetworkTransport> transport{};
        if (replayMode == ReplayTransport::Mode::PLAY) {
            // The journal is the peer: no backend, no delay simulation.
            transport = std::make_unique<ReplayTransport>(nullptr, replayMode, replayFile);
        } else {
            transport = std::make_unique<GnsTransport>();
            // Optional latency simulation for testing (set in NetworkStartupConfig):
            // wraps the transport so inbound messages are held back before delivery.
            if (netConfig.m_simulatedDelayMs > 0.0) {
                std::cout << "[net] simulating " << netConfig.m_simulatedDelayMs
                          << " ms inbound delay" << std::endl;
                transport = std::make_unique<DelayedTransport>(
                    std::move(transport), netConfig.m_simulatedDelayMs,
                    netConfig.m_simulatedJitterMs);
            }
            // The recorder wraps the whole stack: it journals what the game
            // consumes, delay simulation included.
            if (replayMode == ReplayTransport::Mode::RECORD) {
                transport = std::make_unique<ReplayTransport>(std::move(transport),
                                                              replayMode, replayFile);
            }
        }
        bool netOk{isServerRole ? transport->startServer(netConfig.m_port)
                                : transport->connect(netConfig.m_ip, netConfig.m_port)};
        if (!netOk) {
            std::cerr << "Network startup failed" << std::endl;
            return 1;
        }

        // Declared before Game so it outlives Game's teardown (GameBase and
        // the engine keep non-owning pointers to it).
        TimeHandler::Mode timeMode{
            s_sessionMode == SessionMode::RECORD ? TimeHandler::Mode::RECORD
            : s_sessionMode == SessionMode::PLAY ? TimeHandler::Mode::PLAY
                                                 : TimeHandler::Mode::NONE};
        std::unique_ptr<TimeHandler> timeHandler{std::make_unique<TimeHandler>(timeMode)};

        GraphicsEngineBase::Mode controlMode{
            s_sessionMode == SessionMode::RECORD ? GraphicsEngineBase::Mode::RECORD
            : s_sessionMode == SessionMode::PLAY ? GraphicsEngineBase::Mode::PLAY
                                                 : GraphicsEngineBase::Mode::NONE};

        Game game(timeHandler.get(), controlMode, std::move(transport));
        buildTestWorld(game.getGameBase());

        // Each role starts nearest a different character, so the two peers'
        // nearest-camera control requests resolve to different digibots.
        if (netConfig.m_role == NetworkStartupConfig::Role::Client) {
            game.getGameBase()->m_graphicsEngine->getCamPos() = glm::dvec3{5.0, 0.0, 0.0};
        }

        // Testing convenience: park the window at a fixed spot (no-op on Wayland).
        game.getGameBase()->m_graphicsEngine->setWindowPos(1500, 700);

        game.run();
    } catch (const std::bad_alloc& e) {
        std::cerr << "Out of memory: " << e.what() << std::endl;
        return 1;
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return 1;
    }

    return 0;
}
