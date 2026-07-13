// main.cpp
#include "src/game/Game.h"
#include "src/game_base/GameBase.h"
#include "src/network/GnsTransport.h"
#include "src/network/INetworkTransport.h"
#include "src/network/StartupPrompt.h"
#include "utils/TimeHandler.h"
#include "debug/DebugRenderer.h"
#include "debug/DebugGlobals.h"
#include "graphics/GraphicsEngine.h"
#include <iostream>
#include <memory>

int debug1 = 0;
int debug2 = 0;

// Define the global debug renderer (must be in exactly one .cpp file)
DebugRenderer* DebugGlobals::g_debugRenderer = nullptr;
IHashable* DebugGlobals::g_gameBase = nullptr;

int main() {
    try {
        // Choose network role before any engine/window init.
        NetworkStartupConfig netConfig{startupPrompt::prompt()};
        std::unique_ptr<INetworkTransport> transport{std::make_unique<GnsTransport>()};
        bool netOk{netConfig.m_role == NetworkStartupConfig::Role::Server
                       ? transport->startServer(netConfig.m_port)
                       : transport->connect(netConfig.m_ip, netConfig.m_port)};
        if (!netOk) {
            std::cerr << "Network startup failed" << std::endl;
            return 1;
        }

        // Declared before Game so it outlives Game's teardown (GameBase and
        // the engine keep non-owning pointers to it).
        std::unique_ptr<TimeHandler> timeHandler{
            std::make_unique<TimeHandler>(TimeHandler::Mode::RECORD)};

        // Use existing GraphicsEngineBase::Mode for controls
        GraphicsEngineBase::Mode controlMode = GraphicsEngineBase::Mode::RECORD;

        Game game(timeHandler.get(), controlMode, std::move(transport));

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
