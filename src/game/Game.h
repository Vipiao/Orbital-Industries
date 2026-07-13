// Game.h — application layer above GameBase: owns the main loop, the network
// transport and sync, and (later) menus and level lifecycle. GameBase is a
// running world; Game decides that a world runs and what surrounds it.
#pragma once

#include "graphics/GraphicsEngineBase.h"
#include "debug/DebugGlobals.h"

#include <memory>

class GameBase;
class DebugVisualization;
class Mode;
class INetworkTransport;
class GameNetwork;
class TimeHandler;

class Game {
public:
    // The transport must already be listening (server) or connecting (client).
    Game(TimeHandler* timeHandler, GraphicsEngineBase::Mode controlMode,
         std::unique_ptr<INetworkTransport> transport);
    ~Game();

    GameBase* getGameBase() { return m_gameBase.get(); }

    void onFrame();
    void run();

private:
    // Server and client build the identical world so object ids line up;
    // network sync then matches state by id with no create/destroy replication.
    void setupWorld();
    void setupDebugVisualization();

    std::unique_ptr<GameBase> m_gameBase;
    std::unique_ptr<DebugVisualization> m_debugViz;
    std::unique_ptr<Mode> m_mode;
    std::unique_ptr<INetworkTransport> m_transport;
    std::unique_ptr<GameNetwork> m_gameNetwork;
    DebugRendererGuard m_debugGuard;
};
