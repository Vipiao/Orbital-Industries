// main.cpp
#include "src/game/Game.h"
#include "src/game_base/GameBase.h"
#include "src/game_base/Grid.h"
#include "src/game_base/Planet.h"
#include "src/game_base/PlanetTypeConfig.h"
#include "src/characters/digibot/Digibot.h"
#include "src/characters/digibot/DigibotController.h"
#include "src/network/DelayedTransport.h"
#include "src/network/GnsTransport.h"
#include "src/network/INetworkTransport.h"
#include "src/network/ReplayTransport.h"
#include "src/network/StartupPrompt.h"
#include "src/physics/RigidBody.h"
#include "utils/TimeHandler.h"
#include "src/world/PlanetSurface.h"
#include "debug/DebugRenderer.h"
#include "debug/DebugGlobals.h"
#include "graphics/GraphicsEngine.h"
#include <glm/glm.hpp>
#include <cassert>
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
    std::shared_ptr<Grid> ground{gameBase->createGrid(glm::dvec3{0.0, 0.0, 0.0}).lock()};
    if (ground) {
        int size{10};
        for (int ii = -size; ii < size; ii++) {
            for (int jj = -size; jj < size; jj++) {
                ground->addCell(glm::ivec3{ii, jj, -3});
            }
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
                body->setPosition(glm::dvec3{ii * 5.0, 0.0, -1.0});
            }
            // Start in walking mode (jetpack off); both peers build the same
            // state, and the input sync carries the flag from there.
            digibot->getController()->setJetpackEnabled(false);
        }
    }

    // A planet parked off to the side, with no collider yet, given a slow spin so
    // the vertex stage's frame interpolation is visible on it. Sized as a planet,
    // which is what the surface and the LOD tree have to survive.
    GraphicsEngine* graphicsEngine{gameBase->m_graphicsEngine.get()};
    // SSAO works from the depth buffer, which holds the coarse displaced
    // triangles, rather than the per-pixel normals the surface is shaded with.
    // Where the two disagree it darkens facets.
    graphicsEngine->setSsaoEnabled(true);

    // Triangles per selected patch, shared by every CDLOD body.
    graphicsEngine->setCdlodPatchQuads(24);

    // The body's shape, written once: the planet is what the bounds measure and
    // what the physics will eventually collide against, and the shader that draws
    // it is handed these same figures. The renderer knows none of them, and
    // nothing below reads them again -- the planet is asked instead.
    //
    // The tile goes in as the two whole numbers rather than the 1274.2 they come
    // to, which is not exact in either width; both sides then take them apart the
    // same way and read one map at one scale.
    PlanetTypeConfig planetTypeConfig{};
    planetTypeConfig.m_radiusMetres = 6371000.0;
    planetTypeConfig.m_tileSpanMetres = 12742000.0;
    planetTypeConfig.m_tilesPerSpan = 10000.0;
    planetTypeConfig.m_reliefMetres = 2632.0;
    // The layer beneath the octaves. Nothing like their rise over run, and
    // deliberately: it is hundreds of times wider than octave zero and only a
    // few times taller, so it is far the gentlest slope on the body and the
    // quadtree's ranges, which answer for the steepest, never see it.
    //
    // At zero the body carries none of it and the lattice layers stand on a
    // plain sphere, which is the only way to see what one of them does on its
    // own: at this height the layer is the shape of the body and the octaves
    // read as texture on it.
    planetTypeConfig.m_baseReliefMetres = 10000.0;

    // The noise both sides read, generated once. The map is dimensionless -- the
    // field spans exactly [0, 1] and its gradient is per unit of tile -- so it
    // needs nothing alongside it to be read back, and the three figures above
    // are what give it a size on either side.
    TileableNoiseMapConfig& terrainConfig{planetTypeConfig.m_terrain};
    terrainConfig.m_resolution = 2048;
    terrainConfig.m_octaveCount = 8;
    terrainConfig.m_baseFrequency = 2;
    terrainConfig.m_gain = 0.45;
    // Creased per octave rather than per map, which is where the ridges come
    // from: the terrain reads this the other way up, so every octave's crease is
    // a summit at its own scale, and the count above is how many scales of ridge
    // the map carries. The base layer's field is left unfolded.
    terrainConfig.m_octaveShape = TileableNoiseMapConfig::OctaveShape::FOLDED;
    terrainConfig.m_seed = 20260811;

    // The layer under all of them, which the snippet reads by direction off a
    // cube map rather than through the lattice. Nothing here is written twice:
    // the snippet is handed the bake and the one constant that gives it metres.
    //
    // At an honest height this layer is a part in a thousand of the radius and a
    // third of a degree of slope, so it cannot be seen in the silhouette or the
    // shading at all. The snippet's colour bands are what it is read through.
    planetTypeConfig.m_base = PlanetBaseLayerConfig{};

    // Sea level halfway up the base layer's height. Light is down to about a third
    // after 50 metres of water.
    PlanetWaterConfig waterConfig{};
    waterConfig.m_seaLevelMetres = 5000.0;
    waterConfig.m_absorptionPerMetre = 0.02;
    waterConfig.m_color = glm::dvec3{0.02, 0.08, 0.15};
    planetTypeConfig.m_water = waterConfig;

    // Generates the maps, once for every planet of this shape
    const std::weak_ptr<const PlanetType> planetType{
        gameBase->createPlanetType(planetTypeConfig)};

    // Earth's
    const double planetMassKg{5.972e24};

    std::shared_ptr<Planet> planet{gameBase->createPlanet(planetType, planetMassKg).lock()};
    std::shared_ptr<RigidBody> planetBody{planet ? planet->getRigidBody().lock() : nullptr};
    if (planetBody) {
        // Gap between the platform and the highest the terrain can reach.
        const double platformClearanceMetres{200.0};

        // Centred so the terrain's ceiling sits just above the platform.
        planetBody->setPosition(
            glm::dvec3{0.0, planet->getSurface().maxRadius() + platformClearanceMetres, 0.0});

        // Radians per physics step
        planetBody->setAngularVelocityBody(
            glm::normalize(glm::dvec3{0.2, 1.0, 0.35}) * 0.00000001);
    }
}

// One switch drives record/playback for the whole session. The time, control and
// network handlers each take their own Mode enum; each is selected by name below.
// They must move together: a faithful replay needs all three fed from the same
// run's recordings.
//
// RECORD is the resting position, so a crash or a bug met while playing is still
// replayable afterwards: the capture is already on disk when it happens.
enum class SessionMode { NONE, RECORD, PLAY };
static constexpr SessionMode s_sessionMode{SessionMode::RECORD};

// The session folders, holding the three streams a replay needs side by side.
// Relative to the working directory the game runs from, as the media paths are.
//
// Read and written apart, because recording deletes what it finds: a capture
// lands in the scratch folder and is moved to a number of its own once it is
// worth keeping, so no edit of the switch above can overwrite a session that
// already is. Two roles recording at once want two scratch folders, every stream
// but the network journal being per role.
static const std::filesystem::path s_playbackDir{"../recordings/004_digibot_leg_offset"};
static const std::filesystem::path s_recordDir{"../recordings/999_scratch"};

int main() {
    try {
        // Choose network role before any engine/window init.
        NetworkStartupConfig netConfig{startupPrompt::prompt()};
        bool isServerRole{netConfig.m_role == NetworkStartupConfig::Role::Server};

        // Recording deletes what it finds, so the two must never be one folder:
        // a capture aimed at the session being replayed destroys it, and the
        // journals are large enough to live outside the repository.
        assert(s_playbackDir != s_recordDir &&
               "A capture written where playback reads would delete the session");

        const std::filesystem::path& sessionDir{
            s_sessionMode == SessionMode::RECORD ? s_recordDir : s_playbackDir};

        // Each role journals its own file, so one multiplayer session yields two
        // recordings, each replayable on its own.
        ReplayTransport::Mode replayMode{
            s_sessionMode == SessionMode::RECORD ? ReplayTransport::Mode::RECORD
            : s_sessionMode == SessionMode::PLAY ? ReplayTransport::Mode::PLAY
                                                 : ReplayTransport::Mode::NONE};
        std::filesystem::path replayFile{
            sessionDir / (isServerRole ? "recording_network/server.bin"
                                         : "recording_network/client.bin")};

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
        std::unique_ptr<TimeHandler> timeHandler{
            std::make_unique<TimeHandler>(timeMode, sessionDir / "recording_time/time_data.bin")};

        GraphicsEngineBase::Mode controlMode{
            s_sessionMode == SessionMode::RECORD ? GraphicsEngineBase::Mode::RECORD
            : s_sessionMode == SessionMode::PLAY ? GraphicsEngineBase::Mode::PLAY
                                                 : GraphicsEngineBase::Mode::NONE};

        Game game(timeHandler.get(), controlMode, std::move(transport),
                  sessionDir / "recording_mouse_keyboard");
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
