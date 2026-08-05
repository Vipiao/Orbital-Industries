#include "GameNetworkClient.h"

#include "StateSnapshot.h"

#include "../characters/digibot/DigibotController.h"
#include "../game_base/GameBase.h"
#include "../game_base/Grid.h"
#include "../game_base/GridSerializer.h"
#include "../game_base/GridSubsystem.h"
#include "../network/NetworkStartupConfig.h"
#include "../physics/PhysicsEngine.h"
#include "../physics/PhysicsUnits.h"
#include "../physics/RigidBody.h"
#include "../serialization/ByteStream.h"
#include "debug/DebugGlobals.h"
#include "debug/DebugRenderer.h"

#include <cassert>
#include <glm/glm.hpp>
#include <iostream>

namespace {

// World reconciliation cadence: how often this client reports what it holds. The
// window must comfortably exceed a round trip plus one manifest.
const double k_manifestIntervalTicks{PhysicsUnits::seconds(5.0)};

// A state advanced dtTicks along its own velocity and spin, for sub-tick time
// alignment. The body supplies the inverse inertia that turns angular momentum into
// spin, and the centre-of-mass offset the motion is built around: the velocity
// carries the centre of mass while the body turns about it, and the origin the
// state stores follows from the two. Mirrors RigidBody::getInterpolatedTransform.
RigidBodyState shiftedInTime(const RigidBodyState& state, double dtTicks,
                             const RigidBody& body) {
    RigidBodyState result{state};
    glm::dvec3 angularVelocityWorld{glm::mat3_cast(state.m_orientation) *
                                    (body.getInvInertiaTensor() * state.m_angularMomentumBody)};
    result.m_orientation = RigidBody::integrateOrientation(state.m_orientation,
                                                           angularVelocityWorld, dtTicks);

    const glm::dvec3& centerOfMassLocal{body.getCenterOfMassLocal()};
    glm::dvec3 centerOfMass{state.m_position +
                            glm::mat3_cast(state.m_orientation) * centerOfMassLocal};
    result.m_position = centerOfMass + state.m_velocity * dtTicks -
                        glm::mat3_cast(result.m_orientation) * centerOfMassLocal;
    return result;
}

}  // namespace

GameNetworkClient::GameNetworkClient(INetworkTransport* transport, GameBase* gameBase)
    : GameNetworkBase{transport, gameBase} {
    assert(!transport->isServer());
}

void GameNetworkClient::updateCharacterControl(Mode& mode) {
    ControlDesire desire{readControlDesire(mode)};
    if (desire.m_wantsControl && !desire.m_wasWanting && m_localPlayerId < 0) {
        if (!m_connections.empty()) {
            // Ask the server for this character; it answers with a ControlGrant.
            ByteWriter writer{};
            writer.write(static_cast<std::uint8_t>(MessageType::ControlRequest));
            writer.write(static_cast<std::int32_t>(desire.m_preferredCharacterId));
            broadcast(writer.take(), true);
        }
        // else: not connected yet; the request has nowhere to go and is dropped.
        // Wanting control while disconnected requires toggling it off and on
        // again once a connection exists.
    } else if (!desire.m_wantsControl && desire.m_wasWanting) {
        // Give back whatever might be held or asked for. Safe even if nothing
        // actually was — the receiving end no-ops on that.
        ByteWriter writer{};
        writer.write(static_cast<std::uint8_t>(MessageType::ControlRelease));
        broadcast(writer.take(), true);
        m_localPlayerId = -1;
    }
    bindModeToLocalPlayer(mode, desire.m_wantsControl);
}

void GameNetworkClient::setLocalControlsCharacter(bool controls) {
    // Dropping control abandons the prediction so a later takeover does not ease
    // in from a stale anchor.
    if (m_localControlsCharacter && !controls) {
        m_prediction.reset();
        m_hasPredictionAnchor = false;
    }
    m_localControlsCharacter = controls;
}

void GameNetworkClient::handleMessage(MessageType type, ByteReader& reader,
                                      INetworkTransport::Message& message) {
    switch (type) {
    case MessageType::StateSnapshot: {
        // Keep only the freshest snapshot per sender.
        std::uint64_t tick{0};
        if (!reader.read(tick)) {
            break;
        }
        auto lastTick{m_lastSnapshotTick.find(message.m_connection)};
        if (lastTick != m_lastSnapshotTick.end() && tick <= lastTick->second) {
            break;  // stale or reordered; a newer snapshot already arrived
        }
        m_lastSnapshotTick[message.m_connection] = tick;
        m_pendingSnapshots[message.m_connection] = std::move(message.m_data);
        break;
    }
    case MessageType::PlayerInput:
        break;  // only the server accepts and arbitrates player input
    case MessageType::Structural:
        receiveStructural(reader);
        break;
    case MessageType::WorldManifest:
        break;  // only the server reconciles manifests against the world
    case MessageType::GridData:
        // A full-grid correction. Queued with the structural stream so the
        // reliable channel's ordering carries into application.
        m_receivedReliable.push_back(std::move(message.m_data));
        break;
    case MessageType::ControlRequest:
        break;  // only the server arbitrates control requests
    case MessageType::ControlGrant: {
        // The arbiter's answer to this peer's request.
        std::int32_t granted{-1};
        if (!reader.read(granted)) {
            break;
        }
        if (!m_wasWantingCharacterControl) {
            // The ask this answers has since been abandoned; don't adopt it, hand
            // back anything actually granted so it isn't reserved forever.
            if (granted >= 0) {
                ByteWriter writer{};
                writer.write(static_cast<std::uint8_t>(MessageType::ControlRelease));
                broadcast(writer.take(), true);
            }
            break;
        }
        if (granted >= 0) {
            m_localPlayerId = granted;
            std::cout << "[net] granted character " << granted << std::endl;
        } else {
            std::cout << "[net] control denied: all characters taken" << std::endl;
        }
        break;
    }
    case MessageType::ControlRelease:
        break;  // only the server holds ownership to release
    }
}

void GameNetworkClient::onDisconnected(INetworkTransport::ConnectionId /*connection*/) {
    // The server is gone, and with it the session: the grant, prediction and the
    // timeline baseline are all scoped to the dead connection. A future
    // connection starts clean and, if control is still desired, re-asks once
    // reconnected.
    m_localPlayerId = -1;
    m_prediction.reset();
    m_hasPredictionAnchor = false;
    m_timelineFilter = TickTimelineFilter{};
    m_pendingSnapshots.clear();
    m_lastSnapshotTick.clear();
}

void GameNetworkClient::stepApplyRole() {
    // Reconcile the just-integrated prediction against the anchor. The anchor first
    // takes the same one-tick step the world's bodies just took, so the pull never
    // targets a stale moment (error ~ speed * staleness). The weak continuous pull
    // pauses only while the character stands on a surface with no movement input.
    if (m_localControlsCharacter && m_hasPredictionAnchor) {
        std::weak_ptr<RigidBody> bodyWeak{findCharacterBody(m_localPlayerId)};
        if (std::shared_ptr<RigidBody> body{bodyWeak.lock()}) {
            m_predictionAnchor = shiftedInTime(m_predictionAnchor, 1.0, *body);
            DigibotController* controller{findDigibotController(m_localPlayerId)};
            bool standingStill{controller && !controller->isJetpackEnabled() &&
                               controller->hasGroundContact() &&
                               controller->getMovementDirection() == glm::ivec3{0}};
            bool active{controller && !standingStill};
            m_prediction.correct(bodyWeak, m_predictionAnchor, active,
                                 *m_gameBase->m_physicsEngine);
        }
    }

    // Apply authoritative reliable traffic in arrival order (structural edits
    // interleaved with full-grid corrections), then send own edits as requests.
    for (const auto& received : m_receivedReliable) {
        if (std::holds_alternative<StructuralCommand>(received)) {
            m_gameBase->applyStructural(std::get<StructuralCommand>(received));
        } else {
            applyGridData(std::get<std::vector<std::byte>>(received));
        }
    }
    m_receivedReliable.clear();
    broadcast(buildStructuralMessage(m_gameBase->drainStructuralEdits()), true);

    // Apply the freshest snapshot; the predicted character only refreshes its anchor.
    for (const auto& [connection, snapshot] : m_pendingSnapshots) {
        applyStateSnapshot(snapshot);
    }
    m_pendingSnapshots.clear();

    // Debug: the sphere carries the anchor's velocity so the renderer extrapolates
    // it per frame. Drawn after the snapshot apply, so it rides the same timeline
    // shift as the grids around it.
    if (NetworkStartupConfig::s_showPredictionAnchor && m_localControlsCharacter &&
        m_hasPredictionAnchor && DebugGlobals::getDebugRenderer()) {
        DebugGlobals::getDebugRenderer()->createSphere(
            "predict_anchor", m_predictionAnchor.m_position, 0.3);
        DebugGlobals::getDebugRenderer()->setVelocity(
            "predict_anchor", m_predictionAnchor.m_velocity, m_gameBase->getPhysicsTick());
    }
}

void GameNetworkClient::frameSendRole(std::uint64_t tick) {
    if (m_localControlsCharacter && m_localPlayerId >= 0) {
        broadcast(buildPlayerInput(tick), false);
    }

    // World reconciliation heartbeat: report what this client holds; the first
    // manifest doubles as the late-join request for the whole world.
    if (!m_manifestSent || tick - m_lastManifestTick >= k_manifestIntervalTicks) {
        m_manifestSent = true;
        m_lastManifestTick = tick;
        broadcast(buildWorldManifest(), true);
    }
}

void GameNetworkClient::applyGridData(const std::vector<std::byte>& data) {
    // Peek the id so an existing copy is replaced rather than refused.
    ByteReader peek{data};
    std::uint8_t rawType{0};
    std::uint64_t capturedTick{0};
    std::uint64_t id{0};
    if (!peek.read(rawType) || !peek.read(capturedTick) || !peek.read(id)) {
        return;
    }
    if (m_gameBase->getGridSubsystem()->getGridById(id).lock()) {
        m_gameBase->getGridSubsystem()->despawnGrid(id);
    }

    ByteReader reader{data};
    reader.read(rawType);
    reader.read(capturedTick);
    std::shared_ptr<Grid> grid{
        GridSerializer::deserialize(reader, *m_gameBase->getGridSubsystem()).lock()};
    if (!grid) {
        std::cout << "[net] dropped malformed grid data" << std::endl;
        return;
    }

    // The payload was captured a round trip ago, while the rest of this client's
    // world stands at the last applied snapshot. Carry the rebuilt body forward to
    // that same moment, so it does not join the world a round trip behind it.
    std::weak_ptr<RigidBody> bodyWeak{grid->getRigidBody()};
    std::shared_ptr<RigidBody> body{bodyWeak.lock()};
    if (body && m_lastAppliedSnapshotTick > capturedTick) {
        double staleness{static_cast<double>(m_lastAppliedSnapshotTick - capturedTick)};
        shiftedInTime(RigidBodyState::capture(*body), staleness, *body)
            .apply(bodyWeak, *m_gameBase->m_physicsEngine);
    }
}

std::vector<std::byte> GameNetworkClient::buildWorldManifest() const {
    ByteWriter writer{};
    writer.write(static_cast<std::uint8_t>(MessageType::WorldManifest));
    const std::vector<std::shared_ptr<Grid>>& grids{
        m_gameBase->getGridSubsystem()->getGrids()};
    writer.write(static_cast<std::uint32_t>(grids.size()));
    for (const std::shared_ptr<Grid>& grid : grids) {
        writer.write(grid->uniqueId);
        writer.write(static_cast<std::uint64_t>(GridSerializer::structureHash(*grid)));
    }
    return writer.take();
}

std::vector<std::byte> GameNetworkClient::buildPlayerInput(std::uint64_t tick) const {
    assert(m_localPlayerId >= 0);
    ByteWriter writer{};
    writer.write(static_cast<std::uint8_t>(MessageType::PlayerInput));
    writer.write(tick);
    writer.write(static_cast<std::int32_t>(m_localPlayerId));
    captureResolvedInput(m_localPlayerId).serialize(writer);
    captureDockingStatus(m_localPlayerId).serialize(writer);
    return writer.take();
}

void GameNetworkClient::applyStateSnapshot(const std::vector<std::byte>& data) {
    ByteReader reader{data};
    std::uint8_t rawType{0};
    StateSnapshot snapshot{};
    if (!reader.read(rawType) || !snapshot.deserialize(reader)) {
        // Parse before apply: a malformed snapshot changes nothing, not even the
        // timeline filter.
        std::cout << "[net] dropped malformed snapshot" << std::endl;
        return;
    }

    // temp: a jump in the server tick between applied snapshots means the ones in
    // between were dropped on the wire.
    if (m_lastAppliedSnapshotTick != 0 && snapshot.m_tick > m_lastAppliedSnapshotTick + 1) {
        std::cout << "[net] snapshot gap: " << (snapshot.m_tick - m_lastAppliedSnapshotTick - 1)
                  << " dropped before tick " << snapshot.m_tick << " (grids="
                  << snapshot.m_grids.size() << ")" << std::endl;
    }
    m_lastAppliedSnapshotTick = snapshot.m_tick;

    // The drift plays two roles. It nudges the next physics step, bleeding the
    // offset off gradually (the clock controller). It also time-aligns this
    // snapshot — one stale server tick behind the client's current tick — to now,
    // as a single common shift for every object, so the fast per-snapshot arrival
    // jitter the clock cannot track does not smear positions. No EMA: the shift is
    // the raw drift, and the clock keeps that drift small over time.
    TickTimelineFilter::Alignment alignment{
        m_timelineFilter.update(snapshot.m_tick, m_gameBase->getPhysicsTick())};
    double drift{alignment.m_driftTicks};
    bool resync{alignment.m_resync};
    if (resync) {
        std::cout << "[net] tick timeline resync (drift " << drift << " ticks)" << std::endl;
    } else {
        m_gameBase->nudgePhysicsSchedule(alignment.m_scheduleNudgeTicks);
    }

    // temp: watch the clock controller drive tick drift toward zero
    static std::uint64_t s_driftLogCounter{0};
    if (++s_driftLogCounter % 64 == 0) {
        std::cout << "[net] tick drift=" << drift
                  << " nudge=" << alignment.m_scheduleNudgeTicks << std::endl;
    }

    // Grids are not predicted yet, so always applied directly on the client.
    for (const StateSnapshot::GridEntry& entry : snapshot.m_grids) {
        if (!entry.m_state.isFinite()) {
            continue;
        }
        std::weak_ptr<RigidBody> bodyWeak{findGridBody(entry.m_id)};
        if (std::shared_ptr<RigidBody> body{bodyWeak.lock()}) {
            shiftedInTime(entry.m_state, drift, *body)
                .apply(bodyWeak, *m_gameBase->m_physicsEngine);
        }
    }

    for (const StateSnapshot::CharacterEntry& entry : snapshot.m_characters) {
        std::int32_t id{entry.m_id};
        const RigidBodyState& state{entry.m_state};
        if (!state.isFinite() || !entry.m_input.isValid() || !entry.m_docking.isValid()) {
            continue;
        }
        std::weak_ptr<RigidBody> bodyWeak{findCharacterBody(id)};
        std::shared_ptr<RigidBody> body{bodyWeak.lock()};
        if (!body) {
            continue;
        }
        if (id == m_localPlayerId && m_localControlsCharacter) {
            // This client predicts the character and is its input source: feed the
            // authority to reconciliation as an anchor, time-aligned to the current
            // tick, and ignore the relayed input and docking (its own echo).
            RigidBodyState anchor{shiftedInTime(state, drift, *body)};
            if (resync) {
                // Prediction restarts wholesale from the anchor.
                m_prediction.reset();
                anchor.apply(bodyWeak, *m_gameBase->m_physicsEngine);
            }
            m_predictionAnchor = anchor;
            m_hasPredictionAnchor = true;
        } else {
            // A character controlled elsewhere: the pose is authoritative, while
            // the relayed input and docking status drive this peer's copy so its
            // mode, animation and cockpit engagement follow.
            shiftedInTime(state, drift, *body)
                .apply(bodyWeak, *m_gameBase->m_physicsEngine);
            applyResolvedInput(id, entry.m_input);
            forceDockingStatus(id, entry.m_docking);
        }
    }
}
