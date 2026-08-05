#include "GameNetworkServer.h"

#include "StateSnapshot.h"

#include "../characters/Character.h"
#include "../characters/CharacterSubsystem.h"
#include "../characters/digibot/Digibot.h"
#include "../game_base/GameBase.h"
#include "../game_base/Grid.h"
#include "../game_base/GridSerializer.h"
#include "../game_base/GridSubsystem.h"
#include "../game_base/RigidBodyState.h"
#include "../physics/PhysicsUnits.h"
#include "../physics/RigidBody.h"
#include "../serialization/ByteStream.h"

#include <algorithm>
#include <cassert>
#include <cmath>
#include <iostream>

namespace {

// World reconciliation cadence: how long a grid's manifest mismatch is
// attributed to in-flight echoes rather than divergence. The window must
// comfortably exceed a round trip plus one manifest.
const double k_recentEditWindowTicks{PhysicsUnits::seconds(3.0)};

// Grid position-sync scheduler (per connection). A grid comes due when its stale
// copy on the client would have drifted k_syncAngularTolerance off where the viewer
// sees it, so the snapshot spends its byte budget where the error is visible.
const std::size_t k_gridSnapshotBudgetBytes{6000};
const double k_syncAngularTolerance{0.0087};  // rad, half a degree of apparent drift
// The keyframe interval: the deadline for a grid the client is coasting perfectly,
// which heals a lost update and bounds worst-case staleness. A snapshot is built
// once per tick, so one tick is the shortest deadline that means anything.
const double k_keyframeIntervalTicks{PhysicsUnits::seconds(2.0)};
const double k_minIntervalTicks{1.0};
// Peak-held envelope decay per tick, derived from a half-life in seconds (several
// round trips) so a grid that just slowed stays urgent long enough that a lost
// "slowed down" update is resent before the client's stale high velocity carries
// it far — and so the real-time half-life stays fixed if the tick rate changes.
const double k_syncEnvelopeHalfLifeTicks{PhysicsUnits::seconds(0.5)};
const double k_syncEnvelopeDecay{std::pow(0.5, 1.0 / k_syncEnvelopeHalfLifeTicks)};
// A grid entry is id + RigidBodyState; used to stop filling before the byte budget.
const std::size_t k_gridEntryBytes{sizeof(std::uint64_t) + 13 * sizeof(double)};

// Ticks until the viewer would see the client's coasting copy of the grid drift
// k_syncAngularTolerance off the truth. envelope is the coasting-error rate in metres
// per tick, so envelope·T is the drift by the deadline and tolerance·range is what the
// range absorbs; solving those equal at the range the grid will have *then* prices
// approach exactly rather than by a tuned weight, and for straight line relative
// motion closes to  T = range / (closing + sqrt(referenceSpeed² − transverse²)).
// A rate at or below zero means the grid recedes faster than the closing range
// tightens the budget, so it never comes due on drift alone and waits for its keyframe.
// Flooring the root does the same for a transverse sweep quicker than referenceSpeed,
// settling for the arrival time — early rather than late, and rare at that speed.
double resendDeadline(const Grid& grid, const RigidBody& body, const glm::dvec3& viewerPos,
                      const glm::dvec3& viewerVel, double envelope) {
    // Measured to the centre of mass, not the body origin: the origin is the lattice
    // anchor and can sit well outside a grid that has been built out or cut down. The
    // near surface is what betrays a position error, so the radius comes back off.
    glm::dvec3 toViewer{viewerPos - body.getWorldCenterOfMass()};
    double centreRange{glm::length(toViewer)};
    double range{centreRange - grid.getApproximateRadius()};
    if (range <= 0.0) {
        return 0.0;  // the viewer is within the grid, where any drift is its widest
    }
    glm::dvec3 lineOfSight{toViewer / centreRange};
    glm::dvec3 relative{body.m_velocity - viewerVel};
    double closing{glm::dot(relative, lineOfSight)};
    double transverse{glm::length(relative - closing * lineOfSight)};
    double referenceSpeed{envelope / k_syncAngularTolerance};
    double radialSquared{referenceSpeed * referenceSpeed - transverse * transverse};
    double rate{closing + std::sqrt(std::max(radialSquared, 0.0))};
    return rate > 0.0 ? range / rate : k_keyframeIntervalTicks;
}

}  // namespace

GameNetworkServer::GameNetworkServer(INetworkTransport* transport, GameBase* gameBase)
    : GameNetworkBase{transport, gameBase} {
    assert(transport->isServer());
}

void GameNetworkServer::updateCharacterControl(Mode& mode) {
    ControlDesire desire{readControlDesire(mode)};
    if (desire.m_wantsControl && !desire.m_wasWanting && m_localPlayerId < 0) {
        // The arbiter is local: grant immediately, no round trip. A "-1 = any"
        // request resolves to a specific id first; acquireCharacter only ever
        // handles an already-resolved one.
        int id{desire.m_preferredCharacterId >= 0 ? desire.m_preferredCharacterId
                                                  : findAnyFreeCharacter()};
        int granted{id >= 0 ? acquireCharacter(std::nullopt, id) : -1};
        if (granted >= 0) {
            m_localPlayerId = granted;
            std::cout << "[net] granted character " << granted << std::endl;
        } else {
            std::cout << "[net] control denied: all characters taken" << std::endl;
        }
    } else if (!desire.m_wantsControl && desire.m_wasWanting) {
        // Give back whatever might be held. Safe even if nothing actually was.
        endControlOf(std::nullopt);
        m_localPlayerId = -1;
    }
    bindModeToLocalPlayer(mode, desire.m_wantsControl);
}

void GameNetworkServer::setLocalControlsCharacter(bool controls) {
    m_localControlsCharacter = controls;
}

void GameNetworkServer::handleMessage(MessageType type, ByteReader& reader,
                                      INetworkTransport::Message& message) {
    switch (type) {
    case MessageType::StateSnapshot:
        break;  // server-authoritative; a server never receives its own broadcast
    case MessageType::PlayerInput: {
        // Newest input per character wins; held until superseded.
        std::uint64_t tick{0};
        std::int32_t characterId{0};
        PendingInput pending{};
        if (!reader.read(tick) || !reader.read(characterId) ||
            !pending.m_input.deserialize(reader) ||
            !pending.m_docking.deserialize(reader) ||
            !pending.m_input.isValid() || !pending.m_docking.isValid()) {
            break;
        }
        // Only the character's owner may drive it; anything else is a stale
        // packet from a previous owner (or a spoof) and is dropped.
        auto owner{m_characterOwners.find(characterId)};
        if (owner == m_characterOwners.end() || !owner->second ||
            *owner->second != message.m_connection) {
            break;
        }
        auto lastTick{m_lastInputTick.find(characterId)};
        if (lastTick != m_lastInputTick.end() && tick <= lastTick->second) {
            break;
        }
        m_lastInputTick[characterId] = tick;
        pending.m_source = message.m_connection;
        pending.m_controlling = true;
        m_pendingInputs[characterId] = pending;
        break;
    }
    case MessageType::Structural:
        receiveStructural(reader);
        break;
    case MessageType::WorldManifest: {
        // Latest manifest per client wins; diffed at the next step.
        std::uint32_t count{0};
        if (!reader.read(count)) {
            break;
        }
        std::vector<std::pair<std::uint64_t, std::uint64_t>> manifest{};
        bool ok{true};
        for (std::uint32_t ii = 0; ii < count; ii++) {
            std::uint64_t id{0};
            std::uint64_t hash{0};
            if (!reader.read(id) || !reader.read(hash)) {
                ok = false;
                break;
            }
            manifest.push_back({id, hash});
        }
        if (ok) {
            m_pendingManifests[message.m_connection] = std::move(manifest);
        }
        break;
    }
    case MessageType::GridData:
        break;  // the server is the source of truth; it never receives corrections
    case MessageType::ControlRequest: {
        // Arbitrate immediately. Ownership is recorded before the grant is sent,
        // so the client's first input finds it in place.
        std::int32_t preferred{-1};
        if (reader.read(preferred)) {
            int id{preferred >= 0 ? preferred : findAnyFreeCharacter()};
            std::int32_t granted{id >= 0 ? acquireCharacter(message.m_connection, id) : -1};
            ByteWriter writer{};
            writer.write(static_cast<std::uint8_t>(MessageType::ControlGrant));
            writer.write(granted);
            m_transport->send(message.m_connection, writer.take(), true);
        }
        break;
    }
    case MessageType::ControlGrant:
        break;  // a server never requests control, so it never awaits a grant
    case MessageType::ControlRelease:
        endControlOf(message.m_connection);
        break;
    }
}

void GameNetworkServer::onDisconnected(INetworkTransport::ConnectionId connection) {
    // A vanished client implicitly releases everything it holds.
    endControlOf(connection);
    m_pendingManifests.erase(connection);
    m_gridSync.erase(connection);
}

int GameNetworkServer::acquireCharacter(
    const std::optional<INetworkTransport::ConnectionId>& owner, int characterId) {
    assert(characterId >= 0);
    if (!isCharacterAvailable(characterId)) {
        return -1;
    }
    // A fresh grant starts a fresh input-tick epoch: the new owner's tick clock is
    // unrelated to the previous owner's, so its counter must not gate them.
    m_characterOwners[characterId] = owner;
    m_lastInputTick.erase(characterId);
    return characterId;
}

bool GameNetworkServer::isCharacterAvailable(int characterId) const {
    return !m_characterOwners.count(characterId) && findDigibot(characterId);
}

int GameNetworkServer::findAnyFreeCharacter() const {
    for (const std::shared_ptr<Character>& character :
         m_gameBase->m_characterSubsystem->getCharacters()) {
        std::shared_ptr<Digibot> digibot{std::dynamic_pointer_cast<Digibot>(character)};
        if (digibot && !m_characterOwners.count(digibot->getUniqueId())) {
            return digibot->getUniqueId();
        }
    }
    return -1;
}

void GameNetworkServer::endControlOf(
    const std::optional<INetworkTransport::ConnectionId>& owner) {
    for (auto it{m_characterOwners.begin()}; it != m_characterOwners.end();) {
        it = it->second == owner ? m_characterOwners.erase(it) : std::next(it);
    }
    if (owner) {
        // End the connection's live input claims; each is neutralized exactly
        // once at the next step boundary. The server's own player has no claim
        // entries — its input is applied directly, not through the buffer.
        for (auto& [characterId, pending] : m_pendingInputs) {
            if (pending.m_source == *owner) {
                // Not a receive-side guard (the erased ownership above already
                // blocks new packets); tells applyPendingInputs this held entry
                // is stale so it stops reapplying it.
                pending.m_controlling = false;
            }
        }
    }
}

void GameNetworkServer::stepApplyRole() {
    // Apply structural edits (clients' requests and this peer's own tool edits)
    // and broadcast the authoritative result. Only commands are valid from
    // clients; a GridData payload would be a client claiming world authority.
    std::vector<StructuralCommand> applied{};
    for (const auto& received : m_receivedReliable) {
        if (std::holds_alternative<StructuralCommand>(received)) {
            applied.push_back(std::get<StructuralCommand>(received));
        }
    }
    m_receivedReliable.clear();
    for (StructuralCommand& command : m_gameBase->drainStructuralEdits()) {
        applied.push_back(command);
    }
    // Resolve each edit against the world — the game layer decides the
    // consequences (id allocation, split seeding, despawn-on-empty) and hands
    // back follow-up commands, broadcast after the edits that caused them.
    // Clients only ever replay this decided stream.
    std::vector<StructuralCommand> followUps{};
    for (StructuralCommand& command : applied) {
        for (StructuralCommand& followUp : m_gameBase->resolveStructuralEdit(command)) {
            followUps.push_back(std::move(followUp));
        }
    }
    broadcast(buildStructuralMessage(applied), true);
    broadcast(buildStructuralMessage(followUps), true);

    // Splits realised since the last step were already applied locally by the
    // split system; broadcast them so clients reproduce the same pieces and ids.
    std::vector<StructuralCommand> splitCommands{};
    for (GridSplitResult& result : m_gameBase->drainSplitResults()) {
        splitCommands.push_back(
            StructuralCommand::splitGrid(result.m_sourceGridId, std::move(result.m_pieces)));
    }
    broadcast(buildStructuralMessage(splitCommands), true);

    // Stamp every grid whose structure changed this step, so manifest diffs can
    // tell "diverged" from "echo still in flight".
    std::uint64_t tick{m_gameBase->getPhysicsTick()};
    auto touch = [&](std::uint64_t gridId) { m_lastStructureEditTick[gridId] = tick; };
    for (const StructuralCommand& command : applied) {
        touch(command.m_gridId);
    }
    for (const StructuralCommand& command : followUps) {
        touch(command.m_gridId);
    }
    for (const StructuralCommand& command : splitCommands) {
        touch(command.m_gridId);
        for (const GridSplitPiece& piece : command.m_pieces) {
            touch(piece.m_newGridId);
        }
    }

    // Diff client manifests against the world as it stands after this step's
    // edits; corrections ride the same reliable channel as the edits.
    processManifests();

    // Drive remotely-owned characters from their owners' inputs. The server's
    // own character never has a claim here (receive rejects claims from
    // non-owners), so this cannot fight the server player's local control.
    applyPendingInputs();
}

void GameNetworkServer::frameSendRole(std::uint64_t tick) {
    // Each connection gets its own snapshot: the same characters, but grids
    // paced to that client's view. So the message stays small however many
    // grids exist.
    std::size_t sampleSize{0};
    for (INetworkTransport::ConnectionId connection : m_connections) {
        std::vector<std::byte> snapshot{buildConnectionSnapshot(connection, tick)};
        sampleSize = snapshot.size();
        m_transport->send(connection, snapshot, false);
    }
    if (tick % 64 == 0) {  // temp: watch snapshot size stay small as grids grow
        std::cout << "[net] snapshot tick=" << tick << " bytes=" << sampleSize
                  << " grids=" << m_gameBase->getGridSubsystem()->getGrids().size()
                  << std::endl;
    }
}

void GameNetworkServer::sendStructural(INetworkTransport::ConnectionId connection,
                                       const std::vector<StructuralCommand>& commands) {
    std::vector<std::byte> message{buildStructuralMessage(commands)};
    if (message.empty()) {
        return;
    }
    m_transport->send(connection, message, true);
}

void GameNetworkServer::applyPendingInputs() {
    for (auto it{m_pendingInputs.begin()}; it != m_pendingInputs.end();) {
        const auto& [characterId, pending] = *it;
        if (pending.m_controlling) {
            applyResolvedInput(characterId, pending.m_input);
            forceDockingStatus(characterId, pending.m_docking);
            ++it;
        } else {
            // The claim ended (control released or source disconnected): stop the
            // held movement once, leave view/lock/docking as they are, and forget
            // the claim so nothing stale can ever re-apply.
            DigibotInput neutral{captureResolvedInput(characterId)};
            neutral.m_movementDirection = glm::ivec3{0, 0, 0};
            neutral.m_rollInput = 0;
            applyResolvedInput(characterId, neutral);
            it = m_pendingInputs.erase(it);
        }
    }
}

void GameNetworkServer::processManifests() {
    if (m_pendingManifests.empty()) {
        return;
    }
    std::uint64_t tick{m_gameBase->getPhysicsTick()};
    auto recentlyEdited = [&](std::uint64_t gridId) {
        auto it{m_lastStructureEditTick.find(gridId)};
        return it != m_lastStructureEditTick.end() &&
               tick - it->second < k_recentEditWindowTicks;
    };

    for (const auto& [connection, manifest] : m_pendingManifests) {
        std::map<std::uint64_t, std::uint64_t> clientHashes{manifest.begin(), manifest.end()};
        for (const std::shared_ptr<Grid>& grid :
             m_gameBase->getGridSubsystem()->getGrids()) {
            auto it{clientHashes.find(grid->uniqueId)};
            bool matches{it != clientHashes.end() &&
                         it->second == static_cast<std::uint64_t>(
                                           GridSerializer::structureHash(*grid))};
            if (it != clientHashes.end()) {
                clientHashes.erase(it);
            }
            if (matches || recentlyEdited(grid->uniqueId)) {
                continue;
            }
            ByteWriter writer{};
            writer.write(static_cast<std::uint8_t>(MessageType::GridData));
            writer.write(tick);  // capture moment, so the receiver can align the pose
            GridSerializer::serialize(*grid, writer);
            m_transport->send(connection, writer.take(), true);
            // The client rebuilds the body from that payload, one round trip stale.
            // Due it immediately so the next snapshot re-anchors it, rather than
            // leaving it to coast until its paced turn comes around.
            m_gridSync[connection][grid->uniqueId].m_nextSendTick = 0;
            std::cout << "[net] grid " << grid->uniqueId
                      << " corrected via full resend" << std::endl;
        }

        // Ids the client holds that the authority does not: despawn corrections,
        // unless a just-despawned grid's echo is still in flight.
        std::vector<StructuralCommand> despawns{};
        for (const auto& [id, hash] : clientHashes) {
            if (!recentlyEdited(id)) {
                despawns.push_back(StructuralCommand::despawnGrid(id));
            }
        }
        sendStructural(connection, despawns);
    }
    m_pendingManifests.clear();
}

std::vector<std::byte> GameNetworkServer::buildConnectionSnapshot(
    INetworkTransport::ConnectionId connection, std::uint64_t tick) {
    StateSnapshot snapshot{};
    snapshot.m_tick = tick;

    // Characters are the viewpoints and always relevant, so every connection gets
    // all of them each tick.
    for (const std::shared_ptr<Character>& character :
         m_gameBase->m_characterSubsystem->getCharacters()) {
        if (std::shared_ptr<RigidBody> body{character->getRigidBody().lock()}) {
            std::int32_t id{character->getUniqueId()};
            snapshot.m_characters.push_back({id, RigidBodyState::capture(*body),
                                             captureResolvedInput(id),
                                             captureDockingStatus(id)});
        }
    }

    // Reference frame: this connection's controlled character. All grid velocity is
    // measured against it, so a cube resting on a fast platform reads as still.
    glm::dvec3 refPos{0.0, 0.0, 0.0};
    glm::dvec3 refVel{0.0, 0.0, 0.0};
    for (const auto& [characterId, owner] : m_characterOwners) {
        if (owner == connection) {
            if (std::shared_ptr<RigidBody> body{findCharacterBody(characterId).lock()}) {
                refPos = body->getPosition();
                refVel = body->m_velocity;
            }
            break;
        }
    }

    // Deadline every grid, retire sync state for grids that no longer exist, and
    // collect the ones due to send.
    std::map<std::uint64_t, GridSyncState>& sync{m_gridSync[connection]};
    struct Candidate {
        std::uint64_t m_id{0};
        double m_deadline{0.0};
        RigidBodyState m_state{};
    };
    std::vector<Candidate> due{};
    std::vector<std::uint64_t> living{};
    for (const std::shared_ptr<Grid>& grid : m_gameBase->getGridSubsystem()->getGrids()) {
        std::shared_ptr<RigidBody> body{grid->getRigidBody().lock()};
        if (!body) {
            continue;
        }
        living.push_back(grid->uniqueId);
        GridSyncState& state{sync[grid->uniqueId]};

        double errorRate{glm::length(body->m_velocity - state.m_lastSentVelocity)};
        state.m_disturbanceEnvelope =
            std::max(errorRate, state.m_disturbanceEnvelope * k_syncEnvelopeDecay);
        if (tick < state.m_nextSendTick) {
            continue;
        }
        due.push_back({grid->uniqueId,
                       resendDeadline(*grid, *body, refPos, refVel, state.m_disturbanceEnvelope),
                       RigidBodyState::capture(*body)});
    }
    std::erase_if(sync, [&living](const auto& entry) {
        return std::find(living.begin(), living.end(), entry.first) == living.end();
    });

    // Soonest deadline first; grid id breaks ties so the order is reproducible. The
    // sort reads the raw deadline, not the clamped one, so grids pinned to the floor
    // still order among themselves instead of being handed out by id. Fill the
    // snapshot until the byte budget is spent, then reschedule what was sent.
    std::sort(due.begin(), due.end(), [](const Candidate& a, const Candidate& b) {
        return a.m_deadline != b.m_deadline ? a.m_deadline < b.m_deadline : a.m_id < b.m_id;
    });
    std::size_t budget{k_gridSnapshotBudgetBytes};
    for (const Candidate& candidate : due) {
        if (budget < k_gridEntryBytes) {
            break;
        }
        budget -= k_gridEntryBytes;
        snapshot.m_grids.push_back({candidate.m_id, candidate.m_state});
        double interval{std::clamp(candidate.m_deadline, k_minIntervalTicks,
                                   k_keyframeIntervalTicks)};
        GridSyncState& state{sync[candidate.m_id]};
        state.m_lastSentVelocity = candidate.m_state.m_velocity;
        state.m_nextSendTick = tick + static_cast<std::uint64_t>(interval);
    }

    ByteWriter writer{};
    writer.write(static_cast<std::uint8_t>(MessageType::StateSnapshot));
    snapshot.serialize(writer);
    return writer.take();
}
