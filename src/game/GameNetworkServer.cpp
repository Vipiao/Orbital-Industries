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

// Grid position-sync scheduler (per connection). A grid's send priority is its
// on-screen size times how wrong the client's coasting is getting, so the snapshot
// spends its byte budget where the player would actually notice.
//   angular  = radius / max(distance, radius)          — how much of the view it fills
//   envelope = peak-held |velocity − lastSentVelocity|  — coasting-error rate (frame free)
//   closing  = max(0, (gridVel − charVel)·dirToPlayer)  — approaching, so soon relevant
//   score    = angular · (base + wEnv·envelope/vRef + wClose·closing/vRef)
//   interval = baseInterval / (1 + score), clamped      — ticks until the next resend
// Even a score of zero resends every baseInterval, the keyframe floor that heals a
// lost update for a quiet grid and bounds worst-case staleness.
const std::size_t k_gridSnapshotBudgetBytes{6000};
const double k_baseIntervalTicks{PhysicsUnits::seconds(2.0)};
const double k_minIntervalTicks{0.5};
const double k_syncVelReference{PhysicsUnits::metersPerSecond(20.0)};
const double k_syncBaseWeight{0.5};
const double k_syncEnvelopeWeight{8.0};
const double k_syncClosingWeight{1.0};
// Peak-held envelope decay per tick, derived from a half-life in seconds (several
// round trips) so a grid that just slowed stays urgent long enough that a lost
// "slowed down" update is resent before the client's stale high velocity carries
// it far — and so the real-time half-life stays fixed if the tick rate changes.
const double k_syncEnvelopeHalfLifeTicks{PhysicsUnits::seconds(0.5)};
const double k_syncEnvelopeDecay{std::pow(0.5, 1.0 / k_syncEnvelopeHalfLifeTicks)};
// A grid entry is id + RigidBodyState; used to stop filling before the byte budget.
const std::size_t k_gridEntryBytes{sizeof(std::uint64_t) + 13 * sizeof(double)};

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
            GridSerializer::serialize(*grid, writer);
            m_transport->send(connection, writer.take(), true);
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
                refPos = body->m_position;
                refVel = body->m_velocity;
            }
            break;
        }
    }

    // Score every grid, retire sync state for grids that no longer exist, and
    // collect the ones due to send.
    std::map<std::uint64_t, GridSyncState>& sync{m_gridSync[connection]};
    struct Candidate {
        std::uint64_t m_id{0};
        double m_score{0.0};
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
        glm::dvec3 toPlayer{refPos - body->m_position};
        double distance{glm::length(toPlayer)};
        double radius{grid->getApproximateRadius()};
        double angular{radius / std::max(distance, radius)};
        double closing{distance > 0.0
                           ? std::max(0.0, glm::dot(body->m_velocity - refVel,
                                                    toPlayer / distance))
                           : 0.0};
        double score{angular *
                     (k_syncBaseWeight +
                      k_syncEnvelopeWeight * state.m_disturbanceEnvelope / k_syncVelReference +
                      k_syncClosingWeight * closing / k_syncVelReference)};
        due.push_back({grid->uniqueId, score, RigidBodyState::capture(*body)});
    }
    std::erase_if(sync, [&living](const auto& entry) {
        return std::find(living.begin(), living.end(), entry.first) == living.end();
    });

    // Highest score first; grid id breaks ties so the order is reproducible. Fill
    // the snapshot until the byte budget is spent, then reschedule what was sent.
    std::sort(due.begin(), due.end(), [](const Candidate& a, const Candidate& b) {
        return a.m_score != b.m_score ? a.m_score > b.m_score : a.m_id < b.m_id;
    });
    std::size_t budget{k_gridSnapshotBudgetBytes};
    for (const Candidate& candidate : due) {
        if (budget < k_gridEntryBytes) {
            break;
        }
        budget -= k_gridEntryBytes;
        snapshot.m_grids.push_back({candidate.m_id, candidate.m_state});
        double interval{std::clamp(k_baseIntervalTicks / (1.0 + candidate.m_score),
                                   k_minIntervalTicks, k_baseIntervalTicks)};
        GridSyncState& state{sync[candidate.m_id]};
        state.m_lastSentVelocity = candidate.m_state.m_velocity;
        state.m_nextSendTick = tick + static_cast<std::uint64_t>(interval);
    }

    ByteWriter writer{};
    writer.write(static_cast<std::uint8_t>(MessageType::StateSnapshot));
    snapshot.serialize(writer);
    return writer.take();
}
