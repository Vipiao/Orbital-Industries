#include "GameNetwork.h"

#include "../characters/Character.h"
#include "../characters/CharacterSubsystem.h"
#include "../game_base/GameBase.h"
#include "../game_base/Grid.h"
#include "../game_base/GridSubsystem.h"
#include "../physics/PhysicsEngine.h"
#include "../physics/RigidBody.h"

#include <cassert>
#include <cmath>
#include <cstring>
#include <iostream>

namespace {

// Wire format (native x86 little-endian byte order on both ends):
//   uint64 tick
//   uint32 gridCount,      per grid:      uint64 id, body state
//   uint32 characterCount, per character: int64 id,  body state
// Body state is 13 doubles: position xyz, orientation wxyz, velocity xyz,
// angular momentum (body space) xyz.

class ByteWriter {
public:
    template <typename T>
    void write(const T& value) {
        std::size_t offset{m_data.size()};
        m_data.resize(offset + sizeof(T));
        std::memcpy(m_data.data() + offset, &value, sizeof(T));
    }

    std::vector<std::byte> take() { return std::move(m_data); }

private:
    std::vector<std::byte> m_data{};
};

class ByteReader {
public:
    explicit ByteReader(const std::vector<std::byte>& data) : m_data{data} {}

    template <typename T>
    bool read(T& out) {
        if (m_offset + sizeof(T) > m_data.size()) {
            return false;
        }
        std::memcpy(&out, m_data.data() + m_offset, sizeof(T));
        m_offset += sizeof(T);
        return true;
    }

private:
    const std::vector<std::byte>& m_data;
    std::size_t m_offset{0};
};

struct BodyState {
    glm::dvec3 m_position{};
    glm::dquat m_orientation{1.0, 0.0, 0.0, 0.0};
    glm::dvec3 m_velocity{};
    glm::dvec3 m_angularMomentumBody{};
};

void writeBodyState(ByteWriter& writer, const RigidBody& body) {
    writer.write(body.m_position.x);
    writer.write(body.m_position.y);
    writer.write(body.m_position.z);
    writer.write(body.m_orientation.w);
    writer.write(body.m_orientation.x);
    writer.write(body.m_orientation.y);
    writer.write(body.m_orientation.z);
    writer.write(body.m_velocity.x);
    writer.write(body.m_velocity.y);
    writer.write(body.m_velocity.z);
    writer.write(body.m_angularMomentumBody.x);
    writer.write(body.m_angularMomentumBody.y);
    writer.write(body.m_angularMomentumBody.z);
}

bool readBodyState(ByteReader& reader, BodyState& out) {
    return reader.read(out.m_position.x) && reader.read(out.m_position.y) &&
           reader.read(out.m_position.z) && reader.read(out.m_orientation.w) &&
           reader.read(out.m_orientation.x) && reader.read(out.m_orientation.y) &&
           reader.read(out.m_orientation.z) && reader.read(out.m_velocity.x) &&
           reader.read(out.m_velocity.y) && reader.read(out.m_velocity.z) &&
           reader.read(out.m_angularMomentumBody.x) &&
           reader.read(out.m_angularMomentumBody.y) &&
           reader.read(out.m_angularMomentumBody.z);
}

// Remote input is untrusted: non-finite values would poison the integrator,
// so a bad body state is dropped, never asserted on.
bool isFinite(const BodyState& state) {
    return std::isfinite(state.m_position.x) && std::isfinite(state.m_position.y) &&
           std::isfinite(state.m_position.z) && std::isfinite(state.m_orientation.w) &&
           std::isfinite(state.m_orientation.x) && std::isfinite(state.m_orientation.y) &&
           std::isfinite(state.m_orientation.z) && std::isfinite(state.m_velocity.x) &&
           std::isfinite(state.m_velocity.y) && std::isfinite(state.m_velocity.z) &&
           std::isfinite(state.m_angularMomentumBody.x) &&
           std::isfinite(state.m_angularMomentumBody.y) &&
           std::isfinite(state.m_angularMomentumBody.z);
}

void applyBodyState(GameBase* gameBase, const std::weak_ptr<RigidBody>& bodyWeak,
                    const BodyState& state) {
    std::shared_ptr<RigidBody> body{bodyWeak.lock()};
    if (!body || !isFinite(state)) {
        return;
    }
    body->m_position = state.m_position;
    body->m_velocity = state.m_velocity;
    body->m_orientation = state.m_orientation;
    body->invalidateOrientation();
    body->m_angularMomentumBody = state.m_angularMomentumBody;
    body->invalidateAngularMomentum();
    gameBase->m_physicsEngine->updateColliderTransform(bodyWeak);
}

std::weak_ptr<RigidBody> findGridBody(GameBase* gameBase, std::uint64_t id) {
    std::shared_ptr<Grid> grid{gameBase->getGridSubsystem()->getGridById(id).lock()};
    return grid ? grid->getRigidBody() : std::weak_ptr<RigidBody>{};
}

std::weak_ptr<RigidBody> findCharacterBody(GameBase* gameBase, std::int64_t id) {
    std::shared_ptr<Character> character{
        gameBase->m_characterSubsystem->getCharacterById(static_cast<int>(id)).lock()};
    return character ? character->getRigidBody() : std::weak_ptr<RigidBody>{};
}

}  // namespace

GameNetwork::GameNetwork(INetworkTransport* transport, GameBase* gameBase)
    : m_transport{transport}, m_gameBase{gameBase} {
    assert(m_transport != nullptr && m_gameBase != nullptr);
}

void GameNetwork::framePoll() {
    m_transport->poll();

    for (const INetworkTransport::Event& event : m_transport->drainEvents()) {
        if (event.m_type == INetworkTransport::EventType::Connected) {
            std::cout << "[net] connected: " << event.m_detail << std::endl;
            m_connections.push_back(event.m_connection);
        } else {
            std::cout << "[net] disconnected: " << event.m_detail << std::endl;
            std::erase(m_connections, event.m_connection);
            m_pendingSnapshots.erase(event.m_connection);
            m_lastReceivedTick.erase(event.m_connection);
        }
    }

    for (INetworkTransport::Message& message : m_transport->drainMessages()) {
        std::uint64_t tick{0};
        if (!ByteReader{message.m_data}.read(tick)) {
            continue;
        }
        auto lastTick{m_lastReceivedTick.find(message.m_connection)};
        if (lastTick != m_lastReceivedTick.end() && tick <= lastTick->second) {
            continue;  // stale or reordered snapshot; a newer one already arrived
        }
        m_lastReceivedTick[message.m_connection] = tick;
        m_pendingSnapshots[message.m_connection] = std::move(message.m_data);
    }
}

void GameNetwork::stepApply() {
    assert(m_gameBase->isAtStepControlPoint());
    for (const auto& [connection, snapshot] : m_pendingSnapshots) {
        applySnapshot(snapshot);
    }
    m_pendingSnapshots.clear();
}

void GameNetwork::frameSend() {
    std::uint64_t tick{m_gameBase->getPhysicsTick()};
    // Mid-step body state is torn (partially integrated), so wait for the next
    // frame; the snapshot will carry the newer tick then.
    if (tick == m_lastSentTick || m_gameBase->isPhysicsStepInProgress()) {
        return;
    }
    m_lastSentTick = tick;
    if (m_connections.empty()) {
        return;
    }
    std::vector<std::byte> snapshot{buildSnapshot(tick)};
    for (INetworkTransport::ConnectionId connection : m_connections) {
        m_transport->send(connection, snapshot, false);
    }
}

std::vector<std::byte> GameNetwork::buildSnapshot(std::uint64_t tick) const {
    ByteWriter writer{};
    writer.write(tick);

    // The server owns and sends grids; characters are client-owned, but the
    // server relays them too (clients ignore what they own). A client sends
    // only characters.
    std::vector<std::pair<std::uint64_t, std::shared_ptr<RigidBody>>> grids{};
    if (m_transport->isServer()) {
        for (const std::shared_ptr<Grid>& grid :
             m_gameBase->getGridSubsystem()->getGrids()) {
            std::shared_ptr<RigidBody> body{grid->getRigidBody().lock()};
            if (body) {
                grids.push_back({grid->uniqueId, std::move(body)});
            }
        }
    }
    writer.write(static_cast<std::uint32_t>(grids.size()));
    for (const auto& [id, body] : grids) {
        writer.write(id);
        writeBodyState(writer, *body);
    }

    std::vector<std::pair<std::int64_t, std::shared_ptr<RigidBody>>> characters{};
    for (const std::shared_ptr<Character>& character :
         m_gameBase->m_characterSubsystem->getCharacters()) {
        std::shared_ptr<RigidBody> body{character->getRigidBody().lock()};
        if (body) {
            characters.push_back({character->getUniqueId(), std::move(body)});
        }
    }
    writer.write(static_cast<std::uint32_t>(characters.size()));
    for (const auto& [id, body] : characters) {
        writer.write(id);
        writeBodyState(writer, *body);
    }

    return writer.take();
}

void GameNetwork::applySnapshot(const std::vector<std::byte>& data) {
    // Ownership filter: apply only state kinds this side does not own.
    bool applyGrids{!m_transport->isServer()};
    bool applyCharacters{m_transport->isServer()};

    ByteReader reader{data};
    std::uint64_t tick{0};
    std::uint32_t gridCount{0};
    if (!reader.read(tick) || !reader.read(gridCount)) {
        std::cout << "[net] dropped malformed snapshot" << std::endl;
        return;
    }
    for (std::uint32_t ii = 0; ii < gridCount; ii++) {
        std::uint64_t id{0};
        BodyState state{};
        if (!reader.read(id) || !readBodyState(reader, state)) {
            std::cout << "[net] dropped malformed snapshot" << std::endl;
            return;
        }
        if (applyGrids) {
            applyBodyState(m_gameBase, findGridBody(m_gameBase, id), state);
        }
    }

    std::uint32_t characterCount{0};
    if (!reader.read(characterCount)) {
        std::cout << "[net] dropped malformed snapshot" << std::endl;
        return;
    }
    for (std::uint32_t ii = 0; ii < characterCount; ii++) {
        std::int64_t id{0};
        BodyState state{};
        if (!reader.read(id) || !readBodyState(reader, state)) {
            std::cout << "[net] dropped malformed snapshot" << std::endl;
            return;
        }
        if (applyCharacters) {
            applyBodyState(m_gameBase, findCharacterBody(m_gameBase, id), state);
        }
    }
}
