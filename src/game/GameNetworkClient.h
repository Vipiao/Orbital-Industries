// GameNetworkClient.h — the predicting half of GameNetwork: sends local input and
// world manifests, applies snapshots and reliable corrections from the server, and
// reconciles this peer's own predicted character against the server's anchor.
#pragma once

#include "GameNetworkBase.h"
#include "PlayerPrediction.h"
#include "TickTimelineFilter.h"

#include "../game_base/RigidBodyState.h"

#include <cstdint>
#include <map>
#include <vector>

class GameNetworkClient : public GameNetworkBase {
public:
    GameNetworkClient(INetworkTransport* transport, GameBase* gameBase);

private:
    void updateCharacterControl(Mode& mode) override;
    void setLocalControlsCharacter(bool controls) override;
    void handleMessage(MessageType type, ByteReader& reader,
                       INetworkTransport::Message& message) override;
    void onDisconnected(INetworkTransport::ConnectionId connection) override;
    void stepApplyRole() override;
    void frameSendRole(std::uint64_t tick) override;

    void applyStateSnapshot(const std::vector<std::byte>& data);
    // Replace this peer's copy of the grid with the full content of a GridData
    // correction (despawn if present, then materialize).
    void applyGridData(const std::vector<std::byte>& data);
    // What this peer's world holds, as {gridId, structureHash} pairs.
    std::vector<std::byte> buildWorldManifest() const;
    std::vector<std::byte> buildPlayerInput(std::uint64_t tick) const;

    PlayerPrediction m_prediction{};
    // The local player's authoritative state, time-aligned like every other
    // applied body and advanced one tick per step between snapshots. m_prediction
    // reconciles the predicted body against it each step.
    RigidBodyState m_predictionAnchor{};
    bool m_hasPredictionAnchor{false};

    // Newest unapplied snapshot per sender; fresher arrivals overwrite it.
    std::map<INetworkTransport::ConnectionId, std::vector<std::byte>> m_pendingSnapshots{};
    std::map<INetworkTransport::ConnectionId, std::uint64_t> m_lastSnapshotTick{};
    // Server tick of the last applied snapshot: the moment the world this client
    // holds is standing at, so a grid rebuilt from an older payload can join it.
    std::uint64_t m_lastAppliedSnapshotTick{0};

    // Aligns snapshot states to the local tick timeline; consumed in
    // applyStateSnapshot. Session-scoped: reset on disconnect.
    TickTimelineFilter m_timelineFilter{};

    // Manifest send cadence (first one goes out immediately on connect).
    bool m_manifestSent{false};
    std::uint64_t m_lastManifestTick{0};
};
