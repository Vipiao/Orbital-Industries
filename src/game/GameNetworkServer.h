// GameNetworkServer.h — the authoritative half of GameNetwork: arbitrates
// character control, applies structural edits, reconciles client manifests
// against the world, and broadcasts state snapshots.
#pragma once

#include "GameNetworkBase.h"

#include <cstdint>
#include <map>
#include <optional>
#include <utility>
#include <vector>

#include <glm/glm.hpp>

class GameNetworkServer : public GameNetworkBase {
public:
    GameNetworkServer(INetworkTransport* transport, GameBase* gameBase);

private:
    void updateCharacterControl(Mode& mode) override;
    void setLocalControlsCharacter(bool controls) override;
    void handleMessage(MessageType type, ByteReader& reader,
                       INetworkTransport::Message& message) override;
    void onDisconnected(INetworkTransport::ConnectionId connection) override;
    void stepApplyRole() override;
    void frameSendRole(std::uint64_t tick) override;

    // Arbitration: commit characterId to owner if it names a digibot that exists
    // and has no owner yet. Returns characterId back, or -1 (denied). characterId
    // must already be resolved to a specific id — callers translate a "-1 = any"
    // request via findAnyFreeCharacter before calling this.
    int acquireCharacter(const std::optional<INetworkTransport::ConnectionId>& owner,
                         int characterId);
    // Whether characterId names a digibot that exists and has no owner yet.
    bool isCharacterAvailable(int characterId) const;
    // The lowest-id digibot with no owner yet, or -1 if every digibot is taken.
    int findAnyFreeCharacter() const;
    // End everything the owner holds — ownerships freed, live input claims marked
    // ended (neutralized at the next step). Used for an explicit release and for
    // a disconnect alike.
    void endControlOf(const std::optional<INetworkTransport::ConnectionId>& owner);

    // Drive remotely-owned characters from their owners' inputs.
    void applyPendingInputs();
    // Diff each client's manifest against the authoritative world and send
    // corrections (full grids, despawns) to that client only.
    void processManifests();
    // Reliable structural-edit channel, addressed to one client: corrections
    // (manifest diffs). Broadcasting the same channel to everyone (the server's
    // applied edits) has no state worth a named method for and goes straight
    // through broadcast() at the call site.
    void sendStructural(INetworkTransport::ConnectionId connection,
                        const std::vector<StructuralCommand>& commands);
    // The state snapshot addressed to one connection — the characters plus the
    // grids this connection's scheduler picks this tick, paced by how much each
    // grid affects that client's view. Mutates the connection's sync state, so it
    // is built once per connection per tick.
    std::vector<std::byte> buildConnectionSnapshot(INetworkTransport::ConnectionId connection,
                                                   std::uint64_t tick);

    // Which connection holds each character; nullopt marks the server's own
    // player. Absent id = available. Guards the input path: claims from a
    // connection that does not own the character are rejected.
    std::map<int, std::optional<INetworkTransport::ConnectionId>> m_characterOwners{};

    // Newest input per character, held until superseded so a lost packet does not
    // stutter the character. An input is a *claim* by its source connection, valid
    // only while that connection owns the character; the claim ends when the owner
    // releases control or disconnects (m_controlling flips false), and ending a
    // claim neutralizes the held movement exactly once. The docking status rides
    // with the input; both are level-synced control state owned by the sender.
    struct PendingInput {
        DigibotInput m_input{};
        CockpitDockingCoordinator::DockingStatus m_docking{};
        INetworkTransport::ConnectionId m_source{};
        bool m_controlling{false};
    };
    std::map<int, PendingInput> m_pendingInputs{};
    std::map<int, std::uint64_t> m_lastInputTick{};

    // Tick each grid's structure last changed, updated as commands are applied. A
    // manifest mismatch on a recently-edited grid is presumed to be the in-flight
    // echo and skipped; the next manifest re-checks it.
    std::map<std::uint64_t, std::uint64_t> m_lastStructureEditTick{};

    // Newest manifest per client, diffed and cleared at the next step.
    std::map<INetworkTransport::ConnectionId,
             std::vector<std::pair<std::uint64_t, std::uint64_t>>> m_pendingManifests{};

    // Per-connection grid position-sync scheduler. Each grid carries the velocity
    // last sent to this client (so its coasting error reads as a frame independent
    // velocity difference — never an absolute speed), a peak-held envelope of that
    // error for the lost-packet hedge, and its next scheduled resend tick. A grid
    // absent from a connection's map has not been tracked yet.
    struct GridSyncState {
        glm::dvec3 m_lastSentVelocity{0.0, 0.0, 0.0};
        double m_disturbanceEnvelope{0.0};
        std::uint64_t m_nextSendTick{0};
    };
    std::map<INetworkTransport::ConnectionId,
             std::map<std::uint64_t, GridSyncState>> m_gridSync{};
};
