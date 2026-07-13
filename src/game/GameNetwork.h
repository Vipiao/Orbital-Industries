// GameNetwork.h — state sync between server and clients over an INetworkTransport.
// Both sides build the identical initial world, so objects match by their
// deterministic ids. Test-stage ownership: the server owns all grids, the client
// owns all characters; each side applies only the state kinds it does not own.
#pragma once

#include "../network/INetworkTransport.h"

#include <cstddef>
#include <cstdint>
#include <map>
#include <vector>

class GameBase;

class GameNetwork {
public:
    GameNetwork(INetworkTransport* transport, GameBase* gameBase);

    // Call at the top of the frame: pumps the transport, tracks connections,
    // and stashes the newest received snapshot per sender.
    void framePoll();

    // Call in the step-control window (AwaitingStepControl): applies stashed
    // snapshots to the world. The step yield guarantees a clean boundary —
    // remote state lands exactly once per tick, before integration.
    void stepApply();

    // Call after the frame's physics: sends the owned state to every connection
    // when a new physics tick has completed. Snapshots go unreliable — a lost
    // one is superseded by the next tick's — stamped with the sender's tick so
    // receivers can drop stale or reordered arrivals.
    void frameSend();

private:
    void applySnapshot(const std::vector<std::byte>& data);
    std::vector<std::byte> buildSnapshot(std::uint64_t tick) const;

    INetworkTransport* m_transport;
    GameBase* m_gameBase;

    std::vector<INetworkTransport::ConnectionId> m_connections{};
    // Newest unapplied snapshot per sender; fresher arrivals overwrite it.
    std::map<INetworkTransport::ConnectionId, std::vector<std::byte>> m_pendingSnapshots{};
    std::map<INetworkTransport::ConnectionId, std::uint64_t> m_lastReceivedTick{};
    std::uint64_t m_lastSentTick{0};
};
