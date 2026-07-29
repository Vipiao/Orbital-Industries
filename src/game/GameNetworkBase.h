// GameNetworkBase.h — shared plumbing for GameNetworkServer and GameNetworkClient:
// connection tracking, the reliable-message dispatch loop, character-control
// arbitration's shared skeleton, and wire-format helpers both roles need.
//
// The implemented design is documented in notes/multiplayer_implementation.md
// (rationale: notes/multiplayer_design.md).
//
// The server is the single authority (Beta, see notes/multiplayer_design.md §5). It
// simulates the whole world and broadcasts a state snapshot every tick. Authority
// never moves; only who supplies the character's *input* does:
//   - Client in control mode: it drives the character, predicts it locally, and
//     sends its input to the server. It reconciles its prediction against the
//     server's snapshot instead of snapping to it.
//   - Server in control mode: the server supplies the input from its own local
//     controls. Being authoritative and local, it needs no prediction, and it
//     ignores client input while it holds control.
//   - A peer that does not control the character just applies the server's state
//     for it directly (no prediction).
//
// Which objects a client predicts vs. applies directly is decided here. Today that
// set is "the character, when this client controls it"; a piloted grid will join it
// later. Both sides build the identical initial world, so objects match by their
// deterministic ids. There is no create/destroy replication yet.
//
// The role (server or client) is fixed for the object's entire lifetime, decided
// once by Game from the transport. GameNetworkServer and GameNetworkClient each hold
// only the state their role needs; this base class holds only what both genuinely
// share.
#pragma once

#include "../characters/digibot/DigibotInput.h"
#include "../game_base/StructuralCommand.h"
#include "../game_base/cockpit/CockpitDockingCoordinator.h"
#include "../network/INetworkTransport.h"

#include <cstddef>
#include <cstdint>
#include <limits>
#include <memory>
#include <optional>
#include <span>
#include <variant>
#include <vector>

class ByteReader;
class Character;
class Digibot;
class DigibotController;
class GameBase;
class Mode;
class RigidBody;

class GameNetworkBase {
public:
    GameNetworkBase(const GameNetworkBase&) = delete;
    GameNetworkBase& operator=(const GameNetworkBase&) = delete;
    virtual ~GameNetworkBase() = default;

    // Whether this peer is the arbiter (host).
    bool isServer() const { return m_transport->isServer(); }

    // Character-control arbitration, plus binding the result to mode and telling
    // the network layer whether this peer now supplies the character's input
    // (setLocalControlsCharacter, called internally once binding is resolved —
    // see bindModeToLocalPlayer). The server is the sole arbiter: a request asks
    // for mode's preferred character (granted only if free — no silent
    // substitute) or, with none preferred, for any and gets the lowest-id free
    // digibot. Two peers can never drive the same character.
    //
    // Call every frame: reads mode's current desire (wantsCharacterControl,
    // desiredCharacter), and binds/unbinds mode to whatever that resolves to
    // (bindCharacter). The desire is only consulted on the rising edge (the frame
    // it turns on) and otherwise ignored: ask once when desire turns on, give
    // everything back unconditionally when it turns off. A denial or an
    // outstanding request is not tracked as separate state — nothing re-asks
    // while desire stays on. How "ask" and "give back" actually work differs
    // enough by role (the host grants itself directly; a client sends a request
    // and waits for the ControlGrant reply, and must handle one arriving after
    // it has stopped wanting control) that each role implements this whole
    // method itself, using readControlDesire/bindModeToLocalPlayer below.
    virtual void updateCharacterControl(Mode& mode) = 0;

    // One frame's protocol, driven by Game at advanceFrame's yields: framePoll,
    // stepApply at every step-control yield (exactly once each), frameSend.

    // Call at the top of the frame: pumps the transport, tracks connections, and
    // dispatches each drained message to the role's handleMessage.
    void framePoll();

    // Call in the step-control window (AwaitingStepControl), a clean
    // pre-integration boundary. The server applies the controlling client's input;
    // the client applies the snapshot and reconciles any object it predicts.
    void stepApply();

    // Call after the frame's physics: the server broadcasts a state snapshot; a
    // controlling client sends its input. Fires once per completed physics tick.
    void frameSend();

protected:
    // Leading byte of every message: which channel it belongs to.
    //
    // Wire formats (native little-endian on both ends). Every message leads with
    // the uint8 type; the StateSnapshot payload is defined by StateSnapshot.h.
    // A peer applies a snapshot's per-character input and docking status to
    // characters it does not control so their mode, animation and cockpit
    // engagement follow, while the pose stays authoritative from the
    // RigidBodyState.
    // PlayerInput wire format:
    //   uint8 type, uint64 tick, int32 characterId, DigibotInput, DockingStatus.
    // Accepted only from the character's owning connection (see control
    // arbitration), so a straggler packet from a previous owner can never disturb
    // the character.
    // Control arbitration (all reliable):
    //   ControlRequest: uint8 type, int32 preferredCharacterId — asks for that
    //                   character (-1 = any); a taken preference is denied, never
    //                   silently substituted.
    //   ControlGrant:   uint8 type, int32 characterId (-1 = denied).
    //   ControlRelease: uint8 type — gives back everything the sender holds; also
    //                   implied by a disconnect.
    // WorldManifest wire format (client -> server, reliable, periodic):
    //   uint8 type, uint32 gridCount, per grid: uint64 id, uint64 structureHash.
    // GridData wire format (server -> client, reliable, same channel as Structural
    // so corrections stay ordered against the edit stream):
    //   uint8 type, GridSerializer::serialize payload.
    enum class MessageType : std::uint8_t {
        StateSnapshot = 0,
        PlayerInput = 1,
        Structural = 2,
        WorldManifest = 3,   // client -> server: {gridId, structureHash} per grid held
        GridData = 4,        // server -> client: one full grid (GridSerializer bytes)
        ControlRequest = 5,  // client -> server: give me an available character
        ControlGrant = 6,    // server -> client: the granted character id (-1 denied)
        ControlRelease = 7,  // client -> server: I give back what I hold
    };

    GameNetworkBase(INetworkTransport* transport, GameBase* gameBase);

    // Send to every live connection: all clients on the server, the server on a
    // client (its one connection). No-ops on empty data, so a caller building an
    // occasionally-empty message doesn't need to check first.
    void broadcast(std::span<const std::byte> data, bool reliable);

    // Wire-format helpers shared by both roles.
    std::weak_ptr<RigidBody> findGridBody(std::uint64_t id) const;
    std::weak_ptr<RigidBody> findCharacterBody(std::int32_t id) const;
    // Reverse of findGridBody: which grid owns this body, or k_noLockTargetGridId
    // when none (e.g. unlocked, or a body that isn't a grid's).
    std::uint64_t findGridIdForBody(const std::weak_ptr<RigidBody>& body) const;
    std::shared_ptr<Digibot> findDigibot(int characterId) const;
    DigibotController* findDigibotController(int characterId) const;
    // The character's docking situation as a value; non-digibots are always FREE.
    CockpitDockingCoordinator::DockingStatus captureDockingStatus(int characterId) const;
    // Reconcile a character this peer does not control toward its owner's situation.
    void forceDockingStatus(int characterId,
                            const CockpitDockingCoordinator::DockingStatus& status);
    // The character's current control intent, with its lock target resolved to a
    // grid id. Non-digibot characters have no controller and yield a default
    // (idle) input.
    DigibotInput captureResolvedInput(int characterId) const;
    // Drive a character from a relayed input, resolving its lock target grid id
    // back to a body. Used for characters this peer does not itself control.
    void applyResolvedInput(int characterId, const DigibotInput& input);
    // Empty when commands is, so callers can tell "nothing to send" from the
    // bytes alone rather than checking commands.empty() themselves.
    std::vector<std::byte> buildStructuralMessage(
        const std::vector<StructuralCommand>& commands) const;
    // Decode a Structural payload and queue its commands in arrival order. Shared
    // because both roles keep the same reliable-channel ordering discipline.
    void receiveStructural(ByteReader& reader);

    // The result of consulting mode's desire once, for updateCharacterControl.
    struct ControlDesire {
        bool m_wantsControl{false};
        // Whether the mode wanted a character as of the previous call: compare
        // against m_wantsControl to tell a rising edge (m_wantsControl &&
        // !m_wasWanting) from a falling one (!m_wantsControl && m_wasWanting).
        bool m_wasWanting{false};
        // Only meaningful while unbound; -1 means no preference.
        int m_preferredCharacterId{-1};
    };
    // Reads mode's current desire and preferred character, and advances the
    // rising/falling edge detector (m_wasWantingCharacterControl) for next call.
    // Pure with respect to role: every peer resolves "what does mode want"
    // identically, only what to do about it differs.
    ControlDesire readControlDesire(Mode& mode);
    // Binds mode to the local player's character, or unbinds if control isn't
    // wanted, then tells the network layer whether this peer now supplies the
    // character's input (mode.isControllingCharacter() is only current once the
    // bind above is resolved, so this is the one correct place to read it).
    // Shared because both roles resolve this the same way once m_localPlayerId
    // is current.
    void bindModeToLocalPlayer(Mode& mode, bool wantsControl);

    // Role-specific halves, implemented once per concrete class.
    virtual void handleMessage(MessageType type, ByteReader& reader,
                               INetworkTransport::Message& message) = 0;
    virtual void onDisconnected(INetworkTransport::ConnectionId connection) = 0;
    virtual void stepApplyRole() = 0;
    virtual void frameSendRole(std::uint64_t tick) = 0;
    // Whether this peer supplies the character's input this frame, called from
    // bindModeToLocalPlayer once binding is resolved. Only the client has
    // anything extra to do on the falling edge (abandon its prediction), so
    // this is implemented per role rather than through a shared hook.
    virtual void setLocalControlsCharacter(bool controls) = 0;

    INetworkTransport* m_transport;
    GameBase* m_gameBase;

    // The character granted to this peer (-1 = none). Set by the arbitration
    // flow, never assigned by hand.
    int m_localPlayerId{-1};
    bool m_localControlsCharacter{false};
    // Whether the mode wanted a character as of the last updateCharacterControl
    // call: the edge detector that drives (re-)asking on the rising edge and
    // releasing on the falling edge. Read as "currently wanted" by anything that
    // runs later in the same frame (e.g. a ControlGrant reply), since it is
    // updated before framePoll runs.
    bool m_wasWantingCharacterControl{false};

    std::vector<INetworkTransport::ConnectionId> m_connections{};

    // Reliable-channel traffic pending apply, in exact arrival order: structural
    // commands interleaved with full-grid corrections (GridData payloads, client
    // only). One queue, not two, because the channel's ordering is what keeps a
    // correction consistent with the commands sent around it.
    std::vector<std::variant<StructuralCommand, std::vector<std::byte>>> m_receivedReliable{};

    std::uint64_t m_lastSentTick{0};

    // The physics tick stepApply last ran for; guards the once-per-step contract.
    std::uint64_t m_lastStepApplyTick{std::numeric_limits<std::uint64_t>::max()};
};
