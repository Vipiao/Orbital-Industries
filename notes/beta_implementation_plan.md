# Beta Implementation Plan

Companion to `multiplayer_design.md` §5. This is the concrete build plan for Beta:
server simulates everything and applies client inputs on arrival; the client applies
server state directly to everything except its own player, which it predicts locally
and pulls toward the server anchor with an exponential position+velocity blend.

Scope: movement netcode only. Structural replication (building, splitting) stays
out; the test world is built identically on both peers at startup, as today. The
known consequence: block edits during a session still desync — that is build-order
step 4 in the design doc, not part of this plan.

---

## Layering rule (the answer to "where does X live?")

**Game code exports mechanisms. The network layer owns policy.**

- A *mechanism* is a general-purpose function that makes sense with no network in
  the picture: capture/restore a rigid body's state, serialize it, apply an input
  struct to a character. Each doubles as a save-game, replay, or debug feature.
- A *policy* is anything that exists only because of latency: when to send, what
  each peer owns, the input channel, the anchor, the ring buffer, the blend.

So yes — **the buffered player states live in the network layer**, not in the game.
The ring buffer has no meaning without a delayed anchor to compare against; a game
with no network never wants it. The game's only contribution is the generic
`RigidBodyState` capture that the buffer stores. Dependency direction stays clean:
network includes physics/characters headers and calls their exported functions;
nothing under `src/physics`, `src/characters`, `src/game_base` ever includes
anything from `src/game` or `src/network`.

---

## Phase 1 — Serialization mechanisms (game side, network-unaware)

**1a. `src/serialization/ByteStream.h`** — move `ByteWriter`/`ByteReader` out of
`GameNetwork.cpp` into their own header. They are generic byte codecs, not network
code; save games will use the same ones. Header-only is fine at this size.

**1b. `src/physics/RigidBodyState.{h,cpp}`** — the dynamic-state snapshot as a
first-class physics type:

- struct of the 13 doubles: position, orientation, velocity, angular momentum
  (body space) — same fields `GameNetwork.cpp` already ships;
- `capture(const RigidBody&)` and `apply(RigidBody&, PhysicsEngine*)` (the apply
  includes the `invalidateOrientation` / `invalidateAngularMomentum` /
  `updateColliderTransform` dance currently open-coded in `applyBodyState`);
- `serialize(ByteWriter&)` / `deserialize(ByteReader&)`;
- `isFinite()` validation (moves here from `GameNetwork.cpp`).

This kills the anonymous-namespace `BodyState` in `GameNetwork.cpp`. It is also the
unit the ring buffer stores and the unit a future save game or rollback restores.

## Phase 2 — Input as data (game side; the one real refactor)

Today `DigibotPlayerController` drives the character through scattered setters
(`setMovementDirection`, `setRollInput`, `setViewDirection`, jetpack, lock target).
Beta needs the same inputs applied on the server, so input must become a value.

**2a. `src/characters/digibot/DigibotInput.h`** — plain struct:

- `glm::ivec3 m_movementDirection`
- `int m_rollInput`
- `glm::dvec3 m_viewDirection`
- `bool m_jetpackEnabled`
- lock intent: `DigibotLockState m_lockState` + `uint64_t m_lockTargetGridId`
  (0 = none). The controller's lock target is a `weak_ptr<RigidBody>` set from a
  client-side raycast; over the wire it must be an id, resolved on the receiving
  side via `GridSubsystem`.
- `serialize` / `deserialize`.

Docking/seating intents are deferred; first version covers free flight + walking.

**2b. `DigibotController::applyInput(const DigibotInput&)`** — one call that fans
out to the existing setters. Existing setters stay.

**2c. Funnel the player controller.** `DigibotPlayerController::stepControl()`
builds a `DigibotInput` from keyboard/mouse and calls `applyInput` instead of
calling setters directly — plus a getter so the layer above can read the input of
the current step. Local play, server-side remote play, and (later) replay recording
all flow through the same single entry point. This is the refactor to do carefully;
everything else is additive.

## Phase 3 — Authority flip + input channel (network side)

**3a. Message types.** The wire grows a leading `uint8` message type:
`StateSnapshot` and `PlayerInput`.

**3b. Ownership.** Replace the type-based branch with explicit roles, configured
from `Game::setupWorld()`:

- server: authoritative over *all* state; sends `StateSnapshot` (grids +
  characters) every tick, as today;
- client: sends `PlayerInput` only — it stops sending character state entirely;
- `GameNetwork::setLocalPlayer(characterId)` tells the layer which character is
  predicted (client) / which character belongs to which connection (server, needed
  to reject a client driving someone else's character).

**3c. Client send path.** Each completed tick: read the funneled `DigibotInput`,
stamp it with the client tick + character id, send unreliable. Cheap hardening for
packet loss: each packet carries the last 3 inputs; the server applies the newest
it has not yet seen.

**3d. Server receive path.** Store the newest input per remote character. At every
`stepApply()` (the `AwaitingStepControl` boundary), apply it via
`DigibotController::applyInput`. "On arrival" therefore means "at the next tick
boundary after arrival" — inputs never land mid-step. If no new input arrived, the
previous one stays applied (held input), which is the correct guess and also what
the design doc's remote-extrapolation logic will reuse later.

**3e. Anchor tick bookkeeping (compare like with like).** The client must compare
the server's echoed player state against *its own state at the corresponding
tick*. Under apply-on-arrival there is no exact correspondence, so the server
tracks per player character an `anchorTick`: reset to the input's client-tick
stamp when a newer input is first applied, incremented by one every physics step
after that. Each `StateSnapshot` includes it next to that character's state. Under
steady packet flow this simply equals the input stamps; under loss it keeps
advancing, so the comparison stays aligned in time and the input divergence shows
up as genuine error for the rubber band — which is the designed behavior.

**3f. Client apply path.** `applySnapshot` applies grids and remote characters
directly (as today), but routes the *own* character's state + `anchorTick` into
Phase 4 instead of writing the rigid body.

## Phase 4 — Prediction + rubber band (network side)

**`src/game/PlayerPrediction.{h,cpp}`** — a small class owned by `GameNetwork`,
kept separate so `GameNetwork` stays an orchestrator (and under the module size
limit). It speaks only `RigidBodyState` + ticks; no transport, no game types
beyond physics. Fully unit-testable.

- **Ring buffer**: `{tick, RigidBodyState}` for the local player, recorded at every
  completed tick. 256 entries ≈ 26 KB — trivial.
- **On anchor** `(anchorTick, serverState)`: look up own past state at
  `anchorTick`, form the error `serverState − ownState[anchorTick]` (position
  delta, velocity delta; orientation as a delta quaternion). Fold it into a
  persistent pending-correction accumulator (replacing, not summing, since each
  anchor supersedes the last).
- **Each step**: apply the fraction `1 − exp(−dt/τ)` of the pending correction to
  the body — position *and* velocity — and decay the remainder. τ ≈ 200 ms to
  start; must satisfy τ >> one-way latency.
- Guards: no anchor yet → do nothing; non-finite anchor → drop; anchorTick older
  than the buffer → drop (and log — it means latency exceeds the buffer, which
  should not happen at 256 ticks).

Deliberately deferred from the design doc's gotcha list: softening the correction
while resting in contact. First version always blends; if floor jitter shows up in
testing, gate the correction on `|error|` and contact state then.

## Phase 5 — Latency simulation + testing

- **`src/network/DelayedTransport.{h,cpp}`** — an `INetworkTransport` decorator
  that queues messages and releases them after a configurable delay ± jitter,
  with optional drop rate. Backend-agnostic and deterministic (seeded), unlike
  GNS's built-in fake-lag knobs. Wrap the real transport with it behind a flag.
- Run server + client locally, watch stdout (per project testing policy). Log the
  correction magnitude per anchor. Expected signature, straight from the design
  doc's error analysis: ~zero while coasting (any speed), spikes of roughly
  `a · L` in velocity and `a · L · τ` in position during burns, decaying with τ.
  If error grows with *speed* instead of *acceleration*, the anchor-tick
  bookkeeping (3e) is off — that is the like-with-like comparison failing.

---

## File touch list

New:
- `src/serialization/ByteStream.h`
- `src/physics/RigidBodyState.{h,cpp}`
- `src/characters/digibot/DigibotInput.h` (+ `.cpp` if serialize outgrows the header)
- `src/game/PlayerPrediction.{h,cpp}`
- `src/network/DelayedTransport.{h,cpp}`

Modified:
- `src/characters/digibot/DigibotController.{h,cpp}` — `applyInput`
- `src/game_base/DigibotPlayerController.{h,cpp}` — build + funnel `DigibotInput`
- `src/game/GameNetwork.{h,cpp}` — message types, ownership roles, input channel,
  anchor routing; sheds its byte codecs and `BodyState` to Phases 1a/1b
- `src/game/Game.cpp` — `setLocalPlayer`, transport wrapping for latency tests

Order of implementation = phase order; every phase leaves the game runnable
(Phases 1–2 change no behavior at all).

## Known gaps accepted for this iteration

- No structural replication: building/splitting during a session desyncs (as today).
- Remote characters on the client run `stepControl` with stale/absent input between
  snapshots; per-tick snapshots overwrite the result, so this only matters under
  heavy loss.
- Remote characters' view direction / animation state is not replicated (their
  rigid body pose is). Cosmetic; fix when input relay lands with Alpha.
- Server's own player takes the same funnel but skips prediction (no latency to
  hide).
