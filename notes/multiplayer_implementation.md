# Multiplayer — As Implemented

The design that actually runs: Version Beta (multiplayer_design.md §5) plus the
extensions below. That doc holds the Alpha/Beta analysis and the reasoning for
server authority and "consistently in the past"; this one records what was built
and the logic that is not obvious from the code alone.

Code entry point: `src/game/GameNetwork.{h,cpp}`. Everything network-aware lives
there. The game side stays network-blind and only exposes general-purpose
functions the network layer composes: capture/apply (`RigidBodyState`),
serialize (`GridSerializer`, doubles as the save format), structural edits
(`StructuralCommand`), docking reconcile (`CockpitDockingCoordinator`).

## 1. Roles and channels

The server simulates the whole world and is the single authority. Authority
never moves; only who supplies a character's *input* does.

- **Unreliable, per-tick, latest-wins**: state snapshots (grid + character
  `RigidBodyState`s, each character's input and docking status) and client
  input. These are *levels*, not events — a lost packet is superseded by the
  next tick's, so nothing is retransmitted or replayed.
- **Reliable, ordered**: structural edits, full-grid corrections (GridData),
  control arbitration. One receive queue holds commands and corrections in
  exact arrival order: a correction is only consistent with the commands sent
  around it (ModifyCell deltas compose, so applying one twice — or around a
  resend in the wrong order — corrupts the grid).

Clients predict exactly one thing: their own character (piloted grids will join
that set later). Everything else applies server state directly.

## 2. Structural edits

Clients never mutate structure directly; tools emit `StructuralCommand`s that go
to the server as requests. The server resolves them
(`GameBase::resolveStructuralEdit`: id allocation, split seeding,
despawn-on-empty) and broadcasts the decided stream — including back to the
requesting client, which replays its own edit (`GameBase::applyStructural`) only
when it returns.

## 3. World reconciliation (manifests)

Repairing a wrong world is one mechanism, constantly exercised — late join is
just the degenerate case "client holds nothing".

- Every 5 s a client sends `{gridId, structureHash}` for each grid it holds;
  the first manifest doubles as the late-join request.
- The server diffs against its world per client: mismatched or missing grids get
  a full `GridData` resend, leftover client grids get despawn commands — all on
  the reliable channel, interleaved with ordinary edits, for the ordering
  guarantee above.
- The hash covers **structure only** (sorted cell coords, vertices, colors,
  thrusters, cockpits — never pose, motion or throttle), so peers agree
  regardless of edit history or container iteration order.
- A mismatch on a grid edited within the last 3 s is presumed to be the echo
  still in flight and skipped; the next manifest re-checks it.

## 4. Character control arbitration

- A mode *desires* control; the network layer arbitrates. The client requests a
  preferred character (nearest the camera — a UI intent, so computed
  client-side); the server grants it only if free, else denies. Never a silent
  substitute. Denial latches until the desire drops.
- The server keeps an owner per character and accepts input packets **only from
  the owner** — one rule covering spoofing and stale packets from a previous
  owner alike. The server player goes through the same arbiter locally.
- Inputs are *claims*, held until superseded. Release or disconnect ends the
  claim and neutralizes the held movement exactly once.

## 5. Docking

Seating/unseating syncs as **level state**, not commands: the controlling peer's
docking status rides its input packets and the server's snapshots, and every
receiver force-reconciles its copy each tick (`forceDockingStatus`, idempotent —
missing grid or cockpit just retries next tick). Locally, the controller only
*records* transition desires; `CockpitDockingCoordinator` applies them after
control, so one module owns the state machine whether the desire came from local
input or the wire.

## 6. Timeline smoothing (tick-drift filter)

The peers' tick clocks are never phase-locked, so a client receives 0, 1 or 2
snapshots per own tick. Gaps are already smooth (local integration bridges
them), but a burst advances a body's timeline two ticks in one frame — at
platform speed `v` that is a `v·dt` lurch, and the prediction anchor inherits
it, kicking the rubber band. Fix: place every received state on a smoothed
local timeline instead of applying it raw. All in tick units (the engine's
native time unit — stored velocities are per-tick).

    initConnectionTickDiff = firstServerTick − firstClientTick     (fixed at join)
    tickDrift    = initConnectionTickDiff − (serverTick − clientTick)   at apply
    emaTickDrift = mix(emaTickDrift, tickDrift, 0.01)
    shift        = tickDrift − emaTickDrift                (clamped to ±2 ticks)

`tickDrift` starts at zero by construction. The EMA is a low-pass tracking the
slow, real component (clock drift, latency shifts); the residual `shift` is
delivery jitter. Every state in the snapshot — grid bodies, remote characters,
the prediction anchor — is advanced by `shift` kinematically (`p += v·shift`,
orientation rotated by `ω·shift` via `RigidBody::integrateOrientation`, the
integrator's own step; sub-tick spans, so ignoring forces is fine — see
`shiftedInTime` in GameNetwork.cpp). In steady conditions
`shift ≈ 0` and this is a no-op. Raw arrival timing never moves a body — only
the tick stamps do, through the filter.

Two consequences, each with its own compensation:

- **The EMA moving is the world timeline moving.** Each update shifts every
  snapshot-driven body by its velocity × ΔEMA. The one object not on that
  timeline is the locally predicted character, so it gets extrapolated by the
  same −ΔEMA along its own motion: riding a grid its velocity matches the walls
  and it stays put relative to them; on static ground it is a no-op.
- **The clamp is a desync detector, not a limiter.** Exceeding ±2 ticks means a
  freeze on one side, not jitter; easing the EMA across such a gap would pin
  the clamp for its whole convergence. Instead: re-base (`emaTickDrift =
  tickDrift`) in one step, and snap the predicted body onto its fresh anchor —
  the one state guaranteed consistent with the re-based world. Prediction
  restarts from there. The player sees one jump; after a real freeze that is
  unavoidable.

## 7. Prediction and reconciliation

`PlayerPrediction` (pure reconciliation, no transport): the predicted character
integrates immediately; the server state arrives as an anchor, time-aligned by
§6 and owned by GameNetwork, which also advances it one tick per step — the
world's bodies integrate on every step, and a frozen anchor would miss the
timeline by `speed × staleness` (meters, at speed) and fight the controller.
Inside the deadzone a weak pull bleeds drift while moving;
a large error engages the strong band with hysteresis. See
multiplayer_design.md §5 for why anchor-at-server-time keeps the whole world
mutually consistent.

## 8. Not built yet (and where it hooks in)

- **Piloted-grid prediction**: add the seated grid to the predicted set — same
  anchor reconcile, same −ΔEMA nudge.
- **Dynamic characters**: a character section in the manifest plus a
  CharacterData correction, mirroring grids. (Today both peers build the same
  hardcoded world in `main.cpp`.)
- **Interest management / unload**: the manifest diff is already per-client;
  "what a client should hold" just stops being "everything".
- **Structure-hash caching**: dirty flag on Grid, invalidated where structural
  edits already schedule analysis; hashing is microseconds today.
- **Alpha upgrade** (client-side lead/extrapolation): multiplayer_design.md §4.
