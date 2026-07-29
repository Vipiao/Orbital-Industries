# Multiplayer Design Notes

Status: design only, nothing here is implemented yet beyond what is described in
"Where the code is today". Two candidate netcode models are specified: **Alpha**
and **Beta**. Beta is the intended first implementation; Alpha is the target it
upgrades into.

---

## 1. Where the code is today

- `src/network/INetworkTransport.h` — backend-agnostic transport interface
  (connect/listen, poll, send reliable/unreliable, drain events + messages).
  `GnsTransport` implements it over Valve GameNetworkingSockets.
- `src/game/GameNetwork.{h,cpp}` — the current sync layer. Every tick it
  serializes rigid-body state (position, orientation, velocity, angular momentum
  in body space) for all grids and all characters into one unreliable snapshot,
  stamped with the sender's physics tick. Receivers drop stale/reordered
  snapshots and apply the rest.
- `src/game/Game.cpp` — the frame is a `receive -> simulate -> send` pipeline.
  `GameBase::advanceFrame()` yields at `AwaitingStepControl`, a clean pre-integration
  boundary, and network state is applied exactly there. This yield point is
  valuable and both designs below depend on it.
- Ownership today is a placeholder: server owns all grids, client owns all
  characters, decided by branching on `m_transport->isServer()`. Both peers build
  an identical world at startup so object ids line up. There is **no** create/destroy
  replication.

### Known holes in the current code

1. **Ownership is hardcoded by type.** `buildSnapshot`/`applySnapshot` know what a
   `Grid` and a `Character` are, and branch on `isServer()`. This does not scale and
   is the reason game-specific code keeps landing in `GameNetwork`.
2. **No id/lifecycle replication.** The moment a grid splits (`GridSplitter`), the
   server invents new grid ids that the client never creates. Id spaces silently
   diverge and every subsequent snapshot is applied to the wrong object or none.
   **This is the first thing that must be fixed.** It is a bigger problem than the
   wire format.
3. **Received state is written straight into the rigid body**, fighting the local
   integrator. Both designs below replace this.

---

## 2. Foundations both Alpha and Beta assume

These are not netcode models; they are prerequisites shared by both.

### 2.1 Replication registry (keeps game types out of `GameNetwork`)

`GameNetwork` should not know what a `Grid` or a `Character` is. It should walk a
flat registry of replicated objects, each exposing:

- a stable **id**
- an **authority** (which peer owns it) — a field on the object, *not* a
  `isServer()` branch in the network code
- a **type tag** (so spawn can be driven generically through a factory)
- **serialize / deserialize** entry points

Adding a new replicated type should touch zero lines of network code.

### 2.2 Serialization channels — state vs. events, not "arbitrary subsets"

Resist a general "serialize any subset of any object" mechanism. The axis that
matters is **state vs. events**, because they need different transport and
different correctness rules:

| Channel | Contents | Transport | Semantics |
|---|---|---|---|
| **Dynamic state** | position, orientation, velocity, angular momentum | unreliable, every tick | latest-value-wins; a lost packet is repaired by the next |
| **Structural commands** | add/remove/modify/color cell, split, merge, spawn, despawn | reliable, **ordered**, single stream | exactly-once, in-order, validated |
| **Full state** | everything | reliable | save games, and a late-joiner's initial world |

Composition is hierarchical for the full channel: `Grid` serializes its own fields
and asks each block to serialize itself; `GridSubsystem` serializes its grids. The
full channel doubles as **save/load**. The dynamic-state channel doubles as the
**correction/rollback snapshot** — design it so it can *restore* a body, not just
transmit one.

Do **not** derive structural changes as a dirty-diff of the block map. They are
already discrete user actions (`addCell`, `removeCell`, `modifyCell`, `setColor`);
send them as commands. A diff gives the same bytes with worse ordering semantics and
nowhere to reject an illegal placement.

All structural commands must share **one** ordered stream. A "remove cell in grid Y"
must never overtake the "spawn grid Y" that a split produced.

### 2.3 Server authority over structure

Grid splitting, block building, and block destruction are all **server-authoritative**.
The client never mutates a grid; it *requests*, the server validates and broadcasts a
command, and everyone (including the requester) applies the command when it arrives.

- The client **never re-derives a split.** Its grid state is a lagged copy, so it
  cannot be trusted to reach the same split decision on the same tick. The server runs
  `GridSplitter`, allocates the new id, and sends a reliable command.
- Block ops are expressed as `(gridId, ivec3 coord, action)` in **grid-local
  coordinates**, not world space. This is a piece of luck worth understanding: the
  client raycasts against a grid whose *transform* is stale, but the cell it hits in
  grid space is the same one the server would hit, because a grid's *contents* barely
  change over one RTT even though its *transform* moves a lot. Latency on grid motion
  is therefore nearly harmless for block ops.
- The server must still validate: requester has a character within `interactionRange`,
  the cell is occupied (removal) or free-and-legal (placement, via `canModifyCell` and
  the anchor rules), plus rate limiting. The client's raycast is a *convenience for
  picking*, not evidence.
- Conflicts resolve for free: the server serializes competing requests, first wins,
  second is rejected. The client must handle rejection.

**Do not predict structural changes into the simulation.** `removeCell` rebuilds the
collider, remeshes graphics, changes mass and inertia, kicks off structural analysis,
and *can trigger a split*. A client that optimistically removes a block and cascades
into a local split would spawn a grid with a locally-invented id — exactly the
divergence we are trying to eliminate, now reachable from an ordinary player action.

If instant feedback is wanted, predict **cosmetically only**: fade the block out in
graphics, touch no collider, no mass, no structural analysis, no split; revert on
rejection. Start with no prediction at all — 80ms of latency on a deliberate,
discrete action is genuinely hard to notice. Movement is continuous and demands
prediction; construction does not.

Destruction from collisions (later feature) is pure server-side physics emitting the
same structural commands, through the same client apply path. The client never
distinguishes "a block broke because someone hit it with a tool" from "a block broke
because a grid slammed into a wall". That is the property we want.

---

## 3. The core problem both models solve

The server is authoritative. The client wants its own character to feel instant. So
the client simulates its own character locally, ahead of hearing back from the server.
When the server's version arrives, the two must be reconciled without the player
seeing a rubber-band.

Two independent axes, and they are easy to conflate:

**Axis 1 — when does the server execute a client's input?**
- *At the tick the client stamped it for.* Requires the client's clock to run **ahead**
  of the server (a "lead") so the input packet physically arrives before the server
  reaches that tick. Client and server then run the same input on the same tick, so
  their trajectories are **identical** and disagreement is rare (only real surprises:
  you got shoved, you clipped something you could not see).
- *On arrival.* The server runs the client's exact trajectory **delayed by L** (L =
  one-way latency in ticks). Disagreement is then *systematic*, not rare.

**Axis 2 — how is a disagreement corrected?**
- *Rollback + replay.* Restore your character to the server's state at tick T, then
  re-simulate T+1..now from a buffered input history. Exact. Requires deterministic,
  re-runnable physics.
- *Exponential blend (rubber band).* Ease position **and** velocity toward the
  corrected state over a time constant τ. Approximate, far less machinery, degrades
  gracefully (rubberiness rather than a visible rewind).

Alpha and Beta pick different points on these axes.

---

## 4. Version Alpha — "consistently in the present"

**Everything the client displays lives at the client's own (leading) tick.**

- **Clock:** client runs a lead of roughly one-way latency + jitter margin. Inputs are
  stamped with the client's tick. The server buffers them a couple of ticks and
  executes each **at its stamped tick**.
- **Clock sync:** do *not* set the client clock from the last received server
  timestamp — that timestamp is already one-way-latency stale and would leave the
  client running *behind*. Instead close a loop: the server reports, per client, how
  early/late each input arrived relative to the tick it was stamped for; the client
  nudges its lead to keep that in a small positive window (say 1–3 ticks). This is
  self-correcting, absorbs jitter and clock drift, and never needs an accurate RTT
  estimate.
- **Own character:** fully simulated locally with the same physics code the server
  runs. Collisions included — the character's motion is contact-dominated and cannot
  be predicted without predicting its contacts.
- **Grids:** **extrapolated**, not simulated. A grid is a heavy rigid body with smooth
  continuous motion, so advancing its transform a few ticks from the last snapshot
  (position by velocity, orientation by angular momentum) is highly accurate. Cheap.
  The character collides against these extrapolated poses, and they are *rendered* at
  the same tick, so contact and visuals agree.
- **Remote characters:** extrapolated too, but approximately. Extrapolate them **with
  their last known input held constant** (the server relays inputs, not just state) —
  "still holding forward" is the common case and running their *intent* is far better
  than blindly projecting a velocity vector. Extrapolate velocity, **not**
  acceleration (projecting acceleration amplifies error on direction changes rather
  than damping it). Blend corrections in over ~100–200ms. Clamp how far you will
  extrapolate — if a client's packets stop, freeze rather than fly off.
- **Correction:** rollback + replay (exact), or the Beta-style blend (see §6).

### Why not "render remote objects in the past" (standard interpolation delay)?

Standard FPS netcode renders remote objects at `serverTime - interpDelay` and
interpolates between two received snapshots. **That is only free when there is a
shared static reference frame** — in a normal shooter the map does not move, so a
remote player drawn 100ms behind is 100ms behind along their own path and you have
nothing to compare it against.

**This game has no static frame.** Everything is in orbit; grids translate and spin;
characters fly. A 100ms world-space delay becomes visible displacement against *any*
moving thing you look at it against. Fly alongside a ship doing 100 m/s and a remote
player genuinely next to you is drawn 10 m behind. Attached or free-flying, it does
not matter.

Hence: extrapolate forward, do not interpolate backward. This is a game-specific
conclusion that contradicts standard netcode wisdom, and it is correct *here* because
the assumption that wisdom rests on (a static map) does not hold.

### Grid-local representation (Alpha refinement, deferred)

For a character locked to a deck, replicate its state **relative to the grid** rather
than in world space. Not as a latency fix — as a *precision and stability* measure:

- Extrapolation error expressed relative to the deck cannot punch the bot *through the
  floor* of a fast-moving ship; it perturbs it along the deck, which is benign.
- The world is double-precision because it is large. A character's world position on a
  distant orbiting grid is a big number; its grid-relative position is a small one.
  Doing interpolation and error-blending in the small numbers is simply more accurate.

`DigibotPlayerController` already has surface-lock raycasting and
`getSurfaceAngularVelocity()`, so "which grid am I standing on" already exists.

**Deferred for the first version.**

---

## 5. Version Beta — "consistently in the past" (implement this first)

**The client's entire world lives at server time, uniformly delayed by L. The local
player is allowed to run ahead for responsiveness, and is continuously pulled back.**

- **Clock:** no lead. Client and server clocks are nominally aligned.
- **Server:** simulates everything. Receives client inputs and **applies them
  instantly on arrival**.
- **Client, everything except own player:** applies received server state directly.
  No extrapolation. Grids, remote characters — all sit at server time.
- **Client, own player:** simulated locally and immediately, so input feels responsive.
  When the server's authoritative player state arrives, **do not** write it into the
  rigid body. Store it as an **anchor**, and exponentially blend the local player's
  **position and velocity** toward it over a time constant τ.
- **No grid-local frames, no forward extrapolation.** Deliberately simplified.

### Why Beta is self-consistent (the key insight)

Naively, the server's player is behind the client's player by `v·L` — the server runs
the client's exact trajectory delayed by L. It is tempting to call this "the error".

**It is not an error.** In Beta the grid is delayed by L *too* — the client's whole
world sits at server time. A uniform time-shift of an entire world is **unobservable
from inside it**. If the player and the grid are both `v·L` behind where they "truly"
are, the *relative* geometry is exactly right: you are next to the grid on the client,
and you are next to the grid on the server.

The correct error decomposition is:

| Quantity | Magnitude | Observable? |
|---|---|---|
| World-space position offset | `v · L` | **No** — global, uniform, applies to everything equally |
| Velocity error while accelerating | `a · L` | **Yes** — this is the real one |
| Position transient during a maneuver | `≈ a · L · τ` | Yes — what the velocity error accumulates into |

Crucially the error is **zero at constant velocity** (`a = 0`). Coasting alongside a
ship — the case we care most about — has *no* error at all, not merely a small one.

Numbers: thrusting at 20 m/s², L = 50ms, τ = 200ms → velocity error 1 m/s, transient
offset ~20 cm. At 100 m/s², about a metre, transiently, during hard burns. Acceptable.

### What Beta actually costs

In steady state the rubber band pulls the displayed player onto the server's version,
and the server's version is the player's own trajectory delayed by L. So **the rendered
self is the commanded self, delayed by L** — smoothed, but delayed. Beta's price is
**input latency ≈ L**, spread out rather than abrupt.

This is exactly and only what the client lead (Alpha) buys back. Beta therefore
degrades with ping: fine at 50ms, sluggish at 150ms transatlantic, because the latency
paid *is* the ping. Alpha's input latency stays at zero as ping grows; what grows
instead is extrapolation error on other players.

Beta requires `τ >> L` for the player to still feel responsive — a stiff band (τ ≈ L)
would feel like mud. τ ≈ 200ms against L ≈ 50ms is a good starting point.

### Beta implementation gotchas

**Compare like with like.** The anchor describes tick T, but the local player is at tick
T + L. Differencing them directly is meaningless — the difference contains a whole
latency's worth of legitimate motion. Keep a small ring buffer of **your own past
states** (13 doubles × ~128 ticks is nothing) and compute the error against your own
state *at tick T*. This is not a rollback buffer; nothing is replayed. It is just a
history so the subtraction happens at the right time.

**The rubber band is an external influence on a physics body, and the player spends
most of its time in contact.** Correction fighting a contact constraint is where this
gets ugly — the band pulls into the floor, the solver pushes out, jitter or creep
results. Two mitigations:

- Apply the correction as a **velocity bias rather than a position teleport**, so it
  goes *through* the solver and contacts can veto it, rather than around it.
- **Soften or freeze the correction while resting in contact.** This falls out of the
  analysis above: a body at rest on a deck has `a ≈ 0`, so by the error decomposition
  there is nothing to correct — the band is only *needed* when accelerating, and only
  *dangerous* when not.

---

## 6. Alpha vs. Beta, and the upgrade path

|  | **Alpha** | **Beta** |
|---|---|---|
| Philosophy | consistently in the **present** | consistently in the **past** |
| Client clock | leads the server by ~L + margin | aligned with server |
| Server executes input | at the **stamped tick** | **on arrival** |
| Remote objects | extrapolated forward to client tick | applied directly, at server time |
| Own player | predicted exactly; disagreement is rare | predicted, continuously pulled back |
| Input latency | **0**, independent of ping | **≈ L** (smoothed) |
| Main error source | extrapolation error on remote objects | `a·L` velocity error while accelerating |
| Machinery | lead + clock loop + extrapolation + reconciliation | rubber band + small state history |

**The two axes are independent, and this is the thing to remember:** the *correction
mechanism* (rollback vs. blend) and the *input execution timing* (stamped tick vs.
arrival) are separate choices. Beta's exponential blend is a perfectly good correction
mechanism — arguably the better one to start with, since it needs no re-runnable
physics and degrades gracefully. Beta's input latency comes from the *other* axis.

**So the upgrade from Beta to Alpha is additive, not a rewrite:**

1. Add the client lead + the early/late feedback loop. The server now executes inputs
   at their stamped tick, so the client and server run identical trajectories. The
   rubber band you already built simply stops having anything to correct, and input
   latency drops to zero.
2. Switch remote objects from "apply directly" to "extrapolate forward to the client's
   tick" (grids ballistically; remote characters on their last-known input).
3. Optionally, later: swap the blend for exact rollback + replay, reusing the *same*
   ring buffer — replace "blend the error in" with "rewind and replay". Nothing is
   thrown away.
4. Optionally, later: grid-local representation for surface-locked characters.

Nothing built for Beta is wasted.

---

## 7. Recommended build order

1. **Server-allocated ids + spawn/despawn replication.** Nothing else works until grid
   splitting stops desyncing the world. This is the current code's biggest hole.
2. **Replication registry** (id, authority, type tag, serialize/deserialize). This is
   what stops game types from leaking into `GameNetwork`.
3. **Serialization on objects, composed hierarchically**, with the three channels from
   §2.2. Doubles as save/load; the dynamic-state channel doubles as the correction
   snapshot.
4. **Server authority over structure** (§2.3): client requests, server validates and
   broadcasts commands. No structural prediction.
5. **Beta** (§5): inputs to the server, rubber-band correction on the local player.
6. **Alpha** (§6): add the lead, then forward extrapolation.

Steps 1–3 are wanted for save games regardless of multiplayer, so they are worth
getting right independently.
