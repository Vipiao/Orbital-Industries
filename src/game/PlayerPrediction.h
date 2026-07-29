// PlayerPrediction.h — reconciles the locally simulated player against the
// server's authoritative state. When this client controls the character it
// integrates it immediately for responsiveness; the server's version arrives late.
//
// Correction comes in two strengths. Inside the deadzone a very weak pull bleeds off
// drift continuously whenever the body is in motion — flying, driven by movement
// input, or drifting free of any surface — where the motion masks it, plus a short
// hold-off after (so it can settle residual drift as the character comes to rest).
// Only a character standing on a surface with no input gets no pull, so it never
// visibly slides. Only a large error — a genuine desync — engages the strong band,
// which eases the body onto the anchor until the error falls back under a small
// release threshold (hysteresis, so it does not chatter at the boundary).
//
// The anchor is owned by the caller (GameNetworkClient), which time-aligns it with the
// rest of the applied world and advances it along the world's timeline between
// snapshots (notes/multiplayer_implementation.md §6-7). This class is only the
// correction policy: it compares and pulls, nothing else.
#pragma once

#include "../game_base/RigidBodyState.h"

#include <memory>

class PhysicsEngine;
class RigidBody;

class PlayerPrediction {
public:
    // Reconcile the body toward the anchor. Call once per step. active enables the
    // weak continuous pull, with a short hold-off after it drops; pass false only
    // while the character stands on a surface with no movement input. A no-op
    // while inactive, past the hold-off and inside the deadzone.
    void correct(const std::weak_ptr<RigidBody>& bodyWeak, const RigidBodyState& anchor,
                 bool active, PhysicsEngine& physicsEngine);

    // Forget the correction state, e.g. when this peer stops controlling the
    // character, so a later takeover does not resume a stale strong-band ease.
    void reset() {
        m_correcting = false;
        m_correctingTicks = 0;
        m_ticksSinceActive = -1;
    }

private:
    // True while easing toward the anchor after the error crossed the engage
    // threshold; cleared once it falls back under release (hysteresis).
    bool m_correcting{false};

    // Ticks the strong band has been continuously engaged; drives the exponential
    // ramp of the correction strength. Zero whenever the band is not engaged.
    int m_correctingTicks{0};

    // Ticks since the character was last active; keeps the weak pull alive through
    // the hold-off. -1 until first active, and saturates just past the hold-off
    // rather than counting forever.
    int m_ticksSinceActive{-1};

    // Throttles the tuning printout to a readable rate.
    int m_debugTickCounter{0};
};
