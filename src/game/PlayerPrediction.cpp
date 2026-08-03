#include "PlayerPrediction.h"

#include "../physics/PhysicsEngine.h"
#include "../physics/PhysicsUnits.h"
#include "../physics/RigidBody.h"

#include <cmath>
#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>
#include <iostream>

namespace {

// Strong-band time constant, in ticks: how fast a large error eases onto the anchor
// the instant the band engages. See notes/multiplayer_design.md §5.
constexpr double s_strongTauTicks{PhysicsUnits::seconds(0.5)};

// Strong-band half-life, in ticks: while the band stays engaged the effective tau
// halves every this-many ticks, so a desync that persists is forced onto the anchor
// ever harder the longer it lasts.
constexpr double s_strongTauHalfLifeTicks{PhysicsUnits::seconds(0.5)};

// Weak-band time constant, in ticks: a very gentle continuous pull while moving, slow
// enough to be masked by the motion.
constexpr double s_weakTauTicks{PhysicsUnits::seconds(1.0)};

// Hold-off, in ticks: the weak pull stays on this long after the character was last
// active, so residual drift settles while it comes to rest.
constexpr double s_weakHoldTicks{PhysicsUnits::seconds(0.25)};

// Strong-band deadzone, in metres. Both must sit above the normal prediction lead
// (~ own input speed * round-trip latency) or ordinary movement would trip them.
constexpr double s_engageThreshold{1.0};
constexpr double s_releaseThreshold{1.0};

}  // namespace

void PlayerPrediction::correct(const std::weak_ptr<RigidBody>& bodyWeak,
                               const RigidBodyState& anchor, bool active,
                               PhysicsEngine& physicsEngine) {
    std::shared_ptr<RigidBody> body{bodyWeak.lock()};
    if (!body) {
        return;
    }

    // -1 means never active yet; otherwise the count saturates just past the
    // hold-off, beyond which its exact value no longer matters.
    if (active) {
        m_ticksSinceActive = 0;
    } else if (m_ticksSinceActive >= 0 && m_ticksSinceActive <= s_weakHoldTicks) {
        ++m_ticksSinceActive;
    }

    // Strong band with hysteresis: engages above engage, releases below release.
    double positionError{glm::length(anchor.m_position - body->getPosition())};
    if (m_correcting) {
        if (positionError < s_releaseThreshold) {
            m_correcting = false;
            m_correctingTicks = 0;
        } else {
            ++m_correctingTicks;
        }
    } else if (positionError > s_engageThreshold) {
        m_correcting = true;
        m_correctingTicks = 0;
    }

    if (++m_debugTickCounter % 10 == 0) {
        std::cout << "[predict] error=" << positionError << " correcting=" << m_correcting
                  << " correctingTicks=" << m_correctingTicks << " active=" << active
                  << std::endl;
    }

    // Strong band when engaged; otherwise the weak pull while active or within the
    // hold-off (motion masks it), never while standing still — a still character
    // would visibly slide.
    bool withinHold{m_ticksSinceActive >= 0 && m_ticksSinceActive <= s_weakHoldTicks};
    double tau{0.0};
    if (m_correcting) {
        // Halve the effective tau every half-life of continuous engagement, so the
        // pull ramps up exponentially and a stubborn desync is force-corrected.
        tau = s_strongTauTicks * std::exp2(-m_correctingTicks / s_strongTauHalfLifeTicks);
    } else if (withinHold) {
        tau = s_weakTauTicks;
    } else {
        return;
    }

    // Ease onto the anchor; it rides the same timeline as the applied world, so the
    // pull keeps the world mutually consistent (multiplayer_design.md §5).
    double blend{1.0 - std::exp(-1.0 / tau)};

    body->setPosition(body->getPosition() +
                      blend * (anchor.m_position - body->getPosition()));
    body->m_velocity += blend * (anchor.m_velocity - body->m_velocity);
    body->setOrientation(glm::normalize(glm::slerp(body->getOrientation(),
                                                   anchor.m_orientation, blend)));
    body->setAngularMomentumBody(
        body->getAngularMomentumBody() +
        blend * (anchor.m_angularMomentumBody - body->getAngularMomentumBody()));

    physicsEngine.updateColliderTransform(bodyWeak);
}
