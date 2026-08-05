// ReactionWheelControl.cpp
#include "ReactionWheelControl.h"
#include "ReactionWheelBlock.h"
#include "../CoordOrder.h"
#include "../Grid.h"
#include "../cockpit/CockpitBlock.h"
#include "../../physics/PhysicsEngine.h"
#include "../../physics/PhysicsUnits.h"
#include <glm/gtc/quaternion.hpp>

void ReactionWheelControl::setPilotCommand(Grid& grid, const CockpitBlock& cockpit,
                                           const glm::dvec3& angularAcceleration) {
    // Pilot intent is stated in the cockpit's frame; the grid stores it in its own.
    grid.setPilotAngularAcceleration(cockpit.m_orientation * angularAcceleration);
}

void ReactionWheelControl::stepControl(PhysicsEngine* physicsEngine, Grid& grid) {
    // Taken even with no wheels to fit, so a request cannot outlive the step it
    // was made in.
    const glm::dvec3 requested{grid.takePilotAngularAcceleration()};

    const auto& wheels{grid.getReactionWheelCells()};
    if (wheels.empty()) {
        return;
    }
    std::shared_ptr<RigidBody> body{grid.getRigidBody().lock()};
    if (!body) {
        return;
    }

    // Tuning is in SI; the rigid body integrates in ticks.
    constexpr double k_perTick{PhysicsUnits::radiansPerSecondSquared(1.0)};

    // Braking opposes whatever the ship is already doing, so the controls come
    // back to rest rather than to a fixed rate, and an abandoned ship settles.
    const glm::dvec3 braking{-PhysicsUnits::perSecond(BRAKING_RATE_PER_SECOND) *
                             body->getAngularVelocityBody()};

    // Torque this ship's inertia needs for that angular acceleration, split
    // evenly, expressed as a fraction of one wheel's maximum. Clamping by
    // magnitude keeps the commanded direction intact when the wheels saturate.
    const glm::dvec3 perWheel{body->getInertiaTensor() * (requested * k_perTick + braking) /
                              static_cast<double>(wheels.size())};
    glm::dvec3 command{perWheel /
                       PhysicsUnits::newtonMeters(ReactionWheelBlock::TORQUE_NEWTON_METERS)};
    const double magnitude{glm::length(command)};
    if (magnitude > 1.0) {
        command /= magnitude;
    }

    // Every wheel carries the same command, so the torques below are a sum of
    // identical terms and iteration order cannot affect it. Sorted anyway, so
    // that stays true if the wheels ever differ.
    std::weak_ptr<RigidBody> bodyWeak{grid.getRigidBody()};
    for (const glm::ivec3& coord : sortedCoords(wheels)) {
        grid.setReactionWheelCommand(coord, command);
        if (command == glm::dvec3{0.0}) {
            continue;
        }
        // Body-local frame is the grid lattice; a torque has no point of action.
        physicsEngine->applyLocalTorque(
            bodyWeak,
            command * PhysicsUnits::newtonMeters(ReactionWheelBlock::TORQUE_NEWTON_METERS));
    }
}
