// ThrusterControl.cpp
#include "ThrusterControl.h"
#include "ThrusterBlock.h"
#include "../Grid.h"
#include "../cockpit/CockpitBlock.h"
#include "../../physics/PhysicsEngine.h"
#include "../../physics/PhysicsUnits.h"
#include <algorithm>
#include <glm/gtc/quaternion.hpp>
#include <tuple>
#include <vector>

void ThrusterControl::setPilotCommand(Grid& grid, const CockpitBlock& cockpit,
                                      const glm::ivec3& input) {
    glm::dquat cockpitInverse{glm::conjugate(cockpit.m_orientation)};
    for (const auto& [coord, thruster] : grid.getThrusterCells()) {
        // Thruster force direction in the cockpit frame, rounded back to the
        // exact integer axis (quaternion rotation leaves float noise).
        glm::dvec3 forceDir{ThrusterBlock::thrustForceDirection(thruster.m_orientation)};
        glm::ivec3 axis{glm::round(cockpitInverse * forceDir)};

        // axis is a unit axis vector, so the dot with the input is the input
        // component along it: fire exactly when the pilot commands that direction.
        bool fire{glm::dot(glm::dvec3{axis}, glm::dvec3{input}) > 0.0};
        grid.setThrusterLevel(coord, fire ? 1.0 : 0.0);
    }
}

void ThrusterControl::applyThrustForces(PhysicsEngine* physicsEngine, Grid& grid) {
    const auto& thrusters = grid.getThrusterCells();
    if (thrusters.empty()) {
        return;
    }
    std::weak_ptr<RigidBody> body{grid.getRigidBody()};

    // Coordinate-sorted iteration so the force accumulation order (and thus the
    // floating-point sum) is reproducible.
    std::vector<glm::ivec3> coords{};
    coords.reserve(thrusters.size());
    for (const auto& [coord, thruster] : thrusters) {
        coords.push_back(coord);
    }
    std::sort(coords.begin(), coords.end(),
              [](const glm::ivec3& a, const glm::ivec3& b) {
                  return std::tie(a.x, a.y, a.z) < std::tie(b.x, b.y, b.z);
              });

    for (const glm::ivec3& coord : coords) {
        const ThrusterBlock& thruster{thrusters.at(coord)};
        if (thruster.m_thrustLevel <= 0.0) {
            continue;
        }
        glm::dvec3 forceDir{ThrusterBlock::thrustForceDirection(thruster.m_orientation)};
        double magnitude{thruster.m_thrustLevel *
                         PhysicsUnits::newtons(ThrusterBlock::THRUST_FORCE_NEWTONS)};

        // Body-local frame is the grid lattice. The force acts at the centre of
        // the 1x2x1 thruster unit; any point on the thrust axis gives the same
        // torque.
        glm::dvec3 axis{ThrusterBlock::dominantAxis(thruster.m_orientation)};
        glm::dvec3 localPoint{glm::dvec3{coord} + ThrusterBlock::MODEL_CENTRE +
                              axis * 0.5};
        physicsEngine->applyLocalForceAtPoint(body, forceDir * magnitude, localPoint);
    }
}
