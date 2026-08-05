// ReactionWheelControl.h
#pragma once

#include <glm/glm.hpp>

class CockpitBlock;
class Grid;
class PhysicsEngine;

// Ship attitude control: maps a seated pilot's rotation intent onto a grid's
// reaction wheels, and drives the wheels themselves every step.
// Stateless tool. Unlike a thruster's throttle, a wheel's command is recomputed
// from scratch each step, so an empty seat is simply an unasked-for rotation:
// the wheels stop turning the ship and bring it to rest rather than holding the
// pilot's last command.
class ReactionWheelControl {
public:
    // Rate at which the wheels bleed off the grid's rotation whenever it is not
    // the rotation being asked for.
    static constexpr double BRAKING_RATE_PER_SECOND{2.0};

    // Record what the pilot is asking of the ship (an angular acceleration in the
    // cockpit frame, rad/s^2). Consumed by stepControl in the same step.
    static void setPilotCommand(Grid& grid, const CockpitBlock& cockpit,
                                const glm::dvec3& angularAcceleration);

    // Turn the pilot's request and the grid's current rotation into torque on its
    // wheels. Runs for every grid every step, whether or not anyone is seated.
    // The torque is what this grid's own inertia needs for the requested change,
    // shared equally between its wheels: a given input turns any ship the same
    // way, while more wheels only raise the ceiling before they saturate.
    static void stepControl(PhysicsEngine* physicsEngine, Grid& grid);
};
