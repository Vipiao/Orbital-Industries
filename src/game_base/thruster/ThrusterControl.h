// ThrusterControl.h
#pragma once

#include <glm/glm.hpp>

class CockpitBlock;
class Grid;
class PhysicsEngine;

// Ship thrust control: maps a seated pilot's movement intent onto a grid's
// thrusters, and turns stored thrust levels into forces on the grid's rigid body.
// Stateless tool — the throttle lives on each ThrusterBlock, so a commanded burn
// persists until the next command even with nobody in the seat.
class ThrusterControl {
public:
    // Overwrite every thruster's throttle from the pilot's input (x right,
    // y forward, z up in the cockpit frame): full thrust for thrusters pushing
    // along a commanded axis, zero for all others. Axes are not blended or
    // normalized — forward+right fires all forward and all right thrusters at
    // maximum. Zero input therefore cuts all thrust.
    static void setPilotCommand(Grid& grid, const CockpitBlock& cockpit,
                                const glm::ivec3& input);

    // Apply each thruster's stored throttle as a force at the thruster's position
    // (off-centre thrusters torque the ship). Runs for every grid every step,
    // whether or not anyone is seated.
    static void applyThrustForces(PhysicsEngine* physicsEngine, Grid& grid);
};
