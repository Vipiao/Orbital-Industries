// InputSettings.h
#pragma once

namespace InputSettings {
    // Radians of camera rotation per pixel of (raw) mouse movement.
    static constexpr double MOUSE_SENSITIVITY = 0.0004;

    // Piloting a ship from a cockpit seat. The mouse figure is the angular
    // velocity (rad/s) the ship gains per pixel of mouse movement, so a sweep of
    // a given length turns the ship by the same amount however fast it was swept.
    static constexpr double PILOT_MOUSE_SENSITIVITY = 0.0015;

    // Angular acceleration (rad/s^2) commanded while a roll key is held. The
    // wheels brake against the ship's own rotation at
    // ReactionWheelControl::BRAKING_RATE_PER_SECOND, so a held key settles at a
    // roll rate of this figure divided by that one: 2 rad/s, reached with a half
    // second time constant.
    static constexpr double PILOT_ROLL_ACCELERATION = 4.0;
}
