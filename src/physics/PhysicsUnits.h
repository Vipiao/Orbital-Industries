// PhysicsUnits.h
#pragma once

// The physics engine integrates in tick units: each step adds velocity to position
// and acceleration to velocity, with the timestep folded directly into the stored
// values (see PhysicsEngine::updatePositions). A stored velocity is therefore
// displacement per tick and a stored acceleration is velocity change per tick.
//
// Tuning is written in SI units (seconds) and converted to per-tick units at the
// point of definition, so the configured behaviour stays fixed in real-world terms
// even if the tick rate changes. Instead of a raw per-tick number, write e.g.
// PhysicsUnits::metersPerSecond(5.12). All conversions are constexpr and route
// through s_tickRateHz, the single source of truth for the tick length.
namespace PhysicsUnits {

// Physics simulation rate (ticks per second).
inline constexpr double s_tickRateHz{64.0};

// Seconds -> ticks. A real-time duration expressed as a number of physics steps.
inline constexpr double seconds(double timeSeconds) {
    return timeSeconds * s_tickRateHz;
}

// m/s -> m/tick. A RigidBody velocity (displacement added to position each tick).
inline constexpr double metersPerSecond(double velocity) {
    return velocity / s_tickRateHz;
}

// m/s^2 -> m/tick^2. An acceleration (velocity added to velocity each tick).
inline constexpr double metersPerSecondSquared(double acceleration) {
    return acceleration / (s_tickRateHz * s_tickRateHz);
}

// rad/s -> rad/tick. An angular velocity applied to orientation each tick.
inline constexpr double radiansPerSecond(double angularVelocity) {
    return angularVelocity / s_tickRateHz;
}

// rad/s^2 -> rad/tick^2. An angular acceleration.
inline constexpr double radiansPerSecondSquared(double angularAcceleration) {
    return angularAcceleration / (s_tickRateHz * s_tickRateHz);
}

// 1/s -> 1/tick. A first-order response/gain constant (e.g. a velocity-tracking
// stiffness) whose real-time behaviour must stay fixed as the tick rate changes.
inline constexpr double perSecond(double rate) {
    return rate / s_tickRateHz;
}

}
