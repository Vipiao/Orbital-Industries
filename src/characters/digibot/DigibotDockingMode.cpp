// DigibotDockingMode.cpp
#include "DigibotDockingMode.h"
#include "../ArticulationUtils.h"
#include <glm/gtx/norm.hpp>
#include "../MotionServo.h"
#include "../../physics/RigidBody.h"
#include "../../physics/RotatingFrameUtils.h"
#include <cassert>

DigibotDockingMode::SeatFrame DigibotDockingMode::seatFrameWorld(
    const RigidBody& gridBody, const Target& target) {
    SeatFrame frame{};
    frame.m_position = gridBody.getPosition() +
                       gridBody.getOrientation() * target.m_seatPositionLocal;
    frame.m_orientation = gridBody.getOrientation() * target.m_seatOrientationLocal;
    frame.m_forward = frame.m_orientation * glm::dvec3{0.0, 1.0, 0.0};
    frame.m_up = frame.m_orientation * glm::dvec3{0.0, 0.0, 1.0};
    return frame;
}

DigibotDockingMode::Result DigibotDockingMode::updateDocked(
    const std::shared_ptr<RigidBody>& rigidBody,
    const std::shared_ptr<RigidBody>& gridBody,
    const Target& target, const DigibotModeInputs& inputs) {
    Result result{};
    if (!rigidBody || !gridBody || rigidBody->getMass() <= 0.0) {
        result.m_wantRelease = true;
        return result;
    }

    // Producer (makeTarget) must fill every distance; -1 means a field was left unset.
    assert(target.m_seatArriveDistance > 0.0 && target.m_exitBodyDistance > 0.0 &&
           "docking target not fully configured");

    SeatFrame seat{seatFrameWorld(*gridBody, target)};

    // Corridor endpoints in world space: entry (door) -> seat.
    glm::dvec3 entryPoint{gridBody->getPosition() +
                          gridBody->getOrientation() * target.m_entryPositionLocal};
    glm::dvec3 seatPoint{seat.m_position};
    glm::dvec3 axisVec{seatPoint - entryPoint};
    double axisLength{glm::length(axisVec)};
    // A degenerate corridor means the CockpitBlock has entry == seat (bad config).
    // Release builds fall back to the seat forward; debug builds catch it here.
    assert(axisLength > 1e-6 &&
           "docking corridor degenerate: entry and seat points coincide");
    glm::dvec3 axis{axisLength > 1e-9 ? axisVec / axisLength : seat.m_forward};

    // Project the body onto the entry->seat line.
    glm::dvec3 fromEntry{rigidBody->getPosition() - entryPoint};
    double along{glm::dot(fromEntry, axis)};             // 0 at entry, axisLength at seat
    glm::dvec3 nearestOnLine{entryPoint + along * axis};
    glm::dvec3 tangentialOffset{rigidBody->getPosition() - nearestOnLine};
    double alongClamped{glm::clamp(along, 0.0, axisLength)};
    double projToSeat{axisLength - alongClamped};
    double projToEntry{alongClamped};
    double bodyFromEntry{glm::length(fromEntry)};

    // Velocity relative to the grid material frame at the body position.
    glm::dvec3 relativeVelocity{
        rigidBody->m_velocity -
        RotatingFrameUtils::velocityAtPoint(*gridBody, rigidBody->getPosition())};
    double axialVelocity{glm::dot(relativeVelocity, axis)};
    glm::dvec3 tangentialVelocity{relativeVelocity - axialVelocity * axis};

    // Re-arm seat capture once the projection has backed away from the seat.
    if (!m_isSeatCaptureArmed && projToSeat > m_seatRearmDistance) {
        m_isSeatCaptureArmed = true;
    }

    // ========== Axial movement: free WASD travel (walking-like, low speed) ==========
    // Project the view-relative movement input onto the axis, so facing the seat and
    // pressing W moves in, facing the door and pressing W moves out.
    double axialInput{0.0};
    if (inputs.m_movementDirection != glm::ivec3{0, 0, 0}) {
        glm::dquat viewOrientation{glm::conjugate(
            ArticulationUtils::quatLookAtYForward(inputs.m_viewDirection, seat.m_up))};
        glm::dvec3 inputWorld{viewOrientation *
                              glm::dvec3{
                                  static_cast<double>(inputs.m_movementDirection.x),
                                  static_cast<double>(inputs.m_movementDirection.y),
                                  static_cast<double>(inputs.m_movementDirection.z)}};
        double inputLength{glm::length(inputWorld)};
        if (inputLength > 1e-9) {
            axialInput = glm::dot(inputWorld / inputLength, axis);
        }
    }
    // One-sided seat wall: throttle the forward target to the sqrt(2ad) speed that
    // still stops at the seat, and reverse it past the seat. Sharing the free axial
    // channel's profile and accel clamp keeps the approach continuous across the
    // seat, so forward input smoothly reaches zero there instead of overshooting and
    // bouncing. The pilot can only leave through the entry (door), never the front.
    double targetAxialVelocity{axialInput * m_corridorSpeed};
    double axialToSeat{axisLength - along};  // >0 behind the seat, <0 past it (front)
    double seatLimitVelocity{MotionServo::velocityToward(
        glm::dvec3{axialToSeat, 0.0, 0.0}, m_corridorMaxAxialAccel).x};
    targetAxialVelocity = glm::min(targetAxialVelocity, seatLimitVelocity);
    double axialAcceleration{
        glm::clamp((targetAxialVelocity - axialVelocity) * m_corridorAxialResponse,
                   -m_corridorMaxAxialAccel, m_corridorMaxAxialAccel)};

    // ========== Tangential hold: bang-bang back onto the axis ==========
    // Only the tangential (off-axis) direction is force-limited; the axial direction
    // stays free for movement. The near-axis ramp only shapes the profile speed to
    // stop chatter at d=0 - the correction keeps full braking authority
    // (m_corridorTangentialAccel), otherwise it cannot arrest residual tangential
    // velocity on the axis and oscillates.
    double tangentialRampAcc{
        m_corridorTangentialAccel *
        glm::min(glm::length(tangentialOffset) / m_corridorTangentialRamp, 1.0)};
    glm::dvec3 tangentialTargetVelocity{
        MotionServo::velocityToward(-tangentialOffset, tangentialRampAcc * 0.5)};
    glm::dvec3 tangentialAcceleration{tangentialTargetVelocity - tangentialVelocity};
    double tangentialMag{glm::length(tangentialAcceleration)};
    if (tangentialMag > m_corridorTangentialAccel) {
        tangentialAcceleration =
            tangentialAcceleration * (m_corridorTangentialAccel / tangentialMag);
    }

    // ========== Combine, add frame compensation ==========
    glm::dvec3 controlAcceleration{axis * axialAcceleration + tangentialAcceleration};
    glm::dvec3 compensation{RotatingFrameUtils::centrifugalCoriolisCompensation(
        rigidBody->getMass(), gridBody->getAngularVelocityWorld(),
        rigidBody->getPosition() - gridBody->getWorldCenterOfMass(), relativeVelocity)};
    result.m_wrench.m_force = controlAcceleration * rigidBody->getMass() + compensation;

    // ========== Orientation: bang-bang, fully locked to the seat pose ==========
    glm::dvec3 targetAngularVelocity{MotionServo::towardOrientation(
        rigidBody->getOrientation(), seat.m_orientation, m_angularAccelerationMax, 0.5,
        0.1)};
    glm::dmat3 inertia{RotatingFrameUtils::effectiveInertia(*rigidBody, gridBody.get())};
    result.m_wrench.m_torque = MotionServo::torque(
        targetAngularVelocity, gridBody->getAngularVelocityWorld(),
        rigidBody->getAngularVelocityWorld(), m_angularAccelerationMax, inertia);

    result.m_wrench.m_reactionBody = target.m_gridBody;

    // ========== Transitions (projection onto the entry->seat segment) ==========
    // projToEntry clamps to 0 at/behind the entry plane, so a hair above 0 means the
    // projection has backed out to the door - never a mid-corridor or past-seat pose.
    constexpr double exitProjectionEpsilon{1e-3};
    if (m_isSeatCaptureArmed && projToSeat < target.m_seatArriveDistance) {
        result.m_wantSeat = true;
    } else if (projToEntry < exitProjectionEpsilon &&
               bodyFromEntry > target.m_exitBodyDistance) {
        result.m_wantRelease = true;
    }

    return result;
}

DigibotDockingMode::Result DigibotDockingMode::updateSeated(
    const std::shared_ptr<RigidBody>& rigidBody,
    const std::shared_ptr<RigidBody>& gridBody,
    const Target& target) {
    Result result{};
    if (!rigidBody || !gridBody || rigidBody->getMass() <= 0.0) {
        result.m_wantRelease = true;
        return result;
    }

    // Producer (makeTarget) must fill every distance; -1 means a field was left unset.
    assert(target.m_seatArriveDistance > 0.0 && target.m_exitBodyDistance > 0.0 &&
           "docking target not fully configured");

    SeatFrame seat{seatFrameWorld(*gridBody, target)};

    glm::dvec3 toSeat{seat.m_position - rigidBody->getPosition()};
    double distance{glm::length(toSeat)};

    // Restraint overwhelmed - the pilot is thrown back into the docking corridor.
    if (distance > m_unseatDistance) {
        result.m_wantRelease = true;
        return result;
    }

    // ========== Positional restraint: sqrt(2ad) toward the seat ==========
    // Shared bang-bang profile (same as walking's hover control), relative to the
    // seat point's material velocity. The near-seat ramp tames the sqrt curve at d=0
    // to stop rest chatter, so it only shapes the velocity profile - the correction
    // keeps full braking authority (m_seatMaxAcceleration), otherwise it cannot
    // arrest residual velocity near the seat and overshoots.
    double margin{0.5};
    double profileAcceleration{m_seatMaxAcceleration *
                               glm::min(distance / m_seatRampDistance, 1.0)};
    glm::dvec3 seatRelativeVelocity{
        rigidBody->m_velocity -
        RotatingFrameUtils::velocityAtPoint(*gridBody, seat.m_position)};
    glm::dvec3 targetVelocity{MotionServo::velocityToward(
        toSeat, profileAcceleration * (1.0 - margin))};

    glm::dvec3 restraintAcceleration{targetVelocity - seatRelativeVelocity};
    double restraintMagnitude{glm::length(restraintAcceleration)};
    if (restraintMagnitude > m_seatMaxAcceleration) {
        restraintAcceleration =
            restraintAcceleration * (m_seatMaxAcceleration / restraintMagnitude);
        restraintMagnitude = m_seatMaxAcceleration;
    }

    // Effective mass along the control direction (reaction lands on the grid)
    double effectiveMass{rigidBody->getMass()};
    if (restraintMagnitude > 1e-9) {
        effectiveMass = RotatingFrameUtils::effectiveMass(
            *rigidBody, gridBody.get(), restraintAcceleration / restraintMagnitude,
            rigidBody->getPosition() - gridBody->getWorldCenterOfMass());
    }

    // Feed-forward for the rotating frame so the restraint only carries
    // disturbances, not the steady-state centripetal load.
    glm::dvec3 relativeVelocity{
        rigidBody->m_velocity -
        RotatingFrameUtils::velocityAtPoint(*gridBody, rigidBody->getPosition())};
    glm::dvec3 compensation{RotatingFrameUtils::centrifugalCoriolisCompensation(
        rigidBody->getMass(), gridBody->getAngularVelocityWorld(),
        rigidBody->getPosition() - gridBody->getWorldCenterOfMass(), relativeVelocity)};

    // Hard restraint-strength limit on the total force: the seat can only hold on
    // so hard - anything beyond this shows up as sway (and eventually unseats).
    glm::dvec3 force{restraintAcceleration * effectiveMass + compensation};
    double forceLimit{m_seatMaxAcceleration * rigidBody->getMass()};
    double forceMagnitude{glm::length(force)};
    if (forceMagnitude > forceLimit) {
        force = force * (forceLimit / forceMagnitude);
    }

    result.m_wrench.m_force = force;

    // ========== Orientation restraint toward the seat pose ==========
    // Same sqrt(2ad) servo as walking. The angular acceleration limit inside the
    // servo caps the restraint torque at inertia * m_seatAngularAcceleration.
    glm::dvec3 targetAngularVelocity{MotionServo::towardOrientation(
        rigidBody->getOrientation(), seat.m_orientation, m_seatAngularAcceleration, 0.5,
        0.1)};
    glm::dmat3 inertia{RotatingFrameUtils::effectiveInertia(*rigidBody, gridBody.get())};
    result.m_wrench.m_torque = MotionServo::torque(
        targetAngularVelocity, gridBody->getAngularVelocityWorld(),
        rigidBody->getAngularVelocityWorld(), m_seatAngularAcceleration, inertia);

    result.m_wrench.m_reactionBody = target.m_gridBody;
    return result;
}
