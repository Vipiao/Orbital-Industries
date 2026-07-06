// DigibotDockingMode.h
#pragma once

#include "DigibotModeTypes.h"
#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>
#include <memory>

class RigidBody;

// Cockpit entry/exit corridor and seat restraint physics. The corridor is the
// segment entry (door) -> seat. While docked: WASD moves the body freely along the
// axis at a low speed (walking-like), a bang-bang controller holds it onto the axis
// in the tangential direction only, and a bang-bang servo locks orientation to the
// seat. No pull toward the seat while docked. While seated, a force-capped bang-bang
// restraint holds the body in the seat, so hard manoeuvres cause sway and can throw
// the pilot out. Pure wrench producer — the controller applies the result (with the
// equal-opposite reaction on the grid body).
class DigibotDockingMode {
public:
    // Docking target in the grid rigid body's local frame. Seat orientation:
    // +y = cockpit forward (entry is along -y), +z = up.
    struct Target {
        std::weak_ptr<RigidBody> m_gridBody{};
        glm::dvec3 m_entryPositionLocal{0.0, 0.0, 0.0};  // entry (door)
        glm::dvec3 m_seatPositionLocal{0.0, 0.0, 0.0};   // seat
        glm::dquat m_seatOrientationLocal{1.0, 0.0, 0.0, 0.0};
        double m_seatArriveDistance{0.25};    // proj->seat below this: sit down
        double m_exitProjectionDistance{0.3}; // proj->entry below this ...
        double m_exitBodyDistance{1.0};       // ...and body far from entry: release
    };

    struct Result {
        DigibotWrench m_wrench{};
        bool m_wantSeat{false};    // Docked -> seated transition requested
        bool m_wantRelease{false}; // Docked -> free, or seated -> docked (thrown out)
    };

    DigibotDockingMode() = default;

    Result updateDocked(const std::shared_ptr<RigidBody>& rigidBody,
                        const std::shared_ptr<RigidBody>& gridBody,
                        const Target& target, const DigibotModeInputs& inputs);

    Result updateSeated(const std::shared_ptr<RigidBody>& rigidBody,
                        const std::shared_ptr<RigidBody>& gridBody,
                        const Target& target);

    // Called when docking begins and when the pilot unseats (Escape or thrown out),
    // so the seat does not immediately recapture the exiting pilot.
    void armSeatCapture() { m_isSeatCaptureArmed = true; }
    void disarmSeatCapture() { m_isSeatCaptureArmed = false; }

private:
    struct SeatFrame {
        glm::dvec3 m_position{0.0, 0.0, 0.0};
        glm::dquat m_orientation{1.0, 0.0, 0.0, 0.0};
        glm::dvec3 m_forward{0.0, 1.0, 0.0};
        glm::dvec3 m_up{0.0, 0.0, 1.0};
    };
    static SeatFrame seatFrameWorld(const RigidBody& gridBody, const Target& target);

    // All rates and accelerations are in per-physics-step units (like RigidBody).
    // Corridor movement (docked). Axial: free WASD movement at a low max speed.
    // Tangential: bang-bang hold onto the entry->seat axis.
    double m_corridorSpeed{0.03};          // Max axial speed (lower than walking)
    double m_corridorAxialResponse{0.2};   // Axial velocity tracking rate
    double m_corridorMaxAxialAccel{0.01};  // Axial force cap / mass
    double m_corridorTangentialAccel{0.02};// Bang-bang tangential accel cap / mass
    double m_corridorTangentialRamp{0.1};  // Tangential near-axis ramp distance
    double m_angularAccelerationMax{0.016};// Orientation servo accel cap

    // Seat restraint (seated state): sqrt(2ad) bang-bang controller toward the seat
    // pose (same method as walking's hover control), with hard force/torque limits.
    double m_seatMaxAcceleration{0.05};  // Max restraint force / mass; exceeded => sway
    double m_seatRampDistance{0.1};      // Acceleration ramps down inside this range
    double m_seatAngularAcceleration{0.05}; // Max restraint torque / inertia
    double m_unseatDistance{0.5};        // Pulled further than this => thrown out
    double m_seatRearmDistance{0.6};     // proj must leave B by this to re-arm capture

    bool m_isSeatCaptureArmed{true};
};
