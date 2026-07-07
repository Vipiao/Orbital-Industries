// DigibotDockingMode.h
#pragma once

#include "DigibotModeTypes.h"
#include "../../physics/PhysicsUnits.h"
#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>
#include <memory>
#include <vector>

class RigidBody;
class Collider;

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
        // Cockpit's physics cells, ignored by the digibot body while docked so it can phase
        // through the cockpit cube. Raw because grid cells are unique_ptr-owned by the grid.
        std::vector<Collider*> m_cockpitCells{};
        glm::dvec3 m_entryPositionLocal{0.0, 0.0, 0.0};  // entry (door)
        glm::dvec3 m_seatPositionLocal{0.0, 0.0, 0.0};   // seat
        glm::dquat m_seatOrientationLocal{1.0, 0.0, 0.0, 0.0};
        // Distances are filled by the producer (makeTarget) each step; -1 marks unset
        // so a forgotten field trips the asserts in updateDocked/updateSeated instead
        // of silently disabling the transition.
        double m_seatArriveDistance{-1.0};  // proj->seat below this: sit down
        double m_exitBodyDistance{-1.0};    // body far from entry (proj back at door): release
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

    // Rates and accelerations are configured in SI units and converted to per-tick
    // units (PhysicsUnits); distances stay in meters.
    // Corridor movement (docked). Axial: free WASD movement at a low max speed.
    // Tangential: bang-bang hold onto the entry->seat axis.
    double m_corridorSpeed{PhysicsUnits::metersPerSecond(1.92)};        // Max axial speed
    double m_corridorAxialResponse{PhysicsUnits::perSecond(12.8)};      // Axial vel tracking
    double m_corridorMaxAxialAccel{PhysicsUnits::metersPerSecondSquared(40.96)}; // force cap/mass
    double m_corridorTangentialAccel{PhysicsUnits::metersPerSecondSquared(81.92)}; // cap/mass
    double m_corridorTangentialRamp{0.1};  // Tangential near-axis ramp distance (m)
    double m_angularAccelerationMax{PhysicsUnits::radiansPerSecondSquared(65.536)}; // servo cap

    // Seat restraint (seated state): sqrt(2ad) bang-bang controller toward the seat
    // pose (same method as walking's hover control), with hard force/torque limits.
    double m_seatMaxAcceleration{PhysicsUnits::metersPerSecondSquared(204.8)}; // force/mass; => sway
    double m_seatRampDistance{0.1};      // Acceleration ramps down inside this range (m)
    double m_seatAngularAcceleration{PhysicsUnits::radiansPerSecondSquared(204.8)}; // torque/inertia
    double m_unseatDistance{0.5};        // Pulled further than this => thrown out (m)
    double m_seatRearmDistance{0.6};     // proj must leave B by this to re-arm capture (m)

    bool m_isSeatCaptureArmed{true};
};
