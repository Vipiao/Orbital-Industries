// CockpitDockingCoordinator.h
#pragma once

#include "../../characters/digibot/DigibotController.h"
#include "utils/ByteStream.h"

#include <cstdint>
#include <functional>
#include <glm/glm.hpp>
#include <memory>
#include <vector>

class CharacterSubsystem;
class Digibot;
class Grid;
class GridSubsystem;
class PhysicsEngine;
class CockpitBlock;

// Game-layer glue between grids/cockpits and the digibots' docking controllers.
// Owned by GameBase and run once per physics step for every digibot: either scans
// for a cockpit enter zone near the character (FREE state) or validates/refreshes
// the engaged cockpit's docking target (DOCKED/SEATED states). Runs regardless of
// the active mode -- docking is world physics, not control policy. The controllers
// never see Grid; they only get world/body-frame data via DigibotDockingMode::Target.
class CockpitDockingCoordinator {
public:
    // A digibot's docking situation as a value: its state and, when engaged, the
    // cockpit (grid + anchor cell) it is engaged with. Capturable and forcible, so
    // one simulation's situation can be reproduced in another.
    struct DockingStatus {
        DigibotController::DockingState m_state{DigibotController::DockingState::FREE};
        std::uint64_t m_gridId{0};        // meaningful only when not FREE
        glm::ivec3 m_anchor{0, 0, 0};     // meaningful only when not FREE

        void serialize(ByteWriter& writer) const {
            writer.write(m_state);
            writer.write(m_gridId);
            writer.write(m_anchor.x);
            writer.write(m_anchor.y);
            writer.write(m_anchor.z);
        }
        bool deserialize(ByteReader& reader) {
            return reader.read(m_state) && reader.read(m_gridId) &&
                   reader.read(m_anchor.x) && reader.read(m_anchor.y) &&
                   reader.read(m_anchor.z);
        }
        // Deserialized status is untrusted; a raw out-of-range state byte must be
        // dropped at the door, not applied.
        bool isValid() const {
            return m_state >= DigibotController::DockingState::FREE &&
                   m_state <= DigibotController::DockingState::SEATED;
        }
    };

    CockpitDockingCoordinator() = default;

    // Called once per physics step, before integration, using the last completed
    // step's collisions; runs before the character controllers consume their
    // docking targets.
    void stepControl(CharacterSubsystem* characterSubsystem,
                     PhysicsEngine* physicsEngine, GridSubsystem* gridSubsystem);

    // Called once per physics step, after the character controllers ran: applies
    // the docking transitions their physics asked for (seat capture, thrown out,
    // climbed out). The coordinator owns the docking state machine; the
    // controllers only report desires.
    void applyDesiredTransitions();

    // Invoke the callback for every digibot currently SEATED in a cockpit, in
    // engagement order (deterministic). Grid and cockpit are re-validated at call
    // time; the cockpit reference is owned by the grid.
    void forEachSeatedPilot(
        const std::function<void(Digibot&, Grid&, const CockpitBlock&)>& callback) const;

    // This digibot's current docking situation as a value.
    DockingStatus captureDockingStatus(const Digibot* digibot) const;

    // Reconcile the digibot toward the given situation: no-op when it already
    // matches, otherwise release/engage/seat/unseat as needed. A status referring
    // to a grid or cockpit this simulation does not (yet) have is ignored.
    void forceDockingStatus(const std::shared_ptr<Digibot>& digibot,
                            const DockingStatus& status, GridSubsystem* gridSubsystem);

private:
    // Cockpit a digibot is currently engaged with (DOCKED or SEATED)
    struct Engagement {
        std::weak_ptr<Digibot> m_digibot{};
        std::weak_ptr<Grid> m_grid{};
        glm::ivec3 m_anchor{0, 0, 0};
    };

    // Best cockpit found by a scan; m_grid is null when none matched.
    struct Candidate {
        std::shared_ptr<Grid> m_grid{};
        glm::ivec3 m_anchor{0, 0, 0};
    };

    void updateDigibot(const std::shared_ptr<Digibot>& digibot,
                       PhysicsEngine* physicsEngine, GridSubsystem* gridSubsystem);

    Engagement* findEngagement(const Digibot* digibot);
    void removeEngagement(const Digibot* digibot);

    // Single pass over the walking-sensor collisions: resolve each contact's grid
    // via the subsystem's collider->grid map, then hash-look-up the cell behind the
    // contact (and the cell containing the body, once per grid) for cockpit cells
    // whose enter zone contains the body. Returns the best candidate (deterministic
    // tie-break) or an empty grid pointer. O(contacts), independent of world size.
    static Candidate scan(const std::shared_ptr<Digibot>& digibot,
                          PhysicsEngine* physicsEngine, GridSubsystem* gridSubsystem);

    // Distance from the body to the entry point if the body is within the enter
    // radius of it and aligned with the cockpit up; negative otherwise.
    static double entryDistance(const Grid& grid, const CockpitBlock& cockpit,
                                const glm::dvec3& bodyWorldPos,
                                const glm::dvec3& bodyUpWorld);

    // One record per engaged digibot; expired entries are pruned each step.
    // Iteration/lookup is by stable character pointer identity, driven in
    // CharacterSubsystem order, so behaviour stays deterministic.
    std::vector<Engagement> m_engagements{};
};
