// CockpitDockingCoordinator.h
#pragma once

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
    CockpitDockingCoordinator() = default;

    // Called once per physics step, before integration, using the last completed
    // step's collisions; runs before the character controllers consume their
    // docking targets.
    void stepControl(CharacterSubsystem* characterSubsystem,
                     PhysicsEngine* physicsEngine, GridSubsystem* gridSubsystem);

    // Invoke the callback for every digibot currently SEATED in a cockpit, in
    // engagement order (deterministic). Grid and cockpit are re-validated at call
    // time; the cockpit reference is owned by the grid.
    void forEachSeatedPilot(
        const std::function<void(Digibot&, Grid&, const CockpitBlock&)>& callback) const;

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
