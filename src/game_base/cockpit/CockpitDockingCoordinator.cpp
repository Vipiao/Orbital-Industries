// CockpitDockingCoordinator.cpp
#include "CockpitDockingCoordinator.h"
#include "CockpitBlock.h"
#include "../Grid.h"
#include "../GridSubsystem.h"
#include "../../characters/CharacterSubsystem.h"
#include "../../characters/digibot/Digibot.h"
#include "../../characters/digibot/DigibotController.h"
#include "../../physics/PhysicsEngine.h"
#include "../../physics/RigidBody.h"
#include "../../physics/GridCollider.h"
#include <algorithm>
#include <cassert>
#include <limits>
#include <tuple>

namespace {

// Entry (door) and seat in the grid rigid body's local frame (grid lattice minus
// centre of mass, matching the worldToGrid/gridToWorld convention).
glm::dvec3 entryLocal(const Grid& grid, const CockpitBlock& cockpit) {
    glm::dvec3 centreLattice{glm::dvec3{cockpit.coordinates} + CockpitBlock::MODEL_CENTRE};
    glm::dvec3 lattice{centreLattice +
                       cockpit.m_orientation * CockpitBlock::ENTRY_POINT_CANONICAL};
    return lattice - grid.m_centerOfMass;
}

glm::dvec3 seatLocal(const Grid& grid, const CockpitBlock& cockpit) {
    glm::dvec3 centreLattice{glm::dvec3{cockpit.coordinates} + CockpitBlock::MODEL_CENTRE};
    glm::dvec3 lattice{centreLattice +
                       cockpit.m_orientation * CockpitBlock::SEAT_POSITION_CANONICAL};
    return lattice - grid.m_centerOfMass;
}

// Build the controller-facing docking target from a cockpit block on a grid.
// The cockpit's physics cells (one per 2x2x2 footprint offset that still exists), used as
// collision exceptions so the docked digibot can phase through just the cockpit cube. These
// do not change frame to frame, so they are resolved once at the docking transition rather
// than on every target refresh.
std::vector<Collider*> resolveCockpitCells(const Grid& grid, const CockpitBlock& cockpit) {
    std::vector<Collider*> cells;
    auto gridCollider = grid.getCollider().lock();
    if (!gridCollider || gridCollider->getTypeId() != GridCollider::TYPE_ID) {
        return cells;
    }
    GridCollider* gc{static_cast<GridCollider*>(gridCollider.get())};
    for (const glm::ivec3& offset : CockpitBlock::footprintOffsets(cockpit.m_orientation)) {
        if (Collider* cell{gc->getCell(cockpit.coordinates + offset)}) {
            cells.push_back(cell);
        }
    }
    return cells;
}

DigibotDockingMode::Target makeTarget(const Grid& grid, const CockpitBlock& cockpit) {
    DigibotDockingMode::Target target{};
    target.m_gridBody = grid.getRigidBody();
    target.m_entryPositionLocal = entryLocal(grid, cockpit);
    target.m_seatPositionLocal = seatLocal(grid, cockpit);
    target.m_seatOrientationLocal = cockpit.m_orientation;
    target.m_seatArriveDistance = CockpitBlock::SEAT_ARRIVE_DISTANCE;
    target.m_exitBodyDistance = CockpitBlock::EXIT_BODY_DISTANCE;
    return target;
}

// Find the cockpit anchor cell on a grid; nullptr if the anchor is gone or not a
// cockpit anymore. A COCKPIT-typed cell that is its own anchor is a CockpitBlock
// by construction (Grid only creates CockpitBlock anchors plus SecondaryCells).
const CockpitBlock* findCockpit(Grid& grid, const glm::ivec3& anchor) {
    const GridCell* cell{grid.getCellFromRegistry(anchor)};
    if (!cell || cell->type != CellType::COCKPIT || cell->getAnchorCoord() != anchor) {
        return nullptr;
    }
    return static_cast<const CockpitBlock*>(cell);
}

}

void CockpitDockingCoordinator::stepControl(CharacterSubsystem* characterSubsystem,
                                            PhysicsEngine* physicsEngine,
                                            GridSubsystem* gridSubsystem) {
    if (!characterSubsystem || !physicsEngine || !gridSubsystem) {
        return;
    }

    // Prune records whose digibot no longer exists
    m_engagements.erase(
        std::remove_if(m_engagements.begin(), m_engagements.end(),
                       [](const Engagement& engagement) {
                           return engagement.m_digibot.expired();
                       }),
        m_engagements.end());

    for (const auto& character : characterSubsystem->getCharacters()) {
        std::shared_ptr<Digibot> digibot{std::dynamic_pointer_cast<Digibot>(character)};
        if (digibot) {
            updateDigibot(digibot, physicsEngine, gridSubsystem);
        }
    }
}

void CockpitDockingCoordinator::applyDesiredTransitions() {
    std::vector<const Digibot*> released{};
    for (const Engagement& engagement : m_engagements) {
        std::shared_ptr<Digibot> digibot{engagement.m_digibot.lock()};
        DigibotController* controller{digibot ? digibot->getController() : nullptr};
        if (!controller) {
            continue;
        }
        switch (controller->getDesiredDockingTransition()) {
            case DigibotController::DockingTransition::SEAT:
                controller->forceSeat();
                break;
            case DigibotController::DockingTransition::UNSEAT:
                controller->requestUnseat();
                break;
            case DigibotController::DockingTransition::RELEASE:
                controller->clearDockingTarget();
                released.push_back(digibot.get());
                break;
            case DigibotController::DockingTransition::NONE:
                break;
        }
    }
    for (const Digibot* digibot : released) {
        removeEngagement(digibot);
    }
}

CockpitDockingCoordinator::DockingStatus CockpitDockingCoordinator::captureDockingStatus(
    const Digibot* digibot) const {
    DockingStatus status{};
    for (const Engagement& engagement : m_engagements) {
        if (engagement.m_digibot.lock().get() != digibot) {
            continue;
        }
        std::shared_ptr<Grid> grid{engagement.m_grid.lock()};
        const DigibotController* controller{digibot->getController()};
        if (grid && controller &&
            controller->getDockingState() != DigibotController::DockingState::FREE) {
            status.m_state = controller->getDockingState();
            status.m_gridId = grid->uniqueId;
            status.m_anchor = engagement.m_anchor;
        }
        break;
    }
    return status;
}

void CockpitDockingCoordinator::forceDockingStatus(const std::shared_ptr<Digibot>& digibot,
                                                   const DockingStatus& status,
                                                   GridSubsystem* gridSubsystem) {
    DigibotController* controller{digibot ? digibot->getController() : nullptr};
    if (!controller || !gridSubsystem) {
        return;
    }

    if (status.m_state == DigibotController::DockingState::FREE) {
        if (controller->getDockingState() != DigibotController::DockingState::FREE) {
            controller->clearDockingTarget();
            removeEngagement(digibot.get());
        }
        return;
    }

    std::shared_ptr<Grid> grid{gridSubsystem->getGridById(status.m_gridId).lock()};
    const CockpitBlock* cockpit{grid ? findCockpit(*grid, status.m_anchor) : nullptr};
    if (!cockpit) {
        return;  // this simulation does not (yet) have the cockpit; try again later
    }

    // Engaged elsewhere: release first, then engage the right cockpit below.
    Engagement* engagement{findEngagement(digibot.get())};
    bool sameCockpit{engagement && engagement->m_grid.lock() == grid &&
                     engagement->m_anchor == status.m_anchor};
    if (controller->getDockingState() != DigibotController::DockingState::FREE &&
        !sameCockpit) {
        controller->clearDockingTarget();
        removeEngagement(digibot.get());
    }

    if (controller->getDockingState() == DigibotController::DockingState::FREE) {
        DigibotDockingMode::Target target{makeTarget(*grid, *cockpit)};
        target.m_cockpitCells = resolveCockpitCells(*grid, *cockpit);
        controller->setDockingTarget(target);
        m_engagements.push_back({digibot, grid, status.m_anchor});
    }

    if (status.m_state == DigibotController::DockingState::SEATED) {
        controller->forceSeat();
    } else {
        controller->requestUnseat();  // no-op unless seated
    }
}

void CockpitDockingCoordinator::forEachSeatedPilot(
    const std::function<void(Digibot&, Grid&, const CockpitBlock&)>& callback) const {
    for (const Engagement& engagement : m_engagements) {
        std::shared_ptr<Digibot> digibot{engagement.m_digibot.lock()};
        std::shared_ptr<Grid> grid{engagement.m_grid.lock()};
        if (!digibot || !grid) {
            continue;
        }
        DigibotController* controller{digibot->getController()};
        if (!controller || controller->getDockingState() !=
                               DigibotController::DockingState::SEATED) {
            continue;
        }
        const CockpitBlock* cockpit{findCockpit(*grid, engagement.m_anchor)};
        if (!cockpit) {
            continue;
        }
        callback(*digibot, *grid, *cockpit);
    }
}

CockpitDockingCoordinator::Engagement* CockpitDockingCoordinator::findEngagement(
    const Digibot* digibot) {
    for (Engagement& engagement : m_engagements) {
        if (engagement.m_digibot.lock().get() == digibot) {
            return &engagement;
        }
    }
    return nullptr;
}

void CockpitDockingCoordinator::removeEngagement(const Digibot* digibot) {
    m_engagements.erase(
        std::remove_if(m_engagements.begin(), m_engagements.end(),
                       [digibot](const Engagement& engagement) {
                           return engagement.m_digibot.lock().get() == digibot;
                       }),
        m_engagements.end());
}

void CockpitDockingCoordinator::updateDigibot(const std::shared_ptr<Digibot>& digibot,
                                              PhysicsEngine* physicsEngine,
                                              GridSubsystem* gridSubsystem) {
    DigibotController* controller{digibot->getController()};
    if (!controller) {
        return;
    }

    if (controller->getDockingState() == DigibotController::DockingState::FREE) {
        removeEngagement(digibot.get());

        Candidate candidate{scan(digibot, physicsEngine, gridSubsystem)};
        if (candidate.m_grid) {
            const CockpitBlock* cockpit{findCockpit(*candidate.m_grid,
                                                    candidate.m_anchor)};
            if (cockpit) {
                DigibotDockingMode::Target target{makeTarget(*candidate.m_grid, *cockpit)};
                // Resolve the ignored cockpit cells only here, at the docking transition;
                // the docked refresh below does not need them.
                target.m_cockpitCells = resolveCockpitCells(*candidate.m_grid, *cockpit);
                controller->setDockingTarget(target);
                // setDockingTarget must leave FREE, or the engagement we push now
                // desyncs from the controller state (record with no docking).
                assert(controller->getDockingState() !=
                           DigibotController::DockingState::FREE &&
                       "setDockingTarget failed to engage docking");
                m_engagements.push_back(
                    {digibot, candidate.m_grid, candidate.m_anchor});
            }
        }
        return;
    }

    // Docked or seated: validate the engaged cockpit still exists and refresh the
    // target (the grid moves and its centre of mass can shift as blocks change).
    Engagement* engagement{findEngagement(digibot.get())};
    std::shared_ptr<Grid> grid{engagement ? engagement->m_grid.lock() : nullptr};
    const CockpitBlock* cockpit{grid ? findCockpit(*grid, engagement->m_anchor)
                                     : nullptr};
    if (!cockpit) {
        controller->clearDockingTarget();
        removeEngagement(digibot.get());
        return;
    }
    controller->setDockingTarget(makeTarget(*grid, *cockpit));
}

double CockpitDockingCoordinator::entryDistance(const Grid& grid,
                                                const CockpitBlock& cockpit,
                                                const glm::dvec3& bodyWorldPos,
                                                const glm::dvec3& bodyUpWorld) {
    // Work in grid-lattice space; distances there equal world distances (rigid).
    glm::dvec3 bodyLattice{grid.worldToGrid(bodyWorldPos)};
    glm::dvec3 entryLattice{entryLocal(grid, cockpit) + grid.m_centerOfMass};
    double distance{glm::length(bodyLattice - entryLattice)};
    if (distance > CockpitBlock::ENTER_RADIUS) {
        return -1.0;
    }

    // Orientation gate: the body up must be within a limit of the cockpit up.
    // cockpit up in lattice space is cockpitOrientation * +z; bring the body up into
    // lattice space with the grid orientation.
    auto gridBody = grid.getRigidBody().lock();
    if (!gridBody) {
        return -1.0;
    }
    glm::dvec3 cockpitUpLattice{cockpit.m_orientation * glm::dvec3{0.0, 0.0, 1.0}};
    glm::dvec3 bodyUpLattice{glm::conjugate(gridBody->m_orientation) * bodyUpWorld};
    if (glm::dot(bodyUpLattice, cockpitUpLattice) < CockpitBlock::ENTRY_UP_ALIGNMENT_MIN) {
        return -1.0;
    }
    return distance;
}

CockpitDockingCoordinator::Candidate CockpitDockingCoordinator::scan(
    const std::shared_ptr<Digibot>& digibot, PhysicsEngine* physicsEngine,
    GridSubsystem* gridSubsystem) {
    Candidate best{};
    double bestDistance{std::numeric_limits<double>::max()};

    auto rigidBody = digibot->getRigidBody().lock();
    auto sensor = digibot->getWalkingSensor().lock();
    if (!rigidBody || !sensor) {
        return best;
    }
    glm::dvec3 bodyPos{rigidBody->m_position};
    glm::dvec3 bodyUp{rigidBody->m_orientation * glm::dvec3{0.0, 0.0, 1.0}};

    // Evaluate one cell: if it belongs to a cockpit whose entry point the body is
    // close to (and aligned with), keep it as the deterministic best.
    auto considerCell = [&](const std::shared_ptr<Grid>& grid,
                            const glm::ivec3& cellCoord) {
        const GridCell* cell{grid->getCellFromRegistry(cellCoord)};
        if (!cell || cell->type != CellType::COCKPIT) {
            return;
        }
        glm::ivec3 anchor{cell->getAnchorCoord()};
        const CockpitBlock* cockpit{findCockpit(*grid, anchor)};
        if (!cockpit) {
            return;
        }
        double distance{entryDistance(*grid, *cockpit, bodyPos, bodyUp)};
        if (distance < 0.0) {
            return;
        }
        // Deterministic ordering: distance, then grid id, then anchor coordinate
        bool better{distance < bestDistance};
        if (distance == bestDistance && best.m_grid) {
            if (grid->uniqueId != best.m_grid->uniqueId) {
                better = grid->uniqueId < best.m_grid->uniqueId;
            } else {
                better = std::tie(anchor.x, anchor.y, anchor.z) <
                         std::tie(best.m_anchor.x, best.m_anchor.y, best.m_anchor.z);
            }
        }
        if (better) {
            best.m_grid = grid;
            best.m_anchor = anchor;
            bestDistance = distance;
        }
    };

    // Single pass over the sensor's collisions; each contact resolves straight to
    // its grid through the subsystem's collider->grid hash map.
    std::vector<std::shared_ptr<Grid>> overlappedGrids{};
    for (const CollisionData& collision :
         physicsEngine->getCollisions(sensor.get())) {
        auto grid = gridSubsystem->getGridFromCollider(collision.otherCollider).lock();
        if (!grid) {
            continue;
        }
        if (std::find(overlappedGrids.begin(), overlappedGrids.end(), grid) ==
            overlappedGrids.end()) {
            overlappedGrids.push_back(grid);
        }

        // The cell just behind each contact point ("the block you are next to"):
        // nudge past the surface so flooring lands inside the block.
        for (const glm::dvec3& contactPoint : collision.contactPoints) {
            glm::dvec3 toContact{contactPoint - bodyPos};
            double toContactLength{glm::length(toContact)};
            if (toContactLength < 1e-9) {
                continue;
            }
            glm::dvec3 sample{contactPoint + toContact / toContactLength * 0.02};
            glm::ivec3 cellCoord{glm::floor(grid->worldToGrid(sample))};
            considerCell(grid, cellCoord);
        }
    }

    // The cell containing the body ("the block you are in") — tunneling guard: a
    // fast body can be deep inside the cockpit volume within one step, where the
    // contact geometry may be degenerate. This point-in-lattice query cannot miss.
    // Once per overlapped grid.
    for (const std::shared_ptr<Grid>& grid : overlappedGrids) {
        glm::ivec3 bodyCell{glm::floor(grid->worldToGrid(bodyPos))};
        considerCell(grid, bodyCell);
    }

    // best.m_grid is null if nothing matched.
    return best;
}
