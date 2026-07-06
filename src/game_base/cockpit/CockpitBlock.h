// CockpitBlock.h
#pragma once

#include "../BlockAnchor.h"
#include <string_view>
#include <tuple>
#include <vector>

// Anchor cell for a 2x2x2 cockpit block.
// The block always occupies a fixed cube of 8 cells regardless of orientation.
// Orientation controls which direction the cockpit faces and will govern
// control axes in future.
class CockpitBlock : public BlockAnchor {
public:
    static constexpr CellType TYPE = CellType::COCKPIT;

    static constexpr std::string_view GEOMETRY_PATH   = "../media/models/cockpit/model.obj";
    static constexpr std::string_view COLOR_TEX_PATH  = "../media/models/cockpit/albedo.png";
    static constexpr std::string_view NORMAL_TEX_PATH = "../media/models/cockpit/normal.png";
    static constexpr glm::dvec3       MODEL_CENTRE{1.0, 1.0, 1.0};

    // Docking geometry in the cockpit's canonical frame: origin at the block centre
    // (anchor + MODEL_CENTRE), +y forward, +z up. The pilot enters through the back
    // face (-y) along the y axis. The docking corridor is the segment entry -> seat.
    static constexpr glm::dvec3 SEAT_POSITION_CANONICAL{0.0, -0.4, -0.25};   // seat
    static constexpr glm::dvec3 ENTRY_POINT_CANONICAL{0.0, -1.0, -0.2};    // entry (door)

    // Enter: dock when the body is within this of the entry point and roughly aligned
    // with the cockpit up (dot of body-up and cockpit-up above the minimum).
    static constexpr double ENTER_RADIUS{0.4};
    static constexpr double ENTRY_UP_ALIGNMENT_MIN{0.7}; // ~45 degrees

    // Transitions (projection of the body onto the entry -> seat segment):
    static constexpr double SEAT_ARRIVE_DISTANCE{0.05}; // proj near seat => seated
    // Release once the projection has backed out to the entry plane and the body is
    // this far from the entry point.
    static constexpr double EXIT_BODY_DISTANCE{0.6};

    // On entry the body sits ~ENTER_RADIUS from the entry point. If that already
    // exceeded EXIT_BODY_DISTANCE the pilot would release the instant it docked.
    static_assert(ENTER_RADIUS < EXIT_BODY_DISTANCE,
                  "enter radius must be below exit distance, else docking releases on entry");

    CockpitBlock(const glm::ivec3& anchorCoord, const glm::dquat& orientation);
    virtual ~CockpitBlock() = default;

    // All offsets occupied by this block (including {0,0,0} for the anchor).
    // Orientation-independent: the 2x2x2 cube footprint is the same in all orientations.
    static std::vector<glm::ivec3> footprintOffsets(const glm::dquat& orientation);

    virtual std::vector<glm::ivec3> secondaryCoords() const override;
    virtual std::tuple<double, glm::dvec3, glm::dmat3> getMassProperties() const override;
};
