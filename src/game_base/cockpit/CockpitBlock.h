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

    CockpitBlock(const glm::ivec3& anchorCoord, const glm::dquat& orientation);
    virtual ~CockpitBlock() = default;

    // All offsets occupied by this block (including {0,0,0} for the anchor).
    // Orientation-independent: the 2x2x2 cube footprint is the same in all orientations.
    static std::vector<glm::ivec3> footprintOffsets(const glm::dquat& orientation);

    virtual std::vector<glm::ivec3> secondaryCoords() const override;
    virtual std::tuple<double, glm::dvec3, glm::dmat3> getMassProperties() const override;
};
