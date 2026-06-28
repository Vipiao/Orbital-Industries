// ThrusterBlock.h
#pragma once

#include "../BlockAnchor.h"
#include <string_view>
#include <tuple>
#include <vector>

// Anchor cell for a thruster block.
// Occupies this coord and one adjacent coord determined by m_orientation.
// The model's +Y axis is the canonical thrust direction; m_orientation rotates
// that axis into grid-local space, selecting which neighbour cell is occupied.
// Owns the mass/physics representation for the full 1x2x1 thruster unit.
class ThrusterBlock : public BlockAnchor {
public:
    static constexpr CellType TYPE = CellType::THRUSTER;

    static constexpr std::string_view GEOMETRY_PATH   = "../media/models/thruster/thruster_v2.obj";
    static constexpr std::string_view COLOR_TEX_PATH  = "../media/models/thruster/albedo.png";
    static constexpr std::string_view NORMAL_TEX_PATH = "../media/models/thruster/normal.png";
    static constexpr glm::dvec3       MODEL_CENTRE{0.5, 0.5, 0.5};

    ThrusterBlock(const glm::ivec3& anchorCoord, const glm::dquat& orientation);
    virtual ~ThrusterBlock() = default;

    // All offsets occupied by this block (including {0,0,0} for the anchor).
    // Orientation-dependent: the secondary cell follows the dominant axis.
    static std::vector<glm::ivec3> footprintOffsets(const glm::dquat& orientation);

    // Returns the grid axis that the model's +Y maps to under the given orientation.
    static glm::ivec3 dominantAxis(const glm::dquat& orientation);

    // Convenience: anchorCoord + dominantAxis(orientation).
    static glm::ivec3 secondCoord(const glm::ivec3& anchorCoord, const glm::dquat& orientation);

    virtual std::vector<glm::ivec3> secondaryCoords() const override;
    virtual std::tuple<double, glm::dvec3, glm::dmat3> getMassProperties() const override;
};
