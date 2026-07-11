// ThrusterBlock.h
#pragma once

#include "../BlockAnchor.h"
#include "../BlockGeometryPart.h"
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

    static constexpr std::string_view COLOR_TEX_PATH  = "../media/models/thruster/albedo.png";
    static constexpr std::string_view NORMAL_TEX_PATH = "../media/models/thruster/normal.png";
    static constexpr glm::dvec3       MODEL_CENTRE{0.5, 0.5, 0.5};

    // Force produced at full throttle, in newtons.
    static constexpr double THRUST_FORCE_NEWTONS{200000.0};

    // The thruster is a single opaque part.
    static std::vector<BlockGeometryPart> geometryParts();

    ThrusterBlock(const glm::ivec3& anchorCoord, const glm::dquat& orientation);
    virtual ~ThrusterBlock() = default;

    // All offsets occupied by this block (including {0,0,0} for the anchor).
    // Orientation-dependent: the secondary cell follows the dominant axis.
    static std::vector<glm::ivec3> footprintOffsets(const glm::dquat& orientation);

    // Returns the grid axis that the model's +Y maps to under the given orientation.
    static glm::ivec3 dominantAxis(const glm::dquat& orientation);

    // Convenience: anchorCoord + dominantAxis(orientation).
    static glm::ivec3 secondCoord(const glm::ivec3& anchorCoord, const glm::dquat& orientation);

    // Direction the thruster pushes the grid, in grid-local space. The plume
    // exits along the model's +Y (the nozzle), so the force is the opposite axis.
    static glm::ivec3 thrustForceDirection(const glm::dquat& orientation);

    // Throttle in [0, 1]; the applied force is m_thrustLevel * THRUST_FORCE_NEWTONS.
    // Persists until the next pilot command, so a burn outlives its pilot.
    double m_thrustLevel{0.0};

    virtual std::vector<glm::ivec3> secondaryCoords() const override;
    virtual std::tuple<double, glm::dvec3, glm::dmat3> getMassProperties() const override;
    virtual size_t computeHash() const override;
};
