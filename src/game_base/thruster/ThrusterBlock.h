// ThrusterBlock.h
#pragma once

#include "../GridCell.h"
#include <glm/gtc/quaternion.hpp>
#include <tuple>

/**
 * @brief Anchor cell for a thruster block.
 * Occupies this coord and one adjacent coord determined by m_orientation.
 * The model's +Y axis is the canonical thrust direction; m_orientation rotates
 * that axis into grid-local space, selecting which neighbour cell is occupied.
 * Owns the mass/physics representation for the full 1x2x1 thruster unit.
 */
class ThrusterBlock : public GridCell {
public:
    static constexpr CellType TYPE = CellType::THRUSTER;

    ThrusterBlock(const glm::ivec3& anchorCoord, const glm::dquat& orientation);
    virtual ~ThrusterBlock() = default;

    glm::ivec3  m_secondCoord;   // anchorCoord + dominantAxis(orientation)
    glm::dquat  m_orientation;   // full orientation of the block in grid-local space

    /**
     * @brief Returns the grid axis that the model's +Y maps to under the given orientation.
     * Used to determine which neighbour cell the thruster's secondary occupies.
     */
    static glm::ivec3 dominantAxis(const glm::dquat& orientation);

    /** Convenience: anchorCoord + dominantAxis(orientation). */
    static glm::ivec3 secondCoord(const glm::ivec3& anchorCoord, const glm::dquat& orientation);

    virtual std::tuple<double, glm::dvec3, glm::dmat3> getMassProperties() const override;
};
