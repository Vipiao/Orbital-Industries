// ThrusterBlock.h
#pragma once

#include "../GridCell.h"
#include <tuple>

/**
 * @brief Anchor cell for a thruster block.
 * Occupies this coord and the adjacent +Y coord (held by ThrusterSecondaryCell).
 * Owns the mass/physics representation for the full 1x2x1 thruster unit.
 */
class ThrusterBlock : public GridCell {
public:
    static constexpr CellType TYPE = CellType::THRUSTER;

    explicit ThrusterBlock(const glm::ivec3& anchorCoord);
    virtual ~ThrusterBlock() = default;

    glm::ivec3 m_secondCoord; // anchorCoord + {0,1,0}

    virtual std::tuple<double, glm::dvec3, glm::dmat3> getMassProperties() const override;
};
