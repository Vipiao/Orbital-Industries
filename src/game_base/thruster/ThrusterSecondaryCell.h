// ThrusterSecondaryCell.h
#pragma once

#include "../GridCell.h"

class ThrusterBlock;

/**
 * @brief Sentinel occupying the +Y cell of a thruster.
 * Points back to the owning ThrusterBlock anchor.
 * Contributes zero mass (all mass is on the anchor).
 */
class ThrusterSecondaryCell : public GridCell {
public:
    static constexpr CellType TYPE = CellType::THRUSTER_SECONDARY;

    ThrusterSecondaryCell(const glm::ivec3& coord, ThrusterBlock* owner);
    virtual ~ThrusterSecondaryCell() = default;

    ThrusterBlock* m_owner; // Non-owning back-pointer to anchor

    virtual std::tuple<double, glm::dvec3, glm::dmat3> getMassProperties() const override;
};
