// SecondaryCell.h
#pragma once

#include "GridCell.h"

// Generic occupancy sentinel for any multi-cell block's non-anchor cells.
// Stores the anchor coordinate so Grid can resolve the owning anchor without
// a raw pointer. Contributes zero mass — the anchor owns all physics data.
class SecondaryCell : public GridCell {
public:
    glm::ivec3 m_anchorCoord;

    SecondaryCell(const glm::ivec3& coord, const glm::ivec3& anchorCoord, CellType type);
    virtual ~SecondaryCell() = default;

    virtual std::tuple<double, glm::dvec3, glm::dmat3> getMassProperties() const override;
    virtual glm::ivec3 getAnchorCoord() const override { return m_anchorCoord; }
};
