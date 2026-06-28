// BlockAnchor.h
#pragma once

#include "GridCell.h"
#include <glm/gtc/quaternion.hpp>
#include <vector>

// Base class for the anchor cell of any multi-cell (or single-cell) special block.
// Holds orientation and exposes the secondary cell coordinates so Grid can operate
// on any block type without knowing the concrete type at removal time.
class BlockAnchor : public GridCell {
public:
    glm::dquat m_orientation;

    BlockAnchor(const glm::ivec3& coords, CellType type, const glm::dquat& orientation)
        : GridCell{coords, type}, m_orientation{orientation} {}

    virtual ~BlockAnchor() = default;

    // Coordinates of the cells this block occupies beyond the anchor.
    // Returns empty for single-cell blocks.
    virtual std::vector<glm::ivec3> secondaryCoords() const = 0;

protected:
    BlockAnchor() = default;
};
