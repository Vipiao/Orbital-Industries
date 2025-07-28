// StructuralBlock.h
#pragma once

#include "GridCell.h"

/**
 * @brief Concrete implementation of GridCell for structural blocks
 * These are the standard building blocks that make up grid structures
 */
class StructuralBlock : public GridCell {
public:
    StructuralBlock(const glm::ivec3& coords, Grid* parent, CellType cellType = CellType::ARMOR);
    virtual ~StructuralBlock() = default;
};