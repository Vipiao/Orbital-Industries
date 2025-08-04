// StructuralBlock.h
#pragma once

#include "GridCell.h"

/**
 * @brief Concrete implementation of GridCell for structural blocks
 * These are the standard building blocks that make up grid structures
 */
class StructuralBlock : public GridCell {
public:
    // Type identifier for this block type
    static constexpr CellType TYPE = CellType::STRUCTURAL_BLOCK;
    
    StructuralBlock(const glm::ivec3& coords);
    virtual ~StructuralBlock() = default;
};