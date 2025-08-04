// StructuralBlock.h
#pragma once

#include "GridCell.h"
#include <array>

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

    // Shape definition - 8 corner vertices in local integer coordinates
    std::array<glm::ivec3, 8> m_localVertices;
    
    // Scaling factor for local coordinates
    int m_maxSize = 4;
};