// StructuralBlock.cpp
#include "StructuralBlock.h"

StructuralBlock::StructuralBlock(const glm::ivec3& coords, Grid* parent, CellType cellType)
    : GridCell(coords, parent, cellType) {
}