// CellType.h
#pragma once

// Cell type enum for grid cells. A multi-cell block contributes one anchor type;
// the cells it fills beyond the anchor are all SECONDARY regardless of block type,
// so adding a block type adds exactly one enumerator.
enum class CellType {
    STRUCTURAL_BLOCK,
    SECONDARY,
    THRUSTER,
    COCKPIT,
    REACTION_WHEEL
};
