// JobPriorities.h
#pragma once

namespace JobPriorities {
    // Graphics subsystem priorities
    static constexpr int GRAPHICS_REMOVE = -2;
    static constexpr int GRAPHICS_UPDATE = -1;
    
    // Physics subsystem priorities  
    static constexpr int PHYSICS_UPDATE = 0;

    // Grid cell operations
    static constexpr int CELL_OPERATIONS = 2;
    static constexpr int GRID_CLEANUP = 1;
    
    // Classify inner, faces, edges, corners
    static constexpr int GRID_CELL_CLASSIFICATION = -3;
    
    // Analysis subsystem priorities
    static constexpr int STRUCTURAL_ANALYSIS = -4;
}