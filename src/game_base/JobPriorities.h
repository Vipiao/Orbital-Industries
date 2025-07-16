// JobPriorities.h
#pragma once

namespace JobPriorities {
    // Graphics subsystem priorities
    static constexpr int GRAPHICS_REMOVE = -2;
    static constexpr int GRAPHICS_UPDATE = -1;
    
    // Physics subsystem priorities  
    static constexpr int PHYSICS_UPDATE = 0;
    
    // Analysis subsystem priorities
    static constexpr int STRUCTURAL_ANALYSIS = -3;
}