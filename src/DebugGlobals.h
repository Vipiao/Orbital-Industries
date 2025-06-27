// DebugGlobals.h
#pragma once

class DebugRenderer;

namespace DebugGlobals {
    extern DebugRenderer* g_debugRenderer;
    
    inline void setDebugRenderer(DebugRenderer* renderer) {
        g_debugRenderer = renderer;
    }
    
    inline DebugRenderer* getDebugRenderer() {
        return g_debugRenderer;
    }
}