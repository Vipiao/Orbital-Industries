// BuildTool.h
#pragma once

#include <memory>
#include <functional>
#include <glm/glm.hpp>
#include <vector>
#include "../graphics/MeshManager2D/GeometryData.h"

// Forward declarations
class GameBase;
class Grid;
class RadialMenu;
class GeometryInstance;
class Grid;

class BuildTool {
public:
    BuildTool(GameBase* gameBase, RadialMenu* radialMenu, int64_t parentNodeId, double interactionRange);
    ~BuildTool();
    
    // Activation control
    void activate();
    void deactivate();
    bool isActive() const { return m_active; }
    
    // Callback hooks
    void preRenderCallback(bool doCreate, bool doRemove);
    void onPhysicsUpdateComplete(const std::vector<std::weak_ptr<Grid>>& availableGrids);
    
private:
    // Interaction range
    double m_interactionRange;

    GameBase* m_gameBase;
    RadialMenu* m_radialMenu;
    
    // Internal state
    bool m_active{false};
    
    // Input flags
    bool m_doCreate{false};
    bool m_doRemove{false};
    
    // Menu structure
    int64_t m_buildToolParentId{-1};
    int64_t m_centerNodeId{-1};
    
    // Build crosshair
    std::weak_ptr<GeometryData> m_buildCrosshairGeometry;
    std::weak_ptr<GeometryInstance> m_buildCrosshairInstance;
    double m_buildCrosshairTransparency = 0.75;
    glm::dvec2 m_crosshairOffset;
    glm::dvec2 m_crosshairScale;
    
    // Icon texture
    int m_constructionIconTextureIndex{-1};
    
    // Helper methods
    void createMenuStructure(int64_t parentNodeId);
    void addGridBlock(Grid* grid, int x, int y, int z);
    void removeGridBlock(Grid* grid, int x, int y, int z);
    void handleGridSplitting(std::weak_ptr<Grid> targetGrid, const glm::ivec3& removedPos);
};