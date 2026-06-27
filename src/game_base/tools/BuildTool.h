// BuildTool.h
#pragma once

#include <memory>
#include <functional>
#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>
#include <vector>
#include "graphics/MeshManager2D/GeometryData.h"

// Forward declarations
class GameBase;
class Grid;
class RadialMenu;
class GeometryInstance;
class Geometry;
class Instance;

class BuildTool {
public:
    enum class BlockType {
        STRUCTURAL_BLOCK,
        THRUSTER
    };

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
    GameBase*    m_gameBase;
    RadialMenu*  m_radialMenu;
    double       m_interactionRange;

    // Internal state
    bool      m_active{false};
    BlockType m_selectedBlockType{BlockType::STRUCTURAL_BLOCK};

    // Input flags (set in preRenderCallback, consumed in onPhysicsUpdateComplete)
    bool m_doCreate{false};
    bool m_doRemove{false};

    // --- Orientation ---
    // m_targetOrientation snaps on each keypress (90° increments).
    // m_renderedOrientation exponentially slews toward it each render frame.
    glm::dquat m_targetOrientation{1.0, 0.0, 0.0, 0.0};
    glm::dquat m_renderedOrientation{1.0, 0.0, 0.0, 0.0};

    // --- Ghost preview (thruster only) ---
    std::weak_ptr<Geometry> m_ghostGeometry;
    std::weak_ptr<Instance> m_ghostInstance;
    int  m_ghostColorTextureUnit{-1};
    int  m_currentGhostSsboIndex{-1};

    // --- Menu structure ---
    int64_t m_buildToolParentId{-1};
    int64_t m_centerNodeId{-1};

    // --- Build crosshair (2D) ---
    std::weak_ptr<GeometryData>    m_buildCrosshairGeometry;
    std::weak_ptr<GeometryInstance> m_buildCrosshairInstance;
    double     m_buildCrosshairTransparency{0.75};
    glm::dvec2 m_crosshairOffset;
    glm::dvec2 m_crosshairScale;

    // --- Icon textures ---
    int m_constructionIconTextureIndex{-1};
    int m_thrusterIconTextureIndex{-1};

    // Helper methods
    void createMenuStructure(int64_t parentNodeId);
    void addGridBlock(Grid* grid, int x, int y, int z);
    std::vector<glm::ivec3> removeGridBlock(Grid* grid, int x, int y, int z);
    void handleGridSplitting(std::weak_ptr<Grid> targetGrid, const std::vector<glm::ivec3>& removedCoords);

    // Ghost management
    void updateGhost(const std::vector<std::weak_ptr<Grid>>& availableGrids);
    void hideGhost();
};
