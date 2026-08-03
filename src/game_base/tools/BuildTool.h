// BuildTool.h
#pragma once

#include <array>
#include <memory>
#include <functional>
#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>
#include <vector>
#include "graphics/MeshManager2D/Geometry2D.h"
#include "utils/PolyhedronProcessor.h"

// Forward declarations
class GameBase;
class Grid;
class RadialMenu;
class Instance2D;
class Geometry;
class Instance;
class StructuralGhostGeometry;

class BuildTool {
public:
    enum class BlockType {
        STRUCTURAL_BLOCK,
        THRUSTER,
        COCKPIT
    };

    // What the player asked of the tool this frame; consumed by the next control step.
    struct Input {
        bool m_create{false};
        bool m_remove{false};
        bool m_copy{false};  // adopt the aimed-at block as the current selection
    };

    BuildTool(GameBase* gameBase, RadialMenu* radialMenu, int64_t parentNodeId, double interactionRange);
    ~BuildTool();

    // Activation control
    void activate();
    void deactivate();
    bool isActive() const { return m_active; }

    // Frame/step hooks
    void framePreRender(const Input& input);
    void stepControl(const std::vector<std::weak_ptr<Grid>>& availableGrids);

private:
    GameBase*    m_gameBase;
    RadialMenu*  m_radialMenu;
    double       m_interactionRange;

    // Internal state
    bool      m_active{false};
    BlockType m_selectedBlockType{BlockType::STRUCTURAL_BLOCK};

    // Shape placed structural blocks are given, as it sits in its own cell. The
    // build orientation is applied on top of it, so a copy can be rotated freely.
    std::array<glm::ivec3, 8> m_structuralShape{PolyhedronProcessor::DEFAULT_VERTICES};

    // Colour placed structural blocks are given. Independent of the colour tool,
    // which paints blocks that already exist.
    glm::dvec4 m_structuralColor{1.0, 1.0, 1.0, 1.0};

    // Set in framePreRender, consumed in stepControl
    Input m_pendingInput{};

    // --- Orientation ---
    // m_targetOrientation snaps on each keypress (90° increments).
    // m_renderedOrientation exponentially slews toward it each render frame.
    glm::dquat m_targetOrientation{1.0, 0.0, 0.0, 0.0};
    glm::dquat m_renderedOrientation{1.0, 0.0, 0.0, 0.0};

    // --- Ghost preview resources (per block type) ---
    // Structural ghosts are generated from the shape being placed; the others are
    // loaded models.
    std::unique_ptr<StructuralGhostGeometry> m_structuralGhost;

    std::weak_ptr<Geometry> m_thrusterGhostGeometry;
    int m_thrusterGhostColorTextureUnit{-1};

    std::weak_ptr<Geometry> m_cockpitGhostGeometry;
    int m_cockpitGhostColorTextureUnit{-1};

    // Active ghost instance state (one instance at a time)
    std::weak_ptr<Geometry> m_activeGhostGeometry;
    std::weak_ptr<Instance> m_ghostInstance;
    int m_currentGhostSsboIndex{-1};

    // --- Menu structure ---
    int64_t m_buildToolParentId{-1};
    int64_t m_centerNodeId{-1};

    // --- Build crosshair (2D) ---
    // One geometry per block type; the crosshair instance lives in the one that matches
    // the current selection.
    std::weak_ptr<Geometry2D> m_blockCrosshairGeometry;
    std::weak_ptr<Geometry2D> m_thrusterCrosshairGeometry;
    std::weak_ptr<Geometry2D> m_cockpitCrosshairGeometry;

    std::weak_ptr<Geometry2D> m_activeCrosshairGeometry;
    std::weak_ptr<Instance2D> m_buildCrosshairInstance;
    double     m_buildCrosshairTransparency{0.75};
    glm::dvec2 m_crosshairOffset;
    glm::dvec2 m_crosshairScale;

    // --- Icon textures ---
    int m_blockIconTextureIndex{-1};
    int m_thrusterIconTextureIndex{-1};
    int m_cockpitIconTextureIndex{-1};

    // Helper methods
    void createMenuStructure(int64_t parentNodeId);
    void addGridBlock(Grid* grid, int x, int y, int z);

    // Adopt the block at the coord as the current selection: its type, the
    // orientation of oriented blocks and the shape of structural blocks.
    void copyBlockAt(Grid& grid, const glm::ivec3& coord);

    // Ghost management
    void updateGhost(const std::vector<std::weak_ptr<Grid>>& availableGrids);
    void hideGhost();

    // Crosshair management
    std::weak_ptr<Geometry2D> crosshairGeometryFor(BlockType blockType) const;
    void showCrosshair();
    void hideCrosshair();
};
