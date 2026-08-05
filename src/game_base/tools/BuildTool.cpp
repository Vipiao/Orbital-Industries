// BuildTool.cpp
#include "BuildTool.h"
#include "../GameBase.h"
#include "../GridSubsystem.h"
#include "graphics/GraphicsEngine.h"
#include "../RadialMenu.h"
#include "../Grid.h"
#include "../StructuralBlock.h"
#include "../StructuralCommand.h"
#include "../thruster/ThrusterBlock.h"
#include "../cockpit/CockpitBlock.h"
#include "../reaction_wheel/ReactionWheelBlock.h"
#include "graphics/MeshManager2D/MeshManager2D.h"
#include "graphics/MeshManager2D/Instance2D.h"
#include "graphics/instanceHandler/InstanceHandler.h"
#include "graphics/KeyboardHandler.h"
#include "StructuralGhostGeometry.h"
#include <iostream>
#include <optional>
#include <functional>
#include "../GridRaycast.h"

// Exponential decay constant for orientation slerp animation (radians/second feel).
static constexpr double k_orientationSlerpRate = 20.0;

// Opacity every ghost is drawn with.
static constexpr double k_ghostTransparency = 0.45;

// Tries to place a multi-cell block whose cells sit at (anchor + offsets[i]).
// Iterates each offset as the "aimed-at" cell, shifting the anchor accordingly.
// Returns the anchor that makes all cells land on free positions, or nullopt.
static std::optional<glm::ivec3> findMultiCellAnchor(
    const glm::ivec3& targetPos,
    const std::vector<glm::ivec3>& offsets,
    const std::function<bool(const glm::ivec3&)>& isOccupied)
{
    for (const auto& pivot : offsets) {
        const glm::ivec3 candidate = targetPos - pivot;
        bool fits = true;
        for (const auto& o : offsets) {
            if (isOccupied(candidate + o)) { fits = false; break; }
        }
        if (fits) return candidate;
    }
    return std::nullopt;
}

static CellType toCellType(BuildTool::BlockType blockType) {
    switch (blockType) {
        case BuildTool::BlockType::THRUSTER:       return CellType::THRUSTER;
        case BuildTool::BlockType::COCKPIT:        return CellType::COCKPIT;
        case BuildTool::BlockType::REACTION_WHEEL: return CellType::REACTION_WHEEL;
        default:                                   return CellType::STRUCTURAL_BLOCK;
    }
}

BuildTool::BuildTool(GameBase* gameBase, RadialMenu* radialMenu, int64_t parentNodeId, double interactionRange)
    : m_gameBase(gameBase), m_radialMenu(radialMenu), m_interactionRange(interactionRange) {

    if (!m_gameBase) {
        throw std::runtime_error("GameBase cannot be null");
    }
    if (!m_radialMenu) {
        throw std::runtime_error("RadialMenu cannot be null");
    }

    auto* ge = m_gameBase->m_graphicsEngine.get();

    // Built up front, like the ghost models below, so the first block placed does
    // not pay for the mesh upload.
    m_structuralGhost = std::make_unique<StructuralGhostGeometry>(ge);
    m_structuralGhost->get(m_structuralShape);

    // Icon textures
    m_blockIconTextureIndex    = ge->createInstanceTexture("../media/2d_graphics/07_block_icon.png");
    m_thrusterIconTextureIndex = ge->createInstanceTexture("../media/2d_graphics/09_thruster_icon.png");
    m_cockpitIconTextureIndex  = ge->createInstanceTexture("../media/2d_graphics/10_cockpit_icon.png");
    m_reactionWheelIconTextureIndex =
        ge->createInstanceTexture("../media/2d_graphics/11_reaction_wheel_icon.png");

    // Thruster ghost
    m_thrusterGhostColorTextureUnit = ge->createInstanceTexture("../media/models/thruster/albedo_ghost.png");
    m_thrusterGhostGeometry = ge->createInstanceGeometry(
        "../media/models/thruster/thruster_ghost.obj", RenderLayer::Transparent);

    // Cockpit ghost
    m_cockpitGhostColorTextureUnit = ge->createInstanceTexture("../media/models/cockpit/albedo_ghost.png");
    m_cockpitGhostGeometry = ge->createInstanceGeometry(
        "../media/models/cockpit/model_ghost.obj", RenderLayer::Transparent);

    // Reaction wheel ghost
    m_reactionWheelGhostColorTextureUnit =
        ge->createInstanceTexture("../media/models/reaction_wheel/albedo_ghost.png");
    m_reactionWheelGhostGeometry = ge->createInstanceGeometry(
        "../media/models/reaction_wheel/frame.obj", RenderLayer::Transparent);

    // Calculate crosshair offset and scale
    m_crosshairScale = glm::dvec2(0.1, 0.1);
    // 16x16 pixels of a 64x64 image where the aiming marks of every build icon are located
    m_crosshairOffset.x = 2.0 * (0.5 - 16.0 / 64.0) * m_crosshairScale.x;
    m_crosshairOffset.y = 2.0 * (0.5 - 16.0 / 64.0) * m_crosshairScale.y;

    createMenuStructure(parentNodeId);

    // The texture is baked into a Geometry2D, so each block type gets its own crosshair mesh
    // and the instance moves between them when the selection changes.
    MeshManager2D* meshManager2D = m_gameBase->m_graphicsEngine->getMeshManager2D();
    m_blockCrosshairGeometry = meshManager2D->loadMesh(
        "../media/blender/03_face.obj", "../media/2d_graphics/07_block_icon.png", -1, true);
    m_thrusterCrosshairGeometry = meshManager2D->loadMesh(
        "../media/blender/03_face.obj", "../media/2d_graphics/09_thruster_icon.png", -1, true);
    m_cockpitCrosshairGeometry = meshManager2D->loadMesh(
        "../media/blender/03_face.obj", "../media/2d_graphics/10_cockpit_icon.png", -1, true);
    m_reactionWheelCrosshairGeometry = meshManager2D->loadMesh(
        "../media/blender/03_face.obj", "../media/2d_graphics/11_reaction_wheel_icon.png", -1,
        true);
}

BuildTool::~BuildTool() {
    hideGhost();
    hideCrosshair();
}

void BuildTool::activate() {
    m_active = true;
    showCrosshair();
}

void BuildTool::deactivate() {
    m_active = false;
    hideGhost();
    hideCrosshair();
}

void BuildTool::framePreRender(const Input& input) {
    if (!m_active) {
        return;
    }

    if (input.m_create) m_pendingInput.m_create = true;
    if (input.m_remove) m_pendingInput.m_remove = true;
    if (input.m_copy)   m_pendingInput.m_copy = true;

    // --- Orientation cycling (only relevant for thruster) ---
    // Insert/Delete  → ±90° around world Y
    // Home/End       → ±90° around world Z
    // PgUp/PgDn      → ±90° around world X
    KeyboardHandler* kb = m_gameBase->m_graphicsEngine->getKeyboardHandler();
    const double half = glm::half_pi<double>();

    if (kb->m_insert.justPressed()) {
        m_targetOrientation = glm::normalize(
            glm::angleAxis(half, glm::dvec3{0.0, 1.0, 0.0}) * m_targetOrientation);
    }
    if (kb->m_delete.justPressed()) {
        m_targetOrientation = glm::normalize(
            glm::angleAxis(-half, glm::dvec3{0.0, 1.0, 0.0}) * m_targetOrientation);
    }
    if (kb->m_home.justPressed()) {
        m_targetOrientation = glm::normalize(
            glm::angleAxis(half, glm::dvec3{0.0, 0.0, 1.0}) * m_targetOrientation);
    }
    if (kb->m_end.justPressed()) {
        m_targetOrientation = glm::normalize(
            glm::angleAxis(-half, glm::dvec3{0.0, 0.0, 1.0}) * m_targetOrientation);
    }
    if (kb->m_pageUp.justPressed()) {
        m_targetOrientation = glm::normalize(
            glm::angleAxis(half, glm::dvec3{1.0, 0.0, 0.0}) * m_targetOrientation);
    }
    if (kb->m_pageDown.justPressed()) {
        m_targetOrientation = glm::normalize(
            glm::angleAxis(-half, glm::dvec3{1.0, 0.0, 0.0}) * m_targetOrientation);
    }

    // --- Exponential slerp of rendered orientation toward target ---
    double frameRate  = m_gameBase->m_graphicsEngine->getFrameRate();
    double deltaTime  = 1.0 / (frameRate > 0.0 ? frameRate : 60.0);
    double slerpAlpha = 1.0 - glm::exp(-k_orientationSlerpRate * deltaTime);
    m_renderedOrientation = glm::normalize(
        glm::slerp(m_renderedOrientation, m_targetOrientation, slerpAlpha));

    // --- Update ghost preview every render frame ---
    // Collect available grids for ghost raycasting.
    // We re-use the same grids stepControl uses, but gathered here for render-time use.
    // GameBase exposes grids via the grid subsystem.
    std::vector<std::weak_ptr<Grid>> gridsForGhost;
    for (const auto& g : m_gameBase->getGridSubsystem()->getGrids()) {
        if (g) gridsForGhost.push_back(g);
    }
    updateGhost(gridsForGhost);
}

void BuildTool::stepControl(const std::vector<std::weak_ptr<Grid>>& availableGrids) {
    if (!m_active) {
        return;
    }

    if (!m_pendingInput.m_create && !m_pendingInput.m_remove && !m_pendingInput.m_copy) {
        return;
    }

    glm::dvec3 startPos = m_gameBase->m_graphicsEngine->getCamPos();
    glm::dvec3 forward  = m_gameBase->m_graphicsEngine->getCamOri() * glm::dvec3(0.0, 1.0, 0.0);
    glm::dvec3 endPos   = startPos + forward * m_interactionRange;

    auto [_, timeRemainder] = m_gameBase->m_graphicsEngine->getRenderParameters();

    std::optional<GridRayHit> hit{
        GridRaycast::closestHit(availableGrids, startPos, endPos, timeRemainder)};
    std::shared_ptr<Grid> targetGrid{hit ? hit->m_grid.lock() : nullptr};

    if (m_pendingInput.m_copy && targetGrid) {
        copyBlockAt(*targetGrid, hit->m_hitCoord);
    }

    if (m_pendingInput.m_create) {
        if (targetGrid) {
            addGridBlock(targetGrid.get(), hit->m_placeCoord.x, hit->m_placeCoord.y,
                         hit->m_placeCoord.z);
        } else if (!hit) {
            glm::dvec3 newGridPos = startPos + forward * 2.0 - glm::dvec3{0.5};
            m_gameBase->requestStructuralEdit(StructuralCommand::spawnGrid(
                newGridPos, toCellType(m_selectedBlockType), m_targetOrientation,
                StructuralBlock::rotatedVertices(m_structuralShape, m_targetOrientation),
                m_structuralColor));
        }
    }

    if (m_pendingInput.m_remove && targetGrid) {
        m_gameBase->requestStructuralEdit(
            StructuralCommand::removeCell(targetGrid->uniqueId, hit->m_hitCoord));
    }

    m_pendingInput = Input{};
}

void BuildTool::copyBlockAt(Grid& grid, const glm::ivec3& coord) {
    const GridCell* cell = grid.getCellFromRegistry(coord);
    if (!cell) return;

    // Aiming at any cell of a block copies the whole block, so resolve to the
    // anchor: it is the cell that carries the block's type and orientation.
    const glm::ivec3 anchorCoord{cell->getAnchorCoord()};
    const GridCell* anchor = grid.getCellFromRegistry(anchorCoord);
    if (!anchor) return;

    switch (anchor->type) {
        case CellType::SECONDARY:
            return;  // an anchor is never a secondary cell
        case CellType::STRUCTURAL_BLOCK: {
            const StructuralBlock* block = grid.getCell(anchorCoord);
            if (!block) return;
            m_selectedBlockType = BlockType::STRUCTURAL_BLOCK;
            m_structuralShape   = block->m_localVertices;
            m_structuralColor   = block->m_color;
            // The shape already sits the way the source block does in its grid, so
            // the copy starts unrotated and matches what the player is aiming at.
            m_targetOrientation = glm::dquat{1.0, 0.0, 0.0, 0.0};
            break;
        }
        case CellType::THRUSTER: {
            auto it = grid.getThrusterCells().find(anchorCoord);
            if (it == grid.getThrusterCells().end()) return;
            m_selectedBlockType = BlockType::THRUSTER;
            m_targetOrientation = it->second.m_orientation;
            break;
        }
        case CellType::COCKPIT: {
            auto it = grid.getCockpitCells().find(anchorCoord);
            if (it == grid.getCockpitCells().end()) return;
            m_selectedBlockType = BlockType::COCKPIT;
            m_targetOrientation = it->second.m_orientation;
            break;
        }
        case CellType::REACTION_WHEEL: {
            auto it = grid.getReactionWheelCells().find(anchorCoord);
            if (it == grid.getReactionWheelCells().end()) return;
            m_selectedBlockType = BlockType::REACTION_WHEEL;
            m_targetOrientation = it->second.m_orientation;
            break;
        }
    }

    showCrosshair();
}

void BuildTool::createMenuStructure(int64_t parentNodeId) {
    auto activateCallback   = [this]() { activate(); };
    auto deactivateCallback = [this]() { deactivate(); };

    m_buildToolParentId = m_radialMenu->createNode(
        parentNodeId, m_blockIconTextureIndex, activateCallback, deactivateCallback);

    m_centerNodeId = m_radialMenu->createNode(
        m_buildToolParentId, -1, activateCallback, deactivateCallback);

    // Picking the plain block from the menu also drops any copied shape.
    m_radialMenu->createNode(m_buildToolParentId, m_blockIconTextureIndex,
        [this]() {
            m_selectedBlockType = BlockType::STRUCTURAL_BLOCK;
            m_structuralShape   = PolyhedronProcessor::DEFAULT_VERTICES;
            m_structuralColor   = glm::dvec4{1.0, 1.0, 1.0, 1.0};
            activate();
        },
        deactivateCallback);

    m_radialMenu->createNode(m_buildToolParentId, m_thrusterIconTextureIndex,
        [this]() { m_selectedBlockType = BlockType::THRUSTER; activate(); },
        deactivateCallback);

    m_radialMenu->createNode(m_buildToolParentId, m_reactionWheelIconTextureIndex,
        [this]() { m_selectedBlockType = BlockType::REACTION_WHEEL; activate(); },
        deactivateCallback);

    m_radialMenu->createNode(m_buildToolParentId, m_cockpitIconTextureIndex,
        [this]() { m_selectedBlockType = BlockType::COCKPIT; activate(); },
        deactivateCallback);
}

std::weak_ptr<Geometry2D> BuildTool::crosshairGeometryFor(BlockType blockType) const {
    switch (blockType) {
        case BlockType::THRUSTER:       return m_thrusterCrosshairGeometry;
        case BlockType::COCKPIT:        return m_cockpitCrosshairGeometry;
        case BlockType::REACTION_WHEEL: return m_reactionWheelCrosshairGeometry;
        default:                  return m_blockCrosshairGeometry;
    }
}

void BuildTool::showCrosshair() {
    auto geometry = crosshairGeometryFor(m_selectedBlockType).lock();
    if (!geometry) return;

    // A crosshair of the wrong block type is discarded before the new one is created
    if (m_activeCrosshairGeometry.lock() != geometry) hideCrosshair();
    if (m_buildCrosshairInstance.lock()) return;

    m_buildCrosshairInstance = geometry->addInstance();
    m_activeCrosshairGeometry = geometry;

    if (auto instance = m_buildCrosshairInstance.lock()) {
        instance->m_position = glm::dvec2{m_crosshairOffset.x, -m_crosshairOffset.y};
        instance->m_scale    = m_crosshairScale;
        instance->m_color    = glm::dvec4{1.0, 1.0, 1.0, m_buildCrosshairTransparency};
        geometry->updateInstanceInBuffer(instance.get());
    }
}

void BuildTool::hideCrosshair() {
    if (auto instance = m_buildCrosshairInstance.lock()) {
        if (auto geometry = m_activeCrosshairGeometry.lock()) {
            geometry->removeInstance(instance);
        }
    }
    m_buildCrosshairInstance.reset();
    m_activeCrosshairGeometry.reset();
}

void BuildTool::addGridBlock(Grid* grid, int x, int y, int z) {
    if (!grid) return;
    const glm::ivec3 targetPos{x, y, z};
    const auto& registry = grid->getCellRegistry();

    if (m_selectedBlockType == BlockType::STRUCTURAL_BLOCK) {
        m_gameBase->requestStructuralEdit(StructuralCommand::addCell(
            grid->uniqueId, targetPos,
            StructuralBlock::rotatedVertices(m_structuralShape, m_targetOrientation),
            m_structuralColor));
    } else if (m_selectedBlockType == BlockType::THRUSTER) {
        auto anchor = findMultiCellAnchor(
            targetPos, ThrusterBlock::footprintOffsets(m_targetOrientation),
            [&registry](const glm::ivec3& pos) { return registry.count(pos) > 0; });
        if (anchor) {
            m_gameBase->requestStructuralEdit(StructuralCommand::addThruster(
                grid->uniqueId, *anchor, m_targetOrientation));
        }
    } else if (m_selectedBlockType == BlockType::COCKPIT) {
        auto anchor = findMultiCellAnchor(
            targetPos, CockpitBlock::footprintOffsets(m_targetOrientation),
            [&registry](const glm::ivec3& pos) { return registry.count(pos) > 0; });
        if (anchor) {
            m_gameBase->requestStructuralEdit(StructuralCommand::addCockpit(
                grid->uniqueId, *anchor, m_targetOrientation));
        }
    } else if (m_selectedBlockType == BlockType::REACTION_WHEEL) {
        auto anchor = findMultiCellAnchor(
            targetPos, ReactionWheelBlock::footprintOffsets(m_targetOrientation),
            [&registry](const glm::ivec3& pos) { return registry.count(pos) > 0; });
        if (anchor) {
            m_gameBase->requestStructuralEdit(StructuralCommand::addReactionWheel(
                grid->uniqueId, *anchor, m_targetOrientation));
        }
    }
}

void BuildTool::updateGhost(const std::vector<std::weak_ptr<Grid>>& availableGrids) {
    std::weak_ptr<Geometry> desiredGeometry;
    int colorTexUnit = -1;
    std::vector<glm::ivec3> footprint;
    glm::dvec3 modelCentre;
    glm::dvec3 ghostTint{1.0, 1.0, 1.0};

    if (m_selectedBlockType == BlockType::THRUSTER) {
        desiredGeometry = m_thrusterGhostGeometry;
        colorTexUnit    = m_thrusterGhostColorTextureUnit;
        footprint       = ThrusterBlock::footprintOffsets(m_targetOrientation);
        modelCentre     = ThrusterBlock::MODEL_CENTRE;
    } else if (m_selectedBlockType == BlockType::COCKPIT) {
        desiredGeometry = m_cockpitGhostGeometry;
        colorTexUnit    = m_cockpitGhostColorTextureUnit;
        footprint       = CockpitBlock::footprintOffsets(m_targetOrientation);
        modelCentre     = CockpitBlock::MODEL_CENTRE;
    } else if (m_selectedBlockType == BlockType::REACTION_WHEEL) {
        desiredGeometry = m_reactionWheelGhostGeometry;
        colorTexUnit    = m_reactionWheelGhostColorTextureUnit;
        footprint       = ReactionWheelBlock::footprintOffsets(m_targetOrientation);
        modelCentre     = ReactionWheelBlock::MODEL_CENTRE;
    } else {
        // A structural block is the single-cell case: its ghost is built from the
        // shape being placed and tinted rather than textured.
        desiredGeometry = m_structuralGhost->get(m_structuralShape);
        footprint       = {glm::ivec3{0, 0, 0}};
        modelCentre     = glm::dvec3{0.5};
        ghostTint       = glm::dvec3{m_structuralColor};
    }

    auto geom = desiredGeometry.lock();
    if (!geom) { hideGhost(); return; }

    auto [_, timeRemainder] = m_gameBase->m_graphicsEngine->getRenderParameters();
    glm::dvec3 camPos  = m_gameBase->m_graphicsEngine->getCamPos();
    glm::dvec3 forward = m_gameBase->m_graphicsEngine->getCamOri() * glm::dvec3(0.0, 1.0, 0.0);
    glm::dvec3 endPos  = camPos + forward * m_interactionRange;

    std::optional<GridRayHit> hit{
        GridRaycast::closestHit(availableGrids, camPos, endPos, timeRemainder)};
    if (!hit) { hideGhost(); return; }

    auto bestGrid = hit->m_grid.lock();
    if (!bestGrid) { hideGhost(); return; }

    const auto& registry = bestGrid->getCellRegistry();
    auto anchorOpt = findMultiCellAnchor(hit->m_placeCoord, footprint,
        [&registry](const glm::ivec3& pos) { return registry.count(pos) > 0; });

    if (!anchorOpt) { hideGhost(); return; }
    const glm::ivec3 anchorCoord = *anchorOpt;

    // If block type switched, destroy the old instance before creating a new one
    if (m_activeGhostGeometry.lock() != geom) hideGhost();
    m_activeGhostGeometry = desiredGeometry;

    int ssboIndex = bestGrid->getGridSSBOIndex();
    if (ssboIndex != m_currentGhostSsboIndex) {
        if (auto inst = m_ghostInstance.lock()) geom->removeInstance(inst);
        m_ghostInstance.reset();
        m_currentGhostSsboIndex = -1;
    }

    const glm::dvec4 ghostColor{ghostTint.r, ghostTint.g, ghostTint.b, k_ghostTransparency};

    if (!m_ghostInstance.lock()) {
        m_ghostInstance = geom->addInstance(ssboIndex, colorTexUnit, -1, -1, ghostColor, -1);
        m_currentGhostSsboIndex = ssboIndex;
    }

    auto inst = m_ghostInstance.lock();
    if (!inst) return;

    inst->m_localPosition    = glm::dvec3{anchorCoord} + modelCentre
                               - glm::dvec3{m_renderedOrientation * modelCentre};
    inst->m_localOrientation = m_renderedOrientation;
    inst->m_localScale       = glm::dvec3{1.0, 1.0, 1.0};
    inst->m_color            = ghostColor;
    geom->updateInstanceInBuffer(inst.get());
}

void BuildTool::hideGhost() {
    if (auto inst = m_ghostInstance.lock()) {
        if (auto geom = m_activeGhostGeometry.lock()) {
            geom->removeInstance(inst);
        }
    }
    m_ghostInstance.reset();
    m_activeGhostGeometry.reset();
    m_currentGhostSsboIndex = -1;
}
