// BuildTool.cpp
#include "BuildTool.h"
#include "../GameBase.h"
#include "../GridSubsystem.h"
#include "graphics/GraphicsEngine.h"
#include "../RadialMenu.h"
#include "../Grid.h"
#include "../StructuralBlock.h"
#include "../thruster/ThrusterBlock.h"
#include "../cockpit/CockpitBlock.h"
#include "graphics/MeshManager2D/MeshManager2D.h"
#include "graphics/MeshManager2D/GeometryInstance.h"
#include "graphics/instanceHandler/InstanceHandler.h"
#include "graphics/KeyboardHandler.h"
#include <iostream>
#include <optional>
#include <functional>
#include "utils/GridGeometry.h"

// Exponential decay constant for orientation slerp animation (radians/second feel).
static constexpr double k_orientationSlerpRate = 20.0;

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

BuildTool::BuildTool(GameBase* gameBase, RadialMenu* radialMenu, int64_t parentNodeId, double interactionRange)
    : m_gameBase(gameBase), m_radialMenu(radialMenu), m_interactionRange(interactionRange) {

    if (!m_gameBase) {
        throw std::runtime_error("GameBase cannot be null");
    }
    if (!m_radialMenu) {
        throw std::runtime_error("RadialMenu cannot be null");
    }

    auto* ge = m_gameBase->m_graphicsEngine.get();

    // Icon textures
    m_constructionIconTextureIndex = ge->createInstanceTexture("../media/2d_graphics/07_construction_icon.png");
    m_thrusterIconTextureIndex     = ge->createInstanceTexture("../media/2d_graphics/09_thruster_icon.png");
    m_cockpitIconTextureIndex      = ge->createInstanceTexture("../media/2d_graphics/10_cockpit_icon.png");

    // Thruster ghost
    m_thrusterGhostColorTextureUnit = ge->createInstanceTexture("../media/models/thruster/albedo_ghost.png");
    m_thrusterGhostGeometry = ge->createInstanceGeometry("../media/models/thruster/thruster_ghost.obj");
    if (auto geom = m_thrusterGhostGeometry.lock()) geom->setAlphaBlending(true);

    // Cockpit ghost
    m_cockpitGhostColorTextureUnit = ge->createInstanceTexture("../media/models/cockpit/albedo_ghost.png");
    m_cockpitGhostGeometry = ge->createInstanceGeometry("../media/models/cockpit/model_ghost.obj");
    if (auto geom = m_cockpitGhostGeometry.lock()) geom->setAlphaBlending(true);

    // Calculate crosshair offset and scale
    m_crosshairScale = glm::dvec2(0.1, 0.1);
    // 16x16 pixels of a 64x64 image where the construction icon center is located
    m_crosshairOffset.x = 2.0 * (0.5 - 16.0 / 64.0) * m_crosshairScale.x;
    m_crosshairOffset.y = 2.0 * (0.5 - 16.0 / 64.0) * m_crosshairScale.y;

    createMenuStructure(parentNodeId);

    m_buildCrosshairGeometry = m_gameBase->m_graphicsEngine->getMeshManager2D()->loadMesh(
        "../media/blender/03_face.obj", "../media/2d_graphics/07_construction_icon.png", -1, true);
}

BuildTool::~BuildTool() {
    hideGhost();

    if (auto instance = m_buildCrosshairInstance.lock()) {
        if (auto geometry = m_buildCrosshairGeometry.lock()) {
            geometry->removeInstance(instance.get());
        }
    }
}

void BuildTool::activate() {
    m_active = true;

    if (!m_buildCrosshairInstance.lock() && m_buildCrosshairGeometry.lock()) {
        m_buildCrosshairInstance = m_buildCrosshairGeometry.lock()->createInstance();
        if (auto instance = m_buildCrosshairInstance.lock()) {
            glm::vec2 position(static_cast<float>(m_crosshairOffset.x),
                               static_cast<float>(-m_crosshairOffset.y));
            instance->setPosition(position);
            instance->setScale(glm::vec2(static_cast<float>(m_crosshairScale.x),
                                         static_cast<float>(m_crosshairScale.y)));
            instance->setColor(glm::dvec4(1.0, 1.0, 1.0, m_buildCrosshairTransparency));
        }
    }
}

void BuildTool::deactivate() {
    m_active = false;
    hideGhost();

    if (auto instance = m_buildCrosshairInstance.lock()) {
        if (auto geometry = m_buildCrosshairGeometry.lock()) {
            geometry->removeInstance(instance.get());
            m_buildCrosshairInstance.reset();
        }
    }
}

void BuildTool::preRenderCallback(bool doCreate, bool doRemove) {
    if (!m_active) {
        return;
    }

    if (doCreate) m_doCreate = true;
    if (doRemove) m_doRemove = true;

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
    int    frameRate  = m_gameBase->m_graphicsEngine->getFrameRate();
    double deltaTime  = 1.0 / static_cast<double>(frameRate > 0 ? frameRate : 60);
    double slerpAlpha = 1.0 - glm::exp(-k_orientationSlerpRate * deltaTime);
    m_renderedOrientation = glm::normalize(
        glm::slerp(m_renderedOrientation, m_targetOrientation, slerpAlpha));

    // --- Update ghost preview every render frame ---
    // Collect available grids for ghost raycasting.
    // We re-use the same grids the physics callback uses, but gathered here for render-time use.
    // GameBase exposes grids via the grid subsystem.
    std::vector<std::weak_ptr<Grid>> gridsForGhost;
    for (const auto& g : m_gameBase->getGridSubsystem()->getGrids()) {
        if (g) gridsForGhost.push_back(g);
    }
    updateGhost(gridsForGhost);
}

void BuildTool::onPhysicsUpdateComplete(const std::vector<std::weak_ptr<Grid>>& availableGrids) {
    if (!m_active) {
        return;
    }

    if (!m_doCreate && !m_doRemove) {
        return;
    }

    // Perform ray casting against all grids
    std::weak_ptr<Grid> targetGridWeak;
    glm::ivec3 targetPos;
    glm::ivec3 hitPos;
    bool   blockFound = false;
    double closestT   = -1.0;

    glm::dvec3 startPos = m_gameBase->m_graphicsEngine->getCamPos();
    glm::dvec3 forward  = m_gameBase->m_graphicsEngine->getCamOri() * glm::dvec3(0.0, 1.0, 0.0);
    glm::dvec3 endPos   = startPos + forward * m_interactionRange;

    auto [_, timeRemainder] = m_gameBase->m_graphicsEngine->getRenderParameters();

    for (const auto& gridWeak : availableGrids) {
        auto gridShared = gridWeak.lock();
        if (!gridShared) continue;

        glm::dvec3 interpolatedPos;
        glm::dquat interpolatedOri;
        gridShared->getInterpolatedTransform(timeRemainder, interpolatedPos, interpolatedOri);

        glm::dvec3 gridLocalRayStart = GridGeometry::worldToGrid(
            startPos, interpolatedPos, interpolatedOri, gridShared->m_centerOfMass);
        glm::dvec3 gridLocalRayEnd = GridGeometry::worldToGrid(
            endPos, interpolatedPos, interpolatedOri, gridShared->m_centerOfMass);

        RayIntersectionResult result = gridShared->intersectRay(gridLocalRayStart, gridLocalRayEnd);

        if (result.t >= 0.0 && (!blockFound || result.t < closestT)) {
            closestT   = result.t;
            blockFound = true;
            targetGridWeak = gridWeak;

            const double epsilon = 1e-6;
            double adjustedT = result.t + epsilon;
            glm::dvec3 intersectionPoint = gridLocalRayStart + adjustedT * (gridLocalRayEnd - gridLocalRayStart);
            hitPos = glm::ivec3(glm::floor(intersectionPoint));

            glm::dvec3 absNormal = glm::abs(result.surfaceNormal);
            glm::ivec3 dominantAxis;
            if (absNormal.x >= absNormal.y && absNormal.x >= absNormal.z)
                dominantAxis = {result.surfaceNormal.x > 0 ? 1 : -1, 0, 0};
            else if (absNormal.y >= absNormal.z)
                dominantAxis = {0, result.surfaceNormal.y > 0 ? 1 : -1, 0};
            else
                dominantAxis = {0, 0, result.surfaceNormal.z > 0 ? 1 : -1};

            targetPos = hitPos + dominantAxis;
        }
    }

    if (m_doCreate) {
        if (blockFound) {
            auto targetGrid = targetGridWeak.lock();
            if (targetGrid) {
                addGridBlock(targetGrid.get(), targetPos.x, targetPos.y, targetPos.z);
            }
        } else {
            glm::dvec3 newGridPos = startPos + forward * 2.0 - glm::dvec3{0.5};
            auto newGridWeak = m_gameBase->createGrid(newGridPos);
            Grid* newGrid = newGridWeak.lock().get();
            addGridBlock(newGrid, 0, 0, 0);
        }
    }

    if (m_doRemove) {
        if (blockFound) {
            auto targetGrid = targetGridWeak.lock();
            if (targetGrid) {
                auto removedCoords = removeGridBlock(targetGrid.get(), hitPos.x, hitPos.y, hitPos.z);
                handleGridSplitting(targetGridWeak, removedCoords);
                if (targetGrid->isEmpty()) {
                    m_gameBase->removeGrid(targetGridWeak);
                }
            }
        }
    }

    m_doCreate = false;
    m_doRemove = false;
}

void BuildTool::createMenuStructure(int64_t parentNodeId) {
    auto activateCallback   = [this]() { activate(); };
    auto deactivateCallback = [this]() { deactivate(); };

    m_buildToolParentId = m_radialMenu->createNode(
        parentNodeId, m_constructionIconTextureIndex, activateCallback, deactivateCallback);

    m_centerNodeId = m_radialMenu->createNode(
        m_buildToolParentId, -1, activateCallback, deactivateCallback);

    m_radialMenu->createNode(m_buildToolParentId, m_constructionIconTextureIndex,
        [this]() { m_selectedBlockType = BlockType::STRUCTURAL_BLOCK; activate(); },
        deactivateCallback);

    m_radialMenu->createNode(m_buildToolParentId, m_thrusterIconTextureIndex,
        [this]() { m_selectedBlockType = BlockType::THRUSTER; activate(); },
        deactivateCallback);

    m_radialMenu->createNode(m_buildToolParentId, m_cockpitIconTextureIndex,
        [this]() { m_selectedBlockType = BlockType::COCKPIT; activate(); },
        deactivateCallback);
}

void BuildTool::addGridBlock(Grid* grid, int x, int y, int z) {
    if (!grid) return;
    const glm::ivec3 targetPos{x, y, z};
    const auto& registry = grid->getCellRegistry();

    if (m_selectedBlockType == BlockType::STRUCTURAL_BLOCK) {
        grid->addCell(targetPos);
    } else if (m_selectedBlockType == BlockType::THRUSTER) {
        auto anchor = findMultiCellAnchor(
            targetPos, ThrusterBlock::footprintOffsets(m_targetOrientation),
            [&registry](const glm::ivec3& pos) { return registry.count(pos) > 0; });
        if (anchor) grid->addThruster(*anchor, m_targetOrientation);
    } else if (m_selectedBlockType == BlockType::COCKPIT) {
        auto anchor = findMultiCellAnchor(
            targetPos, CockpitBlock::footprintOffsets(m_targetOrientation),
            [&registry](const glm::ivec3& pos) { return registry.count(pos) > 0; });
        if (anchor) grid->addCockpit(*anchor, m_targetOrientation);
    }
}

std::vector<glm::ivec3> BuildTool::removeGridBlock(Grid* grid, int x, int y, int z) {
    if (!grid) return {};
    return grid->removeCell(glm::ivec3(x, y, z));
}

void BuildTool::handleGridSplitting(std::weak_ptr<Grid> targetGrid,
                                     const std::vector<glm::ivec3>& removedCoords) {
    static const glm::ivec3 directions[6] = {
        {1, 0, 0}, {-1, 0, 0}, {0, 1, 0}, {0, -1, 0}, {0, 0, 1}, {0, 0, -1}
    };

    std::vector<glm::ivec3> edgeCoords;
    edgeCoords.reserve(removedCoords.size() * 6);
    for (const auto& pos : removedCoords) {
        for (const auto& dir : directions) {
            edgeCoords.push_back(pos + dir);
        }
    }

    m_gameBase->scheduleGridSplitCheck(targetGrid, edgeCoords);
}

void BuildTool::updateGhost(const std::vector<std::weak_ptr<Grid>>& availableGrids) {
    if (m_selectedBlockType != BlockType::THRUSTER && m_selectedBlockType != BlockType::COCKPIT) {
        hideGhost();
        return;
    }

    std::weak_ptr<Geometry> desiredGeometry;
    int colorTexUnit = -1;
    std::vector<glm::ivec3> footprint;
    glm::dvec3 modelCentre;

    if (m_selectedBlockType == BlockType::THRUSTER) {
        desiredGeometry = m_thrusterGhostGeometry;
        colorTexUnit    = m_thrusterGhostColorTextureUnit;
        footprint       = ThrusterBlock::footprintOffsets(m_targetOrientation);
        modelCentre     = ThrusterBlock::MODEL_CENTRE;
    } else {
        desiredGeometry = m_cockpitGhostGeometry;
        colorTexUnit    = m_cockpitGhostColorTextureUnit;
        footprint       = CockpitBlock::footprintOffsets(m_targetOrientation);
        modelCentre     = CockpitBlock::MODEL_CENTRE;
    }

    auto geom = desiredGeometry.lock();
    if (!geom) return;

    auto [_, timeRemainder] = m_gameBase->m_graphicsEngine->getRenderParameters();
    glm::dvec3 camPos  = m_gameBase->m_graphicsEngine->getCamPos();
    glm::dvec3 forward = m_gameBase->m_graphicsEngine->getCamOri() * glm::dvec3(0.0, 1.0, 0.0);
    glm::dvec3 endPos  = camPos + forward * m_interactionRange;

    std::weak_ptr<Grid> bestGridWeak;
    glm::ivec3 targetPos;
    bool   hitFound = false;
    double closestT = -1.0;

    for (const auto& gridWeak : availableGrids) {
        auto grid = gridWeak.lock();
        if (!grid) continue;

        glm::dvec3 interpolatedPos;
        glm::dquat interpolatedOri;
        grid->getInterpolatedTransform(timeRemainder, interpolatedPos, interpolatedOri);

        glm::dvec3 localStart = GridGeometry::worldToGrid(
            camPos, interpolatedPos, interpolatedOri, grid->m_centerOfMass);
        glm::dvec3 localEnd = GridGeometry::worldToGrid(
            endPos, interpolatedPos, interpolatedOri, grid->m_centerOfMass);

        RayIntersectionResult result = grid->intersectRay(localStart, localEnd);
        if (result.t < 0.0 || (hitFound && result.t >= closestT)) continue;

        closestT = result.t;
        hitFound = true;
        bestGridWeak = gridWeak;

        const double epsilon = 1e-6;
        glm::dvec3 intersection = localStart + (result.t + epsilon) * (localEnd - localStart);
        glm::ivec3 hitPos = glm::ivec3(glm::floor(intersection));

        glm::dvec3 absNormal = glm::abs(result.surfaceNormal);
        glm::ivec3 surfaceAxis;
        if (absNormal.x >= absNormal.y && absNormal.x >= absNormal.z)
            surfaceAxis = {result.surfaceNormal.x > 0 ? 1 : -1, 0, 0};
        else if (absNormal.y >= absNormal.z)
            surfaceAxis = {0, result.surfaceNormal.y > 0 ? 1 : -1, 0};
        else
            surfaceAxis = {0, 0, result.surfaceNormal.z > 0 ? 1 : -1};

        targetPos = hitPos + surfaceAxis;
    }

    if (!hitFound) { hideGhost(); return; }

    auto bestGrid = bestGridWeak.lock();
    if (!bestGrid) { hideGhost(); return; }

    const auto& registry = bestGrid->getCellRegistry();
    auto anchorOpt = findMultiCellAnchor(targetPos, footprint,
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

    if (!m_ghostInstance.lock()) {
        m_ghostInstance = geom->addInstance(ssboIndex, colorTexUnit, -1, -1,
                                            glm::dvec4{1.0, 1.0, 1.0, 0.45}, -1);
        m_currentGhostSsboIndex = ssboIndex;
    }

    auto inst = m_ghostInstance.lock();
    if (!inst) return;

    inst->m_localPosition    = glm::dvec3{anchorCoord} + modelCentre
                               - glm::dvec3{m_renderedOrientation * modelCentre};
    inst->m_localOrientation = m_renderedOrientation;
    inst->m_localScale       = glm::dvec3{1.0, 1.0, 1.0};
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
