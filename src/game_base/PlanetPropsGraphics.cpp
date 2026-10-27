// PlanetPropsGraphics.cpp
#include "PlanetPropsGraphics.h"
#include "src/world/PlanetSurface.h"
#include "src/world/PlanetSurfaceGlsl.h"
#include "graphics/GraphicsEngine.h"
#include "graphics/ShaderProgram.h"
#include <algorithm>
#include <cassert>
#include <functional>
#include <iostream>
#include <stdexcept>
#include <vector>

std::weak_ptr<ScatterSet> PlanetPropsGraphics::createSet(GraphicsEngine* graphics,
                                                         const PropPatchGrid& grid,
                                                         const std::string& surfaceFigures) {
    // Figures written once, the snippet read again on every reload
    const std::string figures{surfaceFigures + propPatchGridGlsl(grid) +
                              "const uint k_propCandidatesPerSide = " +
                              std::to_string(k_candidatesPerSide) + "u;\n\n"};
    const std::weak_ptr<ScatterSet> set{graphics->createScatterSet(setConfig(grid), [figures] {
        return figures + ShaderProgram::loadTextFileFromPath("../src/world/planet_props.glsl");
    })};

    // Beside the binary, for looking at by eye
    if (!graphics->writeScatterImpostorImages(set, ".")) {
        std::cerr << "Could not write the props' impostor images\n";
    }
    return set;
}

ScatterSetConfig PlanetPropsGraphics::setConfig(const PropPatchGrid& grid) {
    // The colour texture's left half is the trunk, its right half the crown. The
    // material texture makes it matte.
    const std::string treeDir{"../media/models/tree/"};
    ScatterKind tree{};
    tree.m_lods = {ScatterLod{treeDir + "tree_lod0.obj", 25.0},
                   ScatterLod{treeDir + "tree_lod1.obj", 50.0}};
    tree.m_colorTexturePath = treeDir + "tree_color.png";
    tree.m_materialTexturePath = treeDir + "tree_material.png";

    // Over the last 300 m trees shrink and sink into the ground by their height,
    // so the forest thins out instead of ending in a line
    ScatterImpostor impostor{};
    impostor.m_maxDistance = 2000.0;
    impostor.m_fadeBandMetres = 300.0;
    impostor.m_fadeOffsetMetres = glm::dvec3{0.0, 0.0, -k_treeHeightMetres};
    // Views about 6 degrees apart; 192 px pictures in 224 = 7 * 32 px cells, clean
    // down to mip 5
    impostor.m_bake.m_viewsPerSide = 16;
    impostor.m_bake.m_viewPixels = 224;
    impostor.m_bake.m_borderPixels = 16;
    tree.m_impostor = impostor;

    ScatterSetConfig config{};
    config.m_kinds = {tree};
    config.m_maxPatches = grid.maxPatchesWithin(keepReach(config.drawDistance()));
    config.m_itemsPerPatch = k_candidatesPerSide * k_candidatesPerSide;
    return config;
}

double PlanetPropsGraphics::keepReach(double rangeMetres) {
    return rangeMetres + k_searchStepMetres + k_rangeMarginMetres;
}

PlanetPropsGraphics::PlanetPropsGraphics(GraphicsEngine* graphics,
                                         std::weak_ptr<ScatterSet> set,
                                         const PropPatchGrid& grid,
                                         std::shared_ptr<const PlanetSurface> surface,
                                         int ssboIndex)
    : m_graphics{graphics}, m_grid{grid}, m_surface{std::move(surface)},
      m_rangeMetres{setConfig(grid).drawDistance()},
      m_maxPatches{setConfig(grid).m_maxPatches} {
    if (!m_graphics || !m_surface) {
        throw std::runtime_error{"PlanetPropsGraphics: graphics and surface must be non-null"};
    }
    m_instance = m_graphics->createScatterInstance(set, ssboIndex);
}

PlanetPropsGraphics::~PlanetPropsGraphics() {
    m_graphics->removeScatterInstance(m_instance);
}

void PlanetPropsGraphics::updateView(const glm::dvec3& cameraLocal) {
    assert(glm::length(cameraLocal) > 0.0 && "the camera is at the planet's centre");
    const double heightAboveGround{
        glm::length(cameraLocal) - glm::length(m_surface->surfacePoint(cameraLocal))};
    const glm::dvec3 direction{glm::normalize(cameraLocal)};

    if (heightAboveGround > keepReach(m_rangeMetres)) {
        removeAll();
        return;
    }

    const bool addable{heightAboveGround <= m_rangeMetres};
    const bool moved{!m_searchDirection ||
                     m_surface->getRadius() * glm::length(direction - *m_searchDirection) >=
                         k_searchStepMetres};
    if (moved || addable != m_searchAddable) {
        search(direction, addable);
    }
    addPending();
}

// Distances run along the sphere and leave out the camera's height, so they err
// on keeping a patch too long, never too short.
void PlanetPropsGraphics::search(const glm::dvec3& direction, bool addable) {
    m_searchDirection = direction;
    m_searchAddable = addable;

    const double keep{keepReach(m_rangeMetres)};
    for (auto live{m_livePatches.begin()}; live != m_livePatches.end();) {
        if (m_grid.nearestDistance(live->first, direction) > keep) {
            m_graphics->removeScatterPatch(live->second);
            live = m_livePatches.erase(live);
        } else {
            ++live;
        }
    }

    m_pendingPatches.clear();
    if (!addable) {
        return;
    }
    for (const PropPatchKey& key :
         m_grid.patchesNear(direction, m_rangeMetres + k_searchStepMetres)) {
        if (!m_livePatches.contains(key)) {
            m_pendingPatches.emplace_back(m_grid.nearestDistance(key, direction), key);
        }
    }
    // Furthest first; ties by key, so the order is deterministic
    std::sort(m_pendingPatches.begin(), m_pendingPatches.end(),
              std::greater<std::pair<double, PropPatchKey>>{});
}

void PlanetPropsGraphics::addPending() {
    for (uint32_t added{0}; added < k_patchesPerFrame && !m_pendingPatches.empty(); ++added) {
        // Sized to never happen; guards against a throw from addScatterPatch
        assert(m_livePatches.size() < m_maxPatches && "the set's patch room was undersized");
        if (m_livePatches.size() >= m_maxPatches) {
            m_pendingPatches.clear();
            return;
        }

        const PropPatchKey key{m_pendingPatches.back().second};
        m_pendingPatches.pop_back();
        m_livePatches.emplace(
            key, m_graphics->addScatterPatch(
                     m_instance, m_grid.patchOrigin(key),
                     glm::uvec4{key.m_face, key.m_column, key.m_row, k_seed}));
    }
}

void PlanetPropsGraphics::removeAll() {
    for (const auto& [key, patch] : m_livePatches) {
        m_graphics->removeScatterPatch(patch);
    }
    m_livePatches.clear();
    m_pendingPatches.clear();
    m_searchDirection.reset();
}
