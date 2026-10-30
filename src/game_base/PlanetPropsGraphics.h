// PlanetPropsGraphics.h
#pragma once

#include "src/world/PropPatchGrid.h"
#include <cstdint>
#include <map>
#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>
#include <glm/glm.hpp>

class GraphicsEngine;
class PlanetSurface;
struct ScatterInstance;
struct ScatterPatch;
struct ScatterSet;
struct ScatterSetConfig;

// One planet's props as drawn: a scatter instance of the planet type's set, and
// the prop patches live around the camera.
//
// The search for near patches reruns once the camera has moved
// k_searchStepMetres, and so reaches that much past the draw distance. Found
// patches are added nearest first, k_patchesPerFrame a frame. A patch is removed
// k_rangeMarginMetres past that reach, so a camera on the border does not churn.
class PlanetPropsGraphics {
public:
    // The side of a prop patch in the middle of a cube face
    static constexpr double k_patchMetres{128.0};

    // surfaceFigures are the type's planetSurfaceGlsl. The caller hands the set
    // the ground's maps and removes it.
    static std::weak_ptr<ScatterSet> createSet(GraphicsEngine* graphics,
                                               const PropPatchGrid& grid,
                                               const std::string& surfaceFigures);

    // set and grid as given to createSet
    PlanetPropsGraphics(GraphicsEngine* graphics, std::weak_ptr<ScatterSet> set,
                        const PropPatchGrid& grid,
                        std::shared_ptr<const PlanetSurface> surface, int ssboIndex);
    ~PlanetPropsGraphics();

    PlanetPropsGraphics(const PlanetPropsGraphics&) = delete;
    PlanetPropsGraphics& operator=(const PlanetPropsGraphics&) = delete;

    // Every frame, with the camera in the planet's frame
    void updateView(const glm::dvec3& cameraLocal);

private:
    static constexpr uint32_t k_candidatesPerSide{32};
    // The snippet places candidates in 65536ths of a cell, exact in a float
    static_assert(k_candidatesPerSide <= 256, "candidate positions must stay below 2^24");
    static constexpr double k_searchStepMetres{16.0};
    static constexpr double k_rangeMarginMetres{16.0};
    // The snippet scales each prop by k_propMinScale plus up to k_propScaleSpan
    static constexpr double k_propMinScale{0.7};
    static constexpr double k_propScaleSpan{0.6};
    static constexpr uint32_t k_patchesPerFrame{16};
    // Mixed into every draw, the same on every peer
    static constexpr uint32_t k_seed{1};

    // Room for as many patches as the keep reach can hold, shared by every planet
    // of the type, the camera being near one at a time
    static ScatterSetConfig setConfig(const PropPatchGrid& grid);
    static double keepReach(double rangeMetres);

    // Removes patches past keepReach and lists the missing ones in reach,
    // nearest last; none while addable is false
    void search(const glm::dvec3& direction, bool addable);
    void addPending();
    void removeAll();
    // A sphere holding the patch's ground and trees, from the ground at its corners
    // and middle; widened, since the ground between them may rise further
    std::pair<glm::dvec3, double> patchBounds(const PropPatchKey& key) const;

    GraphicsEngine* m_graphics;
    PropPatchGrid m_grid;
    std::shared_ptr<const PlanetSurface> m_surface;
    std::weak_ptr<ScatterInstance> m_instance;
    double m_rangeMetres;
    uint32_t m_maxPatches;
    std::map<PropPatchKey, std::weak_ptr<ScatterPatch>> m_livePatches;
    // Distance and key, furthest first
    std::vector<std::pair<double, PropPatchKey>> m_pendingPatches;
    // At the last search; empty before the first
    std::optional<glm::dvec3> m_searchDirection;
    bool m_searchAddable{false};
};
