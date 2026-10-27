// PropPatchGrid.h
#pragma once

#include <compare>
#include <cstdint>
#include <vector>
#include <glm/glm.hpp>

struct PropPatchKey {
    uint32_t m_face{0};
    uint32_t m_column{0};
    uint32_t m_row{0};

    auto operator<=>(const PropPatchKey&) const = default;
};

/**
 * @brief The fixed grid of prop patches over the faces of the body's cube
 * (CubeFaces), so a patch is named by three integers on every run and peer.
 *
 * Grid lines are spaced by equal angle from the centre, which keeps every patch
 * within 1.4 times the area of any other (equal steps on the face would give 5x).
 * planet_props.glsl lays out the same grid from propPatchGridGlsl's figures.
 */
class PropPatchGrid {
public:
    // patchMetres is rounded so a whole number of patches spans a face.
    PropPatchGrid(double radiusMetres, double patchMetres);

    uint32_t getPatchesPerEdge() const { return m_patchesPerEdge; }

    // column and row in patches from the face's corner, 0 .. patchesPerEdge
    glm::dvec3 gridDirection(uint32_t face, double column, double row) const;
    // The patch's corner at (column, row), on the sphere
    glm::dvec3 patchOrigin(const PropPatchKey& key) const;

    // Every patch that may lie within reachMetres of direction along the sphere,
    // in key order.
    std::vector<PropPatchKey> patchesNear(const glm::dvec3& direction,
                                          double reachMetres) const;
    // A lower bound on the distance to the patch's nearest point; negative near
    // its middle.
    double nearestDistance(const PropPatchKey& key, const glm::dvec3& direction) const;

    // An upper bound on the patches within reachMetres of any direction: a
    // circle's area over the smallest patch's.
    uint32_t maxPatchesWithin(double reachMetres) const;

private:
    // Middle to furthest corner, in metres
    double halfWidth(const PropPatchKey& key) const;
    // As two flat triangles, in square metres
    double area(const PropPatchKey& key) const;

    double m_radius;
    uint32_t m_patchesPerEdge;
};
