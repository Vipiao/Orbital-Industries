// CubeSphereBounds.cpp
#include "CubeSphereBounds.h"
#include <cassert>

CubeSphereBounds::CubeSphereBounds(double radius, double reliefMetres)
    : m_radius{radius}, m_relief{reliefMetres} {
    assert(m_radius > 0.0 && "A body with no radius projects every patch to a point");
    assert(m_relief >= 0.0 && "Negative relief would let the surface escape its own bound");
}

CdlodPatchBounds CubeSphereBounds::patchBounds(const CdlodPatchFrame& frame) const {
    const glm::dvec3 centre{glm::normalize(frame.m_centre) * m_radius};

    // Measured from the projected corners, not the frame's own half diagonal:
    // projection shrinks a face unevenly, most at the cube's corners and not at
    // all at its face centres, so the frame overstates a corner patch by twice.
    //
    // The corners suffice because a flat square projects onto a convex piece of
    // sphere, whose furthest points from its centre are its corners.
    double radiusSquared{0.0};
    for (int cornerIndex{0}; cornerIndex < 4; ++cornerIndex) {
        const double uSign{(cornerIndex % 2) == 0 ? -1.0 : 1.0};
        const double vSign{(cornerIndex / 2) == 0 ? -1.0 : 1.0};
        const glm::dvec3 corner{
            frame.m_centre + frame.m_uAxis * uSign + frame.m_vAxis * vSign};
        const glm::dvec3 toCorner{glm::normalize(corner) * m_radius - centre};
        radiusSquared = glm::max(radiusSquared, glm::dot(toCorner, toCorner));
    }

    // The sphere alone bounds only an undisplaced body; the ceiling is what lets
    // the tree measure to a surface it never evaluates.
    return CdlodPatchBounds{centre, glm::sqrt(radiusSquared) + m_relief};
}
