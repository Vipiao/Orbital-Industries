// CubeSphereBounds.cpp
#include "CubeSphereBounds.h"
#include "PlanetSurface.h"
#include <cassert>
#include <utility>

CubeSphereBounds::CubeSphereBounds(std::shared_ptr<const PlanetSurface> surface)
    : m_surface{std::move(surface)} {
    assert(m_surface && "Without a surface a patch has no place, and no distance to measure");
}

CdlodPatchBounds CubeSphereBounds::patchBounds(const CdlodPatchFrame& frame) const {
    // The one elevation lookup, and the only reason the ball tracks the patch
    // rather than the whole body's relief.
    const glm::dvec3 centre{m_surface->surfacePoint(frame.m_centre)};

    // Reach taken on the smooth sphere, and against the smooth centre so the
    // terrain's own offset does not count twice. Projection shrinks a face
    // unevenly, most at the cube's corners, so the frame overstates it.
    const glm::dvec3 smoothCentre{m_surface->spherePointOf(frame.m_centre)};
    double radiusSquared{0.0};
    for (int cornerIndex{0}; cornerIndex < 4; ++cornerIndex) {
        const double uSign{(cornerIndex % 2) == 0 ? -1.0 : 1.0};
        const double vSign{(cornerIndex / 2) == 0 ? -1.0 : 1.0};
        const glm::dvec3 corner{
            frame.m_centre + frame.m_uAxis * uSign + frame.m_vAxis * vSign};
        const glm::dvec3 toCorner{m_surface->spherePointOf(corner) - smoothCentre};
        radiusSquared = glm::max(radiusSquared, glm::dot(toCorner, toCorner));
    }

    return CdlodPatchBounds{centre, glm::sqrt(radiusSquared)};
}
