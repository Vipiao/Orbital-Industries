// CubeFaces.h
#pragma once

#include <vector>
#include <glm/glm.hpp>
#include "graphics/cdlod/CdlodTree.h"

/**
 * @brief The cube a body is built from, and the only copy of it.
 *
 * A face is origin + u * uAxis + v * vAxis for (u, v) in [0, 1]^2, the body
 * spanning [-1, 1]. uAxis x vAxis points outward, so counter-clockwise in (u, v)
 * is counter-clockwise from outside. The ground's quadtree roots, CubeSphereBounds
 * and PropPatchGrid all read it from here.
 */
class CubeFaces {
public:
    static constexpr int k_faceCount{6};

    // The quadtree roots of a body of radius halfExtent
    static std::vector<CdlodPatchFrame> cubeRootFrames(double halfExtent);

    static const glm::dvec3& faceOrigin(int face) { return k_faceOrigin[face]; }
    static const glm::dvec3& faceUAxis(int face) { return k_faceUAxis[face]; }
    static const glm::dvec3& faceVAxis(int face) { return k_faceVAxis[face]; }

private:
    static const glm::dvec3 k_faceOrigin[k_faceCount];
    static const glm::dvec3 k_faceUAxis[k_faceCount];
    static const glm::dvec3 k_faceVAxis[k_faceCount];
};
