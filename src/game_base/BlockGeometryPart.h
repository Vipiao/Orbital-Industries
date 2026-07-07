// BlockGeometryPart.h
#pragma once

#include <string>

// One renderable sub-mesh of a block. All parts of a block share its textures,
// but each carries its own geometry and opacity, so an opaque frame and a
// translucent glass canopy can belong to the same block. alpha < 1.0 draws the
// part in the transparent (OIT) pass and tints its instance colour.
struct BlockGeometryPart {
    std::string geometryPath;
    double      alpha{1.0};
};
