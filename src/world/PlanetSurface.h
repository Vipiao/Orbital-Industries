// PlanetSurface.h
#pragma once

#include <cstdint>
#include <vector>
#include <glm/glm.hpp>
#include "math/TileableNoiseMap.h"

/**
 * @brief The body's shape in C++: a crude point in, the surface out.
 *
 * The twin of media/surfaces/triplanar_noise_surface.glsl, written apart only
 * because the two run in different languages, and it must be kept in step with
 * it. Every constant here appears in the snippet as well -- the three the
 * constructor takes, and the blend sharpness -- and so does every function.
 * Change one side alone and the bounds stop describing the surface being drawn,
 * which is what closes the seams between patches.
 *
 * Owns the noise and bakes the maps the snippet samples from that same field, so
 * the two are reading one generation rather than two.
 *
 * Knows nothing about the quadtree. What subdivides the crude solid, and what
 * needs to know where its squares land, is somebody else's concern.
 */
class PlanetSurface {
public:
    // radiusMetres is the sphere the crude solid projects onto, tileSizeMetres
    // how wide one repeat of the map is laid down, and reliefMetres the floor to
    // ceiling height of the terrain it carries.
    PlanetSurface(double radiusMetres, double tileSizeMetres, double reliefMetres,
                  const TileableNoiseMapConfig& noiseConfig);

    // Where a crude point is drawn. Mirrors cdlodSurfacePoint.
    glm::dvec3 surfacePoint(const glm::dvec3& crudePoint) const;
    // The unit normal there. Mirrors cdlodSurfaceNormal.
    glm::dvec3 surfaceNormal(const glm::dvec3& crudePoint) const;
    // The same point before any terrain, which costs no lookup. Mirrors
    // spherePointOf.
    glm::dvec3 spherePointOf(const glm::dvec3& crudePoint) const;

    // The sphere the crude solid projects onto, before any terrain: the size of
    // the solid a caller builds to subdivide.
    double radius() const { return m_radius; }
    // The highest the surface can reach from the body's centre, since the map is
    // unsigned and the terrain rises off the sphere rather than straddling it.
    double maxRadius() const { return m_radius + m_relief; }

    // The maps the snippet samples, off the field read above.
    std::vector<uint16_t> bakeElevation() const { return m_noise.bake(); }
    std::vector<float> bakeGradient() const { return m_noise.bakeGradient(); }
    int mapResolution() const { return m_noise.config().m_resolution; }

private:
    double elevationAt(const glm::dvec3& spherePosition) const;
    glm::dvec3 gradientAt(const glm::dvec3& spherePosition) const;

    // Weight of each of the three planar projections, summing to one.
    static glm::dvec3 triplanarWeights(const glm::dvec3& spherePosition);

    double sampleElevation(const glm::dvec2& planeCoord) const;
    glm::dvec2 sampleSlope(const glm::dvec2& planeCoord) const;

    // Bilinear over the wrapped field, at the texel centres GL samples between,
    // so a lookup here lands where the snippet's lookup does.
    glm::dvec2 texelCoord(const glm::dvec2& planeCoord) const;

    double m_radius{1.0};
    double m_tileSize{1.0};
    double m_relief{0.0};
    TileableNoiseMap m_noise;
};
