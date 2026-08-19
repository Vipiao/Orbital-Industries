// PlanetSurface.h
#pragma once

#include <array>
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
 * constructor takes, the octave table, and the lattice's cell counts -- and so
 * does every function. Change one side alone and the bounds stop describing the
 * surface being drawn, which is what closes the seams between patches.
 *
 * The shading octaves are the exception, and need not agree with anything: they
 * tilt a normal without moving a vertex, and nothing measures a normal.
 *
 * The snippet reads each octave at the mip level its samples resolve; this reads
 * full detail throughout. That is the safe side rather than a drift to fix: a mip
 * averages what full detail holds, so the bounds go on containing the geometry.
 * They agree where it matters anyway, since the snippet coarsens only as its
 * vertices spread, by which point a patch is kilometres across against metres of
 * relief.
 *
 * The snippet splits its widths -- double where it places a vertex, float where
 * it shades one -- because a float cannot hold a body-sized position. Nothing
 * here is under that pressure, so it is double throughout and matches either
 * side of the split.
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
    // The same point before any terrain, which costs no lookup: the projection
    // the snippet's position path folds into cdlodSurfacePoint.
    glm::dvec3 spherePointOf(const glm::dvec3& crudePoint) const;

    // The sphere the crude solid projects onto, before any terrain: the size of
    // the solid a caller builds to subdivide.
    double radius() const { return m_radius; }
    // The highest the surface can reach from the body's centre, since the map is
    // unsigned and the terrain rises off the sphere rather than straddling it.
    double maxRadius() const;

    // The maps the snippet samples, off the field read above.
    std::vector<uint16_t> bakeElevation() const { return m_noise.bake(); }
    std::vector<float> bakeGradient() const { return m_noise.bakeGradient(); }
    int mapResolution() const { return m_noise.config().m_resolution; }

private:
    // Lattice points a cell is bounded by, and so lookups an octave costs.
    static constexpr int k_latticeCorners{4};

    // One lattice point's plane, as the point being shaded stands in it. The
    // snippet's LatticePlane.
    struct LatticePlane {
        glm::dvec2 m_tileCoord{0.0};   // where the point falls in the map, in tiles
        glm::dvec3 m_tangent{0.0};     // the plane's two axes in the body's frame,
        glm::dvec3 m_bitangent{0.0};   // which the gradient comes back expressed in
        double m_weight{0.0};          // how much of the blend this plane carries
    };

    // The cube face a point stands on, and where across it, found once and read
    // by every octave. The snippet's LatticeFrame.
    struct LatticeFrame {
        int m_major{0};                // the axis the face stands out along,
        int m_uAxis{0};                // and the two cut across it
        int m_vAxis{0};
        double m_coordU{0.0};          // where the point falls across the face, in
        double m_coordV{0.0};          // cells of the coarsest octave
        double m_faceSign{1.0};        // which of the pair of faces it is
        double m_metresPerCell{0.0};   // at that same octave
    };

    // The map summed over the first octaveCount layers of the table: zero is the
    // bare sphere, and each further layer adds the same field at a finer scale.
    double elevationAt(const glm::dvec3& crudePoint, int octaveCount) const;
    glm::dvec3 gradientAt(const glm::dvec3& crudePoint, int octaveCount) const;

    // Cells across one half of a cube face, for the layer named. Mirrors
    // octaveCells; cellsExactly is what it rounds, kept apart so the constructor
    // can check that the rounding had nothing to do.
    int octaveCells(int octave) const;
    double cellsExactly(int octave) const;

    // The frame the point stands in. Mirrors latticeFrameOf, and takes the crude
    // point for the same reason: the cell a point falls in is a ratio of two of
    // its components, so the projection onto the sphere cancels out of it.
    LatticeFrame latticeFrameOf(const glm::dvec3& crudePoint) const;

    // The four lattice points the shading point stands between, each with the
    // plane its share of the terrain is read through. Mirrors latticePlanes.
    std::array<LatticePlane, k_latticeCorners> latticePlanes(
        const LatticeFrame& frame, int octave) const;

    // One plane, at full amplitude: the caller scales, having one weighted sum
    // per octave to do it to rather than four.
    double sampleElevation(const glm::dvec2& tileCoord) const;
    glm::dvec2 sampleSlope(const glm::dvec2& tileCoord, double tilesPerMetre) const;

    // Bilinear over the wrapped field, at the texel centres GL samples between,
    // so a lookup here lands where the snippet's lookup does.
    glm::dvec2 texelCoord(const glm::dvec2& tileCoord) const;

    double m_radius{1.0};
    double m_tileSize{1.0};
    double m_relief{0.0};
    // In metres, as the snippet's fieldMean is: the map's own mean, which the
    // blend leans on because its weights sum to more than one.
    double m_fieldMean{0.0};
    TileableNoiseMap m_noise;
};
