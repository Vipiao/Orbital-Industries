// PlanetSurface.cpp
#include "PlanetSurface.h"
#include <cassert>

namespace {

// How sharply the three projections give way to each other. The snippet's
// k_blendSharpness.
constexpr double k_blendSharpness{6.0};

// The snippet's k_octaves: frequency multiplier, amplitude as a fraction of the
// relief, and a shift in tiles. The same map added up at several scales, sixteen
// to a step, with amplitude running as the reciprocal of frequency so every
// octave carries the same rise over run.
constexpr int k_octaveCount{4};
const glm::dvec4 k_octaves[k_octaveCount]{
    glm::dvec4{0.25, 4.0, 0.0, 0.0}, glm::dvec4{4.0, 1.0 / 4.0, 0.37, 0.71},
    glm::dvec4{64.0, 1.0 / 64.0, 0.61, 0.19},
    glm::dvec4{1024.0, 1.0 / 1024.0, 0.13, 0.44}};

// Octaves the geometry carries and octaves the shading carries. The snippet's
// k_positionOctaves and k_shadingOctaves, and the first of the two is the one
// that must agree: it is the surface the quadtree measures its bounds on.
constexpr int k_positionOctaves{2};
constexpr int k_shadingOctaves{2};

}  // namespace

PlanetSurface::PlanetSurface(double radiusMetres, double tileSizeMetres,
                             double reliefMetres,
                             const TileableNoiseMapConfig& noiseConfig)
    : m_radius{radiusMetres}, m_tileSize{tileSizeMetres}, m_relief{reliefMetres},
      m_noise{noiseConfig} {
    assert(m_radius > 0.0 && "A body with no radius projects every crude point to a point");
    assert(m_tileSize > 0.0 && "A tile of no width repeats infinitely often across the body");
    assert(m_relief >= 0.0 && "Negative relief would sink the terrain into the sphere");
}

glm::dvec3 PlanetSurface::spherePointOf(const glm::dvec3& crudePoint) const {
    return glm::normalize(crudePoint) * m_radius;
}

glm::dvec3 PlanetSurface::triplanarWeights(const glm::dvec3& spherePosition) {
    const glm::dvec3 weights{
        glm::pow(glm::abs(glm::normalize(spherePosition)), glm::dvec3{k_blendSharpness})};

    return weights / (weights.x + weights.y + weights.z);
}

glm::dvec2 PlanetSurface::texelCoord(const glm::dvec2& planeCoord,
                                     const glm::dvec4& octave) const {
    // GL samples a texture at texel centres, which sit half a texel in from the
    // coordinate the uv names, so the half texel comes off before interpolating.
    const double resolution{static_cast<double>(m_noise.config().m_resolution)};

    // Reduced to its tile first, as the snippet's wide fract is: at the finest
    // octave a plane coordinate is millions of tiles from the origin, and the
    // texel index that many tiles out runs past what an int holds.
    glm::dvec2 tiles{planeCoord * (octave.x / m_tileSize)};
    tiles -= glm::floor(tiles);

    return (tiles + glm::dvec2{octave.z, octave.w}) * resolution - 0.5;
}

double PlanetSurface::sampleElevation(const glm::dvec2& planeCoord,
                                      const glm::dvec4& octave) const {
    const glm::dvec2 texel{texelCoord(planeCoord, octave)};
    const glm::dvec2 base{glm::floor(texel)};
    const glm::dvec2 fraction{texel - base};
    const int x{static_cast<int>(base.x)};
    const int y{static_cast<int>(base.y)};

    // Indices wrap in TileableNoiseMap, which is what makes the tile seamless.
    const double lower{
        glm::mix(m_noise.sample(x, y), m_noise.sample(x + 1, y), fraction.x)};
    const double upper{
        glm::mix(m_noise.sample(x, y + 1), m_noise.sample(x + 1, y + 1), fraction.x)};

    // Full amplitude: the caller scales, having one weighted sum per octave to do
    // it to rather than three.
    return glm::mix(lower, upper, fraction.y) * m_relief;
}

glm::dvec2 PlanetSurface::sampleSlope(const glm::dvec2& planeCoord,
                                      const glm::dvec4& octave) const {
    const glm::dvec2 texel{texelCoord(planeCoord, octave)};
    const glm::dvec2 base{glm::floor(texel)};
    const glm::dvec2 fraction{texel - base};
    const int x{static_cast<int>(base.x)};
    const int y{static_cast<int>(base.y)};

    const glm::dvec2 lower{
        glm::mix(m_noise.gradient(x, y), m_noise.gradient(x + 1, y), fraction.x)};
    const glm::dvec2 upper{
        glm::mix(m_noise.gradient(x, y + 1), m_noise.gradient(x + 1, y + 1), fraction.x)};

    // The map holds the gradient per unit of tile, so the same two constants that
    // give the elevation its metres give this its own: rise over run.
    return glm::mix(lower, upper, fraction.y) *
           (m_relief * octave.y * octave.x / m_tileSize);
}

double PlanetSurface::elevationAt(const glm::dvec3& spherePosition, int octaveCount) const {
    // Shared across the octaves, being a property of where on the body the point
    // sits rather than of any one layer.
    const glm::dvec3 weights{triplanarWeights(spherePosition)};

    double elevation{0.0};
    for (int octave{0}; octave < octaveCount; ++octave) {
        elevation +=
            k_octaves[octave].y *
            (weights.x * sampleElevation(glm::dvec2{spherePosition.y, spherePosition.z},
                                         k_octaves[octave]) +
             weights.y * sampleElevation(glm::dvec2{spherePosition.z, spherePosition.x},
                                         k_octaves[octave]) +
             weights.z * sampleElevation(glm::dvec2{spherePosition.x, spherePosition.y},
                                         k_octaves[octave]));
    }

    return elevation;
}

glm::dvec3 PlanetSurface::gradientAt(const glm::dvec3& spherePosition,
                                     int octaveCount) const {
    const glm::dvec3 weights{triplanarWeights(spherePosition)};

    glm::dvec3 gradient{0.0};
    for (int octave{0}; octave < octaveCount; ++octave) {
        const glm::dvec2 slopeX{
            sampleSlope(glm::dvec2{spherePosition.y, spherePosition.z}, k_octaves[octave])};
        const glm::dvec2 slopeY{
            sampleSlope(glm::dvec2{spherePosition.z, spherePosition.x}, k_octaves[octave])};
        const glm::dvec2 slopeZ{
            sampleSlope(glm::dvec2{spherePosition.x, spherePosition.y}, k_octaves[octave])};

        // Each plane's map varies with only two of the three coordinates, so its
        // gradient has a zero in the axis it was projected along.
        gradient += weights.x * glm::dvec3{0.0, slopeX.x, slopeX.y} +
                    weights.y * glm::dvec3{slopeY.y, 0.0, slopeY.x} +
                    weights.z * glm::dvec3{slopeZ.x, slopeZ.y, 0.0};
    }

    return gradient;
}

double PlanetSurface::maxRadius() const {
    // Only what the geometry carries: the shading octaves tilt a normal without
    // moving a vertex, so nothing they add is inside anything.
    double amplitude{0.0};
    for (int octave{0}; octave < k_positionOctaves; ++octave) {
        amplitude += k_octaves[octave].y;
    }

    return m_radius + m_relief * amplitude;
}

glm::dvec3 PlanetSurface::surfacePoint(const glm::dvec3& crudePoint) const {
    const glm::dvec3 spherePosition{spherePointOf(crudePoint)};

    return spherePosition +
           glm::normalize(spherePosition) * elevationAt(spherePosition, k_positionOctaves);
}

glm::dvec3 PlanetSurface::surfaceNormal(const glm::dvec3& crudePoint) const {
    const glm::dvec3 spherePosition{spherePointOf(crudePoint)};
    const glm::dvec3 sphereNormal{glm::normalize(spherePosition)};
    const glm::dvec3 gradient{gradientAt(spherePosition, k_shadingOctaves)};

    // Only the tangential part of the gradient tilts the normal; the radial part
    // moves the point without turning it.
    return glm::normalize(sphereNormal -
                          (gradient - sphereNormal * glm::dot(sphereNormal, gradient)));
}
