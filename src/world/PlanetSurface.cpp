// PlanetSurface.cpp
#include "PlanetSurface.h"
#include <cassert>

namespace {

// How sharply the three projections give way to each other. The snippet's
// k_blendSharpness.
constexpr double k_blendSharpness{6.0};

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

glm::dvec2 PlanetSurface::texelCoord(const glm::dvec2& planeCoord) const {
    // GL samples a texture at texel centres, which sit half a texel in from the
    // coordinate the uv names, so the half texel comes off before interpolating.
    const double resolution{static_cast<double>(m_noise.config().m_resolution)};

    return (planeCoord / m_tileSize) * resolution - 0.5;
}

double PlanetSurface::sampleElevation(const glm::dvec2& planeCoord) const {
    const glm::dvec2 texel{texelCoord(planeCoord)};
    const glm::dvec2 base{glm::floor(texel)};
    const glm::dvec2 fraction{texel - base};
    const int x{static_cast<int>(base.x)};
    const int y{static_cast<int>(base.y)};

    // Indices wrap in TileableNoiseMap, which is what makes the tile seamless.
    const double lower{
        glm::mix(m_noise.sample(x, y), m_noise.sample(x + 1, y), fraction.x)};
    const double upper{
        glm::mix(m_noise.sample(x, y + 1), m_noise.sample(x + 1, y + 1), fraction.x)};

    return glm::mix(lower, upper, fraction.y) * m_relief;
}

glm::dvec2 PlanetSurface::sampleSlope(const glm::dvec2& planeCoord) const {
    const glm::dvec2 texel{texelCoord(planeCoord)};
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
    return glm::mix(lower, upper, fraction.y) * m_relief / m_tileSize;
}

double PlanetSurface::elevationAt(const glm::dvec3& spherePosition) const {
    const glm::dvec3 weights{triplanarWeights(spherePosition)};

    return weights.x * sampleElevation(glm::dvec2{spherePosition.y, spherePosition.z})
         + weights.y * sampleElevation(glm::dvec2{spherePosition.z, spherePosition.x})
         + weights.z * sampleElevation(glm::dvec2{spherePosition.x, spherePosition.y});
}

glm::dvec3 PlanetSurface::gradientAt(const glm::dvec3& spherePosition) const {
    const glm::dvec3 weights{triplanarWeights(spherePosition)};

    const glm::dvec2 slopeX{sampleSlope(glm::dvec2{spherePosition.y, spherePosition.z})};
    const glm::dvec2 slopeY{sampleSlope(glm::dvec2{spherePosition.z, spherePosition.x})};
    const glm::dvec2 slopeZ{sampleSlope(glm::dvec2{spherePosition.x, spherePosition.y})};

    // Each plane's map varies with only two of the three coordinates, so its
    // gradient has a zero in the axis it was projected along.
    return weights.x * glm::dvec3{0.0, slopeX.x, slopeX.y}
         + weights.y * glm::dvec3{slopeY.y, 0.0, slopeY.x}
         + weights.z * glm::dvec3{slopeZ.x, slopeZ.y, 0.0};
}

glm::dvec3 PlanetSurface::surfacePoint(const glm::dvec3& crudePoint) const {
    const glm::dvec3 spherePosition{spherePointOf(crudePoint)};

    return spherePosition + glm::normalize(spherePosition) * elevationAt(spherePosition);
}

glm::dvec3 PlanetSurface::surfaceNormal(const glm::dvec3& crudePoint) const {
    const glm::dvec3 spherePosition{spherePointOf(crudePoint)};
    const glm::dvec3 sphereNormal{glm::normalize(spherePosition)};
    const glm::dvec3 gradient{gradientAt(spherePosition)};

    // Only the tangential part of the gradient tilts the normal; the radial part
    // moves the point without turning it.
    return glm::normalize(sphereNormal -
                          (gradient - sphereNormal * glm::dot(sphereNormal, gradient)));
}
