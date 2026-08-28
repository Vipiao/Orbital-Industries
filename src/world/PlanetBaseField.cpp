// PlanetBaseField.cpp
#include "PlanetBaseField.h"
#include "math/PerlinNoise.h"
#include <cassert>
#include <thread>

namespace {

// A direction out of a cube map's face and a place across it, in GL's own
// layout: faces ordered +X, -X, +Y, -Y, +Z, -Z, with s and t running from the
// first texel of the first row.
//
// These are the signs the sampler undoes when it turns a direction back into a
// face and a place, so a map laid out this way is read back where it was
// written. Getting one of them wrong shows up as terrain that is right in
// itself and mirrored or turned against the body it sits on.
glm::dvec3 cubeFaceDirection(int face, double s, double t) {
    const double u{2.0 * s - 1.0};
    const double v{2.0 * t - 1.0};

    switch (face) {
        case 0: return glm::normalize(glm::dvec3{1.0, -v, -u});
        case 1: return glm::normalize(glm::dvec3{-1.0, -v, u});
        case 2: return glm::normalize(glm::dvec3{u, 1.0, v});
        case 3: return glm::normalize(glm::dvec3{u, -1.0, -v});
        case 4: return glm::normalize(glm::dvec3{u, -v, 1.0});
        default: return glm::normalize(glm::dvec3{-u, -v, -1.0});
    }
}

}   // namespace

PlanetBaseField::PlanetBaseField(const PlanetBaseLayerConfig& config) : m_config{config} {
    for (int side : {m_config.m_elevationResolution, m_config.m_slopeResolution}) {
        assert(side > 0 && (side & (side - 1)) == 0 &&
               "A base map needs a power of two side for a full mip chain");
    }
    assert(m_config.m_octaveCount > 0 && "A base layer with no octaves is a bare sphere");
    assert(m_config.m_baseFrequency > 0 && "A field with no cells has nothing to interpolate");
    assert(m_config.m_reliefMetres >= 0.0 &&
           "Negative relief would sink the base layer into the sphere");

    // The face a direction lands on, and which way up it lies there, are the
    // sampler's to decide; this side only writes the map. A stray sign in the
    // layout would draw terrain that is right in itself and wrong on the body,
    // which is hard to see and easy to check: each face's middle texel must look
    // straight out along that face's own axis.
    for (int face{0}; face < PlanetBaseMaps::k_faceCount; ++face) {
        glm::dvec3 axis{0.0};
        axis[face / 2] = (face % 2) == 0 ? 1.0 : -1.0;
        assert(glm::length(cubeFaceDirection(face, 0.5, 0.5) - axis) < 1e-12 &&
               "A cube face's middle no longer looks out along the face's own axis");
    }
}

double PlanetBaseField::sample(const glm::dvec3& direction, glm::dvec3& slope) const {
    // The field is read on the unit sphere and the slope projected along this
    // same vector. Off unit length the field is read at the wrong scale and the
    // slope still runs partly outward.
    assert(glm::abs(glm::length(direction) - 1.0) < 1e-9 &&
           "The base field is read along a unit direction");

    double value{0.0};
    glm::dvec3 gradient{0.0};
    double amplitude{1.0};
    double total{0.0};
    std::int64_t frequency{m_config.m_baseFrequency};

    for (int octave{0}; octave < m_config.m_octaveCount; ++octave) {
        // A period twice the two units a direction can span, so no part of the
        // body ever meets another part's copy of the field.
        glm::dvec3 octaveGradient{0.0};
        value += amplitude *
                 PerlinNoise::sample(direction * static_cast<double>(frequency), 4 * frequency,
                                     m_config.m_seed + static_cast<std::uint64_t>(octave),
                                     octaveGradient);
        // The layer is read at its own frequency, so its gradient comes back per
        // unit of that and has to be carried back to per unit of direction.
        gradient += (amplitude * static_cast<double>(frequency)) * octaveGradient;

        total += amplitude;
        amplitude *= m_config.m_gain;
        frequency *= 2;
    }

    // Perlin straddles zero and the map that holds this is unsigned, so the sum
    // is folded onto [0, 1]: the layer rises off the sphere rather than cutting
    // into it, exactly as the tileable map does.
    const double fold{0.5 / total};

    // Only what runs across the sphere. A step along the direction itself leaves
    // the point the field was read at exactly where it was, so it moves the
    // surface without turning it.
    slope = (gradient - direction * glm::dot(direction, gradient)) * fold;

    return glm::clamp(0.5 + value * fold, 0.0, 1.0);
}

void PlanetBaseField::bakeElevationFace(int face, std::vector<std::uint16_t>& elevation) const {
    const int resolution{m_config.m_elevationResolution};
    const double texels{static_cast<double>(resolution)};
    elevation.resize(static_cast<size_t>(resolution) * resolution);

    for (int y{0}; y < resolution; ++y) {
        for (int x{0}; x < resolution; ++x) {
            // Texel centres, which is where the sampler reads between.
            const glm::dvec3 direction{
                cubeFaceDirection(face, (x + 0.5) / texels, (y + 0.5) / texels)};

            glm::dvec3 unusedSlope{0.0};
            const double field{sample(direction, unusedSlope)};
            elevation[static_cast<size_t>(y) * resolution + x] =
                static_cast<std::uint16_t>(glm::round(field * 65535.0));
        }
    }
}

void PlanetBaseField::bakeSlopeFace(int face, std::vector<float>& slope) const {
    const int resolution{m_config.m_slopeResolution};
    const double texels{static_cast<double>(resolution)};
    slope.resize(static_cast<size_t>(resolution) * resolution * 3);

    for (int y{0}; y < resolution; ++y) {
        for (int x{0}; x < resolution; ++x) {
            const glm::dvec3 direction{
                cubeFaceDirection(face, (x + 0.5) / texels, (y + 0.5) / texels)};

            glm::dvec3 across{0.0};
            sample(direction, across);

            const size_t texel{static_cast<size_t>(y) * resolution + x};
            for (int axis{0}; axis < 3; ++axis) {
                slope[texel * 3 + axis] = static_cast<float>(across[axis]);
            }
        }
    }
}

PlanetBaseMaps PlanetBaseField::bake() const {
    PlanetBaseMaps maps{};

    // A face apiece. Every texel is its own evaluation of the field written to
    // its own place, so the maps come out the same however the threads
    // interleave -- the field is a function of a direction and of nothing else.
    std::array<std::thread, PlanetBaseMaps::k_faceCount> workers{};
    for (int face{0}; face < PlanetBaseMaps::k_faceCount; ++face) {
        workers[face] = std::thread{[this, face, &maps] {
            bakeElevationFace(face, maps.m_elevation[face]);
            bakeSlopeFace(face, maps.m_slope[face]);
        }};
    }
    for (std::thread& worker : workers) {
        worker.join();
    }

    return maps;
}
