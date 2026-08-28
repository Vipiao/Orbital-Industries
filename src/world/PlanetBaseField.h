// PlanetBaseField.h
#pragma once

#include <array>
#include <cstdint>
#include <vector>
#include <glm/glm.hpp>

/**
 * @brief How the base layer is generated: octaves, seed, and the size of the
 *        maps it is read back through.
 */
struct PlanetBaseLayerConfig {
    // Texels along one side of a cube face, powers of two so each map has a full
    // mip chain to retire into as the samples spread past it.
    //
    // The two need not match, and do not. Neither is anywhere near what the
    // field would require -- its finest cells are hundreds of kilometres wide
    // and both maps put tens of texels across one -- so what sizes the elevation
    // map is the filter rather than the field: the hardware blends two texels at
    // 256 steps, so the terrain sits on 256 terraces per texel and their height
    // is the gap between neighbouring texels over 256. Halving the texel halves
    // the terrace. The slope map has no such pressure, a two-hundred-and-fifty-
    // sixth of a slope being invisible, so it stays small and saves the memory
    // three channels of float would otherwise cost.
    int m_elevationResolution{2048};
    int m_slopeResolution{1024};
    // Metres between the layer's floor and its ceiling, as the relief is for the
    // map: the field is unsigned, so this rises off the sphere.
    //
    // Nothing like the octaves' rise over run, and deliberately: this layer is
    // hundreds of times wider than octave zero and only tens of times taller, so
    // it is far the gentlest slope on the body and the quadtree's ranges, which
    // answer for the steepest, never see it.
    double m_reliefMetres{10000.0};
    int m_octaveCount{5};
    // Noise cells across one unit of direction at octave zero. A cell is the
    // radius over this, so two puts the coarsest features a few thousand
    // kilometres wide and the finest, four octaves up, a couple of hundred.
    int m_baseFrequency{2};
    double m_gain{0.5};
    std::uint64_t m_seed{20260826};
};

/**
 * @brief The two maps the base layer is read back through.
 *
 * A pass apiece, the two being different sizes. The field hands back a value and
 * a slope together, so one pass would serve both at one resolution; two passes
 * cost a quarter more evaluations and save three quarters of the slope map.
 */
struct PlanetBaseMaps {
    static constexpr int k_faceCount{6};

    // Cube faces in GL's order, row major, the field quantized to 16-bit unorm.
    // Fixed point rather than half float for the reason the tileable map is:
    // the field is bounded, so uniform absolute precision beats a mantissa that
    // coarsens with magnitude.
    std::array<std::vector<std::uint16_t>, k_faceCount> m_elevation;
    // The same faces, three floats per texel: the slope across the sphere in the
    // body's own frame, per unit of direction.
    //
    // Three rather than the two a slope has degrees of freedom, because there is
    // no pair of axes to spend them in that survives a face boundary. A face's
    // own axes turn a right angle across an edge, and the filter blends across
    // edges, so a pair stored that way would be blended against a different
    // frame. Vectors in the body's frame on either side of an edge are very
    // nearly coplanar and blend correctly.
    std::array<std::vector<float>, k_faceCount> m_slope;
};

/**
 * @brief The lowest layer of the terrain: what a body is shaped like at its own
 *        size, before any of the octaves lay detail on it.
 *
 * A solid noise field read straight off a direction rather than a map tiled over
 * the sphere. One pass covers the body once, so there is nothing to tile and the
 * lattice that hides the octaves' repeat has nothing left to do.
 *
 * Dimensionless: no radius, no tile size, no metres. It hands back a fraction of
 * its own range and a slope per unit of direction, and the caller gives those a
 * size.
 *
 * The one layer with no twin in the snippet. This evaluates the field, the
 * snippet reads the bake of it, and they part only by what bilinear costs across
 * a texel of a smooth field -- a drift rather than a step, there being no hash
 * down here to make one bit of disagreement a different answer.
 */
class PlanetBaseField {
public:
    explicit PlanetBaseField(const PlanetBaseLayerConfig& config);

    // The field in the direction given, in [0, 1], and how it changes across the
    // sphere there, per unit of direction. The maps hold exactly this pair, so a
    // lookup and this agree before either is given metres.
    double sample(const glm::dvec3& direction, glm::dvec3& slope) const;

    // Both maps, one pass each, six faces at a time.
    PlanetBaseMaps bake() const;

    int elevationResolution() const { return m_config.m_elevationResolution; }
    int slopeResolution() const { return m_config.m_slopeResolution; }
    // What the maps are dimensionless against, and so what a reader needs
    // alongside them to give either one metres.
    double relief() const { return m_config.m_reliefMetres; }

private:
    // One face of each map, written in place. Split out because the faces are
    // baked at once: they share nothing, every texel being its own evaluation of
    // the field into its own place in its own face.
    void bakeElevationFace(int face, std::vector<std::uint16_t>& elevation) const;
    void bakeSlopeFace(int face, std::vector<float>& slope) const;

    PlanetBaseLayerConfig m_config{};
};
