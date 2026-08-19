// PlanetSurface.cpp
#include "PlanetSurface.h"
#include <cassert>

namespace {

// The snippet's k_octaves: frequency multiplier, amplitude as a fraction of the
// relief, and a shift in tiles. The same map added up at several scales, sixteen
// to a step, with amplitude running as the reciprocal of frequency so every
// octave carries the same rise over run.
constexpr int k_octaveCount{4};
const glm::dvec4 k_octaves[k_octaveCount]{
    glm::dvec4{0.25, 4.0, 0.0, 0.0}, glm::dvec4{4.0, 1.0 / 4.0, 0.37, 0.71},
    glm::dvec4{64.0, 1.0 / 64.0, 0.61, 0.19},
    glm::dvec4{1024.0, 1.0 / 1024.0, 0.13, 0.44}};

// Tiles of an octave's own layer that one lattice cell spans, so the lattice
// takes its size from the layer it carries. The snippet's k_cellTiles.
constexpr double k_cellTiles{2.0};

// Most a layer can reach above its own amplitude once the blend has divided by
// the root of its weights' squares. That root is smallest at a cell's centre,
// where all four weights stand equal and it comes to a half. Only maxRadius
// needs it, and only to leave room above the terrain; it is a ceiling, not a
// scale, and nothing the surface is drawn from reads it.
constexpr double k_blendCeiling{2.0};

// A lattice point's own random numbers, drawn from its place on the cube. The
// snippet's latticeHash, and the one function here that must match it bit for
// bit rather than merely closely: one bit apart is a different plane. Both sides
// run the same multiply-xor chain over the same 32-bit wrap-around arithmetic.
std::uint32_t latticeHash(const glm::ivec3& point) {
    std::uint32_t h{static_cast<std::uint32_t>(point.x) * 0x9E3779B1u};
    h = (h ^ static_cast<std::uint32_t>(point.y)) * 0x85EBCA6Bu;
    h = (h ^ static_cast<std::uint32_t>(point.z)) * 0xC2B2AE35u;

    h ^= h >> 16;
    h *= 0x85EBCA6Bu;
    h ^= h >> 13;
    h *= 0xC2B2AE35u;
    h ^= h >> 16;

    return h;
}

// Octaves the geometry carries and octaves the shading carries. The snippet's
// k_positionOctaves and k_shadingOctaves, and the first of the two is the one
// that must agree: it is the surface the quadtree measures its bounds on.
constexpr int k_positionOctaves{2};
constexpr int k_shadingOctaves{3};

static_assert(k_positionOctaves > 0 && k_positionOctaves <= k_octaveCount,
              "The geometry cannot carry octaves the table does not hold");
static_assert(k_shadingOctaves > 0 && k_shadingOctaves <= k_octaveCount,
              "The shading cannot carry octaves the table does not hold");

// Most a lattice coordinate may reach before a float stops naming whole numbers.
// The snippet holds cell counts in a float and builds lattice points out of
// them; past this the two sides stop landing on the same point.
constexpr double k_exactWhole{16777216.0};   // 2^24

}  // namespace

PlanetSurface::PlanetSurface(double radiusMetres, double tileSizeMetres,
                             double reliefMetres,
                             const TileableNoiseMapConfig& noiseConfig)
    : m_radius{radiusMetres}, m_tileSize{tileSizeMetres}, m_relief{reliefMetres},
      m_noise{noiseConfig} {
    assert(m_radius > 0.0 && "A body with no radius projects every crude point to a point");
    assert(m_tileSize > 0.0 && "A tile of no width repeats infinitely often across the body");
    assert(m_relief >= 0.0 && "Negative relief would sink the terrain into the sphere");

    // The snippet reaches this same count through a float. Both land on the
    // whole number itself only while the constants leave one to land on; between
    // two, the two widths are free to round apart and take the lattice with them.
    for (int octave{0}; octave < k_octaveCount; ++octave) {
        const double cells{cellsExactly(octave)};
        assert(glm::abs(cells - glm::round(cells)) < 1e-6 &&
               "The radius, tile size and cell span must leave a whole cell count");
        assert(cells > 1.0 && "A face split no finer than one cell has no lattice");
        assert(cells < k_exactWhole &&
               "A cell count past a float's whole numbers parts the two sides");

        // Every octave reaches its own lattice by scaling the coarsest one's
        // coordinate, which is a shift of exponents and exact only while the step
        // between them is a power of two. Off it the snippet's floats round where
        // these doubles do not, and the two sides read different lattices.
        const int ratio{octaveCells(octave) / octaveCells(0)};
        assert(octaveCells(octave) % octaveCells(0) == 0 && ratio > 0 &&
               (ratio & (ratio - 1)) == 0 &&
               "Each octave's cell count must be a power of two times the coarsest");
    }

    // mapTopLevel reads the field's mean off a 1x1 top, which a chain only
    // reaches from a power of two.
    const int resolution{m_noise.config().m_resolution};
    assert(resolution > 0 && (resolution & (resolution - 1)) == 0 &&
           "The map needs a power of two side to mip down to a single texel");

    // The chain above is the snippet's, and a stray edit to either side would
    // otherwise show up only as terrain that no longer matches its bounds.
    assert(latticeHash(glm::ivec3{416, 176, 416}) == 3387414184u &&
           latticeHash(glm::ivec3{-1, -1, -1}) == 1446123588u &&
           "latticeHash has drifted from the chain the snippet runs");

    // What the snippet reads off the top of the mip chain, where one lookup
    // covers the whole tile. Averaged rather than sampled, there being no mip
    // chain on this side, and taken once because the field never changes.
    double total{0.0};
    for (int y{0}; y < resolution; ++y) {
        for (int x{0}; x < resolution; ++x) {
            total += m_noise.sample(x, y);
        }
    }
    m_fieldMean = total / (static_cast<double>(resolution) * resolution) * m_relief;
    assert(m_fieldMean >= 0.0 && m_fieldMean <= m_relief &&
           "The map spans [0, 1], so its mean lies inside the relief it scales to");
}

int PlanetSurface::octaveCells(int octave) const {
    return static_cast<int>(cellsExactly(octave) + 0.5);
}

double PlanetSurface::cellsExactly(int octave) const {
    return m_radius * k_octaves[octave].x / (k_cellTiles * m_tileSize);
}

glm::dvec3 PlanetSurface::spherePointOf(const glm::dvec3& crudePoint) const {
    return glm::normalize(crudePoint) * m_radius;
}

PlanetSurface::LatticeFrame PlanetSurface::latticeFrameOf(
    const glm::dvec3& crudePoint) const {
    // The point's largest component names the cube face it sits on. Taken by
    // magnitude, which is what leaves a point on a face boundary answering the
    // same from either side.
    const glm::dvec3 magnitude{glm::abs(crudePoint)};

    LatticeFrame frame{};
    frame.m_major = magnitude.x > magnitude.y ? (magnitude.x > magnitude.z ? 0 : 2)
                                              : (magnitude.y > magnitude.z ? 1 : 2);
    frame.m_uAxis = (frame.m_major + 1) % 3;
    frame.m_vAxis = (frame.m_major + 2) % 3;
    frame.m_faceSign = crudePoint[frame.m_major] < 0.0 ? -1.0 : 1.0;

    const double faceSpan{frame.m_faceSign * crudePoint[frame.m_major]};
    assert(faceSpan > 0.0 && "A crude point at the body's centre stands on no face");

    const double cells{static_cast<double>(octaveCells(0))};
    const double cellScale{cells / faceSpan};
    frame.m_coordU = crudePoint[frame.m_uAxis] * cellScale;
    frame.m_coordV = crudePoint[frame.m_vAxis] * cellScale;

    // What turns a fraction of a cell into a distance, measured on the sphere the
    // cells are laid on: the projection pulls a point in by the radius over how
    // far out it stood.
    frame.m_metresPerCell =
        m_radius * (magnitude[frame.m_major] / glm::length(crudePoint)) / cells;

    return frame;
}

std::array<PlanetSurface::LatticePlane, PlanetSurface::k_latticeCorners>
PlanetSurface::latticePlanes(const LatticeFrame& frame, int octave) const {
    assert(octave >= 0 && octave < k_octaveCount && "No such octave in the table");
    const int cells{octaveCells(octave)};

    // Every octave's cell count is the coarsest one's times a power of two, which
    // the constructor checks, so a layer reaches its own lattice by scaling the
    // frame -- exact on both sides, and free of the division that found it.
    const double cellRatio{static_cast<double>(cells) /
                           static_cast<double>(octaveCells(0))};

    const double coordU{frame.m_coordU * cellRatio};
    const double coordV{frame.m_coordV * cellRatio};
    const int cellU{static_cast<int>(glm::floor(coordU))};
    const int cellV{static_cast<int>(glm::floor(coordV))};
    const double fracU{coordU - glm::floor(coordU)};
    const double fracV{coordV - glm::floor(coordV)};
    assert(fracU >= 0.0 && fracU < 1.0 && fracV >= 0.0 && fracV < 1.0 &&
           "A cell fraction outside its cell would weight the wrong four points");

    const double metresPerCell{frame.m_metresPerCell / cellRatio};

    // Smoothed, so the blend arrives at a cell boundary with no slope left in
    // it, the cube's own edges among them.
    const double weightU{glm::smoothstep(0.0, 1.0, fracU)};
    const double weightV{glm::smoothstep(0.0, 1.0, fracV)};

    const glm::dvec2 alongU{1.0 - weightU, weightU};
    const glm::dvec2 alongV{1.0 - weightV, weightV};

    // Four weights summing to one keep only the sum of their squares' worth of
    // the field's variance, what they mix being four unrelated pieces of it.
    // Dividing by the root of that sum instead holds the contrast flat, at the
    // price of weights that no longer sum to one -- which elevationAt answers
    // for and gradientAt need not.
    const double restore{
        1.0 / glm::sqrt(glm::dot(alongU, alongU) * glm::dot(alongV, alongV))};

    const double tilesPerMetre{k_octaves[octave].x / m_tileSize};
    const glm::dvec2 shift{k_octaves[octave].z, k_octaves[octave].w};

    std::array<LatticePlane, k_latticeCorners> planes{};
    for (int corner{0}; corner < k_latticeCorners; ++corner) {
        const int stepU{corner & 1};
        const int stepV{corner >> 1};

        // Named by its place on the cube as three whole numbers, so both faces
        // meeting at an edge name the point there identically.
        glm::ivec3 point{0};
        point[frame.m_major] = static_cast<int>(frame.m_faceSign) * cells;
        point[frame.m_uAxis] = cellU + stepU;
        point[frame.m_vAxis] = cellV + stepV;

        const glm::dvec3 direction{glm::normalize(glm::dvec3{point})};

        // A pair of axes across the plane, built with neither a branch nor a
        // second root: Duff's construction, exact for a unit direction and
        // steady at both poles.
        const double zSign{direction.z >= 0.0 ? 1.0 : -1.0};
        const double spread{-1.0 / (zSign + direction.z)};
        const double cornerTerm{direction.x * direction.y * spread};
        const glm::dvec3 basisU{1.0 + zSign * direction.x * direction.x * spread,
                                zSign * cornerTerm, -zSign * direction.x};
        const glm::dvec3 basisV{cornerTerm, zSign + direction.y * direction.y * spread,
                                -direction.y};

        // A byte apiece. The turn arrives as the direction a pair of them points
        // rather than as an angle, and the odd numerators over an even scale are
        // what keep that pair off the zero that points nowhere.
        const std::uint32_t hash{latticeHash(point)};
        const glm::dvec2 turn{glm::normalize(
            glm::dvec2{static_cast<double>((hash & 0xFFu) * 2u + 1u),
                       static_cast<double>(((hash >> 8) & 0xFFu) * 2u + 1u)} /
                256.0 -
            1.0)};
        const glm::dvec2 turnShift{static_cast<double>((hash >> 16) & 0xFFu) / 256.0,
                                   static_cast<double>((hash >> 24) & 0xFFu) / 256.0};

        // Turned about the lattice point, which together with the shift is what
        // leaves each cell reading the map somewhere else and along some other
        // direction. The axes carry the turn rather than the coordinate, so the
        // gradient comes back in the frame it was taken in.
        const glm::dvec3 tangent{turn.x * basisU - turn.y * basisV};
        const glm::dvec3 bitangent{turn.y * basisU + turn.x * basisV};
        // The plane carries a slope back into the body's frame through these, so
        // a pair that is not orthonormal stretches the terrain it returns.
        assert(glm::abs(glm::dot(tangent, tangent) - 1.0) < 1e-9 &&
               glm::abs(glm::dot(bitangent, bitangent) - 1.0) < 1e-9 &&
               glm::abs(glm::dot(tangent, bitangent)) < 1e-9 &&
               "The turned plane axes must stay orthonormal");

        // From the lattice point to the point being shaded, along the two axes
        // the cell is cut on. What of it stands off the body falls out in the
        // two dot products below.
        glm::dvec3 offset{0.0};
        offset[frame.m_uAxis] = (fracU - static_cast<double>(stepU)) * metresPerCell;
        offset[frame.m_vAxis] = (fracV - static_cast<double>(stepV)) * metresPerCell;

        planes[corner].m_tileCoord =
            glm::dvec2{glm::dot(offset, tangent), glm::dot(offset, bitangent)} *
                tilesPerMetre +
            shift + turnShift;
        planes[corner].m_tangent = tangent;
        planes[corner].m_bitangent = bitangent;
        planes[corner].m_weight = alongU[stepU] * alongV[stepV] * restore;
    }

    return planes;
}

glm::dvec2 PlanetSurface::texelCoord(const glm::dvec2& tileCoord) const {
    // GL samples a texture at texel centres, which sit half a texel in from the
    // coordinate the uv names, so the half texel comes off before interpolating.
    const double resolution{static_cast<double>(m_noise.config().m_resolution)};

    // Reduced to its tile first, as the sampler's own wrap is: the texel index
    // whole tiles out runs past what an int holds.
    const glm::dvec2 tiles{tileCoord - glm::floor(tileCoord)};

    return tiles * resolution - 0.5;
}

double PlanetSurface::sampleElevation(const glm::dvec2& tileCoord) const {
    const glm::dvec2 texel{texelCoord(tileCoord)};
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
    // it to rather than four.
    return glm::mix(lower, upper, fraction.y) * m_relief;
}

glm::dvec2 PlanetSurface::sampleSlope(const glm::dvec2& tileCoord,
                                      double tilesPerMetre) const {
    const glm::dvec2 texel{texelCoord(tileCoord)};
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
    return glm::mix(lower, upper, fraction.y) * (m_relief * tilesPerMetre);
}

double PlanetSurface::elevationAt(const glm::dvec3& crudePoint, int octaveCount) const {
    // The lattice is taken per octave, each layer wanting cells sized to its own
    // tile, so the four planes are found again for every one of them. The face
    // they are found on is the same for all of them, and is found once.
    const LatticeFrame frame{latticeFrameOf(crudePoint)};

    double elevation{0.0};
    for (int octave{0}; octave < octaveCount; ++octave) {
        const std::array<LatticePlane, k_latticeCorners> planes{
            latticePlanes(frame, octave)};

        // The blend's weights sum to more than one, so each layer overshoots by
        // whatever it stands above the map's mean, and that is what the mean is
        // taken back off against.
        double layer{0.0};
        double covered{0.0};
        for (const LatticePlane& plane : planes) {
            layer += plane.m_weight * sampleElevation(plane.m_tileCoord);
            covered += plane.m_weight;
        }

        elevation += k_octaves[octave].y * (layer + m_fieldMean * (1.0 - covered));
    }

    return elevation;
}

glm::dvec3 PlanetSurface::gradientAt(const glm::dvec3& crudePoint,
                                     int octaveCount) const {
    const LatticeFrame frame{latticeFrameOf(crudePoint)};

    glm::dvec3 gradient{0.0};
    for (int octave{0}; octave < octaveCount; ++octave) {
        const std::array<LatticePlane, k_latticeCorners> planes{
            latticePlanes(frame, octave)};
        const double tilesPerMetre{k_octaves[octave].x / m_tileSize};

        // Each plane returns a slope in its own two axes, which those axes carry
        // back into the body's frame.
        glm::dvec3 layer{0.0};
        for (const LatticePlane& plane : planes) {
            const glm::dvec2 slope{sampleSlope(plane.m_tileCoord, tilesPerMetre)};
            layer += plane.m_weight *
                     (slope.x * plane.m_tangent + slope.y * plane.m_bitangent);
        }

        gradient += k_octaves[octave].y * layer;
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

    return m_radius + m_relief * amplitude * k_blendCeiling;
}

glm::dvec3 PlanetSurface::surfacePoint(const glm::dvec3& crudePoint) const {
    // The sphere and the terrain riding on it share one outward direction, so the
    // radius and the height are added before it is scaled by them.
    return glm::normalize(crudePoint) *
           (m_radius + elevationAt(crudePoint, k_positionOctaves));
}

glm::dvec3 PlanetSurface::surfaceNormal(const glm::dvec3& crudePoint) const {
    const glm::dvec3 sphereNormal{glm::normalize(crudePoint)};
    const glm::dvec3 gradient{gradientAt(crudePoint, k_shadingOctaves)};

    // Only the tangential part of the gradient tilts the normal; the radial part
    // moves the point without turning it.
    return glm::normalize(sphereNormal -
                          (gradient - sphereNormal * glm::dot(sphereNormal, gradient)));
}
