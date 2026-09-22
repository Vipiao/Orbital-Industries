// TerrainDisplacement.h
#pragma once

#include <array>
#include <glm/glm.hpp>

/**
 * @brief What the terrain is: how many layers it has, how tall each stands, and
 * how they add up.
 *
 * Where the numbers live. terrain_displacement.glsl runs the same arithmetic on
 * the GPU but holds none of the figures: planetSurfaceGlsl spells these out for
 * it to read, so the table is here once rather than on both sides waiting to be
 * edited apart. The two still have to agree about what to do with them, and a
 * change to the sum belongs in both.
 *
 * Nothing here knows where a reading came from. The caller takes the lookups and
 * hands them over dimensionless; this gives them size. Only the frequency column
 * is exported, because the caller reads the map at its own scale; everything that
 * turns a reading into metres happens here.
 *
 * Height and gradient come back together, being one surface measured two ways.
 * The gradient returned is the derivative of the height returned rather than a
 * second opinion about it: every synthesis differentiates its own arithmetic, so
 * one that bends a reading bends the slope with it.
 */
class TerrainDisplacement {
public:
    // How the layers are put together. The table is the same either way, a
    // frequency and an amplitude per layer; what differs is what a reading is
    // worth once the layers above it are in hand.
    enum class Synthesis {
        // Each layer at its own amplitude, added. Every layer reads the map
        // as it is and none of them knows what the others found.
        FBM,
        // The same sum read the other way up. The map creases along its own
        // zero contour and stands at nothing there, so turning the sum over
        // stands those creases up as summits and hangs the bulk of the map
        // below them.
        RIDGED_MULTIFRACTAL,
        // The same sum as it lies, with one exponent applied after it. The map
        // stands at nothing along its own zero contour, so the sum's floor is
        // that crease network; an exponent above one presses the ground down
        // onto it and leaves only the high ground standing, as mountains on a
        // plain rather than as a surface that rolls everywhere.
        TURBULENCE_POWER,
    };

    // Which of them the body is built with. Compiled rather than chosen at run
    // time: the snippet unrolls its sum over the table, and the two sides have
    // to be summing the same thing for the bounds to hold.
    static constexpr Synthesis k_synthesis{Synthesis::TURBULENCE_POWER};

    // The exponent TURBULENCE_POWER carries. At one the sum is left as it lies
    // and this is FBM again; above one the ground is pressed onto the sum's own
    // floor and the high ground is left standing alone on it, below one that
    // floor is lifted and the high ground flattens against a ceiling.
    static constexpr double k_turbulenceExponent{4.0};

    // Layers the table holds, coarsest first. Public so a caller can check at
    // compile time that the layers it asks for are there.
    //
    // The order is the synthesis's: a layer gated or warped by the ones above it
    // has to meet them first, and every sum past FBM works that way. Reading the
    // table in the order it is written is what keeps that free of a remapping
    // each new sum would have to get right.
    static constexpr int k_levelCount{4};

    // The layer the rest ride on: what the body is shaped like, where they are
    // what it wears. First, being the largest, and read by direction rather than
    // through a lattice, so it lays down no tiles and carries no frequency. In
    // every other way a layer like the rest, and summed like the rest.
    static constexpr int k_baseLevel{0};

    // The rest, each laid down through a lattice sized to its own tile. The one
    // named here is the coarsest, and the frame every finer lattice is scaled
    // from.
    static constexpr int k_firstLatticeLevel{k_baseLevel + 1};

    // What the caller read, per layer, before anything gave it a size. The
    // snippet's TerrainLevels. A layer the caller did not reach is left at zero,
    // which is what lets the sum run the whole table however few were gathered.
    struct Levels {
        std::array<double, k_levelCount> m_height{};        // unit height
        std::array<glm::dvec3, k_levelCount> m_gradient{};  // unit height per metre
    };

    // Where the surface stands above the sphere, and how it leans there. The
    // snippet's TerrainDisplacement.
    struct Displacement {
        double m_height{0.0};             // metres
        glm::dvec3 m_gradient{0.0};       // metres per metre, in the body's frame
    };

    // reliefMetres is the floor to ceiling height of the terrain the lattice
    // layers carry, baseReliefMetres the same for the layer beneath them.
    TerrainDisplacement(double reliefMetres, double baseReliefMetres);

    // Every layer at its own relief, put together by k_synthesis. Fewer layers
    // gathered is a coarser surface and not a different one: the layers left out
    // are the fine end, a layer at zero sits on the map's floor and is worth
    // nothing, and a gate it shuts holds the rest of that fine end at nothing
    // too.
    Displacement displacement(const Levels& levels) const;

    // What a layer's whole range comes to in metres, which is what its reading is
    // worth at full height. The map is unsigned, so it is also the highest that
    // layer can reach, and a caller building bounds reads it for that.
    double levelRelief(int level) const;

    // How much oftener than the field it was built at a layer's map is laid down.
    // The caller sizes its lattice and picks its mip levels off this. Zero at
    // k_baseLevel, which is read by direction and lays down no tiles.
    double levelFrequency(int level) const;

private:
    // Each layer at its own relief, added. Nothing a layer reads depends on what
    // any other found, so the sum is linear and the gradient is the same sum of
    // the same reliefs.
    Displacement fbm(const Levels& levels) const;

    // Each layer at its own relief, subtracted rather than added. The map creases
    // along its own zero contour and stands at nothing there: summed as it lies
    // those creases are the valley floors, and taken the other way up they are
    // the summits. Negation is linear, so the gradient is turned over with the
    // height and stays the derivative of what is returned.
    Displacement ridgedMultifractal(const Levels& levels) const;

    // The lattice layers' sum read as a fraction of the relief they carry
    // between them, taken to k_turbulenceExponent, given that relief back and
    // added to the base layer. The base layer is left out of the exponent: that
    // layer is the shape of the body and the rest are what it wears, so only
    // what they come to between them is bent.
    //
    // The sum stands as it lies rather than turned over. What the exponent
    // leaves is a floor with the high ground standing on it, and the body has
    // to be under that floor rather than hanging from the peaks.
    //
    // The exponent is differentiated with the height, so the gradient returned
    // is still the derivative of what is returned.
    Displacement turbulencePower(const Levels& levels) const;

    double m_relief{0.0};
    double m_baseRelief{0.0};
};
