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
 * Each layer takes the same factor either way, so the gradient returned is the
 * derivative of the height returned rather than a second opinion about it.
 */
class TerrainDisplacement {
public:
    // Octaves the table holds. The snippet's k_octaveCount, and public so a
    // caller can check at compile time that the octaves it asks for are there.
    static constexpr int k_octaveCount{3};

    // The base layer is the last row of the table and the one the octaves ride
    // on: what the body is shaped like, where they are what it wears. It is read
    // by direction rather than through a lattice, so it lays down no tiles and
    // takes no shift. Last, at k_octaveCount, so that an octave keeps the index
    // it is read at; in every other way it is a layer like the rest and is
    // summed like the rest.
    static constexpr int k_levelCount{k_octaveCount + 1};

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

    // reliefMetres is the floor to ceiling height of the terrain the octaves
    // carry, baseReliefMetres the same for the layer beneath them.
    TerrainDisplacement(double reliefMetres, double baseReliefMetres);

    // Every layer at its own relief, summed. Fewer layers gathered is a coarser
    // surface and not a different one: the same sum with the fine end at zero.
    Displacement displacement(const Levels& levels) const;

    // What a layer's whole range comes to in metres, which is what its reading is
    // worth at full height. The map is unsigned, so it is also the highest that
    // layer can reach, and a caller building bounds reads it for that.
    double levelRelief(int level) const;

    // How much oftener than the field it was built at a layer's map is laid down.
    // The caller sizes its lattice and picks its mip levels off this.
    double octaveFrequency(int octave) const;

    // Tiles the layer's coordinate is carried by before it is read, which is what
    // leaves the layers off one another's phase.
    glm::dvec2 octaveShift(int octave) const;

private:
    double m_relief{0.0};
    double m_baseRelief{0.0};
};
