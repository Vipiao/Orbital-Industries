// TerrainDisplacement.h
#pragma once

#include <array>
#include <glm/glm.hpp>

/**
 * @brief What the terrain is: how many layers it has, how tall each stands, and
 * how they add up.
 *
 * The twin of media/surfaces/terrain_displacement.glsl, written apart only
 * because the two run in different languages, and it must be kept in step with
 * it. Change one side alone and the surface moves out from under the bounds the
 * quadtree measures by.
 *
 * Nothing here knows where a reading came from. The caller takes the lookups and
 * hands them over dimensionless; this gives them size. Only the frequency column
 * is exported, because the caller reads the map at its own scale; everything that
 * turns a reading into metres happens here.
 *
 * Height and gradient come back together, being one surface measured two ways.
 * Each layer takes the same factor either way, so the gradient returned is the
 * derivative of the height returned rather than a second opinion about it.
 *
 * The relief is a constant on the snippet's side and a constructor argument here,
 * that side having no one to be handed it by.
 */
class TerrainDisplacement {
public:
    // Layers the table holds. The snippet's k_octaveCount, and public so a caller
    // can check at compile time that the octaves it asks for are there.
    static constexpr int k_octaveCount{3};

    // What the caller read, per layer, before anything gave it a size. The
    // snippet's TerrainLevels.
    //
    // The base layer is not one of the octaves and is never left out: it is what
    // the body is shaped like, and the octaves are what it wears. It is read by
    // direction rather than through a lattice, so it has no frequency here.
    struct Levels {
        double m_baseHeight{0.0};                            // unit height
        glm::dvec3 m_baseGradient{0.0};                      // unit height per metre
        std::array<double, k_octaveCount> m_height{};        // unit height
        std::array<glm::dvec3, k_octaveCount> m_gradient{};  // unit height per metre
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

    // The levels summed over the first octaveCount layers of the table, each at
    // its own amplitude. Fewer layers is a coarser surface and not a different
    // one: every layer is the same sum with the fine end left off.
    Displacement displacement(const Levels& levels, int octaveCount) const;

    // How much oftener than the field it was built at a layer's map is laid down.
    // The caller sizes its lattice and picks its mip levels off this.
    double octaveFrequency(int octave) const;

    // Tiles the layer's coordinate is carried by before it is read, which is what
    // leaves the layers off one another's phase.
    glm::dvec2 octaveShift(int octave) const;

    // Metres the octaves can reach above the sphere between them, before the
    // caller's own blend is allowed for. A ceiling, not a scale: only the bounds
    // read it, and only to leave room above the terrain.
    double octaveCeiling(int octaveCount) const;

    // The same for the layer beneath them, which needs no allowance: it is one
    // map read once rather than a blend, so it reaches its relief and no further.
    double baseCeiling() const { return m_baseRelief; }

private:
    double m_relief{0.0};
    double m_baseRelief{0.0};
};
