// terrain_displacement.glsl
//
// What the terrain is: how the layers add up into a height and a gradient, and
// what the ground is made of where they land. Nothing about where a reading came
// from -- the caller takes the lookups and hands them over dimensionless, and
// this gives them size. Only the tiling is exported, because the caller reads the
// map at its own scale.
//
// Nothing here knows the body is a sphere. A height, a rise over run and a colour
// are the same things on any shape, and wrapping the flat map onto this one is
// lattice_surface.glsl's.
//
// How tall each layer stands is not written here; that is
// src/world/TerrainDisplacement's, held once rather than on both sides waiting
// to be edited apart. A change to the arithmetic still belongs in both.
//
// Height and gradient come back together, being one surface measured two ways.
// Each layer takes the same factor either way, so the gradient returned is the
// derivative of the height returned rather than a second opinion about it.

// In scope already, written ahead of this file by planetSurfaceGlsl in
// src/world/PlanetSurfaceGlsl.cpp, off TerrainDisplacement's table. Change them
// there, in C++; there is nothing to edit here. Compiled rather than sent as
// uniforms: a frequency settles a lattice cell count that has to land on a whole
// number, and the counts size the arrays and unroll the loops below.
//
//    int   k_octaveCount   layers laid down through a lattice
//    int   k_levelCount    those, plus the base layer, which is not
//    float k_levelReliefMetres[k_levelCount]  metres each layer stands, floor
//                                             to ceiling
//    float k_octaveFrequency[k_octaveCount]   how much oftener than the field an
//                                             octave's map is laid down

// What the caller read, per layer, before anything gave it a size. A layer the
// caller did not reach is left at zero, which is what lets the sum below run the
// whole table however few were gathered.
struct TerrainLevels {
   float height[k_levelCount];    // unit height
   vec3 gradient[k_levelCount];   // unit height per metre, in the body's frame
};

// Where the surface stands above the sphere, and how it leans there.
struct TerrainDisplacement {
   float height;    // metres
   vec3 gradient;   // metres per metre, in the body's frame
};

// What the ground is made of where it was measured.
struct TerrainMaterial {
   vec3 colour;
   float roughness;
};

// The column the lattice reads, exported rather than used here. A frequency is
// half of a slope -- the amplitude beside it says how far a layer rises, this
// says across what -- so the two sit in one table, and the ladder asserts they
// step together.
float octaveFrequency(int octave) {
   return k_octaveFrequency[octave];
}

// Every layer at its own relief, summed. Fewer layers gathered is a coarser
// surface and not a different one: the same sum with the fine end at zero.
TerrainDisplacement terrainDisplacement(TerrainLevels levels) {
   TerrainDisplacement displacement;
   displacement.height = 0.0;
   displacement.gradient = vec3(0.0);

   for (int level = 0; level < k_levelCount; ++level) {
      float relief = k_levelReliefMetres[level];
      displacement.height += relief * levels.height[level];
      displacement.gradient += relief * levels.gradient[level];
   }

   return displacement;
}

// Metres of height between one colour band and the next. A contour interval: the
// surface is banded by how high it stands, so relief too gentle to see in the
// shading still reads as a pattern of stripes.
//
// A stand-in for a material, and the whole of what this surface has to say about
// its own colour for now.
const float k_colourBandMetres = 500.0;

// Roughness by slope: flats hold the fine material that settles out of everything
// standing above them and scatter in every direction, while ground steep enough to
// shed it is left as the rock beneath, which carries a highlight.
//
// The ramp is placed off the ladder's own measurements -- mean rise over run is
// 0.60 and the steepest ground reaches 2.27 -- so it spans the middle of the range
// and keeps the far end for ground that is truly bare.
const float k_flatRoughness = 0.85;
const float k_steepRoughness = 0.45;
const float k_roughnessSlopeFrom = 0.25;
const float k_roughnessSlopeTo = 1.20;

// What the ground is made of, from how high it stands and how steeply it leans.
//
// Slope is rise over run across the surface, which is what the caller is left
// holding once the part of the gradient that only moves the point has come off.
// Neither of these knows the body is a sphere, and neither needs to.
TerrainMaterial terrainMaterial(float heightMetres, float slope) {
   const float k_turn = 6.283185307179586;

   TerrainMaterial material;
   material.colour =
      vec3(0.5 + 0.5 * sin(heightMetres * (k_turn / k_colourBandMetres)));
   material.roughness =
      mix(k_flatRoughness, k_steepRoughness,
          smoothstep(k_roughnessSlopeFrom, k_roughnessSlopeTo, slope));

   return material;
}
