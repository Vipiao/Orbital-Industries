// terrain_displacement.glsl
//
// How the layers add up. Nothing about where a reading came from -- the caller
// takes the lookups and hands them over dimensionless, and this gives them size.
// Only the tiling is exported, because the caller reads the map at its own
// scale.
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
//    vec3  k_octaveTiling[k_octaveCount]      how much oftener than the field an
//                                             octave's map is laid down, then the
//                                             tiles its coordinate is carried by

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

// How much oftener than the field it was built at an octave's map is laid down.
// The caller sizes its lattice and picks its mip levels off this.
float octaveFrequency(int octave) {
   return k_octaveTiling[octave].x;
}

// Tiles the octave's coordinate is carried by before it is read, which is what
// leaves the octaves off one another's phase.
vec2 octaveShift(int octave) {
   return k_octaveTiling[octave].yz;
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
