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
//    int   k_levelCount    layers, coarsest first
//    int   k_baseLevel     the one read by direction, which leads
//    int   k_firstLatticeLevel  the coarsest of the rest, which are read through
//                               a lattice
//    int   k_synthesis     which of the sums below the body is built with, named
//                          by the k_synthesis... constants beside it
//    float k_turbulenceExponent  the exponent turbulencePower carries
//    float k_fbmMedian, k_ridgedMultifractalMedian, k_turbulencePowerMedian
//                          where the middle of each sum falls, as a fraction of
//                          the lattice relief, which each takes off what it
//                          returns so they all stand at one height
//    float k_levelReliefMetres[k_levelCount]  metres each layer stands, floor
//                                             to ceiling
//    float k_levelFrequency[k_levelCount]     how much oftener than the field a
//                                             layer's map is laid down

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
float levelFrequency(int level) {
   return k_levelFrequency[level];
}

// The relief the lattice layers carry between them, the base layer being no part
// of it: what a sum below has to spend, and what its median is of.
float latticeRelief() {
   float relief = 0.0;
   for (int level = k_firstLatticeLevel; level < k_levelCount; ++level) {
      relief += k_levelReliefMetres[level];
   }

   return relief;
}

// The base layer at its own relief, which every sum below starts from and none of
// them bends: that layer is the shape of the body and the lattice layers are what
// it wears.
TerrainDisplacement baseDisplacement(TerrainLevels levels) {
   float relief = k_levelReliefMetres[k_baseLevel];

   TerrainDisplacement displacement;
   displacement.height = relief * levels.height[k_baseLevel];
   displacement.gradient = relief * levels.gradient[k_baseLevel];

   return displacement;
}

// The lattice layers each at their own relief, added onto the base layer. Nothing
// a layer reads depends on what any other found, so the sum is linear and the
// gradient is the same sum of the same reliefs.
TerrainDisplacement fbm(TerrainLevels levels) {
   TerrainDisplacement displacement = baseDisplacement(levels);

   for (int level = k_firstLatticeLevel; level < k_levelCount; ++level) {
      float relief = k_levelReliefMetres[level];
      displacement.height += relief * levels.height[level];
      displacement.gradient += relief * levels.gradient[level];
   }

   // The offset moves the sum without bending it, so the gradient is untouched.
   displacement.height -= k_fbmMedian * latticeRelief();

   return displacement;
}

// The same lattice sum subtracted rather than added. The map creases along its
// own zero contour, one crease per octave of its own, and stands at nothing
// there: summed as it lies those creases are the valley floors, and taken the
// other way up they are the summits, with the ground hanging under them rather
// than standing on them.
//
// Negation is linear, so the gradient is turned over with the height and stays
// the derivative of what is returned.
TerrainDisplacement ridgedMultifractal(TerrainLevels levels) {
   TerrainDisplacement displacement = baseDisplacement(levels);

   for (int level = k_firstLatticeLevel; level < k_levelCount; ++level) {
      float relief = k_levelReliefMetres[level];
      displacement.height -= relief * levels.height[level];
      displacement.gradient -= relief * levels.gradient[level];
   }

   displacement.height -= k_ridgedMultifractalMedian * latticeRelief();

   return displacement;
}

// The same sum as it lies, with one exponent applied after it. A rise that never
// turns back cannot reorder altitudes, so the sum's own floor stays its floor and
// only the profile above it moves. The map stands at nothing along its own zero
// contour, so that floor is a crease network: above one the ground is pressed
// down onto it and the high ground is left standing alone on a plain, below one
// the floor is lifted and the high ground flattens against a ceiling.
//
// The exponent is differentiated with the height, so the gradient returned is
// still the derivative of what is returned.
TerrainDisplacement turbulencePower(TerrainLevels levels) {
   float carriedRelief = latticeRelief();

   float latticeSum = 0.0;
   vec3 latticeGradient = vec3(0.0);

   for (int level = k_firstLatticeLevel; level < k_levelCount; ++level) {
      float relief = k_levelReliefMetres[level];
      latticeSum += relief * levels.height[level];
      latticeGradient += relief * levels.gradient[level];
   }

   // The sum as a fraction of the whole it could have come to, so the exponent
   // remaps that fraction onto itself and the layers still stand their own relief
   // between them and no more. Lattice layers at no relief carry nothing to take
   // a fraction of.
   //
   // The magnitude of it, because the map is unsigned but a level's reading is
   // not: the lattice blend holds its contrast flat with weights that sum past one
   // and takes the overshoot back off against the field's mean, so a reading below
   // that mean lands below zero, where pow is undefined. Folding it costs a crease
   // along the level the mean sits at, at the few centimetres of ground that reach
   // under it.
   float carried = carriedRelief > 0.0 ? abs(latticeSum / carriedRelief) : 0.0;

   // The chain rule on relief times carried to the exponent: the exponent comes
   // down, and the relief cancels against the one the fraction was taken over. The
   // fold's own turn is left out, at the same few centimetres.
   float slopeFactor = k_turbulenceExponent * pow(carried, k_turbulenceExponent - 1.0);

   TerrainDisplacement displacement = baseDisplacement(levels);
   displacement.height +=
      carriedRelief * (pow(carried, k_turbulenceExponent) - k_turbulencePowerMedian);
   displacement.gradient += slopeFactor * latticeGradient;

   return displacement;
}

// Every layer at its own relief, put together by k_synthesis. Fewer layers
// gathered is a coarser surface and not a different one: the layers left out are
// the fine end, a layer at zero sits on the map's floor and is worth nothing,
// and a gate it shuts holds the rest of that fine end at nothing too.
TerrainDisplacement terrainDisplacement(TerrainLevels levels) {
   if (k_synthesis == k_synthesisRidgedMultifractal) {
      return ridgedMultifractal(levels);
   }
   if (k_synthesis == k_synthesisTurbulencePower) {
      return turbulencePower(levels);
   }

   return fbm(levels);
}

// Where one ground cover gives way to the next, and the height the change is
// blended over.
//
// Sand covers the sea floor and a strip of beach above it. The snow line depends
// on the air rather than the sea, so it is measured from the sphere.
const float k_beachRiseMetres = 10.0;
const float k_snowFloorMetres = 6000.0;
const float k_coverBlendMetres = 20.0;

// Where the ground has given up holding anything and is bare rock. Placed off
// what the octaves actually reach -- a little over half a rise per run at the
// steepest -- so the flanks turn to rock while the floors between them do not.
const float k_rockSlopeFrom = 0.20;
const float k_rockSlopeTo = 0.45;

// A stand-in for a material, and the whole of what this surface has to say about
// its own colour for now.
const vec3 k_sandColour  = vec3(0.84, 0.84, 0.47);
const vec3 k_grassColour = vec3(0.53, 0.74, 0.21);
const vec3 k_rockColour  = vec3(0.42, 0.53, 0.53);
const vec3 k_snowColour  = vec3(0.95, 0.97, 1.0000);

// Roughness by slope, on the rock's ramp: flat ground holds fine material that
// scatters light, while the bare rock on steep ground carries a highlight.
const float k_flatRoughness = 0.85;
const float k_steepRoughness = 0.45;

// What the ground is made of, from how high it stands and how steeply it leans.
//
// Slope is rise over run across the surface, which is what the caller is left
// holding once the part of the gradient that only moves the point has come off.
// Neither of these knows the body is a sphere, and neither needs to.
TerrainMaterial terrainMaterial(float heightMetres, float slope) {
   TerrainMaterial material;

   // What grows on the ground, by how high it stands: grass until the air gets
   // too thin for it, snow above that.
   vec3 cover = mix(k_grassColour, k_snowColour,
                    smoothstep(k_snowFloorMetres - k_coverBlendMetres,
                               k_snowFloorMetres + k_coverBlendMetres, heightMetres));

   // Ground steep enough sheds whatever settles on it and is left as the rock
   // beneath, which is why the flanks read bare and the floors do not.
   float bare = smoothstep(k_rockSlopeFrom, k_rockSlopeTo, slope);
   vec3 ground = mix(cover, k_rockColour, bare);

   // Sand under the water and along its edge
   float sandCeiling = k_shoreMetres + k_beachRiseMetres;
   material.colour = mix(k_sandColour, ground,
                         smoothstep(sandCeiling - k_coverBlendMetres,
                                    sandCeiling + k_coverBlendMetres, heightMetres));

   material.roughness = mix(k_flatRoughness, k_steepRoughness, bare);

   return material;
}
