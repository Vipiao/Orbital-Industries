// terrain_displacement.glsl
//
// What the terrain is: how many layers it has, how tall each stands, and how
// they add up. Nothing about where a reading came from -- the caller takes the
// lookups and hands them over dimensionless, and this gives them size.
//
// The twin of src/world/TerrainDisplacement.h. Change one side alone and the
// surface moves out from under the bounds the quadtree measures by.
//
// A level arrives in the map's own units: unit height, and unit height per
// metre where it is a gradient. Only the frequency column is exported, because
// the caller reads the map at its own scale; everything that turns a reading
// into metres happens here.
//
// Height and gradient come back together, being one surface measured two ways.
// Each layer takes the same factor either way, so the gradient returned is the
// derivative of the height returned rather than a second opinion about it.

// The same map laid down at several scales and added up: frequency multiplier,
// amplitude as a fraction of the relief, and a shift in tiles. Sixteen to a step
// in frequency, four to a step in amplitude, so each layer is four times the rise
// over run of the one above.
//
// The root of frequency rather than frequency itself: the reciprocal would give
// every layer the same slope, where real ground has gentle big landforms and
// steep small ones. Three steps reach thirty-one degrees, the angle loose ground
// gives way at, past which roughening buys nothing.
//
// Amplitude is not a slope. A tile does not rise by its own range across its own
// width -- the map carries eight layers of its own, averaging nearly three times
// that -- so rise over run is amplitude over tile times what the map does.
// Measured, as averages:
//
//    49.8 km at 658 m        two degrees
//    3.11 km at 165 m        eight and a half degrees
//     194 m at 41.1 m        thirty-one degrees
//
// The map's worst texel runs three times its mean, putting the steepest ground
// near sixty-six, and that is the figure the quadtree's ranges answer for. Steps
// this close cost span at the fine end, leaving the finest feature hundreds of
// metres; a fourth layer would buy it back.
//
// The coarsest picks up a sixty-fourth of the base layer's coarsest feature,
// stretching a tile built at 1274.2 m over thirty-nine of its own.
//
// The frequency need not be a power of two: what has to land exactly is the cell
// count the caller's lattice comes to, which it rounds and asserts is whole. The
// shift is what keeps the layers off one lattice: each repeats a whole number of
// times per coarser tile and would otherwise land on it with the same phase,
// every repeat reinforcing the last.
const int k_octaveCount = 3;
const vec4 k_octaves[k_octaveCount] = vec4[k_octaveCount](
   vec4(0.0256, 1.0, 0.0, 0.0),
   vec4(0.4096, 1.0 / 4.0, 0.37, 0.71),
   vec4(6.5536, 1.0 / 16.0, 0.61, 0.19));

// Metres between the map's floor and its ceiling. The map is unsigned, so the
// terrain rises from the sphere rather than straddling it, and this is the full
// depth of the relief.
//
// Tiny against a planet's radius, so the body reads as a sphere and the relief
// shows in the shading rather than the silhouette. The knob to raise for
// exaggerated terrain, at the cost of a steeper surface: against the tile size
// this sets the slope, and the slope is what the quadtree's ranges have to keep
// up with.
const float k_reliefMetres = 658.0;

// Metres between the base layer's floor and its ceiling. Handed over rather than
// written here, unlike the constants above: the side that generated the base
// maps already holds this number, and a copy of it here would be a second place
// for it to be wrong.
uniform float u_baseReliefMetres;

// What the caller read, per layer, before anything gave it a size.
//
// The base layer is not one of the octaves and is never left out: it is what the
// body is shaped like, and the octaves are what it wears. It is read by
// direction rather than through a lattice, so it has no frequency here.
struct TerrainLevels {
   float baseHeight;                 // unit height
   vec3 baseGradient;                // unit height per metre, in the body's frame
   float height[k_octaveCount];      // unit height
   vec3 gradient[k_octaveCount];     // unit height per metre, in the body's frame
};

// Where the surface stands above the sphere, and how it leans there.
struct TerrainDisplacement {
   float height;    // metres
   vec3 gradient;   // metres per metre, in the body's frame
};

// How much oftener than the field it was built at a layer's map is laid down.
// The caller sizes its lattice and picks its mip levels off this.
float octaveFrequency(int octave) {
   return k_octaves[octave].x;
}

// Tiles the layer's coordinate is carried by before it is read, which is what
// leaves the layers off one another's phase.
vec2 octaveShift(int octave) {
   return k_octaves[octave].zw;
}

// The levels summed over the first octaveCount layers of the table, each at its
// own amplitude. Fewer layers is a coarser surface and not a different one:
// every layer is the same sum with the fine end left off.
TerrainDisplacement terrainDisplacement(TerrainLevels levels, int octaveCount) {
   TerrainDisplacement displacement;
   displacement.height = u_baseReliefMetres * levels.baseHeight;
   displacement.gradient = u_baseReliefMetres * levels.baseGradient;

   for (int octave = 0; octave < octaveCount; ++octave) {
      float amplitude = k_reliefMetres * k_octaves[octave].y;
      displacement.height += amplitude * levels.height[octave];
      displacement.gradient += amplitude * levels.gradient[octave];
   }

   return displacement;
}
