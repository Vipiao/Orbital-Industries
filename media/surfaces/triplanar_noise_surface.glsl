// triplanar_noise_surface.glsl
//
// Terrain read out of a baked, tileable noise map instead of evaluated in
// closed form. The map is flat, square and dimensionless; this is what gives it
// a size and a height and wraps it onto a sphere.
//
// The GPU half of the body's shape. src/world/PlanetSurface.cpp is the other
// half and must be kept in step: the same crude point, the same position and
// normal. Every constant and every function below has a twin there. Change one
// side alone and the surface moves out from under the bounds the quadtree
// measures by, which is what closes the seams between patches.
//
// Four planar projections, one anchored to each of the lattice points the point
// being shaded stands between, blended by how near it stands to each. The
// lattice is the cube the quadtree subdivides, cut into cells, and a projection
// is the offset from a lattice point to the point, taken in a tangent plane
// there -- at most a cell wide, so no plane is ever turned far enough to skew
// what it carries.
//
// A lattice point is named by its place on the cube as three whole numbers,
// never by a face and a pair of indices, and that is why the cube's twelve edges
// and eight corners stop being special: a point on an edge sits at the cube's
// extent along two axes at once, so both faces name it identically.
//
// Which cell a point lands in is a ratio of two of its components, so it does not
// depend on how far out the point stands, only on which way. The lattice is read
// off the crude point itself, and only the position path pays the wide square
// root that projects one onto the sphere.
//
// Every lookup is taken at the level its sampling resolves, and the two stages
// arrive at that level differently because their samples are differently shaped.
// The fragment stage has screen derivatives and asks for the level its pixel
// covers. The vertex stage has none, and is handed the metres between
// neighbouring vertices instead. A spacing rather than a distance: what a sample
// resolves is how far it stands from the next one, and the two part company
// wherever the terrain lifts the camera off the sphere -- a camera an arm's
// length above ground that stands four hundred metres up is four hundred metres
// from the sphere below it, and would read the ground through a level chosen for
// a patch that size.
//
// Neither depends on which patch asked, which is what geometry requires: the
// elevation must be a pure function of position across every quadtree level, or
// two patches meeting at different levels would displace their shared edge to
// different places and reopen the seam that morphing exists to close.
//
// What this buys is the same on both paths: detail finer than a sample arrives
// as its average instead of as whichever point of it the sample landed on, so
// neither normals nor vertices sparkle, and the lookups stay inside the texture
// cache instead of striding a map that no longer fits it.
//
// Both paths take the point in a wide float, and both reduce it to its cell
// before narrowing. A point on this body is millions of metres from the origin,
// where a float's spacing is half a metre -- two hundred texels of the finest
// octave's map -- so a lookup taken from a narrow coordinate reads that lattice
// rather than the terrain. What stays narrow is everything downstream of the
// split: weights, slopes and the normal are directions and heights, not
// positions. The twin is double throughout, being under no such pressure.
//
// Df3 and its arithmetic arrive already in scope, from the graphics engine's
// src/graphics/shared_shaders/dekker_arithmetic.glsl -- this file is spliced
// into a stage that has included it, so there is nothing to include here.

// Metres from the body's centre to the sphere the cube projects onto. Exact in a
// float, so it needs nothing in a low part.
const float k_radiusMetres = 6371000.0;

uniform sampler2D u_noiseMap;     // R16 unorm, one tile, spanning exactly [0, 1]
uniform sampler2D u_gradientMap;  // RG16F, gradient per unit of tile, same tile

// The base layer: what the body is shaped like at its own size, under everything
// the octaves lay on it. Read by direction rather than through the lattice,
// because at this scale there is nothing to tile -- one pass of the field covers
// the body once, so there is no repeat to hide and no seam to close.
//
// The one part of this file with no twin in PlanetSurface.cpp. That side
// evaluates the field these were baked from rather than reading them back, and
// the two part company only by what bilinear costs across a texel of a smooth
// field. A drift rather than a step: there is no hash down here to turn one bit
// of disagreement into a different answer.
uniform samplerCube u_baseElevationMap;  // R16 unorm, spanning exactly [0, 1]
uniform samplerCube u_baseGradientMap;   // RGB16F, slope per unit of direction

// Metres between the base layer's floor and its ceiling. Handed over rather than
// written here, unlike the constants below: the side that generated the maps
// already holds this number, and a copy of it here would be a second place for
// it to be wrong.
uniform float u_baseReliefMetres;

// Metres one tile of the map spans, as a ratio of two whole numbers a float
// holds exactly. Below the body's width the tile repeats, which is what puts
// detail on a planet the map could never cover in one pass.
//
// Written as the ratio rather than as 1274.2 because that value is not a float,
// and the lookups need it to more than a float's worth: a plane coordinate is
// thousands of tiles from the origin, so a part in ten million of the tile size
// is a thousandth of a tile of drift by the time it gets there.
const float k_tileSpanMetres = 12742000.0;
const float k_tilesPerSpan = 10000.0;

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

// Lattice points a cell is bounded by, and so lookups an octave costs.
const int k_latticeCorners = 4;

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
// What has to land exactly is the cell count each frequency comes to, which
// octaveCells rounds and PlanetSurface asserts is whole; the frequency itself
// need not be a power of two, since everything else it scales is a smooth
// coordinate rather than a lattice. The shift is what keeps the layers off one
// lattice: each repeats a whole number of times per coarser tile and would
// otherwise land on it with the same phase, every repeat reinforcing the last.
const int k_octaveCount = 3;
const vec4 k_octaves[k_octaveCount] = vec4[k_octaveCount](
   vec4(0.0256, 1.0, 0.0, 0.0),
   vec4(0.4096, 1.0 / 4.0, 0.37, 0.71),
   vec4(6.5536, 1.0 / 16.0, 0.61, 0.19));

// Tiles of an octave's own layer that one lattice cell spans, so the lattice
// takes its size from the layer it carries and a frequency changed above carries
// the lattice with it.
//
// Few enough that the map never repeats visibly within one cell, many enough
// that the cells do not become the pattern themselves. Cutting finer costs a
// normal: the term this blend drops runs as the reciprocal of a cell.
const float k_cellTiles = 2.0;

// Octaves the drawn geometry carries. PlanetSurface.cpp must use this same
// count: it is the surface the quadtree's bounds are measured on, and the
// vertex and depth stages both reach it through here.
const int k_positionOctaves = 3;

// Octaves the shading carries, and free to run past the geometry: both read the
// same octaves and each stops where its own sampling does, a pixel resolving
// finer than a quad. A layer past the geometry would arrive as relief in the
// normal that the vertices never carry, and since it tilts a normal without
// moving one, nothing that measures the surface has to answer for it.
//
// Level with the geometry here, the table holding nothing further to reach for.
//
// It costs four lookups per layer wherever one is still above the map's top,
// which the gradient path skips it below.
const int k_shadingOctaves = 3;

// One lattice point's plane, as the point being shaded stands in it.
struct LatticePlane {
   vec2 tileCoord;   // where the point falls in the map, in tiles
   vec3 tangent;     // the plane's two axes in the body's frame, which the
   vec3 bitangent;   // gradient comes back expressed in
   float weight;     // how much of the blend this plane carries
};

// Tiles of an octave's own layer to the metre. Its map is laid down that much
// more often than the field it was built at.
float octaveTilesPerMetre(int octave) {
   return k_tilesPerSpan * k_octaves[octave].x / k_tileSpanMetres;
}

// Cells across one half of a cube face for the layer named: that octave's tiles
// across the half face, in groups of k_cellTiles.
//
// Grouped so the arithmetic stays exact -- the radius over the span is a half,
// the rest whole numbers and powers of two -- so this float and the twin's
// double land on the same whole number. PlanetSurface asserts they do; constants
// leaving it between two would let the two sides round apart.
int octaveCells(int octave) {
   float tilesPerHalfFace =
      k_tilesPerSpan * (k_radiusMetres / k_tileSpanMetres) * k_octaves[octave].x;

   return int(tilesPerHalfFace / k_cellTiles + 0.5);
}

// A lattice point's own random numbers, drawn from its place on the cube.
//
// The one thing here that must match PlanetSurface.cpp bit for bit rather than
// merely closely: everything else about a plane is a smooth function of where
// its point sits and may differ in the last place harmlessly, but neighbouring
// points draw unrelated numbers, so one bit apart is a different plane.
//
// No octave in the mix, and none needed: a point's largest coordinate is its own
// octave's cell count, so no two layers ever name the same point.
uint latticeHash(ivec3 point) {
   uint h = uint(point.x) * 0x9E3779B1u;
   h = (h ^ uint(point.y)) * 0x85EBCA6Bu;
   h = (h ^ uint(point.z)) * 0xC2B2AE35u;

   h ^= h >> 16;
   h *= 0x85EBCA6Bu;
   h ^= h >> 13;
   h *= 0xC2B2AE35u;
   h ^= h >> 16;

   return h;
}

// A lattice coordinate split into the cell it falls in and how far across that
// cell it stands, the whole part taken off while the value is still wide. A
// cell at the finest octave is a few metres and the body is millions of them,
// so a coordinate narrowed before the split holds nothing but whole cells.
//
// The same decomposition dfFractToFloat makes, keeping the whole part rather
// than discarding it: which cell a point is in is half of what is wanted here.
float latticeSplit(Df coordinate, out int cell) {
   precise float whole = floor(coordinate.hi);
   precise float part = (coordinate.hi - whole) + coordinate.lo;
   precise float carry = floor(part);

   cell = int(whole) + int(carry);
   return part - carry;
}

// The cube face a point stands on, and where across it, found once and read by
// every octave.
//
// A point's face comes from its largest component, by magnitude rather than
// signed value, so a point on a face boundary answers the same from either side.
struct LatticeFrame {
   int major;             // the axis the face stands out along,
   int uAxis;             // and the two cut across it
   int vAxis;
   Df coordU;             // where the point falls across the face, in cells of
   Df coordV;             // the coarsest octave, which the rest scale from
   float faceSign;        // which of the pair of faces it is
   float metresPerCell;   // at that same octave
};

// The frame the point stands in, taken off the crude point rather than off its
// image on the sphere: a uniform scaling cancels out of the ratio the cell comes
// from, and only the metres a cell spans carry it back in. That is a size rather
// than a place, so it stays narrow, and the wide square root the projection would
// cost is never taken.
LatticeFrame latticeFrameOf(Df3 crudePoint) {
   vec3 crude = df3ToVec(crudePoint);
   vec3 magnitude = abs(crude);

   LatticeFrame frame;
   frame.major = magnitude.x > magnitude.y ? (magnitude.x > magnitude.z ? 0 : 2)
                                           : (magnitude.y > magnitude.z ? 1 : 2);
   frame.uAxis = (frame.major + 1) % 3;
   frame.vAxis = (frame.major + 2) % 3;
   frame.faceSign = crudePoint.hi[frame.major] < 0.0 ? -1.0 : 1.0;

   // How far out the face stands, and the one wide division the lattice costs
   // however many octaves go on to read it.
   Df faceSpan = Df(frame.faceSign * crudePoint.hi[frame.major],
                    frame.faceSign * crudePoint.lo[frame.major]);
   float cells = float(octaveCells(0));
   Df cellScale = dfDiv(dfFromFloat(cells), faceSpan);

   frame.coordU =
      dfMul(Df(crudePoint.hi[frame.uAxis], crudePoint.lo[frame.uAxis]), cellScale);
   frame.coordV =
      dfMul(Df(crudePoint.hi[frame.vAxis], crudePoint.lo[frame.vAxis]), cellScale);

   // What turns a fraction of a cell into a distance, measured on the sphere the
   // cells are laid on: the projection pulls a point in by the radius over how
   // far out it stood. The face's extent stands in for the arc beneath it, the
   // two parting company by the cell's own width against the radius.
   frame.metresPerCell = k_radiusMetres * (magnitude[frame.major] / length(crude)) / cells;

   return frame;
}

// The four lattice points the shading point stands between, each with the plane
// its share of the terrain is read through.
//
// Each plane is anchored to its own lattice point rather than to the face, which
// is what the seamlessness rests on: face normals differ by a right angle across
// an edge, lattice point directions not at all.
//
// Every octave's cell count is the coarsest one's times a power of two, so a
// layer reaches its own lattice by shifting the frame's exponents. Exact, and
// free of wide arithmetic, which leaves the lattice no dearer at four octaves
// than at one. PlanetSurface asserts the counts step that way.
void latticePlanes(LatticeFrame frame, int octave,
                   out LatticePlane planes[k_latticeCorners]) {
   int cells = octaveCells(octave);
   float cellRatio = float(cells) / float(octaveCells(0));

   int cellU;
   int cellV;
   float fracU =
      latticeSplit(Df(frame.coordU.hi * cellRatio, frame.coordU.lo * cellRatio), cellU);
   float fracV =
      latticeSplit(Df(frame.coordV.hi * cellRatio, frame.coordV.lo * cellRatio), cellV);

   float metresPerCell = frame.metresPerCell / cellRatio;

   // Smoothed, so the blend arrives at a cell boundary with no slope left in it.
   // Raw fractions would leave a crease along every one of them, and along the
   // cube's own edges, which are boundaries too.
   float weightU = smoothstep(0.0, 1.0, fracU);
   float weightV = smoothstep(0.0, 1.0, fracV);

   vec2 alongU = vec2(1.0 - weightU, weightU);
   vec2 alongV = vec2(1.0 - weightV, weightV);

   // Four weights summing to one keep only the sum of their squares' worth of
   // the field's variance, what they mix being four unrelated pieces of it: a
   // quarter at a cell's centre where all four stand equal. Dividing by the root
   // of that sum holds the contrast flat, so cells stop showing as soft patches
   // between crisp corners. The sum factors, a weight being one fraction's share
   // times the other's.
   //
   // It costs weights that no longer sum to one. The gradient does not care;
   // elevationAt does, and answers for it.
   float restore = inversesqrt(dot(alongU, alongU) * dot(alongV, alongV));

   float tilesPerMetre = octaveTilesPerMetre(octave);
   vec2 shift = k_octaves[octave].zw;

   for (int corner = 0; corner < k_latticeCorners; ++corner) {
      float stepU = float(corner & 1);
      float stepV = float(corner >> 1);

      ivec3 point;
      point[frame.major] = int(frame.faceSign) * cells;
      point[frame.uAxis] = cellU + (corner & 1);
      point[frame.vAxis] = cellV + (corner >> 1);

      vec3 direction = normalize(vec3(point));

      // A pair of axes across the plane, built with neither a branch nor a
      // second root: Duff's construction, exact for a unit direction and steady
      // at both poles. Taken off the point alone, so every face that names the
      // point builds it the same plane.
      float zSign = direction.z >= 0.0 ? 1.0 : -1.0;
      float spread = -1.0 / (zSign + direction.z);
      float cornerTerm = direction.x * direction.y * spread;
      vec3 basisU = vec3(1.0 + zSign * direction.x * direction.x * spread,
                         zSign * cornerTerm, -zSign * direction.x);
      vec3 basisV = vec3(cornerTerm, zSign + direction.y * direction.y * spread,
                         -direction.y);

      // A byte apiece. The turn arrives as a pair's direction rather than as an
      // angle, which is one root where a sine and a cosine stood -- the same
      // thing to the terrain, and this runs four times per octave.
      //
      // Odd numerators over an even scale, so neither half lands on the middle
      // and the pair is never the zero that points nowhere.
      uint hash = latticeHash(point);
      vec2 turn = normalize(
         vec2(float((hash & 0xFFu) * 2u + 1u),
              float(((hash >> 8) & 0xFFu) * 2u + 1u)) * (1.0 / 256.0) - 1.0);
      vec2 turnShift = vec2(float((hash >> 16) & 0xFFu),
                            float((hash >> 24) & 0xFFu)) * (1.0 / 256.0);

      // Turned about the lattice point, which together with the shift is what
      // leaves each cell reading the map somewhere else and along some other
      // direction: the repeat is still there, and no longer runs far enough in
      // one piece to be read as one. The axes carry the turn rather than the
      // coordinate, so the gradient comes back in the frame it was taken in.
      vec3 tangent = turn.x * basisU - turn.y * basisV;
      vec3 bitangent = turn.y * basisU + turn.x * basisV;

      // From the lattice point to the point being shaded, along the two axes the
      // cell is cut on. What of it stands off the body falls out in the two dot
      // products below, the plane's axes having no reach that way.
      vec3 offset = vec3(0.0);
      offset[frame.uAxis] = (fracU - stepU) * metresPerCell;
      offset[frame.vAxis] = (fracV - stepV) * metresPerCell;

      planes[corner].tileCoord =
         vec2(dot(offset, tangent), dot(offset, bitangent)) * tilesPerMetre
         + shift + turnShift;
      planes[corner].tangent = tangent;
      planes[corner].bitangent = bitangent;
      planes[corner].weight = alongU[corner & 1] * alongV[corner >> 1] * restore;
   }
}

// One plane's elevation, at full amplitude: the caller scales, having one
// weighted sum per octave to do it to rather than four.
//
// The coordinate arrives a few tiles from its plane's own lattice point and is
// left that way, the sampler's wrap taking it home. Two patches meeting at a
// level boundary reach the same lattice point and read the same texel.
//
// The level is given rather than derived: this is read from stages that have no
// derivatives, and the caller knows how far apart its samples stand.
float sampleElevation(vec2 tileCoord, float mipLevel) {
   return textureLod(u_noiseMap, tileCoord, mipLevel).r * k_reliefMetres;
}

// Levels sharper than the spacing each lookup is taken. At zero a texel matches
// the spacing, already a level short of suppressing every wavelength the samples
// cannot carry; raising it buys detail back, and what it buys moves as the
// samples do.
const float k_detailSharpening = 0.0;

// The level at which one texel of an octave's map covers the spacing given. An
// octave lays the map down k_octaves[].x times as often, so its texel is that
// much smaller on the ground.
//
// Not clamped above: the sampler stops at its own 1x1 top, where an octave
// returns the map's mean. So an octave retires by fading into its own average as
// the samples spread past it, with nothing left to step when it does.
//
// Sized off the elevation map for both, the two being baked from one field at
// one resolution.
float octaveMipLevel(float sampleSpacing, float octaveFrequency) {
   float texelSpan = k_tileSpanMetres
      / (k_tilesPerSpan * octaveFrequency * float(textureSize(u_noiseMap, 0).x));

   return max(0.0, log2(sampleSpacing / texelSpan) - k_detailSharpening);
}

// The level at which a lookup covers the whole tile, so that what comes back is
// the field's mean however the coordinate moves.
float mapTopLevel() {
   return log2(float(textureSize(u_noiseMap, 0).x));
}

// The level at which one texel of a base map covers the spacing given. A cube
// face spans a quarter turn of the body, so its texels are that arc cut into as
// many pieces as the map has a side.
//
// The side is passed rather than read from one of the two maps, because they are
// not the same size and a level means a different thing on each: level zero of
// the smaller map is already as coarse as level one of the larger.
float baseMipLevel(float sampleSpacing, float mapSide) {
   const float k_quarterTurn = 1.5707963267948966;
   float texelSpan = k_quarterTurn * k_radiusMetres / mapSide;

   return max(0.0, log2(sampleSpacing / texelSpan));
}

// The base layer's elevation and its gradient in the body's frame.
//
// The direction is narrow, and safely so where nothing else here is: it is
// bounded by one however wide the body, so a float resolves it to a millionth of
// one of this map's texels. What forces the wide arithmetic elsewhere is that a
// position on this body is millions of metres from the origin, and a direction
// never is.
//
// The map holds the slope per unit of direction, and a metre across the surface
// turns the direction by one over the radius. That ratio is what gives the
// gradient the metres per metre the octaves come back in, from the same one
// constant that gives the elevation its own metres.
float baseElevation(vec3 direction, float sampleSpacing) {
   float side = float(textureSize(u_baseElevationMap, 0).x);
   return textureLod(u_baseElevationMap, direction, baseMipLevel(sampleSpacing, side)).r
      * u_baseReliefMetres;
}

vec3 baseGradient(vec3 direction, float sampleSpacing) {
   float side = float(textureSize(u_baseGradientMap, 0).x);
   return textureLod(u_baseGradientMap, direction, baseMipLevel(sampleSpacing, side)).rgb
      * (u_baseReliefMetres / k_radiusMetres);
}

// Gradient of one plane's elevation, in metres per metre, expressed in that
// plane's own two axes.
//
// The map holds the gradient as it stands, per unit of tile, matching the field
// it was differenced from. So the same two constants that give the elevation its
// metres give the gradient its own, and their ratio is exactly rise over run.
//
// The level is given rather than derived, as sampleElevation's is. Each plane's
// coordinate jumps to the next lattice point at every cell boundary, so
// differencing one across a pixel quad would read that jump as an infinite slope
// and pick the coarsest level along a line through every cell.
//
// Full amplitude, again as sampleElevation is: the caller scales, having one
// weighted sum per octave to do it to rather than four.
vec2 sampleSlope(vec2 tileCoord, float tilesPerMetre, float mipLevel) {
   vec2 perTile = textureLod(u_gradientMap, tileCoord, mipLevel).rg;
   return perTile * (k_reliefMetres * tilesPerMetre);
}

// The lattice is taken per octave, each layer wanting cells sized to its own
// tile, so the four planes are found again for every one of them.
//
// sampleSpacing is how far apart the samples reading this stand on the ground.
// Every octave is read at the level that spacing resolves, so a layer finer than
// the samples arrives as its own average rather than as whichever point of it
// each sample landed on. What that removes was never terrain: it was the noise a
// lattice makes.
float elevationAt(Df3 crudePoint, int octaveCount, float sampleSpacing) {
   // The map's own mean, which the top of its mip chain holds: a lookup there
   // covers the whole tile. The blend's weights sum to more than one, so each
   // layer overshoots by whatever it stands above this, and this is what that
   // overshoot is taken back off against.
   //
   // One lookup, outside the octaves, on a texel every pixel shares. The gradient
   // path needs none, the mean slope of a field that wraps being zero.
   float fieldMean = textureLod(u_noiseMap, vec2(0.5), mapTopLevel()).r * k_reliefMetres;

   LatticeFrame frame = latticeFrameOf(crudePoint);

   // The base layer is not one of the octaves and is never left out: it is what
   // the body is shaped like, and the octaves are what it wears.
   float elevation = baseElevation(normalize(df3ToVec(crudePoint)), sampleSpacing);

   for (int octave = 0; octave < octaveCount; ++octave) {
      LatticePlane planes[k_latticeCorners];
      latticePlanes(frame, octave, planes);
      float mipLevel = octaveMipLevel(sampleSpacing, k_octaves[octave].x);

      float layer = 0.0;
      float covered = 0.0;
      for (int corner = 0; corner < k_latticeCorners; ++corner) {
         layer += planes[corner].weight
            * sampleElevation(planes[corner].tileCoord, mipLevel);
         covered += planes[corner].weight;
      }

      elevation += k_octaves[octave].y * (layer + fieldMean * (1.0 - covered));
   }

   return elevation;
}

// Gradient of the elevation above, in the body's own frame. Reads at the level
// the spacing given resolves, exactly as elevationAt does and through the same
// function, the two stages differing only in how they arrive at a spacing.
//
// Each plane returns a slope in its own two axes, which those axes carry back
// into the body's frame. Blending the four as vectors is exact in a way blending
// normals would not be: a gradient is linear, a normalized normal is not.
//
// The weights' own variation is left out. Keeping it would add a term
// proportional to the difference between the planes' elevations, which the
// blend already softens.
vec3 gradientAt(Df3 crudePoint, int octaveCount, float sampleSpacing) {
   float topLevel = mapTopLevel();
   LatticeFrame frame = latticeFrameOf(crudePoint);

   vec3 gradient = baseGradient(normalize(df3ToVec(crudePoint)), sampleSpacing);

   for (int octave = 0; octave < octaveCount; ++octave) {
      float mipLevel = octaveMipLevel(sampleSpacing, k_octaves[octave].x);

      // Past the top the map returns the tile's mean, and the mean gradient of a
      // field that wraps is zero: the octave has nothing left to tilt a normal
      // with, so its four lookups are skipped rather than summed to nothing.
      // The height path has no such exit, a mean elevation being a real offset.
      if (mipLevel >= topLevel) {
         continue;
      }

      LatticePlane planes[k_latticeCorners];
      latticePlanes(frame, octave, planes);
      float tilesPerMetre = octaveTilesPerMetre(octave);

      vec3 layer = vec3(0.0);
      for (int corner = 0; corner < k_latticeCorners; ++corner) {
         vec2 slope = sampleSlope(planes[corner].tileCoord, tilesPerMetre, mipLevel);
         layer += planes[corner].weight
            * (slope.x * planes[corner].tangent + slope.y * planes[corner].bitangent);
      }

      gradient += k_octaves[octave].y * layer;
   }

   return gradient;
}

// Where a crude point is drawn: the sphere, raised along its outward direction.
// A scalar height rather than a free displacement, so the normal below can be
// its gradient rather than a second opinion about the same surface.
//
// The radius is the body-sized term and carries the width; the height rides on it
// as a float, being at most the relief. The two are added before the direction is
// scaled by them, sharing one outward direction, so the sum rounds once. Two
// patches meeting at a level boundary reach the same crude point to a nanometre,
// so the seam the morph closes in principle closes in the arithmetic as well.
//
// The only wide square root the surface costs, and the only path that needs one:
// the lattice reads the crude point directly, so nothing but this result is
// measured from the body's centre. At this width the root has no hardware behind
// it, and the shading stage runs far more often than this one.
Df3 cdlodSurfacePoint(Df3 crudePoint, float sampleSpacing) {
   Df reach = dfAdd(
      dfFromFloat(k_radiusMetres),
      dfFromFloat(elevationAt(crudePoint, k_positionOctaves, sampleSpacing)));

   return df3Scale(df3Normalize(crudePoint), reach);
}

// Metres of height between one colour band and the next. A contour interval:
// the surface is banded by how high it stands, so relief too gentle to see in
// the shading still reads as a pattern of stripes.
//
// A stand-in for a material, and the whole of what this surface has to say about
// its own colour for now.
const float k_colourBandMetres = 500.0;

// Roughness by slope: flats hold the fine material that settles out of
// everything standing above them and scatter in every direction, while ground
// steep enough to shed it is left as the rock beneath, which carries a
// highlight.
//
// The ramp is placed off the ladder's own measurements -- mean rise over run is
// 0.60 and the steepest ground reaches 2.27 -- so it spans the middle of the
// range and keeps the far end for ground that is truly bare.
const float k_flatRoughness = 0.85;
const float k_steepRoughness = 0.45;
const float k_roughnessSlopeFrom = 0.25;
const float k_roughnessSlopeTo = 1.20;

// How the surface faces, what colour it is drawn in, and how tight a highlight
// it carries. Only the
// tangential part of the gradient tilts the normal; the radial part moves the
// point without turning it. The tangent stretch as the surface rises is left out
// -- it scales the tilt by 1 / (1 + height/radius), a part in a hundred thousand
// against a planet.
//
// Reads the octaves the geometry carries, so this is the gradient of the surface
// drawn rather than of a finer one. A pixel still resolves more of them than a
// quad does: both stop where their own sampling stops, and the fragment stage
// samples the finer of the two.
//
// The colour costs an elevation the normal itself has no use for, the gradient
// path never having needed a height. Read here rather than carried down from the
// stage that placed the vertex, because a height is a thing this surface knows
// about and the renderer between them does not: handing it over would put the
// word "elevation" in an interface that is meant to hold whatever a surface
// happens to be.
CdlodSurfaceShading cdlodSurfaceShading(Df3 crudePoint, vec3 crudeDerivX,
                                        vec3 crudeDerivY) {
   vec3 crude = df3ToVec(crudePoint);
   vec3 sphereNormal = normalize(crude);

   // The projection carried through to the derivatives: a step along the outward
   // direction slides the crude point without moving its image on the sphere, and
   // what is left of the step is scaled by how far the projection pulled the
   // point in. Float, and a factor of two in it would cost half a mip level.
   float pullIn = k_radiusMetres / length(crude);
   vec3 metreDerivX = pullIn * (crudeDerivX - sphereNormal * dot(sphereNormal, crudeDerivX));
   vec3 metreDerivY = pullIn * (crudeDerivY - sphereNormal * dot(sphereNormal, crudeDerivY));

   // The two screen axes reduced to the one number both stages pick a level by:
   // how far this sample stands from the next. The wider axis sets it, so a
   // footprint stretched by perspective is read at the level covering its length
   // rather than its width, which is the side that would alias.
   float sampleSpacing = max(length(metreDerivX), length(metreDerivY));

   vec3 gradient = gradientAt(crudePoint, k_shadingOctaves, sampleSpacing);
   vec3 acrossSphere = gradient - sphereNormal * dot(sphereNormal, gradient);

   CdlodSurfaceShading shading;
   shading.normal = normalize(sphereNormal - acrossSphere);

   const float k_turn = 6.283185307179586;
   float elevation = elevationAt(crudePoint, k_shadingOctaves, sampleSpacing);
   shading.colour = vec3(0.5 + 0.5 * sin(elevation * (k_turn / k_colourBandMetres)));

   // Rise over run, which the tangential gradient already is.
   shading.roughness =
      mix(k_flatRoughness, k_steepRoughness,
          smoothstep(k_roughnessSlopeFrom, k_roughnessSlopeTo, length(acrossSphere)));

   return shading;
}
