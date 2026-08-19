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
// Three planar projections, one per axis, blended by how squarely the surface
// faces each. A point's projection onto the x plane is simply its y and z, with
// no divide by x: dividing would be the cube-map parameterization, whose uv runs
// off to infinity as x approaches zero and whose scale varies across the face.
// The raw coordinates keep the terrain the same size everywhere and the
// derivative bounded, and the weight retires each plane long before its
// projection degenerates.
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
// Both paths take the point in a wide float, and both reduce it to its tile
// before narrowing. A point on this sphere is 6.4e6 metres from the origin,
// where a float's spacing is half a metre -- two hundred texels of the finest
// octave's map -- so a lookup taken from a narrow coordinate reads that lattice
// rather than the terrain. What stays narrow is everything downstream of the
// lookup: weights, slopes and the normal are directions and heights, not
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
const float k_reliefMetres = 400.0;

// How sharply the three projections give way to each other. Raising it narrows
// the band where two planes both contribute, which is where their unrelated
// terrain has to cross-fade and detail visibly softens.
const float k_blendSharpness = 6.0;

// The same map laid down at several scales and added up: frequency multiplier,
// amplitude as a fraction of the relief, and a shift in tiles. Sixteen to a
// step, and amplitude runs as the reciprocal of frequency, which leaves every
// octave the same rise over run -- the terrain gains detail without gaining
// slope, so the quadtree's ranges answer for all of them at once.
//
// The coarsest stretches the map over four of its own tiles at four times its
// height: the tile is what the field was built at, not what the body wants to
// wear it at.
//
// Frequencies and amplitudes are powers of two, exact in a float, so the wide
// scale a lookup is taken at is exact as well. The shift is what keeps the
// layers off one lattice: each repeats a whole number of times per coarser tile
// and would otherwise land on it with the same phase, every repeat reinforcing
// the last.
const int k_octaveCount = 4;
const vec4 k_octaves[k_octaveCount] = vec4[k_octaveCount](
   vec4(0.25, 4.0, 0.0, 0.0),
   vec4(4.0, 1.0 / 4.0, 0.37, 0.71),
   vec4(64.0, 1.0 / 64.0, 0.61, 0.19),
   vec4(1024.0, 1.0 / 1024.0, 0.13, 0.44));

// Octaves the drawn geometry carries. PlanetSurface.cpp must use this same
// count: it is the surface the quadtree's bounds are measured on, and the
// vertex and depth stages both reach it through here.
const int k_positionOctaves = 2;

// Octaves the shading carries. Free to differ from the count above -- both read
// the same octaves and each stops where its own sampling does -- but matched to
// it, so the normal is the gradient of the surface actually drawn rather than of
// a finer or coarser one.
const int k_shadingOctaves = 2;

// Weight of each plane, summing to one. Driven by the direction rather than the
// position, so the split is a property of where on the body a point sits.
vec3 triplanarWeights(vec3 spherePosition) {
   vec3 weights = pow(abs(normalize(spherePosition)), vec3(k_blendSharpness));
   return weights / (weights.x + weights.y + weights.z);
}

// Reduced to its tile before it is narrowed. The map wraps, so the fraction is
// all that was ever read, and millions of whole tiles would otherwise swamp it.
// Two patches meeting at a level boundary then read the same texel.
//
// The shift rides on the fraction rather than the coordinate, where it is under
// a tile and costs nothing to hold, and the sampler's own wrap takes it home.
// Full amplitude: the caller scales, having one weighted sum per octave to do it
// to rather than three.
//
// The level is given rather than derived: this is read from stages that have no
// derivatives, and the caller knows how far apart its samples stand.
float sampleElevation(Df2 planeCoord, Df tilesPerMetre, vec2 shift, float mipLevel) {
   vec2 tileUv = df2FractToVec(df2Scale(planeCoord, tilesPerMetre)) + shift;
   return textureLod(u_noiseMap, tileUv, mipLevel).r * k_reliefMetres;
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

// Gradient of one plane's elevation, in metres per metre.
//
// The map holds the gradient as it stands, per unit of tile, matching the field
// it was differenced from. So the same two constants that give the elevation its
// metres give the gradient its own, and their ratio is exactly rise over run.
//
// Reduced to its tile while wide, exactly as the elevation above is and for the
// same reason. That leaves the coordinate discontinuous at every tile boundary,
// so the level cannot be left to the sampler: differencing this one across a
// pixel quad would read the wrap as an infinite slope and pick the coarsest
// level along a line through every tile. It is given, as sampleElevation's is,
// and by a caller that knows how far apart its samples stand.
//
// Full amplitude, again as sampleElevation is: the caller scales, having one
// weighted sum per octave to do it to rather than three.
vec2 sampleSlope(Df2 planeCoord, Df tilesPerMetre, vec2 shift, float mipLevel) {
   vec2 tileUv = df2FractToVec(df2Scale(planeCoord, tilesPerMetre)) + shift;
   vec2 perTile = textureLod(u_gradientMap, tileUv, mipLevel).rg;
   return perTile * (k_reliefMetres * dfToFloat(tilesPerMetre));
}

// Weights from the narrowed position: they follow the direction only, and a
// height is at most the relief. Shared across the octaves, being a property of
// where on the body the point sits rather than of any one layer.
//
// sampleSpacing is how far apart the samples reading this stand on the ground.
// Every octave is read at the level that spacing resolves, so a layer finer than
// the samples arrives as its own average rather than as whichever point of it
// each sample landed on. What that removes was never terrain: it was the noise a
// lattice makes.
float elevationAt(Df3 spherePosition, int octaveCount, float sampleSpacing) {
   vec3 weights = triplanarWeights(df3ToVec(spherePosition));

   // Both operands are whole numbers a float holds exactly, so the reciprocal is
   // as good as the wide divide can make it. Taken once for every lookup below.
   Df tilesPerMetre =
      dfDiv(dfFromFloat(k_tilesPerSpan), dfFromFloat(k_tileSpanMetres));

   float elevation = 0.0;
   for (int octave = 0; octave < octaveCount; ++octave) {
      Df scale = dfMul(tilesPerMetre, dfFromFloat(k_octaves[octave].x));
      vec2 shift = k_octaves[octave].zw;
      float mipLevel = octaveMipLevel(sampleSpacing, k_octaves[octave].x);

      elevation += k_octaves[octave].y
         * (weights.x * sampleElevation(Df2(spherePosition.hi.yz, spherePosition.lo.yz),
                                        scale, shift, mipLevel)
          + weights.y * sampleElevation(Df2(spherePosition.hi.zx, spherePosition.lo.zx),
                                        scale, shift, mipLevel)
          + weights.z * sampleElevation(Df2(spherePosition.hi.xy, spherePosition.lo.xy),
                                        scale, shift, mipLevel));
   }

   return elevation;
}

// Gradient of the elevation above, in the body's own frame. Reads at the level
// the spacing given resolves, exactly as elevationAt does and through the same
// function, the two stages differing only in how they arrive at a spacing.
//
// Each plane's map varies with only two of the three coordinates, so its
// gradient has a zero in the axis it was projected along -- the x plane's
// contribution is (0, dh/dy, dh/dz). Blending the three as vectors is exact in
// a way blending normals would not be: a gradient is linear, a normalized
// normal is not.
//
// The weights' own variation is left out. Keeping it would add a term along the
// outward direction, which cdlodSurfaceNormal discards anyway, plus a
// tangential term proportional to the difference between the planes' elevations
// -- real, but confined to the bands where the blend is already fading one
// unrelated piece of terrain into another.
vec3 gradientAt(Df3 spherePosition, int octaveCount, float sampleSpacing) {
   vec3 weights = triplanarWeights(df3ToVec(spherePosition));

   Df tilesPerMetre =
      dfDiv(dfFromFloat(k_tilesPerSpan), dfFromFloat(k_tileSpanMetres));
   float topLevel = mapTopLevel();

   vec3 gradient = vec3(0.0);
   for (int octave = 0; octave < octaveCount; ++octave) {
      float mipLevel = octaveMipLevel(sampleSpacing, k_octaves[octave].x);

      // Past the top the map returns the tile's mean, and the mean gradient of a
      // field that wraps is zero: the octave has nothing left to tilt a normal
      // with, so its three lookups are skipped rather than summed to nothing.
      // The height path has no such exit, a mean elevation being a real offset.
      if (mipLevel >= topLevel) {
         continue;
      }

      Df scale = dfMul(tilesPerMetre, dfFromFloat(k_octaves[octave].x));
      vec2 shift = k_octaves[octave].zw;

      vec2 slopeX = sampleSlope(Df2(spherePosition.hi.yz, spherePosition.lo.yz),
                                scale, shift, mipLevel);
      vec2 slopeY = sampleSlope(Df2(spherePosition.hi.zx, spherePosition.lo.zx),
                                scale, shift, mipLevel);
      vec2 slopeZ = sampleSlope(Df2(spherePosition.hi.xy, spherePosition.lo.xy),
                                scale, shift, mipLevel);

      gradient += k_octaves[octave].y
         * (weights.x * vec3(0.0, slopeX.x, slopeX.y)
          + weights.y * vec3(slopeY.y, 0.0, slopeY.x)
          + weights.z * vec3(slopeZ.x, slopeZ.y, 0.0));
   }

   return gradient;
}

// The cube the tree subdivides, projected onto the sphere: the only place crude
// space is given a meaning. The position path inlines this, wanting the outward
// direction the projection passes through anyway.
Df3 spherePointOf(Df3 crudePoint) {
   return df3Scale(df3Normalize(crudePoint), dfFromFloat(k_radiusMetres));
}

// Where a crude point is drawn: the sphere, raised along its outward direction.
// A scalar height rather than a free displacement, so the normal below can be
// its gradient rather than a second opinion about the same surface.
//
// The sphere term is the body-sized one and carries the width; the height rides
// on it as a float, being at most the relief. Two patches meeting at a level
// boundary reach the same crude point to a nanometre, so the seam the morph
// closes in principle closes in the arithmetic as well.
//
// The outward direction is taken once and used twice: the projection is that
// direction scaled to the radius, so normalizing the result would ask the same
// question again -- and an inverse square root at this width has no hardware
// behind it.
Df3 cdlodSurfacePoint(Df3 crudePoint, float sampleSpacing) {
   Df3 outward = df3Normalize(crudePoint);
   Df3 spherePosition = df3Scale(outward, dfFromFloat(k_radiusMetres));

   return df3Add(
      spherePosition,
      df3Scale(outward, dfFromFloat(
         elevationAt(spherePosition, k_positionOctaves, sampleSpacing))));
}

// The unit normal of that surface. Only the tangential part of the gradient tilts
// it; the radial part moves the point without turning it. The tangent stretch as
// the surface rises is left out -- it scales the tilt by 1 / (1 + height/radius),
// a part in a hundred thousand against a planet.
//
// Reads the octaves the geometry carries, so this is the gradient of the surface
// drawn rather than of a finer one. A pixel still resolves more of them than a
// quad does: both stop where their own sampling stops, and the fragment stage
// samples the finer of the two.
vec3 cdlodSurfaceNormal(Df3 crudePoint, vec3 crudeDerivX, vec3 crudeDerivY) {
   Df3 spherePosition = spherePointOf(crudePoint);
   vec3 sphereNormal = normalize(df3ToVec(spherePosition));

   // The projection carried through to the derivatives: a step along the outward
   // direction slides the crude point without moving its image on the sphere, and
   // what is left of the step is scaled by how far the projection pulled the
   // point in. Float, and a factor of two in it would cost half a mip level.
   float pullIn = k_radiusMetres / length(df3ToVec(crudePoint));
   vec3 metreDerivX = pullIn * (crudeDerivX - sphereNormal * dot(sphereNormal, crudeDerivX));
   vec3 metreDerivY = pullIn * (crudeDerivY - sphereNormal * dot(sphereNormal, crudeDerivY));

   // The two screen axes reduced to the one number both stages pick a level by:
   // how far this sample stands from the next. The wider axis sets it, so a
   // footprint stretched by perspective is read at the level covering its length
   // rather than its width, which is the side that would alias.
   float sampleSpacing = max(length(metreDerivX), length(metreDerivY));

   vec3 gradient = gradientAt(spherePosition, k_shadingOctaves, sampleSpacing);

   return normalize(sphereNormal - (gradient - sphereNormal * dot(sphereNormal, gradient)));
}
