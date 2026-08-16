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
// Sampled with texture(), whose level the two stages resolve differently and
// deliberately. A vertex stage has no derivatives and so takes level 0, which is
// what geometry requires: the elevation must be a pure function of position
// across every quadtree level, or two patches meeting at different levels would
// displace their shared edge to different places and reopen the seam that
// morphing exists to close. The fragment stage does have derivatives and picks
// the level its pixel covers, which is what keeps per-pixel normals from
// sparkling once the terrain's detail falls below a pixel, and what keeps the
// lookups inside the texture cache when the body is small on screen.
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
// amplitude as a fraction of the relief, and a shift in tiles. Thirty-two to a
// step, and amplitude runs as the reciprocal of frequency, which leaves every
// octave the same rise over run -- the terrain gains detail without gaining
// slope, so the quadtree's ranges answer for all of them at once.
//
// The coarsest is half the map's own scale, twice its height: the tile is what
// the field was built at, not what the body wants to wear it at.
//
// Frequencies and amplitudes are powers of two, exact in a float, so the wide
// scale a lookup is taken at is exact as well. The shift is what keeps the
// layers off one lattice: each repeats a whole number of times per coarser tile
// and would otherwise land on it with the same phase, every repeat reinforcing
// the last.
const int k_octaveCount = 3;
const vec4 k_octaves[k_octaveCount] = vec4[k_octaveCount](
   vec4(0.5, 2.0, 0.0, 0.0),
   vec4(16.0, 1.0 / 16.0, 0.37, 0.71),
   vec4(512.0, 1.0 / 512.0, 0.61, 0.19));

// Octaves the drawn geometry carries. PlanetSurface.cpp must use this same
// count: it is the surface the quadtree's bounds are measured on, and the
// vertex and depth stages both reach it through here.
const int k_positionOctaves = 2;

// Octaves the shading carries, free to go finer than the geometry does. A
// vertex stage takes level 0 and places vertices a quad apart, so detail below
// a quad reaches it as noise; a fragment stage has derivatives to choose a mip
// level with and resolves what a pixel covers.
const int k_shadingOctaves = 3;

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
float sampleElevation(Df2 planeCoord, Df tilesPerMetre, vec2 shift) {
   vec2 tileUv = df2FractToVec(df2Scale(planeCoord, tilesPerMetre)) + shift;
   return texture(u_noiseMap, tileUv).r * k_reliefMetres;
}

// Gradient of one plane's elevation, in metres per metre.
//
// The map holds the gradient as it stands, per unit of tile, matching the field
// it was differenced from. So the same two constants that give the elevation its
// metres give the gradient its own, and their ratio is exactly rise over run.
//
// Reduced to its tile while wide, exactly as the elevation above is and for the
// same reason. That leaves the coordinate discontinuous at every tile boundary,
// which is why the mip level comes from derivatives handed in rather than from
// the coordinate itself: differencing this one across a pixel quad would read
// the wrap as an infinite slope and pick the coarsest level along a line through
// every tile.
vec2 sampleSlope(Df2 planeCoord, Df tilesPerMetre, vec2 tileDerivX, vec2 tileDerivY,
                 vec4 octave) {
   vec2 tileUv = df2FractToVec(df2Scale(planeCoord, tilesPerMetre)) + octave.zw;
   vec2 perTile = textureGrad(u_gradientMap, tileUv, tileDerivX, tileDerivY).rg;
   return perTile * (k_reliefMetres * octave.y * dfToFloat(tilesPerMetre));
}

// Weights from the narrowed position: they follow the direction only, and a
// height is at most the relief. Shared across the octaves, being a property of
// where on the body the point sits rather than of any one layer.
float elevationAt(Df3 spherePosition, int octaveCount) {
   vec3 weights = triplanarWeights(df3ToVec(spherePosition));

   // Both operands are whole numbers a float holds exactly, so the reciprocal is
   // as good as the wide divide can make it. Taken once for every lookup below.
   Df tilesPerMetre =
      dfDiv(dfFromFloat(k_tilesPerSpan), dfFromFloat(k_tileSpanMetres));

   float elevation = 0.0;
   for (int octave = 0; octave < octaveCount; ++octave) {
      Df scale = dfMul(tilesPerMetre, dfFromFloat(k_octaves[octave].x));
      vec2 shift = k_octaves[octave].zw;

      elevation += k_octaves[octave].y
         * (weights.x * sampleElevation(Df2(spherePosition.hi.yz, spherePosition.lo.yz),
                                        scale, shift)
          + weights.y * sampleElevation(Df2(spherePosition.hi.zx, spherePosition.lo.zx),
                                        scale, shift)
          + weights.z * sampleElevation(Df2(spherePosition.hi.xy, spherePosition.lo.xy),
                                        scale, shift));
   }

   return elevation;
}

// Gradient of the elevation above, in the body's own frame. The metre
// derivatives say how far a neighbouring pixel sits on the sphere, and are what
// each lookup's mip level is chosen from.
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
vec3 gradientAt(Df3 spherePosition, vec3 metreDerivX, vec3 metreDerivY,
                int octaveCount) {
   vec3 weights = triplanarWeights(df3ToVec(spherePosition));

   Df tilesPerMetre =
      dfDiv(dfFromFloat(k_tilesPerSpan), dfFromFloat(k_tileSpanMetres));

   vec3 gradient = vec3(0.0);
   for (int octave = 0; octave < octaveCount; ++octave) {
      Df scale = dfMul(tilesPerMetre, dfFromFloat(k_octaves[octave].x));
      float tileScale = dfToFloat(scale);

      vec2 slopeX = sampleSlope(Df2(spherePosition.hi.yz, spherePosition.lo.yz), scale,
                                metreDerivX.yz * tileScale, metreDerivY.yz * tileScale,
                                k_octaves[octave]);
      vec2 slopeY = sampleSlope(Df2(spherePosition.hi.zx, spherePosition.lo.zx), scale,
                                metreDerivX.zx * tileScale, metreDerivY.zx * tileScale,
                                k_octaves[octave]);
      vec2 slopeZ = sampleSlope(Df2(spherePosition.hi.xy, spherePosition.lo.xy), scale,
                                metreDerivX.xy * tileScale, metreDerivY.xy * tileScale,
                                k_octaves[octave]);

      gradient += weights.x * vec3(0.0, slopeX.x, slopeX.y)
                + weights.y * vec3(slopeY.y, 0.0, slopeY.x)
                + weights.z * vec3(slopeZ.x, slopeZ.y, 0.0);
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
Df3 cdlodSurfacePoint(Df3 crudePoint) {
   Df3 outward = df3Normalize(crudePoint);
   Df3 spherePosition = df3Scale(outward, dfFromFloat(k_radiusMetres));

   return df3Add(
      spherePosition,
      df3Scale(outward, dfFromFloat(elevationAt(spherePosition, k_positionOctaves))));
}

// The unit normal of that surface. Only the tangential part of the gradient tilts
// it; the radial part moves the point without turning it. The tangent stretch as
// the surface rises is left out -- it scales the tilt by 1 / (1 + height/radius),
// a part in a hundred thousand against a planet.
//
// Takes more octaves than the geometry does, so the normal is the gradient of a
// finer surface than the one drawn. That is the point: the layers past
// k_positionOctaves exist only in the shading, and are what a pixel can resolve
// and a quad cannot.
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

   vec3 gradient = gradientAt(spherePosition, metreDerivX, metreDerivY, k_shadingOctaves);

   return normalize(sphereNormal - (gradient - sphereNormal * dot(sphereNormal, gradient)));
}
