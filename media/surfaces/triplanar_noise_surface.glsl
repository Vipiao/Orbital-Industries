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
// The position runs in a wide float and the normal in an ordinary one. A point
// on this sphere is 6.4e6 metres from the origin, where a float's spacing is
// half a metre; a normal is only a direction. The twin is double throughout,
// being under no such pressure.
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

// Weight of each plane, summing to one. Driven by the direction rather than the
// position, so the split is a property of where on the body a point sits.
vec3 triplanarWeights(vec3 spherePosition) {
   vec3 weights = pow(abs(normalize(spherePosition)), vec3(k_blendSharpness));
   return weights / (weights.x + weights.y + weights.z);
}

// Reduced to its tile before it is narrowed. The map wraps, so the fraction is
// all that was ever read, and thousands of whole tiles would otherwise swamp it.
// Two patches meeting at a level boundary then read the same texel.
float sampleElevation(Df2 planeCoord, Df tilesPerMetre) {
   vec2 tileUv = df2FractToVec(df2Scale(planeCoord, tilesPerMetre));
   return texture(u_noiseMap, tileUv).r * k_reliefMetres;
}

// Gradient of one plane's elevation, in metres per metre.
//
// The map holds the gradient as it stands, per unit of tile, matching the field
// it was differenced from. So the same two constants that give the elevation its
// metres give the gradient its own, and their ratio is exactly rise over run.
// Float, and per pixel: this feeds a normal, which the last bits of a plane
// coordinate barely move.
vec2 sampleSlope(vec2 planeCoord) {
   float tilesPerMetre = k_tilesPerSpan / k_tileSpanMetres;
   vec2 perTile = texture(u_gradientMap, planeCoord * tilesPerMetre).rg;
   return perTile * (k_reliefMetres * tilesPerMetre);
}

// Weights from the narrowed position: they follow the direction only, and a
// height is at most the relief.
float elevationAt(Df3 spherePosition) {
   vec3 weights = triplanarWeights(df3ToVec(spherePosition));

   // Both operands are whole numbers a float holds exactly, so the reciprocal is
   // as good as the wide divide can make it. Taken once for all three planes.
   Df tilesPerMetre =
      dfDiv(dfFromFloat(k_tilesPerSpan), dfFromFloat(k_tileSpanMetres));

   return weights.x * sampleElevation(Df2(spherePosition.hi.yz, spherePosition.lo.yz),
                                      tilesPerMetre)
        + weights.y * sampleElevation(Df2(spherePosition.hi.zx, spherePosition.lo.zx),
                                      tilesPerMetre)
        + weights.z * sampleElevation(Df2(spherePosition.hi.xy, spherePosition.lo.xy),
                                      tilesPerMetre);
}

// Gradient of the elevation above, in the body's own frame.
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
vec3 gradientAt(vec3 spherePosition) {
   vec3 weights = triplanarWeights(spherePosition);

   vec2 slopeX = sampleSlope(spherePosition.yz);
   vec2 slopeY = sampleSlope(spherePosition.zx);
   vec2 slopeZ = sampleSlope(spherePosition.xy);

   return weights.x * vec3(0.0, slopeX.x, slopeX.y)
        + weights.y * vec3(slopeY.y, 0.0, slopeY.x)
        + weights.z * vec3(slopeZ.x, slopeZ.y, 0.0);
}

// The cube the tree subdivides, projected onto the sphere: the only place crude
// space is given a meaning. The position path inlines this, wanting the outward
// direction the projection passes through anyway.
vec3 spherePointOf(vec3 crudePoint) {
   return normalize(crudePoint) * k_radiusMetres;
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

   return df3Add(spherePosition,
                 df3Scale(outward, dfFromFloat(elevationAt(spherePosition))));
}

// The unit normal of that surface. Only the tangential part of the gradient tilts
// it; the radial part moves the point without turning it. The tangent stretch as
// the surface rises is left out -- it scales the tilt by 1 / (1 + height/radius),
// a part in a hundred thousand against a planet.
vec3 cdlodSurfaceNormal(vec3 crudePoint) {
   vec3 spherePosition = spherePointOf(crudePoint);
   vec3 sphereNormal = normalize(spherePosition);
   vec3 gradient = gradientAt(spherePosition);

   return normalize(sphereNormal - (gradient - sphereNormal * dot(sphereNormal, gradient)));
}
