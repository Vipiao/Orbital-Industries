// triplanar_noise_surface.glsl
//
// Terrain read out of a baked, tileable noise map instead of evaluated in
// closed form. The map is flat, square and dimensionless; this is what gives it
// a size and a height and wraps it onto a sphere.
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

// Metres from the body's centre to the sphere the cube projects onto.
// CubeSphereBounds is the CPU twin and must carry the same radius, and
// k_reliefMetres below as its ceiling.
const float k_radiusMetres = 6371000.0;

uniform sampler2D u_noiseMap;     // R16 unorm, one tile, spanning exactly [0, 1]
uniform sampler2D u_gradientMap;  // RG16F, gradient per unit of tile, same tile

// Metres one tile of the map spans. Below the body's width the tile repeats,
// which is what puts detail on a planet the map could never cover in one pass.
const float k_tileSizeMetres = 12742000.0 * 0.0001;

// Metres between the map's floor and its ceiling. The map is unsigned, so the
// terrain rises from the sphere rather than straddling it, and this is the full
// depth of the relief.
//
// Tiny against a planet's radius, so the body reads as a sphere and the relief
// shows in the shading rather than the silhouette. The knob to raise for
// exaggerated terrain, and the ceiling CubeSphereBounds pads its spheres by:
// raising it here without raising it there lets the surface escape the bound the
// tree measures to.
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

float sampleElevation(vec2 planeCoord) {
   return texture(u_noiseMap, planeCoord / k_tileSizeMetres).r * k_reliefMetres;
}

// Gradient of one plane's elevation, in metres per metre.
//
// The map holds the gradient as it stands, per unit of tile, matching the field
// it was differenced from. So the same two constants that give the elevation its
// metres give the gradient its own, and their ratio is exactly rise over run.
vec2 sampleSlope(vec2 planeCoord) {
   vec2 perTile = texture(u_gradientMap, planeCoord / k_tileSizeMetres).rg;
   return perTile * k_reliefMetres / k_tileSizeMetres;
}

float elevationAt(vec3 spherePosition) {
   vec3 weights = triplanarWeights(spherePosition);

   return weights.x * sampleElevation(spherePosition.yz)
        + weights.y * sampleElevation(spherePosition.zx)
        + weights.z * sampleElevation(spherePosition.xy);
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
// space is given a meaning.
vec3 spherePointOf(vec3 crudePoint) {
   return normalize(crudePoint) * k_radiusMetres;
}

// Where a crude point is drawn: the sphere, raised along its outward direction.
// A scalar height rather than a free displacement, so the normal below can be
// its gradient rather than a second opinion about the same surface.
vec3 cdlodSurfacePoint(vec3 crudePoint) {
   vec3 spherePosition = spherePointOf(crudePoint);
   return spherePosition + normalize(spherePosition) * elevationAt(spherePosition);
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
