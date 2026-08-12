// triplanar_noise_surface.glsl
//
// Terrain read out of a baked, tileable height map instead of evaluated in
// closed form. The map is flat and square; this is what wraps it onto a sphere.
//
// Three planar projections, one per axis, blended by how squarely the surface
// faces each. A point's projection onto the x plane is simply its y and z, with
// no divide by x: dividing would be the cube-map parameterization, whose uv runs
// off to infinity as x approaches zero and whose scale varies across the face.
// The raw coordinates keep the terrain the same size everywhere and the
// derivative bounded, and the weight retires each plane long before its
// projection degenerates.
//
// Sampled with textureLod at level 0 throughout. The elevation must be a pure
// function of position across every quadtree level, or two patches meeting at
// different levels would displace their shared edge to different places and
// reopen the seam that morphing exists to close.

uniform sampler2D u_elevationMap;  // R16 unorm, one tile
uniform sampler2D u_slopeMap;      // RG16 unorm, squashed slope, same tile

uniform float u_tileSizeMetres;    // metres one tile of the map spans
uniform float u_elevationMinMetres; // metres the unorm 0 of the map stands for
uniform float u_elevationRangeMetres;

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
   float unorm = textureLod(u_elevationMap, planeCoord / u_tileSizeMetres, 0.0).r;
   return u_elevationMinMetres + unorm * u_elevationRangeMetres;
}

// Undoes the bake's e = g / (1 + |g|). Fixed point cannot hold an unbounded
// slope, and the squash also spends its bits where they turn the normal most:
// the normal's angle is atan(g), which moves fastest per unit of slope near flat.
vec2 sampleSlope(vec2 planeCoord) {
   vec2 encoded = textureLod(u_slopeMap, planeCoord / u_tileSizeMetres, 0.0).rg * 2.0 - 1.0;
   return encoded / (1.0 - abs(encoded));
}

float cdlodSurfaceElevation(vec3 spherePosition) {
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
// outward direction, which cdlodDisplacedNormal discards anyway, plus a
// tangential term proportional to the difference between the planes' elevations
// -- real, but confined to the bands where the blend is already fading one
// unrelated piece of terrain into another.
vec3 cdlodSurfaceGradient(vec3 spherePosition) {
   vec3 weights = triplanarWeights(spherePosition);

   vec2 slopeX = sampleSlope(spherePosition.yz);
   vec2 slopeY = sampleSlope(spherePosition.zx);
   vec2 slopeZ = sampleSlope(spherePosition.xy);

   return weights.x * vec3(0.0, slopeX.x, slopeX.y)
        + weights.y * vec3(slopeY.y, 0.0, slopeY.x)
        + weights.z * vec3(slopeZ.x, slopeZ.y, 0.0);
}
