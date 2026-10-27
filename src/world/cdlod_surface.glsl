// cdlod_surface.glsl
//
// The ground as the quadtree draws it: cdlodSurfacePoint and cdlodSurfaceShading
// for the engine's cdlod_patch.glsl. Adds only drawing effects on top of
// lattice_surface.glsl (far sea flattening, underwater wobble), so it has no CPU
// twin. Preceded by planetSurfaceGlsl and farWaterGlsl.
#include "lattice_surface.glsl"
#include "far_water.glsl"

// Scales the underwater warp: one with the camera above the sea, zero below it,
// where the surface the warp stands in for is out of view
uniform float u_underwaterWarp;

// Ground under water wobbles as if seen through moving water. No refraction: each
// vertex is pushed along a sinusoid, further the deeper it lies, so the sea floor
// looks alive without a screen-space pass.
//
// Cycles per metre of the pattern along each body axis. A power of two, so a
// point's high part times it is exact in a float and its fraction keeps full
// precision at the body's radius.
const float k_warpCyclesPerMetre = 0.25;
// Cycles the phase advances per metre of depth
const float k_warpCyclesPerDepth = 0.1;
// Metres of warp per metre of depth, and the depth past which it stops growing
const float k_warpPerDepth = 0.04;
const float k_warpDepthCap = 3.0;
// Whole cycles per time wrap, so the pattern runs across the wrap without a jump;
// about half a cycle a second
const uint k_warpCyclesPerWrap = 8192u;

// The offset for a point depth metres under water, in the body's frame, strength
// of the way to the full warp at the time the engine's clock gives
vec3 underwaterWarp(Df3 point, float depth, float strength, uint time,
                    float timeRemainder) {
   vec3 spatial = fract(point.hi * k_warpCyclesPerMetre) + point.lo * k_warpCyclesPerMetre;

   uint whole = (time * k_warpCyclesPerWrap) & (k_timeWrapTicks - 1u);
   float timeCycles = fract(
      (float(whole) + timeRemainder * float(k_warpCyclesPerWrap))
      / float(k_timeWrapTicks));

   // Each component driven by the other two axes, so it varies across the ground
   // rather than along its own direction only
   vec3 cycles = timeCycles + spatial.yzx + spatial.zxy + depth * k_warpCyclesPerDepth
               + vec3(0.0, 1.0 / 3.0, 2.0 / 3.0);

   return sin(k_twoPi * cycles)
        * (min(depth, k_warpDepthCap) * k_warpPerDepth * strength);
}

// Where a crude point is drawn: on the ground, except where the far sea is drawn
// over it, and wobbling under the near sea.
Df3 cdlodSurfacePoint(Df3 crudePoint, float sampleSpacing) {
   TerrainDisplacement displacement = terrainDisplacement(
      gatherHeightLevels(crudePoint, k_positionLevels, sampleSpacing));

   // The far sea is drawn flat, so the floor under it rises to the shore
   float depth = k_shoreMetres - displacement.height;
   float flatness = depth > 0.0 ? farWaterFlatness(farWaterCameraHeight(u_farWaterCamera))
                                : 0.0;
   float height = mix(displacement.height, k_shoreMetres, flatness);

   Df3 point = surfacePointAtHeight(crudePoint, height);

   float warp = u_underwaterWarp * (1.0 - flatness);
   if (depth > 0.0 && warp > 0.0) {
      point = df3AddVec(point, underwaterWarp(point, depth, warp, u_time, u_timeRemainder));
   }

   return point;
}

// How the surface faces, what colour it is drawn in, and how tight a highlight
// it carries. Only the tangential part of the gradient tilts the normal; the
// radial part moves the point without turning it. The tangent stretch as the
// surface rises is left out -- it scales the tilt by 1 / (1 + height/radius), a
// part in a hundred thousand against a planet.
//
// Reads the levels the geometry carries, so this is the gradient of the surface
// drawn rather than of a finer one. A pixel still resolves more of them than a
// quad does: both stop where their own sampling stops, and the fragment stage
// samples the finer of the two.
//
// The height the colour bands read comes back alongside the gradient rather than
// carried down from the stage that placed the vertex, because a height is a thing
// this surface knows about and the renderer between them does not: handing it
// over would put the word "elevation" in an interface that is meant to hold
// whatever a surface happens to be.
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

   TerrainLevels levels = gatherLevels(crudePoint, k_shadingLevels, sampleSpacing);
   TerrainDisplacement displacement = terrainDisplacement(levels);
   vec3 acrossSphere = gradientAcrossSphere(displacement.gradient, sphereNormal);

   // Rise over run, which the tangential gradient already is.
   TerrainMaterial material =
      terrainMaterial(levels, displacement.height, length(acrossSphere));

   CdlodSurfaceShading shading;
   shading.normal = normalize(sphereNormal - acrossSphere);
   shading.colour = material.colour;
   shading.roughness = material.roughness;
   shading.emissive = 0.0;

   float depth = k_shoreMetres - displacement.height;
   float farWater = farWaterShare(farWaterCameraHeight(u_farWaterCamera));
   if (depth > 0.0 && farWater > 0.0) {
      shadeFarWater(shading, sphereNormal, sphereNormal * (k_radiusMetres + k_shoreMetres),
                    depth, farWater, u_farWaterCamera, u_farWaterToLight, u_farWaterSky);
   }

   return shading;
}
