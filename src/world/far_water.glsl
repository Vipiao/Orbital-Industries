// far_water.glsl
//
// The sea seen from far above, where the mesh is too coarse to cut a sharp
// shoreline out of media/planet/water_body.glsl's sphere. Drawn below the shore at
// each pixel's own height and handed to the lighting stage finished, as emissive,
// with the surface of sea_surface.glsl once all its waves are too narrow to draw.
// Lit at full strength, since the camera is never under water while this shows.
#include "../../media/planet/sea_surface.glsl"

// The camera in metres from the body's centre, the direction toward the sun, and
// the sky's colour, in the body's frame
uniform vec3 u_farWaterCamera;
uniform vec3 u_farWaterToLight;
uniform vec3 u_farWaterSky;

// How much of the far sea shows, by the camera's height above it
float farWaterShare() {
   float height = length(u_farWaterCamera) - (k_radiusMetres + k_shoreMetres);
   return smoothstep(k_farSeaFrom, k_farSeaWhole, height);
}

// Turns the ground depth metres under the sea into the sea over it, share of the
// way. up is the sea's normal and point where the view meets it, from the body's
// centre.
void shadeFarWater(inout CdlodSurfaceShading shading, vec3 up, vec3 point, float depth,
                   float share) {
   vec3 toCamera = normalize(u_farWaterCamera - point);
   float cosView = max(dot(up, toCamera), 0.01);

   // The ground as the lighting stage would light it, tinted as the body tints it
   vec3 ground = k_bodyTint * shading.colour
               * seaLight(shading.normal, u_farWaterToLight, 1.0, 1.0);

   // Beer-Lambert along the view ray, down to the ground
   vec3 transmitted = exp(-k_waterAbsorptionPerMetre * depth / cosView);
   vec3 below = mix(k_waterColor * seaLight(up, u_farWaterToLight, 1.0, 1.0), ground,
                    transmitted);

   float mirrored;
   vec3 sea = seaFromAbove(below, up, toCamera, u_farWaterToLight,
                           seaRoughness(k_allWavesVariance), u_farWaterSky, 1.0, mirrored);

   // Emissive from the first sign of the sea, fading in from the ground lit the
   // same way; water_body.glsl covers the switch. The body's tint is applied after.
   shading.colour = mix(ground, sea, share) / k_bodyTint;
   shading.emissive = 1.0;
}
