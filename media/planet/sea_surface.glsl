// sea_surface.glsl
//
// The sea's surface as water_body.glsl draws it up close and
// src/world/far_water.glsl draws it from far above, held once so the two match
// where one hands over to the other. Reads the engine's phong_lighting.glsl,
// which both stages it is spliced into include.

// Reflectance head-on, ((n - 1) / (n + 1))^2
const float k_waterF0 = 0.02;
// Spreads the sun's glint on a calm sea to about the sun's own width
const float k_surfaceRoughness = 0.1;
// The swell's waves, and the peak slope of each; eight together make an rms slope
// of 0.12, a moderate breeze
const int k_waveCount = 8;
const float k_waveSlope = 0.06;

// Camera heights above sea level. The far sea fades in between the first two and
// the near sea out between the last two, so what shows through the near sea as it
// fades is always the whole far sea.
const float k_farSeaFrom = 2000.0;
const float k_farSeaWhole = 3500.0;
const float k_nearSeaGone = 5000.0;

// The roughness of a sea whose waves have lostVariance of their mean square slope
// to being too narrow to draw. roughness^4 is the mean square slope of the lobe,
// so the lost slope adds there.
float seaRoughness(float lostVariance) {
   return pow(pow(k_surfaceRoughness, 4.0) + lostVariance, 0.25);
}

// The mean square slope of all the waves together
const float k_allWavesVariance = float(k_waveCount) * 0.5 * k_waveSlope * k_waveSlope;

// The normalized Blinn-Phong lobe of phong_lighting.glsl
float sunLobe(float cosHalf, float roughness) {
   float exponent = blinnExponent(roughness);
   return (exponent + 8.0) / (8.0 * k_pi) * pow(max(cosHalf, 0.0), exponent);
}

float reflectance(float cosTheta) {
   return fresnelSchlick(vec3(k_waterF0), cosTheta).r;
}

// What lights a surface facing along normal, as the lighting stage lights flat
// ground
float seaLight(vec3 normal, vec3 toLight, float ambientScale, float directScale) {
   return k_ambient * ambientScale + max(dot(normal, toLight), 0.0) * directScale;
}

// The surface seen from above over below, what shows through it: the sky mirrored
// by Fresnel, and the sun's glint. The sun's peak runs to hundreds, enough to
// overflow the fp16 blend targets, so the glint is capped at white, which the
// display clips to anyway. mirrored is the share of the sky.
vec3 seaFromAbove(vec3 below, vec3 normal, vec3 toCamera, vec3 toLight, float roughness,
                  vec3 sky, float directScale, out float mirrored) {
   mirrored = reflectance(dot(normal, toCamera));
   vec3 halfway = normalize(toLight + toCamera);
   vec3 glint = min(fresnelSchlick(vec3(k_waterF0), dot(halfway, toCamera)) *
      sunLobe(dot(normal, halfway), roughness) * max(dot(normal, toLight), 0.0) *
      directScale, vec3(1.0));
   return mix(below, sky, mirrored) + glint;
}
