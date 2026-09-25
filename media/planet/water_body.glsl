// water_body.glsl
//
// A planet's water: a sea-level sphere that absorbs light along the view ray,
// under a flat surface that reflects by Fresnel from either side. value.xyz is
// the absorption per metre in each channel, value.w the sea-level radius, and
// color.rgb the deep-water colour at full ambient light.
//
// The surface does not refract yet; from below it still turns to a mirror past
// the critical angle.
//
// A float resolves only half a metre at a planet's radius, so the intersection
// is written around the camera's height above the water.

const float k_waterIor = 1.33;
// Reflectance head-on, ((n - 1) / (n + 1))^2
const float k_waterF0 = 0.02;
// Spreads the sun's glint on a flat sea to about the sun's own width
const float k_surfaceRoughness = 0.1;

// The normalized Blinn-Phong lobe of phong_lighting.glsl
float sunLobe(float cosHalf) {
   float exponent = blinnExponent(k_surfaceRoughness);
   return (exponent + 8.0) / (8.0 * k_pi) * pow(max(cosHalf, 0.0), exponent);
}

float reflectance(float cosTheta) {
   return fresnelSchlick(vec3(k_waterF0), cosTheta).r;
}

RayVolumeResult rayVolumeShade(
   vec3 viewPos, vec3 rayDir, float backDepth, float sceneDepth,
   vec3 opaqueColor, vec4 value, vec4 color, vec2 uv,
   vec3 centerViewPos, mat3 viewBasis)
{
   RayVolumeResult res;
   res.color = vec3(0.0);
   res.alpha = 0.0;
   res.weightDepth = 1.0;

   vec3 absorption = value.xyz;
   float radius = value.w;

   // Camera height above sea level, negative under water. hi - radius is exact
   // (Sterbenz) at any height where it matters; precise keeps lo from being
   // folded into radius first.
   Df wideDist = centerDistance();
   precise float aboveHi = wideDist.hi - radius;
   precise float height = aboveHi + wideDist.lo;
   float centreDist = wideDist.hi;

   // Cosine between the ray and the local up, and the matching sine
   vec3 up = -normalize(centerViewPos);
   float mu = dot(rayDir, up);
   float sine = sqrt(max(1.0 - mu * mu, 0.0));

   // radius - centreDist * sine, how far inside the sphere the ray passes,
   // rewritten to subtract no two large lengths
   float passInside = centreDist * mu * mu / (1.0 + sine) - height;
   if (passInside <= 0.0) {
      return res;
   }
   float halfChord = sqrt(passInside * (radius + centreDist * sine));

   // Roots of t^2 + 2 centreDist mu t + heightTerm = 0. A root that would
   // cancel is taken from the other through their product, heightTerm.
   float heightTerm = height * (2.0 * radius + height);
   float entry;
   float exit;
   if (height > 0.0) {
      if (mu >= 0.0) {
         return res;   // heading away from the water
      }
      exit = -centreDist * mu + halfChord;
      entry = heightTerm / exit;
   } else {
      entry = 0.0;
      exit = mu > 0.0 ? -heightTerm / (halfChord + centreDist * mu)
                      : -centreDist * mu + halfChord;
   }

   // Distance along the ray to the opaque scene (depth is measured along -z)
   float tScene = sceneDepth / max(-rayDir.z, 1e-4);

   float path = min(exit, tScene) - entry;
   if (path <= 0.0) {
      return res;
   }

   // Beer-Lambert: what survives the path is exp(-absorption * path), in each
   // channel apart
   vec3 transmitted = exp(-absorption * path);
   float leastTransmitted = min(transmitted.r, min(transmitted.g, transmitted.b));
   vec3 waterColor = color.rgb * u_ambientScale;
   vec3 toLight = -normalize(u_lightDir);

   // target is what the pixel shows; kept is the share of the scene behind that
   // every channel keeps at least
   vec3 target;
   float kept;
   if (height > 0.0) {
      // From above, the surface at entry mirrors the sky and lets the rest in
      vec3 normal = normalize(rayDir * entry - centerViewPos);
      vec3 toCamera = -rayDir;
      float mirrored = reflectance(dot(normal, toCamera));
      vec3 halfway = normalize(toLight + toCamera);
      // The sun's peak runs to hundreds, enough to overflow the fp16 blend
      // targets, so both of its terms are capped at white, which the display
      // clips to anyway
      vec3 glint = min(fresnelSchlick(vec3(k_waterF0), dot(halfway, toCamera)) *
         sunLobe(dot(normal, halfway)) * max(dot(normal, toLight), 0.0) * u_directScale,
         vec3(1.0));
      vec3 below = mix(waterColor, opaqueColor, transmitted);
      target = mix(below, u_skyColor, mirrored) + glint;
      kept = (1.0 - mirrored) * leastTransmitted;
   } else if (mu > 0.0 && exit < tScene) {
      // From below, the surface at exit, its normal facing the camera. Past the
      // critical angle refract returns zero and the surface reflects everything.
      vec3 normal = normalize(centerViewPos - rayDir * exit);
      vec3 refracted = refract(rayDir, normal, k_waterIor);
      float mirrored = 1.0;
      vec3 above = vec3(0.0);
      if (refracted != vec3(0.0)) {
         mirrored = reflectance(dot(refracted, -normal));
         // The sun seen along the ray out, as wide as its glint from above
         float sun = sunLobe(dot(normalize(refracted + toLight), toLight));
         above = opaqueColor + vec3(min(sun * u_directScale, 1.0));
      }
      // What the surface mirrors from below is deep water
      vec3 surface = mix(above, waterColor, mirrored);
      target = mix(waterColor, surface, transmitted);
      kept = (1.0 - mirrored) * leastTransmitted;
   } else {
      target = mix(waterColor, opaqueColor, transmitted);
      kept = leastTransmitted;
   }

   // One alpha serves all three channels: it covers what no channel keeps, and
   // the colour gives each channel back its extra share. Exact for a single layer.
   res.alpha = 1.0 - kept;
   res.color = (target - opaqueColor * kept) / max(res.alpha, 1e-6);
   res.weightDepth = entry * max(-rayDir.z, 1e-4);
   return res;
}
