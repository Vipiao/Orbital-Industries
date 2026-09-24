// water_body.glsl
//
// A planet's water: a sea-level sphere absorbing light by the distance the view
// ray travels through it before the opaque scene. No surface shading yet.
// value.x is the absorption per metre, value.y the sea-level radius.
//
// A float resolves only half a metre at a planet's radius, so the intersection
// is written around the camera's height above the water, taken from the wide
// centre distance; the rest needs only relative precision.

RayVolumeResult rayVolumeShade(
   vec3 viewPos, vec3 rayDir, float backDepth, float sceneDepth,
   vec3 opaqueColor, vec4 value, vec4 color, vec2 uv,
   vec3 centerViewPos, mat3 viewBasis)
{
   RayVolumeResult res;
   res.color = color.rgb;
   res.alpha = 0.0;
   res.weightDepth = 1.0;

   float absorption = value.x;
   float radius = value.y;

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

   // Beer-Lambert: what survives the path is exp(-absorption * path)
   res.alpha = 1.0 - exp(-absorption * path);
   res.weightDepth = entry * max(-rayDir.z, 1e-4);
   return res;
}
