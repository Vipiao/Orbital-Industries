// water_body.glsl
//
// A planet's water: a sea-level sphere that absorbs light along the view ray,
// under a surface that reflects by Fresnel from either side. value.xyz is the
// absorption per metre in each channel, value.w the sea-level radius, and
// color.rgb the deep-water colour at full ambient light.
//
// Waves tilt the surface without raising it, so the sphere intersection stands.
// The surface does not refract yet; from below it still turns to a mirror past
// the critical angle.
//
// A float resolves only half a metre at a planet's radius, so the intersection
// is written around the camera's height above the water, and the waves around
// the camera's position in the planet's frame.

const float k_waterIor = 1.33;
// Reflectance head-on, ((n - 1) / (n + 1))^2
const float k_waterF0 = 0.02;
// Spreads the sun's glint on a calm sea to about the sun's own width
const float k_surfaceRoughness = 0.1;
// Water shallower than this shows foam, fading out toward it
const float k_foamDepth = 0.125;
// The share of the surface the foam covers at the waterline
const float k_foamCover = 0.25;
// Foam's grey from below, lit only by what it scatters through
const float k_foamFromBelow = 0.25;
// Foam's albedo from above, lit like the ground
const float k_foamAlbedo = 0.9;
// How far in depth the swell moves the foam's edge, at the swell's rms
const float k_foamSwell = 0.04;
// The rise per metre assumed of a shore, to tell what depth one pixel spans
const float k_shoreSlope = 0.1;
// Surf lines: the depth between two, the depth they reach out to, and their
// share of the foam's cover
const float k_surfSpacing = 0.1;
const float k_surfReach = 0.4;
const float k_surfCover = 0.6;
// Surf lines arriving per time wrap, two every second
const uint k_surfCyclesPerWrap = 32768u;

// Deep-water swell, summed as slopes. Each wave makes a whole number of cycles
// per time wrap, so the sea runs on across the wrap without a jump; the counts
// are picked so the wavelengths step by the golden ratio from 100 m to 3.4 m,
// which keeps the sum from ever lining up again.
const int k_waveCount = 8;
const uint k_waveCyclesPerWrap[k_waveCount] =
   uint[](2047u, 2604u, 3312u, 4214u, 5360u, 6818u, 8672u, 11031u);
// Each wave's heading off the wind in radians, alternating sides out to 35 degrees
const float k_waveHeading[k_waveCount] =
   float[](0.0, 0.44, -0.52, 0.21, -0.31, 0.61, -0.14, 0.35);
// Peak slope of each wave; eight together make an rms slope of 0.12, a moderate
// breeze
const float k_waveSlope = 0.06;
// The wind in the planet's frame, and the axis the headings turn toward. Waves
// fade out toward the two ends of the wind axis, so it is kept off the platform,
// which stands on -y.
const vec3 k_windAxis = vec3(1.0, 0.0, 0.0);
const vec3 k_windCross = vec3(0.0, 0.0, 1.0);
// Mirrors PhysicsUnits::s_tickRateHz
const float k_ticksPerSecond = 64.0;
const float k_gravity = 9.81;

// The normalized Blinn-Phong lobe of phong_lighting.glsl
float sunLobe(float cosHalf, float roughness) {
   float exponent = blinnExponent(roughness);
   return (exponent + 8.0) / (8.0 * k_pi) * pow(max(cosHalf, 0.0), exponent);
}

float reflectance(float cosTheta) {
   return fresnelSchlick(vec3(k_waterF0), cosTheta).r;
}

// Deep-water dispersion, g T^2 / 2 pi, for the period the cycle count sets
float waveLength(uint cyclesPerWrap) {
   float period = float(k_timeWrapTicks) / (float(cyclesPerWrap) * k_ticksPerSecond);
   return k_gravity * period * period / (2.0 * k_pi);
}

// The share of a cycle a wave has made by now. The whole ticks are multiplied in
// integers, which wrap at 2^32, a multiple of the time wrap, so the product stays
// exact however large it grows.
float waveTimeCycles(uint cyclesPerWrap) {
   uint whole = (u_time * cyclesPerWrap) & (k_timeWrapTicks - 1u);
   float cycles = float(whole) + u_timeRemainder * float(cyclesPerWrap);
   return fract(cycles / float(k_timeWrapTicks));
}

// A tilted normal can turn from the camera at grazing angles; it is leaned back
// just enough to face it, so Fresnel never sees the far side
vec3 faceCamera(vec3 normal, vec3 toCamera) {
   float facing = dot(normal, toCamera);
   return facing >= 0.01 ? normal : normalize(normal + (0.01 - facing) * toCamera);
}

struct SeaSurface {
   vec3 normal;   // view space, on the side of the up given
   float roughness;
   float swell;       // the waves' crests summed alike, rms 1 where all are present
   float footprint;   // metres of water one pixel spans
};

// The waves where the ray meets the water at distance t, around the calm normal
// up (view space, unit). A wave fades out as it narrows below a few pixels, and
// the slope it carried goes into the roughness, so the far sea spreads its glint
// instead of sparkling.
SeaSurface seaSurface(vec3 up, vec3 rayDir, float t, mat3 rayVolumeSpaceToView,
                      Df3 cameraLocalPosition) {
   mat3 toLocal = transpose(rayVolumeSpaceToView);
   vec3 upLocal = toLocal * up;
   vec3 offset = toLocal * (rayDir * t);

   // What one pixel spans on the water, stretched as the view grazes it
   float pixelAngle = 2.0 * u_inverseProjection[1][1] / u_screenSize.y;
   float footprint = pixelAngle * t / max(abs(dot(rayDir, up)), 0.05);

   vec3 slope = vec3(0.0);
   float swell = 0.0;
   float lostVariance = 0.0;
   for (int i = 0; i < k_waveCount; ++i) {
      float heading = k_waveHeading[i];
      vec3 direction = cos(heading) * k_windAxis + sin(heading) * k_windCross;
      float wavelength = waveLength(k_waveCyclesPerWrap[i]);
      vec3 cyclesPerMetre = direction / wavelength;

      // The camera's share is planet sized, so it is taken wide and only its
      // fraction kept; the offset from the camera is near enough for a float
      float cycles =
         dfFractToFloat(df3Dot(df3FromVec(cyclesPerMetre), cameraLocalPosition))
         + dot(cyclesPerMetre, offset) - waveTimeCycles(k_waveCyclesPerWrap[i]);

      float present = smoothstep(2.0, 4.0, wavelength / footprint);
      slope += present * k_waveSlope * cos(2.0 * k_pi * cycles) * direction;
      swell += present * sin(2.0 * k_pi * cycles);
      lostVariance += (1.0 - present * present) * 0.5 * k_waveSlope * k_waveSlope;
   }
   // Only the part along the surface tilts it
   slope -= dot(slope, upLocal) * upLocal;

   SeaSurface sea;
   sea.normal = rayVolumeSpaceToView * normalize(upLocal - slope);
   // roughness^4 is the mean square slope of the lobe, so the lost slope adds there
   sea.roughness = pow(pow(k_surfaceRoughness, 4.0) + lostVariance, 0.25);
   sea.swell = swell / sqrt(0.5 * float(k_waveCount));
   sea.footprint = footprint;
   return sea;
}

// How much of the surface is foam where the water is depth deep; from below,
// where the ground beyond stands depth above it
float foamCover(float depth, SeaSurface sea) {
   // The swell lifts and lowers the edge, so it wanders with the waves
   depth += k_foamSwell * sea.swell;

   // What one pixel spans in depth. A band thinner than that would flicker, so it
   // is spread over the pixel and thinned in the same proportion.
   float pixelDepth = sea.footprint * k_shoreSlope;
   float width = max(k_foamDepth, pixelDepth);
   float edge = k_foamDepth / width * (1.0 - smoothstep(0.0, width, depth));

   // Surf lines running in toward the shore, each sharp on its shoreward side
   // and trailing off behind; they fade out with depth, and with distance before
   // a pixel spans one
   float phase = fract(depth / k_surfSpacing + waveTimeCycles(k_surfCyclesPerWrap));
   float line = smoothstep(0.0, 0.05, phase) * (1.0 - smoothstep(0.05, 0.4, phase));
   float surf = k_surfCover * line * (1.0 - smoothstep(0.0, k_surfReach, depth))
              * (1.0 - smoothstep(0.25, 0.5, pixelDepth / k_surfSpacing));

   return k_foamCover * max(edge, surf);
}

RayVolumeResult rayVolumeShade(
   vec3 rayDir, float exitDistance, float sceneDistance,
   vec3 opaqueColor, vec4 value, vec4 color, vec2 uv,
   vec3 centerViewPos, Df centerDistance,
   mat3 rayVolumeSpaceToView, Df3 cameraLocalPosition)
{
   RayVolumeResult res;
   res.color = vec3(0.0);
   res.alpha = 0.0;
   res.weightDepth = 1.0;

   vec3 absorption = value.xyz;
   float radius = value.w;

   // ---- Where the ray enters and leaves the water sphere (entry, exit) ----

   // Camera height above sea level, negative under water. hi - radius is exact
   // (Sterbenz) at any height where it matters; precise keeps lo from being
   // folded into radius first.
   precise float aboveHi = centerDistance.hi - radius;
   precise float height = aboveHi + centerDistance.lo;
   float centreDist = centerDistance.hi; // Rounding is fine.

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

   // ---- How much of each channel survives the water the ray crosses ----

   float path = min(exit, sceneDistance) - entry;
   if (path <= 0.0) {
      return res;
   }

   // Beer-Lambert: what survives the path is exp(-absorption * path), in each
   // channel apart
   vec3 transmitted = exp(-absorption * path);
   float leastTransmitted = min(transmitted.r, min(transmitted.g, transmitted.b));
   vec3 waterColor = color.rgb * u_ambientScale;
   vec3 toLight = -normalize(u_lightDir);

   // ---- The surface seen from above, from below, or not at all ----

   // target is what the pixel shows; kept is the share of the scene behind that
   // every channel keeps at least
   vec3 target;
   float kept;
   if (height > 0.0) {
      // From above, the surface at entry mirrors the sky and lets the rest in
      vec3 toCamera = -rayDir;
      vec3 surfaceUp = normalize(rayDir * entry - centerViewPos);
      SeaSurface sea = seaSurface(
         surfaceUp, rayDir, entry, rayVolumeSpaceToView, cameraLocalPosition);
      vec3 normal = faceCamera(sea.normal, toCamera);
      float mirrored = reflectance(dot(normal, toCamera));
      vec3 halfway = normalize(toLight + toCamera);
      // The sun's peak runs to hundreds, enough to overflow the fp16 blend
      // targets, so both of its terms are capped at white, which the display
      // clips to anyway
      vec3 glint = min(fresnelSchlick(vec3(k_waterF0), dot(halfway, toCamera)) *
         sunLobe(dot(normal, halfway), sea.roughness) * max(dot(normal, toLight), 0.0) *
         u_directScale, vec3(1.0));
      vec3 below = mix(waterColor, opaqueColor, transmitted);
      target = mix(below, u_skyColor, mirrored) + glint;
      kept = (1.0 - mirrored) * leastTransmitted;

      // Foam where the water over the ground is shallow, covering the ground
      // under it. The depth is the path's drop, the shore being near flat on the
      // planet's scale.
      float depth = path * -dot(rayDir, surfaceUp);
      float foam = foamCover(depth, sea);
      float foamLight =
         k_ambient * u_ambientScale + max(dot(surfaceUp, toLight), 0.0) * u_directScale;
      target = mix(target, vec3(k_foamAlbedo * foamLight), foam);
      kept *= 1.0 - foam;
   } else if (mu > 0.0 && exit < sceneDistance) {
      // From below, the surface at exit, its normal facing the camera. Past the
      // critical angle refract returns zero and the surface reflects everything.
      vec3 surfaceUp = normalize(rayDir * exit - centerViewPos);
      SeaSurface sea = seaSurface(
         surfaceUp, rayDir, exit, rayVolumeSpaceToView, cameraLocalPosition);
      vec3 normal = faceCamera(-sea.normal, -rayDir);
      vec3 refracted = refract(rayDir, normal, k_waterIor);
      float mirrored = 1.0;
      vec3 above = vec3(0.0);
      if (refracted != vec3(0.0)) {
         mirrored = reflectance(dot(refracted, -normal));
         // The sun seen along the ray out, as wide as its glint from above
         float sun = sunLobe(dot(normalize(refracted + toLight), toLight), sea.roughness);
         above = opaqueColor + vec3(min(sun * u_directScale, 1.0));
      }
      // What the surface mirrors from below is deep water
      vec3 surface = mix(above, waterColor, mirrored);

      // Foam where the ground beyond stands little above the water, the rise of
      // the ray out mirroring the depth taken from above. It hides the view out
      // and shows the light it scatters down.
      float rise = (sceneDistance - exit) * dot(rayDir, surfaceUp);
      float foam = foamCover(rise, sea);
      surface = mix(surface, vec3(k_foamFromBelow), foam);

      target = mix(waterColor, surface, transmitted);
      kept = (1.0 - mirrored) * (1.0 - foam) * leastTransmitted;
   } else {
      target = mix(waterColor, opaqueColor, transmitted);
      kept = leastTransmitted;
   }

   // ---- Blend output ----

   // One alpha serves all three channels: it covers what no channel keeps, and
   // the colour gives each channel back its extra share. Exact for a single layer.
   res.alpha = 1.0 - kept;
   res.color = (target - opaqueColor * kept) / max(res.alpha, 1e-6);
   res.weightDepth = entry * max(-rayDir.z, 1e-4);
   return res;
}
