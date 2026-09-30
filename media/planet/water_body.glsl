// water_body.glsl
//
// A planet's water: a sea-level sphere that absorbs light along the view ray,
// under a surface that reflects by Fresnel from either side. value.xyz is the
// absorption per metre in each channel, value.w the sea-level radius,
// color.rgb the deep water's albedo, lit like the ground, and color.w the tick
// the camera last crossed the surface at (see the splash below).
//
// Waves tilt the surface without raising it, so the sphere intersection stands.
// The surface does not refract yet; from below it still turns to a mirror past
// the critical angle.
//
// A float resolves only half a metre at a planet's radius, so the intersection
// is written around the camera's height above the water, and the waves around
// the camera's position in the planet's frame.

#include "sea_surface.glsl"

const float k_waterIor = 1.33;
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
const uint k_waveCyclesPerWrap[k_waveCount] =
   uint[](2047u, 2604u, 3312u, 4214u, 5360u, 6818u, 8672u, 11031u);
// Each wave's heading off the wind in radians, alternating sides out to 35 degrees
const float k_waveHeading[k_waveCount] =
   float[](0.0, 0.44, -0.52, 0.21, -0.31, 0.61, -0.14, 0.35);
// The sea is three sets of the same waves, each running in the plane across one
// axis of the planet's frame and weighted by how near up lies to that axis, so
// every set is seen near face on. Each plane's wind, and the axis the headings
// turn toward; the xz plane's covers the platform, which stands on -y.
const vec3 k_windAxes[3] = vec3[](vec3(0.0, 1.0, 0.0), vec3(1.0, 0.0, 0.0),
                                  vec3(1.0, 0.0, 0.0));
const vec3 k_windCrosses[3] = vec3[](vec3(0.0, 0.0, 1.0), vec3(0.0, 0.0, 1.0),
                                     vec3(0.0, 1.0, 0.0));
// A plane's set shows only where up's component along its axis passes this. The
// largest component of a unit vector is at least 1/sqrt(3), so some set always
// shows.
const float k_planeThreshold = 0.5;
// Mirrors PhysicsUnits::s_tickRateHz
const float k_ticksPerSecond = 64.0;
const float k_gravity = 9.81;

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

// Camera height above sea level, negative under water. hi - radius is exact
// (Sterbenz) at any height where it matters; precise keeps lo from being folded
// into radius first.
float cameraHeight(Df centerDistance, float radius) {
   precise float aboveHi = centerDistance.hi - radius;
   precise float height = aboveHi + centerDistance.lo;
   return height;
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

   // The sets are unrelated, so weights of unit length keep the rms slope and
   // swell of one set wherever they blend
   vec3 weights = max(abs(upLocal) - k_planeThreshold, 0.0);
   weights /= length(weights);

   vec3 slope = vec3(0.0);
   float swell = 0.0;
   float lostVariance = 0.0;
   for (int i = 0; i < k_waveCount; ++i) {
      float wavelength = waveLength(k_waveCyclesPerWrap[i]);
      float present = smoothstep(2.0, 4.0, wavelength / footprint);
      lostVariance += (1.0 - present * present) * 0.5 * k_waveSlope * k_waveSlope;
      if (present <= 0.0) {
         continue;
      }
      float heading = k_waveHeading[i];
      float timeCycles = waveTimeCycles(k_waveCyclesPerWrap[i]);

      for (int plane = 0; plane < 3; ++plane) {
         if (weights[plane] <= 0.0) {
            continue;
         }
         vec3 direction =
            cos(heading) * k_windAxes[plane] + sin(heading) * k_windCrosses[plane];
         vec3 cyclesPerMetre = direction / wavelength;

         // The camera's share is planet sized, so it is taken wide and only its
         // fraction kept; the offset from the camera is near enough for a float
         float cycles =
            dfFractToFloat(df3Dot(df3FromVec(cyclesPerMetre), cameraLocalPosition))
            + dot(cyclesPerMetre, offset) - timeCycles;

         float amount = weights[plane] * present;
         slope += amount * k_waveSlope * cos(2.0 * k_pi * cycles) * direction;
         swell += amount * sin(2.0 * k_pi * cycles);
      }
   }
   // Only the part along the surface tilts it
   slope -= dot(slope, upLocal) * upLocal;

   SeaSurface sea;
   sea.normal = rayVolumeSpaceToView * normalize(upLocal - slope);
   sea.roughness = seaRoughness(lostVariance);
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

// Water over the lens for a moment after the camera crosses the surface. Patches
// of noise stand above a threshold that rises until none are left, and each
// bends the view across its slope; the water is then shaded along the bent ray
// against the scene that ray sees. color.w holds the physics tick the crossing
// happened at, modulo the time wrap, or is negative before the first.
const float k_splashSeconds = 0.6;
// Foam filling the patches right after the crossing, gone after this long. White
// and half see-through coming out of the water, black going in.
const float k_splashFoamSeconds = 0.2;
const vec3 k_splashFoamColor = vec3(0.85, 0.9, 0.95);
const float k_splashFoamOpacity = 0.5;
const vec3 k_splashDiveColor = vec3(0.0);
// The threshold at the crossing; it rises to 1 over k_splashSeconds
const float k_splashStartThreshold = 0.25;
// Noise cells per screen height
const float k_splashScale = 7.5;
// How far a patch's slope shifts the view, in screen heights
const float k_splashBend = 0.05;

float splashHash(vec2 p) {
   return fract(sin(dot(p, vec2(127.1, 311.7))) * 43758.5453);
}

// Smooth value noise in [0, 1]
float valueNoise(vec2 p) {
   vec2 i = floor(p);
   vec2 f = fract(p);
   vec2 u = f * f * (3.0 - 2.0 * f);
   return mix(mix(splashHash(i), splashHash(i + vec2(1.0, 0.0)), u.x),
              mix(splashHash(i + vec2(0.0, 1.0)), splashHash(i + vec2(1.0)), u.x), u.y);
}

// How far the water stands above the threshold, in screen-height noise units
float splashHeight(vec2 p, float threshold) {
   float n = 0.65 * valueNoise(p) + 0.35 * valueNoise(2.1 * p + 5.0);
   return max(n - threshold, 0.0);
}

// Seconds since the camera last crossed the surface, or past the splash if never
float secondsSinceCrossing(float crossingTick) {
   if (crossingTick < 0.0) {
      return k_splashSeconds;
   }
   uint elapsed = (u_time - uint(crossingTick)) & (k_timeWrapTicks - 1u);
   return (float(elapsed) + u_timeRemainder) / k_ticksPerSecond;
}

float splashThreshold(float t) {
   return mix(k_splashStartThreshold, 1.0, t / k_splashSeconds);
}

// Where a fragment falls in the noise
vec2 splashNoisePoint(vec2 fragCoord) {
   return fragCoord / u_screenSize.y * k_splashScale;
}

// Pixels the splash shifts the view by at a fragment, t seconds after crossing
vec2 splashOffset(vec2 fragCoord, float t) {
   float threshold = splashThreshold(t);
   vec2 p = splashNoisePoint(fragCoord);
   float e = 0.02;
   vec2 slope = vec2(splashHeight(p + vec2(e, 0.0), threshold)
                        - splashHeight(p - vec2(e, 0.0), threshold),
                     splashHeight(p + vec2(0.0, e), threshold)
                        - splashHeight(p - vec2(0.0, e), threshold)) / (2.0 * e);

   return slope * (k_splashBend * u_screenSize.y);
}

// Share of a fragment the foam covers, t seconds after crossing
float splashFoam(vec2 fragCoord, float t) {
   float fade = 1.0 - t / k_splashFoamSeconds;
   if (fade <= 0.0) {
      return 0.0;
   }
   float height = splashHeight(splashNoisePoint(fragCoord), splashThreshold(t));
   return fade * fade * smoothstep(0.0, 0.1, height);
}

// The view ray through a point on the screen, in view space
vec3 viewRayThrough(vec2 fragCoord) {
   vec2 ndc = fragCoord / u_screenSize * 2.0 - 1.0;
   vec4 viewH = u_inverseProjection * vec4(ndc, 0.5, 1.0);
   return normalize(viewH.xyz / viewH.w);
}

RayVolumeResult waterShade(
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

   float height = cameraHeight(centerDistance, radius);
   float farWater = smoothstep(k_farSeaWhole, k_nearSeaGone, height);
   if (farWater >= 1.0) {
      return res;
   }
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
   vec3 toLight = -normalize(u_lightDir);

   // Lit as flat ground where the ray enters
   vec3 entryUp = normalize(rayDir * entry - centerViewPos);
   float light = seaLight(entryUp, toLight, u_ambientScale, u_directScale);
   vec3 waterColor = color.rgb * light;

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
      vec3 below = mix(waterColor, opaqueColor, transmitted);
      float mirrored;
      target = seaFromAbove(below, normal, toCamera, toLight, sea.roughness, u_skyColor,
                            u_directScale, mirrored);
      kept = (1.0 - mirrored) * leastTransmitted;

      // Foam where the water over the ground is shallow, covering the ground
      // under it. The depth is the path's drop, the shore being near flat on the
      // planet's scale.
      float depth = path * -dot(rayDir, surfaceUp);
      float foam = foamCover(depth, sea);
      target = mix(target, vec3(k_foamAlbedo * light), foam);
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
   res.alpha *= 1.0 - farWater;
   res.weightDepth = entry * max(-rayDir.z, 1e-4);
   return res;
}

RayVolumeResult rayVolumeShade(
   vec3 rayDir, float exitDistance, float sceneDistance,
   vec3 opaqueColor, vec4 value, vec4 color, vec2 uv,
   vec3 centerViewPos, Df centerDistance,
   mat3 rayVolumeSpaceToView, Df3 cameraLocalPosition,
   sampler2D sceneDepthMap, sampler2D opaqueColorMap)
{
   float t = secondsSinceCrossing(color.w);
   if (t >= k_splashSeconds) {
      return waterShade(
         rayDir, exitDistance, sceneDistance, opaqueColor, value, color, uv,
         centerViewPos, centerDistance, rayVolumeSpaceToView, cameraLocalPosition);
   }

   // The pixel the splash bends this one's view onto, and the scene along it
   vec2 bent = gl_FragCoord.xy + splashOffset(gl_FragCoord.xy, t);
   ivec2 pixel = clamp(ivec2(bent), ivec2(0), ivec2(u_screenSize) - 1);
   vec3 bentRay = viewRayThrough(vec2(pixel) + 0.5);
   float bentScene = viewDepthAt(sceneDepthMap, pixel) / max(-bentRay.z, 1e-4);
   vec3 bentOpaque = texelFetch(opaqueColorMap, pixel, 0).rgb;

   RayVolumeResult res = waterShade(
      bentRay, exitDistance, bentScene, bentOpaque, value, color, uv,
      centerViewPos, centerDistance, rayVolumeSpaceToView, cameraLocalPosition);

   // The water over the scene the bent ray sees, covering this pixel's own
   res.color = mix(bentOpaque, res.color, res.alpha);
   res.alpha = 1.0;

   // Foam over it, lit like the foam on the sea, or black after a dive
   bool dove = cameraHeight(centerDistance, value.w) < 0.0;
   vec3 foamColor = dove
      ? k_splashDiveColor
      : k_splashFoamColor * min(k_ambient * u_ambientScale + u_directScale, 1.0);
   float foam = splashFoam(gl_FragCoord.xy, t) * (dove ? 1.0 : k_splashFoamOpacity);
   res.color = mix(res.color, foamColor, foam);
   return res;
}
