// plume_body.glsl
//
// Placeholder ion-plume body: a flat translucent color across the whole proxy,
// with no ray/chord/animation yet. The color (including alpha) comes from the
// per-instance color channel. Later the thrust magnitude will drive value.x to
// scale intensity/length, and viewBasis/localValue can shape the plume.
RayVolumeResult rayVolumeShade(
   vec3 viewPos, vec3 rayDir, float backDepth, float sceneDepth,
   vec3 opaqueColor, vec4 value, vec4 color, vec2 uv,
   vec3 centerViewPos, mat3 viewBasis)
{
   RayVolumeResult res;
   res.color = color.rgb;
   res.alpha = color.a;
   res.weightDepth = -centerViewPos.z;
   return res;
}
