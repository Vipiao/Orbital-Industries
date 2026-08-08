// sinusoid_surface.glsl
//
// Test surface for the CDLOD injection point: an egg-carton of sine waves in
// the body's own frame, raising the sphere along its own outward direction.
//
// The wavelength is deliberately short against the coarsest patch quad, so the
// level-of-detail behaviour is visible rather than hidden. An undivided face
// carries its quads across the whole body, and sampling a wave at a spacing
// near its own period cannot represent it: coarse patches show a wrong,
// low-frequency version of this surface, which resolves into the real one as
// the quadtree subdivides. Lengthen k_wavelength to make the surface behave.

const float k_amplitude = 1.0;    // metres of radial displacement, peak
const float k_wavelength = 3.0;  // metres between crests
const float k_frequency = 6.2831853 / k_wavelength;

// A product of sines, so the crests break up into lumps rather than running as
// parallel ridges.
float cdlodSurfaceElevation(vec3 spherePosition) {
   vec3 s = sin(spherePosition * k_frequency);
   return k_amplitude * s.x * s.y * s.z;
}

// Gradient of the elevation above. Differentiating a product of sines turns
// exactly one factor per component into its cosine.
vec3 cdlodSurfaceGradient(vec3 spherePosition) {
   vec3 s = sin(spherePosition * k_frequency);
   vec3 c = cos(spherePosition * k_frequency);
   return k_amplitude * k_frequency * vec3(c.x * s.y * s.z,
                                           s.x * c.y * s.z,
                                           s.x * s.y * c.z);
}
