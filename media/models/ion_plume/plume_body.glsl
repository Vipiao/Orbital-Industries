// plume_body.glsl
//
// Ion-thruster plume. Analytic cylinder in the instance's local frame, ray
// marched for a hollow-core / bright-ring radial density. Adapted from a
// Shadertoy prototype:
//   - the camera/ray come from the engine (view space), so no camera is built;
//   - the cylinder axis is reconstructed from the instance orientation
//     (viewBasis maps local -> view) and origin (centerViewPos);
//   - the march is clamped by the opaque scene depth as well as the cylinder
//     exit, so the plume fades into geometry behind it.
// The cylinder inscribes the 1x2x1 proxy: axis along local +Y at (0.5, *, 0.5),
// radius 0.5, height 2.

#define PI 3.1415926538

// Local plume radius at axialDist, fitted so sqrt(a*x^2 + b*x + c) passes
// through the three (axialDist, radius) control points p0, p1, p2.
float radiusProfile(float axialDist, vec2 p0, vec2 p1, vec2 p2)
{
    // Fit the SQUARED radii through the three points (Lagrange -> monomial coeffs).
    float y0 = p0.y * p0.y;
    float y1 = p1.y * p1.y;
    float y2 = p2.y * p2.y;

    float w0 = y0 / ((p0.x - p1.x) * (p0.x - p2.x));
    float w1 = y1 / ((p1.x - p0.x) * (p1.x - p2.x));
    float w2 = y2 / ((p2.x - p0.x) * (p2.x - p1.x));

    float a = w0 + w1 + w2;
    float b = -(w0 * (p1.x + p2.x) + w1 * (p0.x + p2.x) + w2 * (p0.x + p1.x));
    float c =   w0 * (p1.x * p2.x) + w1 * (p0.x * p2.x) + w2 * (p0.x * p1.x);

    float r2 = a * axialDist * axialDist + b * axialDist + c;
    return sqrt(max(r2, 1e-6));
}

// returns vec2(tEnter, tExit); a miss is signalled by exit <= enter
vec2 cylinderHit(vec3 rayOrigin, vec3 rayDir, vec3 axisBase, vec3 axisDir,
                 float radius, float height)
{
    vec3  toBase       = rayOrigin - axisBase;
    float dirAlongAxis = dot(rayDir, axisDir);
    float baseAlongAxis= dot(toBase, axisDir);
    vec3  dirPerp      = rayDir - dirAlongAxis * axisDir;   // ray dir off the axis
    vec3  basePerp     = toBase - baseAlongAxis * axisDir;  // origin offset off the axis

    // Quadratic for the infinite-cylinder wall: a*t^2 + b*t + c = 0
    float a = dot(dirPerp, dirPerp);
    float b = 2.0 * dot(basePerp, dirPerp);
    float c = dot(basePerp, basePerp) - radius * radius;

    float wallEnter = -1e20, wallExit = 1e20;
    if (a > 1e-9) {
        float disc = b * b - 4.0 * a * c;
        if (disc < 0.0) return vec2(0.0);          // miss: ray never reaches the wall
        float sq = sqrt(disc);
        wallEnter = (-b - sq) / (2.0 * a);
        wallExit  = (-b + sq) / (2.0 * a);
    } else if (c > 0.0) {
        return vec2(0.0);                          // parallel to axis and outside radius
    }

    // Clip against the two end caps (axial position in [0, height])
    float capEnter = -1e20, capExit = 1e20;
    if (abs(dirAlongAxis) > 1e-9) {
        float tBottom = (0.0    - baseAlongAxis) / dirAlongAxis;
        float tTop    = (height - baseAlongAxis) / dirAlongAxis;
        capEnter = min(tBottom, tTop);
        capExit  = max(tBottom, tTop);
    } else if (baseAlongAxis < 0.0 || baseAlongAxis > height) {
        return vec2(0.0);                          // parallel to caps and outside the ends
    }

    float tEnter = max(max(wallEnter, capEnter), 0.0);
    float tExit  = min(wallExit, capExit);
    return vec2(tEnter, tExit);                    // caller checks exit > enter
}

// Radial density: distance to axis fed through a sine.
// sin(dist*k) is 0 at the axis (dark core) and peaks away from it -> split/ring look.
float density(vec3 point, vec3 axisBase, vec3 axisDir)
{
    vec3  toPoint      = point - axisBase;
    float axialDist    = dot(toPoint, axisDir);
    float radialDist   = length(toPoint - axialDist * axisDir);  // distance to axis
    float radialParam = radialDist / radiusProfile(axialDist, vec2(0.0, 0.65), vec2(0.5, 0.4), vec2(2.0, 1.0));
    //float ring = sin(radialParam * PI*4.0 - 1.57)+1.;                 // rings; tune the 16
    float density = 0.5-radialParam;
    // Fade toward end.
    density *= pow(2.-axialDist, 2.0);
    // Round the ring.
    float ff = 1.0;//sin(iTime)*0.5 + 0.5;
    density *= sin(radialDist * 4. * PI - ff*PI/2.) + ff;

    //Soften.
    //density *= max(density, 0.);

    return max(density, 0.0) * 8.0;// clamp, scale to taste.
}

// Fixed-step march between the two hit t-values, accumulating optical depth.
float opticalDepth(vec3 rayOrigin, vec3 rayDir, vec3 axisBase, vec3 axisDir,
                   float tEnter, float tExit)
{
    const int STEPS = 8;
    float stepLen = (tExit - tEnter) / float(STEPS);
    float depth   = 0.0;
    for (int i = 0; i < STEPS; i++) {
        float t     = tEnter + (float(i) + 0.5) * stepLen;   // sample mid-step
        vec3  point = rayOrigin + t * rayDir;
        depth += density(point, axisBase, axisDir) * stepLen;
    }
    return depth;
}

RayVolumeResult rayVolumeShade(
   vec3 viewPos, vec3 rayDir, float backDepth, float sceneDepth,
   vec3 opaqueColor, vec4 value, vec4 color, vec2 uv,
   vec3 centerViewPos, mat3 viewBasis)
{
   RayVolumeResult res;
   res.color = vec3(0.0);
   res.alpha = 0.0;
   res.weightDepth = -centerViewPos.z;

   // value.x is the thruster throttle in [0, 1]; a cold thruster has no plume
   // and skips the march entirely.
   float thrust = value.x;
   if (thrust < 1.0 / 255.0) return res;

   // Cylinder inscribed in the 1x2x1 proxy, brought into view space. The camera
   // sits at the view-space origin and rayDir is already the unit view ray.
   const vec3  axisBaseLocal = vec3(0.5, 0.0, 0.5);
   const vec3  axisDirLocal  = vec3(0.0, 1.0, 0.0);
   const float radius = 0.5;
   const float height = 2.0;

   vec3 axisBase  = centerViewPos + viewBasis * axisBaseLocal;
   vec3 axisDir   = normalize(viewBasis * axisDirLocal);
   vec3 rayOrigin = vec3(0.0);

   vec2 hit = cylinderHit(rayOrigin, rayDir, axisBase, axisDir, radius, height);
   if (hit.y <= hit.x) return res;   // ray misses the cylinder

   // Limit the chord by the far opaque surface as well as the cylinder exit, so
   // the plume is occluded by geometry behind it (converting scene depth, which
   // is measured along -z, into a ray parameter t).
   float tScene = sceneDepth / max(-rayDir.z, 1e-4);
   float tEnter = max(hit.x, 0.0);
   float tExit  = min(hit.y, tScene);
   if (tExit <= tEnter) return res;

   float depth = opticalDepth(rayOrigin, rayDir, axisBase, axisDir, tEnter, tExit);

   // Throttle scales the optical depth (a thinner plume rather than a uniform
   // transparency fade): the dense core fades last, the fringes first.
   depth *= thrust;

   res.color       = color.rgb + opaqueColor;
   res.alpha       = 1.0 - exp(-depth);
   res.weightDepth = tEnter * max(-rayDir.z, 1e-4);   // depth of the plume front
   return res;
}
