// planet_props.glsl
//
// The scatter snippet placing a planet's props (see scatter_generate.comp). A
// patch is one square of PropPatchGrid, payload (face, column, row, seed), its
// origin the square's corner. Candidates are a k_propCandidatesPerSide grid,
// each nudged inside its cell by a hash.
//
// Every decision is made in integers, so a CPU twin can match it bit for bit;
// floats enter only when the spot is put on the ground.
//
// Preceded by planetSurfaceGlsl, propPatchGridGlsl, k_propCandidatesPerSide,
// k_propMinScale and k_propScaleSpan.
#include "lattice_surface.glsl"

// Candidates kept on all-grass ground, in 65536ths: six in ten
const uint k_propKeep = 39322u;
// Finer than any map texel, as PlanetSurface::surfacePoint reads them
const float k_propSampleSpacing = 0.001;

// lowbias32
uint propHash(uint x) {
   x ^= x >> 16;
   x *= 0x7feb352du;
   x ^= x >> 15;
   x *= 0x846ca68bu;
   x ^= x >> 16;
   return x;
}

// Sixteen bits from the patch, candidate and salt alone; salt picks the draw.
uint propBits(uvec4 payload, uint candidate, uint salt) {
   uint h = propHash(salt);
   h = propHash(h ^ candidate);
   h = propHash(h ^ payload.w);
   h = propHash(h ^ payload.z);
   h = propHash(h ^ payload.y);
   h = propHash(h ^ payload.x);
   return h >> 16;
}

// The patch's two edges from its corner, in metres, body frame. Straight edges
// are off by about a millimetre across a patch.
void propPatchEdges(uint face, uint column, uint row, out vec3 edgeU, out vec3 edgeV) {
   const float faceAngle = 0.5 * k_pi;
   float patches = float(k_propPatchesPerEdge);

   // As PropPatchGrid::gridDirection
   float tanU = tan(faceAngle * (float(column) / patches - 0.5));
   float tanV = tan(faceAngle * (float(row) / patches - 0.5));
   vec3 cubePoint = k_faceOrigin[face] + 0.5 * (tanU + 1.0) * k_faceUAxis[face]
                  + 0.5 * (tanV + 1.0) * k_faceVAxis[face];

   // d tan / d angle = 1 + tan^2
   float stepAngle = faceAngle / patches;
   vec3 cubeStepU = k_faceUAxis[face] * (0.5 * (1.0 + tanU * tanU) * stepAngle);
   vec3 cubeStepV = k_faceVAxis[face] * (0.5 * (1.0 + tanV * tanV) * stepAngle);

   // Projected onto the sphere
   float reach = length(cubePoint);
   vec3 outward = cubePoint / reach;
   float toMetres = k_radiusMetres / reach;
   edgeU = toMetres * (cubeStepU - outward * dot(outward, cubeStepU));
   edgeV = toMetres * (cubeStepV - outward * dot(outward, cubeStepV));
}

// As fromQuaternion reads it back. Each branch divides by the largest component.
vec4 propQuaternion(mat3 rotation) {
   // Element (row, column) is rotation[column][row]
   float xx = rotation[0][0];
   float yy = rotation[1][1];
   float zz = rotation[2][2];
   float trace = xx + yy + zz;

   if (trace > 0.0) {
      float s = 2.0 * sqrt(trace + 1.0);
      return vec4((rotation[1][2] - rotation[2][1]) / s,
                  (rotation[2][0] - rotation[0][2]) / s,
                  (rotation[0][1] - rotation[1][0]) / s,
                  0.25 * s);
   }
   if (xx > yy && xx > zz) {
      float s = 2.0 * sqrt(1.0 + xx - yy - zz);
      return vec4(0.25 * s,
                  (rotation[1][0] + rotation[0][1]) / s,
                  (rotation[2][0] + rotation[0][2]) / s,
                  (rotation[1][2] - rotation[2][1]) / s);
   }
   if (yy > zz) {
      float s = 2.0 * sqrt(1.0 + yy - xx - zz);
      return vec4((rotation[1][0] + rotation[0][1]) / s,
                  0.25 * s,
                  (rotation[2][1] + rotation[1][2]) / s,
                  (rotation[2][0] - rotation[0][2]) / s);
   }
   float s = 2.0 * sqrt(1.0 + zz - xx - yy);
   return vec4((rotation[2][0] + rotation[0][2]) / s,
               (rotation[2][1] + rotation[1][2]) / s,
               0.25 * s,
               (rotation[0][1] - rotation[1][0]) / s);
}

// Mesh +Z along up, turned by turn 65536ths of a full turn from the patch's U
// edge, which is never along up.
vec4 propOrientation(vec3 up, vec3 edgeU, uint turn) {
   vec3 east = normalize(edgeU - up * dot(up, edgeU));
   vec3 north = cross(up, east);

   float angle = k_twoPi * float(turn) / 65536.0;
   vec3 meshX = cos(angle) * east + sin(angle) * north;
   vec3 meshY = cross(up, meshX);

   return propQuaternion(mat3(meshX, meshY, up));
}

bool scatterCandidate(uvec4 payload, Df3 patchOrigin, uint candidate,
                      out vec3 position, out vec4 orientation,
                      out float scale, out uint kind) {
   // Out before the terrain is read if above even the all-grass share
   uint keepDraw = propBits(payload, candidate, 0u);
   if (keepDraw >= k_propKeep) return false;

   // In 65536ths of a cell; below 2^24, so exact as floats
   uvec2 cell = uvec2(candidate % k_propCandidatesPerSide,
                      candidate / k_propCandidatesPerSide);
   uvec2 nudge = uvec2(propBits(payload, candidate, 1u), propBits(payload, candidate, 2u));
   vec2 across = vec2(cell * 65536u + nudge) / float(k_propCandidatesPerSide * 65536u);

   vec3 edgeU;
   vec3 edgeV;
   propPatchEdges(payload.x, payload.y, payload.z, edgeU, edgeV);
   Df3 crudePoint = df3AddVec(patchOrigin, across.x * edgeU + across.y * edgeV);

   TerrainDisplacement displacement = terrainDisplacement(
      gatherLevels(crudePoint, k_positionLevels, k_propSampleSpacing));

   if (displacement.height < k_shoreMetres) return false;

   vec3 up = normalize(df3ToVec(crudePoint));

   // Thinned by the grass share; the draw stays an integer
   float slope = length(gradientAcrossSphere(displacement.gradient, up));
   float grass = terrainCover(displacement.height, slope).grass;
   if (keepDraw >= uint(float(k_propKeep) * grass)) return false;

   Df3 ground = surfacePointAtHeight(crudePoint, displacement.height);
   position = df3ToVec(df3Sub(ground, patchOrigin));
   orientation = propOrientation(up, edgeU, propBits(payload, candidate, 4u));

   scale = k_propMinScale + k_propScaleSpan * float(propBits(payload, candidate, 3u)) / 65536.0;
   kind = 0u;
   return true;
}
