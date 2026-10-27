// test_forest.glsl
// The placement rule for the test forest in main.cpp: trees on a flat plane at
// z = 0 of the body they ride on, whether or not there is ground beneath them.
//
// The payload is (patch column, patch row, 0, seed). The candidates of a patch
// form a square grid over the patch, and each one is nudged inside its grid cell
// by a hash, so the trees do not stand in rows.
//
// main.cpp defines TEST_FOREST_PATCH_SIZE (metres) and TEST_FOREST_GRID_SIDE
// (candidates along a side) ahead of this text, from the numbers it places the
// patches and sizes the set with.


const float k_patchSize = TEST_FOREST_PATCH_SIZE;
const uint k_gridSide = TEST_FOREST_GRID_SIDE;
// Share of the candidates that are not kept.
const float k_dropShare = 0.4;

// A well-mixed 32-bit integer hash ("lowbias32").
uint hashUint(uint x) {
    x ^= x >> 16;
    x *= 0x7feb352du;
    x ^= x >> 15;
    x *= 0x846ca68bu;
    x ^= x >> 16;
    return x;
}

// A number in [0, 1) that depends only on the patch, the candidate and salt,
// so the same candidate always gets the same tree. salt picks one of several
// independent numbers for the same candidate.
float hashFloat(uvec4 payload, uint candidate, uint salt) {
    uint h = hashUint(salt);
    h = hashUint(h ^ candidate);
    h = hashUint(h ^ payload.w);
    h = hashUint(h ^ payload.y);
    h = hashUint(h ^ payload.x);
    // The top 24 bits, which a float holds exactly, so the result is evenly
    // spaced and never rounds up to 1.
    return float(h >> 8) / 16777216.0;
}

// patchOrigin is not needed: the plane is flat, so a tree's place in its patch
// does not depend on where the patch is.
bool scatterCandidate(uvec4 payload, Df3 patchOrigin, uint candidate,
                      out vec3 position, out vec4 orientation,
                      out float scale, out uint kind) {
    if (hashFloat(payload, candidate, 0u) < k_dropShare) return false;

    const float cellSize = k_patchSize / float(k_gridSide);
    const vec2 cell = vec2(candidate % k_gridSide, candidate / k_gridSide);
    const vec2 jitter = vec2(hashFloat(payload, candidate, 1u),
                             hashFloat(payload, candidate, 2u));
    position = vec3((cell + jitter) * cellSize, 0.0);

    // A random turn about +Z, the ground's up.
    const float yaw = hashFloat(payload, candidate, 3u) * k_twoPi;
    orientation = vec4(0.0, 0.0, sin(0.5 * yaw), cos(0.5 * yaw));

    scale = 0.7 + 0.6 * hashFloat(payload, candidate, 4u);
    kind = 0u;
    return true;
}
