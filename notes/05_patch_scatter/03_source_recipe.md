# Source recipe

The design Markus found online, verbatim, kept for reference. `01_design.md` says what was
changed and why.

```
CONTEXT: GPU tree/vegetation instancing on a spherical (cube-sphere) planet. OpenGL 4.6 (gl_BaseInstance), no engine.

DESIGN
- Fixed-size tree tiles on the cube faces (~64m), independent of the terrain LOD levels. Equal-angle (tan) warp on the face so tiles stay close to 64m everywhere.
- Placement is deterministic: each tile has a fixed grid of candidate points (candidate id = grid cell). Jitter via hash(tileID, candidateId), rejected by slope/height/biome/noise. Same code on the CPU (collision) and GPU (rendering), no readback.
- The candidate id is the tree's stable identity. Its slot inside the tile's block is NOT stable (atomicAdd order differs every run), so nothing may refer to a tree by slot index.
- Tree up axis = local sphere normal, random yaw.
- Bushes etc. = more species in the same system, on their own candidate grid. Grass is a separate camera-local system.
- Precision: tile-relative positions + per-tile camera offset computed in double on the CPU, uploaded as float. View matrix has no translation.

DETERMINISM (CPU and GPU must make identical keep/reject decisions)
- All randomness from integer hashes. Noise on an integer lattice (cell = tile coords >> octave), never on large float world coordinates.
- Shader: placement math marked `precise`. C++: -ffp-contract=off.
- lerp/smoothstep written out with basic operators on both sides; no sin/pow/normalize/sqrt in anything that feeds a decision.
- Tested: 450k trees, 0 differences (NVIDIA and Intel/Mesa). Positions differ by under 0.1mm.
- Trees sample the full-detail terrain height, not a LOD-morphed mesh.

BUFFERS
- Mesh VBO/IBO: all meshes (species x LOD) packed together, incl. one billboard quad per species. Startup.
- Impostor atlas: texture array, one layer per species, baked at startup from the LOD0 mesh.
- Species table: LOD distances, max distance, bounding radius, collision radius. Startup.
- Tile buffer (SSBO): per slot the 4 tile corners relative to the tile origin (double -> float), tile normal, face, tile coords. CPU, on tile load.
- Tree buffer (SSBO): TreeData {vec4 posAndScale; vec4 rotQuat; uint species; uint candidateId; pad} = 48 bytes, fixed block of MAX_PER_TILE per tile slot. Generation CS.
- tileCount[]: trees per slot. Generation CS.
- removedMask[]: bitmask per slot of destroyed trees, indexed by candidateId. CPU, on destruction.
- tileOffset[]: float(tileOrigin_d - camera_d). CPU, every frame.
- visibleTiles[]: slots passing the CPU tile frustum test. CPU, every frame.
- visible[]: uint tree indices, one fixed range per (species, LOD), each with a capacity.
- Indirect buffer: one DrawElementsIndirectCommand per (species, LOD). CPU sets mesh fields and baseInstance (range start) once; the culling CS writes instanceCount.

STARTUP
Upload meshes, bake the atlas, allocate the tree buffer (MAX_TILES*MAX_PER_TILE), size the visible ranges, fill the indirect commands.
Range capacity for (species, LOD) = candidates that can lie within that LOD's outer distance (pi*d^2 / tileArea * candidatesPerTile + margin), capped at the total.

TILE LOAD (occasional, budgeted per frame, nearest first)
CPU: pick free slot k, write tiles[k], tileCount[k]=0, restore removedMask[k] for that tileID.
Generation CS, one thread per candidate (workgroups of 64, one row of groups per new tile):
  hash/jitter -> sample terrain -> reject or keep -> species/rot/scale
  uint i = atomicAdd(tileCount[k],1); trees[k*MAX_PER_TILE+i] = t;   // t stores candidateId
Barrier: SHADER_STORAGE.
Unload = mark slot free. Destroyed tree = set the removedMask bit of its candidateId; the CPU owns the fallen tree as a dynamic object and remembers destroyed ids per tileID.

EVERY FRAME
CPU: upload tileOffset[], frustum-cull tiles -> visibleTiles[], reset all instanceCount=0.
Culling CS, workgroups of 64, dispatch (MAX_PER_TILE/64, numVisibleTiles); max 65535 visible tiles:
  k = visibleTiles[WorkGroupID.y]; local = GlobalInvocationID.x
  if local >= tileCount[k] -> return
  treeIndex = k*MAX_PER_TILE+local
  if removedMask bit of trees[treeIndex].candidateId set -> return
  p = tileOffset[k] + trees[treeIndex].pos
  if length(p) > species max distance -> return
  frustum test (bounding sphere), lod = pickLod(length(p), species)
  cmd = species*NUM_LODS+lod
  slot = atomicAdd(commands[cmd].instanceCount,1)
  if slot < capacity[cmd]: visible[commands[cmd].baseInstance+slot] = treeIndex
Clamp CS: instanceCount = min(instanceCount, capacity) per command, so a full range never spills into the next one.
Barrier: SHADER_STORAGE | COMMAND.
Draw: glMultiDrawElementsIndirect(..., NUM_SPECIES*NUM_LODS, ...).
Vertex shader:
  treeIndex = visible[gl_BaseInstance+gl_InstanceID]; t = trees[treeIndex]
  k = treeIndex/MAX_PER_TILE; base = tileOffset[k] + t.pos
  mesh:      camRel = base + rotate(t.rot, inPos*t.scale)
  billboard: up = rotate(t.rot, Y); right = normalize(cross(up, -base)); camRel = base + (right*inPos.x + up*inPos.y)*t.scale
  gl_Position = viewProjNoTranslation * vec4(camRel,1)
Fragment shader: billboards sample their species' atlas layer and alpha-test.

COLLISION
The CPU runs the same placement for tiles near the player (~0.1ms per tile) and builds trunk capsules (species with radius>0), skipping removed candidate ids.

MEASURED (RTX 3060 laptop, 1080p, tree every 4m + bush every 8m)
- 4km view: 0.6M trees drawn, 3.4ms. 8km: 2.1M drawn, 8.3ms. 12km: 4.7M drawn, 17ms.
- Over 97% of drawn trees are billboards: ~2ns to draw + ~1ns to cull per tree.
- 53k trees at one LOD: full mesh 32ms, LOD1 6ms, LOD2 3ms, billboards 0.4ms.

LIMITS
- Memory: MAX_TILES*MAX_PER_TILE*48 bytes, used or not. Keep the tree buffer under ~1.5GB; at 2.6GB frame time collapsed (cause not found).
- One billboard per tree on fixed 64m tiles works to roughly 10km; beyond that thin or merge distant trees.
- Placement noise must be continuous across cube face edges (the demo's is not).

LATER
- Hi-Z occlusion culling in the culling CS.
- Separate culling pass + visible list per shadow cascade.
- Multi-angle impostors with normals to hide the mesh-to-billboard switch.
- Smaller TreeData, hierarchical tiles for far distances.
```
