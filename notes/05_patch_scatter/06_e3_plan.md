# E3 — Cull and draw

Goal: the items E2 generated on the GPU appear in the G-buffer, with a LOD chosen per item by
distance and a hard cutoff at the kind's last LOD distance. Shadows are E4.

Each sub-step is reviewed before the next begins. As in E1, no sub-step contains code that is
later thrown away; the first few draw nothing.

## The shape of one frame

```
CPU, once per frame (ScatterHandler::update)
  for each instance:  the body's pose this frame (same bodyRenderPose CDLOD uses)
  for each patch:     origin − camera, in double, then narrowed to float
  upload:             patch list, instance list; reset every draw command's count to 0

GPU, culling pass (compute), one workgroup per patch
  for each item in the patch's block, up to the block's item count:
    distance to camera → which LOD, or none if past the last
    atomicAdd on that mesh's instanceCount → a place in that mesh's part of the visible list
    write (item index, patch index) there

GPU, one glMultiDrawElementsIndirect: one command per mesh (kind × LOD)
  vertex shader: visible[gl_BaseInstance + gl_InstanceID] → item, patch, instance
                 → position relative to the camera → G-buffer
```

The CPU never reads anything back. The item counts are written by the generation pass, the
draw counts by the culling pass, and both are read on the GPU directly.

## E3.1 — The per-frame data on the CPU

- `ScatterHandler` takes the `SSBOManager`, to read each body's transform.
- `ScatterHandler::update(params)`, called from `GraphicsEngine::render` next to the CDLOD
  update. For each instance it builds the body's pose with `bodyRenderPose`, the same function
  CDLOD uses, so the trees and the ground are placed with one camera and one rotation.
- Two small GPU structs, each with a `static_assert` on its size like `CdlodInstanceData`:
  - **instance**: body rotation (as a `mat3x4`, like CDLOD), body scale.
  - **patch**: origin minus camera in the body frame (float), the patch's slot, and which
    instance it belongs to. Not the item count: only the GPU knows it, in the count buffer.
- The patch list is *compact*: one entry per live patch, rebuilt each frame. That is what lets
  E4 cull only the near patches by sorting the list and dispatching a prefix of it.

Why the subtraction is done on the CPU: the origin and the camera are millions of metres from
the centre; their difference is small near the camera, where precision matters. Same reasoning
as the cheat sheet's "why positions are split up".

## E3.2 — Draw commands, visible list, kind table

Created with the set, in `createSet`, since they depend only on its kinds and capacities:

- **Indirect command buffer**: one `DrawElementsIndirectCommand` per mesh. `count`,
  `firstIndex` and `baseVertex` come from `ScatterMeshes`; `baseInstance` is where the mesh's
  part of the visible list starts; `instanceCount` is set to 0 every frame and counted up by the
  culling pass.
- **Visible list**: `uvec2` entries, one part per mesh.
- **Kind table** (small SSBO): per kind its LOD distances, its first mesh index, its LOD count,
  and its colour and bounding radius.

## E3.3 — The culling pass

`scatter_cull.comp`. One workgroup per patch-list entry, its threads striding over the patch's
items, up to the count the generation pass left in the count buffer. Per item: position
relative to the camera = patch offset + item position (times the body's scale, so distances
are in real metres); pick the first LOD whose distance it is within; past the last, skip it.
Otherwise reserve a place with `atomicAdd` on that mesh's `instanceCount` and write the entry.
Then a memory barrier so the draw sees the counts and the list.

No frustum test yet (E3.6), so items behind the camera are drawn and clipped. Correct, only
slower.

## E3.4 — Drawing into the G-buffer

- `scatter_vertex_shader.vert`: reads its entry from the visible list using `gl_BaseInstance +
  gl_InstanceID`, then the item, its patch and its instance. Rotates the vertex by the item's
  quaternion (`fromQuaternion` from `mesh_transform.glsl`), scales it, adds the item position
  and the patch offset, applies the body's scale and rotation. The normal goes the same way
  without the offsets. Colour from the kind table.
- Fragment stage: the shared `gbuffer_fragment_shader.frag`, with no textures (units -1), if it
  fits without changes; otherwise a small scatter one.
- `renderGeometry(params)`: bind program, VAO, the SSBOs and the command buffer, one
  `glMultiDrawElementsIndirect` per set. Called in the G-buffer pass after CDLOD. Reloads with
  the other shaders.

## E3.5 — A caller, for the first picture

**Moved (Markus, 2026-10-22): the test now lives in the game, not the engine.**
`addTestForest` in the game's `main.cpp`, called from `buildTestWorld` on the platform
grid's SSBO index: 4 × 4 patches of 32 m centred on the grid's origin, at the platform's top
face (z = −2.5 in the grid frame), so most trees hang in the air past the 20 m platform.
Snippet `media/scatter/test_forest.glsl`, meshes `media/models/tree/`. The engine example
is back to the plain spinning cube. Reasons: the grid is a real physics body, the game has
real camera controls, and the engine repo carries no test content.

Earlier decision, superseded: option (a), in the engine repo.

- In the one engine example, `example/main.cpp`, beside the spinning cube (Markus: one
  example, not a second executable). Snippet in `example/scatter_snippet.glsl`.
- A flat 128 m × 128 m ground quad, 1 m below the cube, on its own SSBO index; the trees are
  an instance on the same index. A slow spin of the ground checks that the trees turn with it.
- 4 × 4 patches of 32 m, payload `(x, y, 0, seed)`, 256 candidates each. Snippet: a 16 × 16
  jittered grid at z = 0, random turn about +Z, random scale, some dropped by hash.
- Meshes: the placeholder trees copied into the engine's `media/models/tree/`; Markus will
  overwrite them later.
- Tests LOD, cutoff and the body rotation; not large coordinates (the planet does that).

## E3.6 — Frustum test

Added to the culling pass: an item's bounding sphere (kind's radius × item scale × body scale)
against the camera's frustum planes, uploaded with the frame data.

## Questions for Markus

All four decided (Markus, 2026-10-22): as proposed below, and (a) for question 3.


1. **Visible list size.** Each mesh needs room for the worst case of every item using it. The
   simplest safe choice: every mesh's part is as big as the whole item buffer, so it can never
   overflow. Costs meshes × items × 8 bytes (6 meshes × 1 M items ≈ 48 MB). Alternative: a
   capacity per LOD in the config, smaller but another number to get right, and overflow must
   then be handled. Proposed: the simple one now, tightened in E7.
2. **Visible entries as `uvec2` (item index, patch-list index).** The vertex shader needs both
   the item and its patch's offset. Storing the patch-list index next to the item index keeps
   the patch list compact (needed for E4's prefix). Alternative: one `uint` and a patch buffer
   indexed by slot instead, which gives up the compact list. Proposed: `uvec2`.
3. **Where the first caller lives.** Decided: (a), an engine example on a flat ground. See
   E3.5.
4. **Frustum test after the first picture** (E3.6 as listed), so the first picture depends on
   less. Proposed: yes.
