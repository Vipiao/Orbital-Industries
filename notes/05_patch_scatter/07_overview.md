# Overview of the scatter code, with labels

A map to read the code by. Questions and answers point here by label (`D2`, `F1`...).
The letter E is skipped: E1–E7 already name the build steps in `02_steps.md`.

## A — What we're building

We want to draw a very large number of small objects on a planet: trees and bushes now,
asteroids around a planet later. Drawing each tree on its own would be far too slow, because
the CPU would spend all its time telling the GPU "draw this tree, now this one". So the GPU
does almost all of the work: it decides where the trees are, which ones are close enough to
bother with, and how detailed each one should be. Then it draws them all in one go.

- **A1 — The engine** (`graphics_engine`) holds the general tool, `ScatterHandler`. It
  doesn't know what a tree is; it knows how to make, sort and draw "things scattered over an
  area".
- **A2 — The game** (`Orbital-Industries`) says what the things are: which meshes to use,
  and a small piece of shader code (the **snippet**) that decides where each thing goes.

## B — The vocabulary

From the biggest unit to the smallest:

- **B1 — set**: one kind of scattered content, like "forest". It has its meshes, its snippet
  and its GPU buffers.
- **B2 — kind**: one type of object in a set, like "pine". A kind has several **LODs**: the
  same tree at decreasing detail (25 / 50 / 100 m in the test).
- **B3 — instance**: a set placed on a body (a planet, the platform grid). It moves and turns
  with that body.
- **B4 — patch**: a square area on that body, like 32 m × 32 m. Patches are added and removed
  as needed. Each owns a **slot**: its fixed block in the item buffer.
- **B5 — candidate**: a possible spot in a patch. Each patch tries a fixed number of them
  (256 in the test).
- **B6 — payload**: four numbers the game hands over with each patch (in the test: patch x,
  y, and a seed). The snippet reads them.
- **B7 — item**: a candidate the snippet kept. A tree with a position, rotation, size and
  kind; 48 bytes, living only on the GPU.

Ownership: the handler owns the sets, a set owns its instances, an instance owns its patches.
The game holds only `weak_ptr` handles.

## C — Once, when the game creates a set (`createSet`)

- **C1** — Load the LOD meshes and pack them all into one vertex and index buffer
  (`ScatterMeshes`), so a single draw call can cover them. Load the kinds' textures
  (colour, normal, material, mask) through the engine's texture store; each gets a texture
  unit, its place in the set's `m_textureUnits`.
- **C2** — Build the generation compute shader with the game's snippet spliced in at the
  marker line.
- **C3** — Allocate the set's GPU buffers: items, payloads, counts, visible list.
- **C4** — Write the tables that never change: the kind table (colour, texture units,
  first mesh, LOD count, radius), the LOD distances, and one draw command per mesh. Each mesh gets its own
  part of the visible list, starting at `baseInstance`.

## D — When a patch is added (`addPatch`)

- **D1 — CPU**: take a free slot, write the patch's payload, set its item count to 0.
- **D2 — GPU, generation pass** (`scatter_generate.comp` + snippet): one thread per
  candidate runs the snippet, which answers "is there a tree here, and what does it look
  like?". Kept ones are appended to the patch's block with `atomicAdd` on its count.

After this the CPU never sees the trees again.

## F — Every frame

- **F1 — CPU** (`update` → `buildFrameLists`): for each instance, the body's rotation and
  scale this frame; for each patch, "patch origin minus camera" in double precision, then
  narrowed to float. Upload these two short lists; reset every draw count to 0.
- **F2 — GPU, cull pass** (`scatter_cull.comp`): for every item, its distance to the camera
  picks a LOD, or skips it if too far. The item is added to that mesh's part of the visible
  list, and that mesh's draw count goes up by one.
- **F3 — GPU, draw** (`renderGeometry` + `scatter_vertex_shader.vert`): one multi-draw call;
  each mesh draws as many copies as F2 counted. The vertex shader looks up which item it is,
  then places the vertex: item rotation and size, item position, patch offset, then body
  scale and rotation. It passes on the kind's colour and texture units. `renderGeometry`
  binds the set's textures first. Output goes to the G-buffer through the fragment shader
  every instanced mesh uses (`gbuffer_fragment_shader.frag`, `material_decode.glsl`).

## G — The two ideas the design rests on

- **G1 — The CPU reads nothing back from the GPU.** The GPU writes the counts and the GPU
  reads them. That is why it is fast.
- **G2 — Large numbers are subtracted on the CPU in double.** Planet coordinates run to
  millions of metres, where a float loses detail. The CPU works out how far the patch is from
  the camera, a small number, and only that goes to the GPU. The terrain (CDLOD) does the
  same, with the same body pose, so trees and ground stay together.

## H — Where it lives

**H1 — Engine**, `src/graphics/scatter/`:

| file | part |
|---|---|
| `ScatterConfig.h` | B, the 48-byte item (B7) |
| `ScatterMeshes.{h,cpp}` | C1 |
| `ScatterHandler.h` | B1–B4 as classes, the per-frame GPU structs (F1), the buffers (C3) |
| `scatter_data.glsl` | the same structs on the GPU (binding numbers come from `ScatterHandler.cpp`) |
| `ScatterHandler.cpp` | C, D1, F1, and the calls that start D2, F2, F3 |
| `scatter_generate.comp` | D2 |
| `scatter_cull.comp` | F2 |
| `scatter_vertex_shader.vert` | F3 |

Elsewhere in the engine: `ShaderProgram` can hold a compute shader (C2), and `GraphicsEngine`
forwards the calls and runs `update` and `renderGeometry` next to the terrain.

**H2 — Game**:

| file | part |
|---|---|
| `main.cpp`, `addTestForest` | the caller: createSet, createInstance, 4 × 4 addPatch |
| `media/scatter/test_forest.glsl` | the snippet run in D2 |
| `media/models/tree/` | the three placeholder LODs and `tree_color.png` (left half trunk, right half crown), loaded in C1 |

## I — Not there yet

- **I1 — Frustum test** (step E3.6): trees behind the camera still reach the GPU and are
  clipped there. Correct, only slower.
- **I2 — Shadows** (step E4).
- **I3 — A CPU copy of the trees**, for collision. Deliberately left for later.
- **I4 — Fading at the range limit**: past the last LOD distance a tree just vanishes.

## Reading order

Each entry names the parts it covers.

1. Game `main.cpp`, `addTestForest` (line ~45). **A2, H2.** The interface as a user sees it;
   read it first to know what the engine is asked to do.
2. Game `media/scatter/test_forest.glsl`. **B5–B7, D2.** What the snippet answers per
   candidate.
3. `ScatterConfig.h`. **B.** The vocabulary and the item.
4. `ScatterHandler.h`. **B1–B4, C3, F1.** Classes, ownership, buffers, the per-frame structs.
5. `scatter_data.glsl`. The same structs on the GPU. The binding numbers it names are
   defined by `addScatterDefines` at the top of `ScatterHandler.cpp`.
6. `ScatterMeshes.{h,cpp}`. **C1.**
7. `ScatterHandler.cpp`, top to the `ScatterSet` constructor (~line 72). **C3, C4.**
8. `ScatterHandler.cpp`, `createSet`, `buildGenerateProgram`. **C2.**
9. `ScatterHandler.cpp`, `addPatch`, `generateBlock`. **D1.**
10. `scatter_generate.comp`. **D2.** Reread the snippet beside it.
11. `ScatterHandler.cpp`, `update`, `buildFrameLists`, `cull`. **F1, G2.**
12. `scatter_cull.comp`. **F2.**
13. `ScatterHandler.cpp`, `renderGeometry`. **F3.**
14. `scatter_vertex_shader.vert`. **F3, G2.**
15. Smaller: `ShaderProgram.{h,cpp}` (compute stage, C2), `GraphicsEngine.{h,cpp}`
    (forwarders, the `update` and `renderGeometry` calls, shader reload).
