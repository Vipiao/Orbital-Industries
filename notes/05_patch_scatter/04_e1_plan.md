# E1 — Items on the GPU

Goal: a caller can describe kinds of items, place an instance on a body (its SSBO index), and
hand over patches of items; the engine packs the meshes and puts the items in GPU memory.
Nothing is drawn yet. That is E2, so no code in E1 exists only to be thrown away.

All code is in the engine repo, `src/graphics/scatter/`, except E1.4 (one existing file) and
E1.5 (game media). Each sub-step is reviewed before the next begins.

## The shape of it

```
ScatterHandler                 owns the sets
 └─ ScatterSet                 one group of kinds; later also the snippet and its programs
     ├─ meshes                 every kind's LOD meshes packed into one vertex + index buffer
     ├─ item buffer (SSBO)     maxPatches blocks of itemsPerPatch items, allocated once
     └─ ScatterInstance        the set placed on one body, through the caller's SSBO index
         └─ ScatterPatch       an origin in the body's frame, and a slot (block) in the buffer
```

Same ownership pattern as CDLOD: the handler owns sets, a set owns its instances, an instance
owns its patches; callers hold `weak_ptr` handles that expire when the owner goes.

Why a set level above the instance: two planets of the same type share kinds, meshes and (from
E4) the generation snippet, just as they share a `CdlodSurface`.

## E1.1 — Vocabulary: `ScatterConfig.h`

GL-free structs, so the engine's public header can carry them (like `CdlodConfig.h`).

- `ScatterLod` — one mesh of a kind: model path, and the distance out to which it is used.
- `ScatterKind` — a kind's LODs, nearest first, and a colour (textures come later). The last
  LOD's distance is the kind's maximum distance; a separate field would be a second source of
  truth.
- `ScatterSetConfig` — the kinds, `maxPatches`, `itemsPerPatch`. Fixes the item buffer's size.
- `ScatterItem` — one item, exactly as the GPU stores it: position relative to the patch
  origin and scale (one vec4), orientation quaternion (vec4), kind index, candidate id, two
  pads. 48 bytes, the recipe's plain record; shrinking it is E-later. A `static_assert` pins
  the size, as `CdlodInstanceData` does.

The same record is what the generation pass writes in E4. A caller supplying items itself is
therefore not a separate path, which is what keeps exact collision possible later.

## E1.2 — Packed meshes: `ScatterMeshes.{h,cpp}`

Loads every kind's LOD models through `AssimpLoader`, into one vertex buffer (the standard
`GeometryVertex` layout) and one index buffer, and remembers for each (kind, LOD) where its
indices start, how many there are, and its base vertex. Those three are the mesh fields of an
indirect draw command, which E2 fills in. Also records each kind's bounding radius, taken from
its largest mesh, so the caller does not have to supply one.

Why one buffer: a single multi-draw call can only reach meshes in the buffers bound to it.

## E1.3 — Handler, sets, instances, patches: `ScatterHandler.{h,cpp}`

Public surface:

- `createSet(config)` / `removeSet(set)` — packs the meshes, allocates the item buffer.
- `createInstance(set, ssboIndex)` / `removeInstance(instance)`.
- `addPatch(instance, origin, items)` / `removePatch(patch)` — origin in double, in the body's
  frame; items in patch-relative coordinates. Takes a free slot, uploads the items into that
  slot's block. Throws if the set is full or the patch has more items than a block holds.

Internals: a free list of slots per set; a patch remembers its slot, origin and item count.
Nothing per-frame yet — the camera offsets and the culling are E2.

## E1.4 — Facade: `GraphicsEngine`

`GraphicsEngine` owns a `ScatterHandler` and forwards the calls above, like the CDLOD ones.
Not called from any render pass yet.

## E1.5 — Placeholder tree meshes

Independent of the rest. A trunk and a crown at three LODs, generated once as OBJ files
**outside the repo** (Markus, 2026-10-14): they are temporary, to be replaced by proper trees.
`media/models/tree/` is an empty leftover from the reverted attempt and is not used.

Decided (Markus, 2026-10-14): they live in `/home/markus/repos/01_orbital_insdustries/
placeholder_models/tree/`, beside the game repo, written by `make_tree.py` there. A kind has
one colour, so trunk and crown draw alike for now; their UVs are in separate halves of the
texture so a two-colour texture can split them once textures arrive.

## Not in E1

Drawing, shaders, compute, per-frame updates, culling, LOD choice, shadows, generation,
impostors. Each has its own step in `02_steps.md`.
