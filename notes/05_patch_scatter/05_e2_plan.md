# E2 — Generate on the GPU

Goal: a patch's items are made on the GPU by a snippet the caller injects. The CPU only says
where a patch is and hands over a small payload; it never sees the items. Nothing is drawn
yet; that is E3.

Decided (Markus, 2026-10-21): the GPU generates first; CPU agreement waits for G4.
`addPatch` loses its `items` parameter; the CPU-records path comes back in G4.

## The shape of it

```
CPU, when a patch is added (addPatch)
  take a free block, write the patch's payload, set the block's item count to 0
  dispatch the generation pass for that one patch

GPU, generation pass (compute), one thread per candidate
  ask the snippet: is this candidate kept, and if so where, how oriented, how big, which kind
  kept → atomicAdd on the block's item count → a place in the block
       → write a ScatterItem there, with the candidate id
```

Candidates per patch = `m_itemsPerPatch`, so a block can never overflow.

## E2.1 — A compute stage in `ShaderProgram`

`ShaderProgram` has vertex, fragment, tessellation and geometry stages. Add a compute stage the
same way (`loadComputeShader`, `loadComputeShaderFromPath`), so it reloads like the rest. A
compute program has only that one stage; linking checks that.

Small and general: nothing scatter-specific.

## E2.2 — Patches without items

A refactor of E1 code, before anything new is added on top of it:

- `addPatch(instance, origin, payload)`: the `items` parameter goes, and its checks with it.
- `ScatterPatch::m_itemCount` goes: only the GPU knows how many items were kept.
- Two new per-block buffers in `ScatterSet`, allocated with the item buffer:
  - **payload buffer**: one payload per block, written by `addPatch`;
  - **count buffer**: one `uint` per block, set to 0 by `addPatch`, counted up on the GPU.
- `ScatterItem`'s comment no longer says a caller fills it in.

## E2.3 — The snippet contract

`createSet(config, readSnippet)`, with `readSnippet` a `std::function<std::string()>` as in
`CdlodHandler::createSurface`: asked again on every shader reload, so a snippet kept in a file
is picked up when it changes. The set owns the generation program built from it; a broken
snippet on reload leaves the working program in place, as CDLOD does.

The snippet defines one function, roughly:

```glsl
// Returns false if the candidate is not kept.
bool scatterCandidate(uvec4 payload, uint candidate,
                      out vec3 position, out vec4 orientation,
                      out float scale, out uint kind);
```

The engine's side (`scatter_generate.comp`) owns the buffers, the thread layout, the
`atomicAdd` and the write, so the snippet only answers the question and cannot write anywhere.

## E2.4 — The generation pass

`scatter_generate.comp`, workgroups of 64 threads, `m_itemsPerPatch / 64` groups (rounded up)
for one patch. Each thread: its candidate index → the snippet → if kept, `atomicAdd` on the
block's count and write the item. Then a memory barrier so the culling pass in E3 sees the
items and the count.

Dispatched straight from `addPatch`, one patch at a time. Spreading many new patches over
several frames (a budget) is E5, when the planet adds them by the hundred.

Nothing to see after this step; the buffers can be inspected in RenderDoc.

## Questions for Markus

1. **The payload.** Decided (Markus, 2026-10-21): a fixed `glm::uvec4` per patch. Enough for
   a cube face and two patch coordinates plus a seed; floats fit through `floatBitsToUint`.
2. **The snippet's outputs.** Decided (Markus, 2026-10-21): position, orientation, scale and
   kind, as above; the engine writes the candidate id and builds the record, so a snippet never
   depends on the record's layout. Checked against a future asteroid belt (3D cells, payload
   `(x, y, z, seed)`, full random orientation): fits unchanged.
3. **Dispatch from `addPatch`** right away. Decided (Markus, 2026-10-21). Simpler now; the
   queue arrives with the budget in E5.
