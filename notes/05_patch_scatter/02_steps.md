# Steps

One at a time. After each: Markus reads, questions, suggests; then the next begins.
From E2.2 to the first picture (E3.5) the steps run on without a stop; Markus reviews all of
the code at the first picture (2026-10-22). `[w]` = written, awaiting that review.
Mark a step `[x]` when Markus has accepted it, and note anything that changed the design in
`01_design.md`.

## Engine

- [x] **E1 — Items on the GPU.** Vocabulary, packed meshes, sets / instances / patches,
  items uploaded. Nothing drawn. Detailed in `04_e1_plan.md`.
  - [x] E1.1 Vocabulary (`ScatterConfig.h`)
  - [x] E1.2 Packed meshes (`ScatterMeshes`)
  - [x] E1.3 Handler, sets, instances, patches (`ScatterHandler`)
  - [x] E1.4 Facade forwarders in `GraphicsEngine`
  - [x] E1.5 Placeholder tree meshes, generated outside the repo (temporary; Markus replaces
    them with proper trees)
- [ ] **E2 — Generate on the GPU.** Items made by an injected snippet; the CPU never sees
  them. Nothing drawn. Detailed in `05_e2_plan.md`.
  - [x] E2.1 Compute stage in `ShaderProgram`
  - [w] E2.2 Patches without items: payload and count buffers (refactor of E1.3)
  - [w] E2.3 The snippet contract (`createSet(config, readSnippet)`, `scatter_generate.comp`)
  - [w] E2.4 The generation pass (dispatched from `addPatch`)
- [ ] **E3 — Cull and draw.** First picture. Detailed in `06_e3_plan.md`.
  - [w] E3.1 Per-frame patch and instance data (`ScatterHandler::update`)
  - [w] E3.2 Draw commands, visible list, kind table
  - [w] E3.3 Culling pass (distance, LOD choice, cutoff)
  - [w] E3.4 Vertex shader and multi-draw into the G-buffer
  - [w] E3.5 A caller, for the first picture (written first, so each step can be run)
  - [ ] E3.6 Frustum test
- [ ] **E4 — Depth pass and caster tiers.** Patches sorted by tier on the CPU; per cascade,
  cull the prefix and draw it.
- [ ] **E5 — Snippet inputs and loading.** Named textures and uniforms for the snippet,
  budgeted patch loading, suppression by candidate id.
- [ ] **E6 — Impostors.** Bake colour and normal per kind at load; the impostor is the kind's
  last LOD mesh, turned to the camera in the vertex shader. Optional per kind.
- [ ] **E7 — Measure and tighten.** GPU timestamps as in `03_rendering_pins.md`; shrink the
  record; set capacities.

## Game

- [ ] **G1 — Patches on the planet.** The cube-face grid, which patches are in range, loading
  nearest first, unloading.
- [ ] **G2 — The placement snippet.** Hash, jitter, rules, standing on the terrain's surface.
- [ ] **G3 — Kinds and assets.** Trees and bushes.
- [ ] **G4 — CPU twin and collision.** Near patches supplied as records (brings back an
  `addPatch` that takes items); trunk capsules.
- [ ] **G5 — Destruction and network.**
