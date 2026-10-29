# Props on the planet

Trees (later bushes, rocks) standing on the planet's real ground, near the camera, drawn by
the scatter tool. Steps are named P1, P2... so they do not clash with E1–E7.

## Names (Markus, 2026-10-26)

| name | what it is |
|---|---|
| `PlanetPropsGraphics` | the graphics part of the planet's props, owned by `Planet` beside `PlanetWaterGraphics` |
| prop | one tree, bush or rock (the engine's item) |
| prop kind | tree, bush... (the engine's kind) |
| prop patch | one square of ground whose props are generated at once (the engine's patch) |

Always "prop patch" in game code, never bare "patch": the planet code already says patch
for CDLOD's ground squares. A physics part, and a `PlanetProps` class holding both parts,
come later.

## Progress

- 2026-10-26 — Plan accepted. P1 written: `lattice_surface.glsl` holds where the ground
  stands (plus `surfacePointAtHeight`), the new `cdlod_surface.glsl` the two CDLOD hooks,
  the underwater warp and the far-sea include. `planetSurfaceGlsl` lost the three far-sea
  figures to the new `farWaterGlsl`, in the same file. Builds without warnings; runs 20 s
  with no errors and loads the new file. Not yet looked at by eye.
- 2026-10-26 — P2 written (engine). `ScatterSetConfig::drawDistance()`.
  `scatter_generate.comp` includes `dekker_arithmetic.glsl` and passes `Df3 patchOrigin`
  to `scatterCandidate` (from `u_patchOriginHigh/Low`, split by `splitWide`).
  `ScatterHandler::setSnippetTexture` (sampler name + texture handle, unit = place in
  `m_snippetTextures`, bound in `generateBlock`). `GraphicsEngine` gains
  `createSharedTexture`, `createSharedCubeTexture`, `removeSharedTexture`,
  `setCdlodSurfaceTexture(surface, name, texture)` and `setScatterSetTexture`. A CDLOD
  surface removes only the textures it made itself (`m_madeBySurface`). Test forest takes
  the new parameter. Builds without warnings; runs 20 s with no errors. Nothing calls the
  texture functions yet. Markus ran it: ground and textured test forest look right.
- 2026-10-26 — `splitDekker(dvec3)` added to utils `math/DekkerArithmetic.h`; it replaces
  the six hand-written splits in the engine (CDLOD, scatter, instance frame uniforms, mesh
  handler, ray volumes, SSBO manager). Builds without warnings; runs 20 s with no errors.
- 2026-10-26 — `paddedRotation(dmat3)` in engine `BodyPose.{h,cpp}` replaces the two
  `paddedColumns` copies (CDLOD, scatter). Builds without warnings; runs 20 s with no
  errors.
- 2026-10-26 — P3 decided: prop patches 128 m with 32 × 32 candidates (one per 4 m, six
  in ten kept), on an equal-angle grid (78184 patches per face edge at this radius).
  `PlanetType` owns the ground maps (`createSharedTexture`) and hands them to the ground
  and the props.
- 2026-10-26 — P3 written. New `src/world/PropPatchGrid.{h,cpp}` (keys, origins, which
  patches are near a direction, across face edges); `CdlodCubeFaces` gains public face
  accessors; `propPatchGridGlsl` writes the grid's figures. New
  `src/world/planet_props.glsl` (integer keep / cell / nudge / size; edges from the face
  tables; height through `gatherHeightLevels` and `surfacePointAtHeight`; no turn yet).
  New `PlanetPropsGraphics` (the set's config and snippet, the instance, the live prop
  patches; range = `drawDistance()`, 16 m margin, gate on the camera's height above the
  ground from `PlanetSurface::surfacePoint`). `PlanetType` owns the set and the grid;
  `Planet` owns a `PlanetPropsGraphics`. Builds without warnings; runs 20 s with no
  errors (the start is 800 m up, so no prop patch is added there). Scratch checks: the
  near search misses nothing over 3000 random directions, edges and corners included;
  the snippet's edges reach the next corner within 2.4 mm. Markus looked: trees placed
  on the ground, LODs work; they lie along the body's +Z (no turn yet, P4) and stand
  under water too (no filter yet, P5).
- 2026-10-26 — `CdlodCubeFaces` renamed `CubeFaces` (`src/world/CubeFaces.{h,cpp}`):
  it is the body's cube, read by the ground, its bounds and the prop patch grid.
- 2026-10-26 — P4 written, in `planet_props.glsl` only. A prop's +Z stands straight up
  from the body's centre (against gravity, not along the slope); its turn about up is
  16 hash bits, measured from the patch's U edge laid flat, so no spot on the body
  breaks down. The frame becomes a quaternion in `propQuaternion`. Builds; runs 20 s
  with no errors (the snippet compiles). Markus looked: trees stand right.
- 2026-10-26 — Engine `shared_shaders/math_constants.glsl` (`k_pi`, `k_twoPi`), included
  by `phong_lighting.glsl` (which drops its own `k_pi`) and `scatter_generate.comp`, so
  every snippet has them. Written-out pi values replaced in `planet_props.glsl`,
  `lattice_surface.glsl`, `cdlod_surface.glsl`, `test_forest.glsl`, `plume_body.glsl`
  and `lighting_fragment_shader.frag`. Terrain switched from dunes to
  `RIDGED_MULTIFRACTAL` for testing (`TerrainDisplacement::k_synthesis`). Builds; runs
  20 s with no errors, every changed shader loaded.
- 2026-10-26 — P5 written (thinning, Markus's choice over a hard cut). Reuses the ground's
  own code: `terrain_displacement.glsl` splits `coverMaterial` into `coverShares` /
  `terrainCover` (sand, grass, rock, snow summing to one; dunes all sand) and colours
  from them; `lattice_surface.glsl` gains `gradientAcrossSphere`, which
  `cdlodSurfaceShading` now calls. The snippet reads height and slope in one
  `gatherLevels`, drops anything below the shore, and holds the integer keep draw to
  `k_propKeep × grass share`: trees thin towards beaches, rock flanks and the snow
  line exactly where the ground's colour changes. Builds; runs 20 s with no errors.
  Markus flew over the terrain: looks good.
- 2026-10-26 — P7 decided (see the P7 section below). P7.1 written, tested on the real
  path rather than standalone: the scatter set bakes a kind's impostor when it is
  created. Engine: new `src/graphics/impostor/` — `ImpostorConfig.h` (GL-free bake
  spec), `ImpostorBaker.{h,cpp}` (own framebuffer, saves and restores GL state, the
  engine's reverse-Z; reads back, fills see-through texels per cell, uploads with
  mips; `writeImages` for PNGs), `impostor_views.glsl` (hemi-octahedral cell ↔
  direction, view axes; shared with the drawing side to come), `impostor_bake.vert/
  .frag` (shades through `material_decode.glsl`, so the picture matches the mesh).
  `ScatterKind::m_impostor` (`ScatterImpostor`: distance + bake spec), `ScatterMeshes`
  gains a tight sphere per mesh, `ScatterSet::m_impostors`, baked in `createSet`,
  removed in `removeSet`; `writeScatterImpostorImages` on `GraphicsEngine`. Game: the
  tree kind has an impostor (1000 m, not used until P7.2 draws it), and
  `PlanetPropsGraphics::createSet` writes `impostor_0_{color,normal,coverage}.png`
  beside the binary every run. Builds without warnings; runs 20 s with no errors.
  Atlases checked by eye: side views around the rim, top views in the middle, upright,
  borders clear.
- 2026-10-26 — P7.2 written. `ScatterMeshes` appends one quad per kind with an impostor
  after all LOD meshes (`impostorMesh`, `getLodMeshCount`, `k_noMesh`); the impostor's
  distance follows the LOD distances in the same order. `ScatterKindData`'s padding is
  now `m_impostorMesh`; new per-kind `ScatterImpostorData` buffer (sphere, atlas units,
  views, pixels, border), binding 10, filled after the bake (the atlases join the set's
  texture units). Cull: past the last LOD, the impostor quad out to its distance.
  Draw: the LOD meshes with the G-buffer program, then the quads with the new
  `scatter_impostor.vert/.frag` (nearest cell to the view direction, quad in that
  cell's view plane; level capped at a 4-texel cell, coordinates clamped half a texel
  inside the cell; coverage cut at one half; baked normal turned to view space).
  `drawDistance()` includes the impostor; config validation checks it lies past the
  LODs. Game: impostor to 300 m for this first test, `k_maxPatches` 64 → 128. Builds
  without warnings; runs 20 s with no errors (the start is 800 m up, so nothing is
  drawn there). Not yet looked at by eye.
- 2026-10-26 — The two scatter vertex stages share `scatter/scatter_item.glsl` (the
  item, instance, patch and visible buffers; `scatterDrawnItem`, `scatterBodyOffset`,
  `scatterCameraRelative`), so placement is written once. 195 → 190 lines across the
  three files. Runs 20 s with no errors.
- 2026-10-27 — Markus looked at P7.2: impostors almost identical to the 3D trees, works
  as expected. P7.3 (the tree kind's impostor) came with P7.2.
- 2026-10-27 — P7.4 written, straight to 2 km (Markus). Engine: one memory barrier per
  frame at the start of `ScatterHandler::update` instead of one per generation
  dispatch. `PropPatchGrid::maxPatchesWithin` bounds the patches within a reach (circle
  of reach + two widest half-widths over the smallest area, measured at a face's middle,
  edge middle and corner); the set's `m_maxPatches` comes from it, no hand-typed number.
  `PlanetPropsGraphics`: search again only after 16 m of movement (search reaches 16 m
  further), add at most 16 patches a frame, nearest first, remove past range + 16 + 16.
  Impostor to 2000 m. Scratch check: bound 1362, most actually within reach 1182 over
  300 directions. GPU memory for the set at that bound: items 67 MB, visible lists
  45 MB. Builds without warnings; runs 20 s with no errors, with patches added from the
  start (800 m up is within range). Markus looked: looks very good, impostors almost
  3D, popping not too bad.
- 2026-10-27 — Tree LOD2 dropped (impostors from 50 m). New
  `media/models/tree/tree_material.png` (1×1, roughness 0.9, written with PIL) as the
  tree's material texture: the default roughness 0.35 was too shiny. The bake keeps
  roughness in the normal atlas's alpha (filled like the normal) and the impostor
  writes it, so both stay matte. `ScatterImpostor` gains `m_fadeBandMetres` and
  `m_fadeSinkHeights` (`ScatterImpostorData` 48 → 64 bytes with a `fade` vec4): over
  the band an impostor shrinks to nothing about the item's origin and sinks by that
  many mesh heights (height = top of the bake sphere). Trees: 300 m band, sink 1×.
  Builds; runs 20 s with no errors. Markus looked: looks good.
- 2026-10-27 — Visible-list shrinking (two-step cull) postponed by Markus. Instead of
  blending views, the trees bake 16 × 16 views (`m_viewsPerSide`, about 6° apart),
  a 2048² atlas, ~22 MB per kind; no runtime cost. Blending stays the next step if the
  pop still shows. Builds; runs 20 s with no errors; atlas written at 2048².
- 2026-10-27 — 32 × 32 views (Markus), about 3° apart: 4096² atlas, ~90 MB per kind.
  Builds; runs 20 s with no errors. Startup (from strace): the set's snippet compile,
  bake and read-back together about 0.8 s; writing the three debug PNGs another ~2.3 s
  and 234 MB on disk every run. Next: the camera-facing quad. Markus looked: good.
- 2026-10-27 — Camera-facing impostor quad, in `scatter_impostor.vert` only: the quad
  lies across the line to the camera on the screen's up (defined from every side, no
  singular point straight above); each corner's place on the chosen cell comes from
  projecting it onto the axes that cell was baked with. The quad moves smoothly; only
  the picture changes at a cell boundary. Builds; runs 20 s with no errors. Markus
  looked: good.
- 2026-10-27 — View blending benchmarked (Markus recorded a forest flight, then slept).
  Recording copied to `recordings/007_forest_benchmark` before any run (5709 frames).
  B = the four nearest views blended (bilinear weights over cell centres, each view's
  coordinates by projecting onto its own axes, 8 reads per pixel), A = the engine's
  one-view shaders. GPU time per frame from `OI_FRAME_PROFILE` (GL_TIME_ELAPSED around
  render, vsync off), which the replayed clock does not reach; both variants submit
  the same work. 6 rounds of A B B A, first 600 frames skipped:
  - A 4.709 ms, B 4.991 ms mean: **B +0.28 ms, +6.0%** (median +6.0%, p99 +0.56 ms,
    +9.7%). B slower in 99% of frames; run-to-run spread 0.02–0.03 ms; GPU 44–46 °C.
  - Control C (impostors clipped away, vertices still run), 3 rounds of A C C A:
    drawing impostor pixels costs A 0.13 ms (2.7% of the frame). So blending makes the
    impostor drawing about 3× as costly (~0.41 ms against 0.13 ms).
  - Not adopted: the engine keeps A. B's shaders, the scripts and every run are in
    `recordings/007_forest_benchmark/benchmark/` (`shaders/scatter_impostor_{A,B,C}`,
    `blend_ab/`, `impostor_cost/`; the scripts name the scratch folder they ran from).
    B was not looked at by eye.
  - `main.cpp` back to `RECORD`; `s_playbackDir` now names 007. The recordings README
    has no row for 005–007 yet (Markus writes it).
- 2026-10-27 — Markus looked at B and adopted it. Then 16 × 16 views at 224 px cells
  (192 px picture, 16 px border): a 3584² atlas, ~137 MB for the pair with mips. The
  mip cap is now the coarsest level whose cells are whole texels and ≥ 4 across
  (`min(findLSB(cell), findMSB(cell / 4))`), so non-power-of-two cells stay clean.
  Markus looked: good. `s_sessionMode` left at `NONE` (Markus).
- 2026-10-27 — Comments trimmed across the feature (engine scatter/impostor ~577 → 249
  comment lines, game props ~165 → 86). Fixes: `ScatterMeshes::meshIndex`'s assert
  bounded the last kind's LODs by the impostor quads too; the bake's incomplete
  framebuffer path now frees its targets and restores state; the bake saves and
  restores the read framebuffer too. New checks: visible-list size within 2^32
  (throws), double slot release, slot range, one LOD distance per mesh, bake mesh
  non-empty, both atlases one size, grid face/column/row range, unit direction,
  camera not at the centre, patch room undersized, `k_candidatesPerSide <= 256`.
- 2026-10-27 — Sinking is the game's idea, not the engine's (Markus). The engine's
  `m_fadeSinkHeights` became `m_fadeOffsetMetres`, a neutral offset in the mesh's frame
  reached at the band's end; the game sets it to (0, 0, −`k_treeHeightMetres`). Chosen
  over a vertex-stage hook for the game: one effect is data, a hook pays off only
  once there are several. `ScatterImpostorData` is 80 bytes.
- 2026-10-28 — Multi-material kinds, for an EZ-Tree ash (Markus picked texture arrays
  over an offline bake to one atlas). A kind has a list of `ScatterMaterial`s, matched
  to the model's sub-meshes by material name; each texture role becomes a
  `Texture2DArray` with a layer per material (`TextureStore::createArrayFromFiles`,
  layers of one size, a missing texture filled so it decodes as none). The layer is a
  per-vertex float at location 4 in a buffer of its own; `material_decode.glsl` samples
  arrays under `MATERIAL_TEXTURE_ARRAYS`. UVs are the OBJ's own; no .mtl is needed, as
  Assimp keeps the `usemtl` names without it. Scatter meshes cut at alpha 0.5 and
  draw back faces with the normal flipped, the bake the same. The ash: 3 LODs (13.6k,
  3.7k, 2.0k triangles, to 25, 50, 100 m), 27.7 m tall, files in
  `media/models/ash_tree/`; the bark colour scaled to 1024² to match the leaves.
- 2026-10-28 — The L0 → L1 pop: L1 and L2 held a loose edge (`l` in the OBJ), which
  Assimp keeps as a 2-index face, shifting every later triangle's indices. `AssimpLoader`
  now drops points and lines (`aiProcess_SortByPType`, `AI_CONFIG_PP_SBP_REMOVE`).
  Impostors bake from the last LOD, the one they take over from. Leaf normals point
  out from the crown's centre (written into the OBJs by script for now; the lasting
  way is a Normal Edit modifier in Blender), and back faces keep their normal, so a
  card is lit alike from both sides.
- 2026-10-29 — The ash cost ~2 ms GPU over the old tree (7.04 vs 4.99 ms on
  `007_forest_benchmark`): size, not triangles; ~1.4 ms of it impostors, whose quads
  are 74% cut-out pixels that read all 8 textures before discarding. Benchmarked:
  tight per-view quads (−0.03 ms, the crown fills ~80% of the square; dropped), view
  culling (−0.10), a depth pre-pass (−0.54), impostor colour read before normals
  (−0.34). Kept the last three (Markus): 6.30 ms, −10.5%. The cull pass drops items
  whose sphere (`m_boundingRadius`, now holding the impostor quad) is outside the
  view's side planes. Items are drawn depth-only first, alpha cut, then shaded with
  depth `EQUAL` and early fragment tests; `invariant gl_Position` keeps the passes
  alike. The cutoff is a uniform, `u_alphaCutoff` (1/255 by default, 0.5 for scatter
  and its bake).
- 2026-10-29 — LOD crossfade (Markus): `ScatterLod::m_crossfadeMetres`, the ash 2.5 /
  5 / 10 m. In the band the cull pass draws the next mesh too, and both dither with
  complementary halves of one screen noise (`shared_shaders/dither.glsl`, interleaved
  gradient noise shifted per item). Stateless: each vertex shader works out its own
  fade from distance; the LOD meshes find their mesh by `gl_DrawID`.
- 2026-10-29 — Impostor depth per pixel, against crossing quads popping (Markus, then
  slept). The bake writes a third atlas, R8: surface depth along the view, ±radius.
  Benchmarked on `007_forest_benchmark` against 6.49 ms: quad at the centre and the
  depth written in both passes 11.85 ms (every test late); quad moved along the rays
  to the sphere's front with depth only pushed back (`depth_less`, keeps early
  rejection) 7.61 ms with the pre-pass, 6.96 ms with impostors in one pass, 7.22 with
  everything in one pass. Flat quads in one pass 6.60, everything in one 6.87, so the
  pre-pass still pays for the LOD meshes. Kept: front quads, impostors in one pass,
  6.95 ms (+0.45, +7%).

## The plan

### Who owns what

The same split as the ground. One scatter set per `PlanetType`, like its `CdlodSurface`:
the meshes, the snippet and the terrain textures are the same for every planet of that
type. One scatter instance per `Planet`, on the planet's SSBO index, like its
`CdlodInstance`. `PlanetPropsGraphics` holds the instance and the planet's live prop
patches.

### Every frame

`Planet::updateGraphics(cameraPos)` calls `PlanetPropsGraphics::updateGraphics`. It puts
the camera in the planet's frame. If the camera is within range of the ground, it adds
every prop patch within range that is not there yet and removes those that have left it
(with a margin, so a camera on the border does not add and remove the same one each
frame). Range: read from the set's config, the furthest LOD distance of any kind, so it
is written once (a small engine helper on `ScatterSetConfig`).

### Where a prop patch is

A fixed grid on each of the six cube faces the planet is built from, so a prop patch is
named by three whole numbers (face, column, row) and lands in the same place every time.
Its payload is (face, column, row, seed). Its origin is worked out on the CPU in double.

### Placement in whole numbers (Markus)

Every decision about a candidate is made in integers, so a CPU twin later reaches the same
answer bit for bit: the prop patch (face, column, row), the grid cell in it, the jitter
(hash bits as a fixed-point fraction of a cell), keep or drop by density, kind, turn and
size (hash bits). Integer hashing is exact on both sides. Only the last step, turning that
grid spot into a point on the ground, uses floating point; the ground height and the water
and slope filters depend on it. Making those agree on both sides is deferred.

### What the snippet does per candidate

1. Where on the ground: a spot inside the prop patch, from the integers above.
2. How high: read through the same GLSL functions and the same textures the ground is
   drawn with (P1, P2). One elevation code, not a copy.
3. Leave it out under water (below the shore height) or where the ground is too steep.
4. Which way is up: straight out from the planet's centre at that spot. The snippet works
   it out itself from the spot; nothing extra needs passing in. Then a random turn about
   that up.
5. Return the position relative to the prop patch's origin.

### Steps

- **P1 — game, refactor.** Split `lattice_surface.glsl`. Today one file holds both "how high
  is the ground here" and the things only the ground's own drawing needs (the far sea
  flattening, the underwater warp, colour and normal). The first part moves to a file any
  shader can include. Nothing looks different.
- **P2 — engine, snippet inputs.** General, nothing planet-specific; the engine stays
  unaware of the game. A helper giving a set config's furthest LOD distance, and three
  things the generation pass lacks:
  - wide (double-float) arithmetic in scope;
  - the prop patch's origin, so the snippet can subtract it;
  - textures for a set's snippet, sharing the texture objects the ground already has.

  The test forest is unchanged.
- **P3 — game, `PlanetPropsGraphics`.** The set on `PlanetType`, the instance on `Planet`,
  prop patches added and removed around the camera, and a snippet that places trees on the
  ground height.
- **P4 — the snippet, up and turn.** Trees stand along the local up.
- **P5 — the snippet, filters.** No trees under water or on steep ground.
- **P6 — tuning.** Prop patch size, density, maximum prop patches, the range margin.

### P7 — impostors and a longer range (Markus, 2026-10-26)

Far trees as camera-facing billboards with baked colour and normals, so the forest is
seen out to a kilometre or more. Decided:

- Baked at startup from the kind's LOD0 mesh and its textures, never kept as image
  files: change the mesh and the billboard follows.
- Octahedral: the tree seen from 8 × 8 directions over the upper half-sphere, one
  atlas cell each. The quad faces the camera fully and samples the cell nearest the
  view direction, so a forest seen from above shows the trees' tops, not edges.
- 128 px per view (a 10 m tree is ~100 px on screen at 100 m, where impostors start),
  a 1024² atlas per texture. One number in the kind's impostor config.
- Two textures: colour with alpha (the outline), and normals in the tree's own frame,
  turned by the item's rotation when drawn, so impostors are lit like the 3D trees.
- Against bleeding: an empty border (~8 px) around each view; the shader clamps its
  coordinates inside the chosen cell, shrunk by half a texel at the mip read; mips
  capped where a view is 4 texels wide; see-through pixels take the nearest tree
  pixel's colour and normal, so mips do not fringe dark.
- Engine changes are general (nothing about trees): a camera-facing impostor is a
  rendering idea any kind can use.

Steps, one at a time:

- **P7.1 — engine, `ImpostorBaker`.** Own folder, knows nothing of scatter. Given a mesh
  and its textures, renders the views into the two atlases (own framebuffer, viewport
  and program; restores GL state, `invalidateBindings`), with border, dilation and
  capped mips. A debug dump of the atlases to PNG to check them by eye.
- **P7.2 — engine, scatter impostor LOD.** `ScatterKind` gains an optional impostor
  (draw distance, view size, views per side). The set bakes it at creation and appends
  it as the kind's last LOD, a quad mesh; the cull pass is unchanged. Drawn by its own
  shader pair (face the camera, pick the view, clamp, alpha cut, turn the normal) into
  the G-buffer. Not in the shadow pass at first.
- **P7.3 — game.** The tree kind gets an impostor, first out to ~1 km.
- **P7.4 — range.** What the longer range costs: `k_maxPatches` sized from the range,
  generation spread over frames or batched when hundreds of prop patches arrive at
  once, memory measured. Possibly a sparser far set if needed.
- **Later.** Blend the three nearest views to hide the jump between them; impostor
  shadows.

### Known limits, accepted for now

- One generation dispatch per prop patch, at most 16 a frame. If that still hitches,
  batch the dispatches (origins into a per-slot buffer beside the payloads).
- Each mesh's part of the visible list has room for every item in the set, so the
  visible lists cost (meshes × items × 8 B), 45 MB at 2 km. Could shrink by sizing the
  impostor's part only, or a far set with fewer candidates per patch.

- The ground near the camera is drawn as flat triangles between its sample points, while
  trees stand on the exact height. A tree can float or sink by the difference.
- No CPU copy of where the trees are (collision comes with the physics part).
- No biomes: trees go on any dry, flat enough ground. The current planet is all sand dunes.
