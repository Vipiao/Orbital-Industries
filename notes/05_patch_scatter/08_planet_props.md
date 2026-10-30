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

## How it works

- **Ownership.** One scatter set per `PlanetType` (meshes, snippet, ground textures), like
  its `CdlodSurface`; one scatter instance per `Planet`, like its `CdlodInstance`.
  `PlanetPropsGraphics` holds the instance and the live prop patches.
- **Prop patches.** 128 m squares on an equal-angle grid on each cube face
  (`PropPatchGrid`), named (face, column, row); payload (face, column, row, seed), origin
  in double on the CPU. 32 × 32 candidates, one per 4 m, six in ten kept on grass.
  Searched again after 16 m of movement, added nearest first, 16 a frame, removed past
  range + 16 + 16 m. `m_maxPatches` from `PropPatchGrid::maxPatchesWithin`.
- **Placement in whole numbers (Markus).** Cell, jitter, keep, kind, turn and size are
  integer hashes, so a CPU twin can match bit for bit; only putting the spot on the ground
  uses floats. Height and slope come from the ground's own GLSL (`lattice_surface.glsl`,
  `terrain_displacement.glsl`); trees thin with the grass share, none below the shore.
  Up is straight out from the centre, turned about it by hash bits.
- **Figures from C++.** The snippet `planet_props.glsl` is preceded by
  `planetSurfaceGlsl`, `propPatchGridGlsl`, the candidate count and the scale range, so
  none of them is written twice.
- **The ash.** EZ-Tree, 27.7 m tall, `media/models/ash_tree/`: bark and leaves as two
  materials (texture arrays, a layer per material, matched by `usemtl` name). L0/L1/L2
  (13.6k, 3.7k, 2.5k triangles) to 25/50/100 m, each dithering into the next over its
  last tenth. Leaf normals point out from the crown's centre, as exported from Blender.
  Its materials must be named `branches.001` and `leaves.001`.
- **Impostors.** Baked at startup from the last LOD (`ImpostorBaker`): 16 × 16 views of
  224 px cells, colour, and normal with depth in alpha. Out to 2 km; the four nearest
  views blended to 400 m, the nearest alone beyond; per-pixel depth, the quad at the
  sphere's front; over the last 300 m they shrink and sink by the tree's height.
- **Drawing.** View culling in the cull pass; LOD meshes depth-only first, then shaded
  with depth `EQUAL`; impostors in one pass.
- **Shadows.** Patches carry a bounding sphere (`patchBounds`) and are grouped by caster
  tier like CDLOD's (`CasterTiers`); cascade k culls the first `m_tierEnds[k]` patches
  into its own draw list. The ash casts into 2 cascades (out to 64 m) with
  `ScatterShadow{LS.obj, 100 m, 20 m fade}`: few branches and 256 leaf cards, dithered
  out over its last 20 m. Impostors never cast.

## Progress

- 2026-10-26 — P1–P5: ground height split into an includable `lattice_surface.glsl`;
  engine snippet inputs (wide arithmetic, patch origin, snippet textures); prop patch
  grid, `PlanetPropsGraphics`, up and turn, thinning by grass share (Markus over a hard
  cut). Markus looked: trees stand right, thin at beaches, rock and snow.
- 2026-10-26/27 — P7: impostors (baked, octahedral views), range to 2 km (Markus). View
  blending benchmarked on `007_forest_benchmark` (5709 frames): +0.28 ms, adopted after
  Markus looked. The sink at the far band is game data (`m_fadeOffsetMetres`), not an
  engine idea (Markus).
- 2026-10-28 — Multi-material kinds and the EZ-Tree ash (texture arrays, Markus). A loose
  OBJ edge (`l`) shifted every later triangle's indices; `AssimpLoader` drops points and
  lines.
- 2026-10-29 — Ash cost ~2 ms over the old tree, by size not triangles. Kept (Markus):
  view culling (−0.10 ms), depth pre-pass (−0.54), colour read before normals (−0.34);
  LOD crossfade; impostor per-pixel depth (+0.45); one view past 400 m and depth in the
  normal atlas's alpha (−0.32). No gain: patch sorting, tight quads, octagons.
- 2026-10-30 — Tree shadows. Overnight on 007 against 6.63 ms without tree shadows,
  2 cascades: LODs as the camera picks them +6.09 ms; impostors turned to the light
  +3.48 (4 cascades +10.16); L3 at every distance +1.84. The cost is leaf-card fill in
  cascade 1 (~32 texels/m), not triangles. Kept (Markus): `ScatterShadow`, the ash with
  L3. Caster distance on `008_valley_benchmark` (low sun over a wooded ridge, valley in
  the last ~10 s; 3.92 ms without tree shadows): 2 km +4.85 mean (the valley 32.7 ms
  against 6.2), 150 m +0.31, 100 m +0.16 (7.0). Kept 100 m with a 20 m dither (Markus).
  Alpha-only depth read: no gain, the driver already drops unused reads.
- 2026-10-30 — `s_planetSpins` in `main.cpp`, off; 007 and 008 replay with it on.
- 2026-10-30 — Shadow mesh LS (Markus). Three crown-sized cards cost as L3's 146 (on
  007 / 008 against no tree shadows: +1.20 / +0.14 ms against +1.18 / +0.15; branches
  alone +0.10): the cost is the cards' rasterised area (1739 m² against 1879 a tree),
  alpha-cut texels included, not their count. Kept LS with 256 leaf cards, 2318 m²:
  +1.43 / +0.18 ms, as area predicts (~0.57 ms per 1000 m² on 007). The new L2 has more
  branch triangles (215 to 724): +0.04 ms on 007.

## Shadow optimisations, pinned (Markus, 2026-10-30)

One at a time on top of the L3 shadow mesh, each benchmarked and looked at.

- **Mip bias.** The casters' alpha read 3 mips coarser saved 0.55 ms on LOD casters;
  blurs leaf shapes. Measure on L3 first.
- **Cascade far-plane cull.** The shadow cull tests only side planes, so trees past the
  slab's far end are drawn and clipped. Likely under 0.1 ms.
- **Less leaf-card area in LS.** Cost goes with area: smaller cards, or cards cropped
  closer to their leaves (`leaves.png` is 16% opaque). Or an opaque crown shell (early
  depth works, blob shadows), maybe for cascade 1 only.
- **Cascade 1 resolution or radius.** Fill grows with texel density squared; terrain
  shadows soften too.

## Known limits, accepted for now

- One generation dispatch per prop patch, at most 16 a frame; batch them if it hitches.
- Each mesh's part of the visible list has room for every item: 45 MB at 2 km.
- The ground near the camera is flat triangles between samples, trees stand on the exact
  height, so a tree can float or sink by the difference.
- No CPU copy of where the trees are (collision comes with the physics part).
- No biomes: trees go on any dry, flat enough ground.
