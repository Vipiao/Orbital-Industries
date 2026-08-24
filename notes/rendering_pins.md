# Rendering Pins

Work deferred out of the shadow overhaul, kept here so the reasoning does not have to be
rediscovered. Each pin states what it is, why it was left, and what has to hold for it to be
correct — the last part matters most, because two of these are easy to implement wrongly in a
way that looks fine until it does not.

---

## Measured baseline

Replay of `recordings/003_lattice_benchmark`, 8076 frames, GPU timings.

Measured with per-cascade caster culling on and off, the two runs differing only in the
margin constant so nothing else moves between them.

| pass   | unculled | culled  | change |
|--------|----------|---------|--------|
| shadow | 1.70 ms  | 0.31 ms | −82%   |
| light  | 0.55 ms  | 0.53 ms | −3%    |
| frame  | 3.57 ms  | 2.08 ms | −42%   |
| wall   | 4.02 ms  | 2.49 ms | −38%   |

The shadow pass was ~48% of frame GPU time and is now ~15%.

### How to measure

Two clocks are unavailable here and both mislead if used anyway:

- **Wall clock is pinned by vsync.** `m_swapInterval` defaults to 1, so frame time reads as
  the refresh interval no matter what the GPU did. Benchmark runs set it to 0.
- **`TimeHandler::now()` replays recorded time points.** Under `Mode::PLAY` it returns what
  the recording stored, not what the clock says, so anything timed through it measures the
  session that was captured rather than the one being run.

So timing goes through `GL_TIMESTAMP` queries on the GPU timeline instead. Unlike
`GL_TIME_ELAPSED` those interleave, so an inner pass can be bracketed inside an outer one;
results are read a few frames late so the readback never stalls the pipeline. Discard the
opening ~120 frames — shader compiles, texture uploads and the CDLOD tree settling all land
there.

Run-to-run spread on identical binaries is about **0.2%**. Treat anything under ~0.5% as noise.

To A/B a culling change without touching anything else, set `k_casterBoundsMargin` in
`CdlodHandler.cpp` to a huge value: every patch then falls in tier 0, every cascade draws the
whole selection, and the build reproduces the unculled behaviour exactly.

---

## Pin 1 — Per-cascade caster culling — **DONE**

Implemented. `ShadowRenderer` exports one `Cylinder` per cascade in camera-relative world axes;
`CdlodHandler` carries those into each body's frame, stamps every selected patch with the
innermost volume it can cast into, and counting-sorts the selection so a cascade draws a prefix
of what the camera draws whole. The tier travels down on `FrameRenderParams::casterTier`, whose
default asks for everything, so the handlers that do not group their work needed no changes.

Typical selection: 2034 patches total, of which cascade 0 draws 11 and cascade 1 draws 103.

Two properties worth not losing:

- **Omission is correct without any nesting assumption.** A patch is skipped for cascade *c*
  only when it is in none of volumes 0…*c*, and so in particular not in *c*. Nesting affects
  only how much is drawn needlessly.
- **The volume is a cylinder, not the ortho box, and that is exact rather than conservative.**
  Under light this parallel a caster's perpendicular offset from the axis equals its shadow's,
  so a caster outside the inscribed radius can only shadow points the lighting pass never
  assigns to that cascade.

Still uncalled: `MeshHandler` and `InstanceHandler` draw everything into every cascade. Meshes
would need a bounds concept that may not exist yet; per-instance culling means rebuilding the
instance buffer per cascade and is not localized. Neither is worth touching until measurement
says they cost something.

## Pin 2 — Stable light basis and texel snapping

`updateCascades` rerolls the light basis every frame off `Hash::pcgUnit3(frameNum)`, turning the
texel lattice about the light axis so the staircase along a shadow edge lands somewhere new each
frame rather than standing still.

It does **not** affect what a cascade covers. The lighting pass takes each cascade out to the
sphere inscribed in its square cross-section, and no roll of a square moves its inscribed
circle, so the rim never enters the picture and the caster volumes are round for the same
reason. This is also why the roll costs nothing in culling: the buckets are roll-invariant, and
would stay valid even if the selection were ever cached across frames.

What the roll does cost is texel snapping — quantizing each cascade's centre to whole
shadow-map texels in light space, which is the only exact fix for shadow crawl rather than a
way of spreading it. Snapping needs a basis that holds still, so dropping the roll and adding
snapping are one change, not two. The trade is a stable lattice against a moving one: crawl
disappears, and whatever aliasing the roll was dithering stops being dithered.

Note the cascades are pushed along the view direction by `k_cascadePush`, so their centres move
when the camera turns. Snapping has to quantize the pushed centre.

## Pin 3 — `getLightSpaceMatricesForViewSpace` allocates per frame

Returns `std::vector<glm::dmat4>` by value, one heap allocation per frame. Removing it means
changing the return type, which ripples into `DeferredRenderer`'s public signature — more
interface churn than roughly 100 ns per frame justifies. Worth folding in only if that
signature is being changed for some other reason.

(The two allocations *inside* `endGeometryPassAndRenderLighting` were local to the function
body and are already gone.)
