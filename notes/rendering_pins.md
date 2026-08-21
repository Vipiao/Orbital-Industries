# Rendering Pins

Work deferred out of the shadow overhaul, kept here so the reasoning does not have to be
rediscovered. Each pin states what it is, why it was left, and what has to hold for it to be
correct — the last part matters most, because two of these are easy to implement wrongly in a
way that looks fine until it does not.

---

## Measured baseline

Replay of `recordings/003_lattice_benchmark`, 8076 frames, GPU timings.

| pass   | mean    | median  | p05     | p95     |
|--------|---------|---------|---------|---------|
| shadow | 1.67 ms | 1.65 ms | 0.38 ms | 2.70 ms |
| light  | 0.51 ms | 0.50 ms | 0.23 ms | 0.78 ms |
| frame  | 3.51 ms | 3.71 ms | 1.37 ms | 5.01 ms |

**The shadow pass is ~48% of frame GPU time.** That is the number that makes Pin 1 the only
one of these worth doing for speed.

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

---

## Pin 1 — Per-cascade caster culling

`GraphicsEngine::renderShadowPass` draws the entire scene — every mesh, every instance, the
full CDLOD patch selection — once per cascade, with nothing tested against the cascade being
filled. Cascade 0 covers a 16 m x 16 m column and receives the whole planet.

Rejecting per cascade should collapse four near-full scene draws to roughly one, which against
a 1.67 ms shadow pass is the largest win available anywhere in the frame.

**The condition that makes it correct:** cull against the cascade's *extruded* volume — its
cross-section swept the full slab length along the light — never against the footprint that is
visible. A caster stands up-light of what it shadows by as much as `m_casterReach`; testing
what the camera can see would drop exactly the distant occluders the shared near plane exists
to keep. Get this wrong and shadows vanish only at certain sun angles.

## Pin 2 — Stable light basis and texel snapping

`beginShadowPass` rerolls the light basis every frame off `Hash::pcgUnit3(frameNum)`, so the
square rim of a cascade never sweeps the same texels twice. The round fade band hides *where*
the seam falls, but the roll still moves every sample in the outer cascade by metres per frame,
and it is what makes texel snapping impossible.

Texel snapping — quantizing each cascade's centre to whole shadow-map texels in light space —
is the only exact fix for shadow crawl; dithering only spreads it. It needs a basis that holds
still across frames, so the two are one change, not two.

Note that the cascades are now pushed along the view direction by `k_cascadePush`, so their
centres already move when the camera turns. Snapping has to quantize the pushed centre.

## Pin 3 — `getLightSpaceMatricesForViewSpace` allocates per frame

Returns `std::vector<glm::dmat4>` by value, one heap allocation per frame. Removing it means
changing the return type, which ripples into `DeferredRenderer`'s public signature — more
interface churn than roughly 100 ns per frame justifies. Worth folding in only if that
signature is being changed for some other reason.

(The two allocations *inside* `endGeometryPassAndRenderLighting` were local to the function
body and are already gone.)
