# Frame Pacing Pulse (2026-09-27)

A once-a-second stutter seen for a few days at the end of September 2026. Found in one capture,
not reproducible afterwards, cause not identified. This note keeps the measurements and the
method so the next occurrence can be diagnosed from where this one stopped.

---

## Symptom

Flying in a straight line, the motion slows down in a pulse about once a second, near the planet
and far from it alike. The terminal prints no "Frame drop".

Capture: `recordings/005_pulse_lag` (server role, 42 s of frames, camera from 2 m to 5e7 m).

## What the capture shows

Monitor: one 2560x1440 at 164.835 Hz, so a vsync period is 6.07 ms.

Every second has the same shape, at the same place in the second for the whole 42 s:

| part of each second | frame time | what happens |
|---|---|---|
| x.00 to x.63 | 6.1 ms | full 165 Hz |
| x.63 to x.78 | 7 to 10 ms | gradual slowdown, not whole vsync periods |
| x.78 to x.94 | 12.1 ms | every other vblank, 82 Hz |
| x.94 | 6 ms | snaps back in one frame |

About 900 of 5818 frames are slow. The slow window sits on physics ticks 40 to 63 of every 64.
Ticks run at 64 per second of wall clock, so within one capture the tick phase and the
wall-clock phase cannot be told apart.

Recordings 000, 003 and 004 (August) show no such pattern.

## What was ruled out

- **CPU work in the game.** In the slow frames everything from frame start to the swap takes
  0.75 ms, against 0.44 ms in the fast ones.
- **GPU cost.** Replaying the capture with `OI_FRAME_PROFILE` gives 3.4 ms per frame, flat, and
  the frames that were slow in the capture cost the same as the rest.
- **All the extra time is inside `glfwSwapBuffers`**: 11 ms against 5.6 ms. This is also why no
  "Frame drop" is printed: `GameBase::advanceFrame` checks the budget before `endFrame`, so a
  swap that waits an extra vblank never reaches the check.
- **The game itself.** The same build replaying the capture's mouse and keyboard with live time
  flies the same path at 165 Hz with no cycle:

  | run | time and network | frames | slow (> 9 ms) |
  |---|---|---|---|
  | original capture | recorded | 5818 | ~900, in the cycle above |
  | replayed input | live | 7500 | 28, spread evenly |
  | replayed input | recorded | 9230 | 57, spread evenly |

  So neither the game's per-tick work nor the journal writes of RECORD mode cause it.
- **Something on a 64-tick period in the game.** The only one is the snapshot size log in
  `GameNetworkServer::frameSendRole` (`tick % 64 == 0`), one line in one frame; it cannot slow
  a third of a second.
- **GPU clock drop.** A live run sampled with `nvidia-smi -lms 20` held 2400 MHz in P0
  throughout (at a throttled 55 Hz, so a weaker test than wished).

## What is left

Something outside the game made the compositor or the driver hold back the swap for part of
every second while the capture was taken. Candidates, none confirmed:

- **Kernel 7.0.0-34** (installed 2026-09-24) and **NVIDIA 595.91.07** (from 595.84,
  2026-09-25), both matching "the last few days".
- **A busy process at the time.** Right after the capture a `gjs` process used 92% of a core,
  gnome-shell 24%, and packagekit/apt was downloading. The `gjs` process was gone seconds
  later and was never identified.

The session is Wayland, GNOME (Mutter), with swap interval 1, so the swap is paced by Mutter's
frame callbacks.

Also unresolved: `recording_time/time_data.bin` in 005 is exactly 819200 bytes (102400
stamps). If the session ran longer than 42 s of flying, the time stream was cut short.

## Next time it happens

Leave the game running while it pulses; the cause is only visible while it happens.

1. Busy processes: `ps -eo pid,etime,pcpu,args --sort=-pcpu | head`, and for any `gjs` its
   parent and arguments.
2. GPU: `nvidia-smi --query-gpu=timestamp,clocks.gr,utilization.gpu,pstate --format=csv -lms 20`
   for a few seconds.
3. Check `/var/log/dpkg.log` for driver, kernel, mutter or gnome-shell updates since this note.
4. Quit, then analyse the capture in `999_scratch` with the script below. Copy it to its own
   folder first, since the next RECORD run deletes it.

If the cycle stays on the same wall-clock phase across two captures started at different times,
it is outside the game; if it follows the physics tick, it is the game. A plain vsynced
application fullscreen at the same time answers the same question more directly.

## Method

### Frame times from a capture, no replay needed

`TimeHandler` in RECORD writes every clock read as an 8-byte `high_resolution_clock` stamp,
nanoseconds since the epoch. Frames are not marked, but the swap is the only long wait in a
frame, so a stamp that follows a gap of over 3 ms is a frame start. This matched the exact frame
starts of 005 frame for frame.

```python
import struct, statistics as s
d = open('recordings/005_pulse_lag/recording_time/time_data.bin', 'rb').read()
T = struct.unpack('<%dq' % (len(d) // 8), d)
starts = [T[i + 1] for i in range(len(T) - 1) if 3e6 < T[i + 1] - T[i] < 1e8]
dt = [(starts[i + 1] - starts[i]) / 1e6 for i in range(len(starts) - 1)]
slow = [0] * 10
for i, ms in enumerate(dt):
    if ms > 1.6 * s.median(dt):
        slow[int((starts[i] / 1e9) % 1 * 10)] += 1
print('median ms', s.median(dt), 'slow frames by tenth of the wall-clock second', slow)
```

A healthy capture spreads the slow frames evenly; 005 piles them into tenths 6 to 9.

### Exact frame starts and ticks, by replay

For the tick phase, replay the capture (`s_sessionMode` PLAY, `s_playbackDir` at the session)
with a temporary print in `GameBase::prepareFrame` of
`m_currentFrameStartTime.time_since_epoch().count()` and
`m_physicsEngine->getCurrentPhysicsTimeStep()`. The replay feeds back the recorded clock, so
these are the stamps of the original run. Add `OI_FRAME_PROFILE=<file>` to the same replay for
the GPU time of each frame.

### Replayed input with live time

To rerun the same flight at a new moment: `s_sessionMode` NONE (or RECORD, to also capture the
new run), and pass `GraphicsEngineBase::Mode::PLAY` and the capture's `recording_mouse_keyboard`
folder to `Game` in `main.cpp`. Nothing notices the input running out, so the game keeps going
until closed. The window must stay in front: hidden, Mutter throttles it to 55 Hz and the test
shows nothing.
