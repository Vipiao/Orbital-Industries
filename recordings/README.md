# Recorded sessions

One folder per session, numbered in the order they were captured. A session is
the three journals a replay needs, and they only mean anything together: the
time stream paces the frames, the control stream supplies mouse and keyboard,
and the network stream stands in for the peer.

    recordings/NNN_name/
        recording_time/time_data.bin
        recording_mouse_keyboard/{mouse,keyboard}_data.bin
        recording_network/{client,server}.bin

`main.cpp` names the mode in `s_sessionMode`, the folder a replay reads in
`s_playbackDir`, and the folder a capture is written to in `s_recordDir`. Those
last two are kept apart because recording deletes what it finds: a capture lands
in `999_scratch` and is moved to a number of its own once it is worth keeping.

| # | Session | Captured | What it holds |
|---|---|---|---|
| 000 | `terrain_benchmark` | 2026-08-16 | Single role, camera over the planet. Frame drops near a cube corner; the reference for terrain shading cost. |
| 001 | `client_cockpit_crash` | 2026-08-06 | Both roles' network journals, one time stream. Captured for a crash on cockpit entry. |
| 002 | `client_session` | 2026-08-06 | Client role only, no server journal. |
| 003 | `lattice_benchmark` | 2026-08-19 | Single role, camera over the planet. The reference for the lattice terrain's shading cost, replacing 000 now that the surface reads four planes rather than three. |
| 004 | `digibot_leg_offset` | 2026-08-26 | Server role. A digibot stands on an isolated block pushed off the platform and holds the face-centring key; its feet settle to one side of it. |
| 999 | `scratch` | — | Where `RECORD` writes. Overwritten by every capture. |

Two roles recording at once want two folders: every stream but the network
journal is per role, and one folder holds one of each.
