# OrbitalIndustries

A game built on a separate graphics engine repo. The engine lives in its own
repository and is pulled in at build time via a local path, so you clone both
and tell this project where the engine is.

## Controls

| Key           | Action                                                     |
| ------------- | ---------------------------------------------------------- |
| WASD          | Move camera                                                |
| Mouse         | Look around                                                |
| Space / Shift | Move up / down                                             |
| M             | Toggle mouse lock                                          |
| F             | Apply force to grid                                        |
| R             | Configure block (select corners)                          |
| Q             | Remove block                                               |
| N             | Reload shaders (hot-reload, includes the ion-plume body)   |

## Prerequisites

- CMake 3.16+
- A C++20 compiler (GCC/Clang on Linux, MSVC or MSYS2/UCRT64 on Windows)
- Dependencies: assimp, glfw3, OpenGL
- The graphics engine repo, cloned separately

## Setup

1. Clone this repo and the graphics engine repo.

2. Create a `CMakeUserPresets.json` next to this README, pointing `ENGINE_DIR`
   at your local engine clone. This file is gitignored — it's personal to your
   machine. Example:

```json
   {
     "version": 3,
     "configurePresets": [
       {
         "name": "dev",
         "binaryDir": "${sourceDir}/build",
         "cacheVariables": {
           "ENGINE_DIR": "/path/to/graphics_engine"
         }
       }
     ]
   }
```

   Set `ENGINE_DIR` to wherever you cloned the engine on your machine. Set
   `UTILS_DIR` the same way if the shared utils repo is not at its default
   sibling location.

3. Configure and build:

```sh
   cmake --preset dev-release
   cmake --build --preset dev-release
```

   The binary lands in `bin/` and must be run from there, since it resolves its
   media paths relative to the working directory:

```sh
   cd bin && ./OrbitalIndustries
```

   It asks whether to start as server or client on stdin.

## Editor setup

Configuring is enough to get code navigation working: the build writes a
`compile_commands.json` into the preset's build directory covering all three
repos, game, engine and utils, so a language server can follow a symbol across
them.

Point [clangd](https://clangd.llvm.org/) at it. Because the engine and utils
repos sit outside this one, clangd cannot find the database by searching upward
from a file in them and has to be told where it is:

```jsonc
   "clangd.arguments": [
       "--compile-commands-dir=/path/to/Orbital-Industries/build/dev-release",
       "--background-index",
       "--header-insertion=never"
   ]
```

If the Microsoft C/C++ extension is installed as well, set
`"C_Cpp.intelliSenseEngine": "disabled"` so only one language server parses each
file. The extension is still worth keeping for its debugger, which is what
`.vscode/launch.json` and the natvis visualizers are written against.

Both of those settings hold wherever you open the engine and utils repos, so
they belong in your own user settings rather than in this repo.