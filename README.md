# OrbitalIndustries

A game built on a separate graphics engine repo. The engine lives in its own
repository and is pulled in at build time via a local path, so you clone both
and tell this project where the engine is.

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
           "ENGINE_DIR": "/home/markus/repos/02_graphics_engine/graphics_engine"
         }
       }
     ]
   }
```

   Set `ENGINE_DIR` to wherever you cloned the engine on your machine.

3. Configure and build: