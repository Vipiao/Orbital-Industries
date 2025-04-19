# Orbital-Industries# OpenGL and Assimp Setup Guide for Windows with MSYS2

This guide covers the setup process for developing OpenGL applications with Assimp for 3D model loading using MSYS2's UCRT64 environment on Windows.

## Table of Contents

- [Prerequisites](#prerequisites)
- [Setting up MSYS2](#setting-up-msys2)
- [Setting up OpenGL with GLFW and GLAD](#setting-up-opengl-with-glfw-and-glad)
- [Setting up Assimp](#setting-up-assimp)
## Prerequisites

- Windows 10 or 11
- Administrator access to install software
- Basic familiarity with command line interfaces

## Setting up MSYS2

MSYS2 provides a Unix-like development environment for Windows.

1. **Download and Install MSYS2**:
   - Visit [https://www.msys2.org/](https://www.msys2.org/) and download the installer
   - Run the installer and follow the setup instructions
   - When installation completes, launch the MSYS2 UCRT64 terminal

2. **Update the package database and core packages**:
   ```bash
   pacman -Syu
   ```
   - Close the terminal when prompted, then reopen it and run:
   ```bash
   pacman -Su
   ```

3. **Install development tools**:
   ```bash
   pacman -S mingw-w64-ucrt-x86_64-toolchain mingw-w64-ucrt-x86_64-cmake make
   ```

4. **Add MSYS2 bin directory to your system PATH**:
   - Navigate to System Properties → Advanced → Environment Variables
   - Add `C:\msys64\ucrt64\bin` to your PATH variable

## Setting up OpenGL with GLFW and GLAD

### Install GLFW using MSYS2:
   ```bash
   pacman -S mingw-w64-ucrt-x86_64-glfw
   ```

### Setting up GLAD

GLAD is a loader generator for OpenGL that provides the required headers and functions.

1. **Generate GLAD files**:
   - Visit [https://glad.dav1d.de/](https://glad.dav1d.de/)
   - Set Language to "C/C++"
   - Set Specification to "OpenGL"
   - Set Profile to "Core"
   - Set API gl to at least "Version 3.3" (or higher if needed)
   - Check "Generate a loader" option
   - Click "Generate" and download the ZIP file

2. **Extract GLAD files into your project**:
   - Create directories in your project: `include/glad`, `include/KHR`, and `src`
   - Extract `glad.h` to `include/glad/`
   - Extract `khrplatform.h` to `include/KHR/`
   - Extract `glad.c` to `src/`

## Setting up Assimp

Assimp (Open Asset Import Library) allows you to import various 3D model formats.

1. **Install Assimp in MSYS2 UCRT64**:
   ```bash
   pacman -S mingw-w64-ucrt-x86_64-assimp
   ```

2. **Copy Required DLLs to your project**:
   Run this script to copy the DLLs to your bin directory `copy_assimp_dlls.bat`.

## Building and Running Your Project

Use CMake. Easiest way is using vscode cmake extension.

## Resources
    Check this useful website. But it shows for visual studio not vs code.
https://learnopengl.com/