cmake_minimum_required(VERSION 3.16)
project(OrbitalIndustries)

# Set C++20 standard
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Set default build type to Release if not specified
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

# Compiler-specific flags
if(MSVC)
    add_compile_options(/W4 /O2)
    set(CMAKE_CXX_FLAGS_DEBUG "/Zi /Od /MDd")
    set(CMAKE_CXX_FLAGS_RELEASE "/O2 /MD /DNDEBUG")
else()
    # GCC/Clang flags
    add_compile_options(-Wall -Wextra)
    add_compile_options(-Wall -Wextra -Wno-unused-parameter -Wno-unknown-pragmas)
    set(CMAKE_CXX_FLAGS_DEBUG "-g -O0")
    set(CMAKE_CXX_FLAGS_RELEASE "-O3 -DNDEBUG -march=native")
    set(CMAKE_CXX_FLAGS_RELWITHDEBINFO "-g -O2 -DNDEBUG")
endif()

message(STATUS "Build type: ${CMAKE_BUILD_TYPE}")

# Engine location — set in CMakeUserPresets.json (gitignored), Option 3.
set(ENGINE_DIR "" CACHE PATH "Path to the graphics engine repo")
# Shared utils library. If left empty, the engine adds it from its default
# sibling location; set it here (or in the presets) to control the path.
set(UTILS_DIR "" CACHE PATH "Path to the shared utils library repo")
if(UTILS_DIR AND NOT TARGET utils)
    add_subdirectory(${UTILS_DIR} ${CMAKE_BINARY_DIR}/utils-build)
endif()
add_subdirectory(${ENGINE_DIR} ${CMAKE_BINARY_DIR}/engine-build)

# Specify output directory for the executable
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin)

# Game's own includes. glad/glm/glfw and all libs come from the engine and
# utils targets.
include_directories(${CMAKE_SOURCE_DIR})
if(WIN32)
    include_directories("C:/msys64/ucrt64/include")
    link_directories("C:/msys64/ucrt64/lib")
endif()

# Manually list source files
file(GLOB_RECURSE SRC_FILES
    "${CMAKE_SOURCE_DIR}/main.cpp"
    "${CMAKE_SOURCE_DIR}/src/*.cpp"
)

# Add the executable
add_executable(OrbitalIndustries ${SRC_FILES})

# The game depends on the engine and directly on the shared utils library.
target_link_libraries(OrbitalIndustries PRIVATE
    GraphicsEngine::graphics
    Utils::utils
)
