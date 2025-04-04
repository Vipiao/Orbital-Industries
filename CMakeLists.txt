cmake_minimum_required(VERSION 3.16)
project(OrbitalIndustries)

# Set C++20 standard
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Specify output directory for the executable
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin)

# Include directories for header files
include_directories(${CMAKE_SOURCE_DIR})
include_directories("${CMAKE_SOURCE_DIR}/include")
include_directories("C:/Users/user/OneDrive/cpp/libraries/glad/include")
include_directories("C:/Users/user/OneDrive/cpp/libraries/glm")

# Automatically find all source files using GLOB_RECURSE
file(GLOB_RECURSE SOURCES
    "${CMAKE_SOURCE_DIR}/main.cpp"
    "${CMAKE_SOURCE_DIR}/src/*.cpp"
    "C:/Users/user/OneDrive/cpp/libraries/glad/src/glad.c"
)

# Add the executable
add_executable(OrbitalIndustries ${SOURCES})

# Link with the static library located in your lib folder
target_link_libraries(OrbitalIndustries 
    "${CMAKE_SOURCE_DIR}/lib/libglfw3.a"
    opengl32
    gdi32
    user32
    shell32
)