// PlanetSurfaceGlsl.h
#pragma once

#include <string>
#include "PlanetSurface.h"

/**
 * @brief The body's shape spelled as GLSL, for the shader that draws it.
 *
 * How wide the body is, how often the map is laid across it, and how tall each
 * layer of terrain stands, written as declarations to put in front of the
 * snippet. The shader reads them from here instead of carrying its own copy, so
 * the figures are held once and cannot be edited apart.
 *
 * Declarations rather than uniforms because the shader folds them: a count sizes
 * an array and is unrolled over, and a frequency settles a lattice cell count
 * that has to land on a whole number.
 *
 * The one thing here that knows GLSL exists. PlanetSurface holds the numbers and
 * knows nothing of shaders; this reads it through the same accessors anything
 * else would, and only says how to spell what it answers.
 */
std::string planetSurfaceGlsl(const PlanetSurface& surface);
