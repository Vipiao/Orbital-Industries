// PlanetSurfaceGlsl.h
#pragma once

#include <string>
#include "PlanetSurface.h"
#include "PropPatchGrid.h"

/**
 * @brief The body's shape spelled as GLSL, for the shaders that read it.
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
 *
 * shoreMetres is the height above the sphere the sand is laid along; a value
 * below all ground gives no sand.
 *
 * What lattice_surface.glsl and terrain_displacement.glsl read, so anything that
 * includes them is handed this.
 */
std::string planetSurfaceGlsl(const PlanetSurface& surface, double shoreMetres);

/**
 * @brief The far sea's own figures, for far_water.glsl alone.
 *
 * waterAbsorptionPerMetre and waterColor are the sea's below the shore;
 * bodyTint is what the renderer multiplies the surface's colour by.
 */
std::string farWaterGlsl(const glm::dvec3& waterAbsorptionPerMetre,
                         const glm::dvec3& waterColor, const glm::dvec3& bodyTint);

/**
 * @brief The prop patch grid's figures, for planet_props.glsl.
 *
 * How many prop patches span a face, and the cube's faces themselves, so the
 * snippet lays out the same grid PropPatchGrid does without a copy of either.
 */
std::string propPatchGridGlsl(const PropPatchGrid& grid);
