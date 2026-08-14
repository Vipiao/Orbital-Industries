// CubeSphereBounds.h
#pragma once

#include <memory>
#include "graphics/cdlod/CdlodPatchBounds.h"

class PlanetSurface;

/**
 * @brief Where a body's patches land, measured on the surface itself.
 *
 * Asks the surface where the patch's centre goes, and takes the reach from the
 * smooth sphere the terrain stands on. One elevation lookup per patch: enough to
 * put the ball in the right place, which is what stops it carrying the whole
 * body's relief down to the finest level.
 *
 * The reach ignores what the terrain does across the patch, so a patch can stand
 * a little outside its own ball. cdlod_patch.glsl holds a margin at both ends of
 * the morph band to cover it.
 */
class CubeSphereBounds : public ICdlodPatchBounds {
public:
    // Held rather than borrowed: the tree keeps these for its whole life, and a
    // surface that expired would leave every patch unmeasurable.
    explicit CubeSphereBounds(std::shared_ptr<const PlanetSurface> surface);

    CdlodPatchBounds patchBounds(const CdlodPatchFrame& frame) const override;

private:
    std::shared_ptr<const PlanetSurface> m_surface;
};
