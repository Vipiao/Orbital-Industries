// CubeSphereBounds.h
#pragma once

#include "graphics/cdlod/CdlodPatchBounds.h"

/**
 * @brief Where a cube-sphere body's patches land, for the CDLOD tree.
 *
 * The CPU twin of the shape half of triplanar_noise_surface.glsl, written apart
 * only because they run in different languages. Agreement between them means
 * containment, not equality: the snippet decides where a point goes, this only
 * promises it lands inside -- which is why relief enters as a ceiling rather
 * than as the height field itself.
 */
class CubeSphereBounds : public ICdlodPatchBounds {
public:
    // radius is the sphere the cube projects onto; reliefMetres is the furthest
    // the surface can stand off it, which the snippet decides and this must not
    // undercut.
    CubeSphereBounds(double radius, double reliefMetres);

    CdlodPatchBounds patchBounds(const CdlodPatchFrame& frame) const override;

private:
    double m_radius{1.0};
    double m_relief{0.0};
};
