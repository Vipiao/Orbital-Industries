// TerrainMapDump.cpp
#include "TerrainMapDump.h"
#include "utils/PngWriter.h"
#include <algorithm>
#include <cassert>
#include <cmath>

namespace {

// The gradient's two axes on an image's three, the third left dark: a gradient
// has no third component, and a channel carrying a copy of one of the others
// would read as colour that means something.
constexpr int k_imageSamples{3};

// What a signed value at the scale's own end comes to, and what nothing comes to.
constexpr double k_fullScale{32767.0};
constexpr double k_midGrey{32768.0};

std::uint16_t signedSample(double value, double scale) {
    const double unit{scale > 0.0 ? value / scale : 0.0};
    const double sample{k_midGrey + std::clamp(unit, -1.0, 1.0) * k_fullScale};

    return static_cast<std::uint16_t>(std::lround(sample));
}

}   // namespace

bool TerrainMapDump::writeMaps(const std::filesystem::path& directory, int resolution,
                               const std::vector<std::uint16_t>& elevation,
                               const std::vector<float>& gradient) {
    const std::size_t texels{static_cast<std::size_t>(resolution) * resolution};
    assert(elevation.size() == texels && "The elevation map must hold one sample a texel");
    assert(gradient.size() == texels * 2 && "The gradient map must hold a pair a texel");

    bool written{PngWriter::writeGrey16(directory / "noise_map.png", resolution,
                                        resolution, elevation)};

    // The steepest texel sets the scale, so the image spends its whole range on
    // the map it was taken from however gentle or sharp that map is.
    double scale{0.0};
    for (float component : gradient) {
        scale = std::max(scale, std::abs(static_cast<double>(component)));
    }

    std::vector<std::uint16_t> image(texels * k_imageSamples, 0);
    for (std::size_t texel{0}; texel < texels; ++texel) {
        image[texel * k_imageSamples] = signedSample(gradient[texel * 2], scale);
        image[texel * k_imageSamples + 1] = signedSample(gradient[texel * 2 + 1], scale);
    }

    written = PngWriter::writeRgb16(directory / "gradient_map.png", resolution,
                                    resolution, image) &&
              written;

    return written;
}
