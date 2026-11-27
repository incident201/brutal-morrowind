#ifndef OPENMW_MWWORLD_SURFACEMATERIAL_H
#define OPENMW_MWWORLD_SURFACEMATERIAL_H

#include <string_view>

namespace MWWorld
{
    /// What a missile struck, as far as sticking and sound are concerned.
    enum class SurfaceMaterial
    {
        Unknown,
        Wood,
        Earth,
        Cloth,
        Flesh,
        Stone,
        Metal,
        Ceramic,
    };

    std::string_view getSurfaceMaterialName(SurfaceMaterial material);

    /// Arrows and bolts stick in wood, earth, cloth and flesh, and glance off the rest.
    bool takesMissiles(SurfaceMaterial material);

    /// Classify by the struck texture, then the model, falling back when neither names a material.
    SurfaceMaterial classifySurface(std::string_view texture, std::string_view model, SurfaceMaterial fallback);
}

#endif
