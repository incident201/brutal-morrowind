#ifndef OPENMW_MWRENDER_PROJECTILEMODEL_H
#define OPENMW_MWRENDER_PROJECTILEMODEL_H

#include <algorithm>
#include <array>
#include <string_view>

#include <components/vfs/pathutil.hpp>

namespace MWRender
{
    inline bool projectileModelFacesBackward(VFS::Path::NormalizedView model)
    {
        // These stock knives and darts have their point at the low end of Y.
        constexpr std::array<std::string_view, 6> backwardModels{
            "meshes/w/w_knife_iron.nif",
            "meshes/w/w_knife_glass.nif",
            "meshes/w/w_dart_steel.nif",
            "meshes/w/w_dart_silver.nif",
            "meshes/w/w_dart_ebony.nif",
            "meshes/w/w_dart_daedric.nif",
        };
        return std::ranges::find(backwardModels, model.value()) != backwardModels.end();
    }
}

#endif
