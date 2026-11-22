#include "surfacematerial.hpp"

#include <span>
#include <string>
#include <utility>

#include <components/misc/strings/lower.hpp>

// Surface keywords derived from sunspotdark/morrowind-modern-physics
// (https://github.com/sunspotdark/morrowind-modern-physics, commit eb50a40, apps/openmw/mwworld/materials.cpp), GPLv3.

namespace MWWorld
{
    namespace
    {
        using Keywords = std::pair<SurfaceMaterial, std::span<const std::string_view>>;

        // Hard materials first: they win when a name has both.
        constexpr std::string_view metal[] = { "metal", "iron", "steel", "bronze", "silver", "gold", "dwrv", "dwem" };
        constexpr std::string_view ceramic[] = { "glass", "crystal" };
        constexpr std::string_view stone[] = { "stone", "rock", "brick", "cobble", "marble", "granite", "slate",
            "tile", "plaster", "stucco", "chitin", "bone", "shell", "pave", "ice", "lava", "velothi", "daed", "ruin" };
        constexpr std::string_view wood[] = { "wood", "wd_", "bark", "tree", "log", "plank", "board", "timber",
            "branch", "root", "crate", "barrel", "basket", "wicker", "mushroom", "shroom", "fung" };
        constexpr std::string_view cloth[] = { "straw", "thatch", "hay", "fabric", "cloth", "banner", "rug",
            "carpet", "tapestry", "rope", "leather", "hide", "fur" };
        constexpr std::string_view earth[] = { "flora", "moss", "dirt", "mud", "sand", "grass", "soil", "ground",
            "kelp" };
        constexpr Keywords keywords[] = { { SurfaceMaterial::Metal, metal }, { SurfaceMaterial::Ceramic, ceramic },
            { SurfaceMaterial::Stone, stone }, { SurfaceMaterial::Wood, wood }, { SurfaceMaterial::Cloth, cloth },
            { SurfaceMaterial::Earth, earth } };

        SurfaceMaterial byName(std::string_view name)
        {
            if (name.empty())
                return SurfaceMaterial::Unknown;
            const std::string lower = Misc::StringUtils::lowerCase(name);
            // Only the file name: folders such as "textures/" or "meshes/x/" say nothing.
            const std::size_t slash = lower.find_last_of("/\\");
            const std::string_view file = slash == std::string::npos ? std::string_view(lower)
                                                                     : std::string_view(lower).substr(slash + 1);
            for (const auto& [material, words] : keywords)
                for (const std::string_view word : words)
                    if (file.find(word) != std::string_view::npos)
                        return material;
            return SurfaceMaterial::Unknown;
        }
    }

    std::string_view getSurfaceMaterialName(SurfaceMaterial material)
    {
        switch (material)
        {
            case SurfaceMaterial::Wood:
                return "wood";
            case SurfaceMaterial::Earth:
                return "earth";
            case SurfaceMaterial::Cloth:
                return "cloth";
            case SurfaceMaterial::Flesh:
                return "flesh";
            case SurfaceMaterial::Stone:
                return "stone";
            case SurfaceMaterial::Metal:
                return "metal";
            case SurfaceMaterial::Ceramic:
                return "ceramic";
            case SurfaceMaterial::Unknown:
                break;
        }
        return "unknown";
    }

    bool takesMissiles(SurfaceMaterial material)
    {
        return material == SurfaceMaterial::Wood || material == SurfaceMaterial::Earth
            || material == SurfaceMaterial::Cloth || material == SurfaceMaterial::Flesh;
    }

    SurfaceMaterial classifySurface(std::string_view texture, std::string_view model, SurfaceMaterial fallback)
    {
        if (const SurfaceMaterial named = byName(texture); named != SurfaceMaterial::Unknown)
            return named;
        if (const SurfaceMaterial named = byName(model); named != SurfaceMaterial::Unknown)
            return named;
        return fallback;
    }
}
