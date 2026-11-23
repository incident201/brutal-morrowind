#ifndef OPENMW_ESM_ANIMATIONSTATE_H
#define OPENMW_ESM_ANIMATIONSTATE_H

#include <array>
#include <cstdint>
#include <string>
#include <vector>

#include <components/esm/formid.hpp>
#include <components/esm/refid.hpp>

namespace ESM
{
    class ESMReader;
    class ESMWriter;

    // format 0, saved games only
    struct AnimationState
    {
        struct ScriptedAnimation
        {
            ScriptedAnimation()
                : mTime(0.f)
                , mAbsolute(false)
                , mLoopCount(0)
            {
            }

            std::string mGroup;
            float mTime;
            bool mAbsolute;
            uint64_t mLoopCount;
        };

        typedef std::vector<ScriptedAnimation> ScriptedAnimations;
        ScriptedAnimations mScriptedAnims;

        static constexpr std::size_t MaxStuckProjectiles = 12;

        // Body missiles live in the actor's inventory and are drawn on this bone.
        struct BodyProjectile
        {
            std::string mBone;
            RefId mItem;
            std::array<float, 16> mTransform;
        };
        std::vector<BodyProjectile> mBodyProjectiles;

        // A released shield carries real world item references in its local frame.
        struct ShieldProjectile
        {
            FormId mReference;
            std::array<float, 16> mTransform;
        };
        std::vector<ShieldProjectile> mShieldProjectiles;

        bool empty() const;

        void load(ESMReader& esm);
        void save(ESMWriter& esm) const;
    };
}

#endif
