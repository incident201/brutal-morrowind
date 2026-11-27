#include "animationstate.hpp"

#include <algorithm>
#include <cmath>

#include "esmreader.hpp"
#include "esmwriter.hpp"

namespace ESM
{
    bool AnimationState::empty() const
    {
        return mScriptedAnims.empty() && mBodyProjectiles.empty() && mEquippedShieldProjectiles.empty()
            && mShieldProjectiles.empty();
    }

    void AnimationState::load(ESMReader& esm)
    {
        mScriptedAnims.clear();

        while (esm.isNextSub("ANIS"))
        {
            ScriptedAnimation anim;

            anim.mGroup = esm.getHString();
            esm.getHNOT(anim.mTime, "TIME");
            esm.getHNOT(anim.mAbsolute, "ABST");
            esm.getHNT(anim.mLoopCount, "COUN");

            mScriptedAnims.push_back(std::move(anim));
        }

        const auto checkTransform = [](const auto& transform) {
            return std::ranges::all_of(transform, [](float value) { return std::isfinite(value); });
        };
        mBodyProjectiles.clear();
        while (esm.isNextSub("BPRJ"))
        {
            BodyProjectile projectile;
            projectile.mBone = esm.getHString();
            projectile.mItem = esm.getHNRefId("BPID");
            esm.getHNT("BPMT", projectile.mTransform);
            if (mBodyProjectiles.size() >= MaxStuckProjectiles || projectile.mBone.empty()
                || projectile.mItem.empty() || !checkTransform(projectile.mTransform))
                esm.fail("Invalid embedded body projectile");
            mBodyProjectiles.push_back(std::move(projectile));
        }
        mEquippedShieldProjectiles.clear();
        while (esm.peekNextSub("ESPI"))
        {
            EquippedShieldProjectile projectile;
            projectile.mItem = esm.getHNRefId("ESPI");
            esm.getHNT("ESPM", projectile.mTransform);
            if (mEquippedShieldProjectiles.size() >= MaxStuckProjectiles || projectile.mItem.empty()
                || !checkTransform(projectile.mTransform))
                esm.fail("Invalid equipped shield projectile");
            mEquippedShieldProjectiles.push_back(std::move(projectile));
        }
        mShieldProjectiles.clear();
        while (esm.peekNextSub("SPRF"))
        {
            ShieldProjectile projectile;
            projectile.mReference = esm.getFormId(true, "SPRF");
            esm.getHNT("SPTM", projectile.mTransform);
            if (mShieldProjectiles.size() >= MaxStuckProjectiles || !projectile.mReference.isSet()
                || !checkTransform(projectile.mTransform))
                esm.fail("Invalid embedded shield projectile");
            mShieldProjectiles.push_back(projectile);
        }
    }

    void AnimationState::save(ESMWriter& esm) const
    {
        for (ScriptedAnimations::const_iterator iter = mScriptedAnims.begin(); iter != mScriptedAnims.end(); ++iter)
        {
            esm.writeHNString("ANIS", iter->mGroup);
            if (iter->mTime > 0)
                esm.writeHNT("TIME", iter->mTime);
            if (iter->mAbsolute)
                esm.writeHNT("ABST", iter->mAbsolute);
            esm.writeHNT("COUN", iter->mLoopCount);
        }
        for (const auto& projectile : mBodyProjectiles)
        {
            esm.writeHNString("BPRJ", projectile.mBone);
            esm.writeHNRefId("BPID", projectile.mItem);
            esm.writeHNT("BPMT", projectile.mTransform);
        }
        for (const auto& projectile : mEquippedShieldProjectiles)
        {
            esm.writeHNRefId("ESPI", projectile.mItem);
            esm.writeHNT("ESPM", projectile.mTransform);
        }
        for (const auto& projectile : mShieldProjectiles)
        {
            esm.writeFormId(projectile.mReference, true, "SPRF");
            esm.writeHNT("SPTM", projectile.mTransform);
        }
    }
}
