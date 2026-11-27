/*
 * OpenMW - The completely unofficial reimplementation of Morrowind
 *
 * This file (character.cpp) is part of the OpenMW package.
 *
 * OpenMW is distributed as free software: you can redistribute it
 * and/or modify it under the terms of the GNU General Public License
 * version 3, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * version 3 along with this program. If not, see
 * https://www.gnu.org/licenses/ .
 */

#include "character.hpp"

#include "brutalfootsteps.hpp"

#include "../mwphysics/creaturerig.hpp"
#include "../mwmechanics/brutaloptions.hpp"
#include "../mwphysics/playerkinetics.hpp"

#include <array>
#include <unordered_set>
#include <utility>

#include <components/debug/debuglog.hpp>
#include <components/esm3/loadarmo.hpp>
#include <components/esm3/loadweap.hpp>
#include <components/esm/records.hpp>
#include <components/misc/mathutil.hpp>
#include <components/misc/resourcehelpers.hpp>
#include <components/misc/rng.hpp>
#include <components/misc/strings/algorithm.hpp>
#include <components/misc/strings/conversion.hpp>

#include <components/settings/values.hpp>

#include <components/sceneutil/positionattitudetransform.hpp>

#include "../mwrender/animation.hpp"
#include "../mwrender/brutalblood.hpp"
#include "../mwrender/camera.hpp"
#include "../mwrender/npcanimation.hpp"

#include "../mwbase/environment.hpp"
#include "../mwbase/luamanager.hpp"
#include "../mwbase/mechanicsmanager.hpp"
#include "../mwbase/soundmanager.hpp"
#include "../mwbase/windowmanager.hpp"
#include "../mwbase/world.hpp"

#include "../mwphysics/raycasting.hpp"

#include "../mwworld/cellstore.hpp"
#include "../mwworld/class.hpp"
#include "../mwworld/datetimemanager.hpp"
#include "../mwworld/esmstore.hpp"
#include "../mwworld/inventorystore.hpp"
#include "../mwworld/player.hpp"
#include "../mwworld/spellcaststate.hpp"

#include "actorutil.hpp"
#include "aicombataction.hpp"
#include "combat.hpp"
#include "creaturestats.hpp"
#include "meleeocclusion.hpp"
#include "movement.hpp"
#include "npcstats.hpp"
#include "security.hpp"
#include "spellcasting.hpp"
#include "weapontype.hpp"

namespace
{
    const std::string sGroundFinisher = "brutal_ground_finisher";
    const std::string sBackstabIn = "brutal_backstab_in";
    const std::string sBackstabIdle = "brutal_backstab_idle";
    const std::string sBackstabCancel = "brutal_backstab_cancel";
    const std::string sBackstabOut = "brutal_backstab_out";
    const std::string sBackstabVictim = "brutal_backstabbed";
    constexpr std::string_view sBowKick = "brutal_npc_bow_kick";
    const std::string sSwordWipe = "brutal_sword_wipe";

    constexpr int sFinisherGround = MWPhysics::CollisionType_World | MWPhysics::CollisionType_Door
        | MWPhysics::CollisionType_HeightMap;
    constexpr int sBackstabObstruction = sFinisherGround | MWPhysics::CollisionType_Dynamic;

    float backstabBlend(float fraction)
    {
        const float t = std::clamp(fraction, 0.f, 1.f);
        return t * t * (3.f - 2.f * t);
    }

    bool backstabPathClear(const MWBase::World& world, const osg::Vec3f& from, const osg::Vec3f& to,
        const osg::Vec3f& extent)
    {
        if ((from - to).length2() < .01f)
            return true;
        const auto* rays = world.getRayCasting();
        const float radius = std::min(extent.x(), extent.y()) * .85f;
        for (float height : { extent.z() * .5f, extent.z() * 1.55f })
        {
            const osg::Vec3f offset(0.f, 0.f, std::max(radius + 2.f, height));
            if (rays->castSphere(from + offset, to + offset, radius, sBackstabObstruction).mHit)
                return false;
        }
        return true;
    }

    std::optional<osg::Vec3f> backstabPlacement(const MWBase::World& world, const MWWorld::Ptr& player,
        const MWWorld::Ptr& victim, const osg::Vec3f& forward)
    {
        const osg::Vec3f from = victim.getRefData().getPosition().asVec3();
        const osg::Vec3f extent = world.getHalfExtents(victim);
        // The paired clip brings the bodies inside ordinary movement range.
        // Those larger collision hulls would leave the wrist beyond the neck.
        constexpr float separation = 42.f;
        osg::Vec3f to = player.getRefData().getPosition().asVec3() + forward * separation;
        to.z() = from.z();
        const auto* rays = world.getRayCasting();
        const osg::Vec3f side(forward.y(), -forward.x(), 0.f);
        for (float lateral : { 0.f, -.6f, .6f })
        {
            const osg::Vec3f foot = to + side * (extent.x() * lateral);
            const auto ground = rays->castRay(foot + osg::Vec3f(0.f, 0.f, 12.f),
                foot - osg::Vec3f(0.f, 0.f, 14.f), sFinisherGround);
            if (!ground.mHit || ground.mHitNormal.z() < .7f)
                return std::nullopt;
            if (lateral == 0.f)
                to.z() = ground.mHitPos.z() + 1.f;
        }
        return backstabPathClear(world, from, to, extent) ? std::optional(to) : std::nullopt;
    }

    osg::Vec3f backstabContact(const MWWorld::Ptr& victim, MWRender::NpcAnimation* player, int step,
        osg::Vec3f& outward)
    {
        const auto world = MWBase::Environment::get().getWorld();
        MWPhysics::RayCastingResult hit;
        if (player)
        {
            auto blade = player->getBackstabBlade();
            if (!blade)
                blade = player->getWeaponBladeForBlood();
            if (blade)
            {
                if (world->castRenderingRay(hit, blade->first, blade->second, true, false, {}, true)
                    && hit.mHitObject == victim)
                {
                    MWPhysics::RayCastingResult viewHit;
                    const bool covered = world->castRenderingRay(viewHit,
                        osg::Vec3f(world->getCamera()->getPosition()), blade->second, true, false, {}, true)
                        && viewHit.mHitObject == victim;
                    Log(Debug::Verbose) << "Brutal backstab blade contact " << step << " on "
                        << victim.getCellRef().getRefId() << ", tip covered=" << covered
                        << ", blade past surface=" << (blade->second - hit.mHitPos).length();
                    outward = hit.mHitNormal;
                    return hit.mHitPos;
                }
            }
        }
        const auto* animation = world->getAnimation(victim);
        const auto neck = animation ? animation->getBoneWorldMatrix("Bip01 Neck") : std::nullopt;
        const auto spine = animation ? animation->getBoneWorldMatrix("Bip01 Spine1") : std::nullopt;
        osg::Vec3f destination = MWMechanics::getMeleeTorsoPosition(victim);
        if (neck && spine)
            destination = step == 1 ? (spine->getTrans() + neck->getTrans()) * .5f : neck->getTrans();
        // The animation key is processed before the scene graph traverses the
        // new blade pose. Find the live body surface when the last sample misses.
        if (world->castRenderingRay(hit, destination + outward * 30.f, destination, true, false, {}, true)
            && hit.mHitObject == victim)
        {
            outward = hit.mHitNormal;
            return hit.mHitPos;
        }
        return destination;
    }

    std::string_view getBestAttack(const ESM::Weapon* weapon)
    {
        int slash = weapon->mData.mSlash[0] + weapon->mData.mSlash[1];
        int chop = weapon->mData.mChop[0] + weapon->mData.mChop[1];
        int thrust = weapon->mData.mThrust[0] + weapon->mData.mThrust[1];
        if (slash == chop && slash == thrust)
            return "slash";
        else if (thrust >= chop && thrust >= slash)
            return "thrust";
        else if (slash >= chop && slash >= thrust)
            return "slash";
        else
            return "chop";
    }

    // Converts a movement Run state to its equivalent Walk state, if there is one.
    MWMechanics::CharacterState runStateToWalkState(MWMechanics::CharacterState state)
    {
        using namespace MWMechanics;
        switch (state)
        {
            case CharState_RunForward:
                return CharState_WalkForward;
            case CharState_RunBack:
                return CharState_WalkBack;
            case CharState_RunLeft:
                return CharState_WalkLeft;
            case CharState_RunRight:
                return CharState_WalkRight;
            case CharState_SwimRunForward:
                return CharState_SwimWalkForward;
            case CharState_SwimRunBack:
                return CharState_SwimWalkBack;
            case CharState_SwimRunLeft:
                return CharState_SwimWalkLeft;
            case CharState_SwimRunRight:
                return CharState_SwimWalkRight;
            default:
                return state;
        }
    }

    // Converts a Hit state to its equivalent Death state.
    MWMechanics::CharacterState hitStateToDeathState(MWMechanics::CharacterState state)
    {
        using namespace MWMechanics;
        switch (state)
        {
            case CharState_SwimKnockDown:
                return CharState_SwimDeathKnockDown;
            case CharState_SwimKnockOut:
                return CharState_SwimDeathKnockOut;
            case CharState_KnockDown:
                return CharState_DeathKnockDown;
            case CharState_KnockOut:
                return CharState_DeathKnockOut;
            default:
                return CharState_None;
        }
    }

    // Converts a movement state to its equivalent base animation group as long as it is a movement state.
    std::string_view movementStateToAnimGroup(MWMechanics::CharacterState state)
    {
        using namespace MWMechanics;
        switch (state)
        {
            case CharState_WalkForward:
                return "walkforward";
            case CharState_WalkBack:
                return "walkback";
            case CharState_WalkLeft:
                return "walkleft";
            case CharState_WalkRight:
                return "walkright";

            case CharState_SwimWalkForward:
                return "swimwalkforward";
            case CharState_SwimWalkBack:
                return "swimwalkback";
            case CharState_SwimWalkLeft:
                return "swimwalkleft";
            case CharState_SwimWalkRight:
                return "swimwalkright";

            case CharState_RunForward:
                return "runforward";
            case CharState_RunBack:
                return "runback";
            case CharState_RunLeft:
                return "runleft";
            case CharState_RunRight:
                return "runright";

            case CharState_SwimRunForward:
                return "swimrunforward";
            case CharState_SwimRunBack:
                return "swimrunback";
            case CharState_SwimRunLeft:
                return "swimrunleft";
            case CharState_SwimRunRight:
                return "swimrunright";

            case CharState_SneakForward:
                return "sneakforward";
            case CharState_SneakBack:
                return "sneakback";
            case CharState_SneakLeft:
                return "sneakleft";
            case CharState_SneakRight:
                return "sneakright";

            case CharState_TurnLeft:
                return "turnleft";
            case CharState_TurnRight:
                return "turnright";
            case CharState_SwimTurnLeft:
                return "swimturnleft";
            case CharState_SwimTurnRight:
                return "swimturnright";
            default:
                return {};
        }
    }

    // Converts a death state to its equivalent animation group as long as it is a death state.
    std::string_view deathStateToAnimGroup(MWMechanics::CharacterState state)
    {
        using namespace MWMechanics;
        switch (state)
        {
            case CharState_SwimDeath:
                return "swimdeath";
            case CharState_SwimDeathKnockDown:
                return "swimdeathknockdown";
            case CharState_SwimDeathKnockOut:
                return "swimdeathknockout";
            case CharState_DeathKnockDown:
                return "deathknockdown";
            case CharState_DeathKnockOut:
                return "deathknockout";
            case CharState_Death1:
                return "death1";
            case CharState_Death2:
                return "death2";
            case CharState_Death3:
                return "death3";
            case CharState_Death4:
                return "death4";
            case CharState_Death5:
                return "death5";
            default:
                return {};
        }
    }

    // Converts a hit state to its equivalent animation group as long as it is a hit state.
    std::string hitStateToAnimGroup(MWMechanics::CharacterState state)
    {
        using namespace MWMechanics;
        switch (state)
        {
            case CharState_SwimHit:
                return "swimhit";
            case CharState_SwimKnockDown:
                return "swimknockdown";
            case CharState_SwimKnockOut:
                return "swimknockout";

            case CharState_Hit:
                return "hit";
            case CharState_KnockDown:
                return "knockdown";
            case CharState_KnockOut:
                return "knockout";

            case CharState_Block:
                return "shield";

            default:
                return {};
        }
    }

    // Converts an idle state to its equivalent animation group.
    std::string idleStateToAnimGroup(MWMechanics::CharacterState state)
    {
        using namespace MWMechanics;
        switch (state)
        {
            case CharState_IdleSwim:
                return "idleswim";
            case CharState_IdleSneak:
                return "idlesneak";
            case CharState_Idle:
            case CharState_SpecialIdle:
                return "idle";
            default:
                return {};
        }
    }

    MWRender::Animation::AnimPriority getIdlePriority(MWMechanics::CharacterState state)
    {
        using namespace MWMechanics;
        MWRender::Animation::AnimPriority priority(Priority_Default);
        switch (state)
        {
            case CharState_IdleSwim:
                return Priority_SwimIdle;
            case CharState_IdleSneak:
                priority[MWRender::BoneGroup_LowerBody] = Priority_SneakIdleLowerBody;
                [[fallthrough]];
            default:
                return priority;
        }
    }

    float getFallDamage(const MWWorld::Ptr& ptr, float fallHeight)
    {
        MWBase::World* world = MWBase::Environment::get().getWorld();
        const MWWorld::Store<ESM::GameSetting>& store = world->getStore().get<ESM::GameSetting>();

        const float fallDistanceMin = store.find("fFallDamageDistanceMin")->mValue.getFloat();

        if (fallHeight >= fallDistanceMin)
        {
            const float acrobaticsSkill = static_cast<float>(ptr.getClass().getSkill(ptr, ESM::Skill::Acrobatics));
            const float jumpSpellBonus = ptr.getClass()
                                             .getCreatureStats(ptr)
                                             .getMagicEffects()
                                             .getOrDefault(ESM::MagicEffect::Jump)
                                             .getMagnitude();
            const float fallAcroBase = store.find("fFallAcroBase")->mValue.getFloat();
            const float fallAcroMult = store.find("fFallAcroMult")->mValue.getFloat();
            const float fallDistanceBase = store.find("fFallDistanceBase")->mValue.getFloat();
            const float fallDistanceMult = store.find("fFallDistanceMult")->mValue.getFloat();

            float x = fallHeight - fallDistanceMin;
            x -= (1.5f * acrobaticsSkill) + jumpSpellBonus;
            x = std::max(0.0f, x);

            float a = fallAcroBase + fallAcroMult * (100 - acrobaticsSkill);
            x = fallDistanceBase + fallDistanceMult * x;
            x *= a;

            return x;
        }
        return 0.f;
    }

    bool isRealWeapon(int weaponType)
    {
        return weaponType != ESM::Weapon::HandToHand && weaponType != ESM::Weapon::Spell
            && weaponType != ESM::Weapon::None;
    }

}

namespace MWMechanics
{
    namespace
    {
        // Dark Messiah's sword combos, parry and kicks use one-handed melee weapons.
        bool isBrutalOneHanded(int weaponType)
        {
            return weaponType == ESM::Weapon::ShortBladeOneHand || weaponType == ESM::Weapon::LongBladeOneHand
                || weaponType == ESM::Weapon::BluntOneHand || weaponType == ESM::Weapon::AxeOneHand
                || (brutalStaffWeapon(weaponType) && brutalDmFirstPerson());
        }

        MWRender::Animation::AnimPriority brutalSlidePriority()
        {
            // The authored low body/camera wins over weapon and movement tracks.
            // Arms keep the slide pose at rest, but an ordinary weapon attack can
            // override them without outranking a death or knockdown interruption.
            MWRender::Animation::AnimPriority priority(Priority_Scripted);
            priority[MWRender::BoneGroup_LeftArm] = Priority_Hit;
            priority[MWRender::BoneGroup_RightArm] = Priority_Hit;
            return priority;
        }

        // A combo strike pushes in proportion to its keyvalue impactcoeff, anchored on the shield
        // bash (impactcoeff 2, config.lua bashPushSpeed 300) and capped at the kick's 460.
        // Dark Messiah sets mmplayer_rushattackpushfactor and mmplayer_pushfactor alike (400).
        float brutalComboPush(float impactCoeff)
        {
            return std::min(460.f, impactCoeff * 150.f);
        }

        double brutalSimulationTime()
        {
            return MWBase::Environment::get().getWorld()->getTimeManager()->getSimulationTime();
        }
    }

    float calculateRagdollFallDamage(const MWWorld::Ptr& ptr, float height)
    {
        return getFallDamage(ptr, height)*(1.f-.25f*ptr.getClass().getCreatureStats(ptr).getFatigueTerm());
    }

    std::string CharacterController::chooseRandomGroup(const std::string& prefix, int* num) const
    {
        auto& prng = MWBase::Environment::get().getWorld()->getPrng();

        int numAnims = 0;
        while (mAnimation->hasAnimation(prefix + std::to_string(numAnims + 1)))
            ++numAnims;

        int roll = Misc::Rng::rollDice(numAnims, prng) + 1; // [1, numAnims]
        if (num)
            *num = roll;
        return prefix + std::to_string(roll);
    }

    void CharacterController::clearStateAnimation(std::string& anim) const
    {
        if (anim.empty())
            return;
        if (mAnimation)
            mAnimation->disable(anim);
        anim.clear();
    }

    void CharacterController::resetCurrentJumpState()
    {
        clearStateAnimation(mCurrentJump);
        mJumpState = JumpState_None;
    }

    void CharacterController::resetCurrentMovementState()
    {
        clearStateAnimation(mCurrentMovement);
        mMovementState = CharState_None;
        mMovementAnimationHasMovement = false;
    }

    void CharacterController::resetCurrentIdleState()
    {
        clearStateAnimation(mCurrentIdle);
        mIdleState = CharState_None;
    }

    void CharacterController::resetCurrentHitState()
    {
        clearStateAnimation(mCurrentHit);
        mHitState = CharState_None;
    }

    void CharacterController::resetCurrentWeaponState()
    {
        cancelBrutalWeaponWipe();
        cancelGroundFinisher();
        cancelBackstab(false);
        clearStateAnimation(mCurrentWeapon);
        mUpperBodyState = UpperBodyState::None;
        mBrutalChargedHold = mBrutalGuardCharge = false;
        mBrutalFast = BrutalFast::None;
        mBrutalFastQueued = false;
        mBrutalFastIndex = 0;
        resetBrutalFastContinuation();
        mBrutalBowRepeatUntil = -1.;
        mBrutalBowRepeatQueued = mBrutalBowFastDraw = false;
        mReadyToHit = false;
        mBrutalOffhandType = -1;
        if (mPtr.getClass().isActor())
        {
            mPtr.getClass().getCreatureStats(mPtr).mBrutalSlashRight = false;
            mPtr.getClass().getCreatureStats(mPtr).mBrutalSlideAttack = false;
        }
    }

    void CharacterController::resetCurrentDeathState()
    {
        clearStateAnimation(mCurrentDeath);
        mDeathState = CharState_None;
    }

    // An exhausted NPC (fatigue below zero) slumps into a living ragdoll instead of
    // Morrowind's knockout clip, and stays down until its fatigue recovers
    // (PhysicsSystem holds the recovery). True while the ragdoll owns the body.
    bool CharacterController::collapseIntoRagdoll(bool swimming)
    {
        if (mPtr == getPlayer() || !mPtr.getClass().isNpc() || swimming || !Settings::game().mBrutalEnabled
            || !Settings::game().mBrutalRagdoll)
            return false;
        if (mAnimation->isRagdollPose())
            return true;
        auto& stats = mPtr.getClass().getCreatureStats(mPtr);
        if (stats.isDead() || !stats.mBrutalClinchOpponent.ptrOrEmpty().isEmpty())
            return false;
        // A slight forward slump: the legs give way under the actor's own weight.
        const float yaw = mPtr.getRefData().getPosition().rot[2];
        const osg::Vec3f slump(std::sin(yaw), std::cos(yaw), 0.f);
        return MWBase::Environment::get().getWorld()->applyNpcKick(mPtr, slump * 60.f, MWWorld::Ptr(), true);
    }

    void CharacterController::refreshHitRecoilAnims()
    {
        auto& charClass = mPtr.getClass();
        if (!charClass.isActor())
            return;
        const auto world = MWBase::Environment::get().getWorld();
        auto& stats = charClass.getCreatureStats(mPtr);
        bool knockout = stats.getFatigue().getCurrent() < 0 || stats.getFatigue().getBase() == 0;
        bool recovery = stats.getHitRecovery();
        bool knockdown = stats.getKnockedDown();
        bool block = stats.getBlock() && !knockout && !recovery && !knockdown;
        bool isSwimming = world->isSwimming(mPtr);

        stats.setBlock(false);

        if (mPtr == getPlayer() && mHitState == CharState_Block && block)
        {
            mHitState = CharState_None;
            resetCurrentIdleState();
        }

        if (!mPtr.getClass().isNpc() && mUpperBodyState > UpperBodyState::WeaponEquipped)
        {
            recovery = false;
            stats.setHitRecovery(false);
        }

        if (mHitState != CharState_None)
        {
            if (!mAnimation->isPlaying(mCurrentHit))
            {
                if (isKnockedOut() && mCurrentHit.empty() && knockout)
                    return;

                mHitState = CharState_None;
                mCurrentHit.clear();
                stats.setKnockedDown(false);
                stats.setHitRecovery(false);
                resetCurrentIdleState();
            }
            else if (isKnockedOut())
                mAnimation->setLoopingEnabled(mCurrentHit, knockout);
            return;
        }

        if (!knockout && !knockdown && !recovery && !block)
            return;

        if (knockout && collapseIntoRagdoll(isSwimming))
            return;

        MWRender::Animation::AnimPriority priority(Priority_Knockdown);
        std::string_view startKey = "start";
        std::string_view stopKey = "stop";
        if (knockout)
        {
            mHitState = isSwimming ? CharState_SwimKnockOut : CharState_KnockOut;
            stats.setKnockedDown(true);
        }
        else if (knockdown)
        {
            mHitState = isSwimming ? CharState_SwimKnockDown : CharState_KnockDown;
        }
        else if (recovery)
        {
            mHitState = isSwimming ? CharState_SwimHit : CharState_Hit;
            priority = Priority_Hit;
        }
        else if (block)
        {
            mHitState = CharState_Block;
            priority = Priority_Hit;
            priority[MWRender::BoneGroup_LeftArm] = Priority_Block;
            priority[MWRender::BoneGroup_LowerBody] = Priority_WeaponLowerBody;
            startKey = "block start";
            stopKey = "block stop";
        }

        mCurrentHit = hitStateToAnimGroup(mHitState);

        if (isRecovery())
        {
            mCurrentHit = chooseRandomGroup(mCurrentHit);
            if (mHitState == CharState_SwimHit && !mAnimation->hasAnimation(mCurrentHit))
                mCurrentHit = chooseRandomGroup(hitStateToAnimGroup(CharState_Hit));
        }

        // Cancel upper body animations
        if (isKnockedOut() || isKnockedDown())
        {
            if (!mCurrentWeapon.empty())
                mAnimation->disable(mCurrentWeapon);
            if (mUpperBodyState > UpperBodyState::WeaponEquipped)
            {
                mUpperBodyState = UpperBodyState::WeaponEquipped;
                if (mWeaponType > ESM::Weapon::None)
                    mAnimation->showWeapons(true);
            }
            else if (mUpperBodyState < UpperBodyState::WeaponEquipped)
            {
                mUpperBodyState = UpperBodyState::None;
            }
        }

        if (!mAnimation->hasAnimation(mCurrentHit))
        {
            mCurrentHit.clear();
            return;
        }

        // Imported Source small-hit layers explicitly exclude root/leg weights.
        // Preserve the underlying locomotion mask rather than replacing it with
        // an empty or neutral lower-body track. Unmarked vanilla clips keep All.
        playBlendedAnimation(mCurrentHit, priority,
            isUpperBodyRecovery() ? MWRender::BlendMask_UpperBody : MWRender::BlendMask_All,
            true, 1, startKey, stopKey, 0.0f, std::numeric_limits<uint32_t>::max());
    }

    void CharacterController::refreshJumpAnims(JumpingState jump, bool force)
    {
        if (!force && jump == mJumpState)
            return;

        if (jump == JumpState_None)
        {
            if (!mCurrentJump.empty())
                resetCurrentIdleState();
            resetCurrentJumpState();
            return;
        }

        std::string_view weapShortGroup = getWeaponShortGroup(mWeaponType);
        std::string jumpAnimName = "jump";
        jumpAnimName += weapShortGroup;
        MWRender::Animation::BlendMask jumpmask = MWRender::BlendMask_All;
        if (!weapShortGroup.empty() && !mAnimation->hasAnimation(jumpAnimName))
            jumpAnimName = fallbackShortWeaponGroup("jump", &jumpmask);

        if (!mAnimation->hasAnimation(jumpAnimName))
        {
            if (!mCurrentJump.empty())
                resetCurrentIdleState();
            resetCurrentJumpState();
            return;
        }

        bool startAtLoop = (jump == mJumpState);
        mJumpState = jump;
        clearStateAnimation(mCurrentJump);

        mCurrentJump = jumpAnimName;
        if (mJumpState == JumpState_InAir)
            playBlendedAnimation(jumpAnimName, Priority_Jump, jumpmask, false, 1.0f,
                startAtLoop ? "loop start" : "start", "stop", 0.f, std::numeric_limits<uint32_t>::max());
        else if (mJumpState == JumpState_Landing)
            playBlendedAnimation(jumpAnimName, Priority_Jump, jumpmask, true, 1.0f, "loop stop", "stop", 0.0f, 0);
    }

    bool CharacterController::onOpen() const
    {
        if (mPtr.getType() == ESM::Container::sRecordId && mAnimation)
        {
            if (!mAnimation->hasAnimation("containeropen"))
                return true;

            if (mAnimation->isPlaying("containeropen"))
                return false;

            if (mAnimation->isPlaying("containerclose"))
                return false;

            mAnimation->play(
                "containeropen", Priority_Scripted, MWRender::BlendMask_All, false, 1.0f, "start", "stop", 0.f, 0);
            if (mAnimation->isPlaying("containeropen"))
                return false;
        }

        return true;
    }

    void CharacterController::onClose() const
    {
        if (mPtr.getType() == ESM::Container::sRecordId)
        {
            if (!mAnimation || !mAnimation->hasAnimation("containerclose"))
                return;

            float complete, startPoint = 0.f;
            bool animPlaying = mAnimation->getInfo("containeropen", &complete);
            if (animPlaying)
                startPoint = 1.f - complete;

            mAnimation->play("containerclose", Priority_Scripted, MWRender::BlendMask_All, false, 1.0f, "start", "stop",
                startPoint, 0);
        }
    }

    int CharacterController::brutalAnimationWeaponType(int weaponType) const
    {
        // The staff viewmodel set replaces the one-handed group names, like the
        // shield set: every combo, kick and body group keeps its established name.
        if (brutalStaffWeapon(weaponType) && mPtr == getPlayer() && mAnimation && mAnimation->useBrutalStaffSet())
            return ESM::Weapon::LongBladeOneHand;
        return weaponType;
    }

    std::string_view CharacterController::getWeaponAnimation(int weaponType) const
    {
        weaponType = brutalAnimationWeaponType(weaponType);
        std::string_view weaponGroup = getWeaponType(weaponType)->mLongGroup;
        if (isRealWeapon(weaponType) && !mAnimation->hasAnimation(weaponGroup))
        {
            static const std::string_view oneHandFallback = getWeaponType(ESM::Weapon::LongBladeOneHand)->mLongGroup;
            static const std::string_view twoHandFallback = getWeaponType(ESM::Weapon::LongBladeTwoHand)->mLongGroup;

            const ESM::WeaponType* weapInfo = getWeaponType(weaponType);

            // For real two-handed melee weapons use 2h swords animations as fallback, otherwise use the 1h ones
            if (weapInfo->mFlags & ESM::WeaponType::TwoHanded && weapInfo->mWeaponClass == ESM::WeaponType::Melee)
                weaponGroup = twoHandFallback;
            else
                weaponGroup = oneHandFallback;
        }
        else if (weaponType == ESM::Weapon::HandToHand && !mPtr.getClass().isBipedal(mPtr))
            return "attack1";

        return weaponGroup;
    }

    std::string_view CharacterController::getWeaponShortGroup(int weaponType) const
    {
        if (weaponType == ESM::Weapon::HandToHand && !mPtr.getClass().isBipedal(mPtr))
            return {};
        return getWeaponType(brutalAnimationWeaponType(weaponType))->mShortGroup;
    }

    std::string CharacterController::fallbackShortWeaponGroup(
        const std::string& baseGroupName, MWRender::Animation::BlendMask* blendMask) const
    {
        if (!isRealWeapon(mWeaponType))
        {
            if (blendMask != nullptr)
                *blendMask = MWRender::BlendMask_LowerBody;

            return baseGroupName;
        }

        static const std::string_view oneHandFallback = getWeaponShortGroup(ESM::Weapon::LongBladeOneHand);
        static const std::string_view twoHandFallback = getWeaponShortGroup(ESM::Weapon::LongBladeTwoHand);

        std::string groupName = baseGroupName;
        const ESM::WeaponType* weapInfo = getWeaponType(mWeaponType);

        // For real two-handed melee weapons use 2h swords animations as fallback, otherwise use the 1h ones
        if (weapInfo->mFlags & ESM::WeaponType::TwoHanded && weapInfo->mWeaponClass == ESM::WeaponType::Melee)
            groupName += twoHandFallback;
        else
            groupName += oneHandFallback;

        // Special case for crossbows - we should apply 1h animations a fallback only for lower body
        if (mWeaponType == ESM::Weapon::MarksmanCrossbow && blendMask != nullptr)
            *blendMask = MWRender::BlendMask_LowerBody;

        if (!mAnimation->hasAnimation(groupName))
        {
            groupName = baseGroupName;
            if (blendMask != nullptr)
                *blendMask = MWRender::BlendMask_LowerBody;
        }

        return groupName;
    }

    void CharacterController::refreshMovementAnims(CharacterState movement, bool force)
    {
        if (mPtr.getClass().isNpc() && mPtr != getPlayer()
            && mAnimation->setNpcCombatWalking(mPtr.getClass().getCreatureStats(mPtr).getAiSequence().isInCombat()))
            force |= mCurrentMovement.starts_with("walk");
        if (movement == mMovementState && !force)
            return;

        std::string_view movementAnimGroup = movementStateToAnimGroup(movement);

        if (movementAnimGroup.empty())
        {
            if (!mCurrentMovement.empty())
                resetCurrentIdleState();
            resetCurrentMovementState();
            return;
        }
        std::string movementAnimName{ movementAnimGroup };

        mMovementState = movement;
        std::string::size_type swimpos = movementAnimName.find("swim");
        if (!mAnimation->hasAnimation(movementAnimName))
        {
            if (swimpos != std::string::npos)
            {
                movementAnimName.erase(swimpos, 4);
                swimpos = std::string::npos;
            }
        }

        MWRender::Animation::BlendMask movemask = MWRender::BlendMask_All;

        std::string_view weapShortGroup = getWeaponShortGroup(mWeaponType);

        // Non-biped creatures don't use spellcasting-specific movement animations.
        if (!isRealWeapon(mWeaponType) && !mPtr.getClass().isBipedal(mPtr))
            weapShortGroup = {};

        if (swimpos == std::string::npos && !weapShortGroup.empty())
        {
            std::string weapMovementAnimName;
            // Spellcasting stance turning is a special case
            if (mWeaponType == ESM::Weapon::Spell && isTurning())
            {
                weapMovementAnimName = weapShortGroup;
                weapMovementAnimName += movementAnimName;
            }
            else
            {
                weapMovementAnimName = movementAnimName;
                weapMovementAnimName += weapShortGroup;
            }

            if (!mAnimation->hasAnimation(weapMovementAnimName))
                weapMovementAnimName = fallbackShortWeaponGroup(movementAnimName, &movemask);

            movementAnimName = std::move(weapMovementAnimName);
        }

        // Dark Messiah's player sprint (@Megasprint_*) replaces the forward run.
        mBrutalSprintAnim = false;
        if (mBrutalSprinting && movement == CharState_RunForward)
        {
            std::string sprint = "brutal_sprint";
            sprint += weapShortGroup;
            // Short blades share the imported one-handed set. Ordinary gait
            // groups already use that fallback; the sprint must use it too.
            if (!mAnimation->hasAnimation(sprint) && mWeaponType == ESM::Weapon::ShortBladeOneHand)
                sprint = "brutal_sprint1h";
            if (!mAnimation->hasAnimation(sprint) && weapShortGroup.empty())
                sprint = "brutal_sprint";
            if (mAnimation->hasAnimation(sprint))
            {
                movementAnimName = std::move(sprint);
                movemask = MWRender::BlendMask_All;
                mBrutalSprintAnim = true;
            }
        }

        if (!mAnimation->hasAnimation(movementAnimName))
        {
            std::string::size_type runpos = movementAnimName.find("run");
            if (runpos != std::string::npos)
                movementAnimName.replace(runpos, 3, "walk");

            if (!mAnimation->hasAnimation(movementAnimName))
            {
                if (!mCurrentMovement.empty())
                    resetCurrentIdleState();
                resetCurrentMovementState();
                return;
            }
        }

        // If we're playing the same animation, start it from the point it ended
        float startpoint = 0.f;
        if (!mCurrentMovement.empty() && movementAnimName == mCurrentMovement)
            mAnimation->getInfo(mCurrentMovement, &startpoint);

        mMovementAnimationHasMovement = true;

        clearStateAnimation(mCurrentMovement);
        mCurrentMovement = std::move(movementAnimName);

        // For non-flying creatures, MW uses the Walk animation to calculate the animation velocity
        // even if we are running. This must be replicated, otherwise the observed speed would differ drastically.
        mAdjustMovementAnimSpeed = true;
        if (mPtr.getClass().getType() == ESM::Creature::sRecordId
            && !(mPtr.get<ESM::Creature>()->mBase->mFlags & ESM::Creature::Flies))
        {
            CharacterState walkState = runStateToWalkState(mMovementState);
            std::string_view anim = movementStateToAnimGroup(walkState);

            mMovementAnimSpeed = mAnimation->getVelocity(anim);
            if (mMovementAnimSpeed <= 1.0f)
            {
                // Another bug: when using a fallback animation (e.g. RunForward as fallback to SwimRunForward),
                // then the equivalent Walk animation will not use a fallback, and if that animation doesn't exist
                // we will play without any scaling.
                // Makes the speed attribute of most water creatures totally useless.
                // And again, this can not be fixed without patching game data.
                mAdjustMovementAnimSpeed = false;
                mMovementAnimSpeed = 1.f;
            }
        }
        else
        {
            mMovementAnimSpeed = mAnimation->getVelocity(mCurrentMovement);

            if (mMovementAnimSpeed <= 1.0f)
            {
                // The first person anims don't have any velocity to calculate a speed multiplier from.
                // We use the third person velocities instead.
                // FIXME: should be pulled from the actual animation, but it is not presently loaded.
                bool sneaking = mMovementState == CharState_SneakForward || mMovementState == CharState_SneakBack
                    || mMovementState == CharState_SneakLeft || mMovementState == CharState_SneakRight;
                mMovementAnimSpeed = (sneaking ? 33.5452f : (isRunning() ? 222.857f : 154.064f));
                mMovementAnimationHasMovement = false;
                const auto& cls = mPtr.getClass();
                if (mPtr == getPlayer() && Settings::game().mKineticPlayerMovement
                    && (cls.getCreatureStats(mPtr).mBrutalGait != MWPhysics::PlayerKinetics::Gait_Stance
                        || cls.getNormalizedEncumbrance(mPtr) > 1.f))
                {
                    // Imported Dark Messiah clips play at their authored rate at DM's speeds.
                    namespace K = MWPhysics::PlayerKinetics;
                    mMovementAnimSpeed = sneaking ? K::SneakSpeed
                        : mBrutalSprintAnim     ? K::SprintSpeed
                        : isRunning()           ? K::RunSpeed
                                                : K::WalkSpeed;
                }
            }
        }

        playBlendedAnimation(mCurrentMovement, Priority_Movement, movemask, false, 1.f, "start", "stop", startpoint,
            std::numeric_limits<uint32_t>::max(), true);
    }

    void CharacterController::refreshIdleAnims(CharacterState idle, bool force)
    {
        // FIXME: if one of the below states is close to their last animation frame (i.e. will be disabled in the coming
        // update), the idle animation should be displayed
        if (((mUpperBodyState != UpperBodyState::None && mUpperBodyState != UpperBodyState::WeaponEquipped)
                || mMovementState != CharState_None || !mCurrentHit.empty())
            && !mPtr.getClass().isBipedal(mPtr))
        {
            resetCurrentIdleState();
            return;
        }

        if (!force && idle == mIdleState && (mAnimation->isPlaying(mCurrentIdle) || !mAnimQueue.empty()))
            return;

        mIdleState = idle;

        std::string idleGroup = idleStateToAnimGroup(mIdleState);
        if (idleGroup.empty())
        {
            resetCurrentIdleState();
            return;
        }

        // Imported first-person melee sets distinguish bare and armed crouch.
        // The common bare clip must not replace stock bows, spells, fists or probes.
        const std::string_view shortGroup = getWeaponShortGroup(mWeaponType);
        const bool importedWeaponSneak = mIdleState == CharState_IdleSneak && mPtr == getPlayer()
            && brutalDmFirstPerson() && MWBase::Environment::get().getWorld()->isFirstPerson()
            && !shortGroup.empty();
        bool importedCrouch = false;
        if (importedWeaponSneak && isRealWeapon(mWeaponType) && mWeaponType != ESM::Weapon::PickProbe)
        {
            const std::string_view suffix = shortGroup == "1h" || shortGroup == "1s" ? "1h"
                : usesBrutalPairedWeapon()                                           ? shortGroup
                                                                                     : "";
            std::string armedSneak = idleGroup;
            armedSneak += suffix;
            if (!suffix.empty() && mAnimation->hasAnimation(armedSneak))
            {
                idleGroup = std::move(armedSneak);
                importedCrouch = true;
            }
        }

        MWRender::Animation::AnimPriority priority = getIdlePriority(mIdleState);
        size_t numLoops = std::numeric_limits<uint32_t>::max();

        // Only play "idleswim" or "idlesneak" if they exist. Otherwise, fallback to
        // "idle"+weapon or "idle".
        bool fallback = mIdleState != CharState_Idle
            && (!mAnimation->hasAnimation(idleGroup) || (importedWeaponSneak && !importedCrouch));
        if (fallback)
        {
            priority = getIdlePriority(CharState_Idle);
            idleGroup = idleStateToAnimGroup(CharState_Idle);
        }

        if (fallback || mIdleState == CharState_Idle || mIdleState == CharState_SpecialIdle)
        {
            std::string_view weapShortGroup = getWeaponShortGroup(mWeaponType);
            if (!weapShortGroup.empty())
            {
                std::string weapIdleGroup = idleGroup;
                weapIdleGroup += weapShortGroup;
                if (!mAnimation->hasAnimation(weapIdleGroup))
                    weapIdleGroup = fallbackShortWeaponGroup(idleGroup);
                idleGroup = std::move(weapIdleGroup);

                // play until the Loop Stop key 2 to 5 times, then play until the Stop key
                // this replicates original engine behavior for the "Idle1h" 1st-person animation
                auto& prng = MWBase::Environment::get().getWorld()->getPrng();
                numLoops = 1 + Misc::Rng::rollDice(4, prng);
            }
        }

        if (!mAnimation->hasAnimation(idleGroup))
        {
            resetCurrentIdleState();
            return;
        }

        float startPoint = 0.f;
        // There is no need to restart anim if the new and old anims are the same.
        // Just update the number of loops.
        if (mCurrentIdle == idleGroup)
            mAnimation->getInfo(mCurrentIdle, &startPoint);

        clearStateAnimation(mCurrentIdle);
        mCurrentIdle = std::move(idleGroup);
        playBlendedAnimation(mCurrentIdle, priority, MWRender::BlendMask_All, false, 1.0f, "start", "stop", startPoint,
            static_cast<uint32_t>(numLoops), true);
    }

    void CharacterController::refreshCurrentAnims(
        CharacterState idle, CharacterState movement, JumpingState jump, bool force)
    {
        // If the current animation is scripted, do not touch it
        if (isScriptedAnimPlaying())
            return;

        refreshHitRecoilAnims();
        refreshJumpAnims(jump, force);
        refreshMovementAnims(movement, force);

        // idle handled last as it can depend on the other states
        refreshIdleAnims(idle, force);
    }

    void CharacterController::playDeath(float startpoint, CharacterState death)
    {
        if (!mPtr.getClass().isNpc() && mAnimation && mAnimation->isRagdollPose())
            return; // A creature's falling body owns its pose.
        if (Settings::game().mBrutalRagdoll && mPtr.getClass().isNpc() && mPtr != getPlayer())
        {
            const auto& stats = mPtr.getClass().getCreatureStats(mPtr);
            // NPC death never runs a canned sequence. A loaded finished corpse
            // may sample its saved static endpoint, but must not disturb either
            // a physical ragdoll or the unsupported-skeleton frozen fallback.
            if (!stats.isDead() || !stats.isDeathAnimationFinished() || startpoint < 1.f
                || (mAnimation && mAnimation->isRagdollPose())
                || !MWBase::Environment::get().getWorld()->getNpcRagdollState(mPtr).empty())
                return;
        }
        mDeathState = death;
        mCurrentDeath = deathStateToAnimGroup(mDeathState);
        mPtr.getClass().getCreatureStats(mPtr).setDeathAnimation(
            static_cast<signed char>(mDeathState - CharState_Death1));

        // For dead actors, refreshCurrentAnims is no longer called, so we need to disable the movement state manually.
        // Note that these animations wouldn't actually be visible (due to the Death animation's priority being higher).
        // However, they could still trigger text keys, such as Hit events, or sounds.
        resetCurrentMovementState();
        resetCurrentWeaponState();
        resetCurrentHitState();
        resetCurrentIdleState();
        resetCurrentJumpState();

        playBlendedAnimation(
            mCurrentDeath, Priority_Death, MWRender::BlendMask_All, false, 1.0f, "start", "stop", startpoint, 0);
    }

    CharacterState CharacterController::chooseRandomDeathState() const
    {
        int selected = 0;
        chooseRandomGroup("death", &selected);
        return static_cast<CharacterState>(CharState_Death1 + (selected - 1));
    }

    void CharacterController::playRandomDeath(float startpoint)
    {
        if (mPtr == getPlayer())
        {
            const bool deathScene = brutalPlayerDeathScene();
            if (!deathScene)
                MWBase::Environment::get().getWorld()->getCamera()->setMode(MWRender::Camera::Mode::ThirdPerson, true);
            if (auto* animation = dynamic_cast<MWRender::NpcAnimation*>(mAnimation))
            {
                // Both death views need animations from the full rig.
                if (deathScene)
                    animation->preparePlayerDeathPose();
                else
                    animation->setViewMode(MWRender::NpcAnimation::VM_Normal);
            }
        }

        mDeathState = hitStateToDeathState(mHitState);
        if (mDeathState == CharState_None && MWBase::Environment::get().getWorld()->isSwimming(mPtr))
            mDeathState = CharState_SwimDeath;

        if (mDeathState == CharState_None
            || (mAnimation && !mAnimation->hasAnimation(deathStateToAnimGroup(mDeathState))))
            mDeathState = chooseRandomDeathState();

        // Do not interrupt scripted animation by death
        if (!mAnimation || isScriptedAnimPlaying())
            return;

        playDeath(startpoint, mDeathState);
    }

    std::string CharacterController::chooseRandomAttackAnimation() const
    {
        std::string result;
        bool isSwimming = MWBase::Environment::get().getWorld()->isSwimming(mPtr);

        if (isSwimming)
            result = chooseRandomGroup("swimattack");

        if (!isSwimming || !mAnimation->hasAnimation(result))
            result = chooseRandomGroup("attack");

        return result;
    }

    CharacterController::CharacterController(const MWWorld::Ptr& ptr, MWRender::Animation& anim)
        : mPtr(ptr)
        , mAnimation(&anim)
    {
        mAnimation->setTextKeyListener(this);

        const MWWorld::Class& cls = mPtr.getClass();
        if (cls.isActor())
        {
            /* Accumulate along X/Y only for now, until we can figure out how we should
             * handle knockout and death which moves the character down. */
            mAnimation->setAccumulation(osg::Vec3f(1.0f, 1.0f, 0.0f));

            if (cls.hasInventoryStore(mPtr))
            {
                getActiveWeapon(mPtr, &mWeaponType);
                if (mWeaponType != ESM::Weapon::None)
                {
                    mUpperBodyState = UpperBodyState::WeaponEquipped;
                    mCurrentWeapon = getWeaponAnimation(mWeaponType);
                }

                if (mWeaponType != ESM::Weapon::None && mWeaponType != ESM::Weapon::Spell
                    && mWeaponType != ESM::Weapon::HandToHand)
                {
                    mAnimation->showWeapons(true);
                    // Note: controllers for ranged weapon should use time for beginning of animation to play shooting
                    // properly, for other weapons they should use absolute time. Some mods rely on this behaviour (to
                    // rotate throwing projectiles, for example)
                    ESM::WeaponType::Class weaponClass = getWeaponType(mWeaponType)->mWeaponClass;
                    bool useRelativeDuration = weaponClass == ESM::WeaponType::Ranged;
                    mAnimation->setWeaponGroup(mCurrentWeapon, useRelativeDuration);
                }

                mAnimation->showCarriedLeft(updateCarriedLeftVisible(mWeaponType));
            }

            if (!cls.getCreatureStats(mPtr).isDead())
            {
                mIdleState = CharState_Idle;
                if (cls.getCreatureStats(mPtr).getFallHeight() > 0)
                    mJumpState = JumpState_InAir;
            }
            else
            {
                const MWMechanics::CreatureStats& cStats = mPtr.getClass().getCreatureStats(mPtr);
                if (cStats.isDeathAnimationFinished())
                {
                    // Set the death state, but don't play it yet
                    // We will play it in the first frame, but only if no script set the skipAnim flag
                    signed char deathanim = cStats.getDeathAnimation();
                    if (deathanim == -1)
                        mDeathState = chooseRandomDeathState();
                    else
                        mDeathState = static_cast<CharacterState>(CharState_Death1 + deathanim);

                    mFloatToSurface = cStats.getHealth().getBase() != 0;
                }
                // else: nothing to do, will detect death in the next frame and start playing death animation
            }
        }
        else
        {
            /* Don't accumulate with non-actors. */
            mAnimation->setAccumulation(osg::Vec3f(0.f, 0.f, 0.f));

            mIdleState = CharState_Idle;
        }

        // Do not update animation status for dead actors
        if (mDeathState == CharState_None && (!cls.isActor() || !cls.getCreatureStats(mPtr).isDead()))
            refreshCurrentAnims(mIdleState, mMovementState, mJumpState, true);

        mAnimation->runAnimation(0.f);

        unpersistAnimationState();
    }

    CharacterController::~CharacterController()
    {
        detachAnimation();
    }

    void CharacterController::detachAnimation()
    {
        cancelBackstab(false);
        if (mAnimation)
        {
            persistAnimationState();
            mAnimation->setTextKeyListener(nullptr);
            mAnimation = nullptr;
        }
    }

    void CharacterController::handleTextKey(
        std::string_view groupname, SceneUtil::TextKeyMap::ConstIterator key, const SceneUtil::TextKeyMap& map)
    {
        if (!mAnimation)
            return;
        std::string_view evt = key->second;

        MWBase::Environment::get().getLuaManager()->animationTextKey(mPtr, key->second);

        if (evt.substr(0, 7) == "sound: ")
        {
            MWBase::SoundManager* sndMgr = MWBase::Environment::get().getSoundManager();
            sndMgr->playSound3D(mPtr, ESM::RefId::stringRefId(evt.substr(7)), 1.0f, 1.0f);
            return;
        }

        auto& charClass = mPtr.getClass();
        if (evt.substr(0, 10) == "soundgen: ")
        {
            std::string_view soundgen = evt.substr(10);

            // The event can optionally contain volume and pitch modifiers
            float volume = 1.0f;
            float pitch = 1.0f;

            if (soundgen.find(' ') != std::string::npos)
            {
                std::vector<std::string_view> tokens;
                Misc::StringUtils::split(soundgen, tokens);
                soundgen = tokens[0];

                if (tokens.size() >= 2)
                {
                    volume = Misc::StringUtils::toNumeric<float>(tokens[1], volume);
                }

                if (tokens.size() >= 3)
                {
                    pitch = Misc::StringUtils::toNumeric<float>(tokens[2], pitch);
                }
            }

            if (soundgen == "left" || soundgen == "right")
            {
                if (mPtr != getPlayer())
                    if (auto* blood = MWRender::BrutalBlood::get())
                        blood->queueStep(mPtr, soundgen == "left");
                // The player's Dark Messiah steps follow the leg cycle, not the arms' clip.
                if (mPtr == getPlayer() && BrutalFootsteps::available())
                    return;
                if (BrutalFootsteps::play(mPtr, soundgen == "left", false, BrutalFootsteps::Gait::Walk))
                    return;
            }

            const ESM::RefId sound = charClass.getSoundIdFromSndGen(mPtr, soundgen);
            if (!sound.empty())
            {
                MWBase::SoundManager* sndMgr = MWBase::Environment::get().getSoundManager();
                if (soundgen == "left" || soundgen == "right")
                {
                    sndMgr->playSound3D(
                        mPtr, sound, volume, pitch, MWSound::Type::Foot, MWSound::PlayMode::NoPlayerLocal);
                }
                else
                {
                    sndMgr->playSound3D(mPtr, sound, volume, pitch);
                }
            }
            return;
        }

        if (evt.substr(0, groupname.size()) != groupname || evt.substr(groupname.size(), 2) != ": ")
        {
            // Not ours, skip it
            return;
        }

        std::string_view action = evt.substr(groupname.size() + 2);
        if (groupname == sBowKick)
        {
            if (action == "hit")
                hitBrutalBowKick();
            return;
        }
        if (groupname == sGroundFinisher)
        {
            if (mGroundFinisher && action == "swing")
                playSwishSound();
            else if (action == "hit")
                hitGroundFinisher();
            return;
        }
        if (groupname == sBackstabOut)
        {
            if (mBackstab.mPhase == BrutalBackstabPhase::Executing)
            {
                if (action == "stab1")
                    onBackstabStab(1);
                else if (action == "stab2")
                    onBackstabStab(2);
                else if (action == "release")
                    onBackstabRelease();
            }
            return;
        }
        if (mPtr == getPlayer() && (groupname == "brutal_rush" || groupname == "brutal_fallattack"))
        {
            // first_basic.qct HitPow relative to the charged big slash (1.5), which matches a
            // full-strength Morrowind swing: rush event0 0.5 and event1 1.5; falling attack 2,
            // 1.5 with a shield.
            if (action == "swing" || action == "swing2")
                playSwishSound();
            else if (action == "hit" && groupname == "brutal_rush")
                brutalComboHit(.5f / 1.5f, 0.f, false); // event0 "push 0": an ordinary hit reaction
            else if (action == "hit2")
            {
                // The second dagger delivers the rush's second stroke when there is one.
                auto& offhand = mPtr.getClass().getCreatureStats(mPtr).mBrutalOffhandStrike;
                offhand = hasBrutalOffhandDagger();
                brutalComboHit(1.f, brutalComboPush(1.5f), false);
                offhand = false;
            }
            else if (action == "hit")
            {
                const auto& inv = mPtr.getClass().getInventoryStore(mPtr);
                const auto shield = inv.getSlot(MWWorld::InventoryStore::Slot_CarriedLeft);
                const bool hasShield = shield != inv.end() && shield->getType() == ESM::Armor::sRecordId
                    && updateCarriedLeftVisible(mWeaponType);
                brutalComboHit(hasShield ? 1.f : 2.f / 1.5f, brutalComboPush(hasShield ? 15.f : 2.f), true);
            }
            return;
        }
        if (action == "equip attach")
        {
            if (groupname == "shield")
                mAnimation->showCarriedLeft(true);
            else if (mUpperBodyState == UpperBodyState::Equipping)
                mAnimation->showWeapons(true);
        }
        else if (action == "unequip detach")
        {
            if (groupname == "shield")
                mAnimation->showCarriedLeft(false);
            else if (mUpperBodyState == UpperBodyState::Unequipping)
                mAnimation->showWeapons(false);
        }
        else if (mPtr == getPlayer() && action.starts_with("fast"))
        {
            if (mBrutalFast == BrutalFast::Attack && action.ends_with(" swing"))
                playSwishSound();
            else if (mBrutalFast == BrutalFast::Attack
                && (action.ends_with(" hit") || action.ends_with(" hit left")))
                brutalFastHit(action);
        }
        else if (action == "chop hit" || action == "slash hit" || action == "slashright hit"
            || action == "thrust hit" || action == "neutral hit" || action == "hit")
        {
            int attackType = -1;
            if (action == "hit")
            {
                if (groupname == "attack1" || groupname == "swimattack1")
                    attackType = ESM::Weapon::AT_Chop;
                else if (groupname == "attack2" || groupname == "swimattack2")
                    attackType = ESM::Weapon::AT_Slash;
                else if (groupname == "attack3" || groupname == "swimattack3")
                    attackType = ESM::Weapon::AT_Thrust;
            }
            else if (action == "chop hit")
                attackType = ESM::Weapon::AT_Chop;
            else if (action == "slash hit" || action == "slashright hit")
                attackType = ESM::Weapon::AT_Slash;
            else if (action == "thrust hit")
                attackType = ESM::Weapon::AT_Thrust;
            else if (action == "neutral hit")
                attackType = mAttackType == "slash" ? ESM::Weapon::AT_Slash
                    : mAttackType == "thrust"       ? ESM::Weapon::AT_Thrust
                                                    : ESM::Weapon::AT_Chop;
            // We want to avoid hit keys that come out of nowhere (e.g. in the follow animation)
            // and processing multiple hit keys for a single attack
            if (mReadyToHit)
            {
                ++mBrutalAcceptedStrikes;
                applyMeleeImpact(attackType);
                mBrutalWallBodyHit = !mAttackVictim.isEmpty();
                mReadyToHit = false;
                mBrutalOffhandType = hasBrutalOffhandDagger() || brutalStaffSecondStrike() ? attackType : -1;
                mBrutalOffhandStrength = mAttackStrength;
            }
        }
        else if (mBrutalOffhandType != -1 && (action == "chop swing2" || action == "slash swing2" || action == "thrust swing2" || action == "neutral swing2"))
            playSwishSound();
        else if (action == "chop hit2" || action == "slash hit2" || action == "thrust hit2" || action == "neutral hit2")
        {
            // The left dagger's own strike in the same authored swing: a complete native
            // melee hit with its own target, damage, wear, enchantment and effects.
            if (mBrutalOffhandType != -1 && (hasBrutalOffhandDagger() || brutalStaffSecondStrike()))
            {
                auto& stats = charClass.getCreatureStats(mPtr);
                const MWWorld::Ptr victim = mAttackVictim;
                const osg::Vec3f hitPos = mAttackHitPos;
                const bool success = mAttackSuccess;
                const int part = mAttackContactPart;
                const float strength = mAttackStrength;
                stats.mBrutalOffhandStrike = hasBrutalOffhandDagger();
                mAttackStrength = mBrutalOffhandStrength;
                ++mBrutalAcceptedStrikes;
                applyMeleeImpact(mBrutalOffhandType);
                stats.mBrutalOffhandStrike = false;
                mAttackVictim = victim;
                mAttackHitPos = hitPos;
                mAttackSuccess = success;
                mAttackContactPart = part;
                mAttackStrength = strength;
            }
            mBrutalOffhandType = -1;
        }
        else if (isRandomAttackAnimation(groupname) && action == "start")
        {
            std::multimap<float, std::string>::const_iterator hitKey = key;

            // Not all animations have a hit key defined. If there is none, the hit happens with the start key.
            bool hasHitKey = false;
            while (hitKey != map.end())
            {
                if (hitKey->second.starts_with(groupname))
                {
                    std::string_view suffix = std::string_view(hitKey->second).substr(groupname.size());
                    if (suffix == ": hit")
                    {
                        hasHitKey = true;
                        break;
                    }
                    if (suffix == ": stop")
                        break;
                }
                ++hitKey;
            }
            if (!hasHitKey)
            {
                // State update doesn't expect the start key to be the hit key,
                // so we have to do this early.
                prepareHit();
                resolveMeleeHit();

                if (groupname == "attack1" || groupname == "swimattack1")
                    charClass.hit(
                        mPtr, mAttackStrength, ESM::Weapon::AT_Chop, mAttackVictim, mAttackHitPos, mAttackSuccess,mAttackContactPart);
                else if (groupname == "attack2" || groupname == "swimattack2")
                    charClass.hit(
                        mPtr, mAttackStrength, ESM::Weapon::AT_Slash, mAttackVictim, mAttackHitPos, mAttackSuccess,mAttackContactPart);
                else if (groupname == "attack3" || groupname == "swimattack3")
                    charClass.hit(
                        mPtr, mAttackStrength, ESM::Weapon::AT_Thrust, mAttackVictim, mAttackHitPos, mAttackSuccess,mAttackContactPart);
                mReadyToHit=false;
            }
        }
        else if (action == "shoot attach" || action == "shoot fast attach")
            mAnimation->attachArrow();
        else if (action == "shoot release" || action == "shoot fast release")
        {
            // See notes for melee release above
            if (mReadyToHit)
            {
                MWWorld::ConstPtr ammo;
                if (mPtr == getPlayer() && mWeaponType == ESM::Weapon::MarksmanBow && brutalDmFirstPerson())
                {
                    const auto& inv = charClass.getInventoryStore(mPtr);
                    const auto slot = inv.getSlot(MWWorld::InventoryStore::Slot_Ammunition);
                    if (slot != inv.end())
                        ammo = *slot;
                }
                const int count = ammo.isEmpty() ? 0 : ammo.getCellRef().getCount();
                mAnimation->releaseArrow(mAttackStrength);
                if (count > 0 && ammo.getCellRef().getCount() < count)
                    mBrutalBowRepeatUntil = brutalSimulationTime() + 1.15;
                mReadyToHit = false;
            }
        }
        else if (action == "shoot follow attach")
            mAnimation->attachArrow();
        // Make sure this key is actually for the RangeType we are casting. The flame atronach has
        // the same animation for all range types, so there are 3 "release" keys on the same time, one for each range
        // type.
        else if (groupname == "spellcast" && action == mAttackType + " release")
        {
            if (mCanCast)
                MWBase::Environment::get().getWorld()->castSpell(mPtr, mCastingScriptedSpell);
            mCastingScriptedSpell = false;
            mCanCast = false;
        }
        else if (groupname == "containeropen" && action == "loot")
            MWBase::Environment::get().getWindowManager()->pushGuiMode(MWGui::GM_Container, mPtr);
    }

    void CharacterController::updatePtr(const MWWorld::Ptr& ptr)
    {
        mPtr = ptr;
    }

    void CharacterController::updateIdleStormState(bool inwater) const
    {
        if (!mAnimation->hasAnimation("idlestorm"))
            return;

        bool animPlaying = mAnimation->isPlaying("idlestorm");
        if (mUpperBodyState != UpperBodyState::None || inwater)
        {
            if (animPlaying)
                mAnimation->disable("idlestorm");
            return;
        }

        const auto world = MWBase::Environment::get().getWorld();
        if (world->isInStorm())
        {
            osg::Vec3f stormDirection = world->getStormDirection();
            osg::Vec3f characterDirection = mPtr.getRefData().getBaseNode()->getAttitude() * osg::Vec3f(0, 1, 0);
            stormDirection.normalize();
            characterDirection.normalize();
            if (stormDirection * characterDirection < -0.5f)
            {
                if (!animPlaying)
                {
                    int mask = MWRender::BlendMask_Torso | MWRender::BlendMask_RightArm;
                    playBlendedAnimation("idlestorm", Priority_Storm, mask, true, 1.0f, "start", "stop", 0.0f,
                        std::numeric_limits<uint32_t>::max(), true);
                }
                else
                {
                    mAnimation->setLoopingEnabled("idlestorm", true);
                }
                return;
            }
        }

        if (animPlaying)
        {
            mAnimation->setLoopingEnabled("idlestorm", false);
        }
    }

    bool CharacterController::updateCarriedLeftVisible(const int weaptype) const
    {
        // Shields/torches shouldn't be visible during any operation involving two hands
        // There seems to be no text keys for this purpose, except maybe for "[un]equip start/stop",
        // but they are also present in weapon drawing animation.
        return mAnimation->updateCarriedLeftVisible(weaptype);
    }

    std::string CharacterController::getAttackAnimationType() const
    {
        if (mAttackType == "shoot" && mBrutalBowFastDraw
            && mAnimation->activeSourceHasTextKey(mCurrentWeapon + ": shoot start",
                mCurrentWeapon + ": shoot fast start"))
            return "shoot fast";
        // Source direction affects playback and impact direction; it never changes
        // the logical slash used by contact resolution, damage or adrenaline rules.
        if (mBrutalNeutralAttack && mPtr == getPlayer())
            return "neutral";
        if (mAttackType == "slash" && mPtr == getPlayer()
            && mPtr.getClass().getCreatureStats(mPtr).mBrutalSlashRight)
            return "slashright";
        return mAttackType;
    }

    float CharacterController::calculateWindUp() const
    {
        if (!mAnimation || mCurrentWeapon.empty() || mWeaponType == ESM::Weapon::PickProbe
            || isRandomAttackAnimation(mCurrentWeapon))
            return -1.f;

        const std::string attackAnimation = getAttackAnimationType();
        float minAttackTime = mAnimation->getTextKeyTime(mCurrentWeapon + ": " + attackAnimation + " min attack");
        float maxAttackTime = mAnimation->getTextKeyTime(mCurrentWeapon + ": " + attackAnimation + " max attack");
        if (minAttackTime == -1.f || minAttackTime >= maxAttackTime)
            return -1.f;

        return std::clamp(
            (mAnimation->getCurrentTime(mCurrentWeapon) - minAttackTime) / (maxAttackTime - minAttackTime), 0.f, 1.f);
    }

    void CharacterController::prepareHit()
    {
        if (mReadyToHit)
            return;

        ++mBrutalWallSwing;
        mBrutalWallBodyHit = false;
        mBrutalOffhandType = -1;
        auto& prng = MWBase::Environment::get().getWorld()->getPrng();
        mAttackStrength = calculateWindUp();
        if (mAttackStrength == -1.f)
            mAttackStrength = std::min(1.f, 0.1f + Misc::Rng::rollClosedProbability(prng));
        // A ready adrenaline slide finisher needs one click, rather than a
        // charged hold inside the short glide. Keep the authored windup/release
        // and actual hit key; the existing native token still authenticates and
        // spends the charge only through the ordinary melee receipt.
        const auto& stats = mPtr.getClass().getCreatureStats(mPtr);
        if (mPtr == getPlayer() && stats.mBrutalSlideAttack && !stats.mPowerStrike.token.empty()
            && stats.mPowerStrike.weapon.ptrOrEmpty() == mWeapon)
            mAttackStrength = 1.f;
        ESM::WeaponType::Class weapclass = getWeaponType(mWeaponType)->mWeaponClass;
        if (weapclass != ESM::WeaponType::Ranged)
        {
            if (!Settings::game().mBrutalMeleeGeometry && weapclass != ESM::WeaponType::Thrown)
            {
                mAttackContactPart = -1;
                mAttackSuccess = mPtr.getClass().evaluateHit(mPtr, mAttackVictim, mAttackHitPos, &mAttackContactPart);
                if (!mAttackSuccess) mAttackStrength = 0.f;
            }
            playSwishSound();
        }

        mReadyToHit = true;
    }

    void CharacterController::resolveMeleeHit(std::vector<MeleeFollowThrough>* followThrough)
    {
        // Classic target/accuracy selection belongs to the windup's original prepareHit.
        // Authored multi-hit/fast events have no pending vanilla preparation and resolve here.
        if (!Settings::game().mBrutalMeleeGeometry && mReadyToHit)
        {
            refreshMeleeGuardContact(mPtr, mAttackVictim, mAttackHitPos, mAttackContactPart);
            return;
        }
        // Resolve once at the actual hit key. prepareHit captures charge and
        // starts the swish, but cannot lock in an earlier aim/target/blood point.
        mAttackContactPart=-1;
        const std::string_view swing = mAttackType == "slash" || mAttackType == "chop" || mAttackType == "thrust"
            ? std::string_view(mAttackType) : std::string_view("any");
        mAttackSuccess = mPtr.getClass().evaluateHit(mPtr, mAttackVictim, mAttackHitPos, &mAttackContactPart, swing,
            followThrough);
        if (!Settings::game().mBrutalMeleeGeometry)
            refreshMeleeGuardContact(mPtr, mAttackVictim, mAttackHitPos, mAttackContactPart);
        if(!mAttackSuccess)mAttackStrength=0.f;
    }

    void CharacterController::applyMeleeImpact(int attackType)
    {
        bool sweep = false;
        if (mPtr == getPlayer() && !mWeapon.isEmpty() && brutalDmFirstPerson() && Settings::game().mBrutalMeleeGeometry)
        {
            const bool sidewaysSpear = mWeaponType == ESM::Weapon::SpearTwoWide && mAttackType == "slash"
                && (mBrutalFast == BrutalFast::Attack ? mBrutalFastClip.starts_with("faststrafe")
                                                     : brutalActiveKey("brutal spear grip"));
            sweep = mWeaponType == ESM::Weapon::BluntTwoWide || sidewaysSpear;
        }
        const MWWorld::Ptr weapon = mWeapon;
        const float strength = mAttackStrength;
        std::vector<MeleeFollowThrough> contacts;
        resolveMeleeHit(sweep ? &contacts : nullptr);
        const auto& cls = mPtr.getClass();
        cls.hit(mPtr, mAttackStrength, attackType, mAttackVictim, mAttackHitPos, mAttackSuccess, mAttackContactPart);
        for (const auto& contact : contacts)
            cls.hit(mPtr, strength, attackType, contact.mTarget, contact.mPosition, contact.mSuccessful,
                contact.mPart, false, &weapon);
    }

    bool CharacterController::brutalStaffSecondStrike() const
    {
        return mPtr == getPlayer() && mAnimation && mAnimation->useBrutalStaffSet() && brutalStaffWeapon(mWeaponType);
    }

    bool CharacterController::canPlayGroundFinisher() const
    {
        const auto& settings = Settings::game();
        if (mPtr != getPlayer() || !settings.mBrutalGroundFinisher || !brutalDmFirstPerson()
            || mWeapon.isEmpty() || getWeaponType(mWeaponType)->mWeaponClass != ESM::WeaponType::Melee
            || mWeapon.getType() != ESM::Weapon::sRecordId || !mAnimation)
            return false;
        const auto world = MWBase::Environment::get().getWorld();
        const auto& stats = mPtr.getClass().getCreatureStats(mPtr);
        if (!world->isFirstPerson() || !world->isOnGround(mPtr) || world->isSwimming(mPtr) || world->isFlying(mPtr)
            || stats.isDead() || stats.isParalyzed() || stats.getKnockedDown() || stats.getHitRecovery()
            || stats.mBrutalBlockRequested || stats.mBrutalSlidePhase || stats.mBrutalSlideAttack
            || stats.mBrutalClinchActive || isScriptedAnimPlaying() || stats.wasTeleported()
            || stats.getDrawState() != DrawState::Weapon)
            return false;
        const auto& inventory = mPtr.getClass().getInventoryStore(mPtr);
        const auto equipped = inventory.getSlot(MWWorld::InventoryStore::Slot_CarriedRight);
        return equipped != inventory.end() && *equipped == mWeapon;
    }

    std::optional<osg::Vec3f> CharacterController::groundFinisherContact(
        const MWWorld::Ptr& target, bool lockedTarget) const
    {
        const auto world = MWBase::Environment::get().getWorld();
        if (target.isEmpty() || !target.isInCell() || !target.getRefData().isEnabled()
            || !target.getClass().isNpc() || target == mPtr)
            return std::nullopt;
        const auto& stats = target.getClass().getCreatureStats(target);
        const auto& health = stats.getHealth();
        if (stats.isDead() || health.getCurrent() <= 0.f
            || health.getCurrent() * 100.f >= health.getModified() * Settings::game().mBrutalGroundFinisherHealth.get()
            || !stats.getAiSequence().isInCombat(mPtr))
            return std::nullopt;

        const bool groundedRagdoll = world->isNpcGroundedRagdoll(target);
        if (!groundedRagdoll && (!stats.getKnockedDown() || !world->isOnGround(target) || world->isSwimming(target)))
            return std::nullopt;

        const osg::Vec3f torso = getMeleeTorsoPosition(target);
        const auto targetBounds = world->getHalfExtents(target, true);
        // The knocked-down flag also covers the upright parts of a fall and get-up.
        if (!groundedRagdoll && torso.z() - target.getRefData().getPosition().pos[2] > targetBounds.z() * .8f)
            return std::nullopt;
        const auto& playerPose = mPtr.getRefData().getPosition();
        const auto playerBounds = world->getHalfExtents(mPtr, true);
        const osg::Vec3f origin = getMeleeAim(mPtr).first;
        const float reach = getMeleeWeaponReach(mPtr, mWeapon);
        const float radius = std::max(12.f, targetBounds.x());
        // Measure against the ragdoll's actual chest. Its standing actor bounds can be
        // several feet away when a kick moves the body.
        const osg::Vec2f horizontal(torso.x() - playerPose.pos[0], torso.y() - playerPose.pos[1]);
        const osg::Vec2f forward(std::sin(playerPose.rot[2]), std::cos(playerPose.rot[2]));
        const float distance = std::min(reach + playerBounds.y(), playerBounds.z() * 1.65f)
            + (lockedTarget ? playerBounds.y() : 0.f);
        // Select by proximity and horizontal facing. A locked victim can roll toward either side during the windup.
        if ((!lockedTarget && horizontal * forward < 0.f) || horizontal.length2() > distance * distance
            || torso.z() > origin.z() || torso.z() < playerPose.pos[2] - radius)
            return std::nullopt;

        constexpr int mask = MWPhysics::CollisionType_World | MWPhysics::CollisionType_Door
            | MWPhysics::CollisionType_HeightMap | MWPhysics::CollisionType_Dynamic | MWPhysics::CollisionType_Actor;
        if (world->getRayCasting()->castRay(origin, torso, { mPtr, target }, {}, mask).mHit)
            return std::nullopt;
        MWPhysics::RayCastingResult surface;
        if (world->castRenderingRay(surface, origin, torso, true, false, {}, true) && surface.mHitObject != target)
            return std::nullopt;
        if (world->castRenderingRay(surface, torso + osg::Vec3f(0, 0, 44.f), torso, true, false, {}, true))
        {
            if (surface.mHitObject != target)
                return std::nullopt;
            return surface.mHitPos;
        }
        return torso;
    }

    bool CharacterController::startGroundFinisher()
    {
        if (!canPlayGroundFinisher() || !mAnimation->hasAnimation(sGroundFinisher))
            return false;
        const auto& playerPose = mPtr.getRefData().getPosition();
        const osg::Vec3f origin = playerPose.asVec3();
        const float searchRadius = getMeleeWeaponReach(mPtr, mWeapon) * 2.f
            + MWBase::Environment::get().getWorld()->getHalfExtents(mPtr, true).z() * 2.f;
        std::vector<MWWorld::Ptr> actors;
        MWBase::Environment::get().getMechanicsManager()->getActorsInRange(origin, searchRadius, actors);
        MWWorld::Ptr victim;
        osg::Vec3f entry;
        float closest = std::numeric_limits<float>::max();
        for (const auto& actor : actors)
        {
            const auto contact = groundFinisherContact(actor, false);
            if (!contact)
                continue;
            const float distance = osg::Vec2f(contact->x() - origin.x(), contact->y() - origin.y()).length2();
            if (distance < closest)
            {
                closest = distance;
                victim = actor;
                entry = *contact;
            }
        }
        if (victim.isEmpty())
            return false;

        const float yaw = playerPose.rot[2];
        const float pitch = playerPose.rot[0];
        const float strikePitch = brutalStaffWeapon(mWeaponType) ? .57f : .20f;
        const float eyeHeight = getMeleeAim(mPtr).first.z() - origin.z();
        mGroundFinisher.emplace(GroundFinisher{ MWWorld::SafePtr(victim), MWWorld::SafePtr(mWeapon), entry,
            entry - getMeleeTorsoPosition(victim), std::nullopt, strikePitch, eyeHeight, 0.f, 0.f, yaw, pitch, yaw, pitch });
        mReadyToHit = false;
        mBrutalChargedHold = false;
        mBrutalFast = BrutalFast::None;
        resetBrutalFastContinuation();
        mUpperBodyState = UpperBodyState::WeaponEquipped;
        MWBase::Environment::get().getWorld()->breakInvisibility(mPtr);
        MWRender::Animation::ScopedDeferActiveGroupReset join(*mAnimation);
        mAnimation->disable(mCurrentWeapon);
        playBlendedAnimation(sGroundFinisher, Priority_Knockdown, MWRender::BlendMask_All,
            true, 1.f, "start", "stop", 0.f, 0);
        if (auto* npc = dynamic_cast<MWRender::NpcAnimation*>(mAnimation))
        {
            mGroundFinisher->mStrikeOffset = npc->groundFinisherStrikeOffset(sGroundFinisher, strikePitch);
            mGroundFinisher->mAuthoredDrop = npc->groundFinisherCameraDrop(
                sGroundFinisher, mAnimation->getTextKeyTime(sGroundFinisher + ": hit"));
            mGroundFinisher->mLowering = mGroundFinisher->mAuthoredDrop;
        }
        return true;
    }

    bool CharacterController::updateGroundFinisher()
    {
        if (!mGroundFinisher)
            return false;
        if (!canPlayGroundFinisher() || mGroundFinisher->mWeapon.ptrOrEmpty() != mWeapon
            || !mAnimation->isPlaying(sGroundFinisher))
        {
            cancelGroundFinisher();
            return false;
        }
        auto& finisher = *mGroundFinisher;
        const auto& victim = finisher.mVictim.ptrOrEmpty();
        const auto world = MWBase::Environment::get().getWorld();
        const auto& playerPose = mPtr.getRefData().getPosition();
        const float time = mAnimation->getCurrentTime(sGroundFinisher);
        const float hit = mAnimation->getTextKeyTime(sGroundFinisher + ": hit");
        const float returning = mAnimation->getTextKeyTime(sGroundFinisher + ": return start");
        const float end = mAnimation->getTextKeyTime(sGroundFinisher + ": stop");
        if (!victim.isEmpty() && victim.isInCell())
            finisher.mContact = getMeleeTorsoPosition(victim) + finisher.mTorsoOffset;
        const auto facing = finisher.mContact - playerPose.asVec3();
        const float reach = std::hypot(facing.x(), facing.y());
        const auto strike = finisher.mStrikeOffset;
        // Scale the authored crouch for the actual weapon length and the
        // victim's height. The same curve carries the camera and both arms.
        const float authoredDrop = finisher.mAuthoredDrop;
        if (!victim.isEmpty() && victim.isInCell())
        {
            const float blend = std::clamp(MWBase::Environment::get().getFrameDuration() / .12f, 0.f, 1.f);
            if (strike)
            {
                const float lowering = std::clamp(authoredDrop + strike->z() - facing.z() + 2.f, 8.f, 64.f);
                finisher.mLowering = std::lerp(finisher.mLowering, lowering, blend);
            }
            if (reach > 8.f)
            {
                const float gripAngle = strike ? std::clamp(std::atan2(strike->x(), strike->y()), -.37f, .37f) : 0.f;
                const float angle = std::atan2(facing.x(), facing.y()) - gripAngle - finisher.mInitialYaw;
                const float yawOffset = std::clamp(std::remainder(angle, 2.f * osg::PIf), -1.6f, 1.6f);
                finisher.mFocusYaw = std::lerp(finisher.mFocusYaw, finisher.mInitialYaw + yawOffset, blend);
            }
            // The head layer supplies the rest of the downward look during the drive.
            const float framing = brutalStaffWeapon(mWeaponType) ? .26f : .41f;
            const float pitch = std::clamp(
                std::atan2(finisher.mEyeHeight - finisher.mLowering - facing.z(), reach) - framing, .18f, 1.1f);
            finisher.mFocusPitch = std::lerp(finisher.mFocusPitch, pitch, blend);
        }
        const float weight = backstabBlend(time / std::max(.1f, hit * .72f))
            * (1.f - backstabBlend((time - returning) / std::max(.1f, end - returning)));
        const float pitch = std::lerp(finisher.mInitialPitch, finisher.mFocusPitch, weight);
        const float yaw = std::lerp(finisher.mInitialYaw, finisher.mFocusYaw, weight);
        if (std::abs(playerPose.rot[0] - pitch) > .001f
            || std::abs(std::remainder(playerPose.rot[2] - yaw, 2.f * osg::PIf)) > .001f)
            world->rotateObject(mPtr, osg::Vec3f(pitch, 0.f, yaw));
        if (auto* npc = dynamic_cast<MWRender::NpcAnimation*>(mAnimation))
        {
            const float bodyRatio
                = authoredDrop > 1.f ? npc->groundFinisherCameraDrop(sGroundFinisher, time) / authoredDrop : 0.f;
            npc->setGroundFinisherStance(std::lerp(finisher.mInitialPitch, finisher.mStrikePitch, weight),
                (finisher.mLowering - authoredDrop) * bodyRatio);
        }
        return true;
    }

    void CharacterController::cancelGroundFinisher()
    {
        if (mGroundFinisher && mAnimation)
        {
            if (auto* npc = dynamic_cast<MWRender::NpcAnimation*>(mAnimation))
                npc->setGroundFinisherStance(std::nullopt, 0.f);
            mAnimation->disable(sGroundFinisher);
        }
        mGroundFinisher.reset();
    }

    void CharacterController::hitGroundFinisher()
    {
        if (!mGroundFinisher)
            return;
        const auto selected = std::exchange(mGroundFinisher->mVictim, MWWorld::SafePtr());
        if (!canPlayGroundFinisher() || mGroundFinisher->mWeapon.ptrOrEmpty() != mWeapon)
            return;
        const auto& victim = selected.ptrOrEmpty();
        const auto contact = groundFinisherContact(victim, true);
        if (!contact)
            return;
        mGroundFinisher->mContact = *contact;
        const auto& skill = mWeapon.getClass().getEquipmentSkill(mWeapon);
        const int attack = skill == ESM::Skill::ShortBlade || skill == ESM::Skill::LongBlade || skill == ESM::Skill::Spear
            ? ESM::Weapon::AT_Thrust : ESM::Weapon::AT_Chop;
        mPtr.getClass().hit(mPtr, 1.f, attack, victim, *contact, true, -1, true);
    }

    bool CharacterController::canPlayBackstab(bool executing) const
    {
        const auto& settings = Settings::game();
        if (mPtr != getPlayer() || !settings.mBrutalBackstab || !brutalDmFirstPerson() || !mAnimation
            || mWeapon.isEmpty() || mWeapon.getType() != ESM::Weapon::sRecordId
            || mWeaponType != ESM::Weapon::ShortBladeOneHand || !hasBrutalOffhandDagger())
            return false;
        if (!mAnimation->hasAnimation(sBackstabIn) || !mAnimation->hasAnimation(sBackstabIdle)
            || !mAnimation->hasAnimation(sBackstabCancel) || !mAnimation->hasAnimation(sBackstabOut))
            return false;
        const auto world = MWBase::Environment::get().getWorld();
        const auto& stats = mPtr.getClass().getCreatureStats(mPtr);
        if (!world->isFirstPerson() || !world->isOnGround(mPtr) || world->isSwimming(mPtr) || world->isFlying(mPtr)
            || stats.isDead() || stats.isParalyzed() || stats.getKnockedDown() || stats.getHitRecovery()
            || stats.mBrutalBlockRequested || stats.mBrutalSlidePhase || stats.mBrutalSlideAttack
            || stats.mBrutalClinchActive || stats.mBrutalCarryingObject || mGroundFinisher
            || isScriptedAnimPlaying() || stats.wasTeleported()
            || stats.getDrawState() != DrawState::Weapon)
            return false;
        if (!executing)
        {
            if (mAnimation->isPlaying("brutal_kick") || mAnimation->isPlaying("brutal_kick1h")
                || mAnimation->isPlaying("brutal_kick1h_low") || isBrutalComboActive() || isBrutalShieldBlocking())
                return false;
            const bool stealthActive = isSneaking()
                || stats.getMagicEffects().getOrDefault(ESM::MagicEffect::Invisibility).getMagnitude() > 0.f
                || stats.getMagicEffects().getOrDefault(ESM::MagicEffect::Chameleon).getMagnitude() >= 50.f;
            if (!stealthActive)
                return false;
        }
        const auto& inventory = mPtr.getClass().getInventoryStore(mPtr);
        const auto equipped = inventory.getSlot(MWWorld::InventoryStore::Slot_CarriedRight);
        return equipped != inventory.end() && *equipped == mWeapon;
    }

    bool CharacterController::isValidBackstabTarget(const MWWorld::Ptr& target, bool maintain) const
    {
        if (target.isEmpty() || !target.isInCell() || !target.getRefData().isEnabled()
            || !target.getClass().isNpc() || target == mPtr)
            return false;
        const auto& stats = target.getClass().getCreatureStats(target);
        const auto& npcStats = target.getClass().getNpcStats(target);
        if (stats.isDead() || stats.getHealth().getCurrent() <= 0.f || npcStats.isWerewolf()
            || stats.getKnockedDown() || stats.mBrutalClinchActive || !stats.mBrutalClinchOpponent.ptrOrEmpty().isEmpty()
            || stats.mBrutalBackstabVictim)
            return false;
        const auto& seq = stats.getAiSequence();
        if (seq.isInCombat() || seq.isInPursuit())
            return false;

        const auto world = MWBase::Environment::get().getWorld();
        const auto* animation = world->getAnimation(target);
        if (!animation || !animation->hasAnimation(sBackstabVictim) || !world->isOnGround(target)
            || world->isSwimming(target) || world->isFlying(target) || world->isNpcGroundedRagdoll(target))
            return false;

        const auto& playerPosData = mPtr.getRefData().getPosition();
        const auto& targetPosData = target.getRefData().getPosition();
        const osg::Vec3f playerPos = playerPosData.asVec3();
        const osg::Vec3f targetPos = targetPosData.asVec3();
        const osg::Vec3f delta = targetPos - playerPos;
        if (std::abs(delta.z()) > 36.f)
            return false;

        const float maxDist = maintain ? 155.f : 115.f;
        osg::Vec2f flat(delta.x(), delta.y());
        const float dist = flat.length();
        if (dist < 1e-3f || dist > maxDist)
            return false;
        const osg::Vec2f dirToTarget = flat / dist;

        const osg::Vec2f playerFwd(std::sin(playerPosData.rot[2]), std::cos(playerPosData.rot[2]));
        const osg::Vec2f targetFwd(std::sin(targetPosData.rot[2]), std::cos(targetPosData.rot[2]));

        // The player aims at the target from a cone behind its back.
        const float aimCos = maintain ? 0.60f : 0.707f;
        if (targetFwd * dirToTarget < .707f || playerFwd * dirToTarget < aimCos)
            return false;

        const auto [origin, aim] = getMeleeAim(mPtr);
        const osg::Vec3f torso = getMeleeTorsoPosition(target);
        osg::Vec3f towardTorso = torso - origin;
        if (towardTorso.normalize() > .01f && aim * towardTorso < aimCos)
            return false;
        if (world->getRayCasting()->castRay(origin, torso, { mPtr, target }, {},
                sBackstabObstruction | MWPhysics::CollisionType_Actor).mHit)
            return false;

        return true;
    }

    MWWorld::Ptr CharacterController::findBackstabTarget(bool maintain) const
    {
        if (!canPlayBackstab(false))
            return MWWorld::Ptr();
        const osg::Vec3f origin = mPtr.getRefData().getPosition().asVec3();
        const float searchRadius = maintain ? 165.f : 125.f;
        std::vector<MWWorld::Ptr> actors;
        MWBase::Environment::get().getMechanicsManager()->getActorsInRange(origin, searchRadius, actors);
        MWWorld::Ptr best;
        float bestDist2 = std::numeric_limits<float>::max();
        for (const auto& actor : actors)
        {
            if (!isValidBackstabTarget(actor, maintain))
                continue;
            const float d2 = (actor.getRefData().getPosition().asVec3() - origin).length2();
            if (d2 < bestDist2)
            {
                bestDist2 = d2;
                best = actor;
            }
        }
        return best;
    }

    void CharacterController::enterBackstabStance(const MWWorld::Ptr& target)
    {
        mBackstab.mPhase = BrutalBackstabPhase::Entering;
        mBackstab.mVictim = MWWorld::SafePtr(target);
        mBackstab.mWeapon = MWWorld::SafePtr(mWeapon);
        auto& stats = mPtr.getClass().getCreatureStats(mPtr);
        stats.mBrutalBackstabPhase = static_cast<int>(mBackstab.mPhase);
        stats.mBrutalBackstabTarget = MWWorld::SafePtr(target);
        // Keep the last pose until the controller can replace it on the next update.
        playBlendedAnimation(sBackstabIn, Priority_Weapon, MWRender::BlendMask_All,
            false, 1.f, "start", "stop", 0.f, 0);
    }

    bool CharacterController::startBackstabExecution(const MWWorld::Ptr& victim)
    {
        if (victim.isEmpty() || !canPlayBackstab(true))
            return false;
        const auto world = MWBase::Environment::get().getWorld();
        auto* victimAnim = world->getAnimation(victim);
        const auto& playerPosition = mPtr.getRefData().getPosition();
        const auto& victimPosition = victim.getRefData().getPosition();
        osg::Vec3f forward = victimPosition.asVec3() - playerPosition.asVec3();
        forward.z() = 0.f;
        if (forward.normalize() < .01f || !victimAnim)
            return false;
        const auto goal = backstabPlacement(*world, mPtr, victim, forward);
        if (!goal)
        {
            cancelBackstab(true);
            return false;
        }
        auto& inv = mPtr.getClass().getInventoryStore(mPtr);
        const auto rightSlot = inv.getSlot(MWWorld::InventoryStore::Slot_CarriedRight);
        const auto leftSlot = inv.getSlot(MWWorld::InventoryStore::Slot_CarriedLeft);
        if (rightSlot == inv.end() || leftSlot == inv.end())
            return false;
        MWWorld::Ptr rightWeapon = *rightSlot;
        MWWorld::Ptr leftWeapon = *leftSlot;

        auto weaponPeak = [&](MWWorld::Ptr& weapon) {
            const auto& data = weapon.get<ESM::Weapon>()->mBase->mData;
            float dmg = static_cast<float>(std::max({ data.mChop[1], data.mSlash[1], data.mThrust[1] }));
            MWMechanics::adjustWeaponDamage(dmg, weapon, mPtr);
            MWMechanics::resistNormalWeapon(victim, mPtr, weapon, dmg);
            MWMechanics::applyWerewolfDamageMult(victim, weapon, dmg);
            return std::max(0.f, dmg);
        };

        const float base = std::max(14.f, weaponPeak(rightWeapon) + weaponPeak(leftWeapon));
        const float sneak = static_cast<float>(mPtr.getClass().getSkill(mPtr, ESM::Skill::Sneak));
        const float shortBlade = static_cast<float>(mPtr.getClass().getSkill(mPtr, ESM::Skill::ShortBlade));
        const float skillTerm = 0.55f * sneak + 0.45f * shortBlade;
        const float weaponMult = 4.0f + 0.08f * skillTerm;
        const float flatSkillDamage = 45.f + 1.5f * skillTerm + 0.045f * skillTerm * skillTerm;
        const float totalDamage = base * weaponMult + flatSkillDamage;

        auto& playerStats = mPtr.getClass().getCreatureStats(mPtr);
        auto& victimStats = victim.getClass().getCreatureStats(victim);
        const float victimHp = victimStats.getHealth().getCurrent();

        mBackstab.mPhase = BrutalBackstabPhase::Executing;
        mBackstab.mVictim = MWWorld::SafePtr(victim);
        mBackstab.mWeapon = MWWorld::SafePtr(rightWeapon);
        mBackstab.mOffhandWeapon = MWWorld::SafePtr(leftWeapon);
        mBackstab.mPlayerOrigin = playerPosition.asVec3();
        mBackstab.mVictimOrigin = victimPosition.asVec3();
        mBackstab.mVictimGoal = *goal;
        mBackstab.mPlayerYaw = playerPosition.rot[2];
        mBackstab.mPlayerPitch = playerPosition.rot[0];
        mBackstab.mVictimYaw = victimPosition.rot[2];
        mBackstab.mPairYaw = std::atan2(forward.x(), forward.y());
        if (const auto neck = victimAnim->getBoneWorldMatrix("Bip01 Neck"))
        {
            // The reference raises both actors. Keep the victim on the local floor
            // and preserve the relative height of the eye and blades at the neck.
            const osg::Vec3f towardNeck = osg::Vec3f(neck->getTrans()) - getMeleeAim(mPtr).first;
            mBackstab.mNeckPitch = std::clamp(
                -std::atan2(towardNeck.z(), std::hypot(towardNeck.x(), towardNeck.y())), -.55f, .35f);
            mBackstab.mEyeLift = std::clamp(towardNeck.z() - 2.f, -8.f, 32.f);
        }
        mBackstab.mTotalDamage = totalDamage;
        mBackstab.mDealtDamage = 0.f;
        mBackstab.mLethal = totalDamage >= victimHp;
        mBackstab.mReleased = false;

        playerStats.mBrutalBackstabPhase = static_cast<int>(BrutalBackstabPhase::Executing);
        playerStats.mBrutalBackstabTarget = MWWorld::SafePtr(victim);
        playerStats.mBrutalBackstabLastDamage = totalDamage;

        victimStats.mBrutalBackstabVictim = true;
        victimStats.mBrutalBackstabTarget = MWWorld::SafePtr(mPtr);
        victimStats.mBrutalSilentBackstab = true;
        world->queueMovement(victim, {});
        world->clearActorExternalVelocity(victim);

        mReadyToHit = false;
        mBrutalChargedHold = false;
        mBrutalGuardCharge = false;
        mBrutalFast = BrutalFast::None;
        resetBrutalFastContinuation();
        mUpperBodyState = UpperBodyState::WeaponEquipped;
        setAttackingOrSpell(false);
        world->breakInvisibility(mPtr);
        MWBase::Environment::get().getWindowManager()->setEnemy(victim);

        {
            MWRender::Animation::ScopedDeferActiveGroupReset defer(*mAnimation);
            mAnimation->disable(mCurrentWeapon);
            mAnimation->disable(sBackstabIn);
            mAnimation->disable(sBackstabIdle);
            mAnimation->disable(sBackstabCancel);
            playBlendedAnimation(sBackstabOut, Priority_Knockdown, MWRender::BlendMask_All,
                true, 1.f, "start", "stop", 0.f, 0);
        }
        victimAnim->play(sBackstabVictim, Priority_Knockdown, MWRender::BlendMask_All,
            false, 1.f, "start", "stop", 0.f, 0);
        victimAnim->enableHeadAnimation(false);
        Log(Debug::Info) << "Brutal backstab: executing on " << victim.getCellRef().getRefId()
                         << " (sneak=" << sneak << ", shortBlade=" << shortBlade
                         << ", dmg=" << totalDamage << ", hp=" << victimHp
                         << ", lethal=" << (mBackstab.mLethal ? "yes" : "no") << ")";
        return true;
    }

    void CharacterController::releaseBackstabVictim(const MWWorld::Ptr& victim)
    {
        if (victim.isEmpty() || !victim.getClass().isActor())
            return;
        auto& victimStats = victim.getClass().getCreatureStats(victim);
        victimStats.mBrutalBackstabVictim = false;
        if (!victimStats.isDead())
        {
            victimStats.mBrutalBackstabTarget = {};
            victimStats.mBrutalSilentBackstab = false;
        }
        if (auto* victimAnim = MWBase::Environment::get().getWorld()->getAnimation(victim))
        {
            if (!victimStats.isDead())
            {
                victimAnim->disable(sBackstabVictim);
                victimAnim->enableHeadAnimation(true);
            }
        }
    }

    void CharacterController::cancelBackstab(bool playCancelAnim)
    {
        if (mBackstab.mPhase == BrutalBackstabPhase::None)
            return;
        if (playCancelAnim && mAnimation && mBackstab.mPhase == BrutalBackstabPhase::Entering
            && mAnimation->isPlaying(sBackstabIn)
            && mAnimation->getCurrentTime(sBackstabIn) < mAnimation->getTextKeyTime(sBackstabIn + ": hide offhand"))
            playCancelAnim = false;
        const bool wasExecuting = mBackstab.mPhase == BrutalBackstabPhase::Executing;
        if (wasExecuting)
        {
            const MWWorld::Ptr victim = mBackstab.mVictim.ptrOrEmpty();
            if (!victim.isEmpty() && !mBackstab.mReleased)
                releaseBackstabVictim(victim);
        }
        if (mAnimation)
        {
            mAnimation->disable(sBackstabIn);
            mAnimation->disable(sBackstabIdle);
            mAnimation->disable(sBackstabOut);
            if (!playCancelAnim)
                mAnimation->disable(sBackstabCancel);
        }
        mBackstab = {};
        if (playCancelAnim && mAnimation && mAnimation->hasAnimation(sBackstabCancel)
            && mUpperBodyState == UpperBodyState::WeaponEquipped && canPlayBackstab(true))
        {
            mBackstab.mPhase = BrutalBackstabPhase::Canceling;
            playBlendedAnimation(sBackstabCancel, Priority_Weapon, MWRender::BlendMask_All,
                true, 1.f, "start", "stop", 0.f, 0);
        }
        if (mPtr.getClass().isActor())
        {
            auto& stats = mPtr.getClass().getCreatureStats(mPtr);
            stats.mBrutalBackstabPhase = static_cast<int>(mBackstab.mPhase);
            stats.mBrutalBackstabTarget = {};
            stats.mBrutalBackstabEyeLift = 0.f;
        }
    }

    void CharacterController::onBackstabStab(int step)
    {
        if (mBackstab.mPhase != BrutalBackstabPhase::Executing)
            return;
        const MWWorld::Ptr victim = mBackstab.mVictim.ptrOrEmpty();
        if (victim.isEmpty() || !victim.getClass().isNpc())
            return;
        auto& victimStats = victim.getClass().getCreatureStats(victim);
        if (victimStats.isDead())
            return;

        const auto world = MWBase::Environment::get().getWorld();
        const auto sndMgr = MWBase::Environment::get().getSoundManager();

        sndMgr->playSound3D(victim, ESM::RefId::stringRefId("Brutal Backstab Stab"), 1.f, 1.f);

        if (step == 2)
        {
            const ESM::NPC* npc = victim.get<ESM::NPC>()->mBase;
            const std::string race = Misc::StringUtils::lowerCase(npc->mRace.getRefIdString());
            const char* voiceId = "Brutal Backstab Voice Male";
            if (race == "orc")
                voiceId = "Brutal Backstab Voice Orc";
            else if (!npc->isMale())
                voiceId = "Brutal Backstab Voice Female";
            else if ((victim.getCellRef().getRefNum().mIndex & 1u) != 0)
                voiceId = "Brutal Backstab Voice Male B";
            sndMgr->playSound3D(victim, ESM::RefId::stringRefId(voiceId), 1.f, 1.f, MWSound::Type::Voice);
        }

        osg::Vec3f outward = mPtr.getRefData().getPosition().asVec3() - victim.getRefData().getPosition().asVec3();
        outward.z() = 0.f;
        if (outward.normalize() < 1e-3f)
            outward = osg::Vec3f(0.f, 0.f, 1.f);
        auto* playerNpc = dynamic_cast<MWRender::NpcAnimation*>(mAnimation);
        const osg::Vec3f contact = backstabContact(victim, playerNpc, step, outward);
        const bool redBlood = victim.get<ESM::NPC>()->mBase->mBloodType == 0;
        if (Settings::game().mBrutalEnabled && Settings::game().mBrutalBloodEffects && redBlood)
        {
            world->spawnBrutalImpact("blood", contact, outward, 1.8f, true);
        }
        if (Settings::game().mBrutalEnabled && Settings::game().mNpcBloodDecals && redBlood)
        {
            if (auto* victimNpc = dynamic_cast<MWRender::NpcAnimation*>(world->getAnimation(victim)))
                if (victimNpc->addBloodDecal(contact, outward, "stab", true, -1))
                    MWBase::Environment::get().getLuaManager()->storeNpcBloodDecals(victim, victimNpc->getBloodDecalData());
        }
        if (playerNpc && Settings::game().mBrutalEnabled && Settings::game().mBrutalWeaponBlood && redBlood)
            playerNpc->addWeaponBlood(contact, "stab", true);

        const float hp = victimStats.getHealth().getCurrent();
        float chunk = mBackstab.mTotalDamage * 0.35f;
        // Keep the victim alive until the release key (0.80, end of @die_backstabbed) so ragdoll handoff is synced.
        if (hp - chunk <= 1.f)
            chunk = std::max(0.f, hp - 1.f);
        if (chunk > 0.f)
        {
            const MWWorld::Ptr weapon = mBackstab.mWeapon.ptrOrEmpty();
            const ESM::RefId weaponId = weapon.isEmpty() ? ESM::RefId() : weapon.getCellRef().getRefId();
            mBackstab.mDealtDamage += chunk;
            victim.getClass().onHit(
                victim, { { "health", chunk } }, weaponId, mPtr, true, MWMechanics::DamageSourceType::Melee);
        }
    }

    void CharacterController::onBackstabRelease()
    {
        if (mBackstab.mPhase != BrutalBackstabPhase::Executing || mBackstab.mReleased)
            return;
        mBackstab.mReleased = true;

        const MWWorld::Ptr victim = mBackstab.mVictim.ptrOrEmpty();
        if (victim.isEmpty() || !victim.getClass().isNpc())
            return;

        auto& playerStats = mPtr.getClass().getCreatureStats(mPtr);
        auto& victimStats = victim.getClass().getCreatureStats(victim);
        MWWorld::Ptr rightWeapon = mBackstab.mWeapon.ptrOrEmpty();

        mPtr.getClass().skillUsageSucceeded(mPtr, ESM::Skill::Sneak, ESM::Skill::Sneak_AvoidNotice);
        mPtr.getClass().skillUsageSucceeded(mPtr, ESM::Skill::ShortBlade, ESM::Skill::Weapon_SuccessfulHit);

        const osg::Vec3f neckPos = getMeleeTorsoPosition(victim) + osg::Vec3f(0.f, 0.f, 14.f);
        if (!rightWeapon.isEmpty())
        {
            MWMechanics::reduceWeaponCondition(mBackstab.mTotalDamage * 0.25f, true, rightWeapon, mPtr);
            MWMechanics::applyOnStrikeEnchantment(mPtr, victim, rightWeapon, neckPos);
        }

        if (!victimStats.isDead())
        {
            float remaining = std::max(0.f, mBackstab.mTotalDamage - mBackstab.mDealtDamage);
            if (mBackstab.mLethal)
                remaining = std::max(remaining, victimStats.getHealth().getCurrent() + 1.f);
            const ESM::RefId weaponId = rightWeapon.isEmpty() ? ESM::RefId() : rightWeapon.getCellRef().getRefId();
            victim.getClass().onHit(
                victim, { { "health", remaining } }, weaponId, mPtr, true, MWMechanics::DamageSourceType::Melee);
        }

        if (victimStats.isDead())
        {
            ++playerStats.mBrutalBackstabKills;
            victimStats.mBrutalBackstabVictim = false;
            // killDeadActors consumes the flag when it skips the ordinary death cry.
        }
        else
        {
            // Rare case: massive-HP boss survived a low-skill backstab — exit cleanly into combat.
            releaseBackstabVictim(victim);
            victimStats.setHitRecovery(true);
            MWBase::Environment::get().getMechanicsManager()->actorAttacked(victim, mPtr);
        }
    }

    bool CharacterController::updateBackstabVictim(float duration)
    {
        if (!mPtr.getClass().isNpc() || mPtr == getPlayer())
            return false;
        auto& stats = mPtr.getClass().getCreatureStats(mPtr);
        if (!stats.mBrutalBackstabVictim)
            return false;
        if (stats.isDead())
        {
            stats.mBrutalBackstabVictim = false;
            return false;
        }
        const MWWorld::Ptr attacker = stats.mBrutalBackstabTarget.ptrOrEmpty();
        const bool attackerValid = !attacker.isEmpty() && attacker.isInCell() && attacker.getClass().isActor()
            && !attacker.getClass().getCreatureStats(attacker).isDead()
            && attacker.getClass().getCreatureStats(attacker).mBrutalBackstabPhase
                == static_cast<int>(BrutalBackstabPhase::Executing);
        if (!attackerValid)
        {
            releaseBackstabVictim(mPtr);
            return false;
        }

        auto& movement = mPtr.getClass().getMovementSettings(mPtr);
        std::fill(std::begin(movement.mPosition), std::end(movement.mPosition), 0.f);
        std::fill(std::begin(movement.mRotation), std::end(movement.mRotation), 0.f);
        movement.mSpeedFactor = 0.f;
        stats.setAttackingOrSpell(false);
        stats.mBrutalBlockRaised = stats.mBrutalParry = false;
        if (mAnimation)
        {
            mAnimation->setLegsYawRadians(0.f);
            mAnimation->setUpperBodyYawRadians(0.f);
            mAnimation->setBodyPitchRadians(0.f);
            mAnimation->enableHeadAnimation(false);
            mAnimation->runAnimation(std::max(duration, 0.f));
        }
        return true;
    }

    bool CharacterController::updateBackstab()
    {
        if (mPtr != getPlayer())
            return false;
        auto& playerStats = mPtr.getClass().getCreatureStats(mPtr);
        const auto syncStats = [&] {
            playerStats.mBrutalBackstabPhase = static_cast<int>(mBackstab.mPhase);
            playerStats.mBrutalBackstabTarget = mBackstab.mVictim;
        };

        if (mBackstab.mPhase == BrutalBackstabPhase::Executing)
        {
            const auto& inventory = mPtr.getClass().getInventoryStore(mPtr);
            const auto left = inventory.getSlot(MWWorld::InventoryStore::Slot_CarriedLeft);
            if (!canPlayBackstab(true) || mBackstab.mWeapon.ptrOrEmpty() != mWeapon
                || left == inventory.end() || mBackstab.mOffhandWeapon.ptrOrEmpty() != *left
                || !mAnimation->isPlaying(sBackstabOut))
            {
                cancelBackstab(false);
                mBrutalComboRelease = true;
                return false;
            }
            const auto world = MWBase::Environment::get().getWorld();
            const auto& position = mPtr.getRefData().getPosition();
            const float time = mAnimation->getCurrentTime(sBackstabOut);
            const float align = mAnimation->getTextKeyTime(sBackstabOut + ": align");
            const float release = mAnimation->getTextKeyTime(sBackstabOut + ": release");
            const float placed = backstabBlend((time - align) / .14f);
            const float raised = backstabBlend((time - .10f) / .27f);
            const float recovered = backstabBlend((time - release) / .34f);
            playerStats.mBrutalBackstabEyeLift = mBackstab.mEyeLift * raised * (1.f - recovered);
            const MWWorld::Ptr victim = mBackstab.mVictim.ptrOrEmpty();
            if (!mBackstab.mReleased)
            {
                if (victim.isEmpty() || !victim.isInCell() || !victim.getRefData().isEnabled()
                    || victim.getClass().getCreatureStats(victim).isDead()
                    || (position.asVec3() - mBackstab.mPlayerOrigin).length2() > 14.f * 14.f)
                {
                    cancelBackstab(false);
                    return false;
                }
                const auto& victimPosition = victim.getRefData().getPosition();
                const osg::Vec3f next = mBackstab.mVictimOrigin
                    + (mBackstab.mVictimGoal - mBackstab.mVictimOrigin) * placed;
                if (placed > 0.f && (next - victimPosition.asVec3()).length2() > .04f)
                {
                    if (!backstabPathClear(*world, victimPosition.asVec3(), next, world->getHalfExtents(victim)))
                    {
                        cancelBackstab(false);
                        return false;
                    }
                    world->moveObject(victim, next, true, false);
                }
                const float yaw = mBackstab.mVictimYaw
                    + std::remainder(mBackstab.mPairYaw - mBackstab.mVictimYaw, 2.f * osg::PIf) * placed;
                if (std::abs(std::remainder(yaw - victimPosition.rot[2], 2.f * osg::PIf)) > .001f)
                    world->rotateObject(victim, osg::Vec3f(0.f, 0.f, yaw));
            }
            const float yaw = mBackstab.mPlayerYaw
                + std::remainder(mBackstab.mPairYaw - mBackstab.mPlayerYaw, 2.f * osg::PIf) * raised;
            const float reach = backstabBlend(time / .14f);
            const float pitch = (mBackstab.mPlayerPitch * (1.f - reach) + mBackstab.mNeckPitch * reach)
                * (1.f - placed);
            if (std::abs(position.rot[0] - pitch) > .001f
                || std::abs(std::remainder(position.rot[2] - yaw, 2.f * osg::PIf)) > .001f)
                world->rotateObject(mPtr, osg::Vec3f(pitch, 0.f, yaw));
            syncStats();
            return true;
        }

        if (mBackstab.mPhase == BrutalBackstabPhase::Canceling)
        {
            if (!canPlayBackstab(true) || getAttackingOrSpell() || !mAnimation->isPlaying(sBackstabCancel))
            {
                mAnimation->disable(sBackstabCancel);
                mBackstab = {};
                syncStats();
                return false;
            }
            syncStats();
            return false;
        }

        if (mBackstab.mPhase == BrutalBackstabPhase::Entering || mBackstab.mPhase == BrutalBackstabPhase::Ready)
        {
            const MWWorld::Ptr locked = mBackstab.mVictim.ptrOrEmpty();
            MWWorld::Ptr target = (!locked.isEmpty() && canPlayBackstab(false) && isValidBackstabTarget(locked, true))
                ? locked
                : findBackstabTarget(true);
            if (target.isEmpty() || mUpperBodyState != UpperBodyState::WeaponEquipped)
            {
                cancelBackstab(true);
                return false;
            }
            mBackstab.mVictim = MWWorld::SafePtr(target);
            mBackstab.mAttackQueued |= getAttackingOrSpell();
            if (mBackstab.mPhase == BrutalBackstabPhase::Entering)
            {
                if (mAnimation->isPlaying(sBackstabIn))
                {
                    syncStats();
                    return mBackstab.mAttackQueued;
                }
                if (mBackstab.mAttackQueued)
                    return startBackstabExecution(target);
                mBackstab.mPhase = BrutalBackstabPhase::Ready;
                MWRender::Animation::ScopedDeferActiveGroupReset defer(*mAnimation);
                mAnimation->disable(sBackstabIn);
                playBlendedAnimation(sBackstabIdle, Priority_Weapon, MWRender::BlendMask_All,
                    false, 1.f, "start", "stop", 0.f, std::numeric_limits<uint32_t>::max(), true);
            }
            else if (mBackstab.mAttackQueued)
                return startBackstabExecution(target);
            syncStats();
            return false;
        }

        // Phase::None — auto-enter backstab ready stance when sneaking behind a valid humanoid NPC.
        if (mUpperBodyState == UpperBodyState::WeaponEquipped && !getAttackingOrSpell() && canPlayBackstab(false))
        {
            const MWWorld::Ptr target = findBackstabTarget(false);
            if (!target.isEmpty())
                enterBackstabStance(target);
        }
        return false;
    }

    bool CharacterController::hasBrutalOffhandDagger() const
    {
        if (!brutalDualWield() || mPtr != getPlayer() || !mAnimation || !brutalDmFirstPerson() || !mPtr.getClass().hasInventoryStore(mPtr))
            return false;
        const auto& inv = mPtr.getClass().getInventoryStore(mPtr);
        const auto left = inv.getSlot(MWWorld::InventoryStore::Slot_CarriedLeft);
        return left != inv.end() && left->getType() == ESM::Weapon::sRecordId
            && left->get<ESM::Weapon>()->mBase->mData.mType == ESM::Weapon::ShortBladeOneHand;
    }

    void CharacterController::normalizeBrutalDualDaggers()
    {
        // A second short blade lives in the left slot only beside a first one. Equipment
        // can change through the UI, scripts or combat, so keep the rule in one place.
        if (mPtr != getPlayer() || !mPtr.getClass().hasInventoryStore(mPtr))
            return;
        auto& inv = mPtr.getClass().getInventoryStore(mPtr);
        const auto isShortBlade = [](const MWWorld::ConstPtr& item) {
            return item.getType() == ESM::Weapon::sRecordId
                && item.get<ESM::Weapon>()->mBase->mData.mType == ESM::Weapon::ShortBladeOneHand;
        };
        const auto left = inv.getSlot(MWWorld::InventoryStore::Slot_CarriedLeft);
        if (left == inv.end() || left->getType() != ESM::Weapon::sRecordId)
            return;
        const auto right = inv.getSlot(MWWorld::InventoryStore::Slot_CarriedRight);
        if (!brutalDualWield() || !brutalDmFirstPerson() || !isShortBlade(*left) || (right != inv.end() && !isShortBlade(*right)))
            inv.unequipSlot(MWWorld::InventoryStore::Slot_CarriedLeft);
        else if (right == inv.end())
        {
            const auto moved = inv.unequipSlot(MWWorld::InventoryStore::Slot_CarriedLeft, false);
            inv.equip(MWWorld::InventoryStore::Slot_CarriedRight, moved);
        }
    }

    ESM::RefNum CharacterController::getWeaponWipeItem() const
    {
        if (mWeaponType != ESM::Weapon::LongBladeOneHand || mUpperBodyState != UpperBodyState::WeaponEquipped
            || !brutalDmFirstPerson() || !Settings::game().mBrutalWeaponBlood || mWeapon.isEmpty()
            || mInJump || isKnockedDown() || isKnockedOut() || isRecovery() || isScriptedAnimPlaying()
            || mBrutalBlock != BrutalBlock::None || mBrutalCombo != BrutalCombo::None
            || mGroundFinisher || mBackstab.mPhase != BrutalBackstabPhase::None
            || getAttackingOrSpell() || !mAnimation->hasAnimation(sSwordWipe)
            || mAnimation->isPlaying("brutal_kick1h") || mAnimation->isPlaying("brutal_kick1h_low"))
            return {};

        const auto world = MWBase::Environment::get().getWorld();
        const auto& stats = mPtr.getClass().getCreatureStats(mPtr);
        if (stats.getDrawState() != DrawState::Nothing || stats.isDead() || stats.mBrutalClinchActive
            || stats.mBrutalCarryingObject || stats.mBrutalSlidePhase != 0 || !world->isFirstPerson()
            || !world->isOnGround(mPtr) || world->isSwimming(mPtr) || world->isFlying(mPtr)
            || world->getPlayer().isInCombat())
            return {};

        const auto& inventory = mPtr.getClass().getInventoryStore(mPtr);
        const auto right = inventory.getSlot(MWWorld::InventoryStore::Slot_CarriedRight);
        if (right == inventory.end() || *right != mWeapon)
            return {};
        if (inventory.getSlot(MWWorld::InventoryStore::Slot_CarriedLeft) != inventory.end())
            return {};
        return mWeapon.getCellRef().getRefNum();
    }

    bool CharacterController::updateBrutalWeaponWipe(bool requested)
    {
        if (!requested && !mBrutalWipeWeapon.isSet())
            return false;
        const auto item = getWeaponWipeItem();
        if (mBrutalWipeWeapon.isSet())
        {
            if (item == mBrutalWipeWeapon && mAnimation->isPlaying(sSwordWipe))
                return true;
            cancelBrutalWeaponWipe();
            return false;
        }

        auto* blood = MWRender::BrutalBlood::get();
        auto* animation = dynamic_cast<MWRender::NpcAnimation*>(mAnimation);
        if (!animation || !blood || !blood->isBloody(item))
            return false;

        MWRender::Animation::AnimPriority priority = Priority_Weapon;
        priority[MWRender::BoneGroup_LowerBody] = Priority_WeaponLowerBody;
        mBrutalWipeWeapon = item;
        playBlendedAnimation(sSwordWipe, priority, MWRender::BlendMask_All, true, 1.f, "start", "stop", 0.f, 0);
        if (mAnimation->isPlaying(sSwordWipe))
        {
            animation->setSwordWipeItem(item);
            return true;
        }
        cancelBrutalWeaponWipe();
        return false;
    }

    void CharacterController::cancelBrutalWeaponWipe()
    {
        if (!mBrutalWipeWeapon.isSet())
            return;
        mBrutalWipeWeapon = {};
        if (auto* animation = dynamic_cast<MWRender::NpcAnimation*>(mAnimation))
            animation->setSwordWipeItem({});
        mAnimation->disable(sSwordWipe);
    }

    bool CharacterController::updateWeaponState()
    {
        if (mPtr == getPlayer()
            && updateBrutalWeaponWipe(MWBase::Environment::get().getWorld()->getPlayer().takeWeaponWipeRequest()))
            return false;

        // Source's Use timestamp belongs to the input, not the currently
        // playing attack/parry layer. A guard may cancel a windup without
        // resetting the age of the still-held attack button.
        const bool use = getAttackingOrSpell();
        const bool freshPress = use && !mBrutalUseButton;
        mBrutalUseHeld = use && mBrutalUseButton
            ? mBrutalUseHeld + MWBase::Environment::get().getFrameDuration() : 0.f;
        mBrutalUseButton = use;
        if (!mPtr.getClass().getCreatureStats(mPtr).mBrutalBlockRequested)
            mBrutalGuardCharge = false;
        normalizeBrutalDualDaggers();
        if (updateGroundFinisher())
            return false;
        if (updateBackstab())
            return false;
        // Count the continuation allowance in simulation time; pauses cannot
        // consume it. Guard, scripted actions and a changed set discard it.
        if (mBrutalFastGrace > 0.f)
        {
            mBrutalFastGrace = std::max(0.f,
                mBrutalFastGrace - MWBase::Environment::get().getFrameDuration());
            if (mBrutalFastGrace == 0.f)
                mBrutalFastResumeIndex = 0;
        }
        if (mPtr.getClass().getCreatureStats(mPtr).mBrutalBlockRequested || isScriptedAnimPlaying()
            || mAnimation->isPlaying("brutal_kick1h") || mAnimation->isPlaying("brutal_kick1h_low"))
            resetBrutalFastContinuation();
        const bool sliding = mPtr == getPlayer()
            && mPtr.getClass().getCreatureStats(mPtr).mBrutalSlidePhase != 0;
        if (sliding && !isBrutalOneHanded(mWeaponType))
            return false;
        // If the current animation is scripted, we can't do anything here.
        if (isScriptedAnimPlaying())
        {
            mBrutalBowRepeatUntil = -1.;
            mBrutalBowRepeatQueued = false;
            return false;
        }

        if ((sliding || mPtr.getClass().getCreatureStats(mPtr).mBrutalSlideAttack) && !getAttackingOrSpell())
            mBrutalComboRelease = false;

        // Dark Messiah combos replace the swing that starts while sprinting or airborne.
        if (!sliding && !mPtr.getClass().getCreatureStats(mPtr).mBrutalSlideAttack && updateBrutalCombo())
            return false;

        const auto world = MWBase::Environment::get().getWorld();
        auto& prng = world->getPrng();
        MWBase::SoundManager* sndMgr = MWBase::Environment::get().getSoundManager();

        const MWWorld::Class& cls = mPtr.getClass();
        CreatureStats& stats = cls.getCreatureStats(mPtr);
        int weaptype = ESM::Weapon::None;
        if (stats.getDrawState() == DrawState::Weapon)
            weaptype = ESM::Weapon::HandToHand;
        else if (stats.getDrawState() == DrawState::Spell)
            weaptype = ESM::Weapon::Spell;

        const bool isWerewolf = cls.isNpc() && cls.getNpcStats(mPtr).isWerewolf();

        const ESM::RefId* downSoundId = nullptr;
        bool weaponChanged = false;
        bool ammunition = true;
        float weapSpeed = 1.f;
        if (cls.hasInventoryStore(mPtr))
        {
            MWWorld::InventoryStore& inv = cls.getInventoryStore(mPtr);
            MWWorld::ContainerStoreIterator weapon = getActiveWeapon(mPtr, &weaptype);
            if (stats.getDrawState() == DrawState::Spell)
                weapon = inv.getSlot(MWWorld::InventoryStore::Slot_CarriedRight);

            MWWorld::Ptr newWeapon;
            if (weapon != inv.end())
            {
                newWeapon = *weapon;
                if (isRealWeapon(mWeaponType))
                    downSoundId = &newWeapon.getClass().getDownSoundId(newWeapon);
            }
            // weapon->HtH switch: weapon is empty already, so we need to take sound from previous weapon
            else if (!mWeapon.isEmpty() && weaptype == ESM::Weapon::HandToHand && mWeaponType != ESM::Weapon::Spell)
                downSoundId = &mWeapon.getClass().getDownSoundId(mWeapon);

            if (mWeapon != newWeapon)
            {
                mWeapon = newWeapon;
                weaponChanged = true;
                resetBrutalFastContinuation();
            }

            if (stats.getDrawState() == DrawState::Weapon && !mWeapon.isEmpty()
                && mWeapon.getType() == ESM::Weapon::sRecordId)
            {
                weapSpeed = mWeapon.get<ESM::Weapon>()->mBase->mData.mSpeed;
                MWWorld::ConstContainerStoreIterator ammo = inv.getSlot(MWWorld::InventoryStore::Slot_Ammunition);
                int ammotype = getWeaponType(mWeapon.get<ESM::Weapon>()->mBase->mData.mType)->mAmmoType;
                if (ammotype != ESM::Weapon::None)
                    ammunition = ammo != inv.end() && ammo->get<ESM::Weapon>()->mBase->mData.mType == ammotype;
                // Cancel attack if we no longer have ammunition
                if (!ammunition)
                    setAttackingOrSpell(false);
            }

            MWWorld::ConstContainerStoreIterator torch = inv.getSlot(MWWorld::InventoryStore::Slot_CarriedLeft);
            if (torch != inv.end() && torch->getType() == ESM::Light::sRecordId
                && updateCarriedLeftVisible(weaptype))
            {
                if (mAnimation->isPlaying("shield"))
                    mAnimation->disable("shield");

                playBlendedAnimation("torch", Priority_Torch, MWRender::BlendMask_LeftArm, false, 1.0f, "start", "stop",
                    0.0f, std::numeric_limits<uint32_t>::max(), true);
            }
            else if (mAnimation->isPlaying("torch"))
            {
                mAnimation->disable("torch");
            }
        }

        MWRender::Animation::AnimPriority priorityWeapon(Priority_Default);
        if (cls.isBipedal(mPtr))
        {
            // For bipeds, blend weapon animations with lower body animations with higher priority
            // For non-bipeds, movement takes priority
            priorityWeapon = Priority_Weapon;
            priorityWeapon[MWRender::BoneGroup_LowerBody] = Priority_WeaponLowerBody;
        }
        // Keep a slide-origin sword swing on the arms even if the slide naturally
        // enters OUT or finishes before a long-held windup is released.
        // Imported sections carry their own source clocks, including the original
        // attack-rate multiplier where applicable. Morrowind's weapon speed must
        // not multiply them again or compress the independent item/arm fades.
        if (!sliding && brutalFastAvailable())
            weapSpeed = 1.f;
        const int attackMask = sliding || stats.mBrutalSlideAttack
            ? MWRender::BlendMask_LeftArm | MWRender::BlendMask_RightArm : MWRender::BlendMask_All;

        bool forcestateupdate = false;

        // We should not play equipping animation and sound during weapon->weapon transition
        const bool isStillWeapon = isRealWeapon(mWeaponType) && isRealWeapon(weaptype);

        // If the current weapon type was changed in the middle of attack (e.g. by Equip console command or when bound
        // spell expires), we should force actor to the "weapon equipped" state, interrupt attack and update animations.
        // Morrowind does this at the end of the attack (see #4646 and PR 1972).
        // If we decide to cope with the resulting problems, the thrown weapon->H2H case below should be extended.
        if (isStillWeapon && mWeaponType != weaptype && mUpperBodyState > UpperBodyState::WeaponEquipped)
        {
            forcestateupdate = true;
            if (!mCurrentWeapon.empty())
                mAnimation->disable(mCurrentWeapon);
            mUpperBodyState = UpperBodyState::WeaponEquipped;
            mReadyToHit = false;
            stats.mBrutalSlashRight = false;
            stats.mBrutalSlideAttack = false;
            setAttackingOrSpell(false);
            mAnimation->showWeapons(true);
        }

        if (!isKnockedOut() && !isKnockedDown() && !isRecovery())
        {
            if (mPtr == getPlayer() && stats.mBrutalCarryingObject && weaptype == ESM::Weapon::None && weaptype != mWeaponType)
            {
                if (!mCurrentWeapon.empty())
                    mAnimation->disable(mCurrentWeapon);
                mAnimation->disable("shield");
                mAnimation->disable("torch");
                mAnimation->detachArrow();
                mAnimation->showWeapons(false);
                mAnimation->showCarriedLeft(false);
                mUpperBodyState = UpperBodyState::WeaponEquipped;
                mReadyToHit = false;
                stats.mBrutalSlashRight = false;
                stats.mBrutalSlideAttack = false;
                mBrutalBlock = BrutalBlock::None;
                stats.mBrutalBlockRequested = stats.mBrutalBlockRaised = stats.mBrutalParry = false;
                mWeaponType = weaptype;
                mCurrentWeapon.clear();
            }
            std::string weapgroup;
            if (((!isWerewolf && cls.isBipedal(mPtr)) || mWeaponType != ESM::Weapon::Spell) && weaptype != mWeaponType
                && mUpperBodyState <= UpperBodyState::AttackWindUp && mUpperBodyState != UpperBodyState::Unequipping
                && !isStillWeapon)
            {
                // We can not play un-equip animation if weapon changed since last update
                if (!weaponChanged)
                {
                    // Note: we do not disable the weapon unequipping animation automatically to avoid body desync
                    weapgroup = getWeaponAnimation(mWeaponType);
                    int unequipMask = MWRender::BlendMask_All;
                    mUpperBodyState = UpperBodyState::Unequipping;
                    mReadyToHit = false;
                    stats.mBrutalSlashRight = false;
                    stats.mBrutalSlideAttack = false;
                    bool useShieldAnims = mAnimation->useShieldAnimations();
                    if (useShieldAnims && mWeaponType != ESM::Weapon::HandToHand && mWeaponType != ESM::Weapon::Spell
                        && !(mWeaponType == ESM::Weapon::None && weaptype == ESM::Weapon::Spell))
                    {
                        unequipMask = unequipMask | ~MWRender::BlendMask_LeftArm;
                        mAnimation->disable("shield");
                        playBlendedAnimation("shield", Priority_Block, MWRender::BlendMask_LeftArm, true, 1.0f,
                            "unequip start", "unequip stop", 0.0f, 0);
                    }
                    else if (mWeaponType == ESM::Weapon::HandToHand)
                        mAnimation->showCarriedLeft(false);

                    mAnimation->disable(weapgroup);
                    playBlendedAnimation(
                        weapgroup, priorityWeapon, unequipMask, false, 1.0f, "unequip start", "unequip stop", 0.0f, 0);

                    mAnimation->detachArrow();

                    // If we do not have the "unequip detach" key, hide weapon manually.
                    if (mAnimation->getTextKeyTime(weapgroup + ": unequip detach") < 0)
                        mAnimation->showWeapons(false);
                }

                if (downSoundId && !downSoundId->empty())
                {
                    sndMgr->playSound3D(mPtr, *downSoundId, 1.0f, 1.0f);
                }
            }

            float complete;
            bool animPlaying = mAnimation->getInfo(mCurrentWeapon, &complete);
            if (!animPlaying || complete >= 1.0f)
            {
                // Weapon is changed, no current animation (e.g. unequipping or attack).
                // Start equipping animation now.
                if (weaptype != mWeaponType && mUpperBodyState <= UpperBodyState::WeaponEquipped)
                {
                    forcestateupdate = true;
                    bool useShieldAnims = mAnimation->useShieldAnimations();
                    if (!useShieldAnims)
                        mAnimation->showCarriedLeft(updateCarriedLeftVisible(weaptype));

                    weapgroup = getWeaponAnimation(weaptype);

                    // Note: controllers for ranged weapon should use time for beginning of animation to play shooting
                    // properly, for other weapons they should use absolute time. Some mods rely on this behaviour (to
                    // rotate throwing projectiles, for example)
                    ESM::WeaponType::Class weaponClass = getWeaponType(weaptype)->mWeaponClass;
                    bool useRelativeDuration = weaponClass == ESM::WeaponType::Ranged;
                    mAnimation->setWeaponGroup(weapgroup, useRelativeDuration);

                    if (!isStillWeapon)
                    {
                        if (animPlaying)
                            mAnimation->disable(mCurrentWeapon);
                        if (weaptype != ESM::Weapon::None)
                        {
                            mAnimation->showWeapons(false);
                            int equipMask = MWRender::BlendMask_All;
                            mUpperBodyState = UpperBodyState::Equipping;
                            mResetIdleOnAttackEnd = true;
                            if (useShieldAnims && weaptype != ESM::Weapon::Spell)
                            {
                                equipMask = equipMask | ~MWRender::BlendMask_LeftArm;
                                mAnimation->disable("shield");
                                playBlendedAnimation("shield", Priority_Block, MWRender::BlendMask_LeftArm, true, 1.0f,
                                    "equip start", "equip stop", 0.0f, 0);
                            }

                            if (weaptype != ESM::Weapon::Spell || cls.isBipedal(mPtr))
                            {
                                playBlendedAnimation(weapgroup, priorityWeapon, equipMask, false, 1.0f, "equip start",
                                    "equip stop", 0.0f, 0);
                            }

                            // If we do not have the "equip attach" key, show weapon manually.
                            if (weaptype != ESM::Weapon::Spell
                                && mAnimation->getTextKeyTime(weapgroup + ": equip attach") < 0)
                            {
                                mAnimation->showWeapons(true);
                            }

                            if (!mWeapon.isEmpty() && mWeaponType != ESM::Weapon::HandToHand && isRealWeapon(weaptype))
                            {
                                const ESM::RefId& upSoundId = mWeapon.getClass().getUpSoundId(mWeapon);
                                if (!upSoundId.empty())
                                    sndMgr->playSound3D(mPtr, upSoundId, 1.0f, 1.0f);
                            }
                        }
                    }

                    if (isWerewolf)
                    {
                        const MWWorld::ESMStore& store = world->getStore();
                        const ESM::Sound* sound = store.get<ESM::Sound>().searchRandom("WolfEquip", prng);
                        if (sound)
                        {
                            sndMgr->playSound3D(mPtr, sound->mId, 1.0f, 1.0f);
                        }
                    }

                    mWeaponType = weaptype;
                    mCurrentWeapon = weapgroup;
                }

                // Make sure that we disabled unequipping animation
                if (mUpperBodyState == UpperBodyState::Unequipping)
                {
                    resetCurrentWeaponState();
                    mWeaponType = ESM::Weapon::None;
                }
            }
        }

        if (isWerewolf)
        {
            const ESM::RefId wolfRun = ESM::RefId::stringRefId("WolfRun");
            if (isRunning() && !world->isSwimming(mPtr) && mWeaponType == ESM::Weapon::None)
            {
                if (!sndMgr->getSoundPlaying(mPtr, wolfRun))
                    sndMgr->playSound3D(mPtr, wolfRun, 1.0f, 1.0f, MWSound::Type::Sfx, MWSound::PlayMode::Loop);
            }
            else
                sndMgr->stopSound3D(mPtr, wolfRun);
        }

        brutalBounce();
        // The visible parry recovery can continue after Source accepts the
        // next attack. Dagger left-idle has its own later readiness marker.
        if (mBrutalBlock == BrutalBlock::Lowering && getAttackingOrSpell()
            && mAnimation->activeSourceHasTextKey("brutal_block: start", "brutal_block: attack ready")
            && mAnimation->getCurrentTime("brutal_block")
                >= mAnimation->getTextKeyTime("brutal_block: attack ready"))
        {
            mAnimation->disable("brutal_block");
            mBrutalBlock = BrutalBlock::None;
        }
        if (brutalFastAvailable())
            brutalGuardAttack();
        // Dark Messiah attacks straight out of the sword parry (Parry_Idle + attack); the
        // held guard rises again once the swing is over.
        else if (getAttackingOrSpell() && mBrutalBlock != BrutalBlock::None && stats.mBrutalParry
            && mUpperBodyState == UpperBodyState::WeaponEquipped)
        {
            mAnimation->disable("brutal_block");
            mBrutalBlock = BrutalBlock::None;
            stats.mBrutalBlockRaised = stats.mBrutalParry = false;
            stats.mBrutalBlockLoweredAt = brutalSimulationTime();
        }

        float complete = 0.f;
        bool animPlaying = false;
        ESM::WeaponType::Class weapclass = getWeaponType(mWeaponType)->mWeaponClass;
        const bool canRepeatBow = mPtr == getPlayer() && mWeaponType == ESM::Weapon::MarksmanBow
            && weaptype == mWeaponType && ammunition && brutalDmFirstPerson();
        const double bowTime = canRepeatBow ? brutalSimulationTime() : 0.;
        if (!canRepeatBow || weaponChanged || (mBrutalBowRepeatUntil >= 0. && bowTime > mBrutalBowRepeatUntil))
        {
            mBrutalBowRepeatUntil = -1.;
            mBrutalBowRepeatQueued = false;
        }
        const bool repeatBow = canRepeatBow && mBrutalBowRepeatUntil >= bowTime;
        if (canRepeatBow && freshPress && (mUpperBodyState == UpperBodyState::AttackRelease
                                             || mUpperBodyState == UpperBodyState::AttackEnd))
            mBrutalBowRepeatQueued = true;
        const bool bufferedBow = repeatBow && mBrutalBowRepeatQueued;
        if (getAttackingOrSpell() || bufferedBow)
        {
            mResetIdleOnAttackEnd = true;
            if (mUpperBodyState == UpperBodyState::WeaponEquipped && !mBrutalGuardPress
                && (mHitState == CharState_None || mHitState == CharState_Block) && !isBrutalShieldBlocking()
                // A sword must not advance and emit hit keys beneath the scripted kick pose.
                && !(mPtr == getPlayer()
                    && (mAnimation->isPlaying("brutal_kick") || mAnimation->isPlaying("brutal_kick1h")
                        || mAnimation->isPlaying("brutal_kick1h_low"))))
            {
                mAttackStrength = -1.f;
                mReadyToHit = false;
                mBrutalChargedHold = mBrutalGuardCharge = false;
                stats.mBrutalSlashRight = false;
                stats.mBrutalSlideAttack = false;
                mBrutalBowFastDraw = repeatBow;
                mBrutalBowRepeatUntil = -1.;
                mBrutalBowRepeatQueued = false;

                // Randomize attacks for non-bipedal creatures
                if (!cls.isBipedal(mPtr)
                    && (!mAnimation->hasAnimation(mCurrentWeapon) || isRandomAttackAnimation(mCurrentWeapon)))
                {
                    mCurrentWeapon = chooseRandomAttackAnimation();
                }

                if (mWeaponType == ESM::Weapon::Spell)
                {
                    // Unset casting flag, otherwise pressing the mouse button down would
                    // continue casting every frame if there is no animation
                    setAttackingOrSpell(false);
                    if (mPtr == getPlayer())
                    {
                        // For the player, set the spell we want to cast
                        // This has to be done at the start of the casting animation,
                        // *not* when selecting a spell in the GUI (otherwise you could change the spell mid-animation)
                        const ESM::RefId& selectedSpell
                            = MWBase::Environment::get().getWindowManager()->getSelectedSpell();
                        stats.getSpells().setSelectedSpell(selectedSpell);
                    }
                    ESM::RefId spellid = stats.getSpells().getSelectedSpell();
                    bool isMagicItem = false;

                    // Play hand VFX and allow castSpell use (assuming an animation is going to be played) if
                    // spellcasting is successful. Scripted spellcasting bypasses restrictions.
                    MWWorld::SpellCastState spellCastResult = MWWorld::SpellCastState::Success;
                    if (!mCastingScriptedSpell)
                        spellCastResult = world->startSpellCast(mPtr);
                    mCanCast = spellCastResult == MWWorld::SpellCastState::Success;

                    if (spellid.empty())
                    {
                        const MWWorld::ContainerStore& inv = cls.getContainerStore(mPtr);
                        if (inv.getSelectedEnchantItem() != inv.end())
                        {
                            const MWWorld::ConstPtr& enchantItem = *inv.getSelectedEnchantItem();
                            spellid = enchantItem.getClass().getEnchantment(enchantItem);
                            isMagicItem = true;
                        }
                    }

                    if (isMagicItem && !Settings::game().mUseMagicItemAnimations)
                    {
                        world->breakInvisibility(mPtr);
                        // Enchanted items by default do not use casting animations
                        world->castSpell(mPtr);
                        // Spellcasting animation needs to "play" for at least one frame to reset the aiming factor
                        animPlaying = true;
                        mUpperBodyState = UpperBodyState::Casting;
                        // But idle should not be reset
                        mResetIdleOnAttackEnd = false;
                    }
                    // Play the spellcasting animation/VFX if the spellcasting was successful or failed due to
                    // insufficient magicka. Used up powers are exempt from this from some reason.
                    else if (!spellid.empty() && spellCastResult != MWWorld::SpellCastState::PowerAlreadyUsed)
                    {
                        world->breakInvisibility(mPtr);
                        MWMechanics::CastSpell cast(mPtr, {}, false, mCastingScriptedSpell);

                        const std::vector<ESM::IndexedENAMstruct>* effects{ nullptr };
                        const MWWorld::ESMStore& store = world->getStore();
                        if (isMagicItem)
                        {
                            const ESM::Enchantment* enchantment = store.get<ESM::Enchantment>().find(spellid);
                            effects = &enchantment->mEffects.mList;
                            cast.playSpellCastingEffects(enchantment);
                        }
                        else
                        {
                            const ESM::Spell* spell = store.get<ESM::Spell>().find(spellid);
                            effects = &spell->mEffects.mList;
                            cast.playSpellCastingEffects(spell);
                        }
                        if (!effects->empty())
                        {
                            if (mCanCast)
                            {
                                const ESM::MagicEffect* effect = store.get<ESM::MagicEffect>().find(
                                    effects->back().mData.mEffectID); // use last effect of list for color of VFX_Hands

                                const ESM::Static* castStatic
                                    = world->getStore().get<ESM::Static>().find(ESM::RefId::stringRefId("VFX_Hands"));

                                const VFS::Path::Normalized castStaticModel
                                    = Misc::ResourceHelpers::correctMeshPath(VFS::Path::Normalized(castStatic->mModel));

                                if (mAnimation->getNode("Bip01 L Hand"))
                                    mAnimation->addEffect(
                                        castStaticModel.value(), "", false, "Bip01 L Hand", effect->mParticle);

                                if (mAnimation->getNode("Bip01 R Hand"))
                                    mAnimation->addEffect(
                                        castStaticModel.value(), "", false, "Bip01 R Hand", effect->mParticle);
                            }
                            // first effect used for casting animation
                            const ESM::ENAMstruct& firstEffect = effects->front().mData;

                            std::string startKey;
                            std::string stopKey;
                            if (isRandomAttackAnimation(mCurrentWeapon))
                            {
                                startKey = "start";
                                stopKey = "stop";
                                if (mCanCast)
                                    world->castSpell(mPtr,
                                        mCastingScriptedSpell); // No "release" text key to use, so cast immediately
                                mCastingScriptedSpell = false;
                                mCanCast = false;
                            }
                            else
                            {
                                switch (firstEffect.mRange)
                                {
                                    case 0:
                                        mAttackType = "self";
                                        break;
                                    case 1:
                                        mAttackType = "touch";
                                        break;
                                    case 2:
                                        mAttackType = "target";
                                        break;
                                }

                                startKey = mAttackType + " start";
                                stopKey = mAttackType + " stop";
                            }
                            playBlendedAnimation(mCurrentWeapon, priorityWeapon, MWRender::BlendMask_All, false, 1,
                                startKey, stopKey, 0.0f, 0);
                            mUpperBodyState = UpperBodyState::Casting;
                        }
                    }
                }
                else
                {
                    std::string startKey = "start";
                    std::string stopKey = "stop";

                    MWBase::LuaManager::ActorControls* actorControls
                        = MWBase::Environment::get().getLuaManager()->getActorControls(mPtr);
                    const bool aiInactive
                        = actorControls->mDisableAI || !MWBase::Environment::get().getMechanicsManager()->isAIActive();
                    if (mWeaponType != ESM::Weapon::PickProbe && !isRandomAttackAnimation(mCurrentWeapon))
                    {
                        if (weapclass == ESM::WeaponType::Ranged || weapclass == ESM::WeaponType::Thrown)
                            mAttackType = "shoot";
                        else if (sliding && mCurrentWeapon == "weapononehand")
                        {
                            // The low hand strike always uses the real source _L
                            // slash, independently of strafe, aim height or best-attack.
                            mAttackType = "slash";
                            stats.mBrutalSlideAttack = true;
                        }
                        else if (mPtr == getPlayer())
                        {
                            if (Settings::game().mBestAttack)
                            {
                                if (!mWeapon.isEmpty() && mWeapon.getType() == ESM::Weapon::sRecordId)
                                {
                                    mAttackType = getBestAttack(mWeapon.get<ESM::Weapon>()->mBase);
                                }
                                else
                                {
                                    // There is no "best attack" for Hand-to-Hand
                                    mAttackType = getRandomAttackType();
                                }
                            }
                            else
                            {
                                mAttackType = getMovementBasedAttackType();
                            }
                        }
                        else if (aiInactive)
                        {
                            mAttackType = getDesiredAttackType();
                            if (mAttackType == "")
                                mAttackType = getRandomAttackType();
                        }

                        // else if (mPtr != getPlayer()) use mAttackType set by AiCombat
                        // Dark Messiah's leftward strafe selects source _R, while
                        // rightward/stationary swings retain source _L. Capture it
                        // once: changing direction during a held windup cannot splice
                        // into the opposite release or reverse its hit metadata.
                        stats.mBrutalSlashRight = mPtr == getPlayer()
                            && (mCurrentWeapon == "weapononehand" || usesBrutalPairedWeapon())
                            && !stats.mBrutalSlideAttack && mAttackType == "slash"
                            && cls.getMovementSettings(mPtr).mPosition[0] < -.1f
                            && mAnimation->activeSourceHasTextKey(
                                mCurrentWeapon + ": slash start", mCurrentWeapon + ": slashright start")
                            && mAnimation->getTextKeyTime(mCurrentWeapon + ": slashright start") >= 0.f
                            && mAnimation->getTextKeyTime(mCurrentWeapon + ": slashright max attack") >= 0.f
                            && mAnimation->getTextKeyTime(mCurrentWeapon + ": slashright hit") >= 0.f;
                        mBrutalNeutralAttack = false;
                        const bool spear = mPtr == getPlayer() && mWeaponType == ESM::Weapon::SpearTwoWide
                            && brutalDmFirstPerson()
                            && brutalActiveKey("brutal spear grip");
                        if (brutalFastAvailable() || spear)
                        {
                            chooseBrutalChargedAttack();
                            mBrutalSpearStrafe = getBrutalSpearStrafe();
                        }
                        const std::string attackAnimation = getAttackAnimationType();
                        startKey = attackAnimation + ' ' + startKey;
                        stopKey = attackAnimation + " max attack";
                    }

                    // Latch eligibility at the fresh press, rather than after
                    // the click's short windup. The normal hold threshold still
                    // decides whether release is fast or charged.
                    mBrutalFastWindupIndex = brutalFastAvailable() && mBrutalFastGrace > 0.f
                        ? mBrutalFastResumeIndex : 0;
                    mBrutalFastGrace = 0.f;
                    mBrutalFastResumeIndex = 0;
                    mUpperBodyState = UpperBodyState::AttackWindUp;

                    // Reset the attack results when the attack starts.
                    // Strictly speaking this should probably be done when the attack ends,
                    // but the attack animation might be cancelled in a myriad different ways.
                    mAttackSuccess = false;
                    mAttackVictim = MWWorld::Ptr();
                    mAttackHitPos = osg::Vec3f();
                    mAttackContactPart = -1;

                    const float drawSpeed = mBrutalBowFastDraw && startKey == "shoot start" ? weapSpeed * 1.6f : weapSpeed;
                    playBlendedAnimation(mCurrentWeapon, priorityWeapon, attackMask, false, drawSpeed,
                        startKey, stopKey, 0.0f, 0);
                }
            }
        }

        // Source has a distinct charged idle after the windup. Some are real
        // loops, others settle once and then retain their final pose.
        if (mUpperBodyState == UpperBodyState::AttackWindUp && getAttackingOrSpell()
            && !mBrutalChargedHold && brutalFastAvailable())
        {
            const float end = mAnimation->getTextKeyTime(mCurrentWeapon + ": " + getAttackAnimationType() + " max attack");
            if (end >= 0.f && mAnimation->getCurrentTime(mCurrentWeapon) >= end)
                startBrutalChargedHold();
        }

        // Random attack and pick/probe animations never have wind up and are played to their end.
        // Other animations must be released when the attack state is unset.
        if (mUpperBodyState == UpperBodyState::AttackWindUp
            && (mWeaponType == ESM::Weapon::PickProbe || isRandomAttackAnimation(mCurrentWeapon)
                || !getAttackingOrSpell()))
        {
            const bool fullyCharged = mBrutalChargedHold || calculateWindUp() >= 1.f;
            if (!getAttackingOrSpell() && fullyCharged && startGroundFinisher())
                return false;
            if (!getAttackingOrSpell() && canPlayBackstab(false))
            {
                if (const MWWorld::Ptr backstabTarget = findBackstabTarget(false); !backstabTarget.isEmpty())
                {
                    MWRender::Animation::ScopedDeferActiveGroupReset defer(*mAnimation);
                    mAnimation->disable(mCurrentWeapon);
                    mReadyToHit = false;
                    mBrutalChargedHold = false;
                    mUpperBodyState = UpperBodyState::WeaponEquipped;
                    enterBackstabStance(backstabTarget);
                    mBackstab.mAttackQueued = true;
                    return false;
                }
            }
            if (mBrutalChargedHold)
            {
                // Return to the release marker without playing the authored
                // windup history. The visible held pose remains the blend source.
                const std::string attack = getAttackAnimationType();
                mAnimation->disable(mCurrentWeapon);
                playBlendedAnimation(mCurrentWeapon, priorityWeapon, attackMask, false, 1.f,
                    attack + " max attack", attack + " max attack", 0.f, 0);
                mBrutalChargedHold = false;
            }
            // In Dark Messiah a release before the windup clip ends is a click: it strikes the
            // first fast attack instead of a partly charged swing.
            const float windupEnd = !brutalFastAvailable() ? -1.f
                : mAnimation->getTextKeyTime(mCurrentWeapon + ": " + getAttackAnimationType() + " max attack");
            const bool brutalClick = windupEnd >= 0.f && mAnimation->getCurrentTime(mCurrentWeapon) < windupEnd - 1e-3f;
            mUpperBodyState = UpperBodyState::AttackRelease;
            world->breakInvisibility(mPtr);
            if (brutalClick)
            {
                mBrutalFastIndex = mBrutalFastWindupIndex;
                mBrutalFastQueued = false;
                mBrutalFastButton = false;
                mBrutalFastHeld = 0.f;
                mUpperBodyState = UpperBodyState::AttackEnd;
                startBrutalFastNext();
            }
            else if (mWeaponType == ESM::Weapon::PickProbe)
            {
                // TODO: this will only work for the player, and needs to be fixed if NPCs should ever use
                // lockpicks/probes.
                MWWorld::Ptr target = world->getFocusObject();

                if (!target.isEmpty())
                {
                    std::string_view resultMessage, resultSound;
                    if (mWeapon.getType() == ESM::Lockpick::sRecordId)
                        Security(mPtr).pickLock(target, mWeapon, resultMessage, resultSound);
                    else if (mWeapon.getType() == ESM::Probe::sRecordId)
                        Security(mPtr).probeTrap(target, mWeapon, resultMessage, resultSound);
                    if (!resultMessage.empty())
                        MWBase::Environment::get().getWindowManager()->messageBox(resultMessage);
                    if (!resultSound.empty())
                        sndMgr->playSound3D(target, ESM::RefId::stringRefId(resultSound), 1.0f, 1.0f);
                }
            }
            // Evaluate the attack results and play the swish sound.
            // Attack animations with no hit key do this earlier.
            else
            {
                prepareHit();
            }

            if (mWeaponType == ESM::Weapon::PickProbe || isRandomAttackAnimation(mCurrentWeapon))
                mUpperBodyState = UpperBodyState::AttackEnd;
        }

        if (mUpperBodyState == UpperBodyState::AttackRelease)
        {
            // The release state might have been reached before reaching the wind-up section. We'll play the new section
            // only when the wind-up section is reached.
            float currentTime = mAnimation->getCurrentTime(mCurrentWeapon);
            const std::string attackAnimation = getAttackAnimationType();
            float minAttackTime = mAnimation->getTextKeyTime(mCurrentWeapon + ": " + attackAnimation + " min attack");
            float maxAttackTime = mAnimation->getTextKeyTime(mCurrentWeapon + ": " + attackAnimation + " max attack");
            const bool continuousPlayerSwordRelease = mPtr == getPlayer() && world->isFirstPerson()
                && MWMechanics::brutalDmFirstPerson() && (mCurrentWeapon == "weapononehand" || usesBrutalPairedWeapon())
                && mAnimation->getNode("Brutal Camera Pivot") != nullptr;
            const bool continuousNpcSwordRelease = mPtr != getPlayer() && cls.isNpc()
                && mCurrentWeapon == "weapononehand"
                && mAnimation->getTextKeyTime(mCurrentWeapon + ": brutal continuous release") >= 0.f;
            const bool continuousThrowRelease = mPtr == getPlayer() && mCurrentWeapon == "throwweapon"
                && mAnimation->getTextKeyTime("throwweapon: brutal continuous release") >= 0.f;
            const bool continuousRelease
                = continuousPlayerSwordRelease || continuousNpcSwordRelease || continuousThrowRelease;
            if (continuousRelease && currentTime < maxAttackTime)
            {
                // Strength was captured by prepareHit at release. Traverse the remaining
                // authored windup without seeking to its final pose; then restore normal
                // speed for the strike. At 3x, an early tap adds at most about .22 seconds.
                mAnimation->adjustSpeedMult(mCurrentWeapon, weapSpeed * 3.f);
            }
            if (minAttackTime <= currentTime && currentTime <= maxAttackTime
                && (!continuousRelease || currentTime >= maxAttackTime))
            {
                std::string hit = mAttackType != "shoot" ? "hit" : "release";

                float startPoint = 0.f;

                // Skip a bit of the pre-hit section based on the attack strength
                if (!continuousRelease && minAttackTime != -1.f && minAttackTime < maxAttackTime)
                {
                    startPoint = 1.f - mAttackStrength;
                    float minHitTime = mAnimation->getTextKeyTime(mCurrentWeapon + ": " + attackAnimation + " min hit");
                    float hitTime = mAnimation->getTextKeyTime(mCurrentWeapon + ": " + attackAnimation + ' ' + hit);
                    if (maxAttackTime <= minHitTime && minHitTime < hitTime)
                        startPoint *= (minHitTime - maxAttackTime) / (hitTime - maxAttackTime);
                }

                mAnimation->disable(mCurrentWeapon);
                playBlendedAnimation(mCurrentWeapon, priorityWeapon, attackMask, false, weapSpeed,
                    attackAnimation + " max attack", attackAnimation + ' ' + hit, startPoint, 0);
            }

            animPlaying = mAnimation->getInfo(mCurrentWeapon, &complete);

            // Try playing the "follow" section if the attack animation ended naturally or didn't play at all.
            if (!animPlaying || (currentTime >= maxAttackTime && complete >= 1.f))
            {
                std::string start = "follow start";
                std::string stop = "follow stop";

                if (mAttackType != "shoot")
                {
                    std::string strength = mAttackStrength < 0.33f ? "small"
                        : mAttackStrength < 0.66f                  ? "medium"
                                                                   : "large";
                    start = strength + ' ' + start;
                    stop = strength + ' ' + stop;
                }

                mReadyToHit = false;

                if (animPlaying)
                    mAnimation->disable(mCurrentWeapon);
                playBlendedAnimation(mCurrentWeapon, priorityWeapon, attackMask, false, weapSpeed,
                    attackAnimation + ' ' + start, attackAnimation + ' ' + stop, 0.0f, 0);
                mUpperBodyState = UpperBodyState::AttackEnd;

                animPlaying = mAnimation->getInfo(mCurrentWeapon, &complete);
            }
        }

        if (!animPlaying)
            animPlaying = mAnimation->getInfo(mCurrentWeapon, &complete);

        if (mBrutalFast != BrutalFast::None)
            updateBrutalFast(animPlaying, complete);

        // The staff's regrip and the bow's recovery can hand off at their chain marker.
        if (animPlaying && mUpperBodyState == UpperBodyState::AttackEnd && mPtr == getPlayer()
            && mBrutalFast == BrutalFast::None && mAttackStrength >= 0.f
            && (stats.getAttackingOrSpell() || bufferedBow))
        {
            const float chain = mAnimation->getTextKeyTime(mCurrentWeapon + ": " + getAttackAnimationType() + " chain");
            if (chain >= 0.f && mAnimation->getCurrentTime(mCurrentWeapon) >= chain)
                complete = 1.f;
        }

        if (!animPlaying || complete >= 1.f)
        {
            if (mUpperBodyState == UpperBodyState::Equipping || mUpperBodyState == UpperBodyState::AttackEnd
                || mUpperBodyState == UpperBodyState::Casting)
            {
                if (ammunition && mWeaponType == ESM::Weapon::MarksmanCrossbow)
                    mAnimation->attachArrow();

                // Cancel stagger animation at the end of an attack to avoid abrupt transitions
                // in favor of a different abrupt transition, like Morrowind. Brutal NPC hurt
                // reactions keep playing: a blow landing as an attack ends must still show.
                const bool brutalNpcReaction
                    = mPtr != getPlayer() && Settings::game().mNpcHurtHealthFraction > 0.f;
                if (mUpperBodyState != UpperBodyState::Equipping && isRecovery() && !brutalNpcReaction)
                    mAnimation->disable(mCurrentHit);

                if (animPlaying)
                    mAnimation->disable(mCurrentWeapon);

                // Skip Thrown->H2H idle transition (e.g., if we've run out of ammo)
                // In Morrowind this isn't actually specific to this transition
                // See the weapon->weapon mid-attack skip logic above
                if (mUpperBodyState == UpperBodyState::AttackEnd)
                {
                    mBrutalBowFastDraw = false;
                    if (!repeatBow)
                        mBrutalBowRepeatQueued = false;
                    stats.mBrutalSlashRight = false;
                    stats.mBrutalSlideAttack = false;
                    if (weapclass == ESM::WeaponType::Thrown && weaptype == ESM::Weapon::HandToHand)
                    {
                        forcestateupdate = true;
                        mWeaponType = weaptype;
                        mCurrentWeapon = getWeaponAnimation(mWeaponType);
                        mAnimation->showWeapons(false);
                    }
                }
                // We should not break swim and sneak animations
                if (mResetIdleOnAttackEnd && mIdleState != CharState_IdleSneak && mIdleState != CharState_IdleSwim)
                {
                    resetCurrentIdleState();
                }

                mUpperBodyState = UpperBodyState::WeaponEquipped;
            }
            else if (mUpperBodyState == UpperBodyState::Unequipping)
            {
                if (animPlaying)
                    mAnimation->disable(mCurrentWeapon);
                mUpperBodyState = UpperBodyState::None;
                mWeaponType = ESM::Weapon::None;
            }
        }

        mAnimation->setPitchFactor(0.f);
        if (mUpperBodyState > UpperBodyState::WeaponEquipped
            && (weapclass == ESM::WeaponType::Ranged || weapclass == ESM::WeaponType::Thrown))
        {
            mAnimation->setPitchFactor(1.f);

            // A smooth transition can be provided if a pre-wind-up section is defined. Random attack animations never
            // have one.
            if (mUpperBodyState == UpperBodyState::AttackWindUp && !isRandomAttackAnimation(mCurrentWeapon))
            {
                float currentTime = mAnimation->getCurrentTime(mCurrentWeapon);
                const std::string attack = getAttackAnimationType();
                float minAttackTime = mAnimation->getTextKeyTime(mCurrentWeapon + ": " + attack + " min attack");
                float startTime = mAnimation->getTextKeyTime(mCurrentWeapon + ": " + attack + " start");
                if (startTime <= currentTime && currentTime < minAttackTime)
                    mAnimation->setPitchFactor((currentTime - startTime) / (minAttackTime - startTime));
            }
            else if (mUpperBodyState == UpperBodyState::AttackEnd)
            {
                // technically we do not need a pitch for crossbow reload animation,
                // but we should avoid abrupt repositioning
                if (mWeaponType == ESM::Weapon::MarksmanCrossbow)
                    mAnimation->setPitchFactor(std::max(0.f, 1.f - complete * 10.f));
                else
                    mAnimation->setPitchFactor(1.f - complete);
            }
        }

        mAnimation->setAccurateAiming(mUpperBodyState > UpperBodyState::WeaponEquipped);

        return forcestateupdate;
    }

    void CharacterController::updateAnimQueue()
    {
        if (mAnimQueue.empty())
            return;

        if (!mAnimation->isPlaying(mAnimQueue.front().mGroup))
        {
            // Playing animations through mwscript is weird. If an animation is
            // a looping animation (idle or other cyclical animations), then they
            // will end as expected. However, if they are non-looping animations, they
            // will stick around forever or until another animation appears in the queue.
            bool shouldPlayOrRestart = mAnimQueue.size() > 1;
            if (shouldPlayOrRestart || !mAnimQueue.front().mScripted
                || (mAnimQueue.front().mLoopCount == 0 && mAnimQueue.front().mLooping))
            {
                mAnimation->setPlayScriptedOnly(false);
                mAnimation->disable(mAnimQueue.front().mGroup);
                mAnimQueue.pop_front();
                shouldPlayOrRestart = true;
            }
            else
                // A non-looping animation will stick around forever, so only restart if the animation
                // actually was removed for some reason.
                shouldPlayOrRestart = !mAnimation->getInfo(mAnimQueue.front().mGroup)
                    && mAnimation->hasAnimation(mAnimQueue.front().mGroup);

            if (shouldPlayOrRestart)
            {
                // Move on to the remaining items of the queue
                playAnimQueue();
            }
        }
        else
        {
            float complete;
            uint32_t loopcount;
            mAnimation->getInfo(mAnimQueue.front().mGroup, &complete, nullptr, &loopcount);
            mAnimQueue.front().mLoopCount = loopcount;
            mAnimQueue.front().mTime = complete;
        }

        if (!mAnimQueue.empty())
            mAnimation->setLoopingEnabled(mAnimQueue.front().mGroup, mAnimQueue.size() <= 1);
    }

    void CharacterController::playAnimQueue(bool loopStart)
    {
        if (!mAnimQueue.empty())
        {
            clearStateAnimation(mCurrentIdle);
            mIdleState = CharState_SpecialIdle;
            auto priority = mAnimQueue.front().mScripted ? Priority_Scripted : Priority_Default;
            mAnimation->setPlayScriptedOnly(mAnimQueue.front().mScripted);
            if (mAnimQueue.front().mScripted)
                mAnimation->play(mAnimQueue.front().mGroup, priority, MWRender::BlendMask_All, false,
                    mAnimQueue.front().mSpeed, (loopStart ? "loop start" : mAnimQueue.front().mStartKey),
                    mAnimQueue.front().mStopKey, mAnimQueue.front().mTime, mAnimQueue.front().mLoopCount,
                    mAnimQueue.front().mLooping);
            else
                playBlendedAnimation(mAnimQueue.front().mGroup, priority, MWRender::BlendMask_All, false,
                    mAnimQueue.front().mSpeed, (loopStart ? "loop start" : mAnimQueue.front().mStartKey),
                    mAnimQueue.front().mStopKey, mAnimQueue.front().mTime, mAnimQueue.front().mLoopCount,
                    mAnimQueue.front().mLooping);
        }
    }

    void CharacterController::update(float duration)
    {
        if (!mAnimation)
        {
            mBrutalSprinting = false;
            return;
        }
        MWBase::World* world = MWBase::Environment::get().getWorld();
        MWBase::SoundManager* sndMgr = MWBase::Environment::get().getSoundManager();
        const MWWorld::Class& cls = mPtr.getClass();
        osg::Vec3f movement(0.f, 0.f, 0.f);
        float speed = 0.f;

        updateMagicEffects();
        if (cls.isActor())
        {
            auto& stats = cls.getCreatureStats(mPtr);
            stats.mBrutalClinchBlendOut = std::max(0.f, stats.mBrutalClinchBlendOut - std::max(duration, 0.f));
        }
        // A living falling ragdoll owns all body motion, without a canned fall
        // or a latent weapon hit. Magic/death updates above remain live.
        if (updateSwordClinch(duration) || updateBackstabVictim(duration)
            || (mAnimation->isRagdollPose() && !cls.getCreatureStats(mPtr).isDead()))
        {
            mBrutalSprinting = false;
            return;
        }
        if (cls.isActor() && cls.getCreatureStats(mPtr).mBrutalParried)
        {
            // ACT_MELEE_PARRY_BREAK: the player timed a parry of this actor's blow.
            cls.getCreatureStats(mPtr).mBrutalParried = false;
            stagger("brutal_parrybroken");
        }
        if (cls.isNpc() && mPtr != MWMechanics::getPlayer())
        {
            updateBrutalWounded();
            if (mAnimation->isRagdollPose())
                return;
            updateBrutalPickup();
        }

        bool isPlayer = mPtr == MWMechanics::getPlayer();
        if (isPlayer && cls.isActor() && cls.getCreatureStats(mPtr).mBrutalCarryingObject)
        {
            auto& st = cls.getCreatureStats(mPtr);
            if (st.getDrawState() != DrawState::Nothing)
                st.setDrawState(DrawState::Nothing);
            setAttackingOrSpell(false);
            st.mBrutalBlockRequested = st.mBrutalBlockRaised = st.mBrutalParry = false;
            mBrutalBlock = BrutalBlock::None;
            if (mUpperBodyState > UpperBodyState::WeaponEquipped)
                cancelAttackForKick();
        }
        if (isPlayer && mBrutalFast != BrutalFast::Attack)
            cls.getCreatureStats(mPtr).mBrutalAttackKind = mAttackType == "chop" ? 0 : mAttackType == "thrust" ? 2 : 1;
        if (isPlayer && !cls.getCreatureStats(mPtr).mPowerStrike.token.empty())
        {
            auto& inventory = cls.getInventoryStore(mPtr);
            auto equipped = inventory.getSlot(MWWorld::InventoryStore::Slot_CarriedRight);
            auto& armed = cls.getCreatureStats(mPtr).mPowerStrike;
            if (equipped == inventory.end() || *equipped != armed.weapon.ptrOrEmpty() || cls.getCreatureStats(mPtr).isDead())
                armed = {};
        }

        const bool kineticPlayer = isPlayer && Settings::game().mKineticPlayerMovement;
        const bool forcedWalk = kineticPlayer && cls.getNormalizedEncumbrance(mPtr) > 1.f;
        bool isFirstPersonPlayer = isPlayer && MWBase::Environment::get().getWorld()->isFirstPerson();
        bool godmode = isPlayer && MWBase::Environment::get().getWorld()->getGodModeState();

        float scale = mPtr.getCellRef().getScale();
        if (kineticPlayer)
            scale = 1.f;

        if (!kineticPlayer && !Settings::game().mNormaliseRaceSpeed && cls.isNpc())
        {
            const ESM::NPC* npc = mPtr.get<ESM::NPC>()->mBase;
            const ESM::Race* race = world->getStore().get<ESM::Race>().find(npc->mRace);
            float weight = npc->isMale() ? race->mData.mMaleWeight : race->mData.mFemaleWeight;
            scale *= weight;
        }

        if (cls.isActor() && cls.getCreatureStats(mPtr).wasTeleported())
        {
            if (isPlayer)
            {
                if (cls.getCreatureStats(mPtr).mBrutalSlideAttack)
                    cancelAttackForKick();
                cancelBrutalSlide();
            }
            mSmoothedSpeed = osg::Vec2f();
            world->clearActorExternalVelocity(mPtr);
            cls.getCreatureStats(mPtr).setTeleported(false);
        }

        if (isPlayer)
        {
            if (forcedWalk)
                mBrutalSprinting = false;
            updateBrutalSlide(duration);
        }

        if (!cls.isActor())
            updateAnimQueue();
        else if (!cls.getCreatureStats(mPtr).isDead())
        {
            bool onground = world->isOnGround(mPtr);
            bool inwater = world->isSwimming(mPtr);
            bool flying = world->isFlying(mPtr);
            bool solid = world->isActorCollisionEnabled(mPtr);
            // Can't run and sneak while flying (see speed formula in Npc/Creature::getSpeed)
            bool sneak
                = cls.getCreatureStats(mPtr).getStance(MWMechanics::CreatureStats::Stance_Sneak) && !flying && !inwater;
            if (kineticPlayer && cls.getCreatureStats(mPtr).mBrutalSlideBlocked)
                sneak = true;
            bool isrunning = cls.getCreatureStats(mPtr).getStance(MWMechanics::CreatureStats::Stance_Run) && !flying;
            CreatureStats& stats = cls.getCreatureStats(mPtr);
            // A Lua-selected Dark Messiah gait replaces Morrowind's run stance.
            const int inputGait = (kineticPlayer || Settings::game().mBrutalSlide) ? stats.mBrutalGait : 0;
            const int gait = forcedWalk ? MWPhysics::PlayerKinetics::Gait_Walk : inputGait;
            if (gait != MWPhysics::PlayerKinetics::Gait_Stance)
                isrunning = gait != MWPhysics::PlayerKinetics::Gait_Walk && !flying;
            Movement& movementSettings = cls.getMovementSettings(mPtr);
            if (isPlayer && (mGroundFinisher || mBackstab.mPhase == BrutalBackstabPhase::Executing))
            {
                std::fill(std::begin(movementSettings.mPosition), std::end(movementSettings.mPosition), 0.f);
                movementSettings.mSpeedFactor = 0.f;
            }

            // Force Jump Logic

            bool isMoving
                = (std::abs(movementSettings.mPosition[0]) > .5 || std::abs(movementSettings.mPosition[1]) > .5);
            if (!inwater && !flying)
            {
                // Force Jump
                if (stats.getMovementFlag(MWMechanics::CreatureStats::Flag_ForceJump))
                    movementSettings.mPosition[2] = onground ? 1.f : 0.f;
                // Force Move Jump, only jump if they're otherwise moving
                if (stats.getMovementFlag(MWMechanics::CreatureStats::Flag_ForceMoveJump) && isMoving)
                    movementSettings.mPosition[2] = onground ? 1.f : 0.f;
            }

            osg::Vec3f rot = cls.getRotationVector(mPtr);
            osg::Vec3f vec(movementSettings.asVec3());
            // Rush_Attack: the charge carries the player until its first blow lands; then the
            // movement input stops for the rest of the attack and friction halts the player at
            // the target, so the second blow reaches it (mmplayer_rushattack_friction).
            if (mBrutalCombo == BrutalCombo::Rush && mBrutalRushLanded)
                vec.x() = vec.y() = 0.f;
            if (kineticPlayer && !inwater && !flying)
            {
                const float jumpInput = vec.z();
                vec.z() = 0.f;
                movementSettings.mSpeedFactor = std::min(vec.length(), 1.f);
                vec.normalize();
                vec.z() = jumpInput;
            }
            else
            {
                movementSettings.mSpeedFactor = std::min(vec.length(), 1.f);
                vec.normalize();
            }

            const bool smoothMovement = Settings::game().mSmoothMovement && !kineticPlayer;
            if (smoothMovement)
            {
                float angle = mPtr.getRefData().getPosition().rot[2];
                osg::Vec2f targetSpeed
                    = Misc::rotateVec2f(osg::Vec2f(vec.x(), vec.y()), -angle) * movementSettings.mSpeedFactor;
                osg::Vec2f delta = targetSpeed - mSmoothedSpeed;
                float speedDelta = movementSettings.mSpeedFactor - mSmoothedSpeed.length();
                float deltaLen = delta.length();

                float maxDelta;
                if (isFirstPersonPlayer)
                    maxDelta = 1;
                else if (std::abs(speedDelta) < deltaLen / 2)
                    // Turning is smooth for player and less smooth for NPCs (otherwise NPC can miss a path point).
                    maxDelta = duration * (isPlayer ? 1.0f / Settings::game().mSmoothMovementPlayerTurningDelay : 6.f);
                else if (isPlayer && speedDelta < -deltaLen / 2)
                    // As soon as controls are released, mwinput switches player from running to walking.
                    // So stopping should be instant for player, otherwise it causes a small twitch.
                    maxDelta = 1;
                else // In all other cases speeding up and stopping are smooth.
                    maxDelta = duration * 3.f;

                if (deltaLen > maxDelta)
                    delta *= maxDelta / deltaLen;
                mSmoothedSpeed += delta;

                osg::Vec2f newSpeed = Misc::rotateVec2f(mSmoothedSpeed, angle);
                movementSettings.mSpeedFactor = newSpeed.normalize();
                vec.x() = newSpeed.x();
                vec.y() = newSpeed.y();

                const float eps = 0.001f;
                if (movementSettings.mSpeedFactor < eps)
                {
                    movementSettings.mSpeedFactor = 0;
                    vec.x() = 0;
                    vec.y() = 1;
                }
                else if ((vec.y() < 0) != mIsMovingBackward)
                {
                    if (targetSpeed.length() < eps || (movementSettings.mPosition[1] < 0) == mIsMovingBackward)
                        vec.y() = mIsMovingBackward ? -eps : eps;
                }
                vec.normalize();
            }

            float effectiveRotation = rot.z();
            bool canMove = cls.getMaxSpeed(mPtr) > 0;
            const bool turnToMovementDirection = Settings::game().mTurnToMovementDirection;
            const bool isBiped = mPtr.getClass().isBipedal(mPtr);
            if (!isBiped || !turnToMovementDirection || isFirstPersonPlayer)
            {
                movementSettings.mIsStrafing = std::abs(vec.x()) > std::abs(vec.y()) * 2;
                stats.setSideMovementAngle(0);
            }
            else if (canMove)
            {
                float targetMovementAngle
                    = vec.y() >= 0 ? std::atan2(-vec.x(), vec.y()) : std::atan2(vec.x(), -vec.y());
                movementSettings.mIsStrafing = (stats.getDrawState() != MWMechanics::DrawState::Nothing || inwater)
                    && std::abs(targetMovementAngle) > osg::DegreesToRadians(60.0f);
                if (movementSettings.mIsStrafing)
                    targetMovementAngle = 0;
                float delta = targetMovementAngle - stats.getSideMovementAngle();
                float cosDelta = cosf(delta);

                if ((vec.y() < 0) == mIsMovingBackward)
                    movementSettings.mSpeedFactor
                        *= std::min(std::max(cosDelta, 0.f) + 0.3f, 1.f); // slow down when turn
                if (std::abs(delta) < osg::DegreesToRadians(20.0f))
                    mIsMovingBackward = vec.y() < 0;

                float maxDelta = osg::PIf * duration * (2.5f - cosDelta);
                delta = std::clamp(delta, -maxDelta, maxDelta);
                stats.setSideMovementAngle(stats.getSideMovementAngle() + delta);
                effectiveRotation += delta;
            }

            mAnimation->setLegsYawRadians(stats.getSideMovementAngle());
            if (stats.getDrawState() == MWMechanics::DrawState::Nothing || inwater)
                mAnimation->setUpperBodyYawRadians(stats.getSideMovementAngle() / 2);
            else
                mAnimation->setUpperBodyYawRadians(stats.getSideMovementAngle() / 4);
            if (smoothMovement && !isPlayer && !inwater)
                mAnimation->setUpperBodyYawRadians(mAnimation->getUpperBodyYawRadians() + mAnimation->getHeadYaw() / 2);

            speed = kineticPlayer ? cls.getMaxSpeed(mPtr) * movementSettings.mSpeedFactor
                                  : cls.getCurrentSpeed(mPtr);
            if (kineticPlayer && !inwater && !flying && cls.getMaxSpeed(mPtr) > 0.f)
            {
                // Dark Messiah picks a speed per pressed direction (slow backpedal, no
                // sideways sprint); vec is already the normalized local input.
                const int effective = gait != MWPhysics::PlayerKinetics::Gait_Stance
                    ? gait
                    : (isrunning ? MWPhysics::PlayerKinetics::Gait_Run : MWPhysics::PlayerKinetics::Gait_Walk);
                speed = MWPhysics::PlayerKinetics::directionalSpeed(effective, sneak, vec.x(), vec.y())
                    * movementSettings.mSpeedFactor;
            }
            mBrutalSprinting = (kineticPlayer || Settings::game().mBrutalSlide)
                && gait == MWPhysics::PlayerKinetics::Gait_Sprint && !sneak && !inwater && !flying && speed > 0.f
                && vec.y() > 0.3f && vec.x() * vec.x() <= 0.09f;
            vec.x() *= speed;
            vec.y() *= speed;
            // The physics channel retains the sprint's world momentum throughout a
            // slide. Movement input must neither steer it nor start a jumping attack.
            if (isPlayer && stats.mBrutalSlidePhase != 0)
            {
                vec = osg::Vec3f();
                mBrutalSprinting = false;
            }

            if (isKnockedOut() || isKnockedDown() || (isRecovery() && !isUpperBodyRecovery())
                || isScriptedAnimPlaying())
            {
                vec = osg::Vec3f();
                mBrutalSprinting = false;
            }

            CharacterState movestate = CharState_None;
            CharacterState idlestate = CharState_None;
            JumpingState jumpstate = JumpState_None;

            const MWWorld::Store<ESM::GameSetting>& gmst = world->getStore().get<ESM::GameSetting>();
            if (vec.x() != 0.f || vec.y() != 0.f)
            {
                // advance athletics
                if (isPlayer)
                {
                    if (inwater)
                    {
                        mSecondsOfSwimming += duration;
                        while (mSecondsOfSwimming > 1)
                        {
                            cls.skillUsageSucceeded(mPtr, ESM::Skill::Athletics, ESM::Skill::Athletics_SwimOneSecond);
                            mSecondsOfSwimming -= 1;
                        }
                    }
                    else if (isrunning && !sneak)
                    {
                        mSecondsOfRunning += duration;
                        while (mSecondsOfRunning > 1)
                        {
                            cls.skillUsageSucceeded(mPtr, ESM::Skill::Athletics, ESM::Skill::Athletics_RunOneSecond);
                            mSecondsOfRunning -= 1;
                        }
                    }
                }

                if (!godmode && !(isPlayer && Settings::game().mBrutalStamina))
                {
                    // reduce fatigue
                    float fatigueLoss = 0.f;
                    static const float fFatigueRunBase = gmst.find("fFatigueRunBase")->mValue.getFloat();
                    static const float fFatigueRunMult = gmst.find("fFatigueRunMult")->mValue.getFloat();
                    static const float fFatigueSwimWalkBase = gmst.find("fFatigueSwimWalkBase")->mValue.getFloat();
                    static const float fFatigueSwimRunBase = gmst.find("fFatigueSwimRunBase")->mValue.getFloat();
                    static const float fFatigueSwimWalkMult = gmst.find("fFatigueSwimWalkMult")->mValue.getFloat();
                    static const float fFatigueSwimRunMult = gmst.find("fFatigueSwimRunMult")->mValue.getFloat();
                    static const float fFatigueSneakBase = gmst.find("fFatigueSneakBase")->mValue.getFloat();
                    static const float fFatigueSneakMult = gmst.find("fFatigueSneakMult")->mValue.getFloat();

                    if (cls.getEncumbrance(mPtr) <= cls.getCapacity(mPtr))
                    {
                        const float encumbrance = cls.getNormalizedEncumbrance(mPtr);
                        if (sneak)
                            fatigueLoss = fFatigueSneakBase + encumbrance * fFatigueSneakMult;
                        else
                        {
                            if (inwater)
                            {
                                if (!isrunning)
                                    fatigueLoss = fFatigueSwimWalkBase + encumbrance * fFatigueSwimWalkMult;
                                else
                                    fatigueLoss = fFatigueSwimRunBase + encumbrance * fFatigueSwimRunMult;
                            }
                            else if (isrunning)
                                fatigueLoss = fFatigueRunBase + encumbrance * fFatigueRunMult;
                        }
                    }
                    fatigueLoss *= duration;
                    fatigueLoss *= movementSettings.mSpeedFactor;
                    DynamicStat<float> fatigue = cls.getCreatureStats(mPtr).getFatigue();
                    fatigue.setCurrent(fatigue.getCurrent() - fatigueLoss, fatigue.getCurrent() < 0);
                    cls.getCreatureStats(mPtr).setFatigue(fatigue);
                }
            }

            bool wasInJump = mInJump;
            mInJump = false;
            const float jumpHeight = cls.getJump(mPtr);
            if (jumpHeight <= 0.f || sneak || inwater || flying || !solid)
            {
                vec.z() = 0.f;
                // Following code might assign some vertical movement regardless, need to reset this manually
                // This is used for jumping detection
                movementSettings.mPosition[2] = 0;
            }

            if (!inwater && !flying && solid)
            {
                // In the air (either getting up —ascending part of jump— or falling).
                if (!onground)
                {
                    mInJump = true;
                    jumpstate = JumpState_InAir;

                    static const float fJumpMoveBase = gmst.find("fJumpMoveBase")->mValue.getFloat();
                    static const float fJumpMoveMult = gmst.find("fJumpMoveMult")->mValue.getFloat();
                    float factor = fJumpMoveBase
                        + fJumpMoveMult * mPtr.getClass().getSkill(mPtr, ESM::Skill::Acrobatics) / 100.f;
                    factor = kineticPlayer ? 1.f : std::min(1.f, factor);
                    vec.x() *= factor;
                    vec.y() *= factor;
                    vec.z() = 0.0f;
                }
                // Started a jump.
                else if (mJumpState != JumpState_InAir && vec.z() > 0.f)
                {
                    if (isPlayer && !wasInJump)
                        BrutalFootsteps::playJump(mPtr);
                    mInJump = true;
                    if (kineticPlayer || (vec.x() == 0 && vec.y() == 0))
                        vec.z() = jumpHeight;
                    else
                    {
                        osg::Vec3f lat(vec.x(), vec.y(), 0.0f);
                        lat.normalize();
                        vec = osg::Vec3f(lat.x(), lat.y(), 1.0f) * jumpHeight * 0.707f;
                    }
                }
            }

            if (!mInJump)
            {
                if (mJumpState == JumpState_InAir && !flying && solid && wasInJump)
                {
                    float height = cls.getCreatureStats(mPtr).land(isPlayer);
                    float healthLost = 0.f;
                    if (!inwater)
                        healthLost = getFallDamage(mPtr, height);

                    if (healthLost > 0.0f)
                    {
                        const float fatigueTerm = cls.getCreatureStats(mPtr).getFatigueTerm();

                        // inflict fall damages
                        if (!godmode)
                        {
                            DynamicStat<float> health = cls.getCreatureStats(mPtr).getHealth();
                            float realHealthLost = healthLost * (1.0f - 0.25f * fatigueTerm);
                            health.setCurrent(health.getCurrent() - realHealthLost);
                            cls.getCreatureStats(mPtr).setHealth(health);
                            sndMgr->playSound3D(mPtr, ESM::RefId::stringRefId("Health Damage"), 1.0f, 1.0f);
                            if (isPlayer)
                                MWBase::Environment::get().getWindowManager()->activateHitOverlay(realHealthLost);
                        }

                        const float acrobaticsSkill = cls.getSkill(mPtr, ESM::Skill::Acrobatics);
                        if (healthLost > (acrobaticsSkill * fatigueTerm))
                        {
                            if (!godmode)
                                cls.getCreatureStats(mPtr).setKnockedDown(true);
                        }
                        else
                        {
                            // report acrobatics progression
                            if (isPlayer)
                                cls.skillUsageSucceeded(mPtr, ESM::Skill::Acrobatics, ESM::Skill::Acrobatics_Fall);
                        }
                    }

                    if (onground && !inwater)
                        if (auto* blood = MWRender::BrutalBlood::get())
                        {
                            blood->queueStep(mPtr, true);
                            blood->queueStep(mPtr, false);
                        }

                    if (mPtr.getClass().isNpc()
                        && !BrutalFootsteps::play(
                            mPtr, true, true, static_cast<BrutalFootsteps::Gait>(isPlayer ? mBrutalStepGait : 0)))
                    {
                        std::string_view sound;
                        osg::Vec3f pos(mPtr.getRefData().getPosition().asVec3());
                        if (world->isUnderwater(mPtr.getCell(), pos) || world->isWalkingOnWater(mPtr))
                            sound = "DefaultLandWater";
                        else if (onground)
                            sound = "DefaultLand";

                        if (!sound.empty())
                            sndMgr->playSound3D(mPtr, ESM::RefId::stringRefId(sound), 1.f, 1.f, MWSound::Type::Foot,
                                MWSound::PlayMode::NoPlayerLocal);
                    }
                }

                if (mAnimation->isPlaying(mCurrentJump))
                    jumpstate = JumpState_Landing;

                vec.z() = 0.0f;

                if (movementSettings.mIsStrafing)
                {
                    if (vec.x() > 0.0f)
                        movestate = (inwater ? (isrunning ? CharState_SwimRunRight : CharState_SwimWalkRight)
                                             : (sneak ? CharState_SneakRight
                                                      : (isrunning ? CharState_RunRight : CharState_WalkRight)));
                    else if (vec.x() < 0.0f)
                        movestate = (inwater
                                ? (isrunning ? CharState_SwimRunLeft : CharState_SwimWalkLeft)
                                : (sneak ? CharState_SneakLeft : (isrunning ? CharState_RunLeft : CharState_WalkLeft)));
                }
                else if (vec.length2() > 0.0f)
                {
                    if (vec.y() >= 0.0f)
                        movestate = (inwater ? (isrunning ? CharState_SwimRunForward : CharState_SwimWalkForward)
                                             : (sneak ? CharState_SneakForward
                                                      : (isrunning ? CharState_RunForward : CharState_WalkForward)));
                    else
                        movestate = (inwater
                                ? (isrunning ? CharState_SwimRunBack : CharState_SwimWalkBack)
                                : (sneak ? CharState_SneakBack : (isrunning ? CharState_RunBack : CharState_WalkBack)));
                }
                else
                {
                    // It seems only bipedal actors use turning animations.
                    // Also do not use turning animations in the first-person view and when sneaking.
                    if (!sneak && !isFirstPersonPlayer && isBiped)
                    {
                        if (effectiveRotation > 0.f)
                            movestate = inwater ? CharState_SwimTurnRight : CharState_TurnRight;
                        else if (effectiveRotation < 0.f)
                            movestate = inwater ? CharState_SwimTurnLeft : CharState_TurnLeft;
                    }
                }
            }

            if (turnToMovementDirection && !isFirstPersonPlayer && isBiped
                && (movestate == CharState_SwimRunForward || movestate == CharState_SwimWalkForward
                    || movestate == CharState_SwimRunBack || movestate == CharState_SwimWalkBack))
            {
                float swimmingPitch = mAnimation->getBodyPitchRadians();
                float targetSwimmingPitch = -mPtr.getRefData().getPosition().rot[0];
                float maxSwimPitchDelta = 3.0f * duration;
                swimmingPitch += std::clamp(targetSwimmingPitch - swimmingPitch, -maxSwimPitchDelta, maxSwimPitchDelta);
                mAnimation->setBodyPitchRadians(swimmingPitch);
            }
            else
                mAnimation->setBodyPitchRadians(0);

            if (inwater && isPlayer && !isFirstPersonPlayer && Settings::game().mSwimUpwardCorrection)
            {
                const float swimUpwardCoef = Settings::game().mSwimUpwardCoef;
                vec.z() = std::abs(vec.y()) * swimUpwardCoef;
                vec.y() *= std::sqrt(1.0f - swimUpwardCoef * swimUpwardCoef);
            }

            if (isBiped)
            {
                if (mTurnAnimationThreshold > 0)
                    mTurnAnimationThreshold -= duration;

                if (movestate == CharState_TurnRight || movestate == CharState_TurnLeft
                    || movestate == CharState_SwimTurnRight || movestate == CharState_SwimTurnLeft)
                {
                    mTurnAnimationThreshold = 0.05f;
                }
                else if (movestate == CharState_None && isTurning() && mTurnAnimationThreshold > 0)
                {
                    movestate = mMovementState;
                }
            }

            if (movestate != CharState_None)
            {
                clearAnimQueue();
                jumpstate = JumpState_None;
            }

            if (isPlayer)
                updateBrutalPlayerSteps(duration, movestate, !mInJump && onground && !inwater && !flying);

            updateAnimQueue();
            if (!mAnimQueue.empty())
                idlestate = CharState_SpecialIdle;
            else if (sneak && !mInJump)
                idlestate = CharState_IdleSneak;
            else
                idlestate = CharState_Idle;

            if (inwater)
                idlestate = CharState_IdleSwim;

            if (!mSkipAnim)
            {
                // A paired sprint frees the left hand. Keep its displayed pose until
                // the next gait or idle is ready to blend it back onto the grip.
                std::optional<MWRender::Animation::ScopedDeferActiveGroupReset> sprintReset;
                const bool sprinting = mBrutalSprinting && movestate == CharState_RunForward;
                if (sprinting != mCurrentMovement.starts_with("brutal_sprint") && usesBrutalPairedWeapon())
                    sprintReset.emplace(*mAnimation);
                if (mBrutalSprinting != mBrutalSprintAnim && movestate == CharState_RunForward)
                    refreshMovementAnims(movestate, true);
                refreshCurrentAnims(idlestate, movestate, jumpstate, updateWeaponState());
                sprintReset.reset();
                updateBrutalShieldBlock();
                updateIdleStormState(inwater);
            }

            if (isTurning())
            {
                if (duration > 0)
                {
                    float turnSpeed = std::min(1.5f, std::abs(rot.z()) / duration / static_cast<float>(osg::PI));
                    mAnimation->adjustSpeedMult(mCurrentMovement, turnSpeed);
                }
            }
            else if (mMovementState != CharState_None && mAdjustMovementAnimSpeed)
            {
                // Vanilla caps the played animation speed.
                const float maxSpeedMult = 10.f;
                const float speedMult = speed / mMovementAnimSpeed;
                mAnimation->adjustSpeedMult(mCurrentMovement, std::min(maxSpeedMult, speedMult));
                // Make sure the actual speed is the "expected" speed even though the animation is slower
                if (isMovementAnimationControlled())
                    scale *= std::max(1.f, speedMult / maxSpeedMult);
            }

            if (!mSkipAnim)
            {
                if (isPlayer && (mGroundFinisher || mBackstab.mPhase == BrutalBackstabPhase::Executing))
                    rot = {};
                if (!isKnockedDown() && !isKnockedOut())
                {
                    if (rot != osg::Vec3f())
                        world->rotateObject(mPtr, rot, true);
                }
                else // avoid z-rotating for knockdown
                {
                    if (rot.x() != 0 && rot.y() != 0)
                    {
                        rot.z() = 0.0f;
                        world->rotateObject(mPtr, rot, true);
                    }
                }

                updateHeadTracking(duration);
            }

            movement = vec;
            movementSettings.mPosition[0] = movementSettings.mPosition[1] = 0;

            // Can't reset jump state (mPosition[2]) here in full; we don't know for sure whether the PhysicsSystem will
            // actually handle it in this frame due to the fixed minimum timestep used for the physics update. It will
            // be reset in PhysicsSystem::move once the jump is handled.
            if (movement.z() == 0.f)
                movementSettings.mPosition[2] = 0;
        }
        else if (cls.getCreatureStats(mPtr).isDead())
        {
            // initial start of death animation for actors that started the game as dead
            // not done in constructor since we need to give scripts a chance to set the mSkipAnim flag
            if (!mSkipAnim && mDeathState != CharState_None && mCurrentDeath.empty())
            {
                // Fast-forward death animation to end for persisting corpses or corpses after end of death animation
                if (cls.isPersistent(mPtr) || cls.getCreatureStats(mPtr).isDeathAnimationFinished())
                    playDeath(1.f, mDeathState);
            }
        }

        // Sample the weapon clock across this frame. A seek or an interrupted
        // attack must never move the player through a skipped section.
        const bool spearStep = mPtr == getPlayer() && mWeaponType == ESM::Weapon::SpearTwoWide
            && !mSkipAnim && duration > 0.f && world->isFirstPerson() && brutalDmFirstPerson()
            && (mBrutalFast == BrutalFast::Attack || mUpperBodyState == UpperBodyState::AttackRelease)
            && world->isOnGround(mPtr) && !world->isSwimming(mPtr) && !world->isFlying(mPtr)
            && !isKnockedDown() && !isKnockedOut() && !isScriptedAnimPlaying();
        const float strideFrom = spearStep ? mAnimation->getCurrentTime(mCurrentWeapon) : -1.f;
        osg::Vec3f movementFromAnimation
            = mAnimation->runAnimation(mSkipAnim && !isScriptedAnimPlaying() ? 0.f : duration);

        if (mPtr.getClass().isActor() && !isScriptedAnimPlaying())
        {
            if (isMovementAnimationControlled())
            {
                if (duration != 0.f && movementFromAnimation != osg::Vec3f())
                {
                    movementFromAnimation /= duration;

                    // Ensure we're moving in the right general direction.
                    // In vanilla, all horizontal movement is taken from animations, even when moving diagonally (which
                    // doesn't have a corresponding animation). So to achieve diagonal movement, we have to rotate the
                    // movement taken from the animation to the intended direction.
                    //
                    // Note that while a complete movement animation cycle will have a well defined direction, no
                    // individual frame will, and therefore we have to determine the direction based on the currently
                    // playing cycle instead.
                    if (speed > 0.f)
                    {
                        float animMovementAngle = getAnimationMovementDirection();
                        float targetMovementAngle = std::atan2(-movement.x(), movement.y());
                        float diff = targetMovementAngle - animMovementAngle;
                        movementFromAnimation = osg::Quat(diff, osg::Vec3f(0, 0, 1)) * movementFromAnimation;
                    }

                    movement = movementFromAnimation;
                }
                else
                {
                    movement = osg::Vec3f();
                }
            }
            else if (mSkipAnim)
            {
                movement = osg::Vec3f();
            }

            if (mFloatToSurface && world->isSwimming(mPtr))
            {
                if (cls.getCreatureStats(mPtr).isDead()
                    || (!godmode
                        && cls.getCreatureStats(mPtr)
                                .getMagicEffects()
                                .getOrDefault(ESM::MagicEffect::Paralyze)
                                .getModifier()
                            > 0))
                {
                    movement.z() = 1.0;
                }
            }

            movement.x() *= scale;
            movement.y() *= scale;
            if (spearStep && strideFrom >= 0.f)
            {
                const float strideTo = mAnimation->getCurrentTime(mCurrentWeapon);
                const bool quick = mBrutalFast == BrutalFast::Attack;
                const std::string prefix = mCurrentWeapon + ": " + (quick ? mBrutalFastClip : "thrust");
                const float begin = mAnimation->getTextKeyTime(prefix + " plant");
                const float end = mAnimation->getTextKeyTime(prefix + " weight forward");
                if (begin >= 0.f && end > begin && strideTo > strideFrom && strideTo > begin && strideFrom < end)
                {
                    const auto weight = [=](float t) {
                        const float u = std::clamp((t - begin) / (end - begin), 0.f, 1.f);
                        return u * u * (3.f - 2.f * u);
                    };
                    // Normal actor locomotion resolves walls, other actors, and slopes.
                    movement.y() += (quick ? 18.f : 32.f) * (weight(strideTo) - weight(strideFrom)) / duration;
                }
            }
            if (mGroundFinisher)
            {
                movement = {};
                const float toImpact = mAnimation->getTextKeyTime(sGroundFinisher + ": hit")
                    - mAnimation->getCurrentTime(sGroundFinisher);
                if (toImpact > .06f && !mGroundFinisher->mVictim.ptrOrEmpty().isEmpty())
                {
                    const auto& feet = mPtr.getRefData().getPosition();
                    auto direction = mGroundFinisher->mContact - feet.asVec3();
                    direction.z() = 0.f;
                    const auto strike = mGroundFinisher->mStrikeOffset;
                    const float stanceDistance = strike ? std::clamp(std::hypot(strike->x(), strike->y()), 23.f, 74.f) : 42.f;
                    const float gap = direction.normalize() - stanceDistance;
                    if (std::abs(gap) > 2.f)
                    {
                        // Set the feet for the weapon's reach before the drive, on supported ground.
                        const auto ahead = feet.asVec3() + direction * std::clamp(gap, -18.f, 18.f);
                        const auto floor = world->getRayCasting()->castRay(ahead + osg::Z_AXIS * 14.f,
                            ahead - osg::Z_AXIS * 20.f, sFinisherGround);
                        if (floor.mHit && floor.mHitNormal.z() >= .65f)
                        {
                            const osg::Quat toLocal(feet.rot[2], osg::Z_AXIS);
                            movement = (toLocal * direction) * std::clamp(gap * 7.f, -78.f, 165.f);
                        }
                    }
                }
            }
            world->queueMovement(mPtr, movement);
        }

        mSkipAnim = false;

        updateBrutalWallSweep();
        mAnimation->enableHeadAnimation(cls.isActor() && !cls.getCreatureStats(mPtr).isDead());
    }

    void CharacterController::persistAnimationState() const
    {
        ESM::AnimationState& state = mPtr.getRefData().getAnimationState();

        state.mScriptedAnims.clear();
        for (AnimationQueue::const_iterator iter = mAnimQueue.begin(); iter != mAnimQueue.end(); ++iter)
        {
            // TODO: Probably want to presist lua animations too
            if (!iter->mScripted)
                continue;

            ESM::AnimationState::ScriptedAnimation anim;
            anim.mGroup = iter->mGroup;

            if (iter == mAnimQueue.begin() && mAnimation)
            {
                float complete;
                uint32_t loopcount;
                mAnimation->getInfo(anim.mGroup, &complete, nullptr, &loopcount);
                anim.mTime = complete;
                anim.mLoopCount = loopcount;
            }
            else
            {
                anim.mLoopCount = iter->mLoopCount;
                anim.mTime = 0.f;
            }

            state.mScriptedAnims.push_back(std::move(anim));
        }
    }

    void CharacterController::unpersistAnimationState()
    {
        const ESM::AnimationState& state = mPtr.getRefData().getAnimationState();

        if (!state.mScriptedAnims.empty())
        {
            clearAnimQueue();
            for (const ESM::AnimationState::ScriptedAnimation& animation : state.mScriptedAnims)
            {
                AnimationQueueEntry entry;
                entry.mGroup = animation.mGroup;
                entry.mLoopCount = static_cast<uint32_t>(
                    std::min<uint64_t>(animation.mLoopCount, std::numeric_limits<uint32_t>::max()));
                entry.mLooping = mAnimation->isLoopingAnimation(entry.mGroup);
                entry.mScripted = true;
                entry.mStartKey = "start";
                entry.mStopKey = "stop";
                entry.mSpeed = 1.f;
                entry.mTime = animation.mTime;
                if (animation.mAbsolute)
                {
                    float start = mAnimation->getTextKeyTime(animation.mGroup + ": start");
                    float stop = mAnimation->getTextKeyTime(animation.mGroup + ": stop");
                    float time = std::clamp(animation.mTime, start, stop);
                    entry.mTime = (time - start) / (stop - start);
                }

                mAnimQueue.push_back(std::move(entry));
            }

            playAnimQueue();
        }
    }

    void CharacterController::playBlendedAnimation(const std::string& groupname, const MWRender::AnimPriority& priority,
        int blendMask, bool autodisable, float speedmult, std::string_view start, std::string_view stop,
        float startpoint, uint32_t loops, bool loopfallback) const
    {
        if (mLuaAnimations)
            MWBase::Environment::get().getLuaManager()->playAnimation(mPtr, groupname, priority, blendMask, autodisable,
                speedmult, start, stop, startpoint, loops, loopfallback);
        else
            mAnimation->play(
                groupname, priority, blendMask, autodisable, speedmult, start, stop, startpoint, loops, loopfallback);
    }

    bool CharacterController::playGroup(std::string_view groupname, int mode, uint32_t count, bool scripted)
    {
        if (!mAnimation || !mAnimation->hasAnimation(groupname))
            return false;

        // We should not interrupt scripted animations with non-scripted ones
        if (isScriptedAnimPlaying() && !scripted)
            return true;

        bool looping = mAnimation->isLoopingAnimation(groupname);

        // If this animation is a looped animation that is already playing
        // and has not yet reached the end of the loop, allow it to continue animating with its existing loop count
        // and remove any other animations that were queued.
        // This emulates observed behavior from the original allows the script "OutsideBanner" to animate banners
        // correctly.
        if (!mAnimQueue.empty() && mAnimQueue.front().mGroup == groupname && looping
            && mAnimation->isPlaying(groupname))
        {
            float endOfLoop = mAnimation->getTextKeyTime(mAnimQueue.front().mGroup + ": loop stop");

            if (endOfLoop < 0) // if no Loop Stop key was found, use the Stop key
                endOfLoop = mAnimation->getTextKeyTime(mAnimQueue.front().mGroup + ": stop");

            if (endOfLoop > 0 && (mAnimation->getCurrentTime(mAnimQueue.front().mGroup) < endOfLoop))
            {
                mAnimQueue.resize(1);
                return true;
            }
        }

        // The loop count in vanilla is weird.
        // if played with a count of 0, all objects play exactly once from start to stop.
        // But if the count is x > 0, actors and non-actors behave differently. actors will loop
        // exactly x times, while non-actors will loop x+1 instead.
        if (mPtr.getClass().isActor() && count > 0)
            count--;

        AnimationQueueEntry entry;
        entry.mGroup = groupname;
        entry.mLoopCount = count;
        entry.mTime = 0.f;
        // "PlayGroup idle" is a special case, used to remove to stop scripted animations playing
        entry.mScripted = (scripted && groupname != "idle");
        entry.mLooping = looping;
        entry.mSpeed = 1.f;
        entry.mStartKey = ((mode == 2) ? "loop start" : "start");
        entry.mStopKey = "stop";

        bool playImmediately = false;

        if (mode != 0 || mAnimQueue.empty() || !isAnimPlaying(mAnimQueue.front().mGroup))
        {
            clearAnimQueue(scripted);

            playImmediately = true;
        }
        else
        {
            mAnimQueue.resize(1);
        }

        mAnimQueue.push_back(std::move(entry));

        if (playImmediately)
            playAnimQueue(mode == 2);

        return true;
    }

    bool CharacterController::playGroupLua(std::string_view groupname, float speed, std::string_view startKey,
        std::string_view stopKey, uint32_t loops, bool forceLoop)
    {
        // Note: In mwscript, "idle" is a special case used to clear the anim queue.
        // In lua we offer an explicit clear method instead so this method does not treat "idle" special.

        if (!mAnimation || !mAnimation->hasAnimation(groupname))
            return false;

        AnimationQueueEntry entry;
        entry.mGroup = groupname;
        // Note: MWScript gives one less loop to actors than non-actors.
        // But this is the Lua version. We don't need to reproduce this weirdness here.
        entry.mLoopCount = loops;
        entry.mStartKey = startKey;
        entry.mStopKey = stopKey;
        entry.mLooping = mAnimation->isLoopingAnimation(groupname) || forceLoop;
        entry.mScripted = true;
        entry.mSpeed = speed;
        entry.mTime = 0;

        if (mAnimQueue.size() > 1)
            mAnimQueue.resize(1);
        mAnimQueue.push_back(std::move(entry));

        if (mAnimQueue.size() == 1)
            playAnimQueue();

        return true;
    }

    void CharacterController::enableLuaAnimations(bool enable)
    {
        mLuaAnimations = enable;
    }

    void CharacterController::skipAnim()
    {
        mSkipAnim = true;
    }

    bool CharacterController::isScriptedAnimPlaying() const
    {
        // If the front of the anim queue is scripted, morrowind treats it as if it's
        // still playing even if it's actually done.
        if (!mAnimQueue.empty())
            return mAnimQueue.front().mScripted;

        return false;
    }

    bool CharacterController::isAnimPlaying(std::string_view groupName) const
    {
        if (mAnimation == nullptr)
            return false;
        return mAnimation->isPlaying(groupName);
    }

    bool CharacterController::isMovementAnimationControlled() const
    {
        if (mPtr == getPlayer() && Settings::game().mKineticPlayerMovement)
            return false;

        if ((mHitState != CharState_None && !isUpperBodyRecovery()) || mDeathState != CharState_None)
            return true;

        if (Settings::game().mPlayerMovementIgnoresAnimation && mPtr == getPlayer())
            return false;

        if (mInJump)
            return false;

        bool movementAnimationControlled = mIdleState != CharState_None;
        if (mMovementState != CharState_None)
            movementAnimationControlled = mMovementAnimationHasMovement;
        return movementAnimationControlled;
    }

    void CharacterController::clearAnimQueue(bool clearScriptedAnims)
    {
        // Do not interrupt scripted animations, if we want to keep them
        if (mAnimation && (!isScriptedAnimPlaying() || clearScriptedAnims) && !mAnimQueue.empty())
            mAnimation->disable(mAnimQueue.front().mGroup);

        if (clearScriptedAnims)
        {
            if (mAnimation)
                mAnimation->setPlayScriptedOnly(false);
            mAnimQueue.clear();
            return;
        }

        std::erase_if(mAnimQueue, [](const AnimationQueueEntry& entry) { return !entry.mScripted; });
    }

    bool CharacterController::hasPhysicalDeath() const
    {
        return Settings::game().mBrutalRagdoll && (mPtr != getPlayer() || brutalPlayerDeathScene())
            && (mPtr.getClass().isNpc() || MWPhysics::findCreatureRig(mPtr));
    }

    bool CharacterController::canSwordClinch() const
    {
        return mAnimation && mPtr.getClass().isNpc() && !isDead() && !mAnimation->isRagdollPose()
            && !isKnockedDown() && !isKnockedOut() && !isScriptedAnimPlaying()
            && mAnimation->hasAnimation("brutal_clinch")
            && mAnimation->hasAnimation("brutal_clinch_win")
            && mAnimation->hasAnimation("brutal_clinch_lose");
    }

    void CharacterController::playSwordClinch(std::string_view group, float speed)
    {
        if (!mAnimation)
            return;
        auto& stats = mPtr.getClass().getCreatureStats(mPtr);
        cancelGroundFinisher();
        cancelBrutalCombo();
        cancelBrutalSlide();
        if (!mCurrentWeapon.empty())
            mAnimation->disable(mCurrentWeapon);
        if (!mSwordClinchGroup.empty())
            mAnimation->disable(mSwordClinchGroup);
        if (!mBrutalNpcGuardGroup.empty())
            mAnimation->disable(mBrutalNpcGuardGroup);
        mBrutalNpcGuardGroup.clear();
        stats.mBrutalNpcGuardRequested = false;
        for (std::string_view old : { "brutal_block", "brutal_blockhit1", "brutal_blockhit2",
                 "brutal_blockhit3", "brutal_parrybreak" })
            mAnimation->disable(old);
        resetCurrentHitState();
        resetCurrentMovementState();
        resetCurrentJumpState();
        resetCurrentIdleState();
        mReadyToHit = false;
        mAttackVictim = MWWorld::Ptr();
        mAttackSuccess = false;
        mAttackStrength = -1.f;
        mCanCast = mCastingScriptedSpell = false;
        setAttackingOrSpell(false);
        mUpperBodyState = UpperBodyState::WeaponEquipped;
        mResetIdleOnAttackEnd = false;
        mBrutalBlock = BrutalBlock::None;
        stats.mPowerStrike = {};
        stats.mBrutalBlockRaised = stats.mBrutalParry = stats.mBrutalBlockImpact = false;
        stats.mBrutalParryResult = BrutalParry_None;
        stats.mBrutalParried = false;
        stats.mBrutalSlashRight = stats.mBrutalSlideAttack = false;
        stats.setHitRecovery(false);
        stats.setBlock(false);
        mSwordClinchGroup = group;
        // The session is the sole animation owner. Autodisable would expose an
        // armed idle for one frame between a completed clip and the next phase.
        const bool npcRecovery = mPtr != getPlayer() && group == "brutal_clinch_lose";
        mAnimation->play(mSwordClinchGroup, Priority_Knockdown, MWRender::BlendMask_All,
            npcRecovery, speed, "start", "stop", 0.f, 0);
        mAnimation->enableHeadAnimation(false);
    }

    bool CharacterController::updateSwordClinch(float duration)
    {
        if (mSwordClinchGroup.empty() || !mPtr.getClass().isActor())
            return false;
        auto& stats = mPtr.getClass().getCreatureStats(mPtr);
        if (!stats.mBrutalClinchActive)
            return false;
        auto world = MWBase::Environment::get().getWorld();
        auto& movement = mPtr.getClass().getMovementSettings(mPtr);
        std::fill(std::begin(movement.mPosition), std::end(movement.mPosition), 0.f);
        std::fill(std::begin(movement.mRotation), std::end(movement.mRotation), 0.f);
        movement.mSpeedFactor = 0.f;
        stats.setAttackingOrSpell(false);
        stats.mBrutalBlockRaised = stats.mBrutalParry = false;
        mAnimation->setLegsYawRadians(0.f);
        mAnimation->setUpperBodyYawRadians(0.f);
        mAnimation->setBodyPitchRadians(0.f);
        mAnimation->enableHeadAnimation(false);
        const osg::Vec3f rootMotion = mAnimation->runAnimation(std::max(duration, 0.f));
        const float yaw = mPtr.getRefData().getPosition().rot[2];
        osg::Vec3f velocity = osg::Quat(yaw, osg::Vec3f(0, 0, 1)) * stats.mBrutalClinchVelocity;
        // Preserve authored exit root motion through the usual collision solver.
        if (stats.mBrutalClinchPhase >= 3 && mPtr != getPlayer() && duration > 0.f)
            velocity += rootMotion / duration * mPtr.getCellRef().getScale();
        world->queueMovement(mPtr, velocity);
        mSkipAnim = false;
        return true;
    }

    void CharacterController::finishSwordClinch(bool preserveRecovery)
    {
        if (mSwordClinchGroup.empty())
            return;
        auto& stats = mPtr.getClass().getCreatureStats(mPtr);
        stats.mBrutalClinchBlendOut = .2f;
        if (preserveRecovery && mAnimation && mAnimation->isPlaying(mSwordClinchGroup))
        {
            // Continue the already-playing NPC loss clip as genuine full-body
            // recovery, without replaying its first frame when the player exits.
            mCurrentHit = mSwordClinchGroup;
            mHitState = CharState_Hit;
            stats.setHitRecovery(true);
        }
        else if (mAnimation)
            mAnimation->disable(mSwordClinchGroup);
        mSwordClinchGroup.clear();
        stats.setAttackingOrSpell(false);
        stats.mBrutalBlockLoweredAt = brutalSimulationTime();
        mBrutalComboRelease = true;
        mSmoothedSpeed = osg::Vec2f();
        if (mAnimation)
            mAnimation->enableHeadAnimation(!stats.isDead());
    }

    bool CharacterController::stagger(std::string_view preferred)
    {
        if (mPtr.getClass().isActor() && (mPtr.getClass().getCreatureStats(mPtr).mBrutalClinchActive
                || !mPtr.getClass().getCreatureStats(mPtr).mBrutalClinchOpponent.ptrOrEmpty().isEmpty()))
            return false; // The paired animation owns recovery until it releases this actor.
        if (!mAnimation || !mPtr.getClass().isActor() || mPtr == getPlayer()
            || mPtr.getClass().getCreatureStats(mPtr).isDead() || isDead()
            || mAnimation->isRagdollPose() || isKnockedDown() || isKnockedOut()
            || isScriptedAnimPlaying())
            return false;

        std::string group{ preferred };
        if (group.empty() || !mAnimation->hasAnimation(group))
            group = mPtr.getClass().isNpc() ? "brutal_hit" : "";
        if (!mAnimation->hasAnimation(group))
            group = chooseRandomGroup(hitStateToAnimGroup(CharState_Hit));
        // Some creatures only own a knockdown: that is still their reaction to a blow.
        if (!mAnimation->hasAnimation(group) && !mPtr.getClass().isNpc())
            group = "knockdown";
        if (!mAnimation->hasAnimation(group))
            return false;

        // Disable the source of weapon/cast text keys before clearing its pending result.
        // Recovery is controller state, not an overlay that leaves a hidden attack running.
        if (!mCurrentWeapon.empty())
            mAnimation->disable(mCurrentWeapon);
        mReadyToHit = false;
        mAttackVictim = MWWorld::Ptr();
        mAttackSuccess = false;
        mAttackStrength = -1.f;
        mCanCast = false;
        mCastingScriptedSpell = false;
        mBrutalFast = BrutalFast::None;
        mBrutalFastQueued = false;
        mBrutalFastIndex = 0;
        resetBrutalFastContinuation();
        setAttackingOrSpell(false);
        // The disabled weapon animation may be an equip or unequip that has not reached its
        // attach/detach key yet; that key will not come, so finish the change here.
        if (mUpperBodyState == UpperBodyState::Equipping)
        {
            mAnimation->showWeapons(true);
            if (mAnimation->useShieldAnimations())
            {
                mAnimation->disable("shield");
                mAnimation->showCarriedLeft(updateCarriedLeftVisible(mWeaponType));
            }
        }
        if (mUpperBodyState == UpperBodyState::Unequipping)
        {
            mAnimation->showWeapons(false);
            mUpperBodyState = UpperBodyState::None;
            mWeaponType = ESM::Weapon::None;
        }
        else if (mUpperBodyState != UpperBodyState::None)
            mUpperBodyState = UpperBodyState::WeaponEquipped;
        mResetIdleOnAttackEnd = false;

        resetCurrentHitState();
        auto& stats = mPtr.getClass().getCreatureStats(mPtr);
        if (!mBrutalNpcGuardGroup.empty())
            mAnimation->disable(mBrutalNpcGuardGroup);
        mBrutalNpcGuardGroup.clear();
        stats.mBrutalNpcGuardRequested = stats.mBrutalBlockRaised = stats.mBrutalParry = false;
        stats.mBrutalFastPressure = 0;
        stats.mBrutalGuardBreakingAttack = false;
        stats.mBrutalGuardCooldownUntil = std::max(stats.mBrutalGuardCooldownUntil, brutalSimulationTime() + .8);
        mBrutalBlock = BrutalBlock::None;
        stats.mBrutalSlashRight = false;
        stats.mBrutalSlideAttack = false;
        stats.setBlock(false);
        stats.setHitRecovery(true);
        mHitState = CharState_Hit;
        mCurrentHit = std::move(group);
        playBlendedAnimation(mCurrentHit, Priority_Knockdown, MWRender::BlendMask_All,
            true, 1.f, "start", "stop", 0.f, 0);
        return true;
    }

    namespace
    {
        constexpr std::string_view BrutalPickupGroup = "brutal_npc_pickup";
        constexpr std::string_view BrutalPickupTableGroup = "brutal_npc_pickup_table";
        // Higher than this over the feet, the item lies on a table, a crate or a shelf.
        constexpr float BrutalPickupTableHeight = 25.f;

        bool hasPickupClip(const MWRender::Animation& animation, std::string_view group)
        {
            return animation.hasAnimation(group)
                && animation.getTextKeyTime(std::string(group) + ": pickup contact") >= 0.f;
        }
    }

    bool CharacterController::canBrutalPickup() const
    {
        return mAnimation && mPtr.getClass().isNpc() && mPtr != getPlayer()
            && hasPickupClip(*mAnimation, BrutalPickupGroup);
    }

    bool CharacterController::hasClearBrutalPickupReach(const MWWorld::Ptr& item) const
    {
        if (item.isEmpty() || !item.isInCell() || !mPtr.isInCell())
            return false;
        const auto* actorCell = mPtr.getCell();
        const auto* itemCell = item.getCell();
        if (actorCell != itemCell && (!actorCell->isExterior() || !itemCell->isExterior()
                || actorCell->getCell()->getWorldSpace() != itemCell->getCell()->getWorldSpace()))
            return false;
        const osg::Vec3f actorPos = mPtr.getRefData().getPosition().asVec3();
        const osg::Vec3f itemPos = item.getRefData().getPosition().asVec3();
        const float height = itemPos.z() - actorPos.z();
        if (height <= -80.f || height >= 105.f || (itemPos - actorPos).length2() > 160.f * 160.f)
            return false;
        const auto world = MWBase::Environment::get().getWorld();
        const osg::Vec3f from = getMeleeAim(mPtr).first;
        const osg::Vec3f to = itemPos + osg::Vec3f(0.f, 0.f, 2.f);
        const auto* rays = world->getRayCasting();
        const auto obstruction = rays->castRay(from, to, { mPtr, item }, {}, sBackstabObstruction);
        if (!obstruction.mHit)
            return true;
        // Some loose items rest on a tabletop with their origin slightly below its surface.
        // Allow reaching just above that support, while still tracing against cover on the way.
        if (obstruction.mHitNormal.z() < .7f || (obstruction.mHitPos - itemPos).length2() > 8.f * 8.f)
            return false;
        const osg::Vec3f aboveSupport(itemPos.x(), itemPos.y(), obstruction.mHitPos.z() + 2.f);
        return !rays->castRay(from, aboveSupport, { mPtr, item }, {}, sBackstabObstruction).mHit;
    }

    bool CharacterController::isBrutalPickingUp() const
    {
        return mHitState != CharState_None && !mBrutalPickupGroup.empty() && mCurrentHit == mBrutalPickupGroup
            && mAnimation && mAnimation->isPlaying(mBrutalPickupGroup);
    }

    namespace
    {
        std::optional<osg::Vec3f> bowKickDirection(const MWWorld::Ptr& attacker, const MWWorld::Ptr& target,
            float reach)
        {
            const auto world = MWBase::Environment::get().getWorld();
            if (target.isEmpty() || !target.getRefData().isEnabled() || !target.getRefData().getBaseNode()
                || target.getClass().getCreatureStats(target).isDead()
                || !target.getClass().getCreatureStats(target).mBrutalClinchOpponent.ptrOrEmpty().isEmpty()
                || !isInMeleeReach(attacker, target, reach) || !world->isOnGround(attacker)
                || world->isSwimming(attacker) || world->isFlying(attacker))
                return std::nullopt;
            const auto& pos = attacker.getRefData().getPosition();
            osg::Vec3f away = target.getRefData().getPosition().asVec3() - pos.asVec3();
            away.z() = 0.f;
            const osg::Vec3f forward(std::sin(pos.rot[2]), std::cos(pos.rot[2]), 0.f);
            if (away.normalize() < .001f || forward * away < .5f || !world->getLOS(attacker, target)
                || isNpcMeleeOccluded(attacker, target, getMeleeTorsoPosition(target)))
                return std::nullopt;
            // Eye-level visibility can miss low cover between the kicking leg and its target.
            const auto hip = [&](const MWWorld::Ptr& actor) {
                return actor.getRefData().getPosition().asVec3()
                    + osg::Vec3f(0, 0, world->getHalfExtents(actor).z() * .85f);
            };
            if (world->getRayCasting()->castRay(hip(attacker), hip(target), { attacker, target }, {},
                    sBackstabObstruction).mHit)
                return std::nullopt;
            return away;
        }
    }

    bool CharacterController::isBrutalBowKicking() const
    {
        return mAnimation && mCurrentHit == sBowKick && mAnimation->isPlaying(sBowKick);
    }

    bool CharacterController::startBrutalBowKick(const MWWorld::Ptr& target)
    {
        if (!Settings::game().mDarkMessiahNpcCombat || !mPtr.getClass().isNpc() || mPtr == getPlayer()
            || !mAnimation || !mAnimation->hasAnimation(sBowKick) || mHitState != CharState_None
            || mUpperBodyState < UpperBodyState::WeaponEquipped || brutalSimulationTime() < mBrutalBowKickReadyAt)
            return false;
        const auto weaponClass = getWeaponType(mWeaponType)->mWeaponClass;
        if ((weaponClass != ESM::WeaponType::Ranged && weaponClass != ESM::WeaponType::Thrown)
            || mWeapon.isEmpty())
            return false;
        const auto& inventory = mPtr.getClass().getInventoryStore(mPtr);
        const auto slot = inventory.getSlot(MWWorld::InventoryStore::Slot_CarriedRight);
        if (slot == inventory.end() || *slot != mWeapon)
            return false;
        auto& stats = mPtr.getClass().getCreatureStats(mPtr);
        if (stats.isParalyzed() || stats.getKnockedDown() || stats.getHitRecovery()
            || stats.getDrawState() != DrawState::Weapon)
            return false;
        auto fatigue = stats.getFatigue();
        const float cost = std::max(1.f, fatigue.getBase() * .15f);
        const float minimumFatigue = std::max(cost, fatigue.getModified() * .30f);
        if (fatigue.getCurrent() < minimumFatigue || !bowKickDirection(mPtr, target, 28.f) || !stagger(sBowKick))
            return false;
        if (mWeaponType == ESM::Weapon::MarksmanBow)
            mAnimation->detachArrow();
        fatigue.setCurrent(fatigue.getCurrent() - cost);
        stats.setFatigue(fatigue);
        mBrutalBowKickTarget = MWWorld::SafePtr(target);
        mBrutalBowKickWeapon = MWWorld::SafePtr(mWeapon);
        mBrutalBowKickReadyAt = brutalSimulationTime() + 10.;
        return true;
    }

    void CharacterController::hitBrutalBowKick()
    {
        const auto& stats = mPtr.getClass().getCreatureStats(mPtr);
        const MWWorld::Ptr target = std::exchange(mBrutalBowKickTarget, {}).ptrOrEmpty();
        const MWWorld::Ptr weapon = std::exchange(mBrutalBowKickWeapon, {}).ptrOrEmpty();
        if (target.isEmpty() || weapon.isEmpty() || !isBrutalBowKicking() || stats.isDead()
            || stats.isParalyzed() || stats.getKnockedDown() || stats.getDrawState() != DrawState::Weapon)
            return;
        const auto& inventory = mPtr.getClass().getInventoryStore(mPtr);
        const auto slot = inventory.getSlot(MWWorld::InventoryStore::Slot_CarriedRight);
        if (slot == inventory.end() || *slot != weapon)
            return;
        const auto away = bowKickDirection(mPtr, target, 46.f);
        if (!away)
            return;
        auto& env = MWBase::Environment::get();
        if (target == getPlayer() && env.getWorld()->getGodModeState())
            return;
        env.getWorld()->addActorExternalVelocity(target, *away * 250.f);
        if (Settings::game().mBrutalCombatSounds)
            env.getSoundManager()->playSound3D(target, ESM::RefId::stringRefId("Brutal Kick Hit"), 1.f, 1.f);
        env.getLuaManager()->onHit(mPtr, target, {}, {}, -1, 1.f, 5.f, true,
            getMeleeTorsoPosition(target), true, DamageSourceType::Melee);
    }

    bool CharacterController::startBrutalPickup(const MWWorld::Ptr& item)
    {
        if (!canBrutalPickup() || !hasClearBrutalPickupReach(item))
            return false;
        const float height = item.getRefData().getPosition().pos[2] - mPtr.getRefData().getPosition().pos[2];
        const std::string_view group = height > BrutalPickupTableHeight && hasPickupClip(*mAnimation, BrutalPickupTableGroup)
            ? BrutalPickupTableGroup
            : BrutalPickupGroup;
        if (!stagger(group))
            return false;
        mBrutalPickupGroup = group;
        mBrutalPickupItem = MWWorld::SafePtr(item);
        mPtr.getClass().getCreatureStats(mPtr).mBrutalPickupTable = group == BrutalPickupTableGroup;
        Log(Debug::Info) << "Brutal pickup: " << mPtr.getCellRef().getRefId() << " plays " << group << " for "
                         << item.getCellRef().getRefId() << " " << height << " units over its feet";
        return true;
    }

    void CharacterController::updateBrutalPickup()
    {
        if (mBrutalPickupItem.ptrOrEmpty().isEmpty())
            return;
        auto& stats = mPtr.getClass().getCreatureStats(mPtr);
        if (!isBrutalPickingUp() || stats.isDead() || stats.isParalyzed() || stats.getKnockedDown())
        {
            mBrutalPickupItem = {};
            return;
        }
        if (!Settings::game().mBrutalNpcWeaponPickup || !stats.getAiSequence().isInCombat())
        {
            mBrutalPickupItem = {};
            mBrutalPickupGroup.clear();
            resetCurrentHitState();
            stats.setHitRecovery(false);
            return;
        }
        const float contact = mAnimation->getTextKeyTime(mBrutalPickupGroup + ": pickup contact");
        if (mAnimation->getCurrentTime(mBrutalPickupGroup) < contact)
            return;
        const MWWorld::Ptr item = mBrutalPickupItem.ptrOrEmpty();
        mBrutalPickupItem = {};
        if (!takeBrutalPickup(item))
            Log(Debug::Verbose) << "Brutal pickup by " << mPtr.getCellRef().getRefId() << " found nothing to take";
    }

    bool CharacterController::takeBrutalPickup(const MWWorld::Ptr& item)
    {
        auto world = MWBase::Environment::get().getWorld();
        // The player may have taken or grabbed it, or it fell out of reach during the crouch.
        if (item.isEmpty() || !item.isInCell() || item.getCellRef().getCount() <= 0 || !item.getRefData().isEnabled()
            || !item.getRefData().getBaseNode() || world->getHeldObject() == item
            || !hasClearBrutalPickupReach(item))
            return false;
        const bool shield = item.getType() == ESM::Armor::sRecordId;
        if ((!shield && item.getType() != ESM::Weapon::sRecordId)
            || (item.getClass().hasItemHealth(item) && item.getClass().getItemHealth(item) <= 0))
            return false;
        auto& inventory = mPtr.getClass().getInventoryStore(mPtr);
        const MWWorld::ContainerStoreIterator taken
            = inventory.add(item, item.getCellRef().getCount(), false);
        world->deleteObject(item);
        if (taken == inventory.end())
            return false;
        const MWWorld::Ptr held = *taken;
        auto& stats = mPtr.getClass().getCreatureStats(mPtr);
        if (shield)
        {
            const auto right = inventory.getSlot(MWWorld::InventoryStore::Slot_CarriedRight);
            const bool twoHanded = right != inventory.end() && right->getType() == ESM::Weapon::sRecordId
                && getWeaponType(right->get<ESM::Weapon>()->mBase->mData.mType)->mFlags & ESM::WeaponType::TwoHanded;
            if (!twoHanded)
                inventory.equip(MWWorld::InventoryStore::Slot_CarriedLeft, taken);
        }
        else
        {
            if (getWeaponType(held.get<ESM::Weapon>()->mBase->mData.mType)->mFlags & ESM::WeaponType::TwoHanded)
                inventory.unequipSlot(MWWorld::InventoryStore::Slot_CarriedLeft);
            inventory.equip(MWWorld::InventoryStore::Slot_CarriedRight, taken);
            // Straight into the hand: adopt the weapon as drawn, with no unequip/equip clips
            // after the crouch, as the weapon->weapon path does for a changed type.
            stats.setDrawState(DrawState::Weapon);
            int weaptype = ESM::Weapon::None;
            const auto active = getActiveWeapon(mPtr, &weaptype);
            if (!mCurrentWeapon.empty())
                mAnimation->disable(mCurrentWeapon);
            mWeapon = active != inventory.end() ? *active : MWWorld::Ptr();
            mWeaponType = weaptype;
            mCurrentWeapon = getWeaponAnimation(weaptype);
            mAnimation->setWeaponGroup(
                mCurrentWeapon, getWeaponType(weaptype)->mWeaponClass == ESM::WeaponType::Ranged);
            mUpperBodyState = UpperBodyState::WeaponEquipped;
            mReadyToHit = false;
            setAttackingOrSpell(false);
            resetBrutalFastContinuation();
            mAnimation->showWeapons(true);
        }
        mAnimation->showCarriedLeft(updateCarriedLeftVisible(mWeaponType));
        const ESM::RefId& sound = held.getClass().getUpSoundId(held);
        if (!sound.empty())
            MWBase::Environment::get().getSoundManager()->playSound3D(mPtr, sound, 1.f, 1.f);
        ++stats.mBrutalPickups;
        Log(Debug::Info) << "Brutal pickup: " << mPtr.getCellRef().getRefId() << " took "
                         << held.getCellRef().getRefId() << (shield ? " (shield)" : "");
        return true;
    }

    void CharacterController::cancelAttackForKick()
    {
        // Dark Messiah lets a kick interrupt any sword phase. Disable the source of
        // weapon text keys before clearing its pending hit, as stagger does, so no
        // blow lands beneath the kick. Held attack input waits for the kick to end.
        if (!mAnimation || mPtr != getPlayer()
            || (!mGroundFinisher && mBrutalCombo == BrutalCombo::None && mBackstab.mPhase == BrutalBackstabPhase::None
                && (mCurrentWeapon.empty() || !mAnimation->getInfo(mCurrentWeapon))))
            return;
        cancelGroundFinisher();
        cancelBackstab(false);
        cancelBrutalCombo();
        if (!mCurrentWeapon.empty())
            mAnimation->disable(mCurrentWeapon);
        mReadyToHit = false;
        mBrutalOffhandType = -1;
        mPtr.getClass().getCreatureStats(mPtr).mBrutalSlashRight = false;
        mPtr.getClass().getCreatureStats(mPtr).mBrutalSlideAttack = false;
        mAttackVictim = MWWorld::Ptr();
        mAttackSuccess = false;
        mAttackStrength = -1.f;
        setAttackingOrSpell(false);
        if (mUpperBodyState != UpperBodyState::None)
            mUpperBodyState = UpperBodyState::WeaponEquipped;
        mResetIdleOnAttackEnd = false;
    }

    void CharacterController::cancelBrutalSlide()
    {
        auto& stats = mPtr.getClass().getCreatureStats(mPtr);
        stats.mBrutalSlideRequested = stats.mBrutalSliding = stats.mBrutalSlideLow = false;
        stats.mBrutalSlideDownhill = false;
        stats.mBrutalSlideGroundNormal = osg::Vec3f();
        stats.mBrutalSlideGroundPoint = osg::Vec3f();
        stats.mBrutalSlidePhase = 0;
        stats.mBrutalSlideBlend = 0.f;
        mBrutalSlideAge = mBrutalSlideExitAge = mBrutalSlideTravelTime = 0.f;
        mBrutalSlideStopped = false;
        if (mAnimation)
            mAnimation->disable("brutal_slide");
    }

    void CharacterController::updateBrutalSlide(float duration)
    {
        auto& stats = mPtr.getClass().getCreatureStats(mPtr);
        MWBase::World* world = MWBase::Environment::get().getWorld();
        const bool requested = std::exchange(stats.mBrutalSlideRequested, false);
        if ((stats.mBrutalSlidePhase == 1 || stats.mBrutalSlidePhase == 2) && !stats.mBrutalSliding)
        {
            if (stats.mBrutalSlideAttack)
                cancelAttackForKick();
            cancelBrutalSlide();
        }
        const bool able = Settings::game().mBrutalSlide && Settings::game().mBrutalEnabled && !stats.isDead() && !stats.isParalyzed()
            && !isKnockedDown() && !isKnockedOut() && !isScriptedAnimPlaying()
            && !world->isSwimming(mPtr) && !world->isFlying(mPtr);
        if (!able)
        {
            if (stats.mBrutalSlideAttack)
                cancelAttackForKick();
            cancelBrutalSlide();
            return;
        }
        static const std::string group = "brutal_slide";
        const auto position = mPtr.getRefData().getPosition().asVec3();
        if (requested && stats.mBrutalSlidePhase == 0
            && (mBrutalSprinting || (!Settings::game().mKineticPlayerMovement
                && stats.getStance(MWMechanics::CreatureStats::Stance_Run)
                && mPtr.getClass().getMovementSettings(mPtr).mPosition[1] > .3f)) && world->isOnGround(mPtr)
            && (mAnimation->hasAnimation(group) || !brutalDmFirstPerson())
            && !mAnimation->isPlaying("brutal_shieldbash")
            && !mAnimation->isPlaying("brutal_kick") && !mAnimation->isPlaying("brutal_kick1h")
            && !mAnimation->isPlaying("brutal_kick1h_low"))
        {
            cancelAttackForKick();
            cancelBrutalCombo();
            mBrutalComboRelease = true;
            mAnimation->disable("brutal_block");
            mBrutalBlock = BrutalBlock::None;
            stats.mBrutalBlockRaised = stats.mBrutalParry = false;
            stats.mBrutalSlidePhase = 1;
            stats.mBrutalSliding = stats.mBrutalSlideLow = true;
            stats.mBrutalSlideDownhill = false;
            stats.mBrutalSlideGroundNormal = osg::Vec3f();
            stats.mBrutalSlideGroundPoint = osg::Vec3f();
            mBrutalSlideAge = mBrutalSlideExitAge = mBrutalSlideTravelTime = 0.f;
            mBrutalSlideStopped = false;
            mBrutalSlideTravelStart = position;
            playBlendedAnimation(group, brutalSlidePriority(), MWRender::BlendMask_All, false, 1.f,
                "start", "loop stop", 0.f, std::numeric_limits<uint32_t>::max());
        }
        const float dt = std::max(duration, 0.f);
        if (stats.mBrutalSlidePhase == 0)
        {
            // A completed slide may have to remain crouched under an obstacle.
            // When clearance returns, raise the physical eye over the same exit
            // interval instead of snapping from the low eye to standing.
            stats.mBrutalSlideBlend = stats.mBrutalSlideBlocked ? 1.f
                : std::max(stats.mBrutalSlideBlend - dt / .36666667f, 0.f);
            return;
        }
        if (stats.mBrutalSlidePhase != 3)
        {
            const bool continueDownhill = stats.mBrutalSlideDownhill
                && mPtr.getClass().getMovementSettings(mPtr).mPosition[1] > .3f;
            // Entry and loop use the original Dishonored timing. A distance window
            // avoids treating repeated render positions as a physics collision.
            mBrutalSlideAge += dt;
            mBrutalSlideTravelTime += dt;
            if (mBrutalSlideTravelTime >= .12f)
            {
                osg::Vec3f travel = position - mBrutalSlideTravelStart;
                travel.z() = 0.f;
                mBrutalSlideStopped = mBrutalSlideStopped
                    || (mBrutalSlideAge > .34f
                        && travel.length() < (continueDownhill ? 5.f : 100.f) * mBrutalSlideTravelTime);
                mBrutalSlideTravelStart = position;
                mBrutalSlideTravelTime = 0.f;
            }
            stats.mBrutalSlideBlend = std::min(mBrutalSlideAge / (.33333334f), 1.f);
            if (mBrutalSlideAge >= .33333334f)
                stats.mBrutalSlidePhase = 2;
            const bool bladePreparing = stats.mBrutalSlideAttack
                && (mUpperBodyState == UpperBodyState::AttackWindUp
                    || mUpperBodyState == UpperBodyState::AttackRelease);
            const bool handContactWindow = mBrutalSlideAge < .95f && !stats.mPowerStrike.token.empty()
                && stats.mPowerStrike.weapon.ptrOrEmpty() == mWeapon && isBrutalOneHanded(mWeaponType);
            // A front target can stop travel before the held blade lands. Keep
            // that collision in the low pose through the strike. Only supported
            // downhill travel with forward held can extend the ordinary timeout.
            if ((mBrutalSlideAge >= .95f && !continueDownhill)
                || (mBrutalSlideStopped && !bladePreparing && !handContactWindow) || !world->isOnGround(mPtr))
            {
                stats.mBrutalSlidePhase = 3;
                stats.mBrutalSliding = false;
                stats.mBrutalSlideDownhill = false;
                // The low exit pose still follows fresh accepted ground support.
                mBrutalSlideExitAge = 0.f;
                mAnimation->disable(group);
                playBlendedAnimation(group, brutalSlidePriority(), MWRender::BlendMask_All, true, 1.f,
                    "out start", "stop", 0.f, 0);
            }
        }
        else
        {
            mBrutalSlideExitAge += dt;
            const float target = stats.mBrutalSlideBlocked ? 1.f
                : std::max(1.f - mBrutalSlideExitAge / .36666667f, 0.f);
            stats.mBrutalSlideBlend = std::max(target, stats.mBrutalSlideBlend - dt / .36666667f);
            if (mBrutalSlideExitAge >= .36666667f && !mAnimation->isPlaying(group))
            {
                const float retainedBlend = stats.mBrutalSlideBlend;
                cancelBrutalSlide();
                stats.mBrutalSlideBlend = retainedBlend;
            }
        }
    }

    bool CharacterController::updateBrutalCombo()
    {
        if (mPtr != getPlayer() || !Settings::game().mDarkMessiahComboAttacks || !brutalDmFirstPerson() || !mAnimation)
            return false;
        MWBase::World* world = MWBase::Environment::get().getWorld();
        const CreatureStats& stats = mPtr.getClass().getCreatureStats(mPtr);
        static const std::string rush = "brutal_rush";
        static const std::string fall = "brutal_fallattack";
        if (mBrutalComboRelease && !getAttackingOrSpell())
            mBrutalComboRelease = false;
        const bool onGround = world->isOnGround(mPtr);
        const bool swimming = world->isSwimming(mPtr);
        const bool oneHanded = isBrutalOneHanded(mWeaponType);
        const bool paired = usesBrutalPairedWeapon();
        const bool armed = stats.getDrawState() == DrawState::Weapon && (oneHanded || paired)
            && !stats.isDead() && !swimming && !world->isFlying(mPtr)
            && (mHitState == CharState_None || mHitState == CharState_Block);
        if (mBrutalCombo != BrutalCombo::None && !armed)
        {
            cancelBrutalCombo();
            return false;
        }
        // A new jump can start as the previous strike ends. Recheck combo
        // eligibility immediately, before the ordinary attack starts a windup.
        if ((mBrutalCombo == BrutalCombo::Rush && !mAnimation->isPlaying(rush))
            || (mBrutalCombo == BrutalCombo::FallStrike && !mAnimation->isPlaying(fall)))
            mBrutalCombo = BrutalCombo::None;
        const float z = mPtr.getRefData().getPosition().pos[2];
        switch (mBrutalCombo)
        {
            case BrutalCombo::None:
                if (mBrutalComboRelease && armed)
                    return true; // A held combo input cannot start an ordinary windup either.
                if (!armed || !getAttackingOrSpell() || mBrutalComboRelease
                    || mUpperBodyState != UpperBodyState::WeaponEquipped || isBrutalShieldBlocking()
                    || mAnimation->isPlaying("brutal_kick1h") || mAnimation->isPlaying("brutal_kick1h_low"))
                    return false;
                if (oneHanded && mBrutalSprinting && mAnimation->hasAnimation(rush))
                {
                    // Rush_Attack_Sword: attacking during the megasprint.
                    playBlendedAnimation(rush, Priority_Knockdown, MWRender::BlendMask_All, true, 1.f, "start",
                        "stop", 0.f, 0);
                    mBrutalCombo = BrutalCombo::Rush;
                    mBrutalRushLanded = false;
                }
                else if (!onGround && mAnimation->hasAnimation(fall))
                {
                    if (paired && !mAnimation->activeSourceHasTextKey(
                                      "brutal_fallattack: start", "brutal_fallattack: brutal paired grip"))
                        return false;
                    // Falling_Attack_In_Sword: attacking in the air raises the weapon over the head.
                    playBlendedAnimation(fall, Priority_Knockdown, MWRender::BlendMask_All, false, 1.f, "start",
                        "loop start", 0.f, 0);
                    mBrutalCombo = BrutalCombo::FallRaise;
                    mBrutalFallPeakZ = z;
                    mBrutalFallReleased = false;
                }
                else
                    return false;
                mBrutalComboRelease = true;
                world->breakInvisibility(mPtr);
                return true;
            case BrutalCombo::Rush:
            case BrutalCombo::FallStrike:
                return true;
            case BrutalCombo::FallRaise:
            case BrutalCombo::FallHold:
                mBrutalFallPeakZ = std::max(mBrutalFallPeakZ, z);
                // A queued button release finishes the minimum raise. Otherwise
                // only real ground contact commits the strike; unchanged height
                // across render frames is not evidence of landing.
                mBrutalFallReleased = mBrutalFallReleased || !getAttackingOrSpell();
                if (onGround || (mBrutalFallReleased
                        && (mBrutalCombo == BrutalCombo::FallHold || !mAnimation->isPlaying(fall))))
                {
                    // Falling_Attack_Out_Sword: explicit release or ground contact.
                    mAnimation->disable(fall);
                    playBlendedAnimation(fall, Priority_Knockdown, MWRender::BlendMask_All, true, 1.f, "out start",
                        "stop", 0.f, 0);
                    mBrutalCombo = BrutalCombo::FallStrike;
                }
                else if (mBrutalCombo == BrutalCombo::FallRaise && !mAnimation->isPlaying(fall))
                {
                    playBlendedAnimation(fall, Priority_Knockdown, MWRender::BlendMask_All, false, 1.f, "loop start",
                        "loop stop", 0.f, std::numeric_limits<uint32_t>::max());
                    mBrutalCombo = BrutalCombo::FallHold;
                }
                return true;
        }
        return false;
    }

    bool CharacterController::brutalFastAvailable() const
    {
        const auto& stats = mPtr.getClass().getCreatureStats(mPtr);
        return Settings::game().mBrutalFastAttacks && mPtr == getPlayer() && mAnimation && brutalDmFirstPerson()
            && (mCurrentWeapon == "weapononehand" || usesBrutalPairedWeapon()) && !stats.mBrutalSlideAttack
            && brutalActiveKey("fasta start");
    }

    void CharacterController::chooseBrutalChargedAttack()
    {
        // Spears thrust on either longitudinal step and standing still. Other sets use the
        // source _T and _B swings. Sideways stays the established mapping that the limb cuts rely on.
        auto& stats = mPtr.getClass().getCreatureStats(mPtr);
        const float* move = mPtr.getClass().getMovementSettings(mPtr).mPosition;
        mBrutalNeutralAttack = false;
        if (std::abs(move[1]) > std::abs(move[0]) + 0.2f)
            mAttackType = move[1] > 0.f && !brutalActiveKey("brutal spear grip") ? "chop" : "thrust";
        else if (std::abs(move[0]) > std::abs(move[1]) + 0.2f)
            mAttackType = "slash";
        else if (brutalActiveKey("brutal spear grip"))
            mAttackType = "thrust";
        else if (brutalActiveKey("neutral start"))
        {
            mBrutalNeutralAttack = true;
            // first_basic.qct: the sword's neutral swing is lateral (impactdirY +1, cutheadflag 1),
            // the dagger and staff ones come down and forward.
            const bool lateral = !isBrutalChainStaff() && !isBrutalDaggerSet();
            mAttackType = lateral ? "slash" : "chop";
            stats.mBrutalSlashRight = lateral;
            return;
        }
        stats.mBrutalSlashRight = mAttackType == "slash" && move[0] < -.1f
            && brutalActiveKey("slashright start");
    }

    bool CharacterController::brutalActiveKey(const std::string& key) const
    {
        // The staff and dagger sets load over the sword's, so a section the top set lacks must
        // not resolve to the sword's clip underneath.
        return mAnimation->activeSourceHasTextKey(mCurrentWeapon + ": chop start", mCurrentWeapon + ": " + key);
    }

    bool CharacterController::isBrutalDaggerSet() const
    {
        return brutalActiveKey("fastendb start") && brutalActiveKey("faste start");
    }

    bool CharacterController::isBrutalChainStaff() const
    {
        return brutalActiveKey("fastendb start") && !brutalActiveKey("faste start");
    }

    bool CharacterController::usesBrutalPairedWeapon() const
    {
        return mAnimation && mPtr == getPlayer() && brutalDmFirstPerson() && brutalPairedWeapon(mWeaponType)
            && brutalActiveKey("brutal paired grip");
    }

    bool CharacterController::isBrutalWallWeapon() const
    {
        return mWeaponType == ESM::Weapon::LongBladeOneHand || mWeaponType == ESM::Weapon::ShortBladeOneHand
            || usesBrutalPairedWeapon() || isBrutalChainStaff();
    }

    void CharacterController::startBrutalFastClip(const std::string& key, BrutalFast phase)
    {
        auto& stats = mPtr.getClass().getCreatureStats(mPtr);
        if (phase == BrutalFast::Attack)
        {
            mBrutalFastClip = key;
            // Announce the authored stroke before its impact, rather than
            // leaving defenders to react to the preceding combo's hit key.
            for (const auto& [name, kind] : std::array<std::pair<const char*, int>, 4>{ {
                     { "chop", 0 }, { "slash", 1 }, { "slashright", 1 }, { "thrust", 2 } } })
                if (brutalActiveKey(key + " " + name + " hit") || brutalActiveKey(key + " " + name + " hit left"))
                {
                    stats.mBrutalAttackKind = kind;
                    break;
                }
        }
        const bool sliding = stats.mBrutalSlidePhase != 0;
        MWRender::Animation::AnimPriority priority(Priority_Weapon);
        priority[MWRender::BoneGroup_LowerBody] = Priority_WeaponLowerBody;
        const float speed = phase == BrutalFast::Attack && usesBrutalPairedWeapon() ? .86f : 1.f;
        mAnimation->disable(mCurrentWeapon);
        playBlendedAnimation(mCurrentWeapon, priority,
            sliding ? MWRender::BlendMask_LeftArm | MWRender::BlendMask_RightArm : int(MWRender::BlendMask_All), false,
            speed, key + " start", key + " stop", 0.f, 0);
        mBrutalFast = phase;
    }

    void CharacterController::resetBrutalFastContinuation()
    {
        mBrutalFastGrace = 0.f;
        mBrutalFastResumeIndex = 0;
        mBrutalFastWindupIndex = 0;
        mBrutalSpearStrafe = 0;
    }

    int CharacterController::getBrutalSpearStrafe() const
    {
        if (mWeaponType != ESM::Weapon::SpearTwoWide)
            return 0;
        const auto* move = mPtr.getClass().getMovementSettings(mPtr).mPosition;
        if (std::abs(move[0]) <= std::abs(move[1]) + .2f)
            return 0;
        return move[0] < 0.f ? -1 : 1;
    }

    void CharacterController::startBrutalFastNext()
    {
        // Dark Messiah's attack chains:
        // sword A B C D E B C D E..., daggers A B C D E A..., staff A B C D A...
        const int last = brutalActiveKey("faste start") ? 5 : 4;
        const bool dagger = isBrutalDaggerSet();
        const std::string side = mBrutalSpearStrafe < 0 ? "faststrafeleft" : "faststraferight";
        const bool spearSweep = mBrutalSpearStrafe != 0 && brutalActiveKey(side + " start");
        const bool spear = mWeaponType == ESM::Weapon::SpearTwoWide && brutalActiveKey("fastloopa start");
        const int previous = spear && !spearSweep && mBrutalFastIndex == 0 ? mBrutalLastSpearJab : mBrutalFastIndex;
        int next = previous + 1;
        if (next > last)
            next = last == 5 && !dagger ? 2 : 1;
        const bool spearLoop = spear && next == 1 && previous == last;
        if (spear && !spearSweep)
            mBrutalLastSpearJab = next;
        mBrutalFastIndex = next;
        resetBrutalFastContinuation();
        mBrutalFastQueued = false;
        mReadyToHit = false;
        ++mBrutalWallSwing;
        mBrutalWallBodyHit = false;
        mAttackSuccess = false;
        mAttackVictim = MWWorld::Ptr();
        const char letter = static_cast<char>('a' + next - 1);
        const std::string attack = spearSweep ? side : spearLoop ? "fastloopa" : std::string("fast") + letter;
        startBrutalFastClip(attack, BrutalFast::Attack);
    }

    bool CharacterController::startBrutalChargedHold()
    {
        if (!brutalFastAvailable())
            return false;
        const std::string attack = getAttackAnimationType();
        if (!brutalActiveKey(attack + " hold start") || !brutalActiveKey(attack + " hold stop"))
            return false;
        const bool loop = brutalActiveKey(attack + " hold loop start")
            && brutalActiveKey(attack + " hold loop stop");
        MWRender::Animation::AnimPriority priority(Priority_Weapon);
        priority[MWRender::BoneGroup_LowerBody] = Priority_WeaponLowerBody;
        mAnimation->disable(mCurrentWeapon);
        playBlendedAnimation(mCurrentWeapon, priority, MWRender::BlendMask_All, false, 1.f,
            attack + " hold start", attack + " hold stop", 0.f,
            loop ? std::numeric_limits<uint32_t>::max() : 0, loop);
        mBrutalFast = BrutalFast::None;
        mBrutalFastIndex = 0;
        mBrutalFastQueued = false;
        mReadyToHit = false;
        mBrutalChargedHold = true;
        resetBrutalFastContinuation();
        mUpperBodyState = UpperBodyState::AttackWindUp;
        return true;
    }

    void CharacterController::updateBrutalFast(bool& animPlaying, float& complete)
    {
        if (mUpperBodyState != UpperBodyState::AttackEnd || mPtr != getPlayer()
            || mAnimation->isPlaying("brutal_kick1h") || mAnimation->isPlaying("brutal_kick1h_low"))
        {
            mBrutalFast = BrutalFast::None;
            return;
        }
        const bool button = getAttackingOrSpell();
        const bool pressed = button && !mBrutalFastButton;
        if (pressed && !mBrutalFastQueued)
            mBrutalSpearStrafe = getBrutalSpearStrafe();
        mBrutalFastButton = button;
        const float dt = MWBase::Environment::get().getFrameDuration();
        mBrutalFastHeld = pressed || !button ? 0.f : mBrutalFastHeld + dt;
        const bool dagger = isBrutalDaggerSet();
        const bool staff = isBrutalChainStaff();
        // Dark Messiah's mm_player_mintime_to_bigslash 400 (daggers 200) after its time
        // scale: .36 and .18 s.
        const bool charged = button && mBrutalFastHeld >= (dagger ? .18f : .36f);
        const bool finished = !animPlaying || complete >= 1.f;
        const std::string group = mCurrentWeapon + ": ";
        const char letter = static_cast<char>('a' + std::max(mBrutalFastIndex, 1) - 1);
        const std::string endKey = std::string("fastend") + (!staff && !dagger ? 'a' : letter);
        const bool hasEnd = (staff || dagger || mBrutalFastIndex == 1)
            && brutalActiveKey(endKey + " start");
        const auto refresh = [&] { animPlaying = mAnimation->getInfo(mCurrentWeapon, &complete); };
        const auto fastPriority = [] {
            MWRender::Animation::AnimPriority priority(Priority_Weapon);
            priority[MWRender::BoneGroup_LowerBody] = Priority_WeaponLowerBody;
            return priority;
        };
        const auto finishChain = [&] {
            mBrutalFast = BrutalFast::None;
            mBrutalFastIndex = 0;
            complete = 1.f;
        };

        switch (mBrutalFast)
        {
            case BrutalFast::Attack:
                if (pressed)
                    mBrutalFastQueued = true;
                // Let a dagger or one-handed spear finish its active stroke before entering a charge.
                if (charged && (finished || (!dagger && !brutalActiveKey("fastloopa start"))))
                {
                    mBrutalFastQueued = false;
                    if (staff)
                    {
                        if (hasEnd)
                        {
                            startBrutalFastClip(endKey, BrutalFast::Transition);
                            refresh();
                        }
                        else
                        {
                            mBrutalFast = BrutalFast::TransitionHold;
                            complete = 0.f;
                        }
                        return;
                    }
                    chooseBrutalChargedAttack();
                    if (dagger)
                    {
                        // State 0xa8: a dagger chain charges into the neutral swing whatever the keys.
                        mBrutalNeutralAttack = true;
                        mAttackType = "chop";
                        mPtr.getClass().getCreatureStats(mPtr).mBrutalSlashRight = false;
                    }
                    else if (startBrutalChargedHold())
                    {
                        // The sword enters its charged idle directly. Its MDL
                        // transition clips are not selected by the original handler.
                        refresh();
                        return;
                    }
                    startBrutalFastClip(std::string("trans") + letter, BrutalFast::Transition);
                    refresh();
                    return;
                }
                if (!finished)
                    return;
                if (mBrutalFastQueued)
                {
                    startBrutalFastNext();
                    refresh();
                }
                else
                {
                    // Keep the original recovery/idle and playback rate. For
                    // .35 seconds after this fast clip, a fresh click keeps the
                    // chain instead of being swallowed by its early end clip.
                    mBrutalFastGrace = .35f;
                    mBrutalFastResumeIndex = mBrutalFastIndex;
                    if (hasEnd)
                    {
                        startBrutalFastClip(endKey, BrutalFast::End);
                        refresh();
                    }
                    else
                        finishChain();
                }
                return;
            case BrutalFast::End:
            {
                if (pressed && mBrutalFastGrace > 0.f && mBrutalFastResumeIndex > 0)
                {
                    mBrutalFastIndex = mBrutalFastResumeIndex;
                    startBrutalFastNext();
                    refresh();
                    return;
                }
                // Outside the added allowance, keep the original restart window on the original end clip's clock.
                // Dagger C/D can append the separately authored left-idle/item
                // recovery; that must not postpone the existing .6 threshold.
                const float restart = brutalActiveKey(endKey + " restart")
                    ? mAnimation->getTextKeyTime(group + endKey + " restart") : -1.f;
                const float restartStop = brutalActiveKey(endKey + " restart stop")
                    ? mAnimation->getTextKeyTime(group + endKey + " restart stop") : -1.f;
                const float now = mAnimation->getCurrentTime(mCurrentWeapon);
                const bool canRestart = (restart >= 0.f ? now >= restart : complete >= .6f)
                    && (restartStop < 0.f || now < restartStop);
                if (pressed && animPlaying && canRestart)
                {
                    mBrutalFastIndex = 0;
                    startBrutalFastNext();
                    refresh();
                }
                else if (finished)
                    finishChain();
                return;
            }
            case BrutalFast::Transition:
            case BrutalFast::TransitionHold:
                if (staff)
                {
                    if (!button)
                    {
                        // Let go during the end clip: the chain simply ends with it.
                        if (mBrutalFast == BrutalFast::Transition && !finished)
                            mBrutalFast = BrutalFast::End;
                        else
                            finishChain();
                        return;
                    }
                    if (finished || mBrutalFast == BrutalFast::TransitionHold)
                    {
                        // Observed live: the end clip, then a fresh windup still under the button.
                        mBrutalFastIndex = 0;
                        chooseBrutalChargedAttack();
                        const std::string attack = getAttackAnimationType();
                        mAnimation->disable(mCurrentWeapon);
                        playBlendedAnimation(mCurrentWeapon, fastPriority(), MWRender::BlendMask_All, false, 1.f,
                            attack + " start", attack + " max attack", 0.f, 0);
                        mBrutalFast = BrutalFast::None;
                        mUpperBodyState = UpperBodyState::AttackWindUp;
                        refresh();
                    }
                    return;
                }
                if (mBrutalFast == BrutalFast::Transition && finished)
                {
                    if (button && startBrutalChargedHold())
                    {
                        refresh();
                        return;
                    }
                    mBrutalFast = BrutalFast::TransitionHold;
                }
                if (button)
                {
                    complete = 0.f; // Keep holding the pose: the state ends only at the release.
                    return;
                }
                {
                    // Release: the charged swing's out section, as from a full windup.
                    if (mBrutalFast == BrutalFast::TransitionHold && startGroundFinisher())
                    {
                        refresh();
                        return;
                    }
                    const std::string attack = getAttackAnimationType();
                    mAnimation->disable(mCurrentWeapon);
                    playBlendedAnimation(mCurrentWeapon, fastPriority(), MWRender::BlendMask_All, false, 1.f,
                        attack + " max attack", attack + " max attack", 0.f, 0);
                    mBrutalFast = BrutalFast::None;
                    mBrutalFastIndex = 0;
                    mUpperBodyState = UpperBodyState::AttackRelease;
                    prepareHit();
                    refresh();
                }
                return;
            case BrutalFast::None:
                return;
        }
    }

    void CharacterController::updateBrutalWallSweep()
    {
        auto* animation = dynamic_cast<MWRender::NpcAnimation*>(mAnimation);
        if (mPtr != getPlayer() || !animation)
            return;
        static constexpr std::string_view phases[] = { "none", "attack", "end", "transition", "transitionHold" };
        animation->recordWeaponCycleForAudit({ phases[static_cast<unsigned>(mBrutalFast)], mBrutalFastIndex,
            static_cast<int>(mUpperBodyState), mBrutalFastQueued, mBrutalFastButton, mReadyToHit, mBrutalFastHeld,
            mBrutalChargedHold, mBrutalGuardCharge, mBrutalUseHeld, mBrutalAcceptedStrikes });
        const auto& stats = mPtr.getClass().getCreatureStats(mPtr);
        const bool fast = mBrutalFast == BrutalFast::Attack;
        const std::string attack = fast ? mBrutalFastClip : getAttackAnimationType();
        // Every striking sword can meet a wall. The source has only three recoil
        // directions; quick slashes use their corresponding side, overhead/backward
        // strikes use the neutral recoil with the actual contact-plane response.
        const bool fastRight = attack == "faststrafeleft" || (attack != "faststraferight" && mBrutalFastIndex % 2 != 0);
        const std::string key = fast ? (fastRight ? "bouncer" : "bouncel")
            : attack == "slash" ? "bouncel" : attack == "slashright" ? "bouncer" : "bounce";
        const float time = mAnimation->getCurrentTime(mCurrentWeapon);
        const float start = mAnimation->getTextKeyTime(mCurrentWeapon + ": " + attack + (fast ? " start" : " max attack"));
        const float stop = mAnimation->getTextKeyTime(mCurrentWeapon + ": " + attack + (fast ? " stop" : " large follow stop"));
        const bool armed = Settings::game().mBrutalWallBounce && !key.empty() && brutalDmFirstPerson()
            && isBrutalWallWeapon() && brutalActiveKey(key + " start") && !stats.isDead() && !stats.mBrutalClinchActive
            && !stats.mBrutalSlideAttack && stats.mBrutalSlidePhase == 0 && !isScriptedAnimPlaying()
            && !mBrutalWallBodyHit && (fast || (mBrutalFast == BrutalFast::None && mAttackStrength >= 0.f))
            && (fast || mUpperBodyState == UpperBodyState::AttackRelease
                || mUpperBodyState == UpperBodyState::AttackEnd)
            && start >= 0.f && time >= start && stop > start && time < stop;
        animation->configureSwordWallSweep(mBrutalWallSwing, mCurrentWeapon, armed ? key : "");
    }

    bool CharacterController::brutalBounce()
    {
        // A receipt comes only from the evaluated visible blade meeting visible scenery.
        // Consume it outside text-key iteration: seeking there would fire intervening keys.
        auto* animation = dynamic_cast<MWRender::NpcAnimation*>(mAnimation);
        if (mPtr != getPlayer() || !animation)
            return false;
        std::string key;
        if (!animation->consumeSwordWallContact(mBrutalWallSwing, key))
            return false;
        const auto& stats = mPtr.getClass().getCreatureStats(mPtr);
        if (stats.isDead() || stats.mBrutalClinchActive || stats.mBrutalSlidePhase != 0 || !isBrutalWallWeapon()
            || mBrutalWallBodyHit
            || (mUpperBodyState != UpperBodyState::AttackRelease && mUpperBodyState != UpperBodyState::AttackEnd))
            return false;
        mReadyToHit = false;
        mAttackStrength = -1.f;
        mBrutalOffhandType = -1;
        mBrutalFast = BrutalFast::None;
        mBrutalFastQueued = false;
        mBrutalFastIndex = 0;
        mBrutalFastHeld = 0.f;
        resetBrutalFastContinuation();
        mPtr.getClass().getCreatureStats(mPtr).mBrutalSlashRight = false;
        mBrutalNeutralAttack = false;
        mUpperBodyState = UpperBodyState::AttackEnd;
        MWRender::Animation::AnimPriority priority(Priority_Weapon);
        priority[MWRender::BoneGroup_LowerBody] = Priority_WeaponLowerBody;
        mAnimation->disable(mCurrentWeapon);
        playBlendedAnimation(mCurrentWeapon, priority, MWRender::BlendMask_All, false, 1.f,
            key + " start", key + " stop", 0.f, 0);
        return true;
    }

    void CharacterController::brutalGuardAttack()
    {
        // Sword guard: a short click does nothing (with a shield the bash is
        // Lua's, on release); a long hold enters the charged idle before release.
        // Daggers and staff cannot attack while keeping the guard raised, even
        // on a long hold. Lowering it under a held button starts the ordinary windup.
        auto& stats = mPtr.getClass().getCreatureStats(mPtr);
        const bool raised = mBrutalBlock == BrutalBlock::Raising || mBrutalBlock == BrutalBlock::Holding;
        const bool button = getAttackingOrSpell();
        if (isBrutalChainStaff() || isBrutalDaggerSet() || !raised
            || mUpperBodyState != UpperBodyState::WeaponEquipped)
        {
            // A lowered guard under a held button starts the ordinary windup (live: parry_out,
            // then bigslash_in).
            mBrutalGuardPress = false;
            mBrutalGuardHeld = 0.f;
            return;
        }
        if (button)
        {
            mBrutalGuardPress = true;
            mBrutalGuardHeld = mBrutalUseHeld;
            if (mBrutalGuardHeld < .36f)
                return;
            // Source reads the current movement at charge eligibility, not
            // the direction of a short step made when Use was first pressed.
            chooseBrutalChargedAttack();
            if (startBrutalChargedHold())
            {
                mAnimation->disable("brutal_block");
                mBrutalBlock = BrutalBlock::None;
                stats.mBrutalBlockRaised = stats.mBrutalParry = false;
                stats.mBrutalBlockLoweredAt = brutalSimulationTime();
                mBrutalGuardCharge = true;
                mBrutalGuardPress = false;
                mBrutalGuardHeld = 0.f;
                mAttackSuccess = false;
                mAttackVictim = MWWorld::Ptr();
            }
            return;
        }
        if (!mBrutalGuardPress)
            return;
        const bool charged = mBrutalGuardHeld >= .36f;
        mBrutalGuardPress = false;
        mBrutalGuardHeld = 0.f;
        if (!charged)
            return;
        mAnimation->disable("brutal_block");
        mBrutalBlock = BrutalBlock::None;
        stats.mBrutalBlockRaised = stats.mBrutalParry = false;
        stats.mBrutalBlockLoweredAt = brutalSimulationTime();
        const std::string attack = getAttackAnimationType();
        mAnimation->disable(mCurrentWeapon);
        MWRender::Animation::AnimPriority priority(Priority_Weapon);
        priority[MWRender::BoneGroup_LowerBody] = Priority_WeaponLowerBody;
        playBlendedAnimation(mCurrentWeapon, priority, MWRender::BlendMask_All, false, 1.f,
            attack + " max attack", attack + " max attack", 0.f, 0);
        mAttackSuccess = false;
        mAttackVictim = MWWorld::Ptr();
        mUpperBodyState = UpperBodyState::AttackRelease;
        MWBase::Environment::get().getWorld()->breakInvisibility(mPtr);
        prepareHit();
    }

    void CharacterController::brutalFastHit(std::string_view action)
    {
        const MWWorld::Class& cls = mPtr.getClass();
        CreatureStats& stats = cls.getCreatureStats(mPtr);
        if (stats.isDead())
            return;
        const bool right = action.find(" slashright hit") != std::string_view::npos;
        const bool thrust = action.find(" thrust hit") != std::string_view::npos;
        const bool chop = action.find(" chop hit") != std::string_view::npos;
        mAttackType = thrust ? "thrust" : chop ? "chop" : "slash";
        stats.mBrutalAttackKind = thrust ? 2 : chop ? 0 : 1;
        stats.mBrutalSlashRight = right;
        stats.mBrutalSlideAttack = false;
        mBrutalNeutralAttack = false;
        mAttackStrength = 1.f;
        // Two daggers take turns: each click is one complete native hit with that dagger's own
        // target, damage, wear, enchantment and effects. One dagger always strikes with itself.
        stats.mBrutalOffhandStrike = action.ends_with(" left") && hasBrutalOffhandDagger();
        stats.mBrutalComboMultiplier = .7f / 1.5f; // first_basic.qct HitPow against the charged swing.
        stats.mBrutalFastStrike = true;
        ++mBrutalAcceptedStrikes;
        applyMeleeImpact(thrust ? ESM::Weapon::AT_Thrust : chop ? ESM::Weapon::AT_Chop : ESM::Weapon::AT_Slash);
        mBrutalWallBodyHit = !mAttackVictim.isEmpty();
        stats.mBrutalFastStrike = false;
        stats.mBrutalComboMultiplier = 1.f;
        stats.mBrutalOffhandStrike = false;
        stats.mBrutalSlashRight = false;
        mAttackVictim = MWWorld::Ptr();
        mAttackSuccess = false;
        mAttackStrength = -1.f;
    }

    void CharacterController::cancelBrutalCombo()
    {
        if (mBrutalCombo == BrutalCombo::None)
            return;
        mAnimation->disable("brutal_rush");
        mAnimation->disable("brutal_fallattack");
        mBrutalCombo = BrutalCombo::None;
        mBrutalFallReleased = false;
    }

    void CharacterController::brutalComboHit(float ratio, float push, bool falling)
    {
        const MWWorld::Class& cls = mPtr.getClass();
        CreatureStats& stats = cls.getCreatureStats(mPtr);
        if (stats.isDead())
            return;
        // The native melee hit: target choice, blocking, blood, receipts and adrenaline stay
        // those of an ordinary full-strength swing; only the damage ratio differs.
        // The falling attack strikes down (impactdir Z -1); the rush's two diagonal strokes
        // (impactdir X1 Y-1, X1 Y1) cover both the horizontal and the vertical swing area.
        mAttackType = falling ? "chop" : "any";
        stats.mBrutalSlashRight = false;
        stats.mBrutalSlideAttack = false;
        mAttackStrength = 1.f;
        stats.mBrutalComboMultiplier = ratio;
        const auto& inv = cls.getInventoryStore(mPtr);
        const auto shield = inv.getSlot(MWWorld::InventoryStore::Slot_CarriedLeft);
        stats.mBrutalShieldRush = !falling && shield != inv.end() && shield->getType() == ESM::Armor::sRecordId
            && updateCarriedLeftVisible(mWeaponType);
        applyMeleeImpact(falling ? ESM::Weapon::AT_Chop : ESM::Weapon::AT_Slash);
        stats.mBrutalShieldRush = false;
        stats.mBrutalComboMultiplier = 1.f;
        const MWWorld::Ptr victim = mAttackVictim;
        const bool success = mAttackSuccess;
        mAttackVictim = MWWorld::Ptr();
        mAttackSuccess = false;
        mAttackStrength = -1.f;
        if (success && !falling && !mBrutalRushLanded)
        {
            // The charge ends at the target (pushplayercoeff): the player's momentum stops on
            // contact instead of sliding past the target before the second blow.
            mBrutalRushLanded = true;
            MWBase::Environment::get().getWorld()->clearActorExternalVelocity(mPtr);
        }
        if (!success || victim.isEmpty() || !victim.getClass().isActor()
            || victim.getClass().getCreatureStats(victim).isDead())
            return;
        if (push <= 0.f)
            return;
        // Pushing strikes. The falling attack knocks the target down (chancetofall 0.6) only after a
        // drop of PlayerVerticalAttackHeightToMakeNPCFall (50 inches).
        osg::Vec3f direction = victim.getRefData().getPosition().asVec3() - mPtr.getRefData().getPosition().asVec3();
        direction.z() = 0.f;
        if (direction.normalize() < 1e-3f)
            return;
        constexpr float inch = 1.84265547703229f;
        MWBase::World* world = MWBase::Environment::get().getWorld();
        const float drop = mBrutalFallPeakZ - mPtr.getRefData().getPosition().pos[2];
        const bool knockDown
            = falling && drop >= 50.f * inch && Misc::Rng::rollClosedProbability(world->getPrng()) < .6f;
        world->applyNpcKick(victim, direction * push + osg::Vec3f(0, 0, knockDown ? push * .35f : 0.f), mPtr,
            knockDown);
    }

    void CharacterController::updateBrutalPlayerSteps(float duration, CharacterState movestate, bool grounded)
    {
        // Dark Messiah's player legs (player_animations.mdl) play Walk, run, sprint and Crouch
        // at their own rate, whatever the arms do, with footstep events 2124 (right) and 2123
        // (left) at fixed points of each cycle.
        using Gait = BrutalFootsteps::Gait;
        Gait gait;
        float cycle = 1.f;
        switch (movestate)
        {
            case CharState_WalkForward:
            case CharState_WalkBack:
            case CharState_WalkLeft:
            case CharState_WalkRight:
                gait = Gait::Walk;
                break;
            case CharState_RunForward:
                gait = mBrutalSprinting ? Gait::Sprint : Gait::Run;
                cycle = 20.f / 24.f; // a_RunN and a_sprintN are 21 frames at 24 fps
                break;
            case CharState_RunBack:
            case CharState_RunLeft:
            case CharState_RunRight:
                gait = Gait::Run;
                break;
            case CharState_SneakForward:
            case CharState_SneakBack:
            case CharState_SneakLeft:
            case CharState_SneakRight:
                gait = Gait::Stealth;
                break;
            default:
                mBrutalStepPhase = 0.f;
                return;
        }
        if (!grounded || duration <= 0.f)
        {
            mBrutalStepPhase = 0.f;
            return;
        }
        mBrutalStepGait = static_cast<int>(gait);
        const float right = gait == Gait::Walk ? .292f : .333f;
        const float left = gait == Gait::Walk ? .792f : .75f;
        auto* blood = MWRender::BrutalBlood::get();
        const auto contact = [&](bool isLeft) {
            if (blood)
                blood->queueStep(mPtr, isLeft);
            BrutalFootsteps::play(mPtr, isLeft, false, gait);
        };
        float from = mBrutalStepPhase;
        float to = from + duration / cycle;
        while (true)
        {
            const float end = std::min(to, 1.f);
            if (from < right && right <= end)
                contact(false);
            if (from < left && left <= end)
                contact(true);
            if (to < 1.f)
                break;
            from = 0.f;
            to -= 1.f;
        }
        mBrutalStepPhase = to;
    }

    void CharacterController::updateBrutalNpcGuardTarget(bool enabled, bool shield)
    {
        auto* npc = dynamic_cast<MWRender::NpcAnimation*>(mAnimation);
        if (!npc)
            return;
        const auto disable = [&] { npc->setNpcGuardTarget(false, shield, {}, {}); };
        const auto opponent = getPlayer();
        if (!enabled || opponent.isEmpty() || !opponent.isInCell()
            || opponent.getClass().getCreatureStats(opponent).isDead())
        {
            disable();
            return;
        }
        auto world = MWBase::Environment::get().getWorld();
        const auto [origin, aim] = getMeleeAim(opponent);
        const auto base = mPtr.getRefData().getPosition().asVec3();
        const auto half = world->getHalfExtents(mPtr, true);
        osg::Vec3f outward = origin - base;
        outward.z() = 0.f;
        if (outward.normalize() < .001f)
        {
            disable();
            return;
        }
        const osg::Vec3f facing = osg::Quat(mPtr.getRefData().getPosition().rot[2], osg::Vec3f(0, 0, -1))
            * osg::Vec3f(0, 1, 0);
        if (outward * facing < std::cos(osg::DegreesToRadians(70.f)))
        {
            disable();
            return;
        }
        const osg::Vec3f right = outward ^ osg::Vec3f(0, 0, 1);
        // The plane belongs to the torso, not the moving guard item. A raised
        // shield cannot pull its own goal ever closer with every blocked hit.
        const osg::Vec3f front = base + outward * (half.y() * .65f);
        osg::Vec3f point = front + osg::Vec3f(0, 0, half.z() * 1.35f);
        const float denominator = aim * outward;
        if (denominator < -.05f)
        {
            const float along = ((front - origin) * outward) / denominator;
            if (along > 0.f)
                point = origin + aim * along;
        }
        const float lateral = std::clamp((point - front) * right, -half.x() * 1.15f, half.x() * 1.15f);
        float top = base.z() + 2.f * half.z();
        if (const auto head = mAnimation->getBoneWorldMatrix("Bip01 Head"))
            top = std::max(static_cast<float>(head->getTrans().z()) + 13.f, base.z() + half.z() * 1.65f);
        point = front + right * lateral
            + osg::Vec3f(0, 0, std::clamp(point.z(), base.z() + 6.f, top) - base.z());

        constexpr int mask = MWPhysics::CollisionType_World | MWPhysics::CollisionType_Door
            | MWPhysics::CollisionType_HeightMap | MWPhysics::CollisionType_Dynamic | MWPhysics::CollisionType_Actor;
        const auto cover = world->getRayCasting()->castRay(origin, point, { opponent, mPtr }, {}, mask);
        if (cover.mHit && (cover.mHitPos - origin).length() + .5f < (point - origin).length())
        {
            disable();
            return;
        }
        osg::Vec3f incoming = point - origin;
        if (incoming.normalize() < .001f)
        {
            disable();
            return;
        }
        osg::Vec3f aimRight = aim ^ osg::Vec3f(0, 0, 1);
        if (aimRight.normalize() < .001f)
            aimRight = right;
        const osg::Vec3f aimUp = aimRight ^ aim;
        const int kind = opponent.getClass().getCreatureStats(opponent).mBrutalAttackKind;
        const osg::Vec3f stroke = kind == 0 ? -aimUp : kind == 1 ? aimRight : osg::Vec3f();
        npc->setNpcGuardTarget(true, shield, point, incoming, stroke);
    }

    void CharacterController::updateBrutalNpcGuard()
    {
        auto& stats = mPtr.getClass().getCreatureStats(mPtr);
        const bool wasRaised = stats.mBrutalBlockRaised;
        // The stock shield reaction also uses "shield". Without an adaptive
        // request or owned guard, leave that layer and the vanilla hit state alone.
        if (!stats.mBrutalNpcGuardRequested && !wasRaised && mBrutalNpcGuardGroup.empty())
        {
            updateBrutalNpcGuardTarget(false, false);
            return;
        }
        const double now = brutalSimulationTime();
        const bool minorRecovery = isUpperBodyRecovery();
        bool hasShield = false;
        if (mPtr.getClass().hasInventoryStore(mPtr))
        {
            const auto& inv = mPtr.getClass().getInventoryStore(mPtr);
            const auto shield = inv.getSlot(MWWorld::InventoryStore::Slot_CarriedLeft);
            hasShield = shield != inv.end() && shield->getType() == ESM::Armor::sRecordId
                && updateCarriedLeftVisible(mWeaponType);
        }
        const auto* guardWeapon = getWeaponType(mWeaponType);
        const bool twoHandedGuard = guardWeapon->mWeaponClass == ESM::WeaponType::Melee
            && (guardWeapon->mFlags & ESM::WeaponType::TwoHanded);
        const std::string group = hasShield ? "shield" : twoHandedGuard
            ? (guardWeapon->mShortGroup == "2w" ? "brutal_weapon_guard_2w" : "brutal_weapon_guard_2c")
            : "brutal_weapon_guard";
        const bool hasGuardPose = mAnimation->hasAnimation(group)
            && mAnimation->getTextKeyTime(group + ": brutal guard hold") >= 0.f;
        const bool able = mPtr.getClass().isNpc() && Settings::game().mDarkMessiahNpcCombat
            && stats.mBrutalNpcGuardRequested && now < stats.mBrutalGuardUntil
            && now >= stats.mBrutalGuardBrokenUntil
            && !stats.isDead() && !stats.isParalyzed() && !stats.getKnockedDown()
            && !isKnockedOut() && !isKnockedDown() && !isScriptedAnimPlaying()
            && !stats.mBrutalClinchActive && !mAnimation->isRagdollPose()
            && (!stats.getHitRecovery() || minorRecovery)
            && stats.getDrawState() == DrawState::Weapon && mWeaponType != ESM::Weapon::HandToHand
            && mWeaponType != ESM::Weapon::Spell && mWeaponType != ESM::Weapon::None
            && (mUpperBodyState <= UpperBodyState::WeaponEquipped || minorRecovery)
            && hasGuardPose && !MWBase::Environment::get().getWorld()->isSwimming(mPtr)
            && (!Settings::game().mBrutalStamina || stats.getFatigue().getCurrent() > 0.f);
        if (able)
        {
            // A small reaction may yield to a deliberate defence; a kick, knockdown
            // or full-body stagger must finish first. Both defensive poses come
            // from their own DM source, with the holding key recorded by the importer.
            if (minorRecovery)
            {
                resetCurrentHitState();
                stats.setHitRecovery(false);
                if (!mCurrentWeapon.empty())
                    mAnimation->disable(mCurrentWeapon);
                mUpperBodyState = UpperBodyState::WeaponEquipped;
                mReadyToHit = false;
                mAttackStrength = -1.f;
                stats.mBrutalGuardBreakingAttack = false;
                stats.setAttackingOrSpell(false);
            }
            if (!wasRaised || mBrutalNpcGuardGroup != group || !mAnimation->isPlaying(group))
            {
                MWRender::Animation::ScopedDeferActiveGroupReset deferReset(*mAnimation);
                if (mHitState == CharState_Block && mCurrentHit == "shield")
                {
                    // This stock reaction is being replaced by our held pose.
                    // It must not retain ownership of the same group name and
                    // later make cleanup preserve a frozen adaptive layer.
                    resetCurrentHitState();
                    stats.setBlock(false);
                }
                if (!mBrutalNpcGuardGroup.empty())
                    mAnimation->disable(mBrutalNpcGuardGroup);
                mAnimation->disable(group);
                playBlendedAnimation(group, Priority_Block, MWRender::BlendMask_UpperBody,
                    false, 0.f, "brutal guard hold", hasShield ? "block stop" : "stop", 0.f, 0);
                mBrutalNpcGuardGroup = group;
            }
            if (!wasRaised)
                stats.mBrutalBlockRaisedAt = now;
            stats.mBrutalBlockRaised = true;
            stats.mBrutalParry = !hasShield;
            mBrutalBlock = BrutalBlock::Holding;
        }
        else
        {
            if (!mBrutalNpcGuardGroup.empty())
            {
                // A real stock block may already have replaced the held shield
                // during refreshHitRecoilAnims; do not remove that reaction.
                if (mBrutalNpcGuardGroup != "shield" || mHitState != CharState_Block || mCurrentHit != "shield")
                    mAnimation->disable(mBrutalNpcGuardGroup);
                mBrutalNpcGuardGroup.clear();
            }
            if (wasRaised)
                stats.mBrutalBlockLoweredAt = now;
            stats.mBrutalBlockRaised = stats.mBrutalParry = false;
            mBrutalBlock = BrutalBlock::None;
            if (!hasGuardPose)
            {
                // No compatible imported pose: leave the stock block/attack
                // controller in charge instead of requesting an invisible guard.
                stats.mBrutalNpcGuardRequested = false;
                stats.mBrutalFastPressure = 0;
            }
        }
        stats.mBrutalBlockImpact = false;
        stats.mBrutalParryResult = BrutalParry_None;
        updateBrutalNpcGuardTarget(stats.mBrutalBlockRaised && stats.mBrutalNpcGuardRequested, hasShield);
    }

    void CharacterController::updateBrutalShieldBlock()
    {
        static const std::string group = "brutal_block";
        auto& stats = mPtr.getClass().getCreatureStats(mPtr);
        if (mPtr != getPlayer())
        {
            updateBrutalNpcGuard();
            return;
        }
        if (!Settings::game().mPlayerActiveShieldBlock && !Settings::game().mDarkMessiahSwordParry)
        {
            if (stats.mBrutalBlockRaised && !brutalDmFirstPerson()) mAnimation->disable("shield");
            mAnimation->disable(group);
            mBrutalBlock = BrutalBlock::None;
            stats.mBrutalBlockRaised = stats.mBrutalBlockImpact = stats.mBrutalParry = false;
            stats.mBrutalParryResult = BrutalParry_None;
            return;
        }
        const BrutalBlock before = mBrutalBlock;

        if (mBrutalBlock != BrutalBlock::None && !mAnimation->isPlaying(group))
            mBrutalBlock = BrutalBlock::None; // Lowered, or cleared by another state change.

        bool hasShield = false;
        if (mPtr.getClass().hasInventoryStore(mPtr))
        {
            const auto& inv = mPtr.getClass().getInventoryStore(mPtr);
            const auto shield = inv.getSlot(MWWorld::InventoryStore::Slot_CarriedLeft);
            // Two-handed weapons retain the shield's inventory slot, but cannot
            // use it. A staff raises its own parry even when that slot is occupied.
            hasShield = shield != inv.end() && shield->getType() == ESM::Armor::sRecordId
                && updateCarriedLeftVisible(mWeaponType);
        }
        const bool parry = !hasShield && Settings::game().mDarkMessiahSwordParry
            && (isBrutalOneHanded(mWeaponType) || usesBrutalPairedWeapon());
        static const std::string parryBreak = "brutal_parrybreak";
        const bool able = (hasShield || parry) && !mAnimation->isPlaying(parryBreak) && !isBrutalComboActive()
            && brutalSimulationTime() >= stats.mBrutalGuardBrokenUntil
            && (!Settings::game().mBrutalStamina || stats.getFatigue().getCurrent() > 0.f)
            && stats.mBrutalSlidePhase == 0 && !stats.mBrutalSlideAttack
            && !isDead() && !stats.isDead() && !stats.isParalyzed()
            && stats.getDrawState() == DrawState::Weapon && mWeaponType != ESM::Weapon::Spell && isReadyToBlock()
            && (mHitState == CharState_None || mHitState == CharState_Block) && !isScriptedAnimPlaying()
            && !MWBase::Environment::get().getWorld()->isSwimming(mPtr);
        const bool wanted = able && stats.mBrutalBlockRequested
            && (parry || (hasShield && Settings::game().mPlayerActiveShieldBlock));
        if (!brutalDmFirstPerson() || !mAnimation->hasAnimation(group))
        {
            // Classic hands can use the guard mechanics independently of imported clips.
            const bool wasRaised = stats.mBrutalBlockRaised;
            if (wanted && !wasRaised)
            {
                stats.mBrutalBlockRaisedAt = brutalSimulationTime();
                if (hasShield && mAnimation->hasAnimation("shield"))
                    playBlendedAnimation("shield", Priority_Block, MWRender::BlendMask_LeftArm, false,
                        0.f, "start", "stop", .5f, 0);
            }
            else if (!wanted && wasRaised)
            {
                stats.mBrutalBlockLoweredAt = brutalSimulationTime();
                mAnimation->disable("shield");
            }
            stats.mBrutalBlockRaised = wanted;
            stats.mBrutalParry = wanted && parry;
            stats.mBrutalBlockImpact = false;
            stats.mBrutalParryResult = BrutalParry_None;
            mBrutalBlock = wanted ? BrutalBlock::Holding : BrutalBlock::None;
            return;
        }

        const bool attacking = mUpperBodyState == UpperBodyState::AttackWindUp
            || mUpperBodyState == UpperBodyState::AttackRelease || mUpperBodyState == UpperBodyState::AttackEnd;
        const bool wallRecovery = mUpperBodyState == UpperBodyState::AttackEnd
            && mBrutalFast == BrutalFast::None && mAttackStrength < 0.f;
        if (wanted && brutalFastAvailable() && attacking && !wallRecovery && !mBrutalGuardCharge)
        {
            // Source replaces even an in-flight attack with parry immediately.
            // Already delivered hits remain; fading attack layers must not emit
            // any further hit, offhand strike or queued continuation.
            mAnimation->disable(mCurrentWeapon);
            mUpperBodyState = UpperBodyState::WeaponEquipped;
            mReadyToHit = false;
            mAttackStrength = -1.f;
            mBrutalFast = BrutalFast::None;
            mBrutalFastQueued = false;
            mBrutalFastIndex = 0;
            mBrutalFastHeld = 0.f;
            resetBrutalFastContinuation();
            mBrutalChargedHold = false;
            mBrutalGuardPress = false;
            mBrutalGuardHeld = 0.f;
            mBrutalOffhandType = -1;
            mAttackSuccess = false;
            mAttackVictim = MWWorld::Ptr();
            mBrutalNeutralAttack = false;
            stats.mBrutalSlashRight = false;
        }

        const bool matchGuardPhase
            = !mAnimation->activeSourceHasTextKey("brutal_block: start", "brutal_block: dagger arm frame");
        const bool shieldOverlay
            = mAnimation->activeSourceHasTextKey("brutal_block: start", "brutal_block: shield overlay");
        const auto computeGuardStartPoint = [&](bool toRaise) {
            if (!matchGuardPhase)
                return 0.f;
            const float start = mAnimation->getTextKeyTime(group + ": start");
            const float loopStart = mAnimation->getTextKeyTime(group + ": loop start");
            const float cur = mAnimation->getCurrentTime(group);
            if (start < 0.f || loopStart <= start)
                return 0.f;
            if (toRaise)
            {
                if (shieldOverlay)
                {
                    const float stop = mAnimation->getTextKeyTime(group + ": stop");
                    return stop > start ? (loopStart - start) / (stop - start) : 0.f;
                }
                const float lowerStart = mAnimation->getTextKeyTime(group + ": lower start");
                const float stop = mAnimation->getTextKeyTime(group + ": stop");
                if (lowerStart < 0.f || stop <= lowerStart || cur < lowerStart)
                    return 0.f;
                const float lowerFrac = std::clamp((cur - lowerStart) / (stop - lowerStart), 0.f, 1.f);
                const float targetTime = start + (1.f - lowerFrac) * (loopStart - start);
                return std::clamp((targetTime - start) / (stop - start), 0.f, 1.f);
            }
            if (cur < start)
                return 0.f;
            const float raiseFrac = std::clamp((cur - start) / (loopStart - start), 0.f, 1.f);
            return 1.f - raiseFrac;
        };
        if (wanted && (mBrutalBlock == BrutalBlock::None || mBrutalBlock == BrutalBlock::Lowering)
            && mUpperBodyState == UpperBodyState::WeaponEquipped)
        {
            mBrutalGuardCharge = false;
            const float startPoint = mBrutalBlock == BrutalBlock::Lowering ? computeGuardStartPoint(true) : 0.f;
            MWRender::Animation::ScopedDeferActiveGroupReset deferReset(*mAnimation);
            mAnimation->disable(group);
            playBlendedAnimation(group, Priority_Block, MWRender::BlendMask_UpperBody, false, 1.f, "start", "stop",
                startPoint, std::numeric_limits<uint32_t>::max());
            mBrutalBlock = BrutalBlock::Raising;
        }
        else if (!wanted && (mBrutalBlock == BrutalBlock::Raising || mBrutalBlock == BrutalBlock::Holding))
        {
            if (able)
            {
                const float startPoint = !shieldOverlay && mBrutalBlock == BrutalBlock::Raising
                    ? computeGuardStartPoint(false) : 0.f;
                MWRender::Animation::ScopedDeferActiveGroupReset deferReset(*mAnimation);
                mAnimation->disable(group);
                playBlendedAnimation(group, Priority_Block, MWRender::BlendMask_UpperBody, true, 1.f, "lower start",
                    "stop", startPoint, 0);
                mBrutalBlock = BrutalBlock::Lowering;
            }
            else
            {
                mAnimation->disable(group);
                mBrutalBlock = BrutalBlock::None;
            }
        }

        if (mBrutalBlock == BrutalBlock::Raising
            && mAnimation->getCurrentTime(group) >= mAnimation->getTextKeyTime(group + ": loop start"))
            mBrutalBlock = BrutalBlock::Holding;

        bool raised = mBrutalBlock == BrutalBlock::Raising || mBrutalBlock == BrutalBlock::Holding;
        const bool wasRaised = before == BrutalBlock::Raising || before == BrutalBlock::Holding;
        if (raised && !wasRaised)
            stats.mBrutalBlockRaisedAt = brutalSimulationTime();
        else if (!raised && wasRaised)
            stats.mBrutalBlockLoweredAt = brutalSimulationTime();
        if (stats.mBrutalParryResult == BrutalParry_Broken)
        {
            // Parry_Break_Sword: the guard is knocked open and cannot rise until it recovers.
            if ((raised || wasRaised) && mAnimation->hasAnimation(parryBreak))
            {
                mAnimation->disable(group);
                playBlendedAnimation(parryBreak, Priority_Knockdown, MWRender::BlendMask_UpperBody, true, 1.f,
                    "start", "stop", 0.f, 0);
                mBrutalBlock = BrutalBlock::None;
                raised = false;
                stats.mBrutalBlockLoweredAt = brutalSimulationTime();
            }
        }
        else if (stats.mBrutalParryResult != BrutalParry_None)
            stats.mBrutalBlockImpact = true; // Parry_Parry_Sword plays A, B or C.
        stats.mBrutalParryResult = BrutalParry_None;
        if (stats.mBrutalBlockImpact && raised && mAnimation->hasAnimation("brutal_blockhit1"))
        {
            const std::string impact = chooseRandomGroup("brutal_blockhit");
            {
                MWRender::Animation::ScopedDeferActiveGroupReset deferReset(*mAnimation);
                for (int i = 1; i <= 3; ++i)
                    mAnimation->disable("brutal_blockhit" + std::to_string(i));
                playBlendedAnimation(impact, Priority_Knockdown, MWRender::BlendMask_UpperBody, true, 1.f, "start",
                    "stop", 0.f, 0);
            }
        }
        stats.mBrutalBlockImpact = false;
        stats.mBrutalBlockRaised = raised;
        stats.mBrutalParry = raised && !hasShield;
    }

    void CharacterController::updateBrutalWounded()
    {
        auto& stats = mPtr.getClass().getCreatureStats(mPtr);
        if (stats.mBrutalWoundedFall.length2() > 0.f)
        {
            const osg::Vec3f push = stats.mBrutalWoundedFall;
            const MWWorld::Ptr attacker = stats.mBrutalWoundedFallAttacker.ptrOrEmpty();
            stats.mBrutalWoundedFall = osg::Vec3f();
            stats.mBrutalWoundedFallAttacker = {};
            if (!stats.isDead() && !stats.getKnockedDown() && stats.mBrutalClinchOpponent.ptrOrEmpty().isEmpty())
                MWBase::Environment::get().getWorld()->applyNpcKick(mPtr, push, attacker, true);
        }
        if (stats.isDead() || mAnimation->isRagdollPose())
            return;
        const bool wounded = isBrutalWoundedNpc(mPtr);
        // A claimed paired animation keeps its pose; the change waits until it ends.
        if (wounded == mBrutalWounded || stats.mBrutalClinchActive || !stats.mBrutalClinchOpponent.ptrOrEmpty().isEmpty())
            return;
        mBrutalWounded = wounded;
        if (mAnimation->setBrutalWounded(wounded))
            forceStateUpdate();
    }

    void CharacterController::forceStateUpdate()
    {
        if (mPtr.getClass().isActor() && (mPtr.getClass().getCreatureStats(mPtr).mBrutalClinchActive
                || !mPtr.getClass().getCreatureStats(mPtr).mBrutalClinchOpponent.ptrOrEmpty().isEmpty()))
            return; // Equipment/control refreshes must not replace a claimed paired animation.
        if (!mAnimation)
            return;
        if (!Settings::game().mBrutalRagdoll && mPtr.getClass().getCreatureStats(mPtr).isDead()
            && mPtr.getClass().getCreatureStats(mPtr).isDeathAnimationFinished())
        {
            const auto saved = mPtr.getClass().getCreatureStats(mPtr).getDeathAnimation();
            playDeath(1.f, saved < 0 ? CharState_Death1 : static_cast<CharacterState>(CharState_Death1 + saved));
            return;
        }
        if (hasPhysicalDeath() && mPtr.getClass().getCreatureStats(mPtr).isDead()
            && (mPtr.getClass().isNpc() || (mAnimation && mAnimation->isRagdollPose())))
        {
            const auto& stats = mPtr.getClass().getCreatureStats(mPtr);
            if (!mAnimation->isRagdollPose() && stats.isDeathAnimationFinished())
            {
                // Equipment/skeleton rebuilds restore the saved static corpse
                // pose; they neither restart a random death nor touch live
                // ragdoll pose ownership, history or physical motion.
                CharacterState death = mDeathState;
                if (death == CharState_None)
                {
                    const signed char saved = stats.getDeathAnimation();
                    death = saved < 0 ? CharState_Death1
                        : static_cast<CharacterState>(CharState_Death1 + saved);
                }
                playDeath(1.f, death);
            }
            return;
        }
        if (!Settings::game().mDarkMessiahComboAttacks) cancelBrutalCombo();
        if (!Settings::game().mBrutalSlide) cancelBrutalSlide();
        clearAnimQueue();

        // Make sure we canceled the current attack or spellcasting,
        // because we disabled attack animations anyway.
        mCanCast = false;
        mCastingScriptedSpell = false;
        // Equipment can rebuild the viewmodel with the same right weapon.
        // A former set's queue and continuation history cannot enter the new one.
        mBrutalFast = BrutalFast::None;
        mBrutalFastQueued = false;
        mBrutalFastIndex = 0;
        mBrutalFastButton = false;
        mBrutalFastHeld = 0.f;
        resetBrutalFastContinuation();
        setAttackingOrSpell(false);
        if (mUpperBodyState != UpperBodyState::None)
            mUpperBodyState = UpperBodyState::WeaponEquipped;

        refreshCurrentAnims(mIdleState, mMovementState, mJumpState, true);

        if (mDeathState != CharState_None)
        {
            playRandomDeath();
        }

        updateAnimQueue();

        mAnimation->runAnimation(0.f);
    }

    CharacterController::KillResult CharacterController::kill()
    {
        if (mDeathState == CharState_None)
        {
            cancelBackstab(false);
            cancelGroundFinisher();
            auto& stats = mPtr.getClass().getCreatureStats(mPtr);
            const osg::Vec3f deathPush = std::exchange(stats.mBrutalPlayerDeathPush, {});
            if (hasPhysicalDeath())
            {
                // Capture the last evaluated pose before a death animation or
                // its idle/weapon resets can replace the rendered stance.
                // This base NPC death behavior also applies with enhancements
                // disabled; combat already delivered its native point impulse.
                const auto world = MWBase::Environment::get().getWorld();
                auto* playerAnimation = mPtr == getPlayer() ? dynamic_cast<MWRender::NpcAnimation*>(mAnimation) : nullptr;
                bool physical = mAnimation && mAnimation->isRagdollPose();
                if (!physical)
                {
                    try
                    {
                        if (playerAnimation)
                            playerAnimation->preparePlayerDeathPose();
                        physical = world->startNpcRagdoll(mPtr, deathPush);
                    }
                    catch (const std::exception& error)
                    {
                        Log(Debug::Warning) << "Death ragdoll rejected for " << mPtr.getCellRef().getRefId()
                            << ": " << error.what();
                    }
                    if (!physical && (mPtr == getPlayer() || !mPtr.getClass().isNpc()))
                    {
                        // A player or creature with an unsupported rig can still use the normal death animation.
                        playRandomDeath();
                        resetCurrentIdleState();
                        return Result_DeathAnimStarted;
                    }
                    if (!physical)
                    {
                        // Unsupported skeletons keep their current pose. Never
                        // substitute a canned death or retry every update.
                        if (mAnimation)
                        {
                            mAnimation->beginRagdollPose();
                            world->releaseNpcRagdollEquipment(mPtr);
                        }
                        Log(Debug::Warning) << "NPC death retained its current pose without a supported ragdoll: "
                            << mPtr.getCellRef().getRefId();
                    }
                }
                // Retain native Started -> JustFinished -> Finished reporting
                // for dying words, death counts, scripts and spell cleanup.
                mDeathState = CharState_Death1;
                mCurrentDeath = deathStateToAnimGroup(mDeathState);
                stats.setDeathAnimation(0);
                resetCurrentMovementState();
                resetCurrentWeaponState();
                resetCurrentHitState();
                resetCurrentIdleState();
                resetCurrentJumpState();
                return Result_DeathAnimStarted;
            }
            playRandomDeath();
            resetCurrentIdleState();
            return Result_DeathAnimStarted;
        }

        MWMechanics::CreatureStats& cStats = mPtr.getClass().getCreatureStats(mPtr);
        // A ragdoll owns the pose immediately, but Actors must still receive
        // Started, then JustFinished exactly once for death counts, Died, spell
        // cleanup and corpse inventory. Never fast-forward the rendered pose.
        if (isAnimPlaying(mCurrentDeath) && !(mAnimation && mAnimation->isRagdollPose()))
            return Result_DeathAnimPlaying;
        if (!cStats.isDeathAnimationFinished())
        {
            cStats.setDeathAnimationFinished(true);
            return Result_DeathAnimJustFinished;
        }
        return Result_DeathAnimFinished;
    }

    void CharacterController::resurrect()
    {
        if (mDeathState == CharState_None)
            return;

        // Ordinary physical removal already ends pose ownership. Unsupported
        // skeletons have no PhysicsSystem body, so release their frozen pose
        // here as well before normal living animations resume.
        if (mAnimation)
            mAnimation->endRagdollPose();
        resetCurrentDeathState();
        mWeaponType = ESM::Weapon::None;
        if (mPtr == getPlayer())
            if (auto* animation = dynamic_cast<MWRender::NpcAnimation*>(mAnimation))
                animation->restorePlayerDeathView();
    }

    void CharacterController::updateContinuousVfx() const
    {
        // Keeping track of when to stop a continuous VFX seems to be very difficult to do inside the spells code,
        // as it's extremely spread out (ActiveSpells, Spells, InventoryStore effects, etc...) so we do it here.

        // Stop any effects that are no longer active
        std::vector<std::string_view> effects = mAnimation->getLoopingEffects();

        for (std::string_view effectStr : effects)
        {
            auto effectId = ESM::RefId::deserializeText(effectStr);

            if (MWBase::Environment::get().getESMStore()->get<ESM::MagicEffect>().search(effectId)
                && (mPtr.getClass().getCreatureStats(mPtr).isDeathAnimationFinished()
                    || mPtr.getClass()
                            .getCreatureStats(mPtr)
                            .getMagicEffects()
                            .getOrDefault(MWMechanics::EffectKey(effectId))
                            .getMagnitude()
                        <= 0))
                mAnimation->removeEffect(effectStr);
        }
    }

    void CharacterController::updateMagicEffects() const
    {
        if (!mPtr.getClass().isActor())
            return;

        float light = mPtr.getClass()
                          .getCreatureStats(mPtr)
                          .getMagicEffects()
                          .getOrDefault(ESM::MagicEffect::Light)
                          .getMagnitude();
        mAnimation->setLightEffect(light);

        // If you're dead you don't care about whether you've started/stopped being a vampire or not
        if (mPtr.getClass().getCreatureStats(mPtr).isDead())
            return;

        bool vampire = mPtr.getClass()
                           .getCreatureStats(mPtr)
                           .getMagicEffects()
                           .getOrDefault(ESM::MagicEffect::Vampirism)
                           .getMagnitude()
            > 0.0f;
        mAnimation->setVampire(vampire);
    }

    void CharacterController::setVisibility(float visibility) const
    {
        if (!mAnimation)
            return;
        // We should take actor's invisibility in account
        if (mPtr.getClass().isActor())
        {
            float alpha = 1.f;
            if (mPtr.getClass()
                    .getCreatureStats(mPtr)
                    .getMagicEffects()
                    .getOrDefault(ESM::MagicEffect::Invisibility)
                    .getModifier()) // Ignore base magnitude (see bug #3555).
            {
                if (mPtr == getPlayer())
                    alpha = 0.25f;
                else
                    alpha = 0.05f;
            }
            float chameleon = mPtr.getClass()
                                  .getCreatureStats(mPtr)
                                  .getMagicEffects()
                                  .getOrDefault(ESM::MagicEffect::Chameleon)
                                  .getMagnitude();
            if (chameleon)
            {
                alpha *= std::clamp(1.f - chameleon / 100.f, 0.25f, 0.75f);
            }

            visibility = std::min(visibility, alpha);
        }

        // TODO: implement a dithering shader rather than just change object transparency.
        mAnimation->setAlpha(visibility);
    }

    std::string_view CharacterController::getMovementBasedAttackType() const
    {
        float* move = mPtr.getClass().getMovementSettings(mPtr).mPosition;
        if (std::abs(move[1]) > std::abs(move[0]) + 0.2f) // forward-backward
            return "thrust";
        if (std::abs(move[0]) > std::abs(move[1]) + 0.2f) // sideway
            return "slash";
        return "chop";
    }

    bool CharacterController::isRandomAttackAnimation(std::string_view group)
    {
        return (group == "attack1" || group == "swimattack1" || group == "attack2" || group == "swimattack2"
            || group == "attack3" || group == "swimattack3");
    }

    bool CharacterController::isAttackPreparing() const
    {
        return mUpperBodyState == UpperBodyState::AttackWindUp;
    }

    bool CharacterController::isCastingSpell() const
    {
        return mCastingScriptedSpell || mUpperBodyState == UpperBodyState::Casting;
    }

    bool CharacterController::isReadyToBlock() const
    {
        if (usesBrutalPairedWeapon())
            return true;
        if (mAnimation && mAnimation->useBrutalStaffSet() && brutalStaffWeapon(mWeaponType))
            return true; // The staff guard is a two-handed parry, not a shield stance.
        return mAnimation && updateCarriedLeftVisible(mWeaponType);
    }

    bool CharacterController::isKnockedDown() const
    {
        return mHitState == CharState_KnockDown || mHitState == CharState_SwimKnockDown;
    }

    bool CharacterController::isKnockedOut() const
    {
        return mHitState == CharState_KnockOut || mHitState == CharState_SwimKnockOut;
    }

    bool CharacterController::isTurning() const
    {
        return mMovementState == CharState_TurnLeft || mMovementState == CharState_TurnRight
            || mMovementState == CharState_SwimTurnLeft || mMovementState == CharState_SwimTurnRight;
    }

    bool CharacterController::isRecovery() const
    {
        return mHitState == CharState_Hit || mHitState == CharState_SwimHit;
    }

    bool CharacterController::isUpperBodyRecovery() const
    {
        return mAnimation && isRecovery() && mPtr != getPlayer() && mPtr.getClass().isNpc()
            && mAnimation->getTextKeyTime(mCurrentHit + ": brutal upper body") >= 0.f;
    }

    bool CharacterController::isAttackingOrSpell() const
    {
        return mUpperBodyState != UpperBodyState::None && mUpperBodyState != UpperBodyState::WeaponEquipped;
    }

    bool CharacterController::isSneaking() const
    {
        return mIdleState == CharState_IdleSneak || mMovementState == CharState_SneakForward
            || mMovementState == CharState_SneakBack || mMovementState == CharState_SneakLeft
            || mMovementState == CharState_SneakRight;
    }

    bool CharacterController::isRunning() const
    {
        return mMovementState == CharState_RunForward || mMovementState == CharState_RunBack
            || mMovementState == CharState_RunLeft || mMovementState == CharState_RunRight
            || mMovementState == CharState_SwimRunForward || mMovementState == CharState_SwimRunBack
            || mMovementState == CharState_SwimRunLeft || mMovementState == CharState_SwimRunRight;
    }

    void CharacterController::setAttackingOrSpell(bool attackingOrSpell) const
    {
        mPtr.getClass().getCreatureStats(mPtr).setAttackingOrSpell(attackingOrSpell);
    }

    void CharacterController::castSpell(const ESM::RefId& spellId, bool scriptedSpell)
    {
        setAttackingOrSpell(true);
        mCastingScriptedSpell = scriptedSpell;
        ActionSpell action = ActionSpell(spellId);
        action.prepare(mPtr);
    }

    void CharacterController::setAIAttackType(std::string_view attackType)
    {
        mAttackType = attackType;
    }

    std::string_view CharacterController::getRandomAttackType()
    {
        MWBase::World* world = MWBase::Environment::get().getWorld();
        float random = Misc::Rng::rollProbability(world->getPrng());
        if (random >= 2 / 3.f)
            return "thrust";
        if (random >= 1 / 3.f)
            return "slash";
        return "chop";
    }

    bool CharacterController::readyToPrepareAttack() const
    {
        return (mHitState == CharState_None || mHitState == CharState_Block)
            && mUpperBodyState <= UpperBodyState::WeaponEquipped;
    }

    bool CharacterController::readyToStartBrutalGuard() const
    {
        return readyToPrepareAttack() || isUpperBodyRecovery();
    }

    bool CharacterController::readyToStartAttack() const
    {
        if (mHitState != CharState_None && mHitState != CharState_Block)
            return false;

        return mUpperBodyState == UpperBodyState::WeaponEquipped;
    }

    float CharacterController::getAttackStrength() const
    {
        return mAttackStrength;
    }

    bool CharacterController::getAttackingOrSpell() const
    {
        const auto& stats = mPtr.getClass().getCreatureStats(mPtr);
        return stats.getAttackingOrSpell() && !stats.mBrutalClinchActive
            && stats.mBrutalClinchOpponent.ptrOrEmpty().isEmpty() && !stats.mBrutalClinchReleaseUse;
    }

    std::string_view CharacterController::getDesiredAttackType() const
    {
        return mPtr.getClass().getCreatureStats(mPtr).getAttackType();
    }

    void CharacterController::setActive(int active) const
    {
        if (mAnimation)
            mAnimation->setActive(active);
    }

    void CharacterController::setHeadTrackTarget(const MWWorld::ConstPtr& target)
    {
        mHeadTrackTarget = target;
    }

    void CharacterController::playSwishSound() const
    {
        static ESM::RefId weaponSwish = ESM::RefId::stringRefId("Weapon Swish");
        const ESM::RefId* soundId = &weaponSwish;
        float volume = 0.98f + mAttackStrength * 0.02f;
        float pitch = 0.75f + mAttackStrength * 0.4f;

        const MWWorld::Class& cls = mPtr.getClass();
        if (cls.isNpc() && cls.getNpcStats(mPtr).isWerewolf())
        {
            MWBase::World* world = MWBase::Environment::get().getWorld();
            const MWWorld::ESMStore& store = world->getStore();
            const ESM::Sound* sound = store.get<ESM::Sound>().searchRandom("WolfSwing", world->getPrng());
            if (sound)
                soundId = &sound->mId;
        }

        if (!soundId->empty())
            MWBase::Environment::get().getSoundManager()->playSound3D(mPtr, *soundId, volume, pitch);
    }

    float CharacterController::getAnimationMovementDirection() const
    {
        if (!mAnimation)
            return 0.f;
        switch (mMovementState)
        {
            case CharState_RunLeft:
            case CharState_SneakLeft:
            case CharState_SwimWalkLeft:
            case CharState_SwimRunLeft:
            case CharState_WalkLeft:
                return osg::PI_2f;
            case CharState_RunRight:
            case CharState_SneakRight:
            case CharState_SwimWalkRight:
            case CharState_SwimRunRight:
            case CharState_WalkRight:
                return -osg::PI_2f;
            case CharState_RunForward:
            case CharState_SneakForward:
            case CharState_SwimRunForward:
            case CharState_SwimWalkForward:
            case CharState_WalkForward:
                return mAnimation->getLegsYawRadians();
            case CharState_RunBack:
            case CharState_SneakBack:
            case CharState_SwimWalkBack:
            case CharState_SwimRunBack:
            case CharState_WalkBack:
                return mAnimation->getLegsYawRadians() - osg::PIf;
            default:
                return 0.0f;
        }
    }

    void CharacterController::updateHeadTracking(float duration)
    {
        const osg::Node* head = mAnimation->getNode("Bip01 Head");
        if (!head)
            return;

        float zAngleRadians = 0.f;
        float xAngleRadians = 0.f;

        if (!mHeadTrackTarget.isEmpty())
        {
            osg::NodePathList nodepaths = head->getParentalNodePaths();
            if (nodepaths.empty())
                return;
            osg::Matrixf mat = osg::computeLocalToWorld(nodepaths[0]);
            osg::Vec3f headPos = mat.getTrans();

            osg::Vec3f direction;
            if (const MWRender::Animation* anim = MWBase::Environment::get().getWorld()->getAnimation(mHeadTrackTarget))
            {
                const osg::Node* node = anim->getNode("Head");
                if (node == nullptr)
                    node = anim->getNode("Bip01 Head");
                if (node != nullptr)
                {
                    nodepaths = node->getParentalNodePaths();
                    if (!nodepaths.empty())
                        direction = osg::computeLocalToWorld(nodepaths[0]).getTrans() - headPos;
                }
                else
                    // no head node to look at, fall back to look at center of collision box
                    direction = MWBase::Environment::get().getWorld()->aimToTarget(mPtr, mHeadTrackTarget, false);
            }
            direction.normalize();

            if (!mPtr.getRefData().getBaseNode())
                return;
            const osg::Vec3f actorDirection = mPtr.getRefData().getBaseNode()->getAttitude() * osg::Vec3f(0, 1, 0);

            zAngleRadians
                = std::atan2(actorDirection.x(), actorDirection.y()) - std::atan2(direction.x(), direction.y());
            zAngleRadians = static_cast<float>(
                Misc::normalizeAngle(zAngleRadians - mAnimation->getHeadYaw()) + mAnimation->getHeadYaw());
            zAngleRadians *= (1 - direction.z() * direction.z());
            xAngleRadians = std::asin(direction.z());
        }

        const float xLimit = osg::DegreesToRadians(40.f);
        const float zLimit = osg::DegreesToRadians(30.f);
        float zLimitOffset = mAnimation->getUpperBodyYawRadians();
        xAngleRadians = std::clamp(xAngleRadians, -xLimit, xLimit);
        zAngleRadians = std::clamp(zAngleRadians, -zLimit + zLimitOffset, zLimit + zLimitOffset);

        float factor = duration * 5;
        factor = std::min(factor, 1.f);
        xAngleRadians = (1.f - factor) * mAnimation->getHeadPitch() + factor * xAngleRadians;
        zAngleRadians = (1.f - factor) * mAnimation->getHeadYaw() + factor * zAngleRadians;

        mAnimation->setHeadPitch(xAngleRadians);
        mAnimation->setHeadYaw(zAngleRadians);
    }

    MWWorld::MovementDirectionFlags CharacterController::getSupportedMovementDirections() const
    {
        if (!mAnimation)
            return 0;
        using namespace std::string_view_literals;
        // There are fallbacks in the CharacterController::refreshMovementAnims for certain animations. Arrays below
        // represent them.
        constexpr std::array all = { ""sv };
        constexpr std::array walk = { "walk"sv };
        constexpr std::array swimWalk = { "swimwalk"sv, "walk"sv };
        constexpr std::array sneak = { "sneak"sv };
        constexpr std::array run = { "run"sv, "walk"sv };
        constexpr std::array swimRun = { "swimrun"sv, "run"sv, "walk"sv };
        constexpr std::array swim = { "swim"sv };
        switch (mMovementState)
        {
            case CharState_None:
            case CharState_SpecialIdle:
            case CharState_Idle:
            case CharState_IdleSwim:
            case CharState_IdleSneak:
                return mAnimation->getSupportedMovementDirections(all);
            case CharState_WalkForward:
            case CharState_WalkBack:
            case CharState_WalkLeft:
            case CharState_WalkRight:
                return mAnimation->getSupportedMovementDirections(walk);
            case CharState_SwimWalkForward:
            case CharState_SwimWalkBack:
            case CharState_SwimWalkLeft:
            case CharState_SwimWalkRight:
                return mAnimation->getSupportedMovementDirections(swimWalk);
            case CharState_RunForward:
            case CharState_RunBack:
            case CharState_RunLeft:
            case CharState_RunRight:
                return mAnimation->getSupportedMovementDirections(run);
            case CharState_SwimRunForward:
            case CharState_SwimRunBack:
            case CharState_SwimRunLeft:
            case CharState_SwimRunRight:
                return mAnimation->getSupportedMovementDirections(swimRun);
            case CharState_SneakForward:
            case CharState_SneakBack:
            case CharState_SneakLeft:
            case CharState_SneakRight:
                return mAnimation->getSupportedMovementDirections(sneak);
            case CharState_TurnLeft:
            case CharState_TurnRight:
                return mAnimation->getSupportedMovementDirections(all);
            case CharState_SwimTurnLeft:
            case CharState_SwimTurnRight:
                return mAnimation->getSupportedMovementDirections(swim);
            case CharState_Death1:
            case CharState_Death2:
            case CharState_Death3:
            case CharState_Death4:
            case CharState_Death5:
            case CharState_SwimDeath:
            case CharState_SwimDeathKnockDown:
            case CharState_SwimDeathKnockOut:
            case CharState_DeathKnockDown:
            case CharState_DeathKnockOut:
            case CharState_Hit:
            case CharState_SwimHit:
            case CharState_KnockDown:
            case CharState_KnockOut:
            case CharState_SwimKnockDown:
            case CharState_SwimKnockOut:
            case CharState_Block:
                return mAnimation->getSupportedMovementDirections(all);
        }
        return 0;
    }

}
