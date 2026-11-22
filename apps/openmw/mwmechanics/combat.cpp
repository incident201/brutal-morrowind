
#include "combat.hpp"
#include "weapontype.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdlib>
#include <iomanip>
#include <sstream>
#include <vector>

#include <components/debug/debuglog.hpp>
#include <components/misc/constants.hpp>
#include <components/misc/rng.hpp>
#include <components/misc/strings/lower.hpp>
#include <components/settings/values.hpp>

#include <components/sceneutil/positionattitudetransform.hpp>

#include <components/esm3/loadench.hpp>
#include <components/esm3/loadmgef.hpp>
#include <components/esm3/loadsoun.hpp>

#include "../mwmechanics/brutaloptions.hpp"
#include "../mwbase/dialoguemanager.hpp"
#include "../mwbase/environment.hpp"
#include "../mwbase/luamanager.hpp"
#include "../mwbase/mechanicsmanager.hpp"
#include "../mwbase/soundmanager.hpp"
#include "../mwbase/windowmanager.hpp"
#include "../mwbase/world.hpp"
#include "../mwphysics/raycasting.hpp"
#include "../mwrender/animation.hpp"
#include "../mwrender/npcanimation.hpp"

#include "../mwworld/class.hpp"
#include "../mwworld/datetimemanager.hpp"
#include "../mwworld/esmstore.hpp"
#include "../mwworld/globals.hpp"
#include "../mwworld/inventorystore.hpp"

#include "actorutil.hpp"
#include "difficultyscaling.hpp"
#include "movement.hpp"
#include "npcstats.hpp"
#include "pathfinding.hpp"
#include "spellcasting.hpp"
#include "spellresistance.hpp"

namespace
{
    float signedAngleRadians(const osg::Vec3f& v1, const osg::Vec3f& v2, const osg::Vec3f& normal)
    {
        return std::atan2(normal * (v1 ^ v2), v1 * v2);
    }
}

namespace MWMechanics
{
    bool isBrutalWoundedNpc(const MWWorld::Ptr& actor)
    {
        if (!Settings::game().mBrutalWoundedNpcs || actor.isEmpty() || !actor.getClass().isNpc()
            || actor == getPlayer())
            return false;
        const CreatureStats& stats = actor.getClass().getCreatureStats(actor);
        if (stats.isDead())
            return false;
        const float health = stats.getHealth().getCurrent();
        const float maximum = stats.getHealth().getModified();
        return maximum > 0.f && health > 0.f
            && health < Settings::game().mBrutalNpcWoundedHealthFraction * maximum;
    }

    bool applyOnStrikeEnchantment(const MWWorld::Ptr& attacker, const MWWorld::Ptr& victim, const MWWorld::Ptr& object,
        const osg::Vec3f& hitPosition, const bool fromProjectile)
    {
        const ESM::RefId enchantmentName = !object.isEmpty() ? object.getClass().getEnchantment(object) : ESM::RefId();
        if (!enchantmentName.empty())
        {
            const ESM::Enchantment* enchantment
                = MWBase::Environment::get().getESMStore()->get<ESM::Enchantment>().find(enchantmentName);
            if (enchantment->mData.mType == ESM::Enchantment::WhenStrikes)
            {
                MWMechanics::CastSpell cast(attacker, victim, fromProjectile);
                cast.mHitPosition = hitPosition;
                cast.cast(object, false);
                // Apply magic effects directly instead of waiting a frame to allow soul trap to work on one-hit kills
                if (!victim.isEmpty() && victim.getClass().isActor())
                    MWBase::Environment::get().getMechanicsManager()->updateMagicEffects(victim);
                return true;
            }
        }
        return false;
    }

    namespace
    {
        // The player's held guard protects a frontal arc. The first-person guard is a
        // viewmodel in front of the camera, not a surface on the body's arm, so AI
        // contact rays met it only by chance.
        bool isInFrontalGuard(const MWWorld::Ptr& attacker, const MWWorld::Ptr& blocker)
        {
            constexpr float frontalArcDegrees = 70.f;
            const osg::Vec3f forward = osg::Quat(blocker.getRefData().getPosition().rot[2], osg::Vec3f(0, 0, -1))
                * osg::Vec3f(0, 1, 0);
            osg::Vec3f toAttacker = attacker.getRefData().getPosition().asVec3()
                - blocker.getRefData().getPosition().asVec3();
            toAttacker.z() = 0.f;
            return toAttacker.normalize() >= 1e-3f
                && toAttacker * forward >= std::cos(osg::DegreesToRadians(frontalArcDegrees));
        }

        bool hasAdaptiveGuardMesh(const MWWorld::Ptr& target)
        {
            if (target.isEmpty() || target == getPlayer() || !target.getClass().isNpc()
                || !Settings::game().mDarkMessiahNpcCombat)
                return false;
            const auto& stats = target.getClass().getCreatureStats(target);
            if (stats.isDead() || !stats.mBrutalNpcGuardRequested || !stats.mBrutalBlockRaised)
                return false;
            const auto* npc = dynamic_cast<const MWRender::NpcAnimation*>(
                MWBase::Environment::get().getWorld()->getAnimation(target));
            return npc && npc->hasNpcGuardMeshForCombat();
        }

        bool isAdaptiveGuardSurface(const MWWorld::Ptr& target, int part)
        {
            if (!hasAdaptiveGuardMesh(target))
                return false;
            return part == (target.getClass().getCreatureStats(target).mBrutalParry
                    ? ESM::PRT_Weapon : ESM::PRT_Shield);
        }

        void logNpcGuardContact(const MWWorld::Ptr& blocker, const osg::Vec3f& origin, const osg::Vec3f& end,
            const std::optional<MWRender::NpcAnimation::NpcGuardMeshTrace>& contact)
        {
            static const bool enabled = [] {
                const char* flag = std::getenv("BRUTAL_GUARD_CONTACT_AUDIT");
                return flag && std::string_view(flag) == "1";
            }();
            if (!enabled)
                return;
            std::ostringstream data;
            data << std::setprecision(17) << "{\"time\":"
                 << MWBase::Environment::get().getWorld()->getTimeManager()->getSimulationTime()
                 << ",\"valid\":" << (contact ? "true" : "false");
            const auto vector = [&](const char* name, const osg::Vec3f& value) {
                data << ",\"" << name << "\":[" << value.x() << ',' << value.y() << ',' << value.z() << ']';
            };
            vector("origin", origin);
            vector("end", end);
            if (contact)
            {
                data << ",\"frame\":" << contact->frame << ",\"gap\":";
                if (std::isfinite(contact->gap))
                    data << contact->gap;
                else
                    data << "null";
                data << ",\"hit\":" << (contact->hit ? "true" : "false");
                vector("point", contact->point);
            }
            if (const auto* base = blocker.getRefData().getBaseNode())
            {
                const auto worlds = base->getWorldMatrices();
                if (!worlds.empty())
                {
                    data << ",\"actorWorld\":[";
                    for (unsigned i = 0; i < 16; ++i)
                        data << (i ? "," : "") << worlds.front().ptr()[i];
                    data << ']';
                }
            }
            data << '}';
            Log(Debug::Info) << "BRUTAL_GUARD_CONTACT " << data.str();
        }

        void openBrutalGuard(const MWWorld::Ptr& blocker)
        {
            auto& stats = blocker.getClass().getCreatureStats(blocker);
            const double now = MWBase::Environment::get().getWorld()->getTimeManager()->getSimulationTime();
            stats.mBrutalNpcGuardRequested = false;
            stats.mBrutalGuardUntil = 0.0;
            stats.mBrutalGuardBrokenUntil = std::max(stats.mBrutalGuardBrokenUntil, now + 1.0);
            stats.mBrutalGuardCooldownUntil = std::max(stats.mBrutalGuardCooldownUntil, now + 1.0);
            stats.mBrutalFastPressure = 0;
            stats.mBrutalParryResult = BrutalParry_Broken;
            if (blocker != getPlayer())
                stats.mBrutalParried = true;
        }

        constexpr float heavyBlockStaminaFraction = 2.f / 9.f;

        float blockStaminaCost(const DynamicStat<float>& fatigue, float fraction)
        {
            return std::max(0.f, fatigue.getBase()) * fraction;
        }

        bool canAbsorbChargedAttack(const CreatureStats& playerStats)
        {
            if (!Settings::game().mBrutalStamina || MWBase::Environment::get().getWorld()->getGodModeState())
                return true;
            const auto& fatigue = playerStats.getFatigue();
            return fatigue.getCurrent() >= blockStaminaCost(fatigue, heavyBlockStaminaFraction);
        }

        // Impacts against a guard cost more to deliver than to absorb. Fractions of
        // maximum match the player's nine-unit stamina scale and do not favour a
        // particular race/level. Ordinary open hits and misses keep their old costs.
        void spendBrutalBlockStamina(const MWWorld::Ptr& attacker, const MWWorld::Ptr& blocker, bool heavy)
        {
            ++blocker.getClass().getCreatureStats(blocker).mBrutalGuardBlocks;
            if (!Settings::game().mBrutalStamina)
                return;
            const auto spend = [](const MWWorld::Ptr& actor, float fraction) {
                if (actor == getPlayer() && MWBase::Environment::get().getWorld()->getGodModeState())
                    return;
                auto& stats = actor.getClass().getCreatureStats(actor);
                auto fatigue = stats.getFatigue();
                fatigue.setCurrent(std::max(0.f, fatigue.getCurrent() - blockStaminaCost(fatigue, fraction)));
                stats.setFatigue(fatigue);
            };
            spend(attacker, heavy ? 3.f / 9.f : 2.f / 9.f);
            spend(blocker, heavy ? heavyBlockStaminaFraction : 1.f / 9.f);
            if (blocker.getClass().getCreatureStats(blocker).getFatigue().getCurrent() <= 0.f)
                openBrutalGuard(blocker);
        }

        bool brutalGuardBreakingAttack(const MWWorld::Ptr& attacker, float strength)
        {
            const auto& stats = attacker.getClass().getCreatureStats(attacker);
            return attacker == getPlayer() ? !stats.mBrutalFastStrike && strength >= .8f
                                          : stats.mBrutalGuardBreakingAttack;
        }

        bool parryMeleeAttack(const MWWorld::Ptr& attacker, const MWWorld::Ptr& blocker, float damage)
        {
            MWMechanics::CreatureStats& stats = blocker.getClass().getCreatureStats(blocker);
            if (!isInFrontalGuard(attacker, blocker))
            {
                stats.mBrutalParryResult = BrutalParry_Broken;
                return false; // The blow comes from outside the guard and lands.
            }
            const double now = MWBase::Environment::get().getWorld()->getTimeManager()->getSimulationTime();
            const float parryWindow = Settings::game().mDarkMessiahParryWindow.get();
            const bool timed = now - stats.mBrutalBlockRaisedAt <= parryWindow;

            MWWorld::InventoryStore& inv = blocker.getClass().getInventoryStore(blocker);
            const auto parrying = inv.getSlot(MWWorld::InventoryStore::Slot_CarriedRight);
            if (parrying != inv.end())
            {
                MWWorld::Ptr parryWeapon = *parrying;
                MWMechanics::reduceWeaponCondition(damage, true, parryWeapon, blocker);
            }
            MWBase::Environment::get().getSoundManager()->playSound3D(blocker,
                ESM::RefId::stringRefId(timed ? "Brutal Sword Parry" : "Brutal Sword Contact"), 1.0f, 1.0f);
            if (timed && attacker.getClass().isNpc()
                && !MWBase::Environment::get().getMechanicsManager()->requestSwordClinch(blocker, attacker))
                attacker.getClass().getCreatureStats(attacker).mBrutalParried = true;
            stats.mBrutalParryResult = timed ? BrutalParry_Timed : BrutalParry_Parried;
            spendBrutalBlockStamina(attacker, blocker,
                attacker.getClass().getCreatureStats(attacker).mBrutalGuardBreakingAttack);
            return true;
        }
    }

    bool blockVanillaMeleeAttack(const MWWorld::Ptr& attacker, const MWWorld::Ptr& blocker, const MWWorld::Ptr& weapon,
        float damage, float attackStrength)
    {
        if (!blocker.getClass().hasInventoryStore(blocker))
            return false;

        MWMechanics::CreatureStats& blockerStats = blocker.getClass().getCreatureStats(blocker);

        if (blockerStats.getKnockedDown() // Used for both knockout or knockdown
            || blockerStats.getHitRecovery() || blockerStats.isParalyzed())
            return false;

        if (!MWBase::Environment::get().getMechanicsManager()->isReadyToBlock(blocker))
            return false;

        MWWorld::InventoryStore& inv = blocker.getClass().getInventoryStore(blocker);
        MWWorld::ContainerStoreIterator shield = inv.getSlot(MWWorld::InventoryStore::Slot_CarriedLeft);
        if (shield == inv.end() || shield->getType() != ESM::Armor::sRecordId)
            return false;

        if (!blocker.getRefData().getBaseNode())
            return false; // shouldn't happen

        float angleDegrees = osg::RadiansToDegrees(signedAngleRadians(
            (attacker.getRefData().getPosition().asVec3() - blocker.getRefData().getPosition().asVec3()),
            blocker.getRefData().getBaseNode()->getAttitude() * osg::Vec3f(0, 1, 0), osg::Vec3f(0, 0, 1)));

        const MWWorld::Store<ESM::GameSetting>& gmst
            = MWBase::Environment::get().getESMStore()->get<ESM::GameSetting>();
        static const float fCombatBlockLeftAngle = gmst.find("fCombatBlockLeftAngle")->mValue.getFloat();
        if (angleDegrees < fCombatBlockLeftAngle)
            return false;
        static const float fCombatBlockRightAngle = gmst.find("fCombatBlockRightAngle")->mValue.getFloat();
        if (angleDegrees > fCombatBlockRightAngle)
            return false;

        MWMechanics::CreatureStats& attackerStats = attacker.getClass().getCreatureStats(attacker);

        float blockTerm = blocker.getClass().getSkill(blocker, ESM::Skill::Block)
            + 0.2f * blockerStats.getAttribute(ESM::Attribute::Agility).getModified()
            + 0.1f * blockerStats.getAttribute(ESM::Attribute::Luck).getModified();
        float enemySwing = attackStrength;
        static const float fSwingBlockMult = gmst.find("fSwingBlockMult")->mValue.getFloat();
        static const float fSwingBlockBase = gmst.find("fSwingBlockBase")->mValue.getFloat();
        float swingTerm = enemySwing * fSwingBlockMult + fSwingBlockBase;

        float blockerTerm = blockTerm * swingTerm;
        if (blocker.getClass().getMovementSettings(blocker).mPosition[1] <= 0)
        {
            static const float fBlockStillBonus = gmst.find("fBlockStillBonus")->mValue.getFloat();
            blockerTerm *= fBlockStillBonus;
        }
        blockerTerm *= blockerStats.getFatigueTerm();

        float attackerSkill = 0;
        if (weapon.isEmpty())
            attackerSkill = attacker.getClass().getSkill(attacker, ESM::Skill::HandToHand);
        else
            attackerSkill = attacker.getClass().getSkill(attacker, weapon.getClass().getEquipmentSkill(weapon));
        float attackerTerm = attackerSkill + 0.2f * attackerStats.getAttribute(ESM::Attribute::Agility).getModified()
            + 0.1f * attackerStats.getAttribute(ESM::Attribute::Luck).getModified();
        attackerTerm *= attackerStats.getFatigueTerm();

        static const int iBlockMaxChance = gmst.find("iBlockMaxChance")->mValue.getInteger();
        static const int iBlockMinChance = gmst.find("iBlockMinChance")->mValue.getInteger();
        int x = std::clamp(static_cast<int>(blockerTerm - attackerTerm), iBlockMinChance, iBlockMaxChance);

        auto& prng = MWBase::Environment::get().getWorld()->getPrng();
        if (Misc::Rng::roll0to99(prng) < x)
        {
            MWBase::SoundManager* sndMgr = MWBase::Environment::get().getSoundManager();
            const ESM::RefId skill = shield->getClass().getEquipmentSkill(*shield);
            if (skill == ESM::Skill::LightArmor)
                sndMgr->playSound3D(blocker, ESM::RefId::stringRefId("Light Armor Hit"), 1.0f, 1.0f);
            else if (skill == ESM::Skill::MediumArmor)
                sndMgr->playSound3D(blocker, ESM::RefId::stringRefId("Medium Armor Hit"), 1.0f, 1.0f);
            else if (skill == ESM::Skill::HeavyArmor)
                sndMgr->playSound3D(blocker, ESM::RefId::stringRefId("Heavy Armor Hit"), 1.0f, 1.0f);

            // Reduce shield durability by incoming damage
            int shieldhealth = shield->getClass().getItemHealth(*shield);

            shieldhealth -= std::min(shieldhealth, int(damage));
            shield->getCellRef().setCharge(shieldhealth);
            if (shieldhealth == 0)
                inv.unequipItem(*shield);
            // Reduce blocker fatigue
            static const float fFatigueBlockBase = gmst.find("fFatigueBlockBase")->mValue.getFloat();
            static const float fFatigueBlockMult = gmst.find("fFatigueBlockMult")->mValue.getFloat();
            static const float fWeaponFatigueBlockMult = gmst.find("fWeaponFatigueBlockMult")->mValue.getFloat();
            MWMechanics::DynamicStat<float> fatigue = blockerStats.getFatigue();
            float normalizedEncumbrance = blocker.getClass().getNormalizedEncumbrance(blocker);
            normalizedEncumbrance = std::min(1.f, normalizedEncumbrance);
            float fatigueLoss = fFatigueBlockBase + normalizedEncumbrance * fFatigueBlockMult;
            if (!weapon.isEmpty())
                fatigueLoss += weapon.getClass().getWeight(weapon) * attackStrength * fWeaponFatigueBlockMult;
            fatigue.setCurrent(fatigue.getCurrent() - fatigueLoss);
            blockerStats.setFatigue(fatigue);

            blockerStats.setBlock(true);

            if (blocker == getPlayer())
                blocker.getClass().skillUsageSucceeded(blocker, ESM::Skill::Block, ESM::Skill::Block_Success);

            return true;
        }
        return false;
    }

    bool isBrutalChargedAttack(const MWWorld::Ptr& attacker, float strength)
    {
        return brutalGuardBreakingAttack(attacker, strength);
    }

    bool blockMeleeAttack(const MWWorld::Ptr& attacker, const MWWorld::Ptr& blocker, const MWWorld::Ptr& weapon,
        float damage, float attackStrength, const osg::Vec3f& hitPosition, int contactPart)
    {
        auto& blockerStats = blocker.getClass().getCreatureStats(blocker);
        const auto& attackerStats = attacker.getClass().getCreatureStats(attacker);
        const bool adaptiveNpc = blocker.getClass().isNpc() && blocker != getPlayer()
            && Settings::game().mDarkMessiahNpcCombat;
        if (adaptiveNpc && attacker == getPlayer() && attackerStats.mBrutalFastStrike && !weapon.isEmpty())
        {
            const double now = MWBase::Environment::get().getWorld()->getTimeManager()->getSimulationTime();
            blockerStats.mBrutalFastPressure = now - blockerStats.mBrutalLastFastPressureAt <= 1.8
                ? std::min(3, blockerStats.mBrutalFastPressure + 1) : 1;
            blockerStats.mBrutalLastFastPressureAt = now;
            if (blockerStats.mBrutalFastPressure == 2)
                blockerStats.mBrutalGuardReactAt = now + .12;
        }
        const bool npcGuard = adaptiveNpc && blockerStats.mBrutalNpcGuardRequested && blockerStats.mBrutalBlockRaised;
        bool carriesShield = false;
        if (blocker.getClass().hasInventoryStore(blocker))
        {
            const auto& inventory = blocker.getClass().getInventoryStore(blocker);
            const auto left = inventory.getSlot(MWWorld::InventoryStore::Slot_CarriedLeft);
            const auto right = inventory.getSlot(MWWorld::InventoryStore::Slot_CarriedRight);
            const bool handsOccupied = blockerStats.getDrawState() == DrawState::Weapon && right != inventory.end()
                && right->getType() == ESM::Weapon::sRecordId
                && (getWeaponType(right->get<ESM::Weapon>()->mBase->mData.mType)->mFlags & ESM::WeaponType::TwoHanded);
            carriesShield = left != inventory.end() && left->getType() == ESM::Armor::sRecordId && !handsOccupied;
        }
        const bool playerGuard = blocker == getPlayer() && (Settings::game().mPlayerActiveShieldBlock
            || (!carriesShield && Settings::game().mDarkMessiahSwordParry));
        if (!playerGuard && !npcGuard && !Settings::game().mBrutalMeleeGeometry)
            return blockVanillaMeleeAttack(attacker, blocker, weapon, damage, attackStrength);
        if (!blocker.getClass().hasInventoryStore(blocker))
            return false;

        blockerStats.mBrutalPerfectShieldBlock = false;

        if (blockerStats.getKnockedDown() // Used for both knockout or knockdown
            || blockerStats.getHitRecovery() || blockerStats.isParalyzed())
            return false;

        const bool activeBlock = playerGuard || npcGuard;
        if (activeBlock && !blockerStats.mBrutalBlockRaised)
            return false;

        if (!npcGuard && !MWBase::Environment::get().getMechanicsManager()->isReadyToBlock(blocker))
            return false;

        if (adaptiveNpc && MWBase::Environment::get().getWorld()->getTimeManager()->getSimulationTime()
                < blockerStats.mBrutalGuardBrokenUntil)
            return false;
        if (activeBlock)
        {
            if (!isInFrontalGuard(attacker, blocker))
            {
                if (playerGuard && blockerStats.mBrutalParry)
                    blockerStats.mBrutalParryResult = BrutalParry_Broken;
                return false;
            }
            const double now = MWBase::Environment::get().getWorld()->getTimeManager()->getSimulationTime();
            const bool timedPlayerGuard = playerGuard
                && now - blockerStats.mBrutalBlockRaisedAt <= Settings::game().mDarkMessiahParryWindow.get();
            if (now < blockerStats.mBrutalGuardBrokenUntil)
                return false;
            // A wounded NPC's guard gives way: Dark Messiah's WoundedParryBreakChance (0.5) for a
            // sword parry and WoundedBlockBreakChance (0.8) for a shield block.
            const bool woundedGuardBreak = npcGuard && isBrutalWoundedNpc(blocker)
                && Misc::Rng::rollClosedProbability(MWBase::Environment::get().getWorld()->getPrng())
                    < (blockerStats.mBrutalParry ? .5f : .8f);
            // A timed player guard can deflect even when it cannot pay the
            // full charge cost. The ensuing impact still drains the reserve.
            const bool chargeBreaksGuard = brutalGuardBreakingAttack(attacker, attackStrength)
                && (npcGuard || (!timedPlayerGuard && !canAbsorbChargedAttack(blockerStats)));
            if (chargeBreaksGuard || woundedGuardBreak)
            {
                spendBrutalBlockStamina(attacker, blocker, true);
                openBrutalGuard(blocker);
                if (Settings::game().mBrutalCombatSounds)
                    MWBase::Environment::get().getSoundManager()->playSound3D(blocker,
                        ESM::RefId::stringRefId("Brutal Sword Contact"), 1.f, 1.f);
                return false; // The heavy blow opens the guard and reaches the body.
            }
        }
        if (playerGuard && blockerStats.mBrutalParry)
            return parryMeleeAttack(attacker, blocker, damage);
        if (npcGuard)
        {
            // Vanilla's random blood position is not a guard contact. With
            // exact body targeting disabled, getHitContact still resolves the
            // actual aimed guard surface before returning a blocked hit.
            if (!Settings::game().mBrutalMeleeGeometry && !isAdaptiveGuardSurface(blocker, contactPart))
                return false;
            const auto* animation = dynamic_cast<const MWRender::NpcAnimation*>(
                MWBase::Environment::get().getWorld()->getAnimation(blocker));
            if (!animation)
                return false;
            const auto origin = getMeleeAim(attacker).first;
            const auto contact = animation->traceNpcGuardMeshForCombat(origin, hitPosition);
            logNpcGuardContact(blocker, origin, hitPosition, contact);
            // Allow a small contact margin for the striking blade.
            // A guard still being raised only blocks once its item gets there.
            if (!contact || !std::isfinite(contact->gap) || contact->gap > sNpcGuardContactMargin)
                return false;
        }
        if (npcGuard && blockerStats.mBrutalParry)
        {
            auto& inventory = blocker.getClass().getInventoryStore(blocker);
            const auto right = inventory.getSlot(MWWorld::InventoryStore::Slot_CarriedRight);
            if (right == inventory.end() || right->getType() != ESM::Weapon::sRecordId)
                return false;
            MWWorld::Ptr parrying = *right;
            reduceWeaponCondition(damage, true, parrying, blocker);
            if (Settings::game().mBrutalCombatSounds)
                MWBase::Environment::get().getSoundManager()->playSound3D(blocker,
                    ESM::RefId::stringRefId("Brutal Sword Contact"), 1.f, 1.f);
            spendBrutalBlockStamina(attacker, blocker, false);
            return true;
        }

        MWWorld::InventoryStore& inv = blocker.getClass().getInventoryStore(blocker);
        MWWorld::ContainerStoreIterator shield = inv.getSlot(MWWorld::InventoryStore::Slot_CarriedLeft);
        if (shield == inv.end() || shield->getType() != ESM::Armor::sRecordId)
            return false;

        if (!blocker.getRefData().getBaseNode())
            return false; // shouldn't happen

        if (activeBlock)
        {
            // Dark Messiah: the player's held shield blocks every blow from a frontal arc.
            if (!isInFrontalGuard(attacker, blocker))
                return false;
        }
        // Other blockers: only the nearest actual shield surface can block a melee hit. This rule is
        // independent of the optional accuracy setting and never turns a body hit into an RNG block.
        else if (contactPart != ESM::PRT_Shield)
            return false;
        if (adaptiveNpc && !npcGuard && brutalGuardBreakingAttack(attacker, attackStrength))
        {
            spendBrutalBlockStamina(attacker, blocker, true);
            openBrutalGuard(blocker);
            return false;
        }
        const MWWorld::Store<ESM::GameSetting>& gmst
            = MWBase::Environment::get().getESMStore()->get<ESM::GameSetting>();
        {
            MWBase::SoundManager* sndMgr = MWBase::Environment::get().getSoundManager();
            if (Settings::game().mBrutalCombatSounds)
            {
                // Dark Messiah has shield sounds for metal and wood. Morrowind records no
                // material, so match the metal families also used for sparks.
                const auto* armor = shield->get<ESM::Armor>()->mBase;
                const std::string name
                    = Misc::StringUtils::lowerCase(armor->mId.getRefIdString() + " " + armor->mModel);
                bool metal = false;
                for (std::string_view family : { "iron", "steel", "silver", "ebony", "dwemer", "dwarven", "daedric",
                         "adamantium", "orcish" })
                    metal = metal || name.find(family) != std::string::npos;
                // Weapon blows against Dark Messiah's held player guard use Sword_on_Sword.
                const char* id = activeBlock && !weapon.isEmpty() ? "Brutal Sword Contact"
                    : metal                                       ? "Brutal Shield Metal"
                                                                  : "Brutal Shield Wood";
                sndMgr->playSound3D(blocker, ESM::RefId::stringRefId(id), 1.0f, 1.0f);
            }
            else
            {
                const ESM::RefId skill = shield->getClass().getEquipmentSkill(*shield);
                if (skill == ESM::Skill::LightArmor)
                    sndMgr->playSound3D(blocker, ESM::RefId::stringRefId("Light Armor Hit"), 1.0f, 1.0f);
                else if (skill == ESM::Skill::MediumArmor)
                    sndMgr->playSound3D(blocker, ESM::RefId::stringRefId("Medium Armor Hit"), 1.0f, 1.0f);
                else if (skill == ESM::Skill::HeavyArmor)
                    sndMgr->playSound3D(blocker, ESM::RefId::stringRefId("Heavy Armor Hit"), 1.0f, 1.0f);
            }

            // Reduce shield durability by incoming damage
            int shieldhealth = shield->getClass().getItemHealth(*shield);

            shieldhealth -= std::min(shieldhealth, int(damage));
            shield->getCellRef().setCharge(shieldhealth);
            if (shieldhealth == 0)
                inv.unequipItem(*shield);
            // Reduce blocker fatigue
            static const float fFatigueBlockBase = gmst.find("fFatigueBlockBase")->mValue.getFloat();
            static const float fFatigueBlockMult = gmst.find("fFatigueBlockMult")->mValue.getFloat();
            static const float fWeaponFatigueBlockMult = gmst.find("fWeaponFatigueBlockMult")->mValue.getFloat();
            MWMechanics::DynamicStat<float> fatigue = blockerStats.getFatigue();
            float normalizedEncumbrance = blocker.getClass().getNormalizedEncumbrance(blocker);
            normalizedEncumbrance = std::min(1.f, normalizedEncumbrance);
            float fatigueLoss = fFatigueBlockBase + normalizedEncumbrance * fFatigueBlockMult;
            if (!weapon.isEmpty())
                fatigueLoss += weapon.getClass().getWeight(weapon) * attackStrength * fWeaponFatigueBlockMult;
            // Disabling the replacement stamina system restores Morrowind's
            // original shield-block cost instead of making shield impacts free.
            if (!Settings::game().mBrutalStamina || (!activeBlock && !adaptiveNpc))
            {
                fatigue.setCurrent(fatigue.getCurrent() - fatigueLoss);
                blockerStats.setFatigue(fatigue);
            }
            else
                spendBrutalBlockStamina(attacker, blocker,
                    attacker.getClass().getCreatureStats(attacker).mBrutalGuardBreakingAttack);

            if (activeBlock)
            {
                blockerStats.mBrutalBlockImpact = true; // Held-block impact instead of the vanilla shield recoil.
                // A shield raised within the parry/clinch window before the blow is a perfect block.
                const double now = MWBase::Environment::get().getWorld()->getTimeManager()->getSimulationTime();
                blockerStats.mBrutalPerfectShieldBlock
                    = now - blockerStats.mBrutalBlockRaisedAt <= Settings::game().mDarkMessiahParryWindow.get();
            }
            else
                blockerStats.setBlock(true);

            if (blocker == getPlayer())
                blocker.getClass().skillUsageSucceeded(blocker, ESM::Skill::Block, ESM::Skill::Block_Success);

            return true;
        }
    }

    bool isNormalWeapon(const MWWorld::Ptr& weapon)
    {
        if (weapon.isEmpty())
            return false;

        const int flags = weapon.get<ESM::Weapon>()->mBase->mData.mFlags;
        bool isSilver = flags & ESM::Weapon::Silver;
        bool isMagical = flags & ESM::Weapon::Magical;
        bool isEnchanted = !weapon.getClass().getEnchantment(weapon).empty();

        return !isSilver && !isMagical && (!isEnchanted || !Settings::game().mEnchantedWeaponsAreMagical);
    }

    void resistNormalWeapon(
        const MWWorld::Ptr& actor, const MWWorld::Ptr& attacker, const MWWorld::Ptr& weapon, float& damage)
    {
        if (weapon.isEmpty() || !isNormalWeapon(weapon))
            return;

        const MWMechanics::MagicEffects& effects = actor.getClass().getCreatureStats(actor).getMagicEffects();
        const float resistance = effects.getOrDefault(ESM::MagicEffect::ResistNormalWeapons).getMagnitude() / 100.f;
        const float weakness = effects.getOrDefault(ESM::MagicEffect::WeaknessToNormalWeapons).getMagnitude() / 100.f;

        damage *= 1.f - std::min(1.f, resistance - weakness);

        if (resistance - weakness >= 1.f && attacker == getPlayer())
            MWBase::Environment::get().getWindowManager()->messageBox("#{sMagicTargetResistsWeapons}");
    }

    void applyWerewolfDamageMult(const MWWorld::Ptr& actor, const MWWorld::Ptr& weapon, float& damage)
    {
        if (damage == 0 || weapon.isEmpty() || !actor.getClass().isNpc())
            return;

        const int flags = weapon.get<ESM::Weapon>()->mBase->mData.mFlags;
        bool isSilver = flags & ESM::Weapon::Silver;

        if (isSilver && actor.getClass().getNpcStats(actor).isWerewolf())
        {
            const MWWorld::ESMStore& store = *MWBase::Environment::get().getESMStore();
            damage *= store.get<ESM::GameSetting>().find("fWereWolfSilverWeaponDamageMult")->mValue.getFloat();
        }
    }

    namespace
    {
        // A held Dark Messiah guard (the player's, or an adaptive NPC's raised guard)
        // also meets projectiles from its frontal arc. A shield stops arrows, bolts and
        // thrown weapons. A weapon guard turns thrown weapons, and arrows or bolts only
        // when it was raised within the parry window. Missiles cost the blocker half a light
        // guard impact and the shooter nothing.
        // Half the cost of an ordinary blocked blow: a guard can hold out under a volley.
        constexpr float projectileBlockStaminaFraction = 1.f / 18.f;

        bool isMetalShield(const ESM::Armor& armor)
        {
            const std::string name = Misc::StringUtils::lowerCase(armor.mId.getRefIdString() + " " + armor.mModel);
            for (std::string_view family : { "iron", "steel", "silver", "ebony", "dwemer", "dwarven", "daedric",
                     "adamantium", "orcish", "imperial", "templar", "dragonscale", "indoril", "her_hand", "glass" })
                if (name.find(family) != std::string::npos)
                    return true;
            return false;
        }

        // Whether a missile's path crosses the blocker's shield as it is held now.
        bool flightMeetsShield(const MWWorld::Ptr& blocker, const std::pair<osg::Vec3f, osg::Vec3f>& flight)
        {
            MWBase::World* world = MWBase::Environment::get().getWorld();
            if (blocker == getPlayer())
            {
                // The first-person shield is drawn at the eye in its own field of view, not where a body holds
                // it. The player's shield is a disc held before the face and chest, turning with the view.
                const ESM::Position& position = blocker.getRefData().getPosition();
                const float yaw = position.rot[2];
                const float pitch = position.rot[0];
                const osg::Vec3f forward(
                    std::sin(yaw) * std::cos(pitch), std::cos(yaw) * std::cos(pitch), -std::sin(pitch));
                const osg::Vec3f up(std::sin(yaw) * std::sin(pitch), std::cos(yaw) * std::sin(pitch), std::cos(pitch));
                const float height = 2.f * world->getHalfExtents(blocker).z();
                const osg::Vec3f eye = position.asVec3() + osg::Vec3f(0.f, 0.f, height * .92f);
                constexpr float reach = 30.f;
                constexpr float below = 24.f;
                constexpr float radius = 30.f;
                const osg::Vec3f centre = eye + forward * reach - up * below;
                const osg::Vec3f path = flight.second - flight.first;
                const float across = path * forward;
                if (std::abs(across) < 1e-4f)
                    return false;
                const float t = ((centre - flight.first) * forward) / across;
                if (t < 0.f || t > 1.f)
                    return false;
                return (flight.first + path * t - centre).length() <= radius;
            }
            auto* animation = dynamic_cast<MWRender::NpcAnimation*>(world->getAnimation(blocker));
            if (animation == nullptr)
                return false;
            // An NPC's fitted guard keeps the shield pose it is shown in.
            if (blocker != getPlayer() && animation->hasNpcGuardMeshForCombat())
            {
                const auto trace = animation->traceNpcGuardMeshForCombat(flight.first, flight.second);
                return trace && trace->gap <= sNpcProjectileGuardContactMargin;
            }
            osg::Node* shield = animation->getPartNode(ESM::PRT_Shield);
            return shield != nullptr && animation->intersectNode(shield, flight.first, flight.second).has_value();
        }

        ProjectileOutcome blockProjectile(const MWWorld::Ptr& attacker, const MWWorld::Ptr& blocker,
            const MWWorld::Ptr& projectile, float damage,
            const std::pair<osg::Vec3f, osg::Vec3f>* flight)
        {
            constexpr ProjectileOutcome unblocked = ProjectileOutcome::Body;
            if (attacker.isEmpty() || !blocker.getClass().isActor() || !blocker.getClass().hasInventoryStore(blocker))
                return unblocked;
            auto& stats = blocker.getClass().getCreatureStats(blocker);
            const bool player = blocker == getPlayer();
            const bool npcGuard = !player && blocker.getClass().isNpc() && Settings::game().mDarkMessiahNpcCombat
                && stats.mBrutalNpcGuardRequested;
            if (!stats.mBrutalBlockRaised || (!player && !npcGuard))
                return unblocked;
            if (stats.isDead() || stats.getKnockedDown() || stats.getHitRecovery() || stats.isParalyzed())
                return unblocked;
            const double now = MWBase::Environment::get().getWorld()->getTimeManager()->getSimulationTime();
            if (now < stats.mBrutalGuardBrokenUntil || !isInFrontalGuard(attacker, blocker))
                return unblocked;

            auto& inventory = blocker.getClass().getInventoryStore(blocker);
            const auto* projectileType = getWeaponType(projectile.get<ESM::Weapon>()->mBase->mData.mType);
            const bool thrown = projectileType->mWeaponClass == ESM::WeaponType::Thrown;
            ProjectileOutcome outcome = ProjectileOutcome::Parried;
            auto sounds = MWBase::Environment::get().getSoundManager();
            if (stats.mBrutalParry)
            {
                const bool timed = now - stats.mBrutalBlockRaisedAt <= Settings::game().mDarkMessiahParryWindow.get();
                if (!thrown && !timed)
                    return unblocked;
                const auto right = inventory.getSlot(MWWorld::InventoryStore::Slot_CarriedRight);
                if (right == inventory.end() || right->getType() != ESM::Weapon::sRecordId)
                    return unblocked;
                MWWorld::Ptr parrying = *right;
                reduceWeaponCondition(damage, true, parrying, blocker);
                if (Settings::game().mBrutalCombatSounds)
                    sounds->playSound3D(blocker, ESM::RefId::stringRefId("Brutal Sword Contact"), 1.f, 1.f);
                if (player)
                    stats.mBrutalParryResult = timed ? BrutalParry_Timed : BrutalParry_Parried;
            }
            else
            {
                const auto shield = inventory.getSlot(MWWorld::InventoryStore::Slot_CarriedLeft);
                if (shield == inventory.end() || shield->getType() != ESM::Armor::sRecordId)
                    return unblocked;
                // A physical missile is stopped only by actually striking the shield.
                if (flight != nullptr && !flightMeetsShield(blocker, *flight))
                    return unblocked;
                const auto* armor = shield->get<ESM::Armor>()->mBase;
                const bool metal = isMetalShield(*armor);
                outcome = metal ? ProjectileOutcome::MetalShield : ProjectileOutcome::Shield;
                if (Settings::game().mBrutalCombatSounds)
                {
                    sounds->playSound3D(blocker,
                        ESM::RefId::stringRefId(metal ? "Brutal Shield Metal" : "Brutal Shield Wood"), 1.f, 1.f);
                }
                else
                {
                    const ESM::RefId skill = shield->getClass().getEquipmentSkill(*shield);
                    const char* id = skill == ESM::Skill::LightArmor ? "Light Armor Hit"
                        : skill == ESM::Skill::MediumArmor           ? "Medium Armor Hit"
                                                                     : "Heavy Armor Hit";
                    sounds->playSound3D(blocker, ESM::RefId::stringRefId(id), 1.f, 1.f);
                }
                int shieldHealth = shield->getClass().getItemHealth(*shield);
                shieldHealth -= std::min(shieldHealth, int(damage));
                shield->getCellRef().setCharge(shieldHealth);
                if (shieldHealth == 0)
                    inventory.unequipItem(*shield);
            }
            ++stats.mBrutalGuardBlocks;
            if (Settings::game().mBrutalStamina
                && !(player && MWBase::Environment::get().getWorld()->getGodModeState()))
            {
                auto fatigue = stats.getFatigue();
                fatigue.setCurrent(std::max(
                    0.f, fatigue.getCurrent() - blockStaminaCost(fatigue, projectileBlockStaminaFraction)));
                stats.setFatigue(fatigue);
                if (fatigue.getCurrent() <= 0.f)
                    openBrutalGuard(blocker);
            }
            stats.mBrutalPerfectShieldBlock = false;
            if (player)
            {
                stats.mBrutalBlockImpact = true;
                blocker.getClass().skillUsageSucceeded(blocker, ESM::Skill::Block, ESM::Skill::Block_Success);
            }
            return outcome;
        }
    }

    bool physicalProjectiles()
    {
        return Settings::game().mBrutalEnabled && Settings::game().mBrutalPhysicalProjectiles;
    }

    float getProjectileLaunchSpeed(bool thrown, float attackStrength)
    {
        const auto& gmst = MWBase::Environment::get().getESMStore()->get<ESM::GameSetting>();
        const float minimum
            = gmst.find(thrown ? "fThrownWeaponMinSpeed" : "fProjectileMinSpeed")->mValue.getFloat();
        const float maximum
            = gmst.find(thrown ? "fThrownWeaponMaxSpeed" : "fProjectileMaxSpeed")->mValue.getFloat();
        float speed = minimum + (maximum - minimum) * attackStrength;
        if (!thrown && physicalProjectiles())
            speed *= Settings::game().mBrutalProjectileSpeed;
        return speed;
    }

    float getProjectileGravity()
    {
        const float g = Constants::GravityConst * Constants::UnitsPerMeter;
        return g * (physicalProjectiles() ? Settings::game().mBrutalProjectileGravity.get() : 0.1f);
    }

    ProjectileOutcome projectileHit(const MWWorld::Ptr& attacker, const MWWorld::Ptr& victim, MWWorld::Ptr weapon,
        const MWWorld::Ptr& projectile, const osg::Vec3f& hitPosition, float attackStrength,
        const std::pair<osg::Vec3f, osg::Vec3f>* flight)
    {
        MWBase::World* world = MWBase::Environment::get().getWorld();
        const MWWorld::Store<ESM::GameSetting>& gmst = world->getStore().get<ESM::GameSetting>();

        bool validVictim = !victim.isEmpty() && victim.getClass().isActor();

        ESM::RefId weaponSkill = ESM::Skill::Marksman;
        if (!weapon.isEmpty())
            weaponSkill = weapon.getClass().getEquipmentSkill(weapon);

        float damage = 0.f;
        if (validVictim)
        {
            if (attacker == getPlayer())
                MWBase::Environment::get().getWindowManager()->setEnemy(victim);

            int skillValue = static_cast<int>(attacker.getClass().getSkill(attacker, weaponSkill));

            if (!physicalProjectiles() && Misc::Rng::roll0to99(world->getPrng()) >= getHitChance(attacker, victim, skillValue))
            {
                MWBase::Environment::get().getLuaManager()->onHit(attacker, victim, weapon, projectile, 0,
                    attackStrength, damage, false, hitPosition, false, MWMechanics::DamageSourceType::Ranged);
                MWMechanics::reduceWeaponCondition(damage, false, weapon, attacker);
                return ProjectileOutcome::Missed;
            }

            {
                const auto& attack = weapon.get<ESM::Weapon>()->mBase->mData.mChop;
                damage = attack[0] + ((attack[1] - attack[0]) * attackStrength); // Bow/crossbow damage
            }
            {
                // Arrow/bolt damage
                // NB in case of thrown weapons, we are applying the damage twice since projectile == weapon
                const auto& attack = projectile.get<ESM::Weapon>()->mBase->mData.mChop;
                damage += attack[0] + ((attack[1] - attack[0]) * attackStrength);
            }
            adjustWeaponDamage(damage, weapon, attacker);

            if (const ProjectileOutcome blocked = blockProjectile(attacker, victim, projectile, damage, flight);
                blocked != ProjectileOutcome::Body)
            {
                reduceWeaponCondition(damage, true, weapon, attacker);
                MWBase::Environment::get().getLuaManager()->onHit(attacker, victim, weapon, projectile, 0,
                    attackStrength, 0.f, true, hitPosition, true, MWMechanics::DamageSourceType::Ranged, 0, false,
                    true);
                return blocked;
            }
        }

        reduceWeaponCondition(damage, validVictim, weapon, attacker);

        if (validVictim)
        {
            if (weapon == projectile || Settings::game().mOnlyAppropriateAmmunitionBypassesResistance
                || isNormalWeapon(weapon))
                resistNormalWeapon(victim, attacker, projectile, damage);
            applyWerewolfDamageMult(victim, projectile, damage);

            if (attacker == getPlayer())
                attacker.getClass().skillUsageSucceeded(attacker, weaponSkill, ESM::Skill::Weapon_SuccessfulHit);

            const MWMechanics::AiSequence& sequence = victim.getClass().getCreatureStats(victim).getAiSequence();
            bool unaware = attacker == getPlayer() && !sequence.isInCombat()
                && !MWBase::Environment::get().getMechanicsManager()->awarenessCheck(attacker, victim);
            bool knockedDown = victim.getClass().getCreatureStats(victim).getKnockedDown();
            if (knockedDown || unaware)
            {
                static const float fCombatKODamageMult = gmst.find("fCombatKODamageMult")->mValue.getFloat();
                damage *= fCombatKODamageMult;
                if (!knockedDown)
                    MWBase::Environment::get().getSoundManager()->playSound3D(
                        victim, ESM::RefId::stringRefId("critical damage"), 1.0f, 1.0f);
            }
        }

        // Apply "On hit" effect of the projectile
        bool appliedEnchantment = applyOnStrikeEnchantment(attacker, victim, projectile, hitPosition, true);

        if (validVictim)
        {
            // Non-enchanted arrows shot at enemies have a chance to turn up in their inventory.
            // Physical missiles are kept by the projectile manager, which shows them stuck in the body.
            if (victim != getPlayer() && !appliedEnchantment && !physicalProjectiles())
            {
                static const float fProjectileThrownStoreChance
                    = gmst.find("fProjectileThrownStoreChance")->mValue.getFloat();
                if (Misc::Rng::rollProbability(world->getPrng()) < fProjectileThrownStoreChance / 100.f)
                    victim.getClass().getContainerStore(victim).add(projectile, 1);
            }

            MWBase::Environment::get().getLuaManager()->onHit(attacker, victim, weapon, projectile, 0, attackStrength,
                damage, true, hitPosition, true, MWMechanics::DamageSourceType::Ranged);
            return ProjectileOutcome::Body;
        }
        return ProjectileOutcome::NoActor;
    }

    float getHitChance(const MWWorld::Ptr& attacker, const MWWorld::Ptr& victim, int skillValue)
    {
        MWMechanics::CreatureStats& stats = attacker.getClass().getCreatureStats(attacker);
        const MWMechanics::MagicEffects& mageffects = stats.getMagicEffects();

        MWBase::World* world = MWBase::Environment::get().getWorld();
        const MWWorld::Store<ESM::GameSetting>& gmst = world->getStore().get<ESM::GameSetting>();

        float defenseTerm = 0;
        MWMechanics::CreatureStats& victimStats = victim.getClass().getCreatureStats(victim);
        if (victimStats.getFatigue().getCurrent() >= 0)
        {
            // Maybe we should keep an aware state for actors updated every so often instead of testing every time
            bool unaware = (!victimStats.getAiSequence().isInCombat()) && (attacker == getPlayer())
                && (!MWBase::Environment::get().getMechanicsManager()->awarenessCheck(attacker, victim));
            if (!(victimStats.getKnockedDown() || victimStats.isParalyzed() || unaware))
            {
                defenseTerm = victimStats.getEvasion();
            }
            static const float fCombatInvisoMult = gmst.find("fCombatInvisoMult")->mValue.getFloat();
            defenseTerm += std::min(100.f,
                fCombatInvisoMult
                    * victimStats.getMagicEffects().getOrDefault(ESM::MagicEffect::Chameleon).getMagnitude());
            defenseTerm += std::min(100.f,
                fCombatInvisoMult
                    * victimStats.getMagicEffects().getOrDefault(ESM::MagicEffect::Invisibility).getMagnitude());
        }
        float attackTerm = skillValue + (stats.getAttribute(ESM::Attribute::Agility).getModified() / 5.0f)
            + (stats.getAttribute(ESM::Attribute::Luck).getModified() / 10.0f);
        attackTerm *= stats.getFatigueTerm();
        attackTerm += mageffects.getOrDefault(ESM::MagicEffect::FortifyAttack).getMagnitude()
            - mageffects.getOrDefault(ESM::MagicEffect::Blind).getMagnitude();

        return round(attackTerm - defenseTerm);
    }

    void applyElementalShields(const MWWorld::Ptr& attacker, const MWWorld::Ptr& victim)
    {
        // Don't let elemental shields harm the player in god mode.
        bool godmode = attacker == getPlayer() && MWBase::Environment::get().getWorld()->getGodModeState();
        if (godmode)
            return;
        auto& prng = MWBase::Environment::get().getWorld()->getPrng();
        static const std::array<ESM::RefId, 3> elementalShieldEffects{ ESM::MagicEffect::FireShield,
            ESM::MagicEffect::LightningShield, ESM::MagicEffect::FrostShield };
        for (const auto elementalShieldEffect : elementalShieldEffects)
        {
            float magnitude = victim.getClass()
                                  .getCreatureStats(victim)
                                  .getMagicEffects()
                                  .getOrDefault(elementalShieldEffect)
                                  .getMagnitude();

            if (!magnitude)
                continue;

            CreatureStats& attackerStats = attacker.getClass().getCreatureStats(attacker);
            float saveTerm = attacker.getClass().getSkill(attacker, ESM::Skill::Destruction)
                + 0.2f * attackerStats.getAttribute(ESM::Attribute::Willpower).getModified()
                + 0.1f * attackerStats.getAttribute(ESM::Attribute::Luck).getModified();

            float fatigueMax = attackerStats.getFatigue().getModified();
            float fatigueCurrent = attackerStats.getFatigue().getCurrent();

            float normalisedFatigue = floor(fatigueMax) == 0 ? 1 : std::max(0.0f, (fatigueCurrent / fatigueMax));

            saveTerm *= 1.25f * normalisedFatigue;

            float x = std::max(0.f, saveTerm - Misc::Rng::roll0to99(prng));

            ESM::RefId element = ESM::MagicEffect::FireDamage;
            if (elementalShieldEffect == ESM::MagicEffect::LightningShield)
                element = ESM::MagicEffect::ShockDamage;
            if (elementalShieldEffect == ESM::MagicEffect::FrostShield)
                element = ESM::MagicEffect::FrostDamage;

            float elementResistance
                = MWMechanics::getEffectResistanceAttribute(element, &attackerStats.getMagicEffects());

            x = std::min(100.f, x + elementResistance);

            static const float fElementalShieldMult = MWBase::Environment::get()
                                                          .getESMStore()
                                                          ->get<ESM::GameSetting>()
                                                          .find("fElementalShieldMult")
                                                          ->mValue.getFloat();
            x = fElementalShieldMult * magnitude * (1.f - 0.01f * x);

            // Note swapped victim and attacker, since the attacker takes the damage here.
            x = scaleDamage(x, victim, attacker);

            MWMechanics::DynamicStat<float> health = attackerStats.getHealth();
            health.setCurrent(health.getCurrent() - x);
            attackerStats.setHealth(health);

            MWBase::Environment::get().getSoundManager()->playSound3D(
                attacker, ESM::RefId::stringRefId("Health Damage"), 1.0f, 1.0f);
        }
    }

    void reduceWeaponCondition(float damage, bool hit, MWWorld::Ptr& weapon, const MWWorld::Ptr& attacker)
    {
        if (weapon.isEmpty())
            return;

        if (!hit)
            damage = 0.f;

        const bool weaphashealth = weapon.getClass().hasItemHealth(weapon);
        if (weaphashealth)
        {
            int weaphealth = weapon.getClass().getItemHealth(weapon);

            bool godmode
                = attacker == MWMechanics::getPlayer() && MWBase::Environment::get().getWorld()->getGodModeState();

            // weapon condition does not degrade when godmode is on
            if (!godmode)
            {
                const float fWeaponDamageMult = MWBase::Environment::get()
                                                    .getESMStore()
                                                    ->get<ESM::GameSetting>()
                                                    .find("fWeaponDamageMult")
                                                    ->mValue.getFloat();
                float x = std::max(1.f, fWeaponDamageMult * damage);

                weaphealth -= std::min(int(x), weaphealth);
                weapon.getCellRef().setCharge(weaphealth);
            }

            // Weapon broken? unequip it
            if (weaphealth == 0)
                weapon = *attacker.getClass().getInventoryStore(attacker).unequipItem(weapon);
        }
    }

    void adjustWeaponDamage(float& damage, const MWWorld::Ptr& weapon, const MWWorld::Ptr& attacker)
    {
        if (weapon.isEmpty())
            return;

        const bool weaphashealth = weapon.getClass().hasItemHealth(weapon);
        if (weaphashealth)
        {
            damage *= weapon.getClass().getItemNormalizedHealth(weapon);
        }

        static const float fDamageStrengthBase = MWBase::Environment::get()
                                                     .getESMStore()
                                                     ->get<ESM::GameSetting>()
                                                     .find("fDamageStrengthBase")
                                                     ->mValue.getFloat();
        static const float fDamageStrengthMult = MWBase::Environment::get()
                                                     .getESMStore()
                                                     ->get<ESM::GameSetting>()
                                                     .find("fDamageStrengthMult")
                                                     ->mValue.getFloat();
        damage *= fDamageStrengthBase
            + (attacker.getClass().getCreatureStats(attacker).getAttribute(ESM::Attribute::Strength).getModified()
                * fDamageStrengthMult * 0.1f);
    }

    void getHandToHandDamage(
        const MWWorld::Ptr& attacker, const MWWorld::Ptr& victim, float& damage, bool& healthdmg, float attackStrength)
    {
        const MWWorld::ESMStore& store = *MWBase::Environment::get().getESMStore();
        static const float minstrike = store.get<ESM::GameSetting>().find("fMinHandToHandMult")->mValue.getFloat();
        static const float maxstrike = store.get<ESM::GameSetting>().find("fMaxHandToHandMult")->mValue.getFloat();
        damage = static_cast<float>(attacker.getClass().getSkill(attacker, ESM::Skill::HandToHand));
        damage *= minstrike + ((maxstrike - minstrike) * attackStrength);

        MWMechanics::CreatureStats& otherstats = victim.getClass().getCreatureStats(victim);
        healthdmg = otherstats.isParalyzed() || otherstats.getKnockedDown();
        bool isWerewolf = (attacker.getClass().isNpc() && attacker.getClass().getNpcStats(attacker).isWerewolf());

        // Options in the launcher's combo box: unarmedFactorsStrengthComboBox
        // 0 = Do not factor strength into hand-to-hand combat.
        // 1 = Factor into werewolf hand-to-hand combat.
        // 2 = Ignore werewolves.
        const int factorStrength = Settings::game().mStrengthInfluencesHandToHand;
        if (factorStrength == 1 || (factorStrength == 2 && !isWerewolf))
        {
            damage
                *= attacker.getClass().getCreatureStats(attacker).getAttribute(ESM::Attribute::Strength).getModified()
                / 40.0f;
        }

        if (isWerewolf)
        {
            healthdmg = true;
            // GLOB instead of GMST because it gets updated during a quest
            damage *= MWBase::Environment::get().getWorld()->getGlobalFloat(MWWorld::Globals::sWerewolfClawMult);
        }
        if (healthdmg)
        {
            static const float fHandtoHandHealthPer
                = store.get<ESM::GameSetting>().find("fHandtoHandHealthPer")->mValue.getFloat();
            damage *= fHandtoHandHealthPer;
        }

        MWBase::SoundManager* sndMgr = MWBase::Environment::get().getSoundManager();
        auto& prng = MWBase::Environment::get().getWorld()->getPrng();
        if (isWerewolf)
        {
            const ESM::Sound* sound = store.get<ESM::Sound>().searchRandom("WolfHit", prng);
            if (sound)
                sndMgr->playSound3D(victim, sound->mId, 1.0f, 1.0f);
        }
        else if (!healthdmg)
        {
            static const std::array<ESM::RefId, 2> sounds
                = { ESM::RefId::stringRefId("Hand To Hand Hit"), ESM::RefId::stringRefId("Hand To Hand Hit 2") };
            sndMgr->playSound3D(victim, sounds[Misc::Rng::rollDice(sounds.size(), prng)], 1.0f, 1.0f);
        }
    }

    void applyFatigueLoss(const MWWorld::Ptr& attacker, const MWWorld::Ptr& weapon, float attackStrength)
    {
        // somewhat of a guess, but using the weapon weight makes sense
        const MWWorld::Store<ESM::GameSetting>& store
            = MWBase::Environment::get().getESMStore()->get<ESM::GameSetting>();
        static const float fFatigueAttackBase = store.find("fFatigueAttackBase")->mValue.getFloat();
        static const float fFatigueAttackMult = store.find("fFatigueAttackMult")->mValue.getFloat();
        static const float fWeaponFatigueMult = store.find("fWeaponFatigueMult")->mValue.getFloat();
        CreatureStats& stats = attacker.getClass().getCreatureStats(attacker);
        MWMechanics::DynamicStat<float> fatigue = stats.getFatigue();
        const float normalizedEncumbrance = attacker.getClass().getNormalizedEncumbrance(attacker);

        bool godmode = attacker == MWMechanics::getPlayer() && MWBase::Environment::get().getWorld()->getGodModeState();
        // Dark Messiah's player swings cost no stamina; only sprint and kick use it (mod).
        const bool kinetic = attacker == MWMechanics::getPlayer() && Settings::game().mBrutalEnabled && Settings::game().mBrutalStamina;

        if (!godmode && !kinetic)
        {
            float fatigueLoss = fFatigueAttackBase + normalizedEncumbrance * fFatigueAttackMult;
            if (!weapon.isEmpty())
                fatigueLoss += weapon.getClass().getWeight(weapon) * attackStrength * fWeaponFatigueMult;
            fatigue.setCurrent(fatigue.getCurrent() - fatigueLoss);
            stats.setFatigue(fatigue);
        }
    }

    float getFightDistanceBias(const MWWorld::Ptr& actor1, const MWWorld::Ptr& actor2)
    {
        osg::Vec3f pos1(actor1.getRefData().getPosition().asVec3());
        osg::Vec3f pos2(actor2.getRefData().getPosition().asVec3());

        float d = getAggroDistance(actor1, pos1, pos2);

        static const int iFightDistanceBase = MWBase::Environment::get()
                                                  .getESMStore()
                                                  ->get<ESM::GameSetting>()
                                                  .find("iFightDistanceBase")
                                                  ->mValue.getInteger();
        static const float fFightDistanceMultiplier = MWBase::Environment::get()
                                                          .getESMStore()
                                                          ->get<ESM::GameSetting>()
                                                          .find("fFightDistanceMultiplier")
                                                          ->mValue.getFloat();

        return (iFightDistanceBase - fFightDistanceMultiplier * d);
    }

    float getAggroDistance(const MWWorld::Ptr& actor, const osg::Vec3f& lhs, const osg::Vec3f& rhs)
    {
        if (canActorMoveByZAxis(actor))
            return distanceIgnoreZ(lhs, rhs);
        return distance(lhs, rhs);
    }

    float getDistanceToBounds(const MWWorld::Ptr& actor, const MWWorld::Ptr& target)
    {
        osg::Vec3f actorPos(actor.getRefData().getPosition().asVec3());
        osg::Vec3f targetPos(target.getRefData().getPosition().asVec3());
        MWBase::World* world = MWBase::Environment::get().getWorld();

        float dist = (targetPos - actorPos).length();
        dist -= world->getHalfExtents(actor).y();
        dist -= world->getHalfExtents(target).y();
        return dist;
    }

    float getMeleeWeaponReach(const MWWorld::Ptr& actor, const MWWorld::Ptr& weapon)
    {
        MWBase::World* world = MWBase::Environment::get().getWorld();
        const MWWorld::Store<ESM::GameSetting>& store = world->getStore().get<ESM::GameSetting>();
        const float fCombatDistance = store.find("fCombatDistance")->mValue.getFloat();
        if (!weapon.isEmpty())
            return fCombatDistance * weapon.get<ESM::Weapon>()->mBase->mData.mReach;
        if (actor.getClass().isNpc())
            return fCombatDistance * store.find("fHandToHandReach")->mValue.getFloat();
        return fCombatDistance;
    }

    bool isInMeleeReach(const MWWorld::Ptr& actor, const MWWorld::Ptr& target, const float reach)
    {
        const float heightDiff = actor.getRefData().getPosition().pos[2] - target.getRefData().getPosition().pos[2];
        return std::abs(heightDiff) < reach && getDistanceToBounds(actor, target) < reach;
    }

    std::pair<osg::Vec3f, osg::Vec3f> getMeleeAim(const MWWorld::Ptr& actor)
    {
        auto world = MWBase::Environment::get().getWorld();
        const auto& position = actor.getRefData().getPosition();
        osg::Vec3f origin = position.asVec3()
            + osg::Vec3f(0,0,world->getHalfExtents(actor,true).z()*2.f*.85f);
        if (actor == getPlayer() && MWMechanics::brutalDmFirstPerson())
            if (const auto* animation = world->getAnimation(actor))
                if (const auto camera = animation->getBoneWorldMatrix("Camera"))
                    origin = camera->getTrans();
        const auto direction = osg::Quat(position.rot[2],osg::Vec3f(0,0,-1))
            * (osg::Quat(position.rot[0],osg::Vec3f(-1,0,0))*osg::Vec3f(0,1,0));
        return {origin,direction};
    }

    namespace
    {
        osg::Vec3f meleeTorsoPosition(const MWWorld::Ptr& target)
        {
            auto world = MWBase::Environment::get().getWorld();
            osg::Vec3f position = target.getRefData().getPosition().asVec3()
                + osg::Vec3f(0, 0, world->getHalfExtents(target, true).z() * 1.5f);
            const MWRender::Animation* animation = world->getAnimation(target);
            if (const auto* npc = dynamic_cast<const MWRender::NpcAnimation*>(animation))
                if (npc->getBodyAwarenessAnimation())
                    animation = npc->getBodyAwarenessAnimation();
            if (animation)
            {
                const auto spine = animation->getBoneWorldMatrix("Bip01 Spine1");
                const auto neck = animation->getBoneWorldMatrix("Bip01 Neck");
                if (spine && neck)
                    position = (spine->getTrans() + neck->getTrans()) * .5f;
            }
            return position;
        }
    }

    osg::Vec3f getMeleeTorsoPosition(const MWWorld::Ptr& actor)
    {
        return meleeTorsoPosition(actor);
    }

    bool hasReachableNpcGuardContact(const MWWorld::Ptr& actor, const MWWorld::Ptr& target, float reach,
        const osg::Vec3f& hitPosition, int contactPart)
    {
        if (!isAdaptiveGuardSurface(target, contactPart))
            return false;
        auto world = MWBase::Environment::get().getWorld();
        const osg::Vec3f origin = getMeleeAim(actor).first;
        const float length = reach + world->getHalfExtents(actor).y();
        const float distance = (hitPosition - origin).length();
        if (!std::isfinite(distance) || distance < .01f || distance > length + .05f)
            return false;
        const osg::Vec3f direction = (hitPosition - origin) / distance;
        const std::array<MWWorld::Ptr, 1> ignore{ actor };
        MWPhysics::RayCastingResult surface;
        if (!world->castRenderingRay(surface, origin, origin + direction * length,
                actor == getPlayer(), false, ignore, true)
            || surface.mHitObject != target || surface.mHitPart != contactPart)
            return false;
        const auto* animation = dynamic_cast<const MWRender::NpcAnimation*>(world->getAnimation(target));
        const auto contact = animation->traceNpcGuardMeshForCombat(origin, hitPosition);
        return contact && std::isfinite(contact->gap) && contact->gap <= sNpcGuardContactMargin
            && (contact->point - origin).length() <= length + .05f;
    }

    void refreshMeleeGuardContact(const MWWorld::Ptr& actor, MWWorld::Ptr& victim,
        osg::Vec3f& hitPosition, int& contactPart)
    {
        if (!Settings::game().mDarkMessiahNpcCombat || victim.isEmpty())
            return;
        contactPart = -1;
        auto world = MWBase::Environment::get().getWorld();
        const auto [origin, aimDirection] = getMeleeAim(actor);
        const osg::Vec3f toTorso = meleeTorsoPosition(victim) - origin;
        osg::Vec3f direction = actor == getPlayer() ? aimDirection : toTorso;
        if (direction.normalize() < .01f)
            return;
        MWWorld::Ptr weapon;
        if (actor.getClass().hasInventoryStore(actor))
        {
            auto& inventory = actor.getClass().getInventoryStore(actor);
            const auto slot = inventory.getSlot(actor.getClass().getCreatureStats(actor).mBrutalOffhandStrike
                    ? MWWorld::InventoryStore::Slot_CarriedLeft : MWWorld::InventoryStore::Slot_CarriedRight);
            if (slot != inventory.end() && slot->getType() == ESM::Weapon::sRecordId)
                weapon = *slot;
        }
        const float reach = getMeleeWeaponReach(actor, weapon);
        // Guard interception belongs in front of the selected body. An actor
        // further down the aim ray cannot cover a nearer vanilla target beside it.
        const float length = std::min(reach + world->getHalfExtents(actor).y(), toTorso * direction);
        if (length <= .01f)
            return;
        const std::array<MWWorld::Ptr, 1> ignore{ actor };
        MWPhysics::RayCastingResult surface;
        if (!world->castRenderingRay(surface, origin, origin + direction * length,
                actor == getPlayer(), false, ignore, true))
            return;
        if (surface.mHitObject == victim && isAdaptiveGuardSurface(victim, surface.mHitPart))
        {
            hitPosition = surface.mHitPos;
            contactPart = surface.mHitPart;
        }
        else if (!surface.mHitObject.isEmpty() && surface.mHitObject != victim
            && surface.mHitObject.getClass().isActor())
        {
            // Keep vanilla's chosen victim and its accuracy roll. Another actor
            // is cover, never a new recipient of the selected victim's hit.
            victim = {};
            hitPosition = {};
        }
    }

    namespace
    {
        // Half extents (degrees) of the visible view. A swing on screen crosses it
        // from edge to edge, so its contact area follows the current camera.
        struct SwingShape
        {
            float mYaw;
            float mPitch;
        };

        SwingShape getSwingShape()
        {
            const float pitch = std::clamp(Settings::camera().mFieldOfView.get(), 1.f, 179.f) * .5f;
            const float aspect = static_cast<float>(Settings::video().mResolutionX)
                / std::max(1, static_cast<int>(Settings::video().mResolutionY));
            const float yaw = osg::RadiansToDegrees(std::atan(std::tan(osg::DegreesToRadians(pitch)) * aspect));
            return { yaw, pitch };
        }

        // Rays end at the view edge: nothing off screen is struck. Skeleton points
        // lie inside the body, so the fallback area keeps a small margin.
        constexpr float sSwingRayMargin = 0.f;
        constexpr float sSwingAreaMargin = 6.f;

        // Cost of meeting a body at an offset (yaw right, pitch up; degrees). The
        // crosshair picks the target: nearest first, and leaving the stroke line
        // costs three times its travel, so a slash at head height meets the head
        // before a shoulder slightly below the line. Equal offsets go to the side
        // the blade reaches first: the imported Dark Messiah slash (bigslash_*_L,
        // impactdir actor-right) sweeps left to right; a chop falls from the top.
        float getSwingCost(std::string_view swing, float yaw, float pitch, bool reverseSlash = false)
        {
            constexpr float bladeTieBreak = .01f;
            const float slash = yaw * yaw + 9.f * pitch * pitch
                + ((reverseSlash ? yaw < 0.f : yaw > 0.f) ? bladeTieBreak : 0.f);
            const float chop = 9.f * yaw * yaw + pitch * pitch + (pitch < 0.f ? bladeTieBreak : 0.f);
            if (swing == "slash")
                return slash;
            if (swing == "chop")
                return chop;
            if (swing == "thrust")
                return yaw * yaw + pitch * pitch;
            return std::min(slash, chop);
        }

        // Symmetric 6-degree steps through zero, closed by the exact ends.
        std::vector<float> getSwingSteps(float low, float high)
        {
            std::vector<float> steps{ low, high };
            for (float step = 0.f; step < high; step += 6.f)
                steps.push_back(step);
            for (float step = -6.f; step > low; step -= 6.f)
                steps.push_back(step);
            return steps;
        }

        // Angular offsets swept from the aim for one player swing, nearest first.
        // A slash crosses the view, a chop falls through it and a thrust stays near
        // the point.
        std::vector<osg::Vec2f> getSwingFan(std::string_view swing, const SwingShape& shape, bool reverseSlash)
        {
            std::vector<osg::Vec2f> fan;
            const bool any = swing == "any";
            if (swing == "slash" || any)
                for (float yaw : getSwingSteps(-shape.mYaw - sSwingRayMargin, shape.mYaw + sSwingRayMargin))
                    for (float pitch : { -8.f, 0.f, 8.f })
                        fan.emplace_back(yaw, pitch);
            if (swing == "chop" || any)
                for (float pitch : getSwingSteps(-shape.mPitch - sSwingRayMargin, shape.mPitch + sSwingRayMargin))
                    for (float yaw : { -6.f, 0.f, 6.f })
                        fan.emplace_back(yaw, pitch);
            if (swing == "thrust")
                for (int yaw = -10; yaw <= 10; yaw += 5)
                    for (int pitch = -10; pitch <= 10; pitch += 5)
                        if (yaw * yaw + pitch * pitch <= 100)
                            fan.emplace_back(yaw, pitch);
            if (fan.empty())
                fan.emplace_back(0.f, 0.f);
            std::stable_sort(fan.begin(), fan.end(), [&](const osg::Vec2f& a, const osg::Vec2f& b) {
                return getSwingCost(swing, a.x(), a.y(), reverseSlash) < getSwingCost(swing, b.x(), b.y(), reverseSlash);
            });
            fan.erase(std::unique(fan.begin(), fan.end()), fan.end());
            return fan;
        }

        // The area a swing covers, with tolerance beyond its ray fan (degrees).
        bool isInSwingArea(std::string_view swing, const SwingShape& shape, float yaw, float pitch)
        {
            const bool slash = std::abs(yaw) <= shape.mYaw + sSwingAreaMargin && std::abs(pitch) <= 18.f;
            const bool chop = std::abs(yaw) <= 15.f && pitch >= -shape.mPitch - sSwingAreaMargin
                && pitch <= shape.mPitch + sSwingAreaMargin;
            if (swing == "slash")
                return slash;
            if (swing == "chop")
                return chop;
            if (swing == "thrust")
                return yaw * yaw + pitch * pitch <= 16.f * 16.f;
            return slash || chop;
        }
    }

    std::pair<MWWorld::Ptr, osg::Vec3f> getVanillaHitContact(const MWWorld::Ptr& actor, float reach)
    {
        // Lasciate ogne speranza, voi ch'entrate
        MWWorld::Ptr result;
        osg::Vec3f hitPos;
        float minDist = std::numeric_limits<float>::max();
        MWBase::World* world = MWBase::Environment::get().getWorld();
        const MWWorld::Store<ESM::GameSetting>& store = world->getStore().get<ESM::GameSetting>();

        // These GMSTs are not in degrees. They're tolerance angle sines multiplied by 90.
        // With the default values of 60, the actual tolerance angles are roughly 41.8 degrees.
        // Don't think too hard about it. In this place, thinking can cause permanent damage to your mental health.
        const float fCombatAngleXY = store.find("fCombatAngleXY")->mValue.getFloat() / 90.f;
        const float fCombatAngleZ = store.find("fCombatAngleZ")->mValue.getFloat() / 90.f;

        const ESM::Position& posdata = actor.getRefData().getPosition();
        const osg::Vec3f actorPos(posdata.asVec3());
        const osg::Vec3f actorDirXY = osg::Quat(posdata.rot[2], osg::Vec3(0, 0, -1)) * osg::Vec3f(0, 1, 0);
        // Only the player can look up, apparently.
        const float actorVerticalAngle = actor == getPlayer() ? -std::sin(posdata.rot[0]) : 0.f;
        const float actorEyeLevel = world->getHalfExtents(actor, true).z() * 2.f * 0.85f;
        const osg::Vec3f actorEyePos{ actorPos.x(), actorPos.y(), actorPos.z() + actorEyeLevel };
        const bool canMoveByZ = canActorMoveByZAxis(actor);

        // The player can target any active actor, non-playable actors only target their targets
        std::vector<MWWorld::Ptr> targets;
        if (actor != getPlayer())
            actor.getClass().getCreatureStats(actor).getAiSequence().getCombatTargets(targets);
        else
            MWBase::Environment::get().getMechanicsManager()->getActorsInRange(
                actorPos, static_cast<float>(Settings::game().mActorsProcessingRange), targets);

        for (MWWorld::Ptr& target : targets)
        {
            if (actor == target || target.getClass().getCreatureStats(target).isDead())
                continue;

            const float dist = getDistanceToBounds(actor, target);
            if (dist >= minDist || !isInMeleeReach(actor, target, reach))
                continue;

            const osg::Vec3f targetPos(target.getRefData().getPosition().asVec3());

            // Horizontal angle checks.
            osg::Vec2f actorToTargetXY{ targetPos.x() - actorPos.x(), targetPos.y() - actorPos.y() };
            actorToTargetXY.normalize();

            // Use dot product to check if the target is behind first...
            if (actorToTargetXY.x() * actorDirXY.x() + actorToTargetXY.y() * actorDirXY.y() <= 0.f)
                continue;

            // And then perp dot product to calculate the hit angle sine.
            // This gives us a horizontal hit range of [-asin(fCombatAngleXY / 90); asin(fCombatAngleXY / 90)]
            if (std::abs(actorToTargetXY.x() * actorDirXY.y() - actorToTargetXY.y() * actorDirXY.x()) > fCombatAngleXY)
                continue;

            // Vertical angle checks. Nice cliff racer hack, Todd.
            if (!canMoveByZ)
            {
                // The idea is that the body should always be possible to hit.
                // fCombatAngleZ is the tolerance for hitting the target's feet or head.
                osg::Vec3f actorToTargetFeet = targetPos - actorEyePos;
                osg::Vec3f actorToTargetHead = actorToTargetFeet;
                actorToTargetFeet.normalize();
                actorToTargetHead.z() += world->getHalfExtents(target, true).z() * 2.f;
                actorToTargetHead.normalize();

                if (actorVerticalAngle - actorToTargetHead.z() > fCombatAngleZ
                    || actorVerticalAngle - actorToTargetFeet.z() < -fCombatAngleZ)
                    continue;
            }

            // Gotta use physics somehow!
            if (!world->getLOS(actor, target))
                continue;

            minDist = dist;
            result = target;
        }

        // This hit position is currently used for spawning the blood effect.
        // Morrowind does this elsewhere, but roughly at the same time
        // and it would be hard to track the original hit results outside of this function
        // without code duplication
        // The idea is to use a random point on a plane in front of the target
        // that is defined by its width and height
        if (!result.isEmpty())
        {
            osg::Vec3f resultPos(result.getRefData().getPosition().asVec3());
            osg::Vec3f dirToActor = actorPos - resultPos;
            dirToActor.normalize();

            hitPos = resultPos + dirToActor * world->getHalfExtents(result).y();
            // -25% to 25% of width
            float xOffset = Misc::Rng::deviate(0.f, 0.25f, world->getPrng());
            // 20% to 100% of height
            float zOffset = Misc::Rng::deviate(0.6f, 0.4f, world->getPrng());
            hitPos.x() += world->getHalfExtents(result).x() * 2.f * xOffset;
            hitPos.z() += world->getHalfExtents(result).z() * 2.f * zOffset;
        }

        return std::make_pair(result, hitPos);
    }

    std::pair<MWWorld::Ptr, osg::Vec3f> getHitContact(
        const MWWorld::Ptr& actor, float reach, int* contactPart, std::string_view swing)
    {
        if(contactPart)*contactPart=-1;
        if (!Settings::game().mBrutalMeleeGeometry)
        {
            return getVanillaHitContact(actor, reach);
        }
        // Lasciate ogne speranza, voi ch'entrate
        MWWorld::Ptr result;
        osg::Vec3f hitPos;
        float minDist = std::numeric_limits<float>::max();
        MWBase::World* world = MWBase::Environment::get().getWorld();
        const MWWorld::Store<ESM::GameSetting>& store = world->getStore().get<ESM::GameSetting>();

        const auto [aimOrigin,aimDirection] = getMeleeAim(actor);
        constexpr int coverMask = MWPhysics::CollisionType_World | MWPhysics::CollisionType_Door
            | MWPhysics::CollisionType_HeightMap | MWPhysics::CollisionType_Dynamic;
        const auto clearContact = [&](const osg::Vec3f& point, const MWWorld::Ptr& target) {
            // A victim's own physical limb hulls may lie outside its rendered
            // skin. Ignore that victim, while retaining intervening cover.
            const auto cover = world->getRayCasting()->castRay(aimOrigin,point,{target},{},coverMask);
            return !cover.mHit || (cover.mHitPos-aimOrigin).length()+.05f >= (point-aimOrigin).length();
        };
        if (Settings::game().mBrutalMeleeGeometry && actor == getPlayer())
        {
            // Vanilla reach is measured from actor bounds, not its center.
            // Retain that allowance while requiring a real surface on the stroke.
            const float length = reach+world->getHalfExtents(actor).y();
            const SwingShape shape = getSwingShape();
            const bool reverseSlash = actor.getClass().getCreatureStats(actor).mBrutalSlashRight;
            std::vector<osg::Vec2f> fan{ osg::Vec2f() };
            std::vector<MWWorld::Ptr> candidates;
            bool guardInRange = false;
            if (!swing.empty())
            {
                // Air swings cast nothing: a fan only runs with a living actor in reach.
                std::vector<MWWorld::Ptr> nearby;
                MWBase::Environment::get().getMechanicsManager()->getActorsInRange(
                    actor.getRefData().getPosition().asVec3(), length * 2.f, nearby);
                for (const MWWorld::Ptr& other : nearby)
                    if (other != actor && !other.getClass().getCreatureStats(other).isDead())
                    {
                        if (isInMeleeReach(actor, other, reach))
                            candidates.push_back(other);
                        guardInRange = guardInRange || hasAdaptiveGuardMesh(other);
                    }
                if (candidates.empty() && !guardInRange)
                    return {};
                fan = getSwingFan(swing, shape, reverseSlash);
            }
            osg::Vec3f right = aimDirection ^ osg::Vec3f(0, 0, 1);
            if (right.normalize() < 1e-3f)
                right = osg::Quat(actor.getRefData().getPosition().rot[2], osg::Vec3f(0, 0, -1)) * osg::Vec3f(1, 0, 0);
            const osg::Vec3f up = right ^ aimDirection;
            struct BodyPoint { std::string_view bone; int part; };
            static constexpr std::array<BodyPoint, 13> bodyPoints{ {
                { "Bip01 Head", ESM::PRT_Head }, { "Bip01 Neck", ESM::PRT_Neck },
                { "Bip01 Spine2", ESM::PRT_Cuirass }, { "Bip01 Spine1", ESM::PRT_Cuirass },
                { "Bip01 Pelvis", ESM::PRT_Groin }, { "Bip01 L UpperArm", ESM::PRT_LUpperarm },
                { "Bip01 R UpperArm", ESM::PRT_RUpperarm }, { "Bip01 L Forearm", ESM::PRT_LForearm },
                { "Bip01 R Forearm", ESM::PRT_RForearm }, { "Bip01 L Thigh", ESM::PRT_LLeg },
                { "Bip01 R Thigh", ESM::PRT_RLeg }, { "Bip01 L Calf", ESM::PRT_LKnee },
                { "Bip01 R Calf", ESM::PRT_RKnee } } };
            constexpr float bodyRadius = 20.f;
            if (actor.getClass().getCreatureStats(actor).mBrutalSlideAttack && swing == "slash")
            {
                // Folded limbs can fall between the regular six-degree rays.
                // Their current bones add samples inside the same visible stroke;
                // only a rendered surface hit below can authenticate the contact.
                for (const MWWorld::Ptr& target : candidates)
                {
                    const MWRender::Animation* animation = world->getAnimation(target);
                    if (const auto* npc = dynamic_cast<const MWRender::NpcAnimation*>(animation))
                        if (npc->getBodyAwarenessAnimation())
                            animation = npc->getBodyAwarenessAnimation();
                    if (!animation)
                        continue;
                    for (const BodyPoint& body : bodyPoints)
                    {
                        const auto bone = animation->getBoneWorldMatrix(body.bone);
                        if (!bone)
                            continue;
                        const osg::Vec3f offset = bone->getTrans() - aimOrigin;
                        const float forward = offset * aimDirection;
                        if (!(forward > 0.f) || offset.length() > length + bodyRadius)
                            continue;
                        const float yaw = osg::RadiansToDegrees(std::atan2(offset * right, forward));
                        const float pitch = osg::RadiansToDegrees(
                            std::atan2(offset * up, std::hypot(forward, offset * right)));
                        if (std::isfinite(yaw) && std::isfinite(pitch) && std::abs(yaw) <= shape.mYaw
                            && std::abs(pitch) <= std::min(8.f, shape.mPitch))
                            fan.emplace_back(yaw, pitch);
                    }
                }
                std::stable_sort(fan.begin(), fan.end(), [&](const osg::Vec2f& a, const osg::Vec2f& b) {
                    return getSwingCost(swing, a.x(), a.y(), reverseSlash)
                        < getSwingCost(swing, b.x(), b.y(), reverseSlash);
                });
                fan.erase(std::unique(fan.begin(), fan.end()), fan.end());
            }
            for (const osg::Vec2f& offset : fan)
            {
                const float yaw = osg::DegreesToRadians(offset.x()), pitch = osg::DegreesToRadians(offset.y());
                const osg::Vec3f direction = aimDirection * (std::cos(pitch) * std::cos(yaw))
                    + right * (std::cos(pitch) * std::sin(yaw)) + up * std::sin(pitch);
                MWPhysics::RayCastingResult contact;
                if (world->castRenderingRay(contact,aimOrigin,aimOrigin+direction*length,true,false,{},true)
                    && !contact.mHitObject.isEmpty() && contact.mHitObject.getClass().isActor()
                    && !contact.mHitObject.getClass().getCreatureStats(contact.mHitObject).isDead()
                    && (isInMeleeReach(actor,contact.mHitObject,reach)
                        || isAdaptiveGuardSurface(contact.mHitObject, contact.mHitPart))
                    && clearContact(contact.mHitPos,contact.mHitObject))
                {
                    if(contactPart)*contactPart=contact.mHitPart;
                    return {contact.mHitObject,contact.mHitPos};
                }
            }
            // No ray touched a surface: accept a body whose skeleton lies inside the
            // swing area. The point nearest the stroke is the contact.
            MWWorld::Ptr best;
            osg::Vec3f bestPoint;
            int bestPart = -1;
            float bestAngle = std::numeric_limits<float>::max();
            const auto consider = [&](const MWWorld::Ptr& target, const osg::Vec3f& point, int part) {
                const osg::Vec3f offset = point - aimOrigin;
                const float forward = offset * aimDirection;
                if (forward <= 0.f || offset.length() > length + bodyRadius)
                    return;
                const float yaw = osg::RadiansToDegrees(std::atan2(offset * right, forward));
                const float pitch = osg::RadiansToDegrees(
                    std::atan2(offset * up, std::hypot(forward, offset * right)));
                const float angle = getSwingCost(swing, yaw, pitch, reverseSlash);
                if (angle < bestAngle && isInSwingArea(swing, shape, yaw, pitch) && clearContact(point,target))
                {
                    best = target;
                    bestPoint = point;
                    bestPart = part;
                    bestAngle = angle;
                }
            };
            for (const MWWorld::Ptr& target : candidates)
            {
                if (!world->getLOS(actor, target))
                    continue;
                const MWRender::Animation* animation = world->getAnimation(target);
                if (const auto* npc = dynamic_cast<const MWRender::NpcAnimation*>(animation))
                    if (npc->getBodyAwarenessAnimation())
                        animation = npc->getBodyAwarenessAnimation();
                bool skeleton = false;
                if (animation)
                {
                    for (const BodyPoint& body : bodyPoints)
                        if (const auto bone = animation->getBoneWorldMatrix(body.bone))
                        {
                            consider(target, bone->getTrans(), body.part);
                            skeleton = true;
                        }
                    // The head bone sits at the skull base; the skull centre lies above it.
                    const auto head = animation->getBoneWorldMatrix("Bip01 Head");
                    const auto neck = animation->getBoneWorldMatrix("Bip01 Neck");
                    if (head && neck)
                        consider(target, head->getTrans() + (head->getTrans() - neck->getTrans()) * .6f, ESM::PRT_Head);
                }
                if (!skeleton)
                    consider(target, target.getRefData().getPosition().asVec3()
                        + osg::Vec3f(0, 0, world->getHalfExtents(target, true).z()), -1);
            }
            if (!best.isEmpty())
            {
                MWPhysics::RayCastingResult barrier;
                if (world->castRenderingRay(barrier, aimOrigin, bestPoint, true, false, {}, true)
                    && isAdaptiveGuardSurface(barrier.mHitObject, barrier.mHitPart)
                    && clearContact(barrier.mHitPos, barrier.mHitObject))
                {
                    if (contactPart)
                        *contactPart = barrier.mHitPart;
                    return { barrier.mHitObject, barrier.mHitPos };
                }
                if(contactPart)*contactPart=bestPart;
                return {best,bestPoint};
            }
            return {};
        }

        // These GMSTs are not in degrees. They're tolerance angle sines multiplied by 90.
        // With the default values of 60, the actual tolerance angles are roughly 41.8 degrees.
        // Don't think too hard about it. In this place, thinking can cause permanent damage to your mental health.
        const float fCombatAngleXY = store.find("fCombatAngleXY")->mValue.getFloat() / 90.f;
        const float fCombatAngleZ = store.find("fCombatAngleZ")->mValue.getFloat() / 90.f;

        const ESM::Position& posdata = actor.getRefData().getPosition();
        const osg::Vec3f actorPos(posdata.asVec3());
        const osg::Vec3f actorDirXY = osg::Quat(posdata.rot[2], osg::Vec3(0, 0, -1)) * osg::Vec3f(0, 1, 0);
        // Only the player can look up, apparently.
        const float actorVerticalAngle = actor == getPlayer() ? -std::sin(posdata.rot[0]) : 0.f;
        const float actorEyeLevel = world->getHalfExtents(actor, true).z() * 2.f * 0.85f;
        const osg::Vec3f actorEyePos{ actorPos.x(), actorPos.y(), actorPos.z() + actorEyeLevel };
        const bool canMoveByZ = canActorMoveByZAxis(actor);

        // The player can target any active actor, non-playable actors only target their targets
        std::vector<MWWorld::Ptr> targets;
        if (actor != getPlayer())
            actor.getClass().getCreatureStats(actor).getAiSequence().getCombatTargets(targets);
        else
            MWBase::Environment::get().getMechanicsManager()->getActorsInRange(
                actorPos, static_cast<float>(Settings::game().mActorsProcessingRange), targets);

        for (MWWorld::Ptr& target : targets)
        {
            if (actor == target || target.getClass().getCreatureStats(target).isDead())
                continue;

            const float dist = getDistanceToBounds(actor, target);
            if (dist >= minDist || !isInMeleeReach(actor, target, reach))
                continue;

            const osg::Vec3f targetPos(target.getRefData().getPosition().asVec3());

            // Horizontal angle checks.
            osg::Vec2f actorToTargetXY{ targetPos.x() - actorPos.x(), targetPos.y() - actorPos.y() };
            actorToTargetXY.normalize();

            // Use dot product to check if the target is behind first...
            if (actorToTargetXY.x() * actorDirXY.x() + actorToTargetXY.y() * actorDirXY.y() <= 0.f)
                continue;

            // And then perp dot product to calculate the hit angle sine.
            // This gives us a horizontal hit range of [-asin(fCombatAngleXY / 90); asin(fCombatAngleXY / 90)]
            if (std::abs(actorToTargetXY.x() * actorDirXY.y() - actorToTargetXY.y() * actorDirXY.x()) > fCombatAngleXY)
                continue;

            // Vertical angle checks. Nice cliff racer hack, Todd.
            if (!canMoveByZ)
            {
                // The idea is that the body should always be possible to hit.
                // fCombatAngleZ is the tolerance for hitting the target's feet or head.
                osg::Vec3f actorToTargetFeet = targetPos - actorEyePos;
                osg::Vec3f actorToTargetHead = actorToTargetFeet;
                actorToTargetFeet.normalize();
                actorToTargetHead.z() += world->getHalfExtents(target, true).z() * 2.f;
                actorToTargetHead.normalize();

                if (actorVerticalAngle - actorToTargetHead.z() > fCombatAngleZ
                    || actorVerticalAngle - actorToTargetFeet.z() < -fCombatAngleZ)
                    continue;
            }

            // Gotta use physics somehow!
            if (!world->getLOS(actor, target))
                continue;

            minDist = dist;
            result = target;
        }

        if (!result.isEmpty())
        {
            // AI still chooses its combat target by the existing rules, then
            // resolves a deterministic aimed torso contact instead of a random
            // blood position. The actual current mesh can intercept with a shield.
            const osg::Vec3f target = meleeTorsoPosition(result);
            osg::Vec3f direction=target-aimOrigin;
            const float distance=direction.normalize();
            if(distance<.01f)return {};
            const osg::Vec3f end=target+direction*(world->getHalfExtents(result).y()+16.f);
            const std::array<MWWorld::Ptr,1> ignore{actor};
            MWPhysics::RayCastingResult contact;
            const bool rendered=world->castRenderingRay(contact,aimOrigin,end,false,false,ignore,true);
            if(rendered && contact.mHitObject==result && clearContact(contact.mHitPos,result))
            {
                if(contactPart)*contactPart=contact.mHitPart;
                return {result,contact.mHitPos};
            }
            if(result==getPlayer())
            {
                // The FP torso is intentionally hidden. Its physical body still
                // receives AI blows; a visible shield above was tested first,
                // without the broader character capsule hiding its real surface.
                const auto body=world->getRayCasting()->castRay(aimOrigin,end,{actor},{result},MWPhysics::CollisionType_Actor);
                if(body.mHit && body.mHitObject==result && clearContact(body.mHitPos,result)
                    && (!rendered || (contact.mHitPos-aimOrigin).length()+.05f>=(body.mHitPos-aimOrigin).length()))
                {
                    if(contactPart)*contactPart=ESM::PRT_Cuirass;
                    return {result,body.mHitPos};
                }
            }
            return {};
        }

        return std::make_pair(result, hitPos);
    }

    bool friendlyHit(const MWWorld::Ptr& attacker, const MWWorld::Ptr& target, bool complain)
    {
        const MWWorld::Ptr& player = getPlayer();
        if (attacker != player)
            return false;

        std::set<MWWorld::Ptr> followersAttacker;
        MWBase::Environment::get().getMechanicsManager()->getActorsSidingWith(attacker, followersAttacker);
        if (followersAttacker.find(target) == followersAttacker.end())
            return false;

        MWMechanics::CreatureStats& statsTarget = target.getClass().getCreatureStats(target);
        if (statsTarget.getAiSequence().isInCombat())
            return true;
        statsTarget.friendlyHit();
        if (statsTarget.getFriendlyHits() >= 4)
            return false;

        if (complain)
            MWBase::Environment::get().getDialogueManager()->say(target, ESM::RefId::stringRefId("hit"));
        return true;
    }

}
