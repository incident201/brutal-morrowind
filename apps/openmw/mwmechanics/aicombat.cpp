#include "aicombat.hpp"

#include <algorithm>
#include <limits>
#include <map>

#include <components/detournavigator/navigatorutils.hpp>
#include <components/debug/debuglog.hpp>
#include <components/esm3/aisequence.hpp>
#include <components/esm3/loadarmo.hpp>
#include <components/esm3/loadweap.hpp>
#include <components/misc/coordinateconverter.hpp>
#include <components/misc/mathutil.hpp>
#include <components/misc/pathgridutils.hpp>
#include <components/misc/rng.hpp>
#include <components/sceneutil/positionattitudetransform.hpp>

#include "../mwphysics/raycasting.hpp"

#include "../mwworld/class.hpp"
#include "../mwworld/esmstore.hpp"
#include "../mwworld/datetimemanager.hpp"
#include "../mwworld/inventorystore.hpp"
#include "../mwworld/worldmodel.hpp"

#include "../mwbase/dialoguemanager.hpp"
#include "../mwbase/environment.hpp"
#include "../mwbase/mechanicsmanager.hpp"
#include "../mwbase/world.hpp"

#include "actorutil.hpp"

#include <components/settings/values.hpp>
#include "aicombataction.hpp"
#include "character.hpp"
#include "combat.hpp"
#include "creaturestats.hpp"
#include "npcstats.hpp"
#include "movement.hpp"
#include "pathgrid.hpp"
#include "steering.hpp"
#include "weaponpriority.hpp"
#include "weapontype.hpp"

namespace
{

    // chooses an attack depending on probability to avoid uniformity
    std::string_view chooseBestAttack(const ESM::Weapon* weapon);

    osg::Vec3f AimDirToMovingTarget(const MWWorld::Ptr& actor, const MWWorld::Ptr& target,
        const osg::Vec3f& vLastTargetPos, float duration, int weapType, float strength);

    bool hitAttemptMatchesTarget(const MWWorld::Ptr& actor, const MWWorld::Ptr& target)
    {
        ESM::RefNum hitNum = actor.getClass().getCreatureStats(actor).getHitAttemptActor();
        return hitNum.isSet() && target.getCellRef().getRefNum() == hitNum;
    }
}

namespace MWMechanics
{
    namespace
    {
        // npc_guard.qct zones and timings, converted from DM inches to Morrowind units.
        constexpr float DmInch = 1.84265547703229f;
        constexpr float DmRedZone = 40.f * DmInch; // closer than this, the guard steps back
        constexpr float DmYellowZone = 120.f * DmInch;
        constexpr float DmDodgeDelay = 15.f; // DodgeDelay / PreventiveDodgeDelay
        constexpr float DmDodgeChance = 0.5f; // base_dodge_chance, against an incoming attack
        constexpr float DmPreventiveDodgeChance = 0.2f;
        constexpr float DmDodgeDuration = 0.5f;
        constexpr float DmGuardChargeDelay = 4.f;

        bool darkMessiahMelee(const MWWorld::Ptr& actor, bool ranged)
        {
            return !ranged && actor.getClass().isNpc() && actor != getPlayer()
                && Settings::game().mDarkMessiahNpcCombat;
        }

        // ACT_GRAB_WEAPON. The NPC crouches at this horizontal distance from the item's origin
        // (the source hand lands about 22 units in front of the guard's root; a table blocks the
        // body farther out), gives an item up after these seconds and searches again at this interval.
        constexpr float BrutalPickupReach = 46.f;
        constexpr float BrutalPickupTableReach = 62.f;
        // Above this height over the NPC's feet the item lies on a table, a crate or a shelf, and
        // the standing reach is played instead of the crouch; higher items are out of reach.
        constexpr float BrutalPickupTableHeight = 25.f;
        constexpr float BrutalPickupMaxHeight = 105.f;
        constexpr float BrutalPickupWeaponGiveUp = 8.f;
        constexpr float BrutalPickupShieldGiveUp = 4.f;
        constexpr float BrutalPickupRescan = .75f;
        constexpr float BrutalPickupSettled = .3f; // seconds the item must lie still before the crouch
        // The navmesh ends short of an item whose own collision cuts it, or at a table's edge. The NPC
        // then walks straight at the item; blocked for this long within this much more than its
        // reach, it crouches or leans from there.
        constexpr float BrutalPickupStalled = .4f;
        constexpr float BrutalPickupStretch = 50.f;

        // Two NPCs never run for one item: item -> (claiming actor, claim expiry).
        std::map<ESM::RefNum, std::pair<ESM::RefNum, double>> sBrutalPickupClaims;

        bool isTwoHanded(const MWWorld::ConstPtr& weapon)
        {
            return weapon.getType() == ESM::Weapon::sRecordId
                && getWeaponType(weapon.get<ESM::Weapon>()->mBase->mData.mType)->mFlags & ESM::WeaponType::TwoHanded;
        }

        bool isShield(const MWWorld::ConstPtr& item)
        {
            return item.getType() == ESM::Armor::sRecordId
                && item.get<ESM::Armor>()->mBase->mData.mType == ESM::Armor::Shield;
        }

        bool hasWeapon(const MWWorld::Ptr& actor)
        {
            auto& inventory = actor.getClass().getInventoryStore(actor);
            for (auto it = inventory.begin(MWWorld::ContainerStore::Type_Weapon); it != inventory.end(); ++it)
                if (getWeaponType(it->get<ESM::Weapon>()->mBase->mData.mType)->mWeaponClass != ESM::WeaponType::Ammo
                    && (!it->getClass().hasItemHealth(*it) || it->getClass().getItemHealth(*it) > 0))
                    return true;
            return false;
        }

        bool drawsMissile(const MWWorld::Ptr& target)
        {
            if (!target.getClass().hasInventoryStore(target)
                || target.getClass().getCreatureStats(target).getDrawState() != DrawState::Weapon
                || !MWBase::Environment::get().getMechanicsManager()->isAttackingOrSpell(target))
                return false;
            const auto& inventory = target.getClass().getInventoryStore(target);
            const auto weapon = inventory.getSlot(MWWorld::InventoryStore::Slot_CarriedRight);
            if (weapon == inventory.end() || weapon->getType() != ESM::Weapon::sRecordId)
                return false;
            const auto weaponClass = getWeaponType(weapon->get<ESM::Weapon>()->mBase->mData.mType)->mWeaponClass;
            return weaponClass == ESM::WeaponType::Ranged || weaponClass == ESM::WeaponType::Thrown;
        }

        bool carriesShield(const MWWorld::Ptr& actor)
        {
            if (!actor.getClass().hasInventoryStore(actor))
                return false;
            const auto& inventory = actor.getClass().getInventoryStore(actor);
            const auto left = inventory.getSlot(MWWorld::InventoryStore::Slot_CarriedLeft);
            return left != inventory.end() && isShield(*left);
        }

        bool hasShield(const MWWorld::Ptr& actor)
        {
            auto& inventory = actor.getClass().getInventoryStore(actor);
            for (auto it = inventory.begin(MWWorld::ContainerStore::Type_Armor); it != inventory.end(); ++it)
                if (isShield(*it) && it->getClass().getItemHealth(*it) > 0)
                    return true;
            return false;
        }

        bool holdsTwoHanded(const MWWorld::Ptr& actor)
        {
            auto& inventory = actor.getClass().getInventoryStore(actor);
            const auto right = inventory.getSlot(MWWorld::InventoryStore::Slot_CarriedRight);
            return right != inventory.end() && isTwoHanded(*right);
        }

        // A loose melee or thrown weapon (or a shield) the NPC may take, whoever owns it: one fighting
        // for its life does not ask. Items with a script a quest may rely on stay; so does what the
        // player holds and what lies higher than a table.
        bool isBrutalPickupCandidate(const MWWorld::Ptr& actor, const MWWorld::Ptr& item, bool shield)
        {
            if (shield ? !isShield(item) : item.getType() != ESM::Weapon::sRecordId)
                return false;
            if (!shield)
            {
                const auto weaponClass = getWeaponType(item.get<ESM::Weapon>()->mBase->mData.mType)->mWeaponClass;
                if (weaponClass != ESM::WeaponType::Melee && weaponClass != ESM::WeaponType::Thrown)
                    return false;
            }
            if ((item.getClass().hasItemHealth(item) && item.getClass().getItemHealth(item) <= 0)
                || !item.getClass().getScript(item).empty())
                return false;
            if (MWBase::Environment::get().getWorld()->getHeldObject() == item)
                return false;
            const float height = item.getRefData().getPosition().pos[2] - actor.getRefData().getPosition().pos[2];
            return height > -80.f && height < BrutalPickupMaxHeight;
        }

        bool isBrutalPickupPresent(const MWWorld::Ptr& item)
        {
            return !item.isEmpty() && item.isInCell() && item.getCellRef().getCount() > 0
                && item.getRefData().isEnabled() && item.getRefData().getBaseNode()
                && MWBase::Environment::get().getWorld()->getHeldObject() != item;
        }

        // Weapons are ranked as the AI ranks its own inventory (damage, condition, enchantment and the
        // NPC's skill with the type), a little less the farther they lie; shields by distance only.
        MWWorld::Ptr findBrutalPickup(const MWWorld::Ptr& actor, const MWWorld::Ptr& enemy, bool shield, double now,
            const std::vector<ESM::RefNum>& rejected)
        {
            const float radius = shield ? Settings::game().mBrutalNpcShieldPickupRadius.get()
                                        : Settings::game().mBrutalNpcWeaponPickupRadius.get();
            if (radius <= 0.f)
                return {};
            const osg::Vec3f origin = actor.getRefData().getPosition().asVec3();
            std::vector<MWWorld::Ptr> items;
            MWBase::Environment::get().getWorld()->getLooseEquipmentInRange(origin, radius, items);
            for (auto it = sBrutalPickupClaims.begin(); it != sBrutalPickupClaims.end();)
                it = it->second.second < now ? sBrutalPickupClaims.erase(it) : std::next(it);
            const ESM::RefNum self = actor.getCellRef().getRefNum();
            auto& inventory = actor.getClass().getInventoryStore(actor);
            const auto left = inventory.getSlot(MWWorld::InventoryStore::Slot_CarriedLeft);
            const bool shieldInHand = left != inventory.end() && isShield(*left);
            MWWorld::Ptr best;
            float bestScore = -std::numeric_limits<float>::max();
            for (const MWWorld::Ptr& item : items)
            {
                if (!isBrutalPickupPresent(item) || !isBrutalPickupCandidate(actor, item, shield))
                    continue;
                const ESM::RefNum number = item.getCellRef().getRefNum();
                if (number.isSet())
                {
                    if (std::find(rejected.begin(), rejected.end(), number) != rejected.end())
                        continue;
                    const auto claim = sBrutalPickupClaims.find(number);
                    if (claim != sBrutalPickupClaims.end() && claim->second.first != self)
                        continue;
                }
                osg::Vec3f offset = item.getRefData().getPosition().asVec3() - origin;
                offset.z() = 0.f;
                const float nearness = 1.f - .5f * std::min(offset.length() / radius, 1.f);
                float score = -offset.length();
                if (!shield)
                {
                    // With a shield still on its arm the NPC prefers a one-handed weapon.
                    const float rating = std::max(rateWeapon(item, actor, enemy), .01f);
                    score = rating * nearness * (shieldInHand && isTwoHanded(item) ? .5f : 1.f);
                }
                if (score > bestScore)
                {
                    bestScore = score;
                    best = item;
                }
            }
            return best;
        }

        void stopMovementForBrutalAction(const MWWorld::Ptr& actor)
        {
            auto& movement = actor.getClass().getMovementSettings(actor);
            movement.mPosition[0] = movement.mPosition[1] = movement.mPosition[2] = 0.f;
        }
    }

    AiCombat::AiCombat(const MWWorld::Ptr& actor)
    {
        mTargetActor = actor.getCellRef().getRefNum();
    }

    AiCombat::AiCombat(const ESM::AiSequence::AiCombat* combat)
    {
        mTargetActor = combat->mTargetActor;
    }

    void AiCombat::init() {}

    void AiCombat::clearBrutalPickupClaims()
    {
        sBrutalPickupClaims.clear();
    }

    /*
     * Current AiCombat movement states (as of 0.29.0), ignoring the details of the
     * attack states such as CombatMove, Strike and ReadyToAttack:
     *
     *    +----(within strike range)----->attack--(beyond strike range)-->follow
     *    |                                 | ^                            | |
     *    |                                 | |                            | |
     *  pursue<---(beyond follow range)-----+ +----(within strike range)---+ |
     *    ^                                                                  |
     *    |                                                                  |
     *    +-------------------------(beyond follow range)--------------------+
     *
     *
     * Below diagram is high level only, the code detail is a little different
     * (but including those detail will just complicate the diagram w/o adding much)
     *
     *    +----------(same)-------------->attack---------(same)---------->follow
     *    |                                 |^^                            |||
     *    |                                 |||                            |||
     *    |       +--(same)-----------------+|+----------(same)------------+||
     *    |       |                          |                              ||
     *    |       |                          | (in range)                   ||
     *    |   <---+         (too far)        |                              ||
     *  pursue<-------------------------[door open]<-----+                  ||
     *    ^^^                                            |                  ||
     *    |||                                            |                  ||
     *    ||+----------evade-----+                       |                  ||
     *    ||                     |    [closed door]      |                  ||
     *    |+----> maybe stuck, check --------------> back up, check door    ||
     *    |         ^   |   ^                          |   ^                ||
     *    |         |   |   |                          |   |                ||
     *    |         |   +---+                          +---+                ||
     *    |         +-------------------------------------------------------+|
     *    |                                                                  |
     *    +---------------------------(same)---------------------------------+
     *
     * FIXME:
     *
     * The new scheme is way too complicated, should really be implemented as a
     * proper state machine.
     *
     * TODO:
     *
     * Use the observer pattern to coordinate attacks, provide intelligence on
     * whether the target was hit, etc.
     */

    bool AiCombat::execute(
        const MWWorld::Ptr& actor, CharacterController& characterController, AiState& state, float duration)
    {
        // Get or create temporary storage
        AiCombatStorage& storage = state.get<AiCombatStorage>();

        auto& actorStats = actor.getClass().getCreatureStats(actor);
        const auto clearDefence = [&] {
            actorStats.mBrutalNpcGuardRequested = false;
            actorStats.mBrutalGuardBreakingAttack = false;
            actorStats.mBrutalFastPressure = 0;
        };
        // No combat for dead creatures
        if (actorStats.isDead())
        {
            clearDefence();
            return true;
        }

        const MWWorld::Ptr target = getTarget(); // The target to follow

        // Stop if the target doesn't exist
        if (target.isEmpty() || !target.getCellRef().getCount() || !target.getRefData().isEnabled()
            || target.getClass().getCreatureStats(target).isDead() || !target.getRefData().getBaseNode())
        {
            clearDefence();
            return true;
        }

        if (actor == target) // This should never happen.
            return true;

        // No actions for totally static creatures
        if (!actor.getClass().isMobile(actor))
        {
            storage.mFleeState = AiCombatStorage::FleeState_Idle;
            return false;
        }

        if (characterController.isBrutalBowKicking()
            || (!storage.isFleeing() && characterController.startBrutalBowKick(target)))
        {
            clearDefence();
            storage.stopAttack();
            actorStats.setAttackingOrSpell(false);
            stopMovementForBrutalAction(actor);
            storage.mActionCooldown -= duration;
            return false;
        }

        if (updateBrutalPickup(actor, target, characterController, storage, duration))
        {
            clearDefence();
            storage.mDmTargetGuardTime = 0.f;
            storage.mActionCooldown -= duration;
            return false;
        }

        if (!storage.isFleeing())
        {
            if (storage.mCurrentAction.get()) // need to wait to init action with its attack range
            {
                // Update every frame. UpdateLOS uses a timer, so the LOS check does not happen every frame.
                updateLOS(actor, target, duration, storage);
                const float targetReachedTolerance
                    = storage.mLOS && !storage.mUseCustomDestination ? storage.mAttackRange : 0.0f;
                const osg::Vec3f destination = storage.mUseCustomDestination
                    ? storage.mCustomDestination
                    : target.getRefData().getPosition().asVec3();
                const bool isTargetReached = pathTo(actor, destination, duration,
                    characterController.getSupportedMovementDirections(), targetReachedTolerance);
                if (isTargetReached)
                    storage.mReadyToAttack = true;
            }

            storage.updateCombatMove(duration);
            storage.updateDarkMessiah(duration, actor, target, characterController);
            updateBrutalSpread(actor, target, storage, duration);
            storage.mRotateMove = false;
            if (storage.mReadyToAttack)
                updateActorsMovement(actor, duration, storage);
            if (storage.mRotateMove)
                return false;
            storage.updateAttack(actor, characterController);
        }
        else
        {
            clearDefence();
            storage.mDmTargetGuardTime = 0.f;
            updateFleeing(actor, target, duration, characterController.getSupportedMovementDirections(), storage);
        }
        storage.mActionCooldown -= duration;

        if (storage.mReaction.update(duration) == Misc::TimerStatus::Waiting)
            return false;

        return attack(actor, target, storage, characterController);
    }

    namespace
    {
        constexpr float SpreadRadius = 600.f; // only fighters this close to the target take part
        constexpr float SpreadStartArc = 0.785398f; // 45 degrees
        constexpr float SpreadStopArc = 1.047198f; // 60 degrees
        constexpr float SpreadSpeed = .8f;

        // Which way (-1 left, 1 right, 0 stay) a melee fighter should drift around its target
        // so that it is not standing in the arc of an ally that holds the front. Allies closer
        // to the target keep their place; this actor moves away from them along the circle.
        float chooseBrutalSpreadSide(const MWWorld::Ptr& actor, const MWWorld::Ptr& target, float current)
        {
            const osg::Vec3f targetPos = target.getRefData().getPosition().asVec3();
            const osg::Vec3f actorPos = actor.getRefData().getPosition().asVec3();
            const osg::Vec2f mine(actorPos.x() - targetPos.x(), actorPos.y() - targetPos.y());
            const float myDistance = mine.length();
            if (myDistance > SpreadRadius || myDistance < 1.f)
                return 0.f;
            const float myAngle = std::atan2(mine.x(), mine.y());
            const unsigned myIndex = actor.getCellRef().getRefNum().mIndex;
            const float arc = current != 0.f ? SpreadStopArc : SpreadStartArc;
            float push = 0.f;
            bool crowded = false;
            for (const MWWorld::Ptr& ally : MWBase::Environment::get().getMechanicsManager()->getActorsFighting(target))
            {
                if (ally == actor || ally == target || ally.getClass().getCreatureStats(ally).isDead())
                    continue;
                const osg::Vec3f allyPos = ally.getRefData().getPosition().asVec3();
                const osg::Vec2f theirs(allyPos.x() - targetPos.x(), allyPos.y() - targetPos.y());
                const float distance = theirs.length();
                if (distance > SpreadRadius || std::abs(allyPos.z() - targetPos.z()) > 200.f)
                    continue;
                const bool holdsFront = distance < myDistance - 15.f
                    || (std::abs(distance - myDistance) <= 15.f && ally.getCellRef().getRefNum().mIndex < myIndex);
                if (!holdsFront)
                    continue;
                const float delta = static_cast<float>(Misc::normalizeAngle(myAngle - std::atan2(theirs.x(), theirs.y())));
                if (std::abs(delta) >= arc)
                    continue;
                crowded = true;
                // Directly behind an ally: keep a chosen side, or pick one by identity.
                const float away = std::abs(delta) < osg::DegreesToRadians(3.f)
                    ? (current != 0.f ? -current : (myIndex % 2 ? 1.f : -1.f))
                    : (delta > 0.f ? 1.f : -1.f);
                push += away * (arc - std::abs(delta)) / arc;
            }
            if (!crowded || push == 0.f)
                return 0.f;
            // Increasing the angle around the target is a step to the fighter's left.
            const float side = push > 0.f ? -1.f : 1.f;
            // Only step where there is room and floor.
            const int mask = MWPhysics::CollisionType_World | MWPhysics::CollisionType_HeightMap
                | MWPhysics::CollisionType_Door | MWPhysics::CollisionType_Actor;
            const osg::Vec3f half = MWBase::Environment::get().getWorld()->getHalfExtents(actor);
            const osg::Vec2f facing = -mine / myDistance;
            const osg::Vec3f right(facing.y(), -facing.x(), 0.f);
            const osg::Vec3f source = actorPos + osg::Vec3f(0, 0, 0.75f * half.z());
            const osg::Vec3f beside = source + right * (side * (half.x() + 56.f));
            const auto* rays = MWBase::Environment::get().getWorld()->getRayCasting();
            if (rays->castRay(source, beside, std::vector<MWWorld::ConstPtr>{ actor }, {}, mask).mHit
                || !rays->castRay(beside, beside - osg::Vec3f(0, 0, 0.75f * half.z() + 96.f), mask).mHit)
                return 0.f;
            return side;
        }
    }

    // NPCs attacking one target fan out around it instead of queueing in one arc.
    void AiCombat::updateBrutalSpread(
        const MWWorld::Ptr& actor, const MWWorld::Ptr& target, AiCombatStorage& storage, float duration)
    {
        bool ranged = false;
        if (storage.mCurrentAction)
            storage.mCurrentAction->getCombatRange(ranged);
        const auto& stats = actor.getClass().getCreatureStats(actor);
        const bool allowed = Settings::game().mBrutalEnabled && storage.mCurrentAction
            && darkMessiahMelee(actor, ranged) && !stats.mBrutalNpcGuardRequested && !storage.mCombatMove
            && !stats.getKnockedDown() && !storage.mUseCustomDestination;
        if (!allowed)
        {
            if (storage.mBrutalSpreading && !storage.mCombatMove)
                storage.mMovement.mPosition[0] = 0.f;
            storage.mBrutalSpreading = false;
            storage.mBrutalSpreadSide = 0.f;
            return;
        }
        storage.mBrutalSpreadTimer -= duration;
        if (storage.mBrutalSpreadTimer <= 0.f)
        {
            auto& prng = MWBase::Environment::get().getWorld()->getPrng();
            storage.mBrutalSpreadTimer = .25f + .15f * Misc::Rng::rollProbability(prng);
            storage.mBrutalSpreadSide = chooseBrutalSpreadSide(actor, target, storage.mBrutalSpreadSide);
        }
        const float side = storage.mBrutalSpreadSide * SpreadSpeed;
        if (storage.mReadyToAttack)
        {
            storage.mMovement.mPosition[0] = side;
            storage.mBrutalSpreading = side != 0.f;
        }
        else
        {
            if (storage.mBrutalSpreading)
                storage.mMovement.mPosition[0] = 0.f;
            storage.mBrutalSpreading = false;
            // Approaching: curve onto a free flank while the path still leads to the target.
            if (side != 0.f)
                actor.getClass().getMovementSettings(actor).mPosition[0] = side;
        }
    }

    bool AiCombat::updateBrutalPickup(const MWWorld::Ptr& actor, const MWWorld::Ptr& target,
        CharacterController& characterController, AiCombatStorage& storage, float duration)
    {
        auto& stats = actor.getClass().getCreatureStats(actor);
        const auto finish = [&](bool reject) {
            if (storage.mBrutalPickupItem.isSet())
            {
                if (reject)
                    storage.mBrutalPickupRejected.push_back(storage.mBrutalPickupItem);
                sBrutalPickupClaims.erase(storage.mBrutalPickupItem);
            }
            storage.mBrutalPickupItem = ESM::RefNum();
            storage.mBrutalPickupCrouching = false;
            stats.mBrutalPickupPhase = 0;
            stats.mBrutalPickupTarget = ESM::RefId();
            return false;
        };
        if (!Settings::game().mBrutalNpcWeaponPickup || !actor.getClass().isNpc() || actor == getPlayer()
            || actor.getClass().getNpcStats(actor).isWerewolf() || !characterController.canBrutalPickup())
            return finish(false);

        // The crouch and the rise with the item in hand, until the clip ends.
        if (storage.mBrutalPickupCrouching)
        {
            if (characterController.isBrutalPickingUp())
            {
                stopMovementForBrutalAction(actor);
                stats.setAttackingOrSpell(false);
                return true;
            }
            finish(false);
            storage.mActionCooldown = 0.f; // Choose the next action with the new item at once.
            storage.mBrutalPickupScan = 0.f; // Then look for a shield right beside.
            return false;
        }

        // A knockdown, a paired clinch, paralysis, flight or a health low enough to flee end it;
        // the ordinary combat decision then runs.
        const float health = stats.getHealth().getCurrent();
        const bool shouldFlee = Settings::game().mBrutalNpcFlee
            && health < Settings::game().mBrutalNpcFleeHealthFraction * stats.getHealth().getModified();
        if (storage.isFleeing() || shouldFlee || stats.getKnockedDown() || stats.isParalyzed()
            || characterController.isKnockedDown() || characterController.isKnockedOut() || stats.mBrutalClinchActive
            || !stats.mBrutalClinchOpponent.ptrOrEmpty().isEmpty())
            return finish(false);

        const bool armed = hasWeapon(actor);
        const bool wantShield = armed && !hasShield(actor) && !holdsTwoHanded(actor);
        if (armed && !wantShield)
            return finish(false);
        MWWorld::WorldModel* model = MWBase::Environment::get().getWorldModel();
        MWWorld::Ptr item;
        if (storage.mBrutalPickupItem.isSet())
        {
            item = model->getPtr(storage.mBrutalPickupItem);
            // Taken by someone else, grabbed by the player, or the need changed.
            if (!isBrutalPickupPresent(item) || storage.mBrutalPickupShield != wantShield
                || (item.getClass().hasItemHealth(item) && item.getClass().getItemHealth(item) <= 0))
            {
                finish(false);
                item = MWWorld::Ptr();
                storage.mBrutalPickupScan = 0.f;
            }
        }
        // A blow's reaction: keep the chosen item and resume once it ends.
        if (characterController.isRecovery())
        {
            if (item.isEmpty())
                return false;
            stopMovementForBrutalAction(actor);
            return true;
        }
        const double now = MWBase::Environment::get().getWorld()->getTimeManager()->getSimulationTime();
        if (item.isEmpty())
        {
            storage.mBrutalPickupScan -= duration;
            if (storage.mBrutalPickupScan > 0.f)
                return false;
            storage.mBrutalPickupScan = BrutalPickupRescan;
            model->registerPtr(actor);
            item = findBrutalPickup(actor, target, wantShield, now, storage.mBrutalPickupRejected);
            if (item.isEmpty())
                return false;
            model->registerPtr(item);
            storage.mBrutalPickupItem = item.getCellRef().getRefNum();
            storage.mBrutalPickupShield = wantShield;
            storage.mBrutalPickupLastPos = item.getRefData().getPosition().asVec3();
            storage.mBrutalPickupStill = 0.f;
            storage.mBrutalPickupTime = 0.f;
            storage.mBrutalPickupStall = 0.f;
            storage.mBrutalPickupActorLast = actor.getRefData().getPosition().asVec3();
            stats.mBrutalPickupTarget = item.getCellRef().getRefId();
            Log(Debug::Info) << "Brutal pickup: " << actor.getCellRef().getRefId() << " goes for "
                             << item.getCellRef().getRefId() << " at "
                             << (storage.mBrutalPickupLastPos - actor.getRefData().getPosition().asVec3()).length();
        }
        sBrutalPickupClaims[storage.mBrutalPickupItem] = { actor.getCellRef().getRefNum(), now + 1.0 };

        storage.mBrutalPickupTime += duration;
        if (storage.mBrutalPickupTime > (wantShield ? BrutalPickupShieldGiveUp : BrutalPickupWeaponGiveUp))
        {
            Log(Debug::Info) << "Brutal pickup: " << actor.getCellRef().getRefId() << " gives up "
                             << item.getCellRef().getRefId();
            return finish(true);
        }
        const osg::Vec3f itemPos = item.getRefData().getPosition().asVec3();
        storage.mBrutalPickupStill
            = (itemPos - storage.mBrutalPickupLastPos).length() > 1.5f ? 0.f : storage.mBrutalPickupStill + duration;
        storage.mBrutalPickupLastPos = itemPos;

        stats.setAttackingOrSpell(false);
        storage.stopAttack();
        storage.stopCombatMove();
        osg::Vec3f toItem = itemPos - actor.getRefData().getPosition().asVec3();
        const bool table = toItem.z() > BrutalPickupTableHeight;
        toItem.z() = 0.f;
        const float reach = table ? BrutalPickupTableReach : BrutalPickupReach;
        const osg::Vec3f actorPos = actor.getRefData().getPosition().asVec3();
        bool stalled = false;
        if (toItem.length() > reach || !characterController.hasClearBrutalPickupReach(item))
        {
            stats.mBrutalPickupPhase = 1;
            stats.setMovementFlag(CreatureStats::Flag_Run, true);
            if (pathTo(actor, itemPos, duration, characterController.getSupportedMovementDirections(), reach * .75f))
            {
                // The path ended short of the item: straight at it.
                zTurn(actor, std::atan2(toItem.x(), toItem.y()));
                actor.getClass().getMovementSettings(actor).mPosition[1] = 1.f;
                const float advanced = (actorPos - storage.mBrutalPickupActorLast).length();
                storage.mBrutalPickupStall = advanced < 60.f * duration ? storage.mBrutalPickupStall + duration : 0.f;
            }
            else
                storage.mBrutalPickupStall = 0.f;
            storage.mBrutalPickupActorLast = actorPos;
            stalled = storage.mBrutalPickupStall >= BrutalPickupStalled
                && toItem.length() <= reach + BrutalPickupStretch
                && characterController.hasClearBrutalPickupReach(item);
            if (!stalled)
                return true;
        }
        stopMovementForBrutalAction(actor);
        storage.mBrutalPickupStall = 0.f;
        stats.mBrutalPickupPhase = 1;
        const bool facing = zTurn(actor, std::atan2(toItem.x(), toItem.y()), osg::DegreesToRadians(15.f));
        if (!facing || storage.mBrutalPickupStill < BrutalPickupSettled)
            return true;
        // A scripted animation or a reaction may refuse the crouch for now; the time limit applies.
        if (!characterController.startBrutalPickup(item))
            return true;
        storage.mBrutalPickupCrouching = true;
        stats.mBrutalPickupPhase = 2;
        return true;
    }

    bool AiCombat::attack(const MWWorld::Ptr& actor, const MWWorld::Ptr& target, AiCombatStorage& storage,
        CharacterController& characterController)
    {
        const MWWorld::CellStore*& currentCell = storage.mCell;
        bool cellChange = currentCell && (actor.getCell() != currentCell);
        if (!currentCell || cellChange)
        {
            currentCell = actor.getCell();
        }

        const MWWorld::Class& actorClass = actor.getClass();
        MWMechanics::CreatureStats& stats = actorClass.getCreatureStats(actor);
        if (stats.isParalyzed() || stats.getKnockedDown())
            return false;

        bool forceFlee = false;
        if (!canFight(actor, target))
        {
            storage.stopAttack();
            stats.setAttackingOrSpell(false);
            storage.mActionCooldown = 0.f;
            // Continue combat if target is player or player follower/escorter and an attack has been attempted
            const auto& playerFollowersAndEscorters
                = MWBase::Environment::get().getMechanicsManager()->getActorsSidingWith(MWMechanics::getPlayer());
            bool targetSidesWithPlayer
                = (std::find(playerFollowersAndEscorters.begin(), playerFollowersAndEscorters.end(), target)
                    != playerFollowersAndEscorters.end());
            if ((target == MWMechanics::getPlayer() || targetSidesWithPlayer)
                && (hitAttemptMatchesTarget(actor, target) || hitAttemptMatchesTarget(target, actor)))
                forceFlee = true;
            else // Otherwise end combat
                return true;
        }

        float& actionCooldown = storage.mActionCooldown;
        std::unique_ptr<Action>& currentAction = storage.mCurrentAction;

        // Dark Messiah's flee: a blow that leaves an NPC below the flee fraction of its health makes
        // it run for TimeFleeStop seconds. Afterwards it fights on; a later blow below the
        // fraction makes it run again.
        if (Settings::game().mBrutalNpcFlee && actorClass.isNpc() && actor != getPlayer())
        {
            const double now = MWBase::Environment::get().getWorld()->getTimeManager()->getSimulationTime();
            const float health = stats.getHealth().getCurrent();
            recordBrutalNpcHealthLoss(actor, storage.mBrutalLastHealth);
            storage.mBrutalLastHealth = health;
            if (now < stats.mBrutalFleeUntil)
            {
                forceFlee = true;
                storage.mBrutalFleeing = true;
            }
            else if (storage.mBrutalFleeing)
            {
                storage.mBrutalFleeing = false;
                actionCooldown = 0.f; // Turn back at once rather than after the flight's cooldown.
            }
        }
        else if (storage.mBrutalFleeing)
        {
            storage.mBrutalFleeing = false;
            stats.mBrutalFleeUntil = 0.0;
            actionCooldown = 0.f;
        }

        if (!forceFlee)
        {
            if (actionCooldown > 0)
                return false;

            if (characterController.readyToPrepareAttack())
            {
                currentAction = prepareNextAction(actor, target);
                actionCooldown = currentAction->getActionCooldown();
            }
        }
        else
        {
            currentAction = std::make_unique<ActionFlee>();
            actionCooldown = currentAction->getActionCooldown();
        }

        if (!currentAction)
            return false;

        if (storage.isFleeing() != currentAction->isFleeing())
        {
            if (currentAction->isFleeing())
            {
                storage.startFleeing();
                MWBase::Environment::get().getDialogueManager()->say(actor, ESM::RefId::stringRefId("flee"));
                return false;
            }
            else
                storage.stopFleeing();
        }

        bool isRangedCombat = false;
        float& rangeAttack = storage.mAttackRange;

        rangeAttack = currentAction->getCombatRange(isRangedCombat);

        // Get weapon characteristics
        const ESM::Weapon* weapon = currentAction->getWeapon();

        ESM::Position pos = actor.getRefData().getPosition();
        const osg::Vec3f vActorPos(pos.asVec3());
        const osg::Vec3f vTargetPos(target.getRefData().getPosition().asVec3());

        float distToTarget = getDistanceToBounds(actor, target);
        // Dark Messiah's guards close in at a run but fight at a walk inside their combat
        // zone (Yellow_zone 120 inches beyond their reach).
        stats.setMovementFlag(CreatureStats::Flag_Run,
            !darkMessiahMelee(actor, isRangedCombat) || distToTarget > rangeAttack + DmYellowZone);

        storage.mReadyToAttack = (currentAction->isAttackingOrSpell() && distToTarget <= rangeAttack && storage.mLOS);

        if (isRangedCombat)
        {
            // rotate actor taking into account target movement direction and projectile speed
            osg::Vec3f vAimDir = AimDirToMovingTarget(actor, target, storage.mLastTargetPos, AI_REACTION_TIME,
                (weapon ? weapon->mData.mType : 0), storage.mStrength);

            storage.mMovement.mRotation[0] = getXAngleToDir(vAimDir);
            storage.mMovement.mRotation[2] = getZAngleToDir(vAimDir);
        }
        else
        {
            osg::Vec3f vAimDir = MWBase::Environment::get().getWorld()->aimToTarget(actor, target, false);
            storage.mMovement.mRotation[0] = getXAngleToDir(vAimDir);
            storage.mMovement.mRotation[2] = getZAngleToDir(
                (vTargetPos - vActorPos)); // using vAimDir results in spastic movements since the head is animated
        }

        storage.mLastTargetPos = vTargetPos;

        if (storage.mReadyToAttack)
        {
            storage.startCombatMove(isRangedCombat, distToTarget, rangeAttack, actor, target);
            // start new attack
            bool canShout = true;
            ESM::RefId spellId = storage.mCurrentAction->getSpell();
            if (!spellId.empty())
            {
                const ESM::Spell* spell = MWBase::Environment::get().getESMStore()->get<ESM::Spell>().find(spellId);
                if (spell->mEffects.mList.empty() || spell->mEffects.mList[0].mData.mRange != ESM::RT_Target)
                    canShout = false;
            }
            storage.startAttackIfReady(actor, characterController, weapon, isRangedCombat, canShout);
        }

        // If actor uses custom destination it has to try to rebuild path because environment can change
        // (door is opened between actor and target) or target position has changed and current custom destination
        // is not good enough to attack target.
        if (storage.mCurrentAction->isAttackingOrSpell()
            && ((!storage.mReadyToAttack && !mPathFinder.isPathConstructed())
                || (storage.mUseCustomDestination && (storage.mCustomDestination - vTargetPos).length() > rangeAttack)))
        {
            const MWBase::World* world = MWBase::Environment::get().getWorld();
            // Try to build path to the target.
            const auto agentBounds = world->getPathfindingAgentBounds(actor);
            const DetourNavigator::Flags navigatorFlags = getNavigatorFlags(actor);
            const DetourNavigator::AreaCosts areaCosts = getAreaCosts(actor, navigatorFlags);
            const ESM::Pathgrid* pathgrid = world->getStore().get<ESM::Pathgrid>().search(*actor.getCell()->getCell());
            const auto& pathGridGraph = getPathGridGraph(pathgrid);
            mPathFinder.buildPath(actor, vActorPos, vTargetPos, pathGridGraph, agentBounds, navigatorFlags, areaCosts,
                storage.mAttackRange, PathType::Full);

            if (!mPathFinder.isPathConstructed())
            {
                // If there is no path, try to find a point on a line from the actor position to target projected
                // on navmesh to attack the target from there.
                const auto navigator = world->getNavigator();
                const auto hit
                    = DetourNavigator::raycast(*navigator, agentBounds, vActorPos, vTargetPos, navigatorFlags);

                if (hit.has_value() && (*hit - vTargetPos).length() <= rangeAttack)
                {
                    // If the point is close enough, try to find a path to that point.
                    mPathFinder.buildPath(actor, vActorPos, *hit, pathGridGraph, agentBounds, navigatorFlags, areaCosts,
                        storage.mAttackRange, PathType::Full);
                    if (mPathFinder.isPathConstructed())
                    {
                        // If path to that point is found use it as custom destination.
                        storage.mCustomDestination = *hit;
                        storage.mUseCustomDestination = true;
                    }
                }

                if (!mPathFinder.isPathConstructed())
                {
                    storage.mUseCustomDestination = false;
                    storage.stopAttack();
                    stats.setAttackingOrSpell(false);
                    currentAction = std::make_unique<ActionFlee>();
                    actionCooldown = currentAction->getActionCooldown();
                    storage.startFleeing();
                    MWBase::Environment::get().getDialogueManager()->say(actor, ESM::RefId::stringRefId("flee"));
                }
            }
            else
            {
                storage.mUseCustomDestination = false;
            }
        }

        return false;
    }

    void MWMechanics::AiCombat::updateLOS(
        const MWWorld::Ptr& actor, const MWWorld::Ptr& target, float duration, MWMechanics::AiCombatStorage& storage)
    {
        const float losUpdateDuration = 0.5f;
        if (storage.mUpdateLOSTimer <= 0.f)
        {
            storage.mLOS = MWBase::Environment::get().getWorld()->getLOS(actor, target);
            storage.mUpdateLOSTimer = losUpdateDuration;
        }
        else
            storage.mUpdateLOSTimer -= duration;
    }

    void MWMechanics::AiCombat::updateFleeing(const MWWorld::Ptr& actor, const MWWorld::Ptr& target, float duration,
        MWWorld::MovementDirectionFlags supportedMovementDirections, AiCombatStorage& storage)
    {
        const float blindRunDuration = 1.0f;

        updateLOS(actor, target, duration, storage);

        AiCombatStorage::FleeState& state = storage.mFleeState;
        switch (state)
        {
            case AiCombatStorage::FleeState_None:
                return;

            case AiCombatStorage::FleeState_Idle:
            {
                float triggerDist = getMaxAttackDistance(target);
                const MWWorld::Cell* cellVariant = storage.mCell->getCell();
                if (storage.mLOS && (triggerDist >= 1000 || getDistanceMinusHalfExtents(actor, target) <= triggerDist))
                {
                    const ESM::Pathgrid* pathgrid
                        = MWBase::Environment::get().getESMStore()->get<ESM::Pathgrid>().search(*cellVariant);

                    bool runFallback = true;

                    if (pathgrid != nullptr && !pathgrid->mPoints.empty()
                        && !actor.getClass().isPureWaterCreature(actor))
                    {
                        ESM::Pathgrid::PointList points;
                        const Misc::CoordinateConverter coords
                            = Misc::makeCoordinateConverter(*storage.mCell->getCell());

                        osg::Vec3f localPos = actor.getRefData().getPosition().asVec3();
                        coords.toLocal(localPos);

                        const std::size_t closestPointIndex = Misc::getClosestPoint(*pathgrid, localPos);
                        for (std::size_t i = 0; i < pathgrid->mPoints.size(); i++)
                        {
                            if (i != closestPointIndex
                                && getPathGridGraph(pathgrid).isPointConnected(closestPointIndex, i))
                            {
                                points.push_back(pathgrid->mPoints[i]);
                            }
                        }

                        // Dark Messiah's flight (FindFleeFromEnemy) runs away from the enemy:
                        // keep the points farther from it, on the far side of the actor.
                        if (storage.mBrutalFleeing)
                        {
                            const osg::Vec3f actorPos = actor.getRefData().getPosition().asVec3();
                            const osg::Vec3f enemyPos = target.getRefData().getPosition().asVec3();
                            const osg::Vec3f away = actorPos - enemyPos;
                            ESM::Pathgrid::PointList awayPoints;
                            for (ESM::Pathgrid::Point point : points)
                            {
                                coords.toWorld(point);
                                const osg::Vec3f position(point.mX, point.mY, point.mZ);
                                if ((position - enemyPos).length() > away.length() + 200.f
                                    && (position - actorPos) * away > 0.f)
                                    awayPoints.push_back(point);
                            }
                            if (!awayPoints.empty())
                            {
                                for (ESM::Pathgrid::Point& point : awayPoints)
                                    coords.toLocal(point); // The common path below converts to world.
                                points = std::move(awayPoints);
                            }
                        }

                        if (!points.empty())
                        {
                            auto& prng = MWBase::Environment::get().getWorld()->getPrng();
                            ESM::Pathgrid::Point dest = points[Misc::Rng::rollDice(points.size(), prng)];
                            coords.toWorld(dest);

                            state = AiCombatStorage::FleeState_RunToDestination;
                            storage.mFleeDest = ESM::Pathgrid::Point(dest.mX, dest.mY, dest.mZ);

                            runFallback = false;
                        }
                    }

                    if (runFallback)
                    {
                        state = AiCombatStorage::FleeState_RunBlindly;
                        storage.mFleeBlindRunTimer = 0.0f;
                    }
                }
            }
            break;

            case AiCombatStorage::FleeState_RunBlindly:
            {
                // timer to prevent twitchy movement that can be observed in vanilla MW
                if (storage.mFleeBlindRunTimer < blindRunDuration)
                {
                    storage.mFleeBlindRunTimer += duration;

                    storage.mMovement.mRotation[0] = -actor.getRefData().getPosition().rot[0];
                    storage.mMovement.mRotation[2] = osg::PIf
                        + getZAngleToDir(
                            target.getRefData().getPosition().asVec3() - actor.getRefData().getPosition().asVec3());
                    storage.mMovement.mPosition[1] = 1;
                    updateActorsMovement(actor, duration, storage);
                }
                else
                    state = AiCombatStorage::FleeState_Idle;
            }
            break;

            case AiCombatStorage::FleeState_RunToDestination:
            {
                static const float fFleeDistance = MWBase::Environment::get()
                                                       .getESMStore()
                                                       ->get<ESM::GameSetting>()
                                                       .find("fFleeDistance")
                                                       ->mValue.getFloat();

                float dist
                    = (actor.getRefData().getPosition().asVec3() - target.getRefData().getPosition().asVec3()).length();
                if ((dist > fFleeDistance && !storage.mLOS)
                    || pathTo(
                        actor, Misc::Convert::makeOsgVec3f(storage.mFleeDest), duration, supportedMovementDirections))
                {
                    state = AiCombatStorage::FleeState_Idle;
                }
            }
            break;
        };
    }

    void AiCombat::updateActorsMovement(const MWWorld::Ptr& actor, float duration, AiCombatStorage& storage)
    {
        // apply combat movement
        float deltaAngle = storage.mMovement.mRotation[2] - actor.getRefData().getPosition().rot[2];
        osg::Vec2f movement = Misc::rotateVec2f(
            osg::Vec2f(storage.mMovement.mPosition[0], storage.mMovement.mPosition[1]), -deltaAngle);

        MWMechanics::Movement& actorMovementSettings = actor.getClass().getMovementSettings(actor);
        actorMovementSettings.mPosition[0] = movement.x();
        actorMovementSettings.mPosition[1] = movement.y();
        actorMovementSettings.mPosition[2] = storage.mMovement.mPosition[2];

        rotateActorOnAxis(actor, 2, actorMovementSettings, storage);
        rotateActorOnAxis(actor, 0, actorMovementSettings, storage);
    }

    void AiCombat::rotateActorOnAxis(
        const MWWorld::Ptr& actor, int axis, MWMechanics::Movement& actorMovementSettings, AiCombatStorage& storage)
    {
        actorMovementSettings.mRotation[axis] = 0;
        bool isRangedCombat = false;
        storage.mCurrentAction->getCombatRange(isRangedCombat);
        float eps = isRangedCombat ? osg::DegreesToRadians(0.5f) : osg::DegreesToRadians(3.f);
        float targetAngleRadians = storage.mMovement.mRotation[axis];
        storage.mRotateMove = !smoothTurn(actor, targetAngleRadians, axis, eps);
    }

    void AiCombat::writeState(ESM::AiSequence::AiSequence& sequence) const
    {
        auto combat = std::make_unique<ESM::AiSequence::AiCombat>();
        combat->mTargetActor = mTargetActor;

        ESM::AiSequence::AiPackageContainer package;
        package.mType = ESM::AiSequence::Ai_Combat;
        package.mPackage = std::move(combat);
        sequence.mPackages.push_back(std::move(package));
    }

    AiCombatStorage::AiCombatStorage()
        : mAttackCooldown(0.0f)
        , mReaction(MWBase::Environment::get().getWorld()->getPrng())
        , mTimerCombatMove(0.0f)
        , mReadyToAttack(false)
        , mAttack(false)
        , mAttackRange(0.0f)
        , mCombatMove(false)
        , mRotateMove(false)
        , mLastTargetPos(0, 0, 0)
        , mCell(nullptr)
        , mCurrentAction()
        , mActionCooldown(0.0f)
        , mStrength()
        , mForceNoShortcut(false)
        , mShortcutFailPos()
        , mMovement()
        , mFleeState(FleeState_None)
        , mLOS(false)
        , mUpdateLOSTimer(0.0f)
        , mFleeBlindRunTimer(0.0f)
        , mUseCustomDestination(false)
        , mCustomDestination()
    {
    }

    void AiCombatStorage::startCombatMove(bool isDistantCombat, float distToTarget, float rangeAttack,
        const MWWorld::Ptr& actor, const MWWorld::Ptr& target)
    {
        auto& prng = MWBase::Environment::get().getWorld()->getPrng();

        // get the range of the target's weapon
        MWWorld::Ptr targetWeapon = MWWorld::Ptr();
        const MWWorld::Class& targetClass = target.getClass();

        if (targetClass.hasInventoryStore(target))
        {
            int weapType = ESM::Weapon::None;
            MWWorld::ContainerStoreIterator weaponSlot = MWMechanics::getActiveWeapon(target, &weapType);
            if (weapType > ESM::Weapon::None)
                targetWeapon = *weaponSlot;
        }

        bool targetUsesRanged = false;
        float rangeAttackOfTarget = ActionWeapon(targetWeapon).getCombatRange(targetUsesRanged);

        if (darkMessiahMelee(actor, isDistantCombat))
        {
            if (actor.getClass().getCreatureStats(actor).mBrutalNpcGuardRequested)
            {
                mMovement.mPosition[0] = mMovement.mPosition[1] = 0.f;
                return;
            }
            // Dark Messiah's guards hold their ground and face the target. They do not chain
            // 0.1-0.2 s side steps: a dodge is a deliberate step, at most once per DodgeDelay,
            // against an incoming attack (base_dodge_chance) or preventively.
            if (mCombatMove)
                return;
            mMovement.mPosition[0] = mMovement.mPosition[1] = 0;
            const bool targetAttacks = targetClass.isActor()
                && targetClass.getCreatureStats(target).getAttackingOrSpell();
            float chance = 0.f;
            if (mDmDodgeCooldown <= 0.f && distToTarget <= rangeAttackOfTarget)
            {
                if (targetAttacks)
                    chance = DmDodgeChance;
                else if (mDmPreventiveTimer <= 0.f)
                {
                    chance = DmPreventiveDodgeChance;
                    mDmPreventiveTimer = DmDodgeDelay;
                }
            }
            if (chance > 0.f && Misc::Rng::rollClosedProbability(prng) < chance)
            {
                mMovement.mPosition[0] = Misc::Rng::rollProbability(prng) < 0.5 ? 1.0f : -1.0f;
                mTimerCombatMove = DmDodgeDuration;
                mCombatMove = true;
                mDmDodgeCooldown = DmDodgeDelay;
            }
            else if (distToTarget < DmRedZone)
            {
                // Inside the red zone: step back to the ideal distance if there is floor behind.
                const int mask = MWPhysics::CollisionType_World | MWPhysics::CollisionType_HeightMap
                    | MWPhysics::CollisionType_Door;
                const osg::Vec3f half = MWBase::Environment::get().getWorld()->getHalfExtents(actor);
                const osg::Vec3f pos = actor.getRefData().getPosition().asVec3();
                const osg::Vec3f back = actor.getRefData().getBaseNode()->getAttitude() * osg::Vec3f(0, -1, 0);
                const osg::Vec3f source = pos + osg::Vec3f(0, 0, 0.75f * half.z());
                const auto* rays = MWBase::Environment::get().getWorld()->getRayCasting();
                const osg::Vec3f behind = source + back * (half.y() + 64);
                if (!rays->castRay(source, behind, mask).mHit
                    && rays->castRay(behind, behind - osg::Vec3f(0, 0, 0.75f * half.z() + 96), mask).mHit)
                {
                    mMovement.mPosition[1] = -1;
                    mTimerCombatMove = 0.4f;
                    mCombatMove = true;
                }
            }
            return;
        }

        if (mMovement.mPosition[0])
        {
            mTimerCombatMove = 0.1f + 0.1f * Misc::Rng::rollClosedProbability(prng);
            mCombatMove = true;
        }
        // dodge movements (for NPCs and bipedal creatures)
        // Note: do not use for ranged combat yet since in couple with back up behaviour can move actor out of cliff
        else if (actor.getClass().isBipedal(actor) && !isDistantCombat)
        {
            float moveDuration = 0;
            double angleToTarget
                = Misc::normalizeAngle(mMovement.mRotation[2] - actor.getRefData().getPosition().rot[2]);
            // Apply a big side step if enemy tries to get around and come from behind.
            // Otherwise apply a random side step (kind of dodging) with some probability
            // if actor is within range of target's weapon.
            if (std::abs(angleToTarget) > osg::PI / 4)
                moveDuration = 0.2f;
            else if (distToTarget <= rangeAttackOfTarget && Misc::Rng::rollClosedProbability(prng) < 0.25)
                moveDuration = 0.1f + 0.1f * Misc::Rng::rollClosedProbability(prng);
            if (moveDuration > 0)
            {
                mMovement.mPosition[0] = Misc::Rng::rollProbability(prng) < 0.5 ? 1.0f : -1.0f; // to the left/right
                mTimerCombatMove = moveDuration;
                mCombatMove = true;
            }
        }

        mMovement.mPosition[1] = 0;
        if (isDistantCombat)
        {
            // Backing up behaviour
            // Actor backs up slightly further away than opponent's weapon range
            // (in vanilla - only as far as opponent's weapon range),
            // or not at all if opponent is using a ranged weapon

            if (targetUsesRanged
                || distToTarget > rangeAttackOfTarget * 1.5) // Don't back up if the target is wielding ranged weapon
                return;

            // actor should not back up into water
            if (MWBase::Environment::get().getWorld()->isUnderwater(MWWorld::ConstPtr(actor), 0.5f))
                return;

            int mask
                = MWPhysics::CollisionType_World | MWPhysics::CollisionType_HeightMap | MWPhysics::CollisionType_Door;

            // Actor can not back up if there is no free space behind
            // Currently we take the 35% of actor's height from the ground as vector height.
            // This approach allows us to detect small obstacles (e.g. crates) and curved walls.
            osg::Vec3f halfExtents = MWBase::Environment::get().getWorld()->getHalfExtents(actor);
            osg::Vec3f pos = actor.getRefData().getPosition().asVec3();
            osg::Vec3f source = pos + osg::Vec3f(0, 0, 0.75f * halfExtents.z());
            osg::Vec3f fallbackDirection = actor.getRefData().getBaseNode()->getAttitude() * osg::Vec3f(0, -1, 0);
            osg::Vec3f destination = source + fallbackDirection * (halfExtents.y() + 16);

            const auto* rayCasting = MWBase::Environment::get().getWorld()->getRayCasting();
            bool isObstacleDetected = rayCasting->castRay(source, destination, mask).mHit;
            if (isObstacleDetected)
                return;

            // Check if there is nothing behind - probably actor is near cliff.
            // A current approach: cast ray 1.5-yard ray down in 1.5 yard behind actor from 35% of actor's height.
            // If we did not hit anything, there is a cliff behind actor.
            source = pos + osg::Vec3f(0, 0, 0.75f * halfExtents.z()) + fallbackDirection * (halfExtents.y() + 96);
            destination = source - osg::Vec3f(0, 0, 0.75f * halfExtents.z() + 96);
            bool isCliffDetected = !rayCasting->castRay(source, destination, mask).mHit;
            if (isCliffDetected)
                return;

            mMovement.mPosition[1] = -1;
        }
    }

    void AiCombatStorage::updateDarkMessiah(float duration, const MWWorld::Ptr& actor,
        const MWWorld::Ptr& target, CharacterController& characterController)
    {
        auto& stats = actor.getClass().getCreatureStats(actor);
        bool ranged = false;
        if (mCurrentAction)
            mCurrentAction->getCombatRange(ranged);
        const auto& targetStats = target.getClass().getCreatureStats(target);
        if (!darkMessiahMelee(actor, ranged) || !mCurrentAction || !mCurrentAction->getWeapon()
            || !mCurrentAction->isAttackingOrSpell() || isFleeing())
        {
            stats.mBrutalNpcGuardRequested = false;
            stats.mBrutalGuardBreakingAttack = false;
            mDmTargetGuardTime = 0.f;
            return;
        }
        const double now = MWBase::Environment::get().getWorld()->getTimeManager()->getSimulationTime();
        const bool exhausted = Settings::game().mBrutalStamina && stats.getFatigue().getCurrent() <= 0.f;
        const bool able = !stats.isDead() && !stats.isParalyzed() && !stats.getKnockedDown()
            && !stats.mBrutalClinchActive && now >= stats.mBrutalGuardBrokenUntil && !exhausted && mLOS;
        // A shield bearer covers itself from a player it sees drawing a bow or winding up a throw, at any range.
        const bool missileThreat = able && target == getPlayer() && MWMechanics::physicalProjectiles()
            && carriesShield(actor) && drawsMissile(target);
        const bool available = (able && getDistanceToBounds(actor, target) <= mAttackRange + 48.f) || missileThreat;
        // Count only a continuous guard the NPC can see at close range. A
        // newly raised guard starts over even if the lowered frame fell between updates.
        const bool observingGuard = available && target == getPlayer() && targetStats.mBrutalBlockRequested
            && targetStats.mBrutalBlockRaised;
        mDmTargetGuardTime = observingGuard
            ? std::min(mDmTargetGuardTime + duration, static_cast<float>(now - targetStats.mBrutalBlockRaisedAt))
            : 0.f;
        if (stats.mBrutalNpcGuardRequested)
        {
            // A visible windup/release is still pressure even after the player
            // has released the input. Keep the braced defence through that
            // attack, while the absolute cap still forces a counterattack gap.
            const bool incomingAttack
                = MWBase::Environment::get().getMechanicsManager()->isAttackingOrSpell(target);
            if (missileThreat)
                stats.mBrutalGuardUntil = std::max(stats.mBrutalGuardUntil, now + .6);
            else if (!available || now >= stats.mBrutalGuardUntil
                || (now - stats.mBrutalLastFastPressureAt > 1.2 && !incomingAttack))
            {
                stats.mBrutalNpcGuardRequested = false;
                stats.mBrutalGuardCooldownUntil = std::max(stats.mBrutalGuardCooldownUntil, now + .8);
                mAttackCooldown = std::min(mAttackCooldown, .25f);
            }
        }
        else if (available && target == getPlayer()
            && ((stats.mBrutalFastPressure >= 2 && now - stats.mBrutalLastFastPressureAt <= 1.8
                    && now >= stats.mBrutalGuardReactAt)
                || missileThreat)
            && now >= stats.mBrutalGuardCooldownUntil
            && characterController.readyToStartBrutalGuard())
        {
            stats.mBrutalNpcGuardRequested = true;
            stats.mBrutalGuardUntil = now + 2.4;
            stats.mBrutalFastPressure = 0;
            mAttack = false;
            stats.setAttackingOrSpell(false);
            mMovement.mPosition[0] = mMovement.mPosition[1] = 0.f;
            stopCombatMove();
        }
        auto& prng = MWBase::Environment::get().getWorld()->getPrng();
        mDmDodgeCooldown -= duration;
        mDmPreventiveTimer -= duration;
        // AttackRythmMinTime/MaxTime: alternate the calm and berserk attack rhythm.
        mDmRhythmTimer -= duration;
        if (mDmRhythmTimer <= 0.f)
        {
            mDmBerserk = !mDmBerserk;
            mDmRhythmTimer = 10.f + 5.f * Misc::Rng::rollClosedProbability(prng);
        }
        // PlayerNoAttackDelayToGoBersekerMode: a passive opponent invites the berserk rhythm.
        if (target.getClass().isActor() && target.getClass().getCreatureStats(target).getAttackingOrSpell())
            mDmTargetPassive = 0.f;
        else if ((mDmTargetPassive += duration) >= 2.f)
            mDmBerserk = true;
    }

    void AiCombatStorage::updateCombatMove(float duration)
    {
        if (mCombatMove)
        {
            mTimerCombatMove -= duration;
            if (mTimerCombatMove <= 0)
            {
                stopCombatMove();
            }
        }
    }

    void AiCombatStorage::stopCombatMove()
    {
        mTimerCombatMove = 0;
        mMovement.mPosition[0] = 0;
        mCombatMove = false;
    }

    void AiCombatStorage::startAttackIfReady(const MWWorld::Ptr& actor, CharacterController& characterController,
        const ESM::Weapon* weapon, bool distantCombat, bool canShout)
    {
        if (actor.getClass().getCreatureStats(actor).mBrutalNpcGuardRequested)
            return;
        if (mReadyToAttack && characterController.readyToStartAttack())
        {
            if (mAttackCooldown <= 0)
            {
                mAttack = true; // attack starts just now
                actor.getClass().getCreatureStats(actor).setAttackingOrSpell(true);

                if (!distantCombat)
                    characterController.setAIAttackType(chooseBestAttack(weapon));

                auto& prng = MWBase::Environment::get().getWorld()->getPrng();
                mStrength = Misc::Rng::rollClosedProbability(prng);
                const bool darkMessiah = darkMessiahMelee(actor, distantCombat);
                auto& stats = actor.getClass().getCreatureStats(actor);
                stats.mBrutalGuardBreakingAttack = darkMessiah && mDmTargetGuardTime >= DmGuardChargeDelay;
                mDmChargedHold = stats.mBrutalGuardBreakingAttack ? .45f : 0.f;
                if (darkMessiah)
                    mStrength = 1.f; // guards play the whole authored wind-up

                const MWWorld::ESMStore& store = *MWBase::Environment::get().getESMStore();

                float baseDelay = store.get<ESM::GameSetting>().find("fCombatDelayCreature")->mValue.getFloat();
                if (actor.getClass().isNpc())
                {
                    baseDelay = store.get<ESM::GameSetting>().find("fCombatDelayNPC")->mValue.getFloat();
                }

                if (canShout)
                {
                    // Say a provoking combat phrase
                    const int iVoiceAttackOdds
                        = store.get<ESM::GameSetting>().find("iVoiceAttackOdds")->mValue.getInteger();
                    if (Misc::Rng::roll0to99(prng) < iVoiceAttackOdds)
                    {
                        MWBase::Environment::get().getDialogueManager()->say(actor, ESM::RefId::stringRefId("attack"));
                    }
                }
                mAttackCooldown = std::min(baseDelay + 0.01f * Misc::Rng::roll0to99(prng), baseDelay + 0.9f);
                // AttackCool/BersekerMin/MaxTime, counted while the guard stands ready.
                if (darkMessiah)
                    mAttackCooldown = mDmBerserk ? Misc::Rng::rollClosedProbability(prng)
                                                 : 2.f + 1.5f * Misc::Rng::rollClosedProbability(prng);
            }
            else
                mAttackCooldown -= AI_REACTION_TIME;
        }
    }

    void AiCombatStorage::updateAttack(const MWWorld::Ptr& actor, CharacterController& characterController)
    {
        if (actor.getClass().getCreatureStats(actor).mBrutalNpcGuardRequested)
            mAttack = false;
        if (mAttack)
        {
            const float attackStrength = characterController.calculateWindUp();
            const bool heavyHold = attackStrength >= 1.f && mDmChargedHold > 0.f;
            if (heavyHold)
                mDmChargedHold = std::max(0.f,
                    mDmChargedHold - MWBase::Environment::get().getFrameDuration());
            mAttack = !characterController.readyToPrepareAttack() && attackStrength != -1.f
                && (attackStrength < mStrength || heavyHold);
        }
        actor.getClass().getCreatureStats(actor).setAttackingOrSpell(mAttack);
    }

    void AiCombatStorage::stopAttack()
    {
        mMovement.mPosition[0] = 0;
        mMovement.mPosition[1] = 0;
        mMovement.mPosition[2] = 0;
        mReadyToAttack = false;
        mAttack = false;
    }

    void AiCombatStorage::startFleeing()
    {
        stopFleeing();
        mFleeState = FleeState_Idle;
    }

    void AiCombatStorage::stopFleeing()
    {
        mMovement.mPosition[0] = 0;
        mMovement.mPosition[1] = 0;
        mMovement.mPosition[2] = 0;
        mFleeState = FleeState_None;
        mFleeDest = ESM::Pathgrid::Point(0, 0, 0);
    }

    bool AiCombatStorage::isFleeing() const
    {
        return mFleeState != FleeState_None;
    }
}

namespace
{

    std::string_view chooseBestAttack(const ESM::Weapon* weapon)
    {
        if (weapon != nullptr)
        {
            // the more damage attackType deals the more probability it has
            int slash = (weapon->mData.mSlash[0] + weapon->mData.mSlash[1]) / 2;
            int chop = (weapon->mData.mChop[0] + weapon->mData.mChop[1]) / 2;
            int thrust = (weapon->mData.mThrust[0] + weapon->mData.mThrust[1]) / 2;

            auto& prng = MWBase::Environment::get().getWorld()->getPrng();
            float roll = Misc::Rng::rollClosedProbability(prng) * (slash + chop + thrust);
            if (roll <= slash)
                return "slash";
            else if (roll <= (slash + thrust))
                return "thrust";
            else
                return "chop";
        }
        return MWMechanics::CharacterController::getRandomAttackType();
    }

    osg::Vec3f AimDirToMovingTarget(const MWWorld::Ptr& actor, const MWWorld::Ptr& target,
        const osg::Vec3f& vLastTargetPos, float duration, int weapType, float strength)
    {
        float projSpeed;
        const MWWorld::Store<ESM::GameSetting>& gmst
            = MWBase::Environment::get().getESMStore()->get<ESM::GameSetting>();

        // get projectile speed (depending on weapon type)
        const bool missile = weapType != 0;
        if (MWMechanics::getWeaponType(weapType)->mWeaponClass == ESM::WeaponType::Thrown)
            projSpeed = MWMechanics::getProjectileLaunchSpeed(true, strength);
        else if (weapType != 0)
            projSpeed = MWMechanics::getProjectileLaunchSpeed(false, strength);
        else // weapType is 0 ==> it's a target spell projectile
        {
            projSpeed = gmst.find("fTargetSpellMaxSpeed")->mValue.getFloat();
        }

        // idea: perpendicular to dir to target speed components of target move vector and projectile vector should be
        // the same

        osg::Vec3f vTargetPos = target.getRefData().getPosition().asVec3();
        osg::Vec3f vDirToTarget = MWBase::Environment::get().getWorld()->aimToTarget(actor, target, true);
        float distToTarget = vDirToTarget.length();

        osg::Vec3f vTargetMoveDir = vTargetPos - vLastTargetPos;
        vTargetMoveDir /= duration; // |vTargetMoveDir| is target real speed in units/sec now

        osg::Vec3f vPerpToDir = vDirToTarget ^ osg::Vec3f(0, 0, 1); // cross product

        vPerpToDir.normalize();
        osg::Vec3f vDirToTargetNormalized = vDirToTarget;
        vDirToTargetNormalized.normalize();

        // dot product
        float velPerp = vTargetMoveDir * vPerpToDir;
        float velDir = vTargetMoveDir * vDirToTargetNormalized;

        // time to collision between target and projectile
        float tCollision;

        float projVelDirSquared = projSpeed * projSpeed - velPerp * velPerp;
        if (projVelDirSquared > 0)
        {
            osg::Vec3f vTargetMoveDirNormalized = vTargetMoveDir;
            vTargetMoveDirNormalized.normalize();

            float projDistDiff = vDirToTarget * vTargetMoveDirNormalized; // dot product
            projDistDiff = std::sqrt(distToTarget * distToTarget - projDistDiff * projDistDiff);

            tCollision = projDistDiff / (std::sqrt(projVelDirSquared) - velDir);
        }
        else
            tCollision = 0; // speed of projectile is not enough to reach moving target

        osg::Vec3f aim = vDirToTarget + vTargetMoveDir * tCollision;
        if (missile && MWMechanics::physicalProjectiles() && projSpeed > 0.f)
        {
            // Aim above the target by the drop over the flight. The flight time is taken along the aim line,
            // which is close enough at the angles a bow is used at.
            const float flight = aim.length() / projSpeed;
            aim.z() += 0.5f * MWMechanics::getProjectileGravity() * flight * flight;
        }
        return aim;
    }

}
