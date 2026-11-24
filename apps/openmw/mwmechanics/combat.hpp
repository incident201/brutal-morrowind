#ifndef OPENMW_MECHANICS_COMBAT_H
#define OPENMW_MECHANICS_COMBAT_H

#include <cstdint>
#include <string_view>
#include <utility>
#include <vector>

#include <osg/Vec3f>

#include "../mwworld/ptr.hpp"

namespace MWRender
{
    struct ProjectileContact;
}

namespace MWMechanics
{
    inline constexpr float sNpcGuardContactMargin = 4.f;
    inline constexpr float sNpcProjectileGuardContactMargin = 1.f;

    std::uint64_t nextCombatAttackId();
    bool supportsRangedFinisher(int weaponType);

    bool applyOnStrikeEnchantment(const MWWorld::Ptr& attacker, const MWWorld::Ptr& victim, const MWWorld::Ptr& object,
        const osg::Vec3f& hitPosition, const bool fromProjectile = false);

    /// @return can we block the attack?
    // Dark Messiah sword parry outcomes (CreatureStats::mBrutalParryResult).
    enum BrutalParryResult
    {
        BrutalParry_None = 0,
        BrutalParry_Parried = 1, // Parry_Parry_Sword: the blow is deflected
        BrutalParry_Broken = 2, // Parry_Break_Sword: the guard is knocked open
        BrutalParry_Timed = 3, // our timed parry: deflected and the attacker recoils
    };

    bool blockMeleeAttack(const MWWorld::Ptr& attacker, const MWWorld::Ptr& blocker, const MWWorld::Ptr& weapon,
        float damage, float attackStrength, const osg::Vec3f& hitPosition, int contactPart);

    /// @return does normal weapon resistance and weakness apply to the weapon?
    bool isNormalWeapon(const MWWorld::Ptr& weapon);

    void resistNormalWeapon(
        const MWWorld::Ptr& actor, const MWWorld::Ptr& attacker, const MWWorld::Ptr& weapon, float& damage);

    void applyWerewolfDamageMult(const MWWorld::Ptr& actor, const MWWorld::Ptr& weapon, float& damage);

    /// @note for a thrown weapon, \a weapon == \a projectile, for bows/crossbows, \a projectile is the arrow/bolt
    /// @note \a victim may be empty (e.g. for a hit on terrain), a non-actor (environment objects) or an actor
    /// What became of an arrow, bolt or thrown weapon that reached an actor.
    enum class ProjectileOutcome
    {
        NoActor, ///< Hit no actor: the world, an object or nothing.
        Missed, ///< Reached an actor but did not hurt it (a failed hit roll).
        Body, ///< Hurt the actor.
        Shield, ///< Stopped by a raised wooden, chitin or other soft shield.
        MetalShield, ///< Stopped by a raised metal shield.
        Parried, ///< Turned by a raised weapon.
    };

    /// With \a flight (the missile's path through the victim, from and to), a raised shield stops it only
    /// when that path actually crosses the shield.
    ProjectileOutcome projectileHit(const MWWorld::Ptr& attacker, const MWWorld::Ptr& victim, MWWorld::Ptr weapon,
        const MWWorld::Ptr& projectile, const osg::Vec3f& hitPosition, float attackStrength,
        const std::pair<osg::Vec3f, osg::Vec3f>* flight = nullptr, std::uint64_t attackId = 0, bool finisherHit = false,
        const MWRender::ProjectileContact* contact = nullptr);

    /// Brutal physical missiles: falling arcs, contact hits, sticking and glancing.
    bool physicalProjectiles();
    bool isMissileItem(const MWWorld::Ptr& item);
    /// Launch speed of an arrow or bolt (thrown == false) or a thrown weapon at the given draw strength.
    float getProjectileLaunchSpeed(bool thrown, float attackStrength, bool playerBow = false);
    /// Downward acceleration of arrows, bolts and thrown weapons, in units per second squared.
    float getProjectileGravity();

    /// Get the chance (in percent) for \a attacker to successfully hit \a victim with a given weapon skill value
    float getHitChance(const MWWorld::Ptr& attacker, const MWWorld::Ptr& victim, int skillValue);

    /// Applies damage to attacker based on the victim's elemental shields.
    void applyElementalShields(const MWWorld::Ptr& attacker, const MWWorld::Ptr& victim);

    /// @param damage Unmitigated weapon damage of the attack
    /// @param hit Was the attack successful?
    /// @param weapon The weapon used.
    /// @note if the weapon is unequipped as result of condition damage, a new Ptr will be assigned to \a weapon.
    void reduceWeaponCondition(float damage, bool hit, MWWorld::Ptr& weapon, const MWWorld::Ptr& attacker);

    /// Adjust weapon damage based on its condition. A used weapon will be less effective.
    void adjustWeaponDamage(float& damage, const MWWorld::Ptr& weapon, const MWWorld::Ptr& attacker);

    void getHandToHandDamage(
        const MWWorld::Ptr& attacker, const MWWorld::Ptr& victim, float& damage, bool& healthdmg, float attackStrength);

    /// Apply the fatigue loss incurred by attacking with the given weapon (weapon may be empty = hand-to-hand)
    void applyFatigueLoss(const MWWorld::Ptr& attacker, const MWWorld::Ptr& weapon, float attackStrength);

    float getFightDistanceBias(const MWWorld::Ptr& actor1, const MWWorld::Ptr& actor2);

    float getAggroDistance(const MWWorld::Ptr& actor, const osg::Vec3f& lhs, const osg::Vec3f& rhs);

    // Cursed distance calculation used for combat proximity and hit checks in Morrowind
    float getDistanceToBounds(const MWWorld::Ptr& actor, const MWWorld::Ptr& target);

    float getMeleeWeaponReach(const MWWorld::Ptr& actor, const MWWorld::Ptr& weapon);

    bool isInMeleeReach(const MWWorld::Ptr& actor, const MWWorld::Ptr& target, const float reach);

    // Current gameplay eye and actor-directed aim, independent of observer cameras.
    std::pair<osg::Vec3f, osg::Vec3f> getMeleeAim(const MWWorld::Ptr& actor);

    osg::Vec3f getMeleeTorsoPosition(const MWWorld::Ptr& actor);

    // Vanilla targeting keeps its selected victim and accuracy roll. At the hit
    // key, only refresh physical guard contact and intervening actor cover.
    void refreshMeleeGuardContact(const MWWorld::Ptr& actor, MWWorld::Ptr& victim,
        osg::Vec3f& hitPosition, int& contactPart);

    // A protruding guard item can be reached before its owner's body. This does
    // not extend the distance at which the owner can receive body damage.
    bool hasReachableNpcGuardContact(const MWWorld::Ptr& actor, const MWWorld::Ptr& target, float reach,
        const osg::Vec3f& hitPosition, int contactPart);

    struct MeleeFollowThrough
    {
        MWWorld::Ptr mTarget;
        osg::Vec3f mPosition;
        int mPart;
        bool mSuccessful = false;
    };

    // Directed surface contact for the player; AI retains its selected combat target.
    // A player melee swing ("slash", "chop", "thrust" or "any") sweeps a fan of rays
    // along its stroke; an empty swing keeps the single aimed ray.
    std::pair<MWWorld::Ptr, osg::Vec3f> getHitContact(const MWWorld::Ptr& actor, float reach, int* contactPart = nullptr,
        std::string_view swing = {}, std::vector<MeleeFollowThrough>* followThrough = nullptr);

    bool friendlyHit(const MWWorld::Ptr& attacker, const MWWorld::Ptr& target, bool complain);
}

#endif
