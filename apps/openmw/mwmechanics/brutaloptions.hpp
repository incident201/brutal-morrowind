#ifndef GAME_MWMECHANICS_BRUTALOPTIONS_H
#define GAME_MWMECHANICS_BRUTALOPTIONS_H

#include <components/esm3/loadweap.hpp>
#include <components/settings/values.hpp>

namespace MWWorld
{
    class Ptr;
}

namespace MWMechanics
{
    // Dark Messiah's wounded NPC (life_tohurt): a living NPC other than the player whose health
    // is below the wounded fraction of its maximum.
    bool isBrutalWoundedNpc(const MWWorld::Ptr& actor);
    // A charged melee blow: the player's held, non-fast strike or an NPC's chosen guard breaker.
    bool isBrutalChargedAttack(const MWWorld::Ptr& attacker, float strength);

    // Dark Messiah first-person mode. The classic mode restores OpenMW's own arms,
    // attack animations, passive block and third-person toggle at run time.
    inline bool brutalDmFirstPerson()
    {
        return Settings::game().mBrutalEnabled && Settings::game().mFirstPersonRigidViewmodel && !Settings::game().mBrutalClassicFirstPerson;
    }

    // Requested dual-wield preference is owned by the permanent Lua settings section.
    // This runtime switch avoids a second, potentially conflicting settings.cfg owner.
    inline bool sBrutalDualWield = true;
    inline bool brutalDualWield()
    {
        return Settings::game().mBrutalEnabled && sBrutalDualWield;
    }

    inline bool brutalStaffWeapon(int type)
    {
        return type == ESM::Weapon::BluntTwoWide || type == ESM::Weapon::SpearTwoWide;
    }

    inline bool brutalPairedWeapon(int type)
    {
        return type == ESM::Weapon::LongBladeTwoHand || type == ESM::Weapon::AxeTwoHand
            || type == ESM::Weapon::BluntTwoClose;
    }
}

#endif
