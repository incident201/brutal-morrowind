#ifndef OPENMW_MWMECHANICS_MELEEOCCLUSION_H
#define OPENMW_MWMECHANICS_MELEEOCCLUSION_H

#include <algorithm>
#include <vector>

#include <components/settings/values.hpp>

#include "../mwbase/environment.hpp"
#include "../mwbase/world.hpp"
#include "../mwphysics/raycasting.hpp"
#include "../mwworld/class.hpp"

#include "actorutil.hpp"
#include "combat.hpp"
#include "creaturestats.hpp"

namespace MWMechanics
{
    // AI selects hostile combat targets, rather than everyone in the swing.
    // A nearer body still intercepts the blow: filtering physics to only that
    // target let attacks pass through other actors when the render ray missed
    // their mesh. Stop the strike without redirecting damage to the bystander.
    inline bool isNpcMeleeOccluded(const MWWorld::Ptr& attacker, const MWWorld::Ptr& target, const osg::Vec3f& contact)
    {
        if ((!Settings::game().mBrutalMeleeGeometry && !Settings::game().mDarkMessiahNpcCombat)
            || attacker == getPlayer() || target.isEmpty())
            return false;

        const osg::Vec3f origin = getMeleeAim(attacker).first;
        const osg::Vec3f endpoint = Settings::game().mBrutalMeleeGeometry ? contact : getMeleeTorsoPosition(target);
        const auto* rays = MWBase::Environment::get().getWorld()->getRayCasting();
        std::vector<MWWorld::ConstPtr> ignore{ attacker, target };
        while (true)
        {
            // Actor-only geometry supplements the existing rendered contact and
            // world-cover checks; the target's shield may extend outside its hull.
            const auto hit = rays->castRay(origin, endpoint, ignore, {}, MWPhysics::CollisionType_Actor);
            if (!hit.mHit || (hit.mHitPos - origin).length() + .05f >= (endpoint - origin).length())
                return false;
            if (hit.mHitObject.isEmpty() || !hit.mHitObject.getClass().isActor())
                return true;
            if (!hit.mHitObject.getClass().getCreatureStats(hit.mHitObject).isDead())
                return true;

            // A dead actor must not reserve a standing combat slot. Normally its
            // actor hull is already gone; skip one that is still being retired.
            if (std::find(ignore.begin(), ignore.end(), hit.mHitObject) != ignore.end())
                return true;
            ignore.push_back(hit.mHitObject);
        }
    }
}

#endif
