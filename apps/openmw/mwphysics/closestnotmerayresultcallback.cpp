#include "closestnotmerayresultcallback.hpp"

#include <algorithm>

#include <BulletCollision/CollisionDispatch/btCollisionObject.h>

#include "../mwmechanics/combat.hpp"

#include "collisiontype.hpp"
#include "ptrholder.hpp"

namespace MWPhysics
{
    btScalar ClosestNotMeRayResultCallback::addSingleResult(
        btCollisionWorld::LocalRayResult& rayResult, bool normalInWorldSpace)
    {
        const auto* hitObject = rayResult.m_collisionObject;
        if (std::find(mIgnoreList.begin(), mIgnoreList.end(), hitObject) != mIgnoreList.end())
            return 1.f;

        const int group = hitObject->getBroadphaseHandle()->m_collisionFilterGroup;
        if (group == CollisionType_Actor && !mTargets.empty())
        {
            if ((std::find(mTargets.begin(), mTargets.end(), hitObject) == mTargets.end()))
                return 1.f;
        }

        if (m_collisionFilterGroup == CollisionType_Projectile && MWMechanics::physicalProjectiles()
            && (group & (CollisionType_World | CollisionType_Dynamic)))
        {
            const auto* holder = static_cast<const PtrHolder*>(hitObject->getUserPointer());
            if (holder && MWMechanics::isMissileItem(holder->getPtr()))
                return 1.f;
        }

        return btCollisionWorld::ClosestRayResultCallback::addSingleResult(rayResult, normalInWorldSpace);
    }
}
