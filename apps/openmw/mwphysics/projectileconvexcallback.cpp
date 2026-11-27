#include <BulletCollision/CollisionDispatch/btCollisionObject.h>

#include "../mwmechanics/combat.hpp"

#include "collisiontype.hpp"
#include "projectile.hpp"
#include "projectileconvexcallback.hpp"

namespace MWPhysics
{
    btScalar ProjectileConvexCallback::addSingleResult(
        btCollisionWorld::LocalConvexResult& result, bool normalInWorldSpace)
    {
        const auto* hitObject = result.m_hitCollisionObject;
        // don't hit the caster
        if (hitObject == mCaster)
            return 1.f;

        // don't hit the projectile
        if (hitObject == mMe)
            return 1.f;

        const int group = hitObject->getBroadphaseHandle()->m_collisionFilterGroup;
        auto* otherProjectile = group == CollisionType_Projectile
            ? static_cast<Projectile*>(hitObject->getUserPointer()) : nullptr;
        if ((group == CollisionType_Actor && !mProjectile.isValidTarget(hitObject))
            || (otherProjectile && (!otherProjectile->isActive()
                || !otherProjectile->getCasterCollisionObject()
                || !mProjectile.isValidTarget(otherProjectile->getCasterCollisionObject()))))
            return 1.f;
        if ((group & (CollisionType_World | CollisionType_Dynamic)) && !mProjectile.isMagic()
            && MWMechanics::physicalProjectiles())
        {
            const auto* holder = static_cast<const PtrHolder*>(hitObject->getUserPointer());
            if (holder && MWMechanics::isMissileItem(holder->getPtr()))
                return 1.f;
        }

        return btCollisionWorld::ClosestConvexResultCallback::addSingleResult(result, normalInWorldSpace);
    }

    void ProjectileConvexCallback::applyHit()
    {
        if (!hasHit() || !mProjectile.isActive())
            return;
        const int group = m_hitCollisionObject->getBroadphaseHandle()->m_collisionFilterGroup;
        if (group == CollisionType_Projectile)
        {
            auto* other = static_cast<Projectile*>(m_hitCollisionObject->getUserPointer());
            other->hit(mMe, m_hitPointWorld, -m_hitNormalWorld, ProjectileHitType::Projectile);
        }
        mProjectile.hit(m_hitCollisionObject, m_hitPointWorld, m_hitNormalWorld, getProjectileHitType(group));
    }

}
