#include <memory>

#include <BulletCollision/CollisionShapes/btSphereShape.h>

#include <components/misc/convert.hpp>

#include "../mwbase/environment.hpp"
#include "../mwrender/npcguardik.hpp"
#include "../mwworld/worldmodel.hpp"

#include "actor.hpp"
#include "collisiontype.hpp"
#include "mtphysics.hpp"
#include "object.hpp"
#include "projectile.hpp"

namespace MWPhysics
{
    ProjectileHitType getProjectileHitType(int collisionGroup)
    {
        if (collisionGroup == CollisionType_Water)
            return ProjectileHitType::Water;
        if (collisionGroup == CollisionType_HeightMap)
            return ProjectileHitType::Terrain;
        if (collisionGroup == CollisionType_Projectile)
            return ProjectileHitType::Projectile;
        return ProjectileHitType::World;
    }

    std::optional<ProjectileShield::Contact> ProjectileShield::intersect(
        const osg::Vec3f& from, const osg::Vec3f& to, float limit) const
    {
        const osg::Vec3f travel = to - from;
        const float horizontalLength = std::hypot(travel.x(), travel.y());
        constexpr float minimumFrontalDot = 0.34202014f; // cos(70 degrees)
        if (horizontalLength < 1e-3f || -(travel * mForward) < minimumFrontalDot * horizontalLength)
            return {};

        // Reject rays outside the world-space bounds before testing the shield triangles.
        float start = 0.f;
        float end = limit;
        for (unsigned axis = 0; axis < 3; ++axis)
        {
            if (std::abs(travel[axis]) < 1e-6f)
            {
                if (from[axis] < mBounds._min[axis] || from[axis] > mBounds._max[axis])
                    return {};
                continue;
            }
            const float t0 = (mBounds._min[axis] - from[axis]) / travel[axis];
            const float t1 = (mBounds._max[axis] - from[axis]) / travel[axis];
            const auto [near, far] = std::minmax(t0, t1);
            start = std::max(start, near);
            end = std::min(end, far);
            if (start > end)
                return {};
        }

        std::optional<Contact> closest;
        for (const auto& triangle : mTriangles)
        {
            const auto fraction = MWRender::NpcGuardIk::segmentTriangleHit(from, to, triangle);
            if (!fraction || *fraction > limit)
                continue;
            osg::Vec3f normal = (triangle[1] - triangle[0]) ^ (triangle[2] - triangle[0]);
            normal.normalize();
            if (normal * travel > 0.f)
                normal = -normal;
            closest = Contact{ *fraction, from + travel * *fraction, normal };
            limit = *fraction;
        }
        return closest;
    }

    Projectile::Projectile(const MWWorld::Ptr& caster, const osg::Vec3f& position, float radius, bool magic,
        PhysicsTaskScheduler* scheduler, PhysicsSystem* physicssystem)
        : PtrHolder(MWWorld::Ptr(), position)
        , mMagic(magic)
        , mActive(true)
        , mPhysics(physicssystem)
        , mTaskScheduler(scheduler)
    {
        mShape = std::make_unique<btSphereShape>(radius);
        mConvexShape = static_cast<btConvexShape*>(mShape.get());

        mCollisionObject = std::make_unique<btCollisionObject>();
        mCollisionObject->setCollisionFlags(btCollisionObject::CF_KINEMATIC_OBJECT);
        mCollisionObject->setActivationState(DISABLE_DEACTIVATION);
        mCollisionObject->setCollisionShape(mShape.get());
        mCollisionObject->setUserPointer(this);

        mPosition = position;
        mPreviousPosition = position;
        mSimulationPosition = position;
        setCaster(caster);

        const int collisionMask = CollisionType_World | CollisionType_HeightMap | CollisionType_Actor
            | CollisionType_Door | CollisionType_Water | CollisionType_Projectile;
        mTaskScheduler->addCollisionObject(mCollisionObject.get(), CollisionType_Projectile, collisionMask);

        updateCollisionObjectPosition();
    }

    Projectile::~Projectile()
    {
        if (!mActive)
            mPhysics->reportCollision(mHitPosition, mHitNormal);
        mTaskScheduler->removeCollisionObject(mCollisionObject.get());
    }

    void Projectile::updateCollisionObjectPosition()
    {
        std::scoped_lock lock(mMutex);
        auto& trans = mCollisionObject->getWorldTransform();
        trans.setOrigin(Misc::Convert::toBullet(mPosition));
        mCollisionObject->setWorldTransform(trans);
    }

    void Projectile::hit(const btCollisionObject* target, btVector3 pos, btVector3 normal, ProjectileHitType type)
    {
        // Earlier impacts in this batch can replace the collider or remove its owner.
        const auto* holder = static_cast<const PtrHolder*>(target->getUserPointer());
        const auto ptr = holder ? holder->getPtr() : MWWorld::Ptr();
        hit(ptr.isEmpty() ? ESM::RefNum{} : ptr.getCellRef().getRefNum(), pos, normal, type);
    }

    void Projectile::hit(ESM::RefNum target, btVector3 pos, btVector3 normal, ProjectileHitType type)
    {
        std::scoped_lock lock(mMutex);
        if (!mActive.load(std::memory_order_relaxed))
            return;
        mHitTarget = target;
        mHitPosition = pos;
        mHitNormal = normal;
        mHitType = type;
        mActive.store(false, std::memory_order_release);
    }

    MWWorld::Ptr Projectile::getTarget() const
    {
        assert(!isActive());
        if (!mHitTarget.isSet())
            return {};
        const auto ptr = MWBase::Environment::get().getWorldModel()->getPtr(mHitTarget);
        return ptr.isInCell() && !ptr.mRef->isDeleted() && ptr.getRefData().isEnabled() ? ptr : MWWorld::Ptr();
    }

    MWWorld::Ptr Projectile::getCaster() const
    {
        return mCaster;
    }

    void Projectile::setCaster(const MWWorld::Ptr& caster)
    {
        mCaster = caster;
        mCasterColObj = [this, &caster]() -> const btCollisionObject* {
            const Actor* actor = mPhysics->getActor(caster);
            if (actor)
                return actor->getCollisionObject();
            const Object* object = mPhysics->getObject(caster);
            if (object)
                return object->getCollisionObject();
            return nullptr;
        }();
    }

    void Projectile::setValidTargets(const std::vector<MWWorld::Ptr>& targets)
    {
        std::scoped_lock lock(mMutex);
        mValidTargets.clear();
        for (const auto& ptr : targets)
        {
            const auto* physicActor = mPhysics->getActor(ptr);
            if (physicActor)
                mValidTargets.push_back(physicActor->getCollisionObject());
        }
    }

    bool Projectile::isValidTarget(const btCollisionObject* target) const
    {
        assert(target);
        std::scoped_lock lock(mMutex);
        if (mCasterColObj == target)
            return false;

        if (mValidTargets.empty())
            return true;

        return std::any_of(mValidTargets.begin(), mValidTargets.end(),
            [target](const btCollisionObject* actor) { return target == actor; });
    }

}
