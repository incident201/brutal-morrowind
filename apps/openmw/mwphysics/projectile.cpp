#include <memory>

#include <BulletCollision/CollisionShapes/btSphereShape.h>

#include <components/misc/convert.hpp>

#include "../mwbase/environment.hpp"
#include "../mwworld/worldmodel.hpp"

#include "actor.hpp"
#include "collisiontype.hpp"
#include "mtphysics.hpp"
#include "object.hpp"
#include "projectile.hpp"

namespace MWPhysics
{
    Projectile::Projectile(const MWWorld::Ptr& caster, const osg::Vec3f& position, float radius, bool magic,
        PhysicsTaskScheduler* scheduler, PhysicsSystem* physicssystem)
        : PtrHolder(MWWorld::Ptr(), position)
        , mMagic(magic)
        , mHitWater(false)
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

    void Projectile::hit(const btCollisionObject* target, btVector3 pos, btVector3 normal, bool water)
    {
        std::scoped_lock lock(mMutex);
        if (!mActive.load(std::memory_order_relaxed))
            return;

        // Earlier impacts in this batch can replace the collider or remove its owner.
        const auto* holder = static_cast<const PtrHolder*>(target->getUserPointer());
        if (holder)
        {
            const auto ptr = holder->getPtr();
            if (!ptr.isEmpty())
                mHitTarget = ptr.getCellRef().getRefNum();
        }
        mHitPosition = pos;
        mHitNormal = normal;
        mHitWater = water;
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
