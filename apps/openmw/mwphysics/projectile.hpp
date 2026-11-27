#ifndef OPENMW_MWPHYSICS_PROJECTILE_H
#define OPENMW_MWPHYSICS_PROJECTILE_H

#include <array>
#include <atomic>
#include <memory>
#include <mutex>
#include <optional>
#include <vector>

#include <LinearMath/btVector3.h>
#include <osg/BoundingBox>

#include "ptrholder.hpp"

class btCollisionObject;
class btCollisionShape;
class btConvexShape;

namespace osg
{
    class Vec3f;
}

namespace MWPhysics
{
    class PhysicsTaskScheduler;
    class PhysicsSystem;

    enum class ProjectileHitType
    {
        World,
        Terrain,
        Water,
        Projectile,
        Shield,
    };

    ProjectileHitType getProjectileHitType(int collisionGroup);

    struct ProjectileShield
    {
        struct Contact
        {
            float mFraction;
            osg::Vec3f mPoint;
            osg::Vec3f mNormal;
        };

        ESM::RefNum mActor;
        const btCollisionObject* mCollider;
        osg::Vec3f mForward;
        osg::BoundingBox mBounds;
        std::vector<std::array<osg::Vec3f, 3>> mTriangles;

        std::optional<Contact> intersect(const osg::Vec3f& from, const osg::Vec3f& to, float limit) const;
    };

    class Projectile final : public PtrHolder
    {
    public:
        Projectile(const MWWorld::Ptr& caster, const osg::Vec3f& position, float radius, bool magic,
            PhysicsTaskScheduler* scheduler, PhysicsSystem* physicssystem);
        ~Projectile() override;

        btConvexShape* getConvexShape() const { return mConvexShape; }

        void updateCollisionObjectPosition();

        bool isActive() const { return mActive.load(std::memory_order_acquire); }
        bool isMagic() const { return mMagic; }

        MWWorld::Ptr getTarget() const;

        MWWorld::Ptr getCaster() const;
        void setCaster(const MWWorld::Ptr& caster);
        const btCollisionObject* getCasterCollisionObject() const { return mCasterColObj; }

        ProjectileHitType getHitType() const { return mHitType; }

        void hit(const btCollisionObject* target, btVector3 pos, btVector3 normal, ProjectileHitType type);
        void hit(ESM::RefNum target, btVector3 pos, btVector3 normal, ProjectileHitType type);

        void setValidTargets(const std::vector<MWWorld::Ptr>& targets);
        bool isValidTarget(const btCollisionObject* target) const;

        btVector3 getHitPosition() const { return mHitPosition; }
        btVector3 getHitNormal() const { return mHitNormal; }

    private:
        std::unique_ptr<btCollisionShape> mShape;
        btConvexShape* mConvexShape;

        const bool mMagic;
        std::atomic<bool> mActive;
        MWWorld::Ptr mCaster;
        const btCollisionObject* mCasterColObj;
        ESM::RefNum mHitTarget;
        ProjectileHitType mHitType = ProjectileHitType::World;
        btVector3 mHitPosition;
        btVector3 mHitNormal;

        std::vector<const btCollisionObject*> mValidTargets;

        mutable std::mutex mMutex;

        PhysicsSystem* mPhysics;
        PhysicsTaskScheduler* mTaskScheduler;

        Projectile(const Projectile&);
        Projectile& operator=(const Projectile&);
    };

}

#endif
