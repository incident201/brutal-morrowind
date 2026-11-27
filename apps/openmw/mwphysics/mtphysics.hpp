#ifndef OPENMW_MWPHYSICS_MTPHYSICS_H
#define OPENMW_MWPHYSICS_MTPHYSICS_H

#include <atomic>
#include <array>
#include <condition_variable>
#include <cstdint>
#include <memory>
#include <map>
#include <string>
#include <optional>
#include <set>
#include <shared_mutex>
#include <thread>
#include <unordered_map>
#include <unordered_set>

#include <BulletCollision/CollisionDispatch/btCollisionWorld.h>

#include <osg/Timer>

#include "components/misc/budgetmeasurement.hpp"
#include "components/esm3/refnum.hpp"
#include "physicssystem.hpp"
#include "ptrholder.hpp"
#include "dynamictransform.hpp"

class btRigidBody;
class btTypedConstraint;
class btActionInterface;

namespace Misc
{
    class Barrier;
}

namespace MWRender
{
    class DebugDrawer;
}

namespace MWPhysics
{
    enum class LockingPolicy
    {
        NoLocks,
        ExclusiveLocksOnly,
        AllowSharedLocks,
    };

    class PhysicsTaskScheduler
    {
    public:
        PhysicsTaskScheduler(float physicsDt, btCollisionWorld* collisionWorld, MWRender::DebugDrawer* debugDrawer);
        ~PhysicsTaskScheduler();

        /// @brief move actors taking into account desired movements and collisions
        /// @param numSteps how much simulation step to run
        /// @param timeAccum accumulated time from previous run to interpolate movements
        /// @param actorsData per actor data needed to compute new positions
        /// @return new position of each actor
        void applyQueuedMovements(float& timeAccum, std::vector<Simulation>& simulations, osg::Timer_t frameStart,
            unsigned int frameNumber, osg::Stats& stats);

        void resetSimulation(const ActorMap& actors);
        void applyObjectImpulse(const std::shared_ptr<Object>& object, const osg::Vec3f& impulse, float mass);
        void addReleasedObject(const std::shared_ptr<Object>& object, const std::vector<osg::Vec3f>& vertices, float mass);
        void setReleasedObjectMotion(const std::shared_ptr<Object>& object, const osg::Vec3f& linear,
            const osg::Vec3f& angular, const std::vector<btRigidBody*>& gripBodies);
        std::vector<osg::Vec3f> readReleasedObject(const std::shared_ptr<Object>& object) const;
        void changeRagdollVelocities(const std::vector<std::pair<btRigidBody*, btVector3>>& changes);
        void configureRagdoll(btActionInterface* action, const struct RagdollControlConfig& config);
        bool applyRagdollImpact(btActionInterface* action, const struct RagdollImpact& event);
        bool applyRagdollKick(btActionInterface* action, std::size_t part, const btVector3& point,
            const btVector3& velocity, std::uint64_t id);
        std::map<std::string, double> readRagdollControl(btActionInterface* action) const;
        struct SuperKickActor
        {
            const btCollisionObject* object;
            osg::Vec3f velocity;
            ESM::RefNum refNum;
        };
        void setSuperKickActors(const std::vector<SuperKickActor>& actors);
        bool hasActiveSuperKicks() const;
        void startSuperKick(std::uint64_t token, std::uint64_t kickId, const std::vector<btRigidBody*>& bodies,
            const osg::Vec3f& velocity, const btCollisionObject* attacker);
        std::map<std::string, double> readSuperKick(std::uint64_t token) const;
        void cancelSuperKick(std::uint64_t token);
        struct SuperKickContact
        {
            std::uint64_t token;
            const btCollisionObject* actor;
            osg::Vec3f transfer;
        };
        std::vector<SuperKickContact> takeSuperKickContacts();
        // Newly created victims can overlap the projectile by the time the main
        // thread receives its witness. Exclude only genuinely overlapping pairs
        // until narrow-phase separation, without changing normal world contacts.
        void guardSuperKickSpawn(const std::vector<btRigidBody*>& bodies);
        // Resolve the current batch once before selecting a render timestamp.
        void waitForRagdollStep();

        void addRagdollBodies(const std::vector<btRigidBody*>& bodies,
            const std::vector<btTypedConstraint*>& constraints, btActionInterface* action);
        void removeRagdollBodies(const std::vector<btRigidBody*>& bodies,
            const std::vector<btTypedConstraint*>& constraints, btActionInterface* action);
        struct RagdollDetachSnapshot
        {
            btTransform before, after;
            btVector3 linearBefore, linearAfter, angularBefore, angularAfter;
            struct Momentum
            {
                btVector3 center{ 0, 0, 0 }, linear{ 0, 0, 0 }, angular{ 0, 0, 0 };
                btScalar mass = 0;
            } retainedBefore, retainedAfter;
            btScalar retainedTransformError = 0, retainedLinearVelocityError = 0,
                     retainedAngularVelocityError = 0;
            btVector3 requestedVelocity{ 0, 0, 0 }, requestedHeadImpulse{ 0, 0, 0 },
                headAngularExpected{ 0, 0, 0 }, appliedHeadImpulse{ 0, 0, 0 };
            btScalar targetLinearVelocityError = 0, unclampedHeadAngularSpeed = 0;
            bool launched = false;
            std::vector<btVector3> partLinearAfter;
        };
        RagdollDetachSnapshot detachRagdollConstraint(btTypedConstraint* constraint, btActionInterface* oldAction,
            btActionInterface* newAction, btRigidBody* detachedBody, const std::vector<btRigidBody*>& bodies,
            const btVector3& cutVelocity, const btVector3& headAngularImpulse);
        // Remove one torso joint and replace its controller under one world lock.
        // Return maximum transform, requested-linear and preserved-angular errors.
        struct RagdollTorsoDetachSnapshot
        {
            // Errors are relative to the intended transforms/velocities, which
            // include the deliberate fragment placement when restoring a save.
            std::array<btScalar, 3> errors{};
            bool poseRestored = false;
            btScalar poseRestoreError = 0, poseRestoreDisplacement = 0,
                poseRestoreRotation = 0, poseAnchorDistance = 0;
        };
        RagdollTorsoDetachSnapshot detachRagdollTorso(btTypedConstraint* constraint,
            btActionInterface* oldAction, btActionInterface* newAction,
            const std::vector<btRigidBody*>& bodies, const btVector3& cutVelocity,
            const btVector3& upperSeparation,
            const std::optional<std::array<btTransform, 2>>& desiredAnchors = {});
        struct RagdollLimbDetachSnapshot
        {
            std::array<btScalar, 3> errors{};
            bool poseRestored = false;
            btScalar poseRestoreError = 0;
        };
        RagdollLimbDetachSnapshot detachRagdollLimbs(const std::vector<btTypedConstraint*>& constraints,
            btActionInterface* oldAction, btActionInterface* newAction,
            const std::vector<btRigidBody*>& bodies, unsigned mask, const btVector3& cutVelocity,
            const std::optional<std::array<btTransform, 11>>& desiredPose = {});
        std::vector<btTransform> readRigidBodyTransforms(const std::vector<btRigidBody*>& bodies,
            std::vector<btVector3>* linear = nullptr, std::vector<btVector3>* angular = nullptr,
            std::vector<btTransform>* rendered = nullptr, std::optional<float> interpolation = std::nullopt) const;
        struct DynamicObjectTransform
        {
            MWWorld::Ptr ptr;
            btTransform authoritative, rendered;
            bool authoritativeChanged;
            bool missile;
            bool settled;
        };
        std::vector<DynamicObjectTransform> readDynamicObjectTransforms();
        void disableDynamicObjects(bool props, bool equipment);
        void holdObject(const std::shared_ptr<Object>& object, const std::vector<osg::Vec3f>& vertices,
            float mass, const btCollisionObject* owner);
        float holdDetachedFragment(const std::vector<btRigidBody*>& bodies, btTypedConstraint* joint,
            const btCollisionObject* owner, osg::Vec3f& minimum, osg::Vec3f& maximum, osg::Matrixf& pickupTransform);
        void setHeldPose(const osg::Vec3f& center, const osg::Quat& rotation, const Object* object);
        float sweepHeldPose(const osg::Vec3f& from, const osg::Vec3f& to,
            const osg::Quat& rotation) const;
        bool releaseHeldObject(const btCollisionObject* owner, const osg::Vec3f& linearVelocity,
            const osg::Vec3f& angularVelocity, bool thrown, bool adrenaline, float mass);
        bool hasHeldObject() const;
        osg::Vec3f getHeldCenter() const;
        struct FlyingObjectHit
        {
            const btCollisionObject* item = nullptr;
            const btCollisionObject* actor = nullptr;
            const btCollisionObject* owner = nullptr;
            osg::Vec3f hitPos;
            osg::Vec3f velocity;
            osg::Vec3f hitNormal;
            float mass = 1.f;
            bool adrenaline = false;
        };
        std::vector<FlyingObjectHit> takeFlyingObjectHits();
        struct DynamicObjectImpact
        {
            const btCollisionObject* item = nullptr;
            osg::Vec3f position;
            osg::Vec3f normal;
            float speed = 0.f;
        };
        std::vector<DynamicObjectImpact> takeDynamicImpacts();

        // Thread safe wrappers
        void rayTest(const btVector3& rayFromWorld, const btVector3& rayToWorld,
            btCollisionWorld::RayResultCallback& resultCallback) const;
        void convexSweepTest(const btConvexShape* castShape, const btTransform& from, const btTransform& to,
            btCollisionWorld::ConvexResultCallback& resultCallback) const;
        void contactTest(btCollisionObject* colObj, btCollisionWorld::ContactResultCallback& resultCallback);
        std::optional<btVector3> getHitPoint(const btTransform& from, btCollisionObject* target);
        void aabbTest(const btVector3& aabbMin, const btVector3& aabbMax, btBroadphaseAabbCallback& callback);
        void getAabb(const btCollisionObject* obj, btVector3& min, btVector3& max);
        void setCollisionFilterMask(btCollisionObject* collisionObject, int collisionFilterMask);
        void addCollisionObject(btCollisionObject* collisionObject, int collisionFilterGroup, int collisionFilterMask);
        void removeCollisionObject(btCollisionObject* collisionObject);
        void updateSingleAabb(const std::shared_ptr<PtrHolder>& ptr, bool immediate = false);
        bool getLineOfSight(const std::shared_ptr<Actor>& actor1, const std::shared_ptr<Actor>& actor2);
        void debugDraw();
        void* getUserPointer(const btCollisionObject* object) const;
        void releaseSharedStates(); // destroy all objects whose destructor can't be safely called from
                                    // ~PhysicsTaskScheduler()

    private:
        class WorkersSync;

        void doSimulation();
        void worker();
        void updateActorsPositions();
        bool hasLineOfSight(const Actor* actor1, const Actor* actor2);
        void refreshLOSCache();
        void updateAabbs();
        void updatePtrAabb(const std::shared_ptr<PtrHolder>& ptr);
        void updateStats(osg::Timer_t frameStart, unsigned int frameNumber, osg::Stats& stats);
        std::tuple<unsigned, float> calculateStepConfig(float timeAccum) const;
        void afterPreStep();
        void afterPostStep();
        void afterPostSim();
        void syncWithMainThread();
        void waitForWorkers();
        void prepareWork(float& timeAccum, std::vector<Simulation>& simulations, osg::Timer_t frameStart,
            unsigned int frameNumber, osg::Stats& stats);

        std::unique_ptr<WorldFrameData> mWorldFrameData;
        std::vector<Simulation>* mSimulations = nullptr;
        std::unordered_set<const btCollisionObject*> mCollisionObjects;
        struct RagdollTransformHistory
        {
            btTransform previous, current;
        };
        // Both the step history and its render clock are protected by mCollisionWorldMutex.
        std::unordered_map<const btRigidBody*, RagdollTransformHistory> mRagdollTransforms;
        std::vector<btActionInterface*> mRagdollActions;
        struct SuperKickProjectile
        {
            std::uint64_t kickId = 0;
            std::vector<btRigidBody*> bodies;
            std::vector<btTransform> before;
            std::vector<btVector3> linear, angular;
            std::unordered_set<const btCollisionObject*> hitActors;
            std::unordered_set<ESM::RefNum> hitRefs;
            const btCollisionObject* attacker = nullptr;
            btVector3 direction{ 0, 0, 0 }, initialCenter{ 0, 0, 0 }, velocity{ 0, 0, 0 },
                lastPoint{ 0, 0, 0 }, lastNormal{ 0, 0, 0 }, lastTransfer{ 0, 0, 0 };
            btScalar age = 0, unsupported = 0, lastFraction = 0, lastContactSpeed = 0;
            unsigned steps = 0, sweeps = 0, contacts = 0, blocked = 0, rejectedSpeed = 0, lastPart = 0;
            int stopped = 0; // 0 active, 1 lifetime, 2 momentum, 3 landing, 4 invalid
            bool airborne = false, lastOverlap = false;
        };
        std::unordered_map<std::uint64_t, SuperKickProjectile> mSuperKicks;
        std::unordered_set<const btCollisionObject*> mSuperKickActors;
        std::unordered_map<const btCollisionObject*, btVector3> mSuperKickActorVelocities;
        std::unordered_map<const btCollisionObject*, ESM::RefNum> mSuperKickActorRefs;
        std::vector<SuperKickContact> mSuperKickContacts;
        void beforeSuperKickStep();
        void afterSuperKickStep(btScalar duration);
        void forgetSuperKickObject(const btCollisionObject* object);
        int mRagdollBaselineSolverMode = 0;
        float mRagdollInterpolation = 1.f;
        std::vector<std::weak_ptr<Object>> mDynamicObjects;
        // Separate registry: loose items must not alter ragdoll substep policy.
        std::unordered_map<const btRigidBody*, DynamicTransformHistory> mDynamicTransforms;
        struct MissileRest
        {
            btTransform pose;
            float duration = 0.f;
        };
        std::unordered_map<const btRigidBody*, MissileRest> mSettlingMissiles;
        void resetDynamicObjectHistory(const Object& object);
        struct HeldState
        {
            btRigidBody* body = nullptr;
            const btCollisionObject* owner = nullptr;
            struct Part
            {
                btRigidBody* body;
                btTransform relative;
                float mass;
            };
            std::vector<Part> fragment;
            btTypedConstraint* joint = nullptr;
        };
        void makeKinematicUnsafe(btRigidBody* body);
        void beginHoldUnsafe(btRigidBody* body, const btCollisionObject* owner);
        mutable std::mutex mHoldStateMutex;
        HeldState mHeldState;
        struct FlyingObjectState
        {
            btRigidBody* body = nullptr;
            const btCollisionObject* owner = nullptr;
            float age = 0.f;
            float mass = 1.f;
            bool adrenaline = false;
            btVector3 prevOrigin{ 0, 0, 0 };
            btRigidBody* secondBody = nullptr;
            btVector3 secondPrevOrigin{ 0, 0, 0 };
        };
        std::vector<FlyingObjectState> mFlyingObjects;
        struct ActorIgnoreEntry
        {
            float timer = 0.f;
            const btCollisionObject* owner = nullptr;
        };
        std::unordered_map<const btRigidBody*, ActorIgnoreEntry> mActorIgnoreTimers;
        std::unordered_map<const btRigidBody*, float> mImpactSoundCooldowns;
        mutable std::mutex mImpactQueueMutex;
        std::vector<FlyingObjectHit> mFlyingHits;
        std::vector<DynamicObjectImpact> mDynamicImpacts;
        void updateFlyingObjectsUnsafe(btScalar dt);
        void collectDynamicImpactsUnsafe(btScalar dt);
        // A thin loose item whose centre of mass crossed a static floor within one step is returned to
        // where it was before the step: Bullet's contact against a triangle mesh flips once a thin hull
        // sinks deeper than half its thickness, and the item then falls through the world.
        void keepDynamicObjectsAboveFloorUnsafe(const std::vector<std::pair<btRigidBody*, btTransform>>& before);
        void clearObjectTrackingUnsafe(btCollisionObject* collisionObject);
        struct ReleasedGripPair { btCollisionObject* item; btCollisionObject* grip; };
        std::vector<ReleasedGripPair> mReleasedGripPairs;
        void clearReleasedGripPairs(btCollisionObject* removed);
        void updateReleasedGripPairs();
        float mDefaultPhysicsDt;
        float mPhysicsDt;
        float mTimeAccum;
        btCollisionWorld* mCollisionWorld;
        MWRender::DebugDrawer* mDebugDrawer;
        std::vector<LOSRequest> mLOSCache;
        std::set<std::weak_ptr<PtrHolder>, std::owner_less<std::weak_ptr<PtrHolder>>> mUpdateAabb;

        // TODO: use std::experimental::flex_barrier or std::barrier once it becomes a thing
        std::unique_ptr<Misc::Barrier> mPreStepBarrier;
        std::unique_ptr<Misc::Barrier> mPostStepBarrier;
        std::unique_ptr<Misc::Barrier> mPostSimBarrier;

        LockingPolicy mLockingPolicy;
        unsigned mNumThreads;
        int mNumJobs;
        unsigned mRemainingSteps;
        int mLOSCacheExpiry;
        bool mAdvanceSimulation;
        std::atomic<int> mNextJob;
        std::atomic<int> mNextLOS;
        std::vector<std::thread> mThreads;

        mutable std::shared_mutex mSimulationMutex;
        mutable std::shared_mutex mCollisionWorldMutex;
        mutable std::shared_mutex mLOSCacheMutex;
        mutable std::mutex mUpdateAabbMutex;

        unsigned int mFrameNumber;
        const osg::Timer* mTimer;

        unsigned mPrevStepCount;
        Misc::BudgetMeasurement mBudget;
        Misc::BudgetMeasurement mAsyncBudget;
        unsigned int mBudgetCursor;
        osg::Timer_t mAsyncStartTime;
        osg::Timer_t mTimeBegin;
        osg::Timer_t mTimeEnd;
        osg::Timer_t mFrameStart;

        std::unique_ptr<WorkersSync> mWorkersSync;
    };

}
#endif
