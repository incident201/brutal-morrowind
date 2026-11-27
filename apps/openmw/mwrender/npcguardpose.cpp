#include "npcanimation.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <optional>
#include <utility>
#include <vector>

#include <osg/BoundingBox>
#include <osg/FrameStamp>
#include <osg/Geometry>
#include <osg/NodeCallback>
#include <osg/TriangleIndexFunctor>

#include <components/sceneutil/positionattitudetransform.hpp>
#include <components/sceneutil/skeleton.hpp>
#include <components/settings/values.hpp>

#include "../mwbase/environment.hpp"
#include "../mwbase/world.hpp"
#include "../mwmechanics/actorutil.hpp"
#include "../mwmechanics/combat.hpp"
#include "../mwmechanics/creaturestats.hpp"
#include "../mwworld/class.hpp"
#include "actorutil.hpp"
#include "animblendcontroller.hpp"
#include "npcguardik.hpp"
#include "vismask.hpp"

namespace MWRender
{
    using namespace NpcGuardIk;

    struct NpcAnimation::NpcGuardPose
    {
        struct Callback : osg::NodeCallback
        {
            osg::observer_ptr<NpcAnimation> owner;
            explicit Callback(NpcAnimation* animation)
                : owner(animation)
            {
            }
            void operator()(osg::Node* node, osg::NodeVisitor* visitor) override
            {
                osg::ref_ptr<NpcAnimation> animation;
                const bool update
                    = visitor && visitor->getVisitorType() == osg::NodeVisitor::UPDATE_VISITOR && owner.lock(animation);
                if (update)
                    animation->restoreNpcGuardPose();
                traverse(node, visitor);
                // All keyframe, blend and aiming controllers have evaluated.
                // These are the same matrices that feed the following CULL.
                if (update)
                    animation->fitNpcGuardPose(*visitor);
            }
        };

        bool enabled = false, shield = false, prepared = false;
        bool orientationPrepared = false, witnessPrepared = false, choicePrepared = false;
        bool published = false, cacheValid = false, cachedSafe = false, cachedExact = false;
        bool referencePrepared = false, releasing = false;
        float pendingDuration = 0.f, pendingElapsed = 0.f, weight = 0.f;
        float filteredOffset = 8.f, filteredSwivel = 0.f;
        osg::Vec3f point, direction, stroke;
        osg::Vec3f localPoint, localDirection, localStroke;
        osg::observer_ptr<osg::Node> root, item;
        bool meshSnapshotValid = false, meshActorValid = false;
        unsigned meshFrame = 0;
        osg::Matrixf meshItemWorld, meshItemActor;
        osg::ref_ptr<Callback> callback;
        std::vector<std::pair<osg::observer_ptr<osg::MatrixTransform>, osg::Matrixf>> source;
        std::vector<std::array<osg::Vec3f, 3>> triangles;
        std::vector<osg::Vec3f> witnesses;
        std::vector<osg::Vec3f> clearancePoints;
        osg::Vec3f meshCenter;
        osg::Vec3f axis;
        std::array<osg::Vec3f, 2> shieldTangents;
        osg::Quat filteredTurn;
        osg::Vec3f filteredWitness;
        osg::Vec3f filteredGoal;
        std::array<osg::Quat, 3> publishedRotation;
        std::array<osg::Matrixf, 3> cachedSource;
        osg::Matrixf cachedMetricToActor;
        osg::Quat cachedTurn;
        osg::Vec3f cachedGoal, cachedPoint, cachedDirection, cachedStroke;
        float cachedOffset = 8.f, cachedSwivel = 0.f;
        float releaseElapsed = 0.f, releaseStartWeight = 0.f;
        std::array<osg::Quat, 3> publishedDelta, releaseDelta;
        std::array<osg::Matrixf, 3> referenceRotation;
        osg::Matrixf referenceForeZero, referenceHandActor;
        osg::Vec3f referenceUpperHinge, referenceForeHinge, referencePalmForward, referencePalmLateral;
        unsigned selectedWitness = 0;
        osg::Vec3f selectedAxis;
        float minimumMaterialCoordinate = -std::numeric_limits<float>::infinity();
        float maximumMaterialCoordinate = std::numeric_limits<float>::infinity();

        explicit NpcGuardPose(NpcAnimation* animation)
            : callback(new Callback(animation))
        {
        }
    };

    bool NpcAnimation::hasNpcGuardMeshForCombat() const
    {
        if (mNpcTwoHandGuardPose)
            return hasNpcTwoHandGuardMeshForCombat();
        if (!mNpcGuardPose || !mNpcGuardPose->meshSnapshotValid || !mNpcGuardPose->meshActorValid)
            return false;
        const auto& pose = *mNpcGuardPose;
        osg::Node* current = pose.shield                    ? mShowCarriedLeft && mObjectParts[ESM::PRT_Shield]
                ? mObjectParts[ESM::PRT_Shield]->getNode().get()
                : nullptr
            : mShowWeapons && mObjectParts[ESM::PRT_Weapon] ? mObjectParts[ESM::PRT_Weapon]->getNode().get()
                                                            : nullptr;
        return current && current == pose.item.get();
    }

    std::optional<NpcAnimation::NpcGuardMeshTrace> NpcAnimation::traceNpcGuardMeshForAudit(
        const osg::Vec3f& origin, const osg::Vec3f& end) const
    {
        return traceNpcGuardMeshSnapshot(origin, end, false);
    }

    std::optional<NpcAnimation::NpcGuardMeshTrace> NpcAnimation::traceNpcGuardMeshForCombat(
        const osg::Vec3f& origin, const osg::Vec3f& end) const
    {
        return traceNpcGuardMeshSnapshot(origin, end, true);
    }

    std::optional<NpcAnimation::NpcGuardMeshTrace> NpcAnimation::traceNpcGuardMeshSnapshot(
        const osg::Vec3f& origin, const osg::Vec3f& end, bool combat, osg::Matrixf* itemToWorld) const
    {
        if (mNpcTwoHandGuardPose)
            return traceNpcTwoHandGuardMeshForAudit(origin, end, combat);
        if (!mNpcGuardPose || !mNpcGuardPose->meshSnapshotValid || !finite(origin) || !finite(end)
            || mNpcGuardPose->meshFrame != mNpcGuardGeometry.frame)
            return {};
        const auto& pose = *mNpcGuardPose;
        auto itemWorld = pose.meshItemWorld;
        if (combat)
        {
            if (!hasNpcGuardMeshForCombat())
                return {};
            const auto actorWorld = nodeWorld(mPtr.getRefData().getBaseNode());
            if (!actorWorld)
                return {};
            itemWorld = pose.meshItemActor * *actorWorld;
        }
        if (itemToWorld != nullptr)
            *itemToWorld = itemWorld;
        SegmentMeshGap nearest;
        float firstHit = std::numeric_limits<float>::infinity();
        for (const auto& local : pose.triangles)
        {
            std::array<osg::Vec3f, 3> triangle;
            for (unsigned point = 0; point < 3; ++point)
                triangle[point] = local[point] * itemWorld;
            if (!finite(triangle[0]) || !finite(triangle[1]) || !finite(triangle[2])
                || ((triangle[1] - triangle[0]) ^ (triangle[2] - triangle[0])).length2() < 1e-12f)
                continue;
            if (const auto hit = segmentTriangleHit(origin, end, triangle))
                firstHit = std::min(firstHit, *hit);
            if (std::isfinite(firstHit))
                continue;
            for (const auto& endpoint : { origin, end })
            {
                const auto meshPoint = closestPoint(endpoint, triangle);
                const double squared = (osg::Vec3d(endpoint) - osg::Vec3d(meshPoint)).length2();
                if (squared < nearest.squared)
                    nearest = { squared, meshPoint };
            }
            for (unsigned edge = 0; edge < 3; ++edge)
            {
                const auto gap = segmentEdgeGap(origin, end, triangle[edge], triangle[(edge + 1) % 3]);
                if (gap.squared < nearest.squared)
                    nearest = gap;
            }
        }
        if (std::isfinite(firstHit))
            return NpcGuardMeshTrace{ pose.meshFrame, true, 0.f, origin + (end - origin) * firstHit };
        if (std::isfinite(nearest.squared))
            return NpcGuardMeshTrace{ pose.meshFrame, false, float(std::sqrt(nearest.squared)), nearest.point };
        return {};
    }

    ProjectileAttachment NpcAnimation::attachStuckProjectileToShield(
        VFS::Path::NormalizedView model, const osg::Vec3f& hitPosition,
        const osg::Vec3f& flight, const std::pair<osg::Vec3f, osg::Vec3f>& path, const ESM::RefId& item)
    {
        osg::Node* part = getPartNode(ESM::PRT_Shield);
        osg::Group* shield = part != nullptr ? part->asGroup() : nullptr;
        if (shield == nullptr)
            return ProjectileAttachment::NoSurface;
        auto& inventory = mPtr.getClass().getInventoryStore(mPtr);
        const auto held = inventory.getSlot(MWWorld::InventoryStore::Slot_CarriedLeft);
        if (held == inventory.end() || held->getType() != ESM::Armor::sRecordId)
            return ProjectileAttachment::NoSurface;
        const auto heldItem = *held;
        auto& saved = heldItem.getRefData().getAnimationState().mEquippedShieldProjectiles;
        if (!item.empty() && saved.size() >= ESM::AnimationState::MaxStuckProjectiles)
            return ProjectileAttachment::Crowded;

        auto result = ProjectileAttachment::NoSurface;
        if (mNpcGuardPose && mNpcGuardPose->shield && !mNpcTwoHandGuardPose)
        {
            // The fitted pose used for the hit can be restored to the source
            // animation between draw updates. Mount against the same mesh pose as combat.
            osg::Matrixf itemToWorld;
            const auto contact = traceNpcGuardMeshSnapshot(path.first, path.second, true, &itemToWorld);
            if (contact && contact->gap <= MWMechanics::sNpcProjectileGuardContactMargin)
                result = addStuckProjectile(model, contact->point + flight * 3.f, flight, item, *shield, itemToWorld);
        }
        if (result == ProjectileAttachment::NoSurface)
            if (const auto contact = findProjectileContact(hitPosition, flight, shield))
                result = attachStuckProjectile(model, *contact, flight, item, shield);
        if (result == ProjectileAttachment::Attached && !item.empty())
        {
            const auto transform = mStuckProjectiles.back().mNode->getMatrix();
            auto& entry = saved.emplace_back();
            entry.mItem = item;
            std::copy_n(transform.ptr(), entry.mTransform.size(), entry.mTransform.begin());
        }
        return result;
    }

    void NpcAnimation::setNpcGuardTarget(bool enabled, bool shield, const osg::Vec3f& worldPoint,
        const osg::Vec3f& incomingDirection, const osg::Vec3f& strokeDirection)
    {
        if (!enabled && !mNpcGuardPose && !mNpcTwoHandGuardPose)
            return;
        if (!shield && usesNpcTwoHandGuard())
        {
            clearNpcGuardPose();
            setNpcTwoHandGuardTarget(enabled, worldPoint, incomingDirection);
            return;
        }
        clearNpcTwoHandGuardPose();
        enabled = enabled && finite(worldPoint) && finite(incomingDirection) && incomingDirection.length2() > .01f;
        if (!mNpcGuardPose)
        {
            if (!enabled)
                return;
            mNpcGuardPose = std::make_shared<NpcGuardPose>(this);
        }
        auto& pose = *mNpcGuardPose;
        if (pose.enabled && !enabled && pose.published)
        {
            pose.releasing = true;
            pose.releaseElapsed = 0.f;
            pose.releaseStartWeight = pose.weight * pose.weight * (3.f - 2.f * pose.weight);
            pose.releaseDelta = pose.publishedDelta;
        }
        if (enabled)
        {
            pose.releasing = false;
            if (!pose.enabled && pose.weight <= 0.f && !pose.published)
            {
                pose.prepared = pose.orientationPrepared = pose.witnessPrepared = pose.choicePrepared = false;
                pose.cacheValid = false;
            }
            if (pose.shield != shield)
            {
                restoreNpcGuardPose();
                pose.item = nullptr;
                pose.weight = 0.f;
                pose.published = pose.cacheValid = false;
                pose.referencePrepared = pose.releasing = false;
                pose.prepared = pose.orientationPrepared = pose.witnessPrepared = pose.choicePrepared = false;
            }
            pose.shield = shield;
            pose.point = worldPoint;
            pose.direction = incomingDirection;
            pose.direction.normalize();
            pose.stroke = finite(strokeDirection) ? strokeDirection : osg::Vec3f();
        }
        pose.enabled = enabled;
        mNpcGuardGeometry.enabled = enabled;
        updateNpcGuardPose(0.f);
    }

    void NpcAnimation::updateNpcGuardPose(float duration)
    {
        if (!mNpcGuardPose)
            return;
        auto& pose = *mNpcGuardPose;
        pose.pendingDuration += std::clamp(duration, 0.f, .1f);
        pose.pendingElapsed += std::max(duration, 0.f);
        if (pose.root.get() != mObjectRoot.get())
        {
            restoreNpcGuardPose();
            if (pose.root.valid())
                pose.root->removeUpdateCallback(pose.callback);
            pose.root = mObjectRoot.get();
            pose.item = nullptr;
            pose.prepared = pose.orientationPrepared = pose.witnessPrepared = pose.choicePrepared = false;
            pose.weight = 0.f;
            pose.published = pose.cacheValid = false;
            pose.referencePrepared = pose.releasing = false;
            if (mObjectRoot)
                mObjectRoot->addUpdateCallback(pose.callback);
        }
    }

    void NpcAnimation::restoreNpcGuardPose()
    {
        if (!mNpcGuardPose || mNpcGuardPose->source.empty())
            return;
        for (const auto& [bone, matrix] : mNpcGuardPose->source)
            if (bone.valid())
                bone->setMatrix(matrix);
        mNpcGuardPose->source.clear();
        if (mSkeleton)
            mSkeleton->markDirty();
    }

    void NpcAnimation::clearNpcGuardPose()
    {
        if (!mNpcGuardPose)
            return;
        restoreNpcGuardPose();
        if (mNpcGuardPose->root.valid())
            mNpcGuardPose->root->removeUpdateCallback(mNpcGuardPose->callback);
        mNpcGuardPose.reset();
        mNpcGuardGeometry = {};
        mNpcGuardRigAudit.reset();
        mNpcGuardRigAuditRequested = false;
    }

    void NpcAnimation::fitNpcGuardPose(osg::NodeVisitor& visitor)
    {
        if (!mNpcGuardPose)
            return;
        auto& pose = *mNpcGuardPose;
        pose.meshSnapshotValid = false;
        pose.meshActorValid = false;
        auto& audit = mNpcGuardGeometry;
        audit = {};
        audit.enabled = pose.enabled;
        audit.shield = pose.shield;
        audit.referenceReady = pose.referencePrepared;
        audit.frame = visitor.getTraversalNumber();
        const float duration = std::exchange(pose.pendingDuration, 0.f);
        const float elapsed = std::exchange(pose.pendingElapsed, 0.f);
        if (!pose.enabled && !pose.releasing)
            return;
        const auto& stats = mPtr.getClass().getCreatureStats(mPtr);
        if (mPtr == MWMechanics::getPlayer() || mBodyAwareness || mViewMode != VM_Normal
            || !Settings::game().mDarkMessiahNpcCombat || Settings::game().mBrutalClassicNpcAnimations || stats.isDead()
            || stats.getKnockedDown() || stats.isParalyzed() || isRagdollPose() || stats.mBrutalClinchActive
            || (stats.getHitRecovery() && !stats.mBrutalNpcGuardRequested))
        {
            pose.enabled = pose.prepared = false;
            pose.weight = 0.f;
            pose.published = pose.cacheValid = false;
            pose.releasing = false;
            mNpcGuardRigAudit.reset();
            mNpcGuardRigAuditRequested = false;
            audit.enabled = false;
            return;
        }
        osg::Node* item = pose.shield ? mShowCarriedLeft && mObjectParts[ESM::PRT_Shield]
                ? mObjectParts[ESM::PRT_Shield]->getNode().get()
                : nullptr
            : mShowWeapons            ? getWeaponNode()
                                      : nullptr;
        const auto itemWorld = nodeWorld(item);
        if (!itemWorld)
        {
            pose.weight = 0.f;
            pose.published = pose.cacheValid = false;
            pose.releasing = false;
            pose.prepared = false;
            mNpcGuardRigAudit.reset();
            mNpcGuardRigAuditRequested = false;
            return;
        }
        if (pose.item.get() != item)
        {
            pose.item = item;
            pose.weight = 0.f;
            pose.published = pose.cacheValid = false;
            pose.referencePrepared = pose.releasing = false;
            pose.prepared = pose.orientationPrepared = pose.witnessPrepared = pose.choicePrepared = false;
            pose.triangles.clear();
            pose.witnesses.clear();
            pose.clearancePoints.clear();
            GuardMeshVisitor mesh(item);
            item->accept(mesh);
            pose.triangles = std::move(mesh.triangles);
            if (!mesh.bounds.valid() || pose.triangles.empty())
                return;
            const osg::Vec3f span = mesh.bounds._max - mesh.bounds._min;
            pose.meshCenter = mesh.bounds.center();
            unsigned axis = 0;
            for (unsigned i = 1; i < 3; ++i)
                if (pose.shield ? span[i] < span[axis] : span[i] > span[axis])
                    axis = i;
            pose.axis = {};
            pose.axis[axis] = 1.f;
            pose.minimumMaterialCoordinate = -std::numeric_limits<float>::infinity();
            pose.maximumMaterialCoordinate = std::numeric_limits<float>::infinity();
            const auto witness = [&](const osg::Vec3f& desired, float minimum, float maximum) {
                if (const auto result = materialWitness(pose.triangles, desired, pose.axis, minimum, maximum))
                    pose.witnesses.push_back(*result);
            };
            if (pose.shield)
            {
                osg::Vec3f normalSum;
                for (const auto& triangle : pose.triangles)
                {
                    const auto first = triangle[1] - triangle[0];
                    const auto second = triangle[2] - triangle[0];
                    auto normal = first ^ second;
                    const float area = normal.normalize();
                    const float alignment = normal * pose.axis;
                    if (std::abs(alignment) > .5f)
                        normalSum += normal * (area * (alignment >= 0.f ? 1.f : -1.f));
                }
                if (normalSum.normalize() < .001f)
                    normalSum = pose.axis;
                osg::Vec3f tangent;
                tangent[(axis + 1) % 3] = 1.f;
                tangent -= normalSum * (tangent * normalSum);
                tangent.normalize();
                pose.shieldTangents = { tangent, normalSum ^ tangent };
                witness(mesh.bounds.center(), -std::numeric_limits<float>::infinity(),
                    std::numeric_limits<float>::infinity());
                // A low blow can meet the lower face while the wrist stays
                // above it. Every anchor is projected onto real shield material;
                // no invented radius or unattached shield translation is used.
                const unsigned first = (axis + 1) % 3, second = (axis + 2) % 3;
                for (float u : { -1.f, -.5f, 0.f, .5f, 1.f })
                    for (float v : { -1.f, -.5f, 0.f, .5f, 1.f })
                    {
                        if (u == 0.f && v == 0.f)
                            continue;
                        auto desired = mesh.bounds.center();
                        desired[first] += span[first] * .5f * u;
                        desired[second] += span[second] * .5f * v;
                        witness(desired, pose.minimumMaterialCoordinate, pose.maximumMaterialCoordinate);
                    }
            }
            else
            {
                // Native blade meshes place their grip at the item origin. Use
                // only actual blade material, excluding the hilt and unstable
                // tip. Prefer its near-hilt part: using a far contact for a low
                // blow needlessly leaves the hand and elbow at head height.
                const float tip = std::abs(mesh.bounds._max[axis]) >= std::abs(mesh.bounds._min[axis])
                    ? mesh.bounds._max[axis]
                    : mesh.bounds._min[axis];
                pose.axis[axis] = tip >= 0.f ? 1.f : -1.f;
                const float reach = std::abs(tip);
                pose.minimumMaterialCoordinate = reach * .25f;
                pose.maximumMaterialCoordinate = reach * .75f;
                for (float fraction : { .25f, .3f, .4f, .5f, .6f, .7f })
                    witness(
                        pose.axis * (reach * fraction), pose.minimumMaterialCoordinate, pose.maximumMaterialCoordinate);
            }
            pose.clearancePoints = pose.witnesses;
            for (unsigned coordinate = 0; coordinate < 3; ++coordinate)
                for (float value : { mesh.bounds._min[coordinate], mesh.bounds._max[coordinate] })
                {
                    auto desired = mesh.bounds.center();
                    desired[coordinate] = value;
                    if (const auto point = materialWitness(pose.triangles, desired, pose.axis,
                            -std::numeric_limits<float>::infinity(), std::numeric_limits<float>::infinity()))
                        pose.clearancePoints.push_back(*point);
                }
        }
        audit.meshAvailable = !pose.witnesses.empty();
        if (pose.witnesses.empty())
            return;
        pose.meshItemWorld = *itemWorld;
        pose.meshFrame = audit.frame;
        pose.meshSnapshotValid = true;
        if (const auto actorWorld = nodeWorld(mPtr.getRefData().getBaseNode()))
        {
            pose.meshItemActor = *itemWorld * osg::Matrixf::inverse(*actorWorld);
            pose.meshActorValid = true;
        }

        const unsigned side = pose.shield ? 1 : 0;
        GripArm arm;
        if (!captureGuardArm(getNodeMap(), pose.shield, arm))
            return;
        const std::string_view guardGroup = pose.shield ? "shield" : "brutal_weapon_guard";
        audit.sourceGroupMatches = isPlaying(guardGroup)
            && getActiveGroup(pose.shield ? BoneGroup_LeftArm : BoneGroup_RightArm) == guardGroup
            && getActiveGroup(BoneGroup_Torso) == guardGroup;
        audit.sourceTime = getCurrentTime(guardGroup);
        audit.sourceHoldTime = getTextKeyTime(std::string(guardGroup) + ": brutal guard hold");
        float sourceSpeed = 1.f;
        const bool sourceActive = getInfo(guardGroup, nullptr, &sourceSpeed);
        audit.sourceHoldReached = sourceActive && audit.sourceHoldTime >= 0.f
            && std::abs(audit.sourceTime - audit.sourceHoldTime) < .0005f && std::abs(sourceSpeed) < .0001f;
        const auto inspectBlend = [&](osg::Node* bone) {
            const osg::ref_ptr<osg::Node> node = bone;
            const auto isActive = [&](osg::Callback* callback) {
                return std::any_of(mActiveControllers.begin(), mActiveControllers.end(),
                    [&](const auto& active) { return active.first.get() == bone && active.second.get() == callback; });
            };
            if (const auto controller = mAnimBlendControllers.find(node); controller != mAnimBlendControllers.end())
                audit.sourceBlendActive = audit.sourceBlendActive
                    || (isActive(controller->second->getAsCallback()) && controller->second->isBlending());
            if (const auto controller = mBoneAnimBlendControllers.find(node);
                controller != mBoneAnimBlendControllers.end())
                audit.sourceBlendActive = audit.sourceBlendActive
                    || (isActive(controller->second->getAsCallback()) && controller->second->isBlending());
        };
        for (auto* bone : arm.bones)
            inspectBlend(bone);
        if (!pose.referencePrepared)
            for (osg::Node* parent = arm.bones[0]->getNumParents() ? arm.bones[0]->getParent(0) : nullptr;
                parent && parent != mObjectRoot.get();
                parent = parent->getNumParents() ? parent->getParent(0) : nullptr)
                inspectBlend(parent);
        if (!pose.referencePrepared && !pose.releasing
            && (!audit.sourceGroupMatches || !audit.sourceHoldReached || audit.sourceBlendActive))
        {
            // A frozen animation clock at the hold key does not imply that its
            // simulation-time crossfade has finished. Calibrate the reference
            // only from the actual authored guard, never its transient blend.
            pose.published = pose.cacheValid = false;
            pose.weight = 0.f;
            pose.prepared = pose.orientationPrepared = pose.witnessPrepared = pose.choicePrepared = false;
            return;
        }
        const osg::Vec3f shoulder = arm.world[0].getTrans(), elbow = arm.world[1].getTrans();
        const osg::Vec3f wrist = arm.world[2].getTrans();
        const float upper = (elbow - shoulder).length(), fore = (wrist - elbow).length();
        audit.sourceUpperLength = upper;
        audit.sourceForeLength = fore;
        audit.sourceWrist = wrist;
        const auto actorWorld = nodeWorld(mPtr.getRefData().getBaseNode());
        if (!actorWorld)
            return;
        const auto actorInverse = osg::Matrixf::inverse(*actorWorld);
        GuardJointSource rig;
        rig.arm = arm;
        rig.metricToWorld = arm.parent;
        const auto worldToMetric = osg::Matrixf::inverse(rig.metricToWorld);
        rig.metricToActor = rig.metricToWorld * actorInverse;
        for (auto& matrix : rig.arm.world)
            matrix = matrix * worldToMetric;
        rig.arm.parent.makeIdentity();
        rig.parentInverse.makeIdentity();
        rig.upperLength = (rig.arm.world[1].getTrans() - rig.arm.world[0].getTrans()).length();
        rig.foreLength = (rig.arm.world[2].getTrans() - rig.arm.world[1].getTrans()).length();
        if (rig.upperLength < .001f || rig.foreLength < .001f)
            return;
        bool newReference = false;
        if (!pose.referencePrepared)
        {
            osg::Vec3f upperLocal = rig.arm.source[1].getTrans();
            auto foreLocal
                = osg::Matrixf::transform3x3(osg::Vec3f(rig.arm.source[2].getTrans()), rotationOnly(rig.arm.source[1]));
            upperLocal.normalize();
            foreLocal.normalize();
            auto hinge = upperLocal ^ foreLocal;
            const float flex = std::acos(std::clamp(upperLocal * foreLocal, -1.f, 1.f));
            // Calibrate once from a bent authored stance. A later straight arm
            // uses this same reference instead of redefining its hinge plane.
            if (hinge.normalize() < .001f || flex < osg::DegreesToRadians(15.f) || flex > osg::DegreesToRadians(145.f))
                return;
            for (unsigned joint = 0; joint < 3; ++joint)
                pose.referenceRotation[joint] = rotationOnly(rig.arm.source[joint]);
            pose.referenceUpperHinge = hinge;
            pose.referenceForeHinge
                = osg::Matrixf::transform3x3(hinge, osg::Matrixf::inverse(pose.referenceRotation[1]));
            pose.referenceForeZero = pose.referenceRotation[1] * osg::Matrixf::rotate(-flex, hinge);
            newReference = true;
        }
        rig.handRotation = pose.referenceRotation[2];
        rig.upperRotation = pose.referenceRotation[0];
        rig.foreZeroRotation = pose.referenceForeZero;
        rig.upperHingeLocal = pose.referenceUpperHinge;
        rig.foreHingeLocal = pose.referenceForeHinge;
        rig.wristAxis = rig.arm.source[2].getTrans();
        rig.wristAxis.normalize();
        rig.upperAxis = osg::Matrixf::transform3x3(osg::Vec3f(rig.arm.source[1].getTrans()), rig.upperRotation);
        rig.upperAxis.normalize();
        audit.sourceUpperLengthUnscaled = rig.upperLength;
        audit.sourceForeLengthUnscaled = rig.foreLength;
        const auto actualSourceItem = *itemWorld * worldToMetric;
        rig.itemToHand = actualSourceItem * osg::Matrixf::inverse(rig.arm.world[2]);
        rig.itemPoints = pose.clearancePoints;
        const auto boneActor = [&](std::string_view name, const osg::Vec3f& fallback) -> osg::Vec3f {
            const auto matrix = getBoneWorldMatrix(name);
            return matrix ? osg::Vec3f(matrix->getTrans() * actorInverse) : fallback;
        };
        const auto shoulderActor = shoulder * actorInverse;
        const auto spine = boneActor("Bip01 Spine2", shoulderActor + osg::Vec3f(0, 0, -8));
        const auto waist = boneActor("Bip01 Spine1", spine + osg::Vec3f(0, 0, -20));
        const auto pelvis = boneActor("Bip01 Pelvis", waist + osg::Vec3f(0, 0, -12));
        const auto head = boneActor("Bip01 Head", spine + osg::Vec3f(0, 0, 20));
        rig.body = { { waist, spine, 9.f }, { pelvis + osg::Vec3f(0, 0, -4), pelvis + osg::Vec3f(0, 0, 8), 10.f },
            { head, head + osg::Vec3f(0, 0, 7), 7.5f } };
        rig.shield = pose.shield;
        rig.itemCenter = pose.meshCenter;
        rig.shieldTangents = pose.shieldTangents;
        rig.pelvisActor = pelvis;
        rig.torsoActor = spine;
        rig.minimumShieldHeight = std::max(waist.z() + 2.f, pelvis.z() + 18.f);
        auto outwardActor = shoulderActor - spine;
        outwardActor.z() = 0.f;
        if (outwardActor.normalize() < .001f)
            outwardActor = osg::Vec3f(side ? -1.f : 1.f, 0, 0);
        rig.outward = osg::Matrixf::transform3x3(outwardActor, osg::Matrixf::inverse(rig.metricToActor));
        rig.outward.normalize();
        if (newReference)
        {
            std::vector<osg::Vec3f> mcp;
            const std::string prefix = side ? "Bip01 L Finger" : "Bip01 R Finger";
            for (int finger = 1; finger <= 4; ++finger)
                if (const auto found = getNodeMap().find(prefix + std::to_string(finger)); found != getNodeMap().end()
                    && found->second->getNumParents() && found->second->getParent(0) == arm.bones[2])
                    mcp.push_back(found->second->getMatrix().getTrans());
            if (mcp.size() >= 2)
            {
                for (const auto& point : mcp)
                    rig.palmForward += point;
                rig.palmForward.normalize();
                rig.palmLateral = mcp.back() - mcp.front();
            }
            else
            {
                rig.palmForward = osg::Matrixf::transform3x3(
                    rig.arm.world[2].getTrans() - rig.arm.world[1].getTrans(), osg::Matrixf::inverse(rig.arm.world[2]));
                rig.palmForward.normalize();
                rig.palmLateral = osg::Vec3f(0, 1, 0);
            }
            rig.palmLateral -= rig.palmForward * (rig.palmLateral * rig.palmForward);
            if (rig.palmLateral.normalize() < .001f)
            {
                rig.palmLateral = rig.palmForward ^ osg::Vec3f(0, 0, 1);
                rig.palmLateral.normalize();
            }
            pose.referencePalmForward = rig.palmForward;
            pose.referencePalmLateral = rig.palmLateral;
        }
        rig.palmForward = pose.referencePalmForward;
        rig.palmLateral = pose.referencePalmLateral;
        if (!pose.published && !pose.releasing)
        {
            // A native crossfade can pass through an invalid transient wrist
            // frame. Let that animation reach its braced pose before fixing a
            // neutral reference or taking ownership of any joint.
            const auto authored = rig.measure(rig.arm.source, rig.arm.world, 0.f);
            bool pure = true;
            for (const auto& matrix : rig.arm.source)
                pure = pure && orthogonalityError(matrix) < .001f;
            if (!authored.natural() || !pure)
            {
                pose.cacheValid = false;
                return;
            }
            if (newReference)
            {
                pose.referenceHandActor = rig.arm.world[2] * rig.metricToActor;
                pose.referencePrepared = true;
            }
        }
        audit.referenceReady = pose.referencePrepared;
        const auto canonicalHand = pose.referenceHandActor * osg::Matrixf::inverse(rig.metricToActor);
        const auto sourceItem = rig.itemToHand * canonicalHand;
        const auto canonicalItemWorld = sourceItem * rig.metricToWorld;
        const auto metricToActorRotation = rotationOnly(rig.metricToActor);
        const auto actorToMetricRotation = osg::Matrixf::inverse(metricToActorRotation);
        const auto turnToActor
            = [&](const osg::Matrixf& rotation) { return actorToMetricRotation * rotation * metricToActorRotation; };
        const auto turnToMetric = [&](const osg::Quat& rotation) {
            return metricToActorRotation * osg::Matrixf::rotate(rotation) * actorToMetricRotation;
        };
        if (pose.enabled)
        {
            const auto localPoint = pose.point * actorInverse;
            auto localDirection = osg::Matrixf::transform3x3(pose.direction, actorInverse);
            localDirection.normalize();
            const auto localStroke = osg::Matrixf::transform3x3(pose.stroke, actorInverse);
            if (!pose.prepared)
            {
                pose.localPoint = localPoint;
                pose.localDirection = localDirection;
                pose.localStroke = localStroke;
                pose.prepared = true;
            }
            const float response = 1.f - std::exp(-duration / .055f);
            pose.localPoint += (localPoint - pose.localPoint) * response;
            pose.localDirection += (localDirection - pose.localDirection) * response;
            pose.localDirection.normalize();
            pose.localStroke += (localStroke - pose.localStroke) * response;
        }
        if (pose.enabled)
            pose.weight = std::clamp(pose.weight + duration / .18f, 0.f, 1.f);
        float weight = pose.weight * pose.weight * (3.f - 2.f * pose.weight);
        if (pose.releasing)
        {
            pose.releaseElapsed += elapsed;
            const float amount = std::clamp(1.f - pose.releaseElapsed / .20f, 0.f, 1.f);
            weight = pose.releaseStartWeight * amount * amount * (3.f - 2.f * amount);
            audit.releasing = true;
            audit.releaseTimeLeft = std::max(.20f - pose.releaseElapsed, 0.f);
        }
        audit.weight = weight;
        if (!pose.prepared || (weight <= .0001f && !pose.published))
            return;
        auto direction = osg::Matrixf::transform3x3(pose.localDirection, *actorWorld);
        direction.normalize();
        const osg::Vec3f requested = pose.localPoint * *actorWorld;
        const auto rawAxis = [&](const osg::Matrixf& matrix) {
            osg::Vec3f result = osg::Matrixf::transform3x3(pose.axis, matrix);
            if (pose.shield)
                result = osg::Matrixf::transform3x3(pose.shieldTangents[0], matrix)
                    ^ osg::Matrixf::transform3x3(pose.shieldTangents[1], matrix);
            result.normalize();
            return result;
        };
        const float normalSign = pose.shield
                && rawAxis(canonicalItemWorld) * (pose.meshCenter * canonicalItemWorld - spine * *actorWorld) < 0.f
            ? -1.f
            : 1.f;
        rig.shieldNormalSign = normalSign;
        rig.forwardActor = osg::Matrixf::transform3x3(-direction, actorInverse);
        rig.forwardActor.z() = 0.f;
        rig.forwardActor.normalize();
        const auto worldAxis = [&](const osg::Matrixf& matrix) { return rawAxis(matrix) * normalSign; };
        const auto normalToMetric = [&](const osg::Vec3f& normal) {
            osg::Vec3f result;
            for (unsigned i = 0; i < 3; ++i)
                result[i] = normal.x() * rig.metricToWorld(i, 0) + normal.y() * rig.metricToWorld(i, 1)
                    + normal.z() * rig.metricToWorld(i, 2);
            result.normalize();
            return result;
        };
        const auto sourceAxis = worldAxis(sourceItem);
        osg::Vec3f desiredAxis;
        if (pose.shield)
            desiredAxis = -direction;
        else
        {
            const auto stroke = osg::Matrixf::transform3x3(pose.localStroke, *actorWorld);
            desiredAxis = direction ^ stroke;
            if (desiredAxis.normalize() < .01f)
            {
                const auto authoredAxis = worldAxis(canonicalItemWorld);
                desiredAxis = authoredAxis - direction * (authoredAxis * direction);
                if (desiredAxis.normalize() < .01f)
                    desiredAxis = direction ^ osg::Vec3f(0, 0, 1);
                desiredAxis.normalize();
            }
        }
        std::vector<osg::Vec3f> axisCandidates;
        const auto addAxis = [&](const osg::Vec3f& axis) {
            if (std::none_of(axisCandidates.begin(), axisCandidates.end(),
                    [&](const osg::Vec3f& existing) { return existing * axis > .9999f; }))
                axisCandidates.push_back(axis);
        };
        if (pose.shield)
        {
            addAxis(desiredAxis);
            auto upright = desiredAxis;
            upright.z() = 0.f;
            if (upright.normalize() > .01f)
                for (float amount : { .5f, 1.f })
                {
                    auto candidate = desiredAxis * (1.f - amount) + upright * amount;
                    candidate.normalize();
                    if (candidate * desiredAxis >= .7f)
                        addAxis(candidate);
                }
        }
        else
        {
            // The authored hanging/diagonal blade is a real defensive pose.
            // A damage-class label is a style preference, not the physical
            // velocity direction of the incoming weapon.
            const auto authoredAxis = worldAxis(canonicalItemWorld);
            addAxis(authoredAxis);
            for (float angle : { -15.f, 15.f })
                addAxis(osg::Quat(osg::DegreesToRadians(angle), direction) * authoredAxis);
            auto projected = authoredAxis - direction * (authoredAxis * direction);
            if (projected.normalize() > .001f)
                addAxis(projected);
            addAxis(desiredAxis);
            for (float angle : { -15.f, 15.f, -30.f, 30.f })
                addAxis(osg::Quat(osg::DegreesToRadians(angle), direction) * desiredAxis);
        }

        if (mNpcGuardRigAuditRequested)
        {
            // Opt-in, one-frame copy of the actual solver inputs. Ordinary
            // guards never allocate these maps; the same schema as the paired
            // solver lets the standalone native driver replay the right arm.
            NpcGuardRigAudit capture;
            capture.frame = audit.frame;
            capture.matrices["actorWorld"] = *actorWorld;
            capture.matrices["itemWorld"] = *itemWorld;
            capture.matrices["canonicalHand"] = canonicalHand;
            capture.matrices["sourceItem"] = sourceItem;
            capture.matrices["referenceItemActor"] = sourceItem * rig.metricToActor;
            capture.matrices["currentItemActor"] = *itemWorld * actorInverse;
            capture.matrices["cachedTurn"] = osg::Matrixf::rotate(pose.cachedTurn);
            capture.vectors["target"] = requested;
            capture.vectors["incoming"] = direction;
            capture.vectors["stroke"] = pose.localStroke;
            capture.vectors["itemAxis"] = pose.axis;
            capture.vectors["desiredAxis"] = desiredAxis;
            capture.vectors["cachedTarget"] = pose.cachedPoint * *actorWorld;
            capture.vectors["cachedDirection"] = pose.cachedDirection;
            capture.scalars["enabled"] = pose.enabled;
            capture.scalars["releasing"] = pose.releasing;
            capture.scalars["published"] = pose.published;
            capture.scalars["referenceReady"] = pose.referencePrepared;
            capture.scalars["sourceGroupMatches"] = audit.sourceGroupMatches;
            capture.scalars["sourceHoldReached"] = audit.sourceHoldReached;
            capture.scalars["sourceBlendActive"] = audit.sourceBlendActive;
            capture.scalars["duration"] = duration;
            capture.scalars["elapsed"] = elapsed;
            capture.scalars["weight"] = pose.weight;
            capture.scalars["releaseElapsed"] = pose.releaseElapsed;
            capture.scalars["cache"] = pose.cacheValid;
            capture.scalars["cachedNatural"] = pose.cachedExact;
            capture.scalars["cachedWitness"] = pose.selectedWitness;
            capture.scalars["cachedOffset"] = pose.cachedOffset;
            capture.scalars["witnessCount"] = pose.witnesses.size();
            capture.scalars["axisCount"] = axisCandidates.size();
            capture.scalars["minimumMaterialCoordinate"] = pose.minimumMaterialCoordinate;
            capture.scalars["maximumMaterialCoordinate"] = pose.maximumMaterialCoordinate;
            for (unsigned index = 0; index < pose.witnesses.size(); ++index)
                capture.vectors["witness" + std::to_string(index)] = pose.witnesses[index];
            for (unsigned index = 0; index < axisCandidates.size(); ++index)
                capture.vectors["axisCandidate" + std::to_string(index)] = axisCandidates[index];
            const std::string prefix = pose.shield ? "left." : "right.";
            for (unsigned joint = 0; joint < 3; ++joint)
            {
                const auto index = std::to_string(joint);
                capture.matrices[prefix + "source" + index] = rig.arm.source[joint];
                capture.matrices[prefix + "world" + index] = rig.arm.world[joint];
                capture.matrices[prefix + "sourceWorld" + index] = arm.world[joint];
                capture.matrices[prefix + "referenceRotation" + index] = pose.referenceRotation[joint];
                auto published = pose.published ? osg::Matrixf::rotate(pose.publishedRotation[joint])
                                                : rotationOnly(rig.arm.source[joint]);
                published.setTrans(rig.arm.source[joint].getTrans());
                capture.matrices[prefix + "publishedLocal" + index] = published;
            }
            capture.matrices[prefix + "parent"] = rig.arm.parent;
            capture.matrices[prefix + "parentInverse"] = rig.parentInverse;
            capture.matrices[prefix + "handRotation"] = rig.handRotation;
            capture.matrices[prefix + "upperRotation"] = rig.upperRotation;
            capture.matrices[prefix + "foreZeroRotation"] = rig.foreZeroRotation;
            capture.matrices[prefix + "metricToWorld"] = rig.metricToWorld;
            capture.matrices[prefix + "metricToActor"] = rig.metricToActor;
            capture.matrices[prefix + "itemToHand"] = rig.itemToHand;
            capture.vectors[prefix + "wristAxis"] = rig.wristAxis;
            capture.vectors[prefix + "upperAxis"] = rig.upperAxis;
            capture.vectors[prefix + "upperHingeLocal"] = rig.upperHingeLocal;
            capture.vectors[prefix + "foreHingeLocal"] = rig.foreHingeLocal;
            capture.vectors[prefix + "palmForward"] = rig.palmForward;
            capture.vectors[prefix + "palmLateral"] = rig.palmLateral;
            capture.vectors[prefix + "outward"] = rig.outward;
            capture.vectors[prefix + "itemCenter"] = rig.itemCenter;
            capture.vectors[prefix + "torsoActor"] = rig.torsoActor;
            capture.vectors[prefix + "pelvisActor"] = rig.pelvisActor;
            capture.vectors[prefix + "forwardActor"] = rig.forwardActor;
            for (unsigned tangent = 0; tangent < 2; ++tangent)
                capture.vectors[prefix + "shieldTangent" + std::to_string(tangent)] = rig.shieldTangents[tangent];
            capture.scalars[prefix + "shieldNormalSign"] = rig.shieldNormalSign;
            capture.scalars[prefix + "minimumShieldHeight"] = rig.minimumShieldHeight;
            capture.scalars[prefix + "shield"] = rig.shield;
            capture.scalars[prefix + "upperLength"] = rig.upperLength;
            capture.scalars[prefix + "foreLength"] = rig.foreLength;
            capture.scalars[prefix + "cachedSwivel"] = pose.cachedSwivel;
            capture.scalars[prefix + "bodyCount"] = rig.body.size();
            capture.scalars[prefix + "itemPointCount"] = rig.itemPoints.size();
            for (unsigned capsule = 0; capsule < rig.body.size(); ++capsule)
            {
                const auto name = prefix + "body" + std::to_string(capsule);
                capture.vectors[name + ".from"] = rig.body[capsule].from;
                capture.vectors[name + ".to"] = rig.body[capsule].to;
                capture.scalars[name + ".radius"] = rig.body[capsule].radius;
            }
            for (unsigned point = 0; point < rig.itemPoints.size(); ++point)
                capture.vectors[prefix + "itemPoint" + std::to_string(point)] = rig.itemPoints[point];
            mNpcGuardRigAudit = std::move(capture);
            mNpcGuardRigAuditRequested = false;
        }

        const auto visualCost = [&](const GuardJointPose& solution) {
            if (pose.shield)
                return 0.f;
            const osg::Vec3f elbowActor = solution.world[1].getTrans() * rig.metricToActor;
            const osg::Vec3f wristActor = solution.world[2].getTrans() * rig.metricToActor;
            const bool high = (requested * actorInverse).z() >= shoulderActor.z() - 4.f;
            const auto squared = [](float value) { return value * value; };
            const auto excess = [](float value, float limit) { return std::max(std::abs(value) - limit, 0.f); };
            // This is a soft pose preference, not another joint limit. Support a
            // high guard from below; a low guard may put its wrist below its
            // elbow, but must not raise the elbow like a wing beside the head.
            float cost = squared(std::max(elbowActor.z() - shoulderActor.z() + 1.f, 0.f)) * .20f;
            if (high)
                cost += squared(std::max(elbowActor.z() - wristActor.z() + 3.f, 0.f)) * .20f;
            else
                cost += squared(std::max((elbowActor - shoulderActor) * outwardActor - rig.upperLength * .65f, 0.f))
                    * .08f;
            cost += squared(excess(solution.metrics.palmFlex, 20.f)) * .012f
                + squared(excess(solution.metrics.palmSide, 15.f)) * .01f
                + squared(excess(solution.metrics.wristSwing, 20.f)) * .01f
                + squared(excess(solution.metrics.foreTwist, 25.f)) * .005f;
            // A lateral parry can stack forearm and wrist twist while each
            // individual joint still passes. Prefer a straighter palm without
            // exchanging that twist for a deeply folded wrist in another pose.
            cost += squared(excess(solution.metrics.palmSide, 10.f)) * .03f
                + squared(excess(solution.metrics.palmFlex, 30.f)) * .04f
                + squared(excess(solution.metrics.wristTwist, 20.f)) * .01f
                + squared(excess(solution.metrics.foreTwist + solution.metrics.wristTwist, 45.f)) * .01f;
            return cost;
        };

        const auto solveTarget = [&](const osg::Matrixf& hand, float swivel, unsigned* evaluations = nullptr) {
            if (evaluations)
                ++*evaluations;
            auto solution = solveGuardJoint(rig, hand, swivel);
            if (!pose.shield && std::abs(solution.metrics.foreTwist) > 39.9f)
            {
                // Keep the target inside the unchanged hard 40-degree limit.
                // A quaternion round-trip at exactly 40 can measure 40.000004,
                // invalidating a published pose and repeatedly restarting entry.
                // Re-solve the same hand/grip target, rather than relaxing the
                // anatomical test or clipping a final joint after publication.
                if (evaluations)
                    ++*evaluations;
                solution = solveGuardJoint(rig, hand, swivel, std::copysign(39.9f, solution.metrics.foreTwist));
            }
            return solution;
        };

        osg::Vec3f witness = pose.witnessPrepared ? pose.filteredWitness : pose.witnesses.front();
        osg::Vec3f chosenAxis = desiredAxis;
        osg::Vec3f intercept = requested - direction * pose.filteredOffset;
        osg::Vec3f handTargetWorld;
        GuardJointPose solved;
        if (pose.releasing)
        {
            const float amount = std::clamp(1.f - pose.releaseElapsed / .20f, 0.f, 1.f);
            const float envelope = amount * amount * (3.f - 2.f * amount);
            // Ownership has a fixed deadline. Apply a decaying correction to
            // each NEW native frame, never a guard-limited chase of that frame.
            for (unsigned joint = 0; joint < 3; ++joint)
            {
                osg::Quat delta;
                delta.slerp(envelope, osg::Quat(), pose.releaseDelta[joint]);
                solved.local[joint] = rotationOnly(rig.arm.source[joint]) * osg::Matrixf::rotate(delta);
                solved.local[joint].setTrans(rig.arm.source[joint].getTrans());
                solved.world[joint] = solved.local[joint] * (joint ? solved.world[joint - 1] : rig.arm.parent);
            }
            handTargetWorld = solved.world[2].getTrans() * rig.metricToWorld;
            solved.metrics = rig.measure(solved.local, solved.world, 0.f);
            if (pose.releaseElapsed >= .20f)
            {
                pose.releasing = pose.published = false;
                pose.weight = 0.f;
                audit.releasing = false;
            }
        }
        else
        {
            float best = std::numeric_limits<float>::max();
            osg::Quat chosenTurn;
            unsigned chosenWitness = 0;
            float chosenOffset = 8.f, chosenSwivel = 0.f;
            bool fullGoalWorking = false, safeChoice = false, cached = false;
            bool bestExactCandidate = false;
            std::optional<GuardJointPose> rawGoal;
            const float baseOffset = std::clamp((upper + fore) * .14f, 6.f, 12.f);
            osg::Vec3f chosenGoal = requested - direction * baseOffset;
            if (pose.cacheValid && pose.selectedWitness < pose.witnesses.size())
            {
                // Positive reuse is decided by today's FK and item geometry,
                // not equality of yesterday's moving animation matrices.
                const auto goalWorld
                    = pose.cachedSafe ? pose.cachedGoal * *actorWorld : requested - direction * pose.cachedOffset;
                witness = pose.witnesses[pose.selectedWitness];
                const auto correction = osg::Matrixf::translate(-(witness * sourceItem)) * turnToMetric(pose.cachedTurn)
                    * osg::Matrixf::translate(goalWorld * worldToMetric);
                const auto handGoal = canonicalHand * correction;
                const auto solution = solveTarget(handGoal, pose.cachedSwivel, &audit.searchCandidates);
                const auto goalItem = rig.itemToHand * solution.world[2] * rig.metricToWorld;
                const bool facing = !pose.shield || worldAxis(goalItem) * -direction >= .7f;
                const bool exact
                    = solution.metrics.working() && (witness * goalItem - goalWorld).length() < 1.f && facing;
                bool similarRig = rotationDistanceDegrees(rotationOnly(rig.metricToActor).getRotate(),
                                      rotationOnly(pose.cachedMetricToActor).getRotate())
                        < 10.f
                    && osg::Vec3f(rig.metricToActor.getTrans() - pose.cachedMetricToActor.getTrans()).length2() < 4.f;
                for (unsigned joint = 0; joint < 3; ++joint)
                    similarRig = similarRig
                        && rotationDistanceDegrees(
                               rig.arm.source[joint].getRotate(), pose.cachedSource[joint].getRotate())
                            < 10.f;
                const bool sameProblem = similarRig && (pose.localPoint - pose.cachedPoint).length2() < 4.f
                    && pose.localDirection * pose.cachedDirection > .995f
                    && (pose.localStroke - pose.cachedStroke).length2() < .01f;
                const bool sameFailedProblem = !pose.cachedExact && sameProblem;
                // An exact old orientation may still create a raised low elbow
                // after an aim change. Compare alternatives once for that new
                // problem; reuse the resulting optimum while it stays steady.
                const bool reuseExact = exact && (pose.shield || visualCost(solution) < 1.f || sameProblem);
                if (reuseExact || sameFailedProblem || (!pose.shield && exact))
                {
                    // Preserve the FK-validated exact guard as the incumbent
                    // while a changed target triggers a bounded style search.
                    // Failing to find a prettier goal must not lose that block.
                    cached = reuseExact || sameFailedProblem;
                    fullGoalWorking = solution.metrics.working();
                    safeChoice = pose.cachedSafe;
                    chosenGoal = goalWorld;
                    chosenAxis = worldAxis(goalItem);
                    chosenTurn = pose.cachedTurn;
                    chosenOffset = pose.cachedOffset;
                    chosenSwivel = pose.cachedSwivel;
                    chosenWitness = pose.selectedWitness;
                    best = solution.metrics.score();
                    if (!pose.shield)
                        best += visualCost(solution) + (handGoal.getTrans() - canonicalHand.getTrans()).length() * .02f
                            + (witness - pose.witnesses.front()).length() * .06f
                            + std::max(chosenOffset - baseOffset, 0.f) * .025f + std::abs(chosenSwivel) * .003f;
                    bestExactCandidate = exact;
                    rawGoal = solution;
                    audit.cacheUsed = cached;
                    audit.cacheNegative = cached && !exact;
                    if (exact)
                        pose.cachedExact = true;
                }
            }
            audit.fullSearch = !cached && weight > .0001f;
            if (audit.fullSearch)
            {
                // Cover the preferred authored orientations coarsely first.
                // Refine only the best branch, with a hard cold-search budget.
                constexpr unsigned maxCandidates = 768;
                const unsigned searchLimit = pose.shield ? maxCandidates - 32 : maxCandidates;
                bool suitableFound = false;
                const auto tryCandidate = [&](const osg::Vec3f& axisWorld, int sign, float roll, unsigned i,
                                              float extraOffset, float swivel) {
                    // A one-handed target may need the interior-pronation
                    // re-solve above. Count both native evaluations, and reserve
                    // their slots before starting, within the same 768 budget.
                    if (audit.searchCandidates + (pose.shield ? 1u : 2u) > searchLimit)
                        return true;
                    if ((pose.shield && sign < 0) || (pose.shield && std::abs(roll) > 30.f))
                        return false;
                    const auto candidateWorldAxis = axisWorld * float(sign);
                    auto candidateAxis = pose.shield ? normalToMetric(candidateWorldAxis)
                                                     : osg::Matrixf::transform3x3(candidateWorldAxis, worldToMetric);
                    candidateAxis.normalize();
                    osg::Quat align;
                    align.makeRotate(sourceAxis, candidateAxis);
                    const auto rotation = osg::Matrixf::rotate(align)
                        * osg::Matrixf::rotate(osg::DegreesToRadians(roll), candidateAxis);
                    const auto& candidate = pose.witnesses[i];
                    const auto goalWorld = requested - direction * (baseOffset + extraOffset);
                    const auto correction = osg::Matrixf::translate(-(candidate * sourceItem)) * rotation
                        * osg::Matrixf::translate(goalWorld * worldToMetric);
                    const auto handGoal = canonicalHand * correction;
                    const float preference = (handGoal.getTrans() - canonicalHand.getTrans()).length() * .02f
                        + (candidate - pose.witnesses.front()).length() * .06f + extraOffset * .025f
                        + std::abs(roll) * .004f + (1.f - sourceAxis * candidateAxis) * .5f;
                    const float outsideReach
                        = std::max(osg::Vec3f(handGoal.getTrans() - rig.arm.world[0].getTrans()).length()
                                - rig.upperLength - rig.foreLength,
                            0.f);
                    if ((pose.shield || bestExactCandidate) && best < 1e6f && outsideReach * 100.f + preference > best)
                        return false;
                    const auto solution = solveTarget(handGoal, swivel, &audit.searchCandidates);
                    const float style = visualCost(solution);
                    float score = solution.metrics.score() + preference + std::abs(swivel) * .003f + style;
                    if (pose.choicePrepared)
                    {
                        if (pose.selectedAxis * candidateWorldAxis < 0.f)
                            score += .75f;
                        if (i != pose.selectedWitness)
                            score += .15f;
                    }
                    const auto item = rig.itemToHand * solution.world[2] * rig.metricToWorld;
                    const bool exactCandidate
                        = solution.metrics.working() && (candidate * item - goalWorld).length() < 1.f;
                    // Style ranks feasible guards; it must never let a pleasant
                    // but non-contacting or invalid arm displace a real block.
                    const bool better = pose.shield ? score < best
                                                    : (exactCandidate && !bestExactCandidate)
                            || (exactCandidate == bestExactCandidate && score < best);
                    if (better)
                    {
                        best = score;
                        bestExactCandidate = exactCandidate;
                        witness = candidate;
                        chosenTurn = turnToActor(rotation).getRotate();
                        chosenAxis = candidateWorldAxis;
                        chosenWitness = i;
                        chosenOffset = baseOffset + extraOffset;
                        chosenSwivel = swivel;
                        chosenGoal = goalWorld;
                        fullGoalWorking = solution.metrics.working();
                        rawGoal = solution;
                    }
                    if ((pose.shield || better) && solution.metrics.working()
                        && (candidate * item - goalWorld).length() < 1.f && score < 20.f
                        && (pose.shield || style < 1.f))
                    {
                        suitableFound = true;
                        return true;
                    }
                    return false;
                };
                const auto coarse = [&] {
                    if (!pose.shield)
                    {
                        // Breadth before refinement: every blade direction and
                        // contact gets each already permitted stand-off with a
                        // neutral elbow pole. A short blade's low guard may need
                        // the far stand-off; refining only the first near goal
                        // previously exhausted the budget before reaching it.
                        for (unsigned i = 0; i < pose.witnesses.size(); ++i)
                            for (const auto& axis : axisCandidates)
                                for (int sign : { 1, -1 })
                                    for (float offset : { 0.f, 8.f, 16.f })
                                        if (tryCandidate(axis, sign, 0.f, i, offset, 0.f))
                                            return;
                        // Give near-hilt contacts real roll alternatives before
                        // refining only one orientation. Total search remains
                        // bounded by the same maxCandidates counter.
                        for (unsigned i = 0; i < std::min<unsigned>(2, pose.witnesses.size()); ++i)
                            for (const auto& axis : axisCandidates)
                                for (int sign : { 1, -1 })
                                    for (float roll : { -60.f, 60.f })
                                        for (float swivel : { 0.f, -60.f, 60.f })
                                            if (tryCandidate(axis, sign, roll, i, 0.f, swivel))
                                                return;
                        return;
                    }
                    for (const auto& axis : axisCandidates)
                        for (int sign : { 1, -1 })
                            for (unsigned i = 0; i < pose.witnesses.size(); ++i)
                                for (float offset : { 0.f, 8.f })
                                    for (float swivel : { 0.f, -30.f, 30.f })
                                        if (tryCandidate(axis, sign, 0.f, i, offset, swivel))
                                            return;
                };
                coarse();
                if (!suitableFound && rawGoal && audit.searchCandidates < searchLimit)
                {
                    const auto refineAxis = chosenAxis;
                    const unsigned anchorWitness = chosenWitness;
                    const float anchorOffset = chosenOffset - baseOffset;
                    std::vector<unsigned> order{ chosenWitness };
                    for (unsigned i = 0; i < pose.witnesses.size(); ++i)
                        if (i != chosenWitness)
                            order.push_back(i);
                    std::vector<float> offsets{ anchorOffset };
                    for (float offset : { 0.f, 8.f, 16.f })
                        if (std::none_of(offsets.begin(), offsets.end(),
                                [&](float existing) { return std::abs(existing - offset) < .001f; }))
                            offsets.push_back(offset);
                    const auto refine = [&] {
                        // Each roll, including zero, gets the selected contact
                        // and the full elbow circle before material neighbours.
                        for (float swivel :
                            { 0.f, -60.f, 60.f, -90.f, 90.f, -120.f, 120.f, 180.f, -20.f, 20.f, -40.f, 40.f })
                            for (float roll : { 0.f, -30.f, 30.f, -60.f, 60.f, -90.f, 90.f, 180.f })
                                if (tryCandidate(refineAxis, 1, roll, anchorWitness, anchorOffset, swivel))
                                    return;
                        for (float swivel :
                            { 0.f, -60.f, 60.f, -90.f, 90.f, -120.f, 120.f, 180.f, -20.f, 20.f, -40.f, 40.f })
                            for (unsigned i : order)
                                for (float offset : offsets)
                                {
                                    if (i == anchorWitness && std::abs(offset - anchorOffset) < .001f)
                                        continue;
                                    for (float roll : { 0.f, -30.f, 30.f, -60.f, 60.f, -90.f, 90.f, 180.f })
                                        if (tryCandidate(refineAxis, 1, roll, i, offset, swivel))
                                            return;
                                }
                    };
                    refine();
                }
            }
            if (!cached && !fullGoalWorking && pose.shield && weight > .0001f)
            {
                // Select a safe FULL guard goal before either entry/exit weighting
                // or final joint filtering. Working shield height is a requirement
                // of the raised stance, not of an idle shield at weight zero.
                auto centerActor = spine;
                centerActor.z() = std::max(rig.minimumShieldHeight + 4.f, spine.z() - 6.f);
                auto normalWorld = osg::Matrixf::transform3x3(rig.forwardActor, *actorWorld);
                normalWorld.normalize();
                const auto normal = normalToMetric(normalWorld);
                osg::Quat align;
                align.makeRotate(sourceAxis, normal);
                float safeBest = std::numeric_limits<float>::max();
                for (float advance : { 24.f, 32.f })
                    for (float roll : { 0.f, -30.f, 30.f })
                    {
                        const auto goalWorld = (centerActor + rig.forwardActor * advance) * *actorWorld;
                        const auto rotation
                            = osg::Matrixf::rotate(align) * osg::Matrixf::rotate(osg::DegreesToRadians(roll), normal);
                        const auto goal = canonicalHand
                            * osg::Matrixf::translate(-(pose.witnesses.front() * sourceItem)) * rotation
                            * osg::Matrixf::translate(goalWorld * worldToMetric);
                        for (float swivel : { 0.f, -20.f, 20.f, -40.f, 40.f })
                        {
                            ++audit.searchCandidates;
                            const auto safe = solveGuardJoint(rig, goal, swivel);
                            if (safe.metrics.working() && safe.metrics.score() < safeBest)
                            {
                                safeBest = safe.metrics.score();
                                fullGoalWorking = safeChoice = true;
                                witness = pose.witnesses.front();
                                chosenWitness = 0;
                                chosenTurn = turnToActor(rotation).getRotate();
                                chosenAxis = normalWorld;
                                chosenGoal = goalWorld;
                                chosenOffset = baseOffset;
                                chosenSwivel = swivel;
                                rawGoal = safe;
                            }
                        }
                    }
            }
            if (!cached && rawGoal)
            {
                pose.cacheValid = true;
                const auto goalItem = rig.itemToHand * rawGoal->world[2] * rig.metricToWorld;
                pose.cachedExact = rawGoal->metrics.working() && (witness * goalItem - chosenGoal).length() < 1.f
                    && (!pose.shield || worldAxis(goalItem) * -direction >= .7f);
                pose.cachedSafe = safeChoice;
                pose.cachedSource = rig.arm.source;
                pose.cachedMetricToActor = rig.metricToActor;
                pose.cachedGoal = chosenGoal * actorInverse;
                pose.cachedPoint = pose.localPoint;
                pose.cachedDirection = pose.localDirection;
                pose.cachedStroke = pose.localStroke;
                pose.cachedTurn = chosenTurn;
                pose.cachedOffset = chosenOffset;
                pose.cachedSwivel = chosenSwivel;
            }
            else if (!cached)
                pose.cacheValid = false;
            if (rawGoal)
            {
                const auto& metrics = rawGoal->metrics;
                audit.goalNatural = metrics.natural();
                audit.goalWorking = metrics.working();
                audit.goalFromSafe = safeChoice;
                audit.goalWrist = rawGoal->world[2].getTrans() * rig.metricToWorld;
                audit.goalContact = witness * rig.itemToHand * rawGoal->world[2] * rig.metricToWorld;
                audit.goalContactError = (audit.goalContact - (requested - direction * chosenOffset)).length();
                audit.goalTargetError = (audit.goalContact - chosenGoal).length();
                audit.goalReachClamp = metrics.reach;
                audit.goalWristSwingDegrees = metrics.wristSwing;
                audit.goalWristTwistDegrees = metrics.wristTwist;
                audit.goalPalmFlexDegrees = metrics.palmFlex;
                audit.goalPalmSideDegrees = metrics.palmSide;
                audit.goalForearmTwistDegrees = metrics.foreTwist;
                audit.goalUpperSwingDegrees = metrics.upperSwing;
                audit.goalUpperTwistDegrees = metrics.upperTwist;
                audit.goalSignedElbowFlexDegrees = metrics.signedElbowFlex;
                audit.goalElbowHingeErrorDegrees = metrics.elbowHingeError;
                audit.goalBodyClearance = metrics.clearance;
                audit.goalInwardError = metrics.inward;
                audit.goalStanceMargin = pose.shield ? metrics.stanceMargin : 0.f;
            }
            pose.choicePrepared = true;
            pose.selectedWitness = chosenWitness;
            pose.selectedAxis = chosenAxis;
            if (!pose.orientationPrepared)
            {
                pose.filteredTurn = chosenTurn;
                pose.filteredOffset = chosenOffset;
                pose.filteredSwivel = chosenSwivel;
                pose.filteredGoal = chosenGoal * actorInverse;
                pose.orientationPrepared = true;
            }
            else
            {
                const float response = 1.f - std::exp(-duration / .065f);
                osg::Quat filtered;
                filtered.slerp(response, pose.filteredTurn, chosenTurn);
                pose.filteredTurn = filtered;
                pose.filteredOffset += (chosenOffset - pose.filteredOffset) * response;
                pose.filteredSwivel += (chosenSwivel - pose.filteredSwivel) * response;
                pose.filteredGoal += (chosenGoal * actorInverse - pose.filteredGoal) * response;
            }
            if (!pose.witnessPrepared)
            {
                pose.filteredWitness = witness;
                pose.witnessPrepared = true;
            }
            else
                pose.filteredWitness += (witness - pose.filteredWitness) * (1.f - std::exp(-duration / .055f));
            if (const auto material = materialWitness(pose.triangles, pose.filteredWitness, pose.axis,
                    pose.minimumMaterialCoordinate, pose.maximumMaterialCoordinate))
                witness = *material;
            intercept = requested - direction * pose.filteredOffset;
            const auto sourceContact = witness * sourceItem;
            const auto goalMetric = pose.filteredGoal * *actorWorld * worldToMetric;
            const auto fullTargetHand = canonicalHand * osg::Matrixf::translate(-sourceContact)
                * turnToMetric(pose.filteredTurn) * osg::Matrixf::translate(goalMetric);
            const auto targetHand = [&](float amount) {
                osg::Quat turn;
                turn.slerp(weight * amount, rig.arm.world[2].getRotate(), fullTargetHand.getRotate());
                auto hand = osg::Matrixf::rotate(turn);
                const osg::Vec3f current = rig.arm.world[2].getTrans();
                const osg::Vec3f desired = fullTargetHand.getTrans();
                hand.setTrans(current + (desired - current) * (weight * amount));
                return hand;
            };
            handTargetWorld = targetHand(1.f).getTrans() * rig.metricToWorld;
            solved = solveTarget(targetHand(1.f), pose.filteredSwivel);
            if (!solved.metrics.natural())
            {
                audit.jointLimited = true;
                for (float amount : { .875f, .75f, .625f, .5f, .375f, .25f, .125f, 0.f })
                {
                    auto candidate = solveTarget(targetHand(amount), pose.filteredSwivel * amount);
                    if (candidate.metrics.natural() || amount == 0.f)
                    {
                        solved = candidate;
                        break;
                    }
                }
            }
            if (!solved.metrics.natural())
            {
                // The authored stance is preferable to manufacturing a broken
                // joint when all constrained goals are outside the feasible zone.
                audit.jointLimited = true;
                solved.local = rig.arm.source;
                solved.world = rig.arm.world;
                solved.metrics = rig.measure(solved.local, solved.world, 0.f);
            }
            // Limit the actual published joints, rather than assuming independent
            // item-turn/material filters imply a continuous arm. Elbow flex and
            // pronation remain anatomical parameters during interpolation, so the
            // transition cannot turn the hinge into a free ball joint.
            GuardJointPose previous;
            for (unsigned joint = 0; joint < 3; ++joint)
            {
                previous.local[joint] = pose.published ? osg::Matrixf::rotate(pose.publishedRotation[joint])
                                                       : rotationOnly(rig.arm.source[joint]);
                previous.local[joint].setTrans(rig.arm.source[joint].getTrans());
                previous.world[joint] = previous.local[joint] * (joint ? previous.world[joint - 1] : rig.arm.parent);
            }
            previous.metrics = rig.measure(previous.local, previous.world, 0.f);
            const float stepDuration = std::clamp(duration, 0.f, .05f);
            const std::array<float, 3> speed{ 240.f, 360.f, 360.f };
            float fraction = 1.f;
            for (unsigned joint = 0; joint < 3; ++joint)
            {
                const float distance
                    = rotationDistanceDegrees(previous.local[joint].getRotate(), solved.local[joint].getRotate());
                if (distance > .001f)
                    fraction = std::min(fraction, speed[joint] * stepDuration / distance);
            }
            const float handDistance
                = rotationDistanceDegrees(previous.world[2].getRotate(), solved.world[2].getRotate());
            if (handDistance > .001f)
                fraction = std::min(fraction, 360.f * stepDuration / handDistance);
            const auto desired = solved;
            bool accepted = false;
            for (unsigned attempt = 0; attempt < 12; ++attempt)
            {
                GuardJointPose candidate;
                osg::Quat upperRotation, handRotation;
                upperRotation.slerp(fraction, previous.local[0].getRotate(), desired.local[0].getRotate());
                handRotation.slerp(fraction, previous.local[2].getRotate(), desired.local[2].getRotate());
                candidate.local[0] = osg::Matrixf::rotate(upperRotation);
                candidate.local[0].setTrans(rig.arm.source[0].getTrans());
                candidate.world[0] = candidate.local[0] * rig.arm.parent;
                const float flex = osg::DegreesToRadians(previous.metrics.signedElbowFlex
                    + (desired.metrics.signedElbowFlex - previous.metrics.signedElbowFlex) * fraction);
                const float pronation = osg::DegreesToRadians(
                    previous.metrics.foreTwist + (desired.metrics.foreTwist - previous.metrics.foreTwist) * fraction);
                candidate.local[1] = rig.foreLocal(flex, pronation);
                candidate.world[1] = candidate.local[1] * candidate.world[0];
                candidate.local[2] = osg::Matrixf::rotate(handRotation);
                candidate.local[2].setTrans(rig.arm.source[2].getTrans());
                candidate.world[2] = candidate.local[2] * candidate.world[1];
                candidate.metrics = rig.measure(candidate.local, candidate.world, 0.f);
                bool withinSpeed
                    = rotationDistanceDegrees(previous.world[2].getRotate(), candidate.world[2].getRotate())
                    <= 360.f * stepDuration + .01f;
                for (unsigned joint = 0; joint < 3; ++joint)
                    withinSpeed = withinSpeed
                        && rotationDistanceDegrees(
                               previous.local[joint].getRotate(), candidate.local[joint].getRotate())
                            <= speed[joint] * stepDuration + .01f;
                const bool recovering = !previous.metrics.natural() && candidate.metrics.improves(previous.metrics);
                if ((candidate.metrics.natural() || recovering) && withinSpeed)
                {
                    solved = candidate;
                    accepted = true;
                    audit.recovering = recovering && !candidate.metrics.natural();
                    break;
                }
                fraction *= .5f;
            }
            if (!accepted)
            {
                if (!previous.metrics.natural())
                {
                    // Do not turn a transient or externally invalidated pose
                    // into an immortal owned guard. Native animation retains
                    // ownership until a valid entry exists again.
                    pose.published = pose.cacheValid = false;
                    pose.weight = 0.f;
                    pose.prepared = pose.orientationPrepared = pose.witnessPrepared = pose.choicePrepared = false;
                    return;
                }
                // A valid previous published pose is safer than teleporting to a
                // different endpoint when the path between the two is obstructed.
                solved = previous;
                audit.jointLimited = true;
            }
            audit.pathAccepted = accepted;
            audit.pathFraction = accepted ? fraction : 0.f;
        }
        audit.materialPoint = witness;
        audit.requestedAxis = chosenAxis;
        audit.requestedTarget = requested;
        audit.interceptPoint = intercept;
        audit.sourceContact = witness * *itemWorld;
        audit.handTarget = handTargetWorld;
        const osg::Matrixf sourceGrip = *itemWorld * osg::Matrixf::inverse(arm.world[2]);
        for (unsigned joint = 0; joint < arm.bones.size(); ++joint)
        {
            auto* bone = arm.bones[joint];
            if (pose.enabled || pose.releasing)
            {
                pose.source.emplace_back(bone, arm.source[joint]);
                bone->setMatrix(solved.local[joint]);
            }
        }
        if (mSkeleton)
            mSkeleton->markDirty();
        osg::NodeVisitor bounds(osg::NodeVisitor::TRAVERSE_ALL_CHILDREN);
        bounds.setVisitorType(osg::NodeVisitor::UPDATE_VISITOR);
        bounds.setTraversalNumber(visitor.getTraversalNumber());
        if (visitor.getFrameStamp())
            bounds.setFrameStamp(new osg::FrameStamp(*visitor.getFrameStamp()));
        bounds.setNodeMaskOverride(~0u);
        mObjectRoot->accept(bounds);

        const auto actualItem = nodeWorld(item);
        const auto actualShoulder = nodeWorld(arm.bones[0]);
        const auto actualElbow = nodeWorld(arm.bones[1]);
        const auto actualHand = nodeWorld(arm.bones[2]);
        if (!actualItem || !actualShoulder || !actualElbow || !actualHand)
            return;
        pose.meshItemWorld = *actualItem;
        pose.meshFrame = audit.frame;
        pose.meshSnapshotValid = true;
        const auto renderActor = nodeWorld(mPtr.getRefData().getBaseNode()).value_or(*actorWorld);
        pose.meshItemActor = *actualItem * osg::Matrixf::inverse(renderActor);
        pose.meshActorValid = true;
        audit.renderActorOrigin = renderActor.getTrans();
        audit.renderActorRight = osg::Matrixf::transform3x3(osg::Vec3f(1, 0, 0), renderActor);
        audit.renderActorForward = osg::Matrixf::transform3x3(osg::Vec3f(0, 1, 0), renderActor);
        audit.renderActorUp = osg::Matrixf::transform3x3(osg::Vec3f(0, 0, 1), renderActor);
        audit.active = pose.enabled || pose.releasing;
        audit.itemAxis = worldAxis(*actualItem);
        audit.itemContact = witness * *actualItem;
        audit.shoulder = actualShoulder->getTrans();
        audit.elbow = actualElbow->getTrans();
        audit.wrist = actualHand->getTrans();
        audit.finalUpperLength = (audit.elbow - audit.shoulder).length();
        audit.finalForeLength = (audit.wrist - audit.elbow).length();
        audit.shoulderError = (audit.shoulder - shoulder).length();
        audit.gripError = matrixError(sourceGrip, *actualItem * osg::Matrixf::inverse(*actualHand));
        audit.reachClamp = (audit.wrist - audit.handTarget).length();
        audit.contactError = (audit.itemContact - intercept).length();
        audit.skeletonShoulder = audit.shoulder * actorInverse;
        audit.skeletonElbow = audit.elbow * actorInverse;
        audit.skeletonWrist = audit.wrist * actorInverse;
        audit.finalUpperLengthUnscaled = (audit.skeletonElbow - audit.skeletonShoulder).length();
        audit.finalForeLengthUnscaled = (audit.skeletonWrist - audit.skeletonElbow).length();
        std::array<osg::Matrixf, 3> actualLocal;
        for (unsigned joint = 0; joint < arm.bones.size(); ++joint)
        {
            actualLocal[joint] = arm.bones[joint]->getMatrix();
            pose.publishedRotation[joint] = actualLocal[joint].getRotate();
            pose.publishedDelta[joint]
                = (osg::Matrixf::inverse(rotationOnly(arm.source[joint])) * rotationOnly(actualLocal[joint]))
                      .getRotate();
            audit.overlayCorrectionDegrees = std::max(
                audit.overlayCorrectionDegrees, rotationDistanceDegrees(osg::Quat(), pose.publishedDelta[joint]));
            audit.maxLocalTranslationError = std::max(audit.maxLocalTranslationError,
                osg::Vec3f(actualLocal[joint].getTrans() - arm.source[joint].getTrans()).length());
        }
        pose.published = audit.active;
        audit.upperLocalOrthoError = orthogonalityError(actualLocal[0]);
        audit.foreLocalOrthoError = orthogonalityError(actualLocal[1]);
        audit.handLocalOrthoError = orthogonalityError(actualLocal[2]);
        audit.maxLocalOrthoError
            = std::max({ audit.upperLocalOrthoError, audit.foreLocalOrthoError, audit.handLocalOrthoError });
        const std::array<osg::Matrixf, 3> actualMetric{ *actualShoulder * worldToMetric, *actualElbow * worldToMetric,
            *actualHand * worldToMetric };
        const auto joints = rig.measure(actualLocal, actualMetric,
            osg::Vec3f(actualMetric[2].getTrans() - handTargetWorld * worldToMetric).length());
        audit.wristSwingDegrees = joints.wristSwing;
        audit.wristTwistDegrees = joints.wristTwist;
        audit.palmFlexDegrees = joints.palmFlex;
        audit.palmSideDegrees = joints.palmSide;
        audit.forearmTwistDegrees = joints.foreTwist;
        audit.totalPronationDegrees = joints.foreTwist + joints.wristTwist;
        audit.upperSwingDegrees = joints.upperSwing;
        audit.upperTwistDegrees = joints.upperTwist;
        audit.signedElbowFlexDegrees = joints.signedElbowFlex;
        audit.elbowHingeErrorDegrees = joints.elbowHingeError;
        audit.elbowFlexDegrees = joints.elbowFlex;
        audit.bodyClearance = joints.clearance;
        audit.palmForward = joints.palmForward;
        audit.palmLateral = joints.palmLateral;
        audit.palmNormal = joints.palmNormal;
        audit.shieldCenter = joints.shieldCenter * *actorWorld;
        audit.shieldCenterAbovePelvis = joints.shieldHeight;
        audit.shieldForwardDistance = joints.shieldForward;
        audit.shieldTiltDegrees = joints.shieldTilt;
        audit.guardPelvis = pelvis * *actorWorld;
        audit.guardTorso = spine * *actorWorld;
        if (pose.shield)
        {
            float low = std::numeric_limits<float>::infinity(), high = -low;
            float back = low, front = high;
            for (const auto& triangle : pose.triangles)
                for (const auto& point : triangle)
                {
                    const auto worldPoint = point * *actualItem;
                    const auto localPoint = worldPoint * actorInverse;
                    const float depth = (localPoint - spine) * rig.forwardActor;
                    if (localPoint.z() < low)
                    {
                        low = localPoint.z();
                        audit.shieldBottom = worldPoint;
                    }
                    if (localPoint.z() > high)
                    {
                        high = localPoint.z();
                        audit.shieldTop = worldPoint;
                    }
                    if (depth < back)
                    {
                        back = depth;
                        audit.shieldBackPoint = worldPoint;
                    }
                    if (depth > front)
                    {
                        front = depth;
                        audit.shieldFrontPoint = worldPoint;
                    }
                }
        }
        audit.jointLimited = audit.jointLimited || (pose.enabled && !joints.natural());
        audit.clamped = audit.reachClamp > .1f || audit.jointLimited || (weight > .99f && joints.stanceMargin < 0.f);
        audit.fitted = weight > .99f && audit.contactError < 1.f && audit.shoulderError < .01f && audit.gripError < .01f
            && !audit.jointLimited && joints.working() && audit.maxLocalOrthoError < .001f;
    }
}
