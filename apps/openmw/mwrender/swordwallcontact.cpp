#include <components/settings/values.hpp>
#include "npcanimation.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <chrono>
#include <cstdlib>
#include <limits>
#include <set>

#include <osg/Geometry>
#include <osg/FrameStamp>
#include <osg/TriangleIndexFunctor>
#include <components/debug/debuglog.hpp>
#include <components/esm3/loadweap.hpp>
#include <components/sceneutil/positionattitudetransform.hpp>
#include <components/sceneutil/skeleton.hpp>

#include "camera.hpp"
#include "griparms.hpp"
#include "renderingmanager.hpp"
#include "vismask.hpp"
#include "../mwbase/environment.hpp"
#include "../mwbase/world.hpp"
#include "../mwmechanics/actorutil.hpp"
#include "../mwmechanics/creaturestats.hpp"
#include "../mwphysics/raycasting.hpp"
#include "../mwworld/class.hpp"

namespace MWRender
{
    namespace
    {
        struct Triangles
        {
            std::vector<std::array<unsigned int, 3>> triangles;
            void operator()(unsigned int a, unsigned int b, unsigned int c) { triangles.push_back({ a, b, c }); }
        };

        // Cache the real rendered blade, including its tip and width. There is no reach
        // extension, contact sphere, eye ray or invented side of the swing here.
        class BladeMeshVisitor : public osg::NodeVisitor
        {
            osg::Matrixf mRootInverse;
        public:
            std::vector<osg::Vec3f> vertices;
            std::vector<std::pair<osg::Vec3f, osg::Vec3f>> edges;
            float tip = 0.f;
            explicit BladeMeshVisitor(osg::Node* root)
                : osg::NodeVisitor(TRAVERSE_ALL_CHILDREN)
                , mRootInverse(osg::Matrixf::inverse(osg::computeLocalToWorld(osg::NodePath{ root })))
            {
                setTraversalMask(~(Mask_UpdateVisitor | Mask_Effect | Mask_ParticleSystem));
            }
            void limitSamples()
            {
                // Keep real material edges, including all six coordinate
                // extrema. Dense replacement meshes must not make every swing
                // trace tens of thousands of almost identical segments.
                constexpr std::size_t limit = 64;
                if (edges.size() <= limit)
                    return;
                std::vector<std::pair<osg::Vec3f, osg::Vec3f>> selected;
                std::vector<bool> used(edges.size(), false);
                std::vector<float> distance(edges.size(), std::numeric_limits<float>::infinity());
                const auto take = [&](std::size_t index) {
                    if (used[index]) return;
                    used[index] = true;
                    const auto edge = edges[index];
                    selected.push_back(edge);
                    for (std::size_t i = 0; i < edges.size(); ++i)
                    {
                        const auto center = (edges[i].first + edges[i].second) * .5f;
                        distance[i] = std::min(distance[i], std::min((center - edge.first).length2(),
                            (center - edge.second).length2()));
                    }
                };
                for (unsigned axis = 0; axis < 3; ++axis)
                    for (const float sign : { -1.f, 1.f })
                    {
                        std::size_t best = 0;
                        float support = -std::numeric_limits<float>::infinity();
                        for (std::size_t i = 0; i < edges.size(); ++i)
                        {
                            const float value = std::max(edges[i].first[axis] * sign, edges[i].second[axis] * sign);
                            if (value > support) { support = value; best = i; }
                        }
                        take(best);
                    }
                while (selected.size() < limit)
                {
                    std::size_t best = edges.size();
                    float farthest = -1.f;
                    for (std::size_t i = 0; i < edges.size(); ++i)
                        if (!used[i] && distance[i] > farthest) { best = i; farthest = distance[i]; }
                    if (best == edges.size()) break;
                    take(best);
                }
                edges = std::move(selected);
            }
            void apply(osg::Geometry& geometry) override
            {
                const auto* array = dynamic_cast<const osg::Vec3Array*>(geometry.getVertexArray());
                if (!array)
                    return;
                const osg::Matrixf local = osg::computeLocalToWorld(getNodePath()) * mRootInverse;
                std::vector<osg::Vec3f> points;
                for (const auto& vertex : *array)
                {
                    const osg::Vec3f point = vertex * local;
                    points.push_back(point);
                    if (std::isfinite(point.y()))
                        tip = std::max(tip, point.y());
                }
                osg::TriangleIndexFunctor<Triangles> triangles;
                geometry.accept(triangles);
                std::set<std::pair<unsigned int, unsigned int>> unique;
                for (const auto& triangle : triangles.triangles)
                    for (unsigned int i = 0; i < 3; ++i)
                    {
                        auto a = triangle[i], b = triangle[(i + 1) % 3];
                        if (a >= points.size() || b >= points.size() || a == b)
                            continue;
                        if (a > b)
                            std::swap(a, b);
                        const auto finite = [](const osg::Vec3f& point) {
                            return std::isfinite(point.x()) && std::isfinite(point.y()) && std::isfinite(point.z());
                        };
                        if (finite(points[a]) && finite(points[b]) && unique.emplace(a, b).second)
                            edges.emplace_back(points[a], points[b]);
                    }
            }
        };

        struct Pose
        {
            osg::Vec3f translation, scale;
            osg::Quat rotation, scaleRotation;
            explicit Pose(const osg::Matrixf& matrix)
            {
                matrix.decompose(translation, rotation, scale, scaleRotation);
            }
        };

        osg::Matrixf between(const Pose& a, const Pose& b, float t)
        {
            osg::Quat rotation, scaleRotation;
            rotation.slerp(t, a.rotation, b.rotation);
            scaleRotation.slerp(t, a.scaleRotation, b.scaleRotation);
            return osg::Matrixf::rotate(scaleRotation.inverse())
                * osg::Matrixf::scale(a.scale * (1.f - t) + b.scale * t)
                * osg::Matrixf::rotate(scaleRotation) * osg::Matrixf::rotate(rotation)
                * osg::Matrixf::translate(a.translation * (1.f - t) + b.translation * t);
        }
    }


    void NpcAnimation::configureSwordWallSweep(std::uint64_t swing, std::string_view group, std::string_view key)
    {
        auto& contact = mSwordWallContact;
        if (contact.swing != swing)
        {
            contact.swing = swing;
            contact.pending = false;
            contact.samples = 0;
            mSwordWallStruck = false;
            for (auto& weapon : mWallWeapons)
                weapon.recoilAge = -1.f;
        }
        contact.armed = !key.empty() && !mSwordWallStruck;
        mSwordWallGroup = group;
        mSwordWallKey = key;
        if (key.empty())
            contact.pending = false;
    }

    bool NpcAnimation::consumeSwordWallContact(std::uint64_t swing, std::string& key)
    {
        auto& contact = mSwordWallContact;
        if (!contact.pending)
            return false;
        contact.pending = false;
        if (contact.swing != swing || (mWallStruckMount == 0 ? getWeaponNode()
                : mShowCarriedLeft && mObjectParts[ESM::PRT_Shield] ? mObjectParts[ESM::PRT_Shield]->getNode().get() : nullptr)
                != mWallWeapons[mWallStruckMount].node.get())
            return false;
        key = contact.key;
        return true;
    }

    bool NpcAnimation::applySwordWallCorrection(
        const osg::Matrixf& correction, const osg::FrameStamp& frameStamp, osg::Matrixf* applied)
    {
        struct CorrectedBone
        {
            osg::MatrixTransform* node;
            osg::Matrixf sourceWorld, published;
        };
        std::vector<CorrectedBone> checked;
        osg::Matrixf sourceGrip;
        float checkedClamp = 0.f;
        const bool paired = hasPairedViewmodel();
        const bool solveArms = paired || hasPolearmViewmodel();
        const bool auditArms = solveArms && std::getenv("BRUTAL_SWORD_WALL_DIAGNOSTICS");
        // Keep the authored pose for restoring, also when the clinch fit moved this bone first.
        const auto authored = [&](osg::MatrixTransform* bone) {
            const auto clinch = std::find_if(mClinchSourcePose.begin(), mClinchSourcePose.end(),
                [bone](const auto& entry) { return entry.first.get() == bone; });
            return clinch == mClinchSourcePose.end() ? osg::Matrixf(bone->getMatrix()) : clinch->second;
        };
        const auto remember = [&](osg::MatrixTransform* bone) {
            if (std::none_of(mSwordWallSourcePose.begin(), mSwordWallSourcePose.end(),
                    [bone](const auto& entry) { return entry.first.get() == bone; }))
                mSwordWallSourcePose.emplace_back(bone, authored(bone));
        };
        if (applied)
            *applied = correction;
        if (solveArms)
        {
            // Solve both grips around the evaluated shoulders. A second response
            // during a tail composes once from the remembered authored pose.
            const osg::Matrixf total = mSwordWallSourcePose.empty()
                ? correction : mSwordWallAppliedTail * correction;
            const auto sourceLocal = [&](osg::MatrixTransform* bone) {
                const auto found = std::find_if(mSwordWallSourcePose.begin(), mSwordWallSourcePose.end(),
                    [bone](const auto& entry) { return entry.first.get() == bone; });
                return found == mSwordWallSourcePose.end() ? osg::Matrixf(bone->getMatrix()) : found->second;
            };
            std::array<GripArm, 2> arms;
            float maxClamp = 0.f;
            if (paired)
            {
                osg::Matrixf fitted;
                const osg::Vec3f pivot = mPtr.getRefData().getPosition().asVec3();
                if (!solveSharedGripArms(getNodeMap(), total, pivot, sourceLocal, arms, fitted))
                    return false;
                if (applied)
                    *applied = fitted;
            }
            else if (!solveGripArms(getNodeMap(), total, sourceLocal, arms, maxClamp))
                return false;
            if (auditArms)
            {
                const auto* weaponNode = getWeaponNode();
                if (weaponNode && !weaponNode->getParentalNodePaths().empty())
                {
                    const osg::Matrixf item = osg::computeLocalToWorld(weaponNode->getParentalNodePaths().front());
                    const osg::Matrixf currentHand = osg::computeLocalToWorld(arms[0].bones[2]->getParentalNodePaths().front());
                    const osg::Matrixf sourceItem = item * osg::Matrixf::inverse(currentHand) * arms[0].world[2];
                    sourceGrip = arms[1].world[2] * osg::Matrixf::inverse(sourceItem);
                }
                for (const auto& arm : arms)
                    for (unsigned joint = 0; joint < arm.bones.size(); ++joint)
                        checked.push_back({ arm.bones[joint], arm.world[joint], arm.result[joint] });
                checkedClamp = maxClamp;
            }
            // All validation and both solves finish before any bone is modified.
            for (auto& arm : arms)
                for (unsigned joint = 0; joint < arm.bones.size(); ++joint)
                {
                    remember(arm.bones[joint]);
                    arm.bones[joint]->setMatrix(arm.result[joint]);
                }
            if (std::getenv("BRUTAL_SWORD_WALL_DIAGNOSTICS") && maxClamp > .01f)
                Log(Debug::Info) << "Polearm recoil reach clamp: " << maxClamp;
        }
        else
        {
            std::array<osg::MatrixTransform*, 2> arms{};
            unsigned index = 0;
            for (const char* name : { "Bip01 L Clavicle", "Bip01 R Clavicle" })
            {
                const auto found = getNodeMap().find(name);
                if (found == getNodeMap().end() || found->second->getParentalNodePaths().empty())
                    return false;
                arms[index++] = found->second.get();
            }
            for (auto* arm : arms)
            {
                auto path = arm->getParentalNodePaths().front();
                path.pop_back();
                const osg::Matrixf parentWorld = osg::computeLocalToWorld(path);
                const auto source = arm->getMatrix();
                // A new contact can be fitted on top of an outgoing recovery. Restore
                // the true authored pose only once before the next UPDATE.
                if (std::none_of(mSwordWallSourcePose.begin(), mSwordWallSourcePose.end(),
                        [arm](const auto& entry) { return entry.first.get() == arm; }))
                    mSwordWallSourcePose.emplace_back(arm, authored(arm));
                arm->setMatrix(source * parentWorld * correction * osg::Matrixf::inverse(parentWorld));
            }
        }
        if (mSkeleton)
            mSkeleton->markDirty();
        osg::NodeVisitor bounds(osg::NodeVisitor::TRAVERSE_ALL_CHILDREN);
        bounds.setVisitorType(osg::NodeVisitor::UPDATE_VISITOR);
        bounds.setTraversalNumber(frameStamp.getFrameNumber());
        bounds.setFrameStamp(new osg::FrameStamp(frameStamp));
        bounds.setNodeMaskOverride(~0u);
        mObjectRoot->accept(bounds);
        if (!checked.empty())
        {
            double anchorError = 0., lengthError = 0., callbackError = 0.;
            std::array<osg::Matrixf, 6> actual;
            for (unsigned i = 0; i < checked.size(); ++i)
            {
                actual[i] = osg::computeLocalToWorld(checked[i].node->getParentalNodePaths().front());
                if (i % 3 == 0)
                    anchorError = std::max(anchorError,
                        (actual[i].getTrans() - checked[i].sourceWorld.getTrans()).length());
                else
                    lengthError = std::max(lengthError, std::abs(
                        (actual[i].getTrans() - actual[i - 1].getTrans()).length()
                        - (checked[i].sourceWorld.getTrans() - checked[i - 1].sourceWorld.getTrans()).length()));
                const auto& local = checked[i].node->getMatrix();
                for (unsigned element = 0; element < 16; ++element)
                    callbackError = std::max(callbackError,
                        std::abs(local.ptr()[element] - checked[i].published.ptr()[element]));
            }
            const auto* item = getWeaponNode();
            const osg::Matrixf itemWorld = osg::computeLocalToWorld(item->getParentalNodePaths().front());
            const osg::Matrixf actualGrip = actual[5] * osg::Matrixf::inverse(itemWorld);
            const float gripError = (actualGrip.getTrans() - sourceGrip.getTrans()).length();
            Log(Debug::Info) << (paired ? "Paired recoil audit: anchorError=" : "Polearm recoil audit: anchorError=")
                             << anchorError << " lengthError=" << lengthError << " gripError=" << gripError
                             << " reachClamp=" << checkedClamp << " callbackError=" << callbackError;
        }
        return true;
    }

    bool NpcAnimation::sampleSwordWallContact(float duration, const osg::FrameStamp& frameStamp)
    {
        mSwordWallAppliedTail.makeIdentity();
        // Disabled/third-person actors never resolve animation keys or item mounts.
        if (!Settings::game().mBrutalEnabled || !Settings::game().mBrutalWallBounce
            || mViewMode != VM_FirstPerson || !mShowWeapons || mBodyAwareness || useFullBodyFirstPerson())
        {
            mSwordWallCorrectionValid = false;
            mSwordWallTailAge = -1.f;
            for (auto& weapon : mWallWeapons)
            {
                weapon.history = false;
                weapon.recoilAge = -1.f;
            }
            return false;
        }
        const auto& stats = mPtr.getClass().getCreatureStats(mPtr);
        if (stats.isDead() || stats.mBrutalClinchActive || stats.mBrutalSlidePhase != 0)
        {
            mSwordWallCorrectionValid = false;
            mSwordWallTailAge = -1.f;
            for (auto& weapon : mWallWeapons)
            {
                weapon.history = false;
                weapon.recoilAge = -1.f;
            }
            return false;
        }
        osg::Node* left = mViewmodelSet == BrutalViewmodel_Dagger && mShowCarriedLeft
            && carriesLeftWeapon() && mObjectParts[ESM::PRT_Shield]
            ? mObjectParts[ESM::PRT_Shield]->getNode().get() : nullptr;
        const std::array<osg::Node*, 2> nodes{ getWeaponNode(), left };
        const auto& struck = mWallWeapons[mWallStruckMount];
        if (struck.node.get() != nodes[mWallStruckMount])
        {
            // A replacement weapon/rig cannot inherit the outgoing model's recovery.
            mSwordWallCorrectionValid = false;
            mSwordWallTailAge = -1.f;
        }
        bool active = false;
        if (mSwordWallCorrectionValid)
        {
            const auto& contact = mSwordWallContact;
            const float time = mSwordWallGroup.empty() ? -1.f : getCurrentTime(mSwordWallGroup);
            const float start = getTextKeyTime(mSwordWallGroup + ": " + contact.key + " start");
            const float stop = getTextKeyTime(mSwordWallGroup + ": " + contact.key + " stop");
            active = struck.materialWitness && struck.recoilAge >= 0.f
                && struck.recoilAge + std::max(duration, 0.f) <= .45f
                && (contact.pending || (time >= start && time <= stop));
        }
        if (mSwordWallCorrectionValid && !active)
        {
            // The clearance fold does not become zero when the extra retreat
            // envelope ends. Carry the actual last correction into the incoming
            // idle/attack pose and release it continuously, also on a new swing.
            mSwordWallTailAge = mSwordWallTailAge < 0.f ? 0.f : mSwordWallTailAge + std::max(duration, 0.f);
            const float t = std::clamp(mSwordWallTailAge / .28f, 0.f, 1.f);
            // Interpolate in a nearby frame. A world-space rotation about an
            // exterior pivot has a huge cancelling translation; interpolating
            // those independently sends the hands thousands of units away.
            const osg::Matrixf tail = osg::Matrixf::translate(-mSwordWallCorrectionOrigin)
                * between(Pose(mSwordWallLastCorrection), Pose(osg::Matrixf()), t * t * (3.f - 2.f * t))
                * osg::Matrixf::translate(mSwordWallCorrectionOrigin);
            applySwordWallCorrection(tail, frameStamp, &mSwordWallAppliedTail);
            if (t >= 1.f)
            {
                mSwordWallCorrectionValid = false;
                mSwordWallTailAge = -1.f;
            }
        }
        // Both real dagger mounts participate. A shield is never treated as a blade.
        // The first certified contact owns the swing, the effect and the recoil.
        bool hit = false;
        for (unsigned mount = 0; mount < nodes.size(); ++mount)
        {
            if (mSwordWallStruck && mount != mWallStruckMount)
            {
                // Its arm also recoils, although only the struck item owns the
                // response. Never sweep this sibling from a pre-recoil pose.
                mWallWeapons[mount].history = false;
                continue;
            }
            hit = sampleWeaponWallContact(mount, nodes[mount], duration, frameStamp) || hit;
        }
        return hit;
    }

    bool NpcAnimation::sampleWeaponWallContact(unsigned mount, osg::Node* node, float duration,
        const osg::FrameStamp& frameStamp)
    {
        auto& weapon = mWallWeapons[mount];
        auto& contact = mSwordWallContact;
        auto world = MWBase::Environment::get().getWorld();
        auto* camera = world->getCamera();
        if (mPtr != MWMechanics::getPlayer() || mViewMode != VM_FirstPerson || useFullBodyFirstPerson()
            || mBodyAwareness || !mShowWeapons || !camera)
        {
            weapon.history = false;
            return false;
        }
        if (node != weapon.node.get())
        {
            weapon.node = node;
            weapon.history = false;
            weapon.materialWitness = false;
            weapon.recoilAge = -1.f;
            contact.pending = false;
            weapon.vertices.clear();
            weapon.edges.clear();
            if (node)
            {
                BladeMeshVisitor mesh(node);
                node->accept(mesh);
                mesh.limitSamples();
                weapon.tip = mesh.tip;
                weapon.base = 0.f;
                const auto vertex = [&](const osg::Vec3f& point) {
                    const auto found = std::find_if(weapon.vertices.begin(), weapon.vertices.end(),
                        [&](const osg::Vec3f& v) { return (point - v).length2() < 1e-8f; });
                    if (found != weapon.vertices.end())
                        return std::size_t(found - weapon.vertices.begin());
                    weapon.vertices.push_back(point);
                    return weapon.vertices.size() - 1;
                };
                std::set<std::pair<std::size_t, std::size_t>> unique;
                if (std::isfinite(mesh.tip))
                    for (auto [a, b] : mesh.edges)
                    {
                        // The heel and guard are real sword geometry too. Cropping a
                        // guessed percentage discarded the only crossing when pressed
                        // against a wall, leaving every retained blade point inside it.
                        weapon.base = std::min({ weapon.base, a.y(), b.y() });
                        auto i = vertex(a), j = vertex(b);
                        if (i > j)
                            std::swap(i, j);
                        if (i != j && unique.emplace(i, j).second)
                            weapon.edges.emplace_back(i, j);
                    }
            }
        }
        if (!node || weapon.vertices.empty())
            return false;
        const auto paths = node->getParentalNodePaths();
        if (paths.empty())
            return false;
        const osg::Matrixf raw = osg::computeLocalToWorld(paths.front());
        const auto* root = mPtr.getRefData().getBaseNode();
        const auto rootPaths = root->getParentalNodePaths();
        if (rootPaths.empty())
            return false;
        const osg::Vec3f rootScale = root->getScale(), rootPosition = root->getPosition();
        const osg::Quat rootAttitude = root->getAttitude();
        const osg::Matrixf rootLocal = osg::Matrixf::scale(rootScale) * osg::Matrixf::rotate(rootAttitude)
            * osg::Matrixf::translate(rootPosition);
        const osg::Matrixf rootWorld = osg::computeLocalToWorld(rootPaths.front());
        const osg::Matrixf parent = osg::Matrixf::inverse(rootLocal) * rootWorld;
        const osg::Matrixf previousRoot = osg::Matrixf::scale(weapon.previousRootScale)
            * osg::Matrixf::rotate(weapon.previousRootAttitude)
            * osg::Matrixf::translate(weapon.previousRootPosition) * parent;

        // Match the FP cull projection. Its shared near/far and jitter cancel in
        // P_fp * inverse(P_world), leaving a view-space XY scale. Interpolate the
        // raw weapon, race root and camera separately: decomposing the FOV-scaled
        // world matrix would permute scale axes and invent a different blade.
        const osg::Matrixf view = camera->getViewMatrix();
        const osg::Matrixf cameraWorld = osg::Matrixf::inverse(view);
        double fov = mFirstPersonFieldOfView, aspect = 1., near = 1., far = 8192.;
        float fovRatio = 1.f;
        if (camera->getProjectionMatrix().getPerspective(fov, aspect, near, far))
            fovRatio = std::tan(osg::DegreesToRadians(fov) * .5) / std::tan(osg::DegreesToRadians(mFirstPersonFieldOfView) * .5f);
        const auto projected = [](const osg::Matrixf& weapon, const osg::Matrixf& cameraMatrix, float ratio) {
            if (std::abs(ratio - 1.f) < 1e-5f)
                return weapon;
            return weapon * osg::Matrixf::inverse(cameraMatrix)
                * osg::Matrixf::scale(ratio, ratio, 1.f) * cameraMatrix;
        };
        const osg::Matrixf matrix = projected(raw, cameraWorld, fovRatio);
        const osg::Matrixf previous = projected(weapon.previousMatrix,
            weapon.previousCamera, weapon.previousFovRatio);
        const Pose a(weapon.previousMatrix * osg::Matrixf::inverse(previousRoot));
        const Pose b(raw * osg::Matrixf::inverse(rootWorld));
        const Pose cameraA(weapon.previousCamera), cameraB(cameraWorld);
        const auto angleBetween = [](const osg::Quat& x, const osg::Quat& y) {
            return float(2. * std::acos(std::clamp(std::abs(x.asVec4() * y.asVec4()), 0., 1.)));
        };
        const float rootAngle = angleBetween(weapon.previousRootAttitude, rootAttitude);
        const float angle = angleBetween(a.rotation, b.rotation) + rootAngle
            + (std::abs(fovRatio - 1.f) > 1e-5f ? angleBetween(cameraA.rotation, cameraB.rotation) : 0.f);
        const auto uniform = [](const osg::Vec3f& scale) {
            return std::abs(scale.x() - scale.y()) < .002f && std::abs(scale.x() - scale.z()) < .002f;
        };
        const float time = mSwordWallGroup.empty() ? -1.f : getCurrentTime(mSwordWallGroup);
        const osg::Vec3f position = mPtr.getRefData().getPosition().asVec3();
        const bool history = weapon.history && duration > 0.f && duration <= .15f && uniform(a.scale) && uniform(b.scale)
            && (position - weapon.previousPosition).length2() < 128.f * 128.f
            && !(time >= 0.f && weapon.previousTime >= 0.f && time < weapon.previousTime - .001f)
            && angle <= osg::DegreesToRadians(100.f);
        const osg::Vec3f previousRootScale = weapon.previousRootScale;
        const osg::Vec3f previousRootPosition = weapon.previousRootPosition;
        const osg::Quat previousRootAttitude = weapon.previousRootAttitude;
        const float previousFovRatio = weapon.previousFovRatio;
        weapon.previousMatrix = raw;
        weapon.previousCamera = cameraWorld;
        weapon.previousRootScale = rootScale;
        weapon.previousRootPosition = rootPosition;
        weapon.previousRootAttitude = rootAttitude;
        weapon.previousFovRatio = fovRatio;
        weapon.previousPosition = position;
        weapon.previousTime = time;
        weapon.history = true;
        contact.currentBase = osg::Vec3f(0, weapon.base, 0) * matrix;
        contact.currentTip = osg::Vec3f(0, weapon.tip, 0) * matrix;
        if (weapon.materialWitness)
            contact.currentContactPoint = weapon.materialPoint * matrix;
        const auto& stats = mPtr.getClass().getCreatureStats(mPtr);
        const auto& inventory = mPtr.getClass().getInventoryStore(mPtr);
        const auto right = inventory.getSlot(MWWorld::InventoryStore::Slot_CarriedRight);
        const bool shortBlade = right != inventory.end() && right->getType() == ESM::Weapon::sRecordId
            && right->get<ESM::Weapon>()->mBase->mData.mType == ESM::Weapon::ShortBladeOneHand;
        const bool pairedGrasp = hasPolearmViewmodel() || hasPairedViewmodel();
        const auto recoil = [&](bool fresh) {
            if (!weapon.materialWitness || weapon.recoilAge < 0.f || stats.isDead()
                || stats.mBrutalClinchActive || stats.mBrutalSlidePhase != 0)
                return;
            if (!fresh)
            {
                weapon.recoilAge += std::max(duration, 0.f);
                const float start = getTextKeyTime(mSwordWallGroup + ": " + contact.key + " start");
                const float stop = getTextKeyTime(mSwordWallGroup + ": " + contact.key + " stop");
                if (!contact.pending && (time < start || time > stop))
                {
                    weapon.recoilAge = -1.f;
                    return;
                }
            }
            if (weapon.recoilAge > .45f)
            {
                weapon.recoilAge = -1.f;
                return;
            }
            const auto smooth = [](float t) { t = std::clamp(t, 0.f, 1.f); return t * t * (3.f - 2.f * t); };
            const float age = weapon.recoilAge;
            const float retreat = age < .1f ? 8.f * smooth(age / .1f)
                : age < .25f ? 8.f : 8.f * (1.f - smooth((age - .25f) / .2f));
            const osg::Matrixf inverseProjection = view * osg::Matrixf::scale(1.f / fovRatio, 1.f / fovRatio, 1.f) * cameraWorld;
            const osg::Vec3f pivot = contact.position * inverseProjection;
            osg::Vec3f bladeDirection = osg::Matrixf::transform3x3(osg::Vec3f(0, 1, 0), raw);
            bladeDirection.normalize();
            osg::Vec3f up = osg::Matrixf::transform3x3(osg::Vec3f(0, 1, 0), cameraWorld);
            up -= contact.normal * (up * contact.normal);
            if (up.normalize() < .01f)
                up = osg::Matrixf::transform3x3(osg::Vec3f(1, 0, 0), cameraWorld);
            osg::Vec3f axis = bladeDirection ^ up;
            float axisLength = axis.normalize();
            if (shortBlade && axisLength < .01f)
            {
                axis = bladeDirection ^ contact.normal;
                axisLength = axis.normalize();
            }
            if (!fresh && weapon.recoilFoldValid)
            {
                // A stable plane and nearest feasible angle avoid sign/plane
                // changes while the authored blade crosses the camera-up axis.
                axis = weapon.recoilFoldAxis;
                axisLength = axis.length();
            }
            osg::Matrixf correction;
            float bestPenalty = std::numeric_limits<float>::infinity();
            // In tight space pulling a long sword backwards can put it through the
            // near plane. Fold the whole arm pose upward about the struck point instead;
            // the wrist, fingers and weapon remain one rigid attached chain.
            int selectedDegrees = 0;
            osg::Vec3f selectedAxis = axis;
            float selectedRequired = 0.f, selectedDepth = 0.f;
            // The source pose can approach either side of the contact tangent.
            // Try both folds at each angle; a one-sided search can choose a
            // penetrating compromise while the opposite fold is fully clear.
            // A short blade may be carried much nearer the eye than a long
            // sword. Its own source set still has to clear the actual near plane,
            // but the sword's 22-unit comfort margin must not force it into a wall.
            const float maximumWithdrawal = shortBlade || pairedGrasp ? 32.f : 16.f;
            const float minimumDepth = std::max(float(near) + 2.f, shortBlade || pairedGrasp ? 10.f : 22.f);
            float bestMovement = std::numeric_limits<float>::infinity();
            const auto fitStarted = std::chrono::steady_clock::now();
            struct Reach
            {
                osg::Vec3f shoulder, wrist;
                float minimum, maximum;
            };
            std::vector<Reach> reaches;
            if (pairedGrasp)
                for (const char* prefix : { "Bip01 L ", "Bip01 R " })
                {
                    std::array<osg::Vec3f, 3> joints;
                    unsigned index = 0;
                    for (const char* suffix : { "UpperArm", "Forearm", "Hand" })
                    {
                        const auto found = getNodeMap().find(std::string(prefix) + suffix);
                        if (found == getNodeMap().end() || found->second->getParentalNodePaths().empty())
                            break;
                        joints[index++] = osg::computeLocalToWorld(found->second->getParentalNodePaths().front()).getTrans();
                    }
                    if (index == 3)
                    {
                        const float upperLength = (joints[1] - joints[0]).length();
                        const float foreLength = (joints[2] - joints[1]).length();
                        reaches.push_back({ joints[0], joints[2], std::abs(upperLength - foreLength) + .05f, upperLength + foreLength - .05f });
                    }
                }
            const auto consider = [&](const osg::Vec3f& foldAxis, int degrees, float budget) {
                const osg::Matrixf turn = osg::Matrixf::translate(-pivot)
                    * osg::Matrixf::rotate(osg::DegreesToRadians(float(degrees)), foldAxis)
                    * osg::Matrixf::translate(pivot);
                const osg::Matrixf candidate = projected(raw * turn, cameraWorld, fovRatio);
                float clearance = std::numeric_limits<float>::infinity();
                for (const auto& vertex : weapon.vertices)
                    clearance = std::min(clearance, (vertex * candidate - contact.position) * contact.normal);
                const float required = std::max(.25f + retreat - clearance, 0.f);
                const osg::Vec3f delta = osg::Matrixf::transform3x3(contact.normal * std::min(required, budget), inverseProjection);
                const osg::Matrixf adjusted = turn * osg::Matrixf::translate(delta);
                const osg::Matrixf visible = projected(raw * adjusted, cameraWorld, fovRatio);
                float nearest = std::numeric_limits<float>::infinity(), movement = 0.f;
                for (const auto& vertex : weapon.vertices)
                {
                    const auto point = vertex * visible;
                    nearest = std::min(nearest, -(point * view).z());
                    if (shortBlade || pairedGrasp)
                        movement += (point - vertex * matrix).length2();
                }
                float reachPenalty = 0.f;
                for (const auto& reach : reaches)
                {
                    const float distance = (reach.wrist * adjusted - reach.shoulder).length();
                    reachPenalty += std::max(distance - reach.maximum, 0.f)
                        + std::max(reach.minimum - distance, 0.f);
                }
                const float penalty = std::max(required - budget, 0.f) * 10.f
                    + std::max(minimumDepth - nearest, 0.f) * 20.f + reachPenalty * 40.f;
                if (penalty < bestPenalty
                    || (penalty < .001f && bestPenalty < .001f
                        && ((shortBlade || pairedGrasp) ? movement < bestMovement
                            : std::abs(degrees) < std::abs(selectedDegrees))))
                {
                    bestPenalty = penalty;
                    bestMovement = movement;
                    correction = adjusted;
                    selectedDegrees = degrees;
                    selectedAxis = foldAxis;
                    selectedRequired = required;
                    selectedDepth = nearest;
                }
                return std::pair(penalty, shortBlade || pairedGrasp ? movement : float(std::abs(degrees)));
            };
            const auto search = [&](const osg::Vec3f& foldAxis, float budget, int center) {
                std::array<int, 2> coarseBest{ -91, 91 };
                const std::pair<float, float> worst{ std::numeric_limits<float>::infinity(),
                    std::numeric_limits<float>::infinity() };
                std::array<std::pair<float, float>, 2> score{ worst, worst };
                // Coarse sweep followed by one-degree local refinement. The
                // previous fold is an exact candidate, keeping recoil stable.
                for (int option = 0; option <= 36; ++option)
                {
                    const int magnitude = ((option + 1) / 2) * 10;
                    const int degrees = center + (option % 2 ? magnitude : -magnitude);
                    if (std::abs(degrees) > 90) continue;
                    const auto candidate = consider(foldAxis, degrees, budget);
                    const unsigned side = degrees < 0 ? 0 : 1;
                    if (candidate < score[side]) { score[side] = candidate; coarseBest[side] = degrees; }
                }
                // Equal coarse fits on opposite sides can hide different
                // one-degree minima. Refine both signs rather than whichever
                // happened to be visited first.
                for (const int coarse : coarseBest)
                    if (std::abs(coarse) <= 90)
                        for (int delta = -9; delta <= 9; ++delta)
                            if (delta && std::abs(coarse + delta) <= 90)
                                consider(foldAxis, coarse + delta, budget);
            };
            if (axisLength >= .01f)
                search(axis, maximumWithdrawal, !fresh && weapon.recoilFoldValid ? weapon.recoilFoldDegrees : 0);
            else
                consider(axis, 0, maximumWithdrawal);
            if (bestPenalty > .001f && axisLength >= .01f)
            {
                // The primary fold can be blocked by the eye plane, including
                // an ordinary longsword alongside a sloping wall. Search other
                // planes, accepting only full mesh clearance AND actual near depth.
                // Prefer the feasible correction with least material displacement.
                osg::Vec3f crossAxis = bladeDirection ^ axis;
                crossAxis.normalize();
                osg::Vec3f diagonalA = axis + crossAxis, diagonalB = axis - crossAxis;
                diagonalA.normalize();
                diagonalB.normalize();
                const std::array<osg::Vec3f, 4> axes{ axis, crossAxis, diagonalA, diagonalB };
                // The recorded overhead return required 45.98 units along the
                // first fold, while the old 32-unit budget left visible penetration.
                // Try all additional planes within 32 first, expanding to 48 only
                // if none clears both the real wall and the real eye plane.
                const std::array<float, 2> budgets{
                    maximumWithdrawal, shortBlade || pairedGrasp ? 48.f : 24.f };
                for (const float budget : budgets)
                {
                    for (unsigned plane = budget == maximumWithdrawal ? 1u : 0u; plane < axes.size(); ++plane)
                        search(axes[plane], budget, 0);
                    if (bestPenalty < .001f)
                        break;
                }
            }
            if (bestPenalty > .001f && std::getenv("BRUTAL_SWORD_WALL_DIAGNOSTICS"))
                Log(Debug::Info) << "Sword recoil clearance compromise: age=" << age
                    << " degrees=" << selectedDegrees << " required=" << selectedRequired
                    << " nearDepth=" << selectedDepth << " penalty=" << bestPenalty;
            if ((shortBlade || pairedGrasp) && std::getenv("BRUTAL_SWORD_WALL_DIAGNOSTICS"))
                Log(Debug::Info) << "Blade recoil fit: age=" << age << " degrees=" << selectedDegrees
                    << " required=" << selectedRequired << " nearDepth=" << selectedDepth << " penalty=" << bestPenalty
                    << " fitMilliseconds=" << std::chrono::duration<float, std::milli>(
                        std::chrono::steady_clock::now() - fitStarted).count();
            weapon.recoilFoldAxis = selectedAxis;
            weapon.recoilFoldDegrees = selectedDegrees;
            weapon.recoilFoldValid = true;
            osg::Matrixf applied;
            if (!applySwordWallCorrection(correction, frameStamp, &applied))
                return;
            const osg::Matrixf total = hasPairedViewmodel() ? applied : mSwordWallAppliedTail * correction;
            mSwordWallCorrectionOrigin = mPtr.getRefData().getPosition().asVec3();
            mSwordWallLastCorrection = osg::Matrixf::translate(mSwordWallCorrectionOrigin) * total
                * osg::Matrixf::translate(-mSwordWallCorrectionOrigin);
            mSwordWallCorrectionValid = true;
            mSwordWallTailAge = -1.f;
            const osg::Matrixf finalRaw = osg::computeLocalToWorld(paths.front());
            const osg::Matrixf final = projected(finalRaw, cameraWorld, fovRatio);
            weapon.previousMatrix = finalRaw;
            contact.currentBase = osg::Vec3f(0, weapon.base, 0) * final;
            contact.currentTip = osg::Vec3f(0, weapon.tip, 0) * final;
            contact.currentContactPoint = weapon.materialPoint * final;
        };
        if (!contact.armed || mSwordWallStruck || stats.isDead() || stats.mBrutalClinchActive
            || stats.mBrutalSlidePhase != 0)
        {
            recoil(false);
            return false;
        }

        contact.sourceTime = time;
        contact.frame = frameStamp.getFrameNumber();
        ++contact.samples;
        contact.bladeBase = contact.currentBase;
        contact.bladeTip = contact.currentTip;
        contact.previousBase = osg::Vec3f(0, weapon.base, 0) * previous;
        contact.previousTip = osg::Vec3f(0, weapon.tip, 0) * previous;
        if (!history || (contact.bladeTip - contact.previousTip).length2() > 200.f * 200.f)
            return false; // Replacement/load/teleport is a new pose, never a swing through the gap.
        const int steps = std::max(1, int(std::ceil(angle / osg::DegreesToRadians(5.f))));
        const auto matrixAt = [&](float t) {
            osg::Quat attitude;
            attitude.slerp(t, previousRootAttitude, rootAttitude);
            const osg::Matrixf actor = osg::Matrixf::scale(previousRootScale * (1.f - t) + rootScale * t)
                * osg::Matrixf::rotate(attitude)
                * osg::Matrixf::translate(previousRootPosition * (1.f - t) + rootPosition * t) * parent;
            return projected(between(a, b, t) * actor, between(cameraA, cameraB, t),
                previousFovRatio * (1.f - t) + fovRatio * t);
        };
        float earliest = std::numeric_limits<float>::infinity();
        const auto test = [&](const osg::Vec3f& from, const osg::Vec3f& to, float t0, float t1, bool motion, const osg::Vec3f& localFrom, const osg::Vec3f& localTo) {
            if ((to - from).length2() < 1e-8f)
                return;
            MWPhysics::RayCastingResult hit{};
            if (!world->castRenderingRay(hit, from, to, true, true))
                return;
            const float ratio = std::clamp(((hit.mHitPos - from) * (to - from)) / (to - from).length2(), 0.f, 1.f);
            const float when = motion ? t0 + ratio * (t1 - t0) : t1;
            if (when >= earliest)
                return;
            earliest = when;
            contact.position = hit.mHitPos;
            contact.normal = hit.mHitNormal;
            contact.normal.normalize();
            if (contact.normal * (osg::Vec3f(camera->getPosition()) - hit.mHitPos) < 0.f)
                contact.normal = -contact.normal;
            contact.sweepFrom = from;
            contact.sweepTo = to;
            contact.sweepFraction = ratio;
            contact.kind = motion ? "motion" : "edge";
            weapon.materialPoint = localFrom * (1.f - ratio) + localTo * ratio;
        };
        struct Segment
        {
            osg::Vec3f from, to, localFrom, localTo;
            float t0, t1;
            unsigned bucket;
            bool motion;
        };
        std::vector<Segment> segments;
        std::vector<std::pair<osg::Vec3f, osg::Vec3f>> rays;
        std::vector<unsigned> buckets;
        const auto add = [&](const osg::Vec3f& from, const osg::Vec3f& to, float t0, float t1,
                             bool motion, const osg::Vec3f& localFrom, const osg::Vec3f& localTo,
                             unsigned bucket) {
            if ((to - from).length2() < 1e-8f)
                return;
            segments.push_back({ from, to, localFrom, localTo, t0, t1, bucket, motion });
            rays.emplace_back(from, to);
            buckets.push_back(bucket);
        };
        osg::Matrixf p = previous;
        // Sweep the cached material samples; broad-phase bounds only select
        // candidates and the renderer still certifies each actual contact.
        for (const auto& [i, j] : weapon.edges)
            add(weapon.vertices[i] * previous, weapon.vertices[j] * previous, 0.f, 0.f, false,
                weapon.vertices[i], weapon.vertices[j], 0);
        for (int step = 1; step <= steps; ++step)
        {
            const float t0 = float(step - 1) / steps, t1 = float(step) / steps;
            const osg::Matrixf q = step == steps ? matrix : matrixAt(t1);
            if (history)
                for (const auto& vertex : weapon.vertices)
                    add(vertex * p, vertex * q, t0, t1, true, vertex, vertex, step);
            for (const auto& [i, j] : weapon.edges)
                add(weapon.vertices[i] * q, weapon.vertices[j] * q, t0, t1, false,
                    weapon.vertices[i], weapon.vertices[j], step);
            p = q;
        }
        const auto sweepStarted = std::chrono::steady_clock::now();
        const bool legacy = std::getenv("BRUTAL_SWORD_WALL_LEGACY") != nullptr;
        const std::vector<float> candidates = legacy ? std::vector<float>()
            : world->findRenderingSegmentCandidates(rays, buckets);
        contact.sweepSegments = static_cast<unsigned>(segments.size());
        contact.sweepCandidates = contact.sweepConfirmed = 0;
        unsigned currentBucket = 0;
        for (std::size_t index = 0; index < segments.size(); ++index)
        {
            const auto& segment = segments[index];
            if (segment.bucket != currentBucket && std::isfinite(earliest))
                break; // Preserve the first contacting interpolation step.
            currentBucket = segment.bucket;
            if (!legacy)
            {
                if (index >= candidates.size() || !std::isfinite(candidates[index]))
                    continue;
                ++contact.sweepCandidates;
                const float candidateTime = segment.motion
                    ? segment.t0 + candidates[index] * (segment.t1 - segment.t0) : segment.t1;
                // The candidate pass is deliberately conservative at triangle
                // boundaries. Only the existing renderer ray certifies contact.
                if (candidateTime > earliest + 1e-3f)
                    continue;
            }
            ++contact.sweepConfirmed;
            test(segment.from, segment.to, segment.t0, segment.t1, segment.motion,
                segment.localFrom, segment.localTo);
        }
        contact.sweepMilliseconds = std::chrono::duration<float, std::milli>(
            std::chrono::steady_clock::now() - sweepStarted).count();
        if (!std::isfinite(earliest))
            return false;
        mWallStruckMount = mount;
        contact.mount = mount == 0 ? "right" : "left";
        weapon.materialWitness = true;
        contact.currentContactPoint = weapon.materialPoint * matrix;
        ++contact.count;
        contact.key = mSwordWallKey;
        contact.pending = true;
        contact.armed = false;
        mSwordWallStruck = true;
        weapon.recoilAge = 0.f;
        recoil(true);
        return true;
    }
}
