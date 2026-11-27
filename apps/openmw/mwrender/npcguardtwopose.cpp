#include "../mwbase/environment.hpp"
#include "../mwbase/world.hpp"
#include "../mwmechanics/actorutil.hpp"
#include "../mwmechanics/creaturestats.hpp"
#include "../mwmechanics/weapontype.hpp"
#include "../mwworld/class.hpp"
#include "actorutil.hpp"
#include "animblendcontroller.hpp"
#include "npcanimation.hpp"
#include "npcguardik.hpp"
#include <components/esm3/loadweap.hpp>
#include <components/sceneutil/positionattitudetransform.hpp>
#include <components/sceneutil/skeleton.hpp>
#include <components/settings/values.hpp>
#include <osg/FrameStamp>
#include <osg/NodeCallback>

namespace MWRender
{
    using namespace NpcGuardIk;
    namespace
    {
        // Measured with the actual native solver: 384 pair evaluations take
        // about 2.2 ms in the release SDK. Remaining work is the final step.
        constexpr unsigned sPairedGoalBudget = 384;

        std::string_view pairedGroup(const MWWorld::Ptr& actor)
        {
            if (!actor.getClass().hasInventoryStore(actor))
                return {};
            const auto& inv = actor.getClass().getInventoryStore(actor);
            const auto slot = inv.getSlot(MWWorld::InventoryStore::Slot_CarriedRight);
            if (slot == inv.end() || slot->getType() != ESM::Weapon::sRecordId)
                return {};
            const auto* type = MWMechanics::getWeaponType(slot->get<ESM::Weapon>()->mBase->mData.mType);
            if (type->mWeaponClass != ESM::WeaponType::Melee || !(type->mFlags & ESM::WeaponType::TwoHanded))
                return {};
            return type->mShortGroup == "2w" ? "brutal_weapon_guard_2w" : "brutal_weapon_guard_2c";
        }

        struct PairedArmReference
        {
            std::array<osg::Matrixf, 3> rotation;
            osg::Matrixf foreZero;
            osg::Vec3f upperHinge, foreHinge, palmForward, palmLateral, palmMcp;
        };

        bool makePairedRig(const Animation::NodeMap& nodes, const GripArm& original, bool left,
            const osg::Matrixf& actorInverse, const std::vector<GuardJointSource::Capsule>& body,
            const osg::Vec3f& torso, PairedArmReference& reference, bool calibrate, GuardJointSource& rig)
        {
            rig.arm = original;
            rig.metricToWorld = original.parent;
            const auto inverse = osg::Matrixf::inverse(original.parent);
            rig.metricToActor = original.parent * actorInverse;
            for (auto& matrix : rig.arm.world)
                matrix = matrix * inverse;
            rig.arm.parent.makeIdentity();
            rig.parentInverse.makeIdentity();
            if (calibrate)
            {
                for (unsigned joint = 0; joint < 3; ++joint)
                    reference.rotation[joint] = rotationOnly(original.source[joint]);
                osg::Vec3f upper = original.source[1].getTrans();
                upper.normalize();
                auto fore
                    = osg::Matrixf::transform3x3(osg::Vec3f(original.source[2].getTrans()), reference.rotation[1]);
                fore.normalize();
                auto hinge = upper ^ fore;
                const float flex = std::acos(std::clamp(upper * fore, -1.f, 1.f));
                if (hinge.normalize() < .001f || flex < osg::DegreesToRadians(5.f)
                    || flex > osg::DegreesToRadians(145.f))
                    return false;
                reference.upperHinge = hinge;
                reference.foreHinge = osg::Matrixf::transform3x3(hinge, osg::Matrixf::inverse(reference.rotation[1]));
                reference.foreZero = reference.rotation[1] * osg::Matrixf::rotate(-flex, hinge);
                std::vector<osg::Vec3f> mcp;
                const std::string prefix = left ? "Bip01 L Finger" : "Bip01 R Finger";
                for (int finger = 1; finger <= 4; ++finger)
                    if (const auto found = nodes.find(prefix + std::to_string(finger)); found != nodes.end()
                        && found->second->getNumParents() && found->second->getParent(0) == original.bones[2])
                        mcp.push_back(found->second->getMatrix().getTrans());
                reference.palmForward = {};
                reference.palmLateral = {};
                if (mcp.size() < 2)
                    return false;
                for (const auto& point : mcp)
                    reference.palmForward += point;
                reference.palmMcp = reference.palmForward / float(mcp.size());
                reference.palmForward.normalize();
                reference.palmLateral = mcp.back() - mcp.front();
                reference.palmLateral -= reference.palmForward * (reference.palmLateral * reference.palmForward);
                if (reference.palmLateral.normalize() < .001f)
                    return false;
            }
            rig.handRotation = reference.rotation[2];
            rig.upperRotation = reference.rotation[0];
            rig.foreZeroRotation = reference.foreZero;
            rig.upperHingeLocal = reference.upperHinge;
            rig.foreHingeLocal = reference.foreHinge;
            rig.palmForward = reference.palmForward;
            rig.palmLateral = reference.palmLateral;
            rig.wristAxis = original.source[2].getTrans();
            rig.wristAxis.normalize();
            rig.upperAxis = osg::Matrixf::transform3x3(osg::Vec3f(original.source[1].getTrans()), rig.upperRotation);
            rig.upperAxis.normalize();
            rig.upperLength = osg::Vec3f(rig.arm.world[1].getTrans() - rig.arm.world[0].getTrans()).length();
            rig.foreLength = osg::Vec3f(rig.arm.world[2].getTrans() - rig.arm.world[1].getTrans()).length();
            rig.body = body;
            auto outward = osg::Vec3f(original.world[0].getTrans()) * actorInverse - torso;
            outward.z() = 0.f;
            if (outward.normalize() < .001f)
                outward = osg::Vec3f(left ? -1.f : 1.f, 0, 0);
            rig.outward = osg::Matrixf::transform3x3(outward, osg::Matrixf::inverse(rig.metricToActor));
            rig.outward.normalize();
            return rig.upperLength > .001f && rig.foreLength > .001f;
        }

        osg::Matrixf interpolateItem(const osg::Matrixf& from, const osg::Matrixf& to, float amount)
        {
            osg::Quat rotation;
            rotation.slerp(amount, rotationOnly(from).getRotate(), rotationOnly(to).getRotate());
            auto result = osg::Matrixf::scale(from.getScale()) * osg::Matrixf::rotate(rotation);
            const osg::Vec3f a = from.getTrans(), b = to.getTrans();
            result.setTrans(a + (b - a) * amount);
            return result;
        }

        struct PairedSolution
        {
            std::array<GuardJointPose, 2> arms;
            std::array<float, 2> swivel{ 0.f, 0.f }, pronation{ 0.f, 0.f };
            osg::Matrixf itemActor;
            float gripError = 0.f;
            bool reachable = false, natural = false;
        };

        PairedSolution solvePaired(const std::array<GuardJointSource, 2>& rigs, const osg::Matrixf& itemActor,
            const std::array<osg::Matrixf, 2>& handFromItem, const std::array<float, 2>& swivel,
            const std::optional<std::array<float, 2>>& pronation = {}, bool searchSwivel = false,
            unsigned* evaluations = nullptr)
        {
            unsigned calls = 0;
            PairedSolution result;
            result.itemActor = itemActor;
            result.swivel = swivel;
            for (unsigned side = 0; side < 2; ++side)
            {
                const auto goal = handFromItem[side] * itemActor * osg::Matrixf::inverse(rigs[side].metricToActor);
                float best = std::numeric_limits<float>::infinity();
                for (float offset : { 0.f, -30.f, 30.f, -60.f, 60.f, -90.f, 90.f })
                {
                    if (!searchSwivel && offset != 0.f)
                        break;
                    const float angle = swivel[side] + offset;
                    const auto solution = solveGuardJoint(
                        rigs[side], goal, angle, pronation ? std::optional<float>((*pronation)[side]) : std::nullopt);
                    ++calls;
                    const float score = solution.metrics.score() + std::abs(offset) * .005f;
                    const bool feasible = solution.metrics.natural() && solution.metrics.reach < .01f;
                    if (feasible || score < best)
                    {
                        best = score;
                        result.arms[side] = solution;
                        result.swivel[side] = angle;
                        result.pronation[side] = solution.metrics.foreTwist;
                    }
                    if (feasible)
                        break;
                }
            }
            const auto actualItem = rigs[0].itemToHand * result.arms[0].world[2] * rigs[0].metricToActor;
            const auto actualLeft = result.arms[1].world[2] * rigs[1].metricToActor;
            result.gripError = matrixError(actualLeft * osg::Matrixf::inverse(actualItem), handFromItem[1]);
            result.reachable
                = result.arms[0].metrics.reach < .02f && result.arms[1].metrics.reach < .02f && result.gripError < .02f;
            result.natural = result.reachable && result.arms[0].metrics.natural() && result.arms[1].metrics.natural();
            if (evaluations)
                *evaluations += (calls + 1) / 2;
            return result;
        }

        struct WideHeadStyle
        {
            bool active = false;
            std::array<osg::Vec3f, 2> palmMcp;
        };

        float wideHeadElbowHeight(const GuardJointSource& rig, const GuardJointPose& pose)
        {
            return (osg::Vec3f(pose.world[1].getTrans()) * rig.metricToActor).z()
                - (osg::Vec3f(pose.world[0].getTrans()) * rig.metricToActor).z();
        }

        float wideHeadPalmAngle(const GuardJointSource& rig, const GuardJointPose& pose, const osg::Vec3f& palmMcp)
        {
            const osg::Vec3f elbow = pose.world[1].getTrans() * rig.metricToActor,
                             wrist = pose.world[2].getTrans() * rig.metricToActor,
                             grip = palmMcp * pose.world[2] * rig.metricToActor;
            auto fore = wrist - elbow;
            auto palm = grip - wrist;
            if (fore.normalize() < .001f || palm.normalize() < .001f)
                return 180.f;
            return osg::RadiansToDegrees(std::acos(std::clamp(fore * palm, -1.f, 1.f)));
        }

        bool wideHeadArmPreferred(const GuardJointSource& rig, const GuardJointPose& pose, const osg::Vec3f& palmMcp)
        {
            const osg::Vec3f elbow = pose.world[1].getTrans() * rig.metricToActor,
                             shoulder = pose.world[0].getTrans() * rig.metricToActor,
                             grip = palmMcp * pose.world[2] * rig.metricToActor;
            return wideHeadElbowHeight(rig, pose) <= 4.f && pose.metrics.wristSwing <= 30.f
                && std::abs(pose.metrics.palmFlex) <= 40.f && wideHeadPalmAngle(rig, pose, palmMcp) <= 40.f
                && (grip.z() <= shoulder.z() - 3.f || elbow.z() <= grip.z() + 1.f);
        }

        double wideHeadArmCost(const GuardJointSource& rig, const GuardJointPose& pose, const osg::Vec3f& palmMcp)
        {
            const float high = std::max(wideHeadElbowHeight(rig, pose) - 2.f, 0.f);
            const float bent = std::max(wideHeadPalmAngle(rig, pose, palmMcp) - 30.f, 0.f);
            return high * high * 3. + bent * bent * .03;
        }

        // Rank the actual palm and elbow, rather than accepting the first
        // anatomically legal swivel with the elbow folded over the forehead.
        // The zero/explicit calls are counted together as one pair evaluation.
        PairedSolution solveWideHeadEndpoint(const std::array<GuardJointSource, 2>& rigs, const osg::Matrixf& itemActor,
            const std::array<osg::Matrixf, 2>& handFromItem, const WideHeadStyle& style, unsigned& evaluations,
            unsigned limit, bool coarseOnly = false)
        {
            PairedSolution result;
            result.itemActor = itemActor;
            result.gripError = 1000.f;
            for (unsigned side = 0; side < 2; ++side)
            {
                const auto goal = handFromItem[side] * itemActor * osg::Matrixf::inverse(rigs[side].metricToActor);
                float best = std::numeric_limits<float>::infinity();
                bool selected = false, bestFeasible = false, bestPreferred = false;
                const float sign = side ? 1.f : -1.f;
                unsigned sampled = 0;
                for (float angle : { 60.f, 75.f, 45.f, 90.f, 30.f, 15.f, 0.f, 105.f, -30.f, -60.f, -90.f })
                {
                    if (coarseOnly && sampled++ >= 4)
                        break;
                    if (evaluations >= limit)
                        break;
                    const float swivel = angle * sign;
                    const auto zero = solveGuardJoint(rigs[side], goal, swivel, 0.f);
                    ++evaluations;
                    const float lower = std::max(-39.9f, zero.metrics.wristTwist - 44.9f),
                                upper = std::min(39.9f, zero.metrics.wristTwist + 44.9f);
                    if (lower > upper)
                        continue;
                    const float pronation = std::clamp(0.f, lower, upper);
                    const auto pose = solveGuardJoint(rigs[side], goal, swivel, pronation);
                    const bool feasible = pose.metrics.natural() && pose.metrics.reach < .02f;
                    const bool preferred = feasible && wideHeadArmPreferred(rigs[side], pose, style.palmMcp[side]);
                    const float cost = pose.metrics.score() + wideHeadArmCost(rigs[side], pose, style.palmMcp[side]);
                    if (!selected || (preferred && !bestPreferred)
                        || (preferred == bestPreferred
                            && ((feasible && !bestFeasible) || (feasible == bestFeasible && cost < best))))
                    {
                        selected = true;
                        best = cost;
                        bestFeasible = feasible;
                        bestPreferred = preferred;
                        result.arms[side] = pose;
                        result.swivel[side] = swivel;
                        result.pronation[side] = pronation;
                    }
                    if (preferred)
                        break;
                }
                if (!selected || (coarseOnly && !bestPreferred))
                    return result;
            }
            const auto actual = rigs[0].itemToHand * result.arms[0].world[2] * rigs[0].metricToActor;
            result.gripError = matrixError(
                result.arms[1].world[2] * rigs[1].metricToActor * osg::Matrixf::inverse(actual), handFromItem[1]);
            result.reachable
                = result.arms[0].metrics.reach < .02f && result.arms[1].metrics.reach < .02f && result.gripError < .02f;
            result.natural = result.reachable && result.arms[0].metrics.natural() && result.arms[1].metrics.natural();
            return result;
        }

        // Continue each elbow from the last published pose while both hands
        // follow the rigid item. Elbow swivel need not interpolate linearly.
        bool solvePairedStep(const std::array<GuardJointSource, 2>& rigs, const osg::Matrixf& itemActor,
            const std::array<osg::Matrixf, 2>& handFromItem, const WideHeadStyle& style, const PairedSolution& previous,
            const std::array<std::array<osg::Quat, 3>, 2>& previousRotations, float duration, unsigned& evaluations,
            PairedSolution& result)
        {
            result.itemActor = itemActor;
            bool both = true;
            unsigned calls = 0;
            for (unsigned side = 0; side < 2; ++side)
            {
                const auto goal = handFromItem[side] * itemActor * osg::Matrixf::inverse(rigs[side].metricToActor);
                float best = std::numeric_limits<float>::infinity();
                bool selected = false;
                // Drift of the torso may temporarily move an existing pose
                // outside the style preference. Permit a continuous recovery;
                // all actual joint/body limits remain mandatory.
                const float heightLimit = std::max(4.f, wideHeadElbowHeight(rigs[side], previous.arms[side]));
                const float palmLimit
                    = std::max(45.f, wideHeadPalmAngle(rigs[side], previous.arms[side], style.palmMcp[side]));
                for (float offset = -12.f; offset <= 12.f; offset += 1.f)
                {
                    const float swivel = previous.swivel[side] + offset;
                    const auto zero = solveGuardJoint(rigs[side], goal, swivel, 0.f);
                    ++calls;
                    const float lower = std::max(-39.9f, zero.metrics.wristTwist - 44.9f),
                                upper = std::min(39.9f, zero.metrics.wristTwist + 44.9f);
                    if (lower > upper)
                        continue;
                    const float pronation = std::clamp(previous.pronation[side], lower, upper);
                    const auto pose = solveGuardJoint(rigs[side], goal, swivel, pronation);
                    ++calls;
                    if (!pose.metrics.natural() || pose.metrics.reach >= .02f
                        || (style.active && (wideHeadElbowHeight(rigs[side], pose) > heightLimit
                            || wideHeadPalmAngle(rigs[side], pose, style.palmMcp[side]) > palmLimit)))
                        continue;
                    float cost = 0.f;
                    bool speedOk = true;
                    for (unsigned joint = 0; joint < 3; ++joint)
                    {
                        const float change
                            = rotationDistanceDegrees(previousRotations[side][joint], pose.local[joint].getRotate());
                        speedOk = speedOk && change <= (joint ? 360.f : 240.f) * duration + .05f;
                        cost += change * change;
                    }
                    if (speedOk && cost < best)
                    {
                        best = cost;
                        selected = true;
                        result.arms[side] = pose;
                        result.swivel[side] = swivel;
                        result.pronation[side] = pronation;
                    }
                }
                both = both && selected;
            }
            evaluations += (calls + 1) / 2;
            if (!both)
                return false;
            const auto actual = rigs[0].itemToHand * result.arms[0].world[2] * rigs[0].metricToActor;
            result.gripError = matrixError(
                result.arms[1].world[2] * rigs[1].metricToActor * osg::Matrixf::inverse(actual), handFromItem[1]);
            result.reachable
                = result.arms[0].metrics.reach < .02f && result.arms[1].metrics.reach < .02f && result.gripError < .02f;
            result.natural = result.reachable && result.arms[0].metrics.natural() && result.arms[1].metrics.natural();
            return result.natural;
        }

        double pairedViolation(const PairedSolution& pose)
        {
            double result = double(pose.gripError) * pose.gripError * 1000.;
            for (const auto& arm : pose.arms)
            {
                for (float violation : arm.metrics.hardViolations())
                    result += double(violation) * violation;
                result += double(arm.metrics.reach) * arm.metrics.reach * 10000.;
            }
            return result;
        }

        float pairedHeadStyle(const std::array<GuardJointSource, 2>& rigs, const osg::Vec3f& targetActor)
        {
            float shoulderHeight = 0.f;
            for (const auto& rig : rigs)
                shoulderHeight += (osg::Vec3f(rig.arm.world[0].getTrans()) * rig.metricToActor).z() * .5f;
            return std::clamp((targetActor.z() - shoulderHeight + 5.f) / 10.f, 0.f, 1.f);
        }

        float pairedForearmHeadGap(const GuardJointSource& rig, const GuardJointPose& arm)
        {
            if (rig.body.empty())
                return 100.f;
            const auto& head = rig.body.back();
            const auto axis = head.to - head.from;
            const osg::Vec3f elbow = arm.world[1].getTrans() * rig.metricToActor;
            const osg::Vec3f wrist = arm.world[2].getTrans() * rig.metricToActor;
            float result = 100.f;
            for (float fraction : { .25f, .5f, .75f })
            {
                const auto point = elbow + (wrist - elbow) * fraction;
                const float along = std::clamp((point - head.from) * axis / std::max(axis.length2(), .001f), 0.f, 1.f);
                result = std::min(result, (point - head.from - axis * along).length() - head.radius - 2.5f);
            }
            return result;
        }

        double pairedStyleCost(const std::array<GuardJointSource, 2>& rigs, const PairedSolution& pose, float headStyle,
            const WideHeadStyle* wideStyle = nullptr)
        {
            double result = 0.;
            for (unsigned side = 0; side < 2; ++side)
            {
                const auto& arm = pose.arms[side];
                const auto& m = arm.metrics;
                result += m.wristSwing * m.wristSwing * .02 + m.wristTwist * m.wristTwist * .005
                    + m.upperTwist * m.upperTwist * .002 + m.palmFlex * m.palmFlex * .005;
                const float lateral = std::max(std::abs(m.palmSide) - 20.f, 0.f);
                const float clearance = std::max(4.f - m.clearance, 0.f);
                result += lateral * lateral * .03 + clearance * clearance * .2;
                const osg::Vec3f elbow = arm.world[1].getTrans() * rigs[side].metricToActor;
                const osg::Vec3f wrist = arm.world[2].getTrans() * rigs[side].metricToActor;
                const float fold = std::max(elbow.z() - wrist.z() + 2.f, 0.f);
                const float face = std::max(3.f - pairedForearmHeadGap(rigs[side], arm), 0.f);
                result += headStyle * .5 * (fold * fold + face * face);
                if (wideStyle && wideStyle->active)
                    result += headStyle * wideHeadArmCost(rigs[side], arm, wideStyle->palmMcp[side]);
            }
            return result;
        }

        bool pairedPreferredStyle(const std::array<GuardJointSource, 2>& rigs, const PairedSolution& pose,
            float headStyle, const WideHeadStyle* wideStyle = nullptr)
        {
            if (!pose.natural)
                return false;
            if (headStyle <= 0.f)
                return true;
            // This is a goal preference for high guards, not a relaxation of
            // the anatomical limiter. Hold a safe partial guard while refining
            // instead of folding a forearm over the face to get exact contact.
            for (unsigned side = 0; side < 2; ++side)
            {
                const auto& arm = pose.arms[side];
                const osg::Vec3f elbow = arm.world[1].getTrans() * rigs[side].metricToActor;
                const osg::Vec3f wrist = arm.world[2].getTrans() * rigs[side].metricToActor;
                if (wideStyle && wideStyle->active)
                {
                    if (!wideHeadArmPreferred(rigs[side], arm, wideStyle->palmMcp[side]))
                        return false;
                }
                else if (elbow.z() > wrist.z() || arm.metrics.wristSwing > 30.f
                    || std::abs(arm.metrics.palmFlex) > 40.f)
                    return false;
            }
            return true;
        }

        struct PairedGoalQuality
        {
            PairedSolution pose;
            double cost = 0.;
            bool ready = false;
        };

        PairedGoalQuality assessPairedGoal(const std::array<GuardJointSource, 2>& rigs, const osg::Matrixf& itemActor,
            const std::array<osg::Matrixf, 2>& handFromItem, const std::array<float, 2>& swivel,
            const std::array<float, 2>& pronation, const osg::Matrixf& fromItem, const std::array<float, 2>& fromSwivel,
            const std::array<float, 2>& fromPronation, float headStyle, unsigned& evaluations,
            const WideHeadStyle* wideStyle, bool useElbowContinuation)
        {
            PairedGoalQuality result;
            result.pose = solvePaired(rigs, itemActor, handFromItem, swivel, pronation);
            ++evaluations;
            result.cost
                = pairedViolation(result.pose) * 10000. + pairedStyleCost(rigs, result.pose, headStyle, wideStyle);
            if (useElbowContinuation)
            {
                // Validate the actual elbow continuation when publishing each step.
                // A linear swivel corridor is not a constraint of that motion.
                result.ready = pairedPreferredStyle(rigs, result.pose, headStyle, wideStyle);
                return result;
            }
            bool connected = true;
            // Endpoints alone do not prove that both arms can follow the same
            // rigid item. Include the actual interpolation family in the fit.
            for (float fraction : { .125f, .25f, .375f, .5f, .625f, .75f, .875f })
            {
                std::array<float, 2> pathSwivel, pathPronation;
                for (unsigned side = 0; side < 2; ++side)
                {
                    pathSwivel[side] = fromSwivel[side] + (swivel[side] - fromSwivel[side]) * fraction;
                    pathPronation[side] = fromPronation[side] + (pronation[side] - fromPronation[side]) * fraction;
                }
                const auto path = solvePaired(
                    rigs, interpolateItem(fromItem, itemActor, fraction), handFromItem, pathSwivel, pathPronation);
                ++evaluations;
                connected = connected && path.natural;
                result.cost += pairedViolation(path) * 10000.;
            }
            result.ready = connected && pairedPreferredStyle(rigs, result.pose, headStyle, wideStyle);
            return result;
        }
    }

    struct NpcAnimation::NpcTwoHandGuardPose
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
                    animation->restoreNpcTwoHandGuardPose();
                traverse(node, visitor);
                if (update)
                    animation->fitNpcTwoHandGuardPose(*visitor);
            }
        };
        bool enabled = false, releasing = false, published = false, referenceReady = false;
        bool snapshot = false, cache = false, cachedNatural = false, wideHeadMode = false;
        bool cachedExplicitPronation = false, refining = false, refinementFinished = false;
        bool refinementSweepImproved = false;
        float pendingDuration = 0.f, pendingElapsed = 0.f, weight = 0.f, releaseElapsed = 0.f;
        osg::Vec3f target, incoming, cachedTarget, cachedDirection, axis;
        osg::observer_ptr<osg::Node> root, item;
        osg::ref_ptr<Callback> callback;
        std::vector<std::pair<osg::observer_ptr<osg::MatrixTransform>, osg::Matrixf>> source;
        std::vector<std::array<osg::Vec3f, 3>> triangles;
        std::vector<osg::Vec3f> witnesses, material;
        std::array<PairedArmReference, 2> reference;
        osg::Matrixf referenceItemActor, leftFromItem, currentItemActor, lastSourceItemActor, releaseDelta;
        osg::Matrixf snapshotItemWorld, snapshotItemActor;
        unsigned snapshotFrame = 0, cachedWitness = 0;
        osg::Quat cachedTurn;
        osg::Quat refinementBaseTurn;
        std::array<float, 8> refinementParameters{};
        unsigned refinementStep = 0, refinementCoordinate = 0, refinementIteration = 0;
        std::array<osg::Matrixf, 2> cachedMetricToActor;
        float cachedOffset = 8.f;
        std::array<float, 2> currentSwivel{ 0.f, 0.f }, currentPronation{ 0.f, 0.f }, cachedSwivel{ 0.f, 0.f };
        std::array<float, 2> cachedPronation{ 0.f, 0.f };
        std::array<std::array<osg::Quat, 3>, 2> publishedRotations;
        explicit NpcTwoHandGuardPose(NpcAnimation* animation)
            : callback(new Callback(animation))
        {
        }
    };

    bool NpcAnimation::usesNpcTwoHandGuard() const
    {
        return !pairedGroup(mPtr).empty();
    }

    void NpcAnimation::requestNpcGuardRigAudit()
    {
        mNpcGuardRigAudit.reset();
        mNpcGuardRigAuditRequested = true;
    }

    std::optional<NpcAnimation::NpcGuardRigAudit> NpcAnimation::getNpcGuardRigForAudit() const
    {
        return mNpcGuardRigAudit;
    }

    void NpcAnimation::setNpcTwoHandGuardTarget(bool enabled, const osg::Vec3f& point, const osg::Vec3f& direction)
    {
        enabled = enabled && finite(point) && finite(direction) && direction.length2() > .01f;
        if (!mNpcTwoHandGuardPose)
        {
            if (!enabled)
                return;
            mNpcTwoHandGuardPose = std::make_shared<NpcTwoHandGuardPose>(this);
        }
        auto& pose = *mNpcTwoHandGuardPose;
        if (!enabled && pose.enabled && pose.published)
        {
            pose.refining = false;
            pose.releasing = true;
            pose.releaseElapsed = 0.f;
            pose.releaseDelta = osg::Matrixf::inverse(pose.lastSourceItemActor) * pose.currentItemActor;
        }
        if (enabled)
        {
            if (!pose.enabled && !pose.published && !pose.releasing)
            {
                // A completed release belongs to ordinary native animation.
                // Re-enter from the next fully blended authored paired pose,
                // rather than reusing a finished attack's item/arm state.
                pose.referenceReady = pose.cache = pose.refining = pose.refinementFinished = false;
                pose.cachedExplicitPronation = false;
                pose.weight = 0.f;
                mNpcGuardRigAudit.reset();
            }
            pose.releasing = false;
            pose.target = point;
            pose.incoming = direction;
            pose.incoming.normalize();
        }
        pose.enabled = enabled;
        updateNpcTwoHandGuardPose(0.f);
    }

    void NpcAnimation::updateNpcTwoHandGuardPose(float duration)
    {
        if (!mNpcTwoHandGuardPose)
            return;
        auto& pose = *mNpcTwoHandGuardPose;
        pose.pendingDuration += std::clamp(duration, 0.f, .05f);
        pose.pendingElapsed += std::max(duration, 0.f);
        if (pose.root.get() != mObjectRoot.get())
        {
            mNpcGuardRigAudit.reset();
            mNpcGuardRigAuditRequested = false;
            restoreNpcTwoHandGuardPose();
            if (pose.root.valid())
                pose.root->removeUpdateCallback(pose.callback);
            pose.root = mObjectRoot.get();
            pose.item = nullptr;
            pose.referenceReady = pose.published = pose.cache = pose.releasing = false;
            pose.refining = pose.refinementFinished = pose.cachedExplicitPronation = false;
            pose.weight = 0.f;
            if (mObjectRoot)
                mObjectRoot->addUpdateCallback(pose.callback);
        }
    }

    void NpcAnimation::restoreNpcTwoHandGuardPose()
    {
        if (!mNpcTwoHandGuardPose)
            return;
        for (const auto& [bone, matrix] : mNpcTwoHandGuardPose->source)
            if (bone.valid())
                bone->setMatrix(matrix);
        if (!mNpcTwoHandGuardPose->source.empty() && mSkeleton)
            mSkeleton->markDirty();
        mNpcTwoHandGuardPose->source.clear();
    }

    void NpcAnimation::clearNpcTwoHandGuardPose()
    {
        if (!mNpcTwoHandGuardPose)
            return;
        mNpcGuardRigAudit.reset();
        mNpcGuardRigAuditRequested = false;
        restoreNpcTwoHandGuardPose();
        if (mNpcTwoHandGuardPose->root.valid())
            mNpcTwoHandGuardPose->root->removeUpdateCallback(mNpcTwoHandGuardPose->callback);
        mNpcTwoHandGuardPose.reset();
        mNpcGuardGeometry = {};
    }

    void NpcAnimation::fitNpcTwoHandGuardPose(osg::NodeVisitor& visitor)
    {
        if (!mNpcTwoHandGuardPose)
            return;
        auto& pose = *mNpcTwoHandGuardPose;
        auto& audit = mNpcGuardGeometry;
        audit = {};
        audit.twoHanded = true;
        audit.enabled = pose.enabled;
        audit.frame = visitor.getTraversalNumber();
        pose.snapshot = false;
        const float duration = std::exchange(pose.pendingDuration, 0.f),
                    elapsed = std::exchange(pose.pendingElapsed, 0.f);
        if (!pose.enabled && !pose.releasing)
            return;
        const auto& stats = mPtr.getClass().getCreatureStats(mPtr);
        if (mPtr == MWMechanics::getPlayer() || mBodyAwareness || mViewMode != VM_Normal
            || !Settings::game().mDarkMessiahNpcCombat || Settings::game().mBrutalClassicNpcAnimations || stats.isDead()
            || stats.getKnockedDown() || stats.isParalyzed() || isRagdollPose() || stats.mBrutalClinchActive
            || (stats.getHitRecovery() && !stats.mBrutalNpcGuardRequested))
        {
            pose.enabled = pose.releasing = pose.published = false;
            pose.weight = 0.f;
            audit.enabled = false;
            mNpcGuardRigAudit.reset();
            mNpcGuardRigAuditRequested = false;
            return;
        }
        osg::Node* item = mShowWeapons ? getWeaponNode() : nullptr;
        const auto itemWorld = nodeWorld(item), actorWorld = nodeWorld(mPtr.getRefData().getBaseNode());
        if (!itemWorld || !actorWorld)
        {
            pose.published = false;
            return;
        }
        const auto actorInverse = osg::Matrixf::inverse(*actorWorld);
        if (pose.item.get() != item)
        {
            mNpcGuardRigAudit.reset();
            mNpcGuardRigAuditRequested = false;
            pose.item = item;
            pose.referenceReady = pose.published = pose.cache = pose.releasing = false;
            pose.refining = pose.refinementFinished = pose.cachedExplicitPronation = false;
            pose.weight = 0.f;
            GuardMeshVisitor mesh(item);
            item->accept(mesh);
            pose.triangles = std::move(mesh.triangles);
            pose.witnesses.clear();
            pose.material.clear();
            if (!mesh.bounds.valid() || pose.triangles.empty())
                return;
            const auto span = mesh.bounds._max - mesh.bounds._min;
            unsigned axis = 0;
            for (unsigned i = 1; i < 3; ++i)
                if (span[i] > span[axis])
                    axis = i;
            pose.axis = {};
            pose.axis[axis] = 1.f;
            const bool wide = pairedGroup(mPtr) == "brutal_weapon_guard_2w";
            const float low = mesh.bounds._min[axis], high = mesh.bounds._max[axis];
            const float tip = std::abs(high) >= std::abs(low) ? high : low;
            if (tip < 0.f)
                pose.axis[axis] = -1.f;
            for (float fraction : { .5f, .35f, .65f, .2f, .8f, .1f, .9f })
            {
                auto desired = mesh.bounds.center();
                desired[axis] = wide ? low + (high - low) * fraction : tip * fraction;
                if (const auto witness = materialWitness(pose.triangles, desired, pose.axis,
                        -std::numeric_limits<float>::infinity(), std::numeric_limits<float>::infinity()))
                    pose.witnesses.push_back(*witness);
            }
            pose.material = pose.witnesses;
            for (unsigned coordinate = 0; coordinate < 3; ++coordinate)
                for (float value : { mesh.bounds._min[coordinate], mesh.bounds._max[coordinate] })
                {
                    auto desired = mesh.bounds.center();
                    desired[coordinate] = value;
                    if (const auto point = materialWitness(pose.triangles, desired, pose.axis,
                            -std::numeric_limits<float>::infinity(), std::numeric_limits<float>::infinity()))
                        pose.material.push_back(*point);
                }
        }
        if (pose.witnesses.empty())
            return;
        pose.snapshot = true;
        pose.snapshotItemWorld = *itemWorld;
        pose.snapshotItemActor = *itemWorld * actorInverse;
        pose.snapshotFrame = audit.frame;
        audit.meshAvailable = true;
        std::array<GripArm, 2> original;
        if (!captureGuardArm(getNodeMap(), false, original[0]) || !captureGuardArm(getNodeMap(), true, original[1]))
            return;
        const auto group = pairedGroup(mPtr);
        audit.sourceGroupMatches = !group.empty() && isPlaying(group) && getActiveGroup(BoneGroup_RightArm) == group
            && getActiveGroup(BoneGroup_LeftArm) == group && getActiveGroup(BoneGroup_Torso) == group;
        audit.sourceTime = getCurrentTime(group);
        audit.sourceHoldTime = getTextKeyTime(std::string(group) + ": brutal guard hold");
        float speed = 1.f;
        getInfo(group, nullptr, &speed);
        audit.sourceHoldReached = audit.sourceHoldTime >= 0.f
            && std::abs(audit.sourceTime - audit.sourceHoldTime) < .0005f && std::abs(speed) < .0001f;
        const auto inspectBlend = [&](osg::Node* node) {
            const osg::ref_ptr<osg::Node> key = node;
            const auto active = [&](osg::Callback* callback) {
                return std::any_of(mActiveControllers.begin(), mActiveControllers.end(),
                    [&](const auto& entry) { return entry.first.get() == node && entry.second.get() == callback; });
            };
            if (const auto c = mAnimBlendControllers.find(key); c != mAnimBlendControllers.end())
                audit.sourceBlendActive
                    = audit.sourceBlendActive || (active(c->second->getAsCallback()) && c->second->isBlending());
            if (const auto c = mBoneAnimBlendControllers.find(key); c != mBoneAnimBlendControllers.end())
                audit.sourceBlendActive
                    = audit.sourceBlendActive || (active(c->second->getAsCallback()) && c->second->isBlending());
        };
        for (const auto& arm : original)
            for (auto* bone : arm.bones)
                inspectBlend(bone);
        if (!pose.referenceReady)
            for (const auto& arm : original)
                for (osg::Node* parent = arm.bones[0]->getNumParents() ? arm.bones[0]->getParent(0) : nullptr;
                    parent && parent != mObjectRoot.get();
                    parent = parent->getNumParents() ? parent->getParent(0) : nullptr)
                    inspectBlend(parent);
        if (!pose.referenceReady && (!audit.sourceGroupMatches || !audit.sourceHoldReached || audit.sourceBlendActive))
            return;
        const auto boneActor = [&](std::string_view name, const osg::Vec3f& fallback) -> osg::Vec3f {
            const auto matrix = getBoneWorldMatrix(name);
            return matrix ? osg::Vec3f(matrix->getTrans() * actorInverse) : fallback;
        };
        const osg::Vec3f spine = boneActor("Bip01 Spine2", osg::Vec3f(0, 0, 90)),
                         waist = boneActor("Bip01 Spine1", spine - osg::Vec3f(0, 0, 20));
        const osg::Vec3f pelvis = boneActor("Bip01 Pelvis", waist - osg::Vec3f(0, 0, 12)),
                         head = boneActor("Bip01 Head", spine + osg::Vec3f(0, 0, 20));
        const std::vector<GuardJointSource::Capsule> body{ { waist, spine, 9.f },
            { pelvis - osg::Vec3f(0, 0, 4), pelvis + osg::Vec3f(0, 0, 8), 10.f },
            { head, head + osg::Vec3f(0, 0, 7), 7.5f } };
        std::array<GuardJointSource, 2> rigs;
        const bool calibrate = !pose.referenceReady;
        for (unsigned side = 0; side < 2; ++side)
            if (!makePairedRig(getNodeMap(), original[side], side != 0, actorInverse, body, spine, pose.reference[side],
                    calibrate, rigs[side]))
                return;
        rigs[0].itemToHand = *itemWorld * osg::Matrixf::inverse(original[0].world[2]);
        rigs[0].itemPoints = pose.material;
        const auto sourceItemActor = *itemWorld * actorInverse;
        std::array<osg::Matrixf, 2> handFromItem{ osg::Matrixf::inverse(rigs[0].itemToHand), pose.leftFromItem };
        if (calibrate)
        {
            for (unsigned side = 0; side < 2; ++side)
                if (!rigs[side].measure(rigs[side].arm.source, rigs[side].arm.world, 0.f).natural())
                    return;
            pose.referenceItemActor = sourceItemActor;
            pose.leftFromItem = original[1].world[2] * osg::Matrixf::inverse(*itemWorld);
            handFromItem[1] = pose.leftFromItem;
            pose.currentItemActor = sourceItemActor;
            pose.referenceReady = true;
            pose.currentSwivel = { 0.f, 0.f };
            pose.currentPronation = { 0.f, 0.f };
        }
        audit.referenceReady = pose.referenceReady;
        if (mNpcGuardRigAuditRequested)
        {
            NpcGuardRigAudit capture;
            capture.frame = audit.frame;
            capture.matrices["actorWorld"] = *actorWorld;
            capture.matrices["itemWorld"] = *itemWorld;
            capture.matrices["referenceItemActor"] = pose.referenceItemActor;
            capture.matrices["leftFromItem"] = pose.leftFromItem;
            capture.matrices["currentItemActor"] = pose.currentItemActor;
            capture.matrices["lastSourceItemActor"] = pose.lastSourceItemActor;
            capture.matrices["releaseDelta"] = pose.releaseDelta;
            capture.matrices["cachedTurn"] = osg::Matrixf::rotate(pose.cachedTurn);
            capture.vectors["target"] = pose.target;
            capture.vectors["incoming"] = pose.incoming;
            capture.vectors["cachedTarget"] = pose.cachedTarget;
            capture.vectors["cachedDirection"] = pose.cachedDirection;
            capture.vectors["itemAxis"] = pose.axis;
            capture.scalars["enabled"] = pose.enabled;
            capture.scalars["releasing"] = pose.releasing;
            capture.scalars["published"] = pose.published;
            capture.scalars["referenceReady"] = pose.referenceReady;
            capture.scalars["sourceGroupMatches"] = audit.sourceGroupMatches;
            capture.scalars["sourceHoldReached"] = audit.sourceHoldReached;
            capture.scalars["sourceBlendActive"] = audit.sourceBlendActive;
            capture.scalars["duration"] = duration;
            capture.scalars["elapsed"] = elapsed;
            capture.scalars["weight"] = pose.weight;
            capture.scalars["releaseElapsed"] = pose.releaseElapsed;
            capture.scalars["cache"] = pose.cache;
            capture.scalars["cachedNatural"] = pose.cachedNatural;
            capture.scalars["cachedExplicitPronation"] = pose.cachedExplicitPronation;
            capture.scalars["refining"] = pose.refining;
            capture.scalars["refinementFinished"] = pose.refinementFinished;
            capture.scalars["refinementStep"] = pose.refinementStep;
            capture.scalars["refinementCoordinate"] = pose.refinementCoordinate;
            capture.matrices["refinementBaseTurn"] = osg::Matrixf::rotate(pose.refinementBaseTurn);
            for (unsigned parameter = 0; parameter < pose.refinementParameters.size(); ++parameter)
                capture.scalars["refinementParameter" + std::to_string(parameter)]
                    = pose.refinementParameters[parameter];
            capture.scalars["cachedWitness"] = pose.cachedWitness;
            capture.scalars["cachedOffset"] = pose.cachedOffset;
            capture.scalars["witnessCount"] = pose.witnesses.size();
            capture.scalars["wide"] = group == "brutal_weapon_guard_2w";
            capture.scalars["wideHeadMode"] = pose.wideHeadMode;
            for (unsigned index = 0; index < pose.witnesses.size(); ++index)
                capture.vectors["witness" + std::to_string(index)] = pose.witnesses[index];
            for (unsigned side = 0; side < 2; ++side)
            {
                const auto& rig = rigs[side];
                const std::string prefix = side ? "left." : "right.";
                for (unsigned joint = 0; joint < 3; ++joint)
                {
                    const auto index = std::to_string(joint);
                    capture.matrices[prefix + "source" + index] = rig.arm.source[joint];
                    capture.matrices[prefix + "world" + index] = rig.arm.world[joint];
                    capture.matrices[prefix + "sourceWorld" + index] = original[side].world[joint];
                    capture.matrices[prefix + "referenceRotation" + index] = pose.reference[side].rotation[joint];
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
                capture.vectors[prefix + "palmMcp"] = pose.reference[side].palmMcp;
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
                capture.scalars[prefix + "cachedSwivel"] = pose.cachedSwivel[side];
                capture.scalars[prefix + "cachedPronation"] = pose.cachedPronation[side];
                capture.scalars[prefix + "currentSwivel"] = pose.currentSwivel[side];
                capture.scalars[prefix + "currentPronation"] = pose.currentPronation[side];
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
            }
            mNpcGuardRigAudit = std::move(capture);
            mNpcGuardRigAuditRequested = false;
        }
        osg::Vec3f targetActor = pose.target * actorInverse;
        osg::Vec3f directionActor = osg::Matrixf::transform3x3(pose.incoming, actorInverse);
        directionActor.normalize();
        int weaponType = ESM::Weapon::None;
        MWMechanics::getActiveWeapon(mPtr, &weaponType);
        const bool closeBlade = weaponType == ESM::Weapon::LongBladeTwoHand;
        const float targetHeight = pairedHeadStyle(rigs, targetActor);
        // On a compact blade guard the low-elbow head preference starts above
        // shoulder height. A chest interception may require a raised elbow.
        const float headStyle = closeBlade && targetHeight <= .5f ? 0.f : targetHeight;
        const WideHeadStyle wideStyle{ group == "brutal_weapon_guard_2w" && headStyle > 0.f,
            { pose.reference[0].palmMcp, pose.reference[1].palmMcp } };
        const bool useElbowContinuation = wideStyle.active || closeBlade;
        if (!pose.releasing && pose.wideHeadMode != wideStyle.active)
        {
            // A low goal is not a seed for the supported high-shaft stance.
            // Keep the actual pose and its finite release, discard only the
            // endpoint search when crossing between the two strategies.
            pose.cache = pose.refining = pose.refinementFinished = pose.cachedExplicitPronation = false;
            pose.wideHeadMode = wideStyle.active;
        }
        osg::Vec3f interceptWorld = pose.target - pose.incoming * 8.f;
        unsigned witnessIndex = pose.cache ? pose.cachedWitness : 0;
        PairedSolution desired;
        bool found = false;
        if (pose.releasing)
        {
            pose.releaseElapsed += elapsed;
            audit.releasing = true;
            audit.releaseTimeLeft = std::max(.20f - pose.releaseElapsed, 0.f);
            const float remain = std::clamp(1.f - pose.releaseElapsed / .20f, 0.f, 1.f),
                        envelope = remain * remain * (3.f - 2.f * remain);
            audit.weight = pose.weight * envelope;
            handFromItem[1] = original[1].world[2] * osg::Matrixf::inverse(*itemWorld);
            for (unsigned side = 0; side < 2; ++side)
            {
                auto temporary = pose.reference[side];
                if (!makePairedRig(getNodeMap(), original[side], side != 0, actorInverse, body, spine, temporary, true,
                        rigs[side]))
                {
                    pose.releasing = pose.published = false;
                    return;
                }
            }
            rigs[0].itemToHand = *itemWorld * osg::Matrixf::inverse(original[0].world[2]);
            rigs[0].itemPoints = pose.material;
            for (float attenuation : { 1.f, .75f, .5f, .25f, 0.f })
            {
                osg::Quat turn;
                turn.slerp(envelope * attenuation, osg::Quat(), rotationOnly(pose.releaseDelta).getRotate());
                auto delta = osg::Matrixf::rotate(turn);
                delta.setTrans(osg::Vec3f(pose.releaseDelta.getTrans()) * (envelope * attenuation));
                desired = solvePaired(
                    rigs, sourceItemActor * delta, handFromItem, { 0.f, 0.f }, std::array<float, 2>{ 0.f, 0.f });
                if (desired.reachable)
                {
                    found = true;
                    break;
                }
            }
            if (pose.releaseElapsed >= .20f || !found)
            {
                pose.releasing = pose.published = false;
                pose.weight = 0.f;
                audit.releasing = false;
                return;
            }
        }
        else
        {
            pose.weight = std::clamp(pose.weight + duration / .18f, 0.f, 1.f);
            audit.weight = pose.weight * pose.weight * (3.f - 2.f * pose.weight);
            const auto candidateItem = [&](unsigned witness, const osg::Quat& turn, float offset) {
                const auto anchor = (pose.target - pose.incoming * offset) * actorInverse;
                return pose.referenceItemActor
                    * osg::Matrixf::translate(-(pose.witnesses[witness] * pose.referenceItemActor))
                    * osg::Matrixf::rotate(turn) * osg::Matrixf::translate(anchor);
            };
            bool same = pose.cache && (targetActor - pose.cachedTarget).length2() < 4.f
                && directionActor * pose.cachedDirection > .995f;
            if (same && pose.refinementFinished && !pose.cachedNatural)
                for (unsigned side = 0; side < 2; ++side)
                    same = same && matrixError(rigs[side].metricToActor, pose.cachedMetricToActor[side]) < .05f;
            const bool changedRefinementTarget = pose.refining
                && ((targetActor - pose.cachedTarget).length2() > 256.f
                    || directionActor * pose.cachedDirection < .95f);
            if (pose.cache && ((!same && !pose.cachedNatural && !pose.refining) || changedRefinementTarget))
            {
                pose.cache = pose.refining = pose.refinementFinished = pose.cachedExplicitPronation = false;
            }
            if (pose.cache)
            {
                desired = solvePaired(rigs, candidateItem(pose.cachedWitness, pose.cachedTurn, pose.cachedOffset),
                    handFromItem, pose.cachedSwivel,
                    pose.cachedExplicitPronation ? std::optional(pose.cachedPronation) : std::nullopt,
                    !pose.cachedExplicitPronation, wideStyle.active ? &audit.searchCandidates : nullptr);
                if (!wideStyle.active)
                    ++audit.searchCandidates;
                if (desired.natural)
                {
                    found = true;
                    audit.cacheUsed = true;
                }
                else if (!pose.cachedNatural && same && pose.refinementFinished)
                {
                    audit.cacheUsed = true;
                    audit.cacheNegative = true;
                }
            }
            if (!found && !audit.cacheNegative && !pose.refining)
            {
                // A new coarse seed has not exhausted refinement, even when
                // the previous successful cached goal just became infeasible.
                pose.refinementFinished = false;
                audit.fullSearch = true;
                float best = std::numeric_limits<float>::infinity();
                const auto search = [&] {
                    if (wideStyle.active)
                    {
                        auto sourceAxis = osg::Matrixf::transform3x3(pose.axis, pose.referenceItemActor);
                        sourceAxis.normalize();
                        auto forward = directionActor;
                        forward.z() = 0.f;
                        forward.normalize();
                        auto lateral = forward ^ osg::Vec3f(0, 0, 1);
                        lateral.normalize();
                        if (lateral * sourceAxis < 0.f)
                            lateral = -lateral;
                        desired = solvePaired(rigs, candidateItem(0, osg::Quat(), 8.f), handFromItem, { 0.f, 0.f },
                            std::array<float, 2>{ 0.f, 0.f });
                        ++audit.searchCandidates;
                        witnessIndex = 0;
                        pose.cachedTurn = {};
                        pose.cachedSwivel = desired.swivel;
                        pose.cachedOffset = 8.f;
                        best
                            = pairedViolation(desired) * 10000. + pairedStyleCost(rigs, desired, headStyle, &wideStyle);
                        struct Choice
                        {
                            unsigned witness, orientation;
                            osg::Quat turn;
                            float offset, travel;
                        };
                        std::vector<Choice> choices;

                        const std::array<osg::Matrixf, 2> actorToMetric{ osg::Matrixf::inverse(rigs[0].metricToActor),
                            osg::Matrixf::inverse(rigs[1].metricToActor) };
                        const std::array<osg::Vec3f, 2> sourceHands{
                            (handFromItem[0] * pose.referenceItemActor).getTrans(),
                            (handFromItem[1] * pose.referenceItemActor).getTrans()
                        };
                        unsigned orientation = 0;
                        const auto addOrientation = [&](float slope, float front, float roll) {
                            auto axis = lateral * std::cos(osg::DegreesToRadians(slope))
                                - osg::Vec3f(0, 0, 1) * std::sin(osg::DegreesToRadians(slope)) - forward * front;
                            axis.normalize();
                            osg::Quat alignment;
                            alignment.makeRotate(sourceAxis, axis);
                            const auto rotation = osg::Matrixf::rotate(alignment)
                                * osg::Matrixf::rotate(osg::DegreesToRadians(roll), axis);
                            for (unsigned witness = 0; witness < pose.witnesses.size(); ++witness)
                                for (float offset : { 8.f, 24.f, 40.f })
                                {
                                    const auto anchor = (pose.target - pose.incoming * offset) * actorInverse;
                                    const auto contact = pose.witnesses[witness] * pose.referenceItemActor;
                                    bool reachable = true;
                                    float travel = 0;
                                    for (unsigned side = 0; side < 2; ++side)
                                    {
                                        const osg::Vec3f hand
                                            = (osg::Matrixf::transform3x3(sourceHands[side] - contact, rotation)
                                                  + anchor)
                                            * actorToMetric[side];
                                        const float distance
                                            = osg::Vec3f(hand - rigs[side].arm.world[0].getTrans()).length();
                                        const float minimum = std::sqrt(std::max(0.f,
                                            rigs[side].upperLength * rigs[side].upperLength
                                                + rigs[side].foreLength * rigs[side].foreLength
                                                + 2.f * rigs[side].upperLength * rigs[side].foreLength
                                                    * std::cos(osg::DegreesToRadians(145.f))));
                                        reachable = reachable && distance >= minimum
                                            && distance <= rigs[side].upperLength + rigs[side].foreLength - .01f;
                                        travel += osg::Vec3f(hand - rigs[side].arm.world[2].getTrans()).length();
                                    }
                                    if (reachable)
                                    {
                                        // Prefer a contact supported between the two grips
                                        // when hand travel is nearly tied across anchors.
                                        travel += .1f * (contact - (sourceHands[0] + sourceHands[1]) * .5f).length();
                                        choices.push_back(
                                            { witness, orientation, rotation.getRotate(), offset, travel });
                                    }
                                }
                            ++orientation;
                        };
                        // Breadth first in pitch, forward cant and axial roll.
                        // Nearby material anchors never starve another orientation.
                        for (int radius = 0; radius <= 3; ++radius)
                            for (int pitch : { 0, 1, -1 })
                                for (int front : { 0, 1, -1 })
                                    for (int roll : { 0, 1, -1 })
                                        if (std::abs(pitch) + std::abs(front) + std::abs(roll) == radius)
                                            addOrientation(
                                                40.f + pitch * 10.f, .25f + front * .25f, -60.f + roll * 15.f);
                        // Keep wider orientations after the neutral neighbourhood.
                        for (float slope : { 40.f, 30.f, 50.f, 45.f, 20.f, 60.f })
                            for (float front : { .25f, 0.f, .5f })
                                for (float roll : { -60.f, -45.f, -75.f, -30.f, 60.f, 45.f, 75.f, 0.f, 90.f })
                                    if (!((slope == 40.f || slope == 30.f || slope == 50.f)
                                            && (roll == -60.f || roll == -45.f || roll == -75.f)))
                                        addOrientation(slope, front, roll);
                        std::stable_sort(choices.begin(), choices.end(), [](const Choice& a, const Choice& b) {
                            if (a.orientation != b.orientation)
                                return a.orientation < b.orientation;
                            return a.travel < b.travel;
                        });
                        const auto evaluate = [&](const Choice& c, bool coarseOnly) {
                            if (audit.searchCandidates >= 96)
                                return;
                            auto solution = solveWideHeadEndpoint(rigs, candidateItem(c.witness, c.turn, c.offset),
                                handFromItem, wideStyle, audit.searchCandidates, 96, coarseOnly);
                            const bool preferred = pairedPreferredStyle(rigs, solution, headStyle, &wideStyle);
                            const float score = pairedViolation(solution) * 10000.
                                + pairedStyleCost(rigs, solution, headStyle, &wideStyle);
                            if (preferred || score < best)
                            {
                                best = score;
                                desired = solution;
                                witnessIndex = c.witness;
                                pose.cachedTurn = c.turn;
                                pose.cachedSwivel = solution.swivel;
                                pose.cachedOffset = c.offset;
                            }
                            found = preferred;
                        };
                        std::vector<bool> visited(orientation, false);
                        for (unsigned index = 0; index < choices.size(); ++index)
                        {
                            if (visited[choices[index].orientation])
                                continue;
                            visited[choices[index].orientation] = true;
                            evaluate(choices[index], true);
                            if (found)
                                return;
                            if (audit.searchCandidates >= 96)
                                break;
                        }
                        for (unsigned index = 0; index < choices.size(); ++index)
                        {
                            evaluate(choices[index], false);
                            if (found || audit.searchCandidates >= 96)
                                return;
                        }
                        return;
                    }
                    // The native slash-ready blade points away from the body.
                    // A guard must also admit cross-body, upright and hanging
                    // blade directions; small yaw/pitch offsets alone cannot
                    // put that source's material in front of the body.
                    auto sourceAxis = osg::Matrixf::transform3x3(pose.axis, pose.referenceItemActor);
                    sourceAxis.normalize();
                    osg::Vec3f lateral = directionActor ^ osg::Vec3f(0, 0, 1);
                    if (lateral.normalize() < .001f)
                        lateral = osg::Vec3f(1, 0, 0);
                    const osg::Vec3f up(0, 0, 1);
                    const std::array<osg::Vec3f, 9> axes{ sourceAxis, up, -up, lateral, -lateral, lateral + up,
                        lateral - up, -lateral + up, -lateral - up };
                    struct Choice
                    {
                        unsigned witness;
                        osg::Quat turn;
                        float offset, score;
                        unsigned orientation;
                    };
                    std::vector<Choice> choices;
                    unsigned orientation = 0;
                    const std::array<osg::Matrixf, 2> actorToMetric{ osg::Matrixf::inverse(rigs[0].metricToActor),
                        osg::Matrixf::inverse(rigs[1].metricToActor) };
                    for (auto axis : axes)
                    {
                        axis.normalize();
                        osg::Quat align;
                        align.makeRotate(sourceAxis, axis);
                        for (float roll : { 0.f, -30.f, 30.f, -60.f, 60.f, -90.f, 90.f })
                        {
                            const auto turn = (osg::Matrixf::rotate(align)
                                * osg::Matrixf::rotate(osg::DegreesToRadians(roll), axis))
                                                  .getRotate();
                            for (unsigned witness = 0; witness < pose.witnesses.size(); ++witness)
                                for (float offset : { 8.f, 24.f, 40.f })
                                {
                                    const auto itemGoal = candidateItem(witness, turn, offset);
                                    bool reachable = true;
                                    float travel = 0.f;
                                    for (unsigned side = 0; side < 2; ++side)
                                    {
                                        const auto hand = handFromItem[side] * itemGoal * actorToMetric[side];
                                        const float distance
                                            = osg::Vec3f(hand.getTrans() - rigs[side].arm.world[0].getTrans()).length();
                                        const float minimum = std::sqrt(std::max(0.f,
                                            rigs[side].upperLength * rigs[side].upperLength
                                                + rigs[side].foreLength * rigs[side].foreLength
                                                + 2.f * rigs[side].upperLength * rigs[side].foreLength
                                                    * std::cos(osg::DegreesToRadians(145.f))));
                                        reachable = reachable && distance >= minimum
                                            && distance <= rigs[side].upperLength + rigs[side].foreLength - .01f;
                                        travel += osg::Vec3f(hand.getTrans() - rigs[side].arm.world[2].getTrans())
                                                      .length();
                                    }
                                    // Both fixed grip points must be reachable
                                    // before spending the bounded joint budget.
                                    if (reachable)
                                        choices.push_back({ witness, turn, offset,
                                            travel + rotationDistanceDegrees(osg::Quat(), turn) * .05f, orientation });
                                }
                            ++orientation;
                        }
                    }
                    std::stable_sort(choices.begin(), choices.end(),
                        [](const Choice& a, const Choice& b) { return a.score < b.score; });
                    const auto evaluate = [&](const Choice& choice) {
                        if (audit.searchCandidates >= 96)
                            return;
                        auto solution = solvePaired(rigs, candidateItem(choice.witness, choice.turn, choice.offset),
                            handFromItem, { 0.f, 0.f }, {}, true);
                        ++audit.searchCandidates;
                        const float score = solution.arms[0].metrics.score() + solution.arms[1].metrics.score()
                            + solution.gripError * 1000.f;
                        if (solution.natural || score < best)
                        {
                            best = score;
                            desired = solution;
                            witnessIndex = choice.witness;
                            pose.cachedTurn = choice.turn;
                            pose.cachedSwivel = solution.swivel;
                            pose.cachedOffset = choice.offset;
                        }
                        found = solution.natural;
                    };
                    // Give every orientation its best reachable material point
                    // before refining neighbours. No near-hilt witness or roll
                    // is starved by a long sequence of impossible tip goals.
                    std::vector<bool> tested(choices.size(), false), visited(orientation, false);
                    for (unsigned index = 0; index < choices.size(); ++index)
                        if (!visited[choices[index].orientation])
                        {
                            visited[choices[index].orientation] = true;
                            tested[index] = true;
                            evaluate(choices[index]);
                            if (found || audit.searchCandidates >= 96)
                                return;
                        }
                    for (unsigned index = 0; index < choices.size(); ++index)
                        if (!tested[index])
                        {
                            evaluate(choices[index]);
                            if (found || audit.searchCandidates >= 96)
                                return;
                        }
                };
                search();
                pose.cache = true;
                pose.cachedNatural = found;
                pose.cachedWitness = witnessIndex;
                pose.cachedTarget = targetActor;
                pose.cachedDirection = directionActor;
                pose.cachedExplicitPronation = false;
                for (unsigned side = 0; side < 2; ++side)
                    pose.cachedMetricToActor[side] = rigs[side].metricToActor;
            }
            else if (found)
                witnessIndex = pose.cachedWitness;
            // Refine a common rigid item pose and both arm configurations.
            // A coarse endpoint may need a lower elbow or a different route
            // from the current pose. The bounded search resumes next frame.
            const bool exhaustedNegative = pose.refinementFinished && !pose.cachedNatural && same;
            if (exhaustedNegative)
            {
                found = false;
                audit.cacheNegative = true;
            }
            else if (audit.searchCandidates + 8 <= sPairedGoalBudget)
            {
                const bool continuing = pose.refining;
                if (!continuing)
                {
                    pose.refinementBaseTurn = pose.cachedTurn;
                    pose.refinementParameters = { 0.f, 0.f, 0.f, pose.cachedOffset, pose.cachedSwivel[0],
                        pose.cachedSwivel[1], std::clamp(desired.pronation[0], -39.9f, 39.9f),
                        std::clamp(desired.pronation[1], -39.9f, 39.9f) };
                    pose.refinementStep = pose.refinementCoordinate = pose.refinementIteration = 0;
                    pose.refinementSweepImproved = false;
                    pose.refinementFinished = false;
                }
                const auto refinedTurn = [&](const std::array<float, 8>& parameters) {
                    return (osg::Matrixf::rotate(pose.refinementBaseTurn)
                        * osg::Matrixf::rotate(osg::DegreesToRadians(parameters[0]), osg::Vec3f(1, 0, 0))
                        * osg::Matrixf::rotate(osg::DegreesToRadians(parameters[1]), osg::Vec3f(0, 1, 0))
                        * osg::Matrixf::rotate(osg::DegreesToRadians(parameters[2]), osg::Vec3f(0, 0, 1)))
                        .getRotate();
                };
                const auto assess = [&](const std::array<float, 8>& parameters) {
                    return assessPairedGoal(rigs,
                        candidateItem(pose.cachedWitness, refinedTurn(parameters), parameters[3]), handFromItem,
                        { parameters[4], parameters[5] }, { parameters[6], parameters[7] }, pose.currentItemActor,
                        pose.currentSwivel, pose.currentPronation, headStyle, audit.searchCandidates, &wideStyle,
                        useElbowContinuation);
                };
                auto quality = assess(pose.refinementParameters);
                found = quality.ready;
                pose.refining = !found;
                const std::array<float, 7> steps{ 30.f, 15.f, 7.5f, 3.f, 1.f, .3f, .1f };
                while (pose.refining && audit.searchCandidates + 8 <= sPairedGoalBudget)
                {
                    if (pose.refinementStep >= steps.size())
                    {
                        pose.refining = false;
                        pose.refinementFinished = true;
                        break;
                    }
                    const unsigned dimension = pose.refinementCoordinate / 2;
                    const float sign = pose.refinementCoordinate % 2 ? 1.f : -1.f;
                    auto parameters = pose.refinementParameters;
                    parameters[dimension] += sign * steps[pose.refinementStep] * (dimension == 3 ? .25f : 1.f);
                    if (dimension == 3)
                        parameters[dimension] = std::clamp(parameters[dimension], 8.f, 40.f);
                    else if (dimension == 4 || dimension == 5)
                        parameters[dimension] = std::clamp(parameters[dimension], -180.f, 180.f);
                    else if (dimension == 6 || dimension == 7)
                        // Keep the target inside the unchanged 40-degree hard
                        // bound despite float roundoff as the torso moves.
                        parameters[dimension] = std::clamp(parameters[dimension], -39.9f, 39.9f);
                    const auto candidate = assess(parameters);
                    if (candidate.ready || candidate.cost + 1e-8 < quality.cost)
                    {
                        pose.refinementParameters = parameters;
                        pose.refinementSweepImproved = true;
                        quality = candidate;
                    }
                    found = quality.ready;
                    if (found)
                    {
                        pose.refining = false;
                        pose.refinementFinished = true;
                        break;
                    }
                    if (++pose.refinementCoordinate == 16)
                    {
                        pose.refinementCoordinate = 0;
                        if (!pose.refinementSweepImproved || ++pose.refinementIteration >= 20)
                        {
                            ++pose.refinementStep;
                            pose.refinementIteration = 0;
                        }
                        pose.refinementSweepImproved = false;
                    }
                }
                desired = quality.pose;
                witnessIndex = pose.cachedWitness;
                pose.cachedNatural = found;
                if (found || pose.refinementFinished)
                {
                    pose.cachedTurn = refinedTurn(pose.refinementParameters);
                    pose.cachedOffset = pose.refinementParameters[3];
                    pose.cachedSwivel = { pose.refinementParameters[4], pose.refinementParameters[5] };
                    pose.cachedPronation = { pose.refinementParameters[6], pose.refinementParameters[7] };
                    pose.cachedExplicitPronation = true;
                    pose.refinementFinished = true;
                    pose.cachedTarget = targetActor;
                    pose.cachedDirection = directionActor;
                    for (unsigned side = 0; side < 2; ++side)
                        pose.cachedMetricToActor[side] = rigs[side].metricToActor;
                }
            }
            else
                found = false;
            audit.goalNatural = desired.natural;
            audit.goalWorking = found;
            // The pair goal reports the more constrained arm, before any safe
            // authored fallback replaces it. Published left/right metrics below
            // continue to describe the actual rendered pose separately.
            const auto& goal = desired.arms[0].metrics.score() >= desired.arms[1].metrics.score()
                ? desired.arms[0].metrics
                : desired.arms[1].metrics;
            audit.goalReachClamp = std::max(desired.arms[0].metrics.reach, desired.arms[1].metrics.reach);
            audit.goalWristSwingDegrees = goal.wristSwing;
            audit.goalWristTwistDegrees = goal.wristTwist;
            audit.goalPalmFlexDegrees = goal.palmFlex;
            audit.goalPalmSideDegrees = goal.palmSide;
            audit.goalForearmTwistDegrees = goal.foreTwist;
            audit.goalUpperSwingDegrees = goal.upperSwing;
            audit.goalUpperTwistDegrees = goal.upperTwist;
            audit.goalSignedElbowFlexDegrees = goal.signedElbowFlex;
            audit.goalElbowHingeErrorDegrees = goal.elbowHingeError;
            audit.goalBodyClearance = std::min(desired.arms[0].metrics.clearance, desired.arms[1].metrics.clearance);
            audit.goalInwardError = std::max(desired.arms[0].metrics.inward, desired.arms[1].metrics.inward);
            audit.goalWrist = desired.arms[0].world[2].getTrans() * rigs[0].metricToWorld;
            const auto goalItemWorld = rigs[0].itemToHand * desired.arms[0].world[2] * rigs[0].metricToWorld;
            audit.goalContact = pose.witnesses[witnessIndex] * goalItemWorld;
            const float goalOffset = pose.refining ? pose.refinementParameters[3] : pose.cachedOffset;
            audit.goalContactError = (audit.goalContact - (pose.target - pose.incoming * goalOffset)).length();
            if (!found)
            {
                bool retained = false;
                if (pose.published)
                {
                    desired = solvePaired(
                        rigs, pose.currentItemActor, handFromItem, pose.currentSwivel, pose.currentPronation);
                    ++audit.searchCandidates;
                    retained = desired.natural;
                }
                if (!retained)
                {
                    desired = solvePaired(rigs, pose.referenceItemActor, handFromItem, { 0.f, 0.f }, {}, true,
                        wideStyle.active ? &audit.searchCandidates : nullptr);
                    if (!wideStyle.active)
                        ++audit.searchCandidates;
                }
                audit.goalFromSafe = true;
                audit.jointLimited = true;
                if (!desired.natural)
                {
                    pose.published = false;
                    return;
                }
            }
            const auto targetItem = desired.itemActor;
            const float angular = rotationDistanceDegrees(
                rotationOnly(pose.currentItemActor).getRotate(), rotationOnly(targetItem).getRotate());
            const float travel = osg::Vec3f(targetItem.getTrans() - pose.currentItemActor.getTrans()).length();
            float amount = 1.f;
            if (angular > .001f)
                amount = std::min(amount, 240.f * duration / angular);
            if (travel > .001f)
                amount = std::min(amount, 180.f * duration / travel);
            amount *= audit.weight;
            bool accepted = false;
            const auto full = desired;
            PairedSolution previous;
            auto previousRotations = pose.publishedRotations;
            if (useElbowContinuation)
            {
                previous
                    = solvePaired(rigs, pose.currentItemActor, handFromItem, pose.currentSwivel, pose.currentPronation);
                ++audit.searchCandidates;
                if (!pose.published)
                    for (unsigned side = 0; side < 2; ++side)
                        for (unsigned joint = 0; joint < 3; ++joint)
                            previousRotations[side][joint] = previous.arms[side].local[joint].getRotate();
            }
            const auto tryStep = [&](float fraction, PairedSolution& candidate) {
                if (useElbowContinuation)
                    return solvePairedStep(rigs, interpolateItem(pose.currentItemActor, targetItem, fraction),
                        handFromItem, wideStyle, previous, previousRotations, duration, audit.searchCandidates,
                        candidate);
                std::array<float, 2> swivel, pronation;
                for (unsigned side = 0; side < 2; ++side)
                {
                    swivel[side] = pose.currentSwivel[side] + (full.swivel[side] - pose.currentSwivel[side]) * fraction;
                    pronation[side]
                        = pose.currentPronation[side] + (full.pronation[side] - pose.currentPronation[side]) * fraction;
                }
                candidate = solvePaired(rigs, interpolateItem(pose.currentItemActor, targetItem, fraction),
                    handFromItem, swivel, pronation);
                ++audit.searchCandidates;
                bool candidateSpeedOk = true;
                if (pose.published)
                    for (unsigned side = 0; side < 2; ++side)
                        for (unsigned joint = 0; joint < 3; ++joint)
                            candidateSpeedOk = candidateSpeedOk
                                && rotationDistanceDegrees(pose.publishedRotations[side][joint],
                                       candidate.arms[side].local[joint].getRotate())
                                    <= (joint ? 360.f : 240.f) * duration + .05f;
                return candidate.natural && candidateSpeedOk;
            };
            if (useElbowContinuation)
            {
                // Six bounded continuation trials cost at most 300 pair
                // evaluations. They use exactly the existing speed limits.
                float lower = 0.f, upper = amount;
                for (unsigned trial = 0; trial < 6; ++trial)
                {
                    const float fraction = trial ? (lower + upper) * .5f : upper;
                    PairedSolution candidate;
                    if (tryStep(fraction, candidate))
                    {
                        desired = candidate;
                        accepted = true;
                        lower = fraction;
                        if (trial == 0)
                            break;
                    }
                    else
                        upper = fraction;
                }
                amount = lower;
            }
            else
            {
                float invalidAmount = amount;
                for (unsigned attempt = 0; attempt < 12; ++attempt)
                {
                    PairedSolution candidate;
                    if (tryStep(amount, candidate))
                    {
                        desired = candidate;
                        accepted = true;
                        // Halving locates a safe bracket. Refine within it instead
                        // of leaving most of the unchanged angular speed budget
                        // unused. Every published candidate passes both arm limits.
                        if (attempt > 0)
                        {
                            float lower = amount, upper = invalidAmount;
                            for (unsigned refinement = 0; refinement < 5; ++refinement)
                            {
                                const float middle = (lower + upper) * .5f;
                                PairedSolution refined;
                                if (tryStep(middle, refined))
                                {
                                    lower = middle;
                                    desired = refined;
                                }
                                else
                                    upper = middle;
                            }
                            amount = lower;
                        }
                        break;
                    }
                    invalidAmount = amount;
                    amount *= .5f;
                }
            }
            audit.pathAccepted = accepted;
            audit.pathFraction = accepted ? amount : 0.f;
            if (!accepted)
            {
                desired
                    = solvePaired(rigs, pose.currentItemActor, handFromItem, pose.currentSwivel, pose.currentPronation);
                ++audit.searchCandidates;
                audit.jointLimited = true;
                if (!desired.natural)
                {
                    pose.published = false;
                    return;
                }
            }
            pose.currentSwivel = desired.swivel;
            pose.currentPronation = desired.pronation;
        }
        if (pose.cache)
            interceptWorld = pose.target - pose.incoming * pose.cachedOffset;
        pose.snapshot = false;
        for (unsigned side = 0; side < 2; ++side)
            for (unsigned joint = 0; joint < 3; ++joint)
            {
                pose.source.emplace_back(original[side].bones[joint], original[side].source[joint]);
                original[side].bones[joint]->setMatrix(desired.arms[side].local[joint]);
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
        if (!actualItem)
            return;
        const auto renderActor = nodeWorld(mPtr.getRefData().getBaseNode()).value_or(*actorWorld);
        pose.snapshot = true;
        pose.snapshotItemWorld = *actualItem;
        pose.snapshotItemActor = *actualItem * osg::Matrixf::inverse(renderActor);
        pose.snapshotFrame = audit.frame;
        pose.currentItemActor = pose.snapshotItemActor;
        pose.lastSourceItemActor = sourceItemActor;
        pose.published = true;
        audit.active = true;
        audit.interceptPoint = interceptWorld;
        audit.requestedTarget = pose.target;
        const auto witness = pose.witnesses[std::min(witnessIndex, unsigned(pose.witnesses.size() - 1))];
        audit.materialPoint = witness;
        audit.itemContact = witness * *actualItem;
        audit.contactError = (audit.itemContact - interceptWorld).length();
        audit.itemAxis = osg::Matrixf::transform3x3(pose.axis, *actualItem);
        audit.itemAxis.normalize();
        audit.requestedAxis = audit.itemAxis;
        audit.renderActorOrigin = renderActor.getTrans();
        audit.renderActorRight = osg::Matrixf::transform3x3(osg::Vec3f(1, 0, 0), renderActor);
        audit.renderActorForward = osg::Matrixf::transform3x3(osg::Vec3f(0, 1, 0), renderActor);
        audit.renderActorUp = osg::Matrixf::transform3x3(osg::Vec3f(0, 0, 1), renderActor);
        audit.guardPelvis = pelvis * *actorWorld;
        audit.guardTorso = spine * *actorWorld;
        std::array<GuardJointMetrics, 2> metrics;
        std::array<std::array<osg::Matrixf, 3>, 2> actualWorld;
        for (unsigned side = 0; side < 2; ++side)
        {
            std::array<osg::Matrixf, 3> local, metric;
            for (unsigned joint = 0; joint < 3; ++joint)
            {
                local[joint] = original[side].bones[joint]->getMatrix();
                const auto world = nodeWorld(original[side].bones[joint]);
                if (!world)
                    return;
                actualWorld[side][joint] = *world;
                metric[joint] = *world * osg::Matrixf::inverse(rigs[side].metricToWorld);
                pose.publishedRotations[side][joint] = local[joint].getRotate();
                const float ortho = orthogonalityError(local[joint]),
                            translation
                    = osg::Vec3f(local[joint].getTrans() - original[side].source[joint].getTrans()).length();
                if (side)
                {
                    audit.leftMaxLocalOrthoError = std::max(audit.leftMaxLocalOrthoError, ortho);
                    audit.leftMaxLocalTranslationError = std::max(audit.leftMaxLocalTranslationError, translation);
                }
                else
                {
                    audit.maxLocalOrthoError = std::max(audit.maxLocalOrthoError, ortho);
                    audit.maxLocalTranslationError = std::max(audit.maxLocalTranslationError, translation);
                    if (joint == 0)
                        audit.upperLocalOrthoError = ortho;
                    else if (joint == 1)
                        audit.foreLocalOrthoError = ortho;
                    else
                        audit.handLocalOrthoError = ortho;
                }
                audit.overlayCorrectionDegrees = std::max(audit.overlayCorrectionDegrees,
                    rotationDistanceDegrees(original[side].source[joint].getRotate(), local[joint].getRotate()));
            }
            metrics[side] = rigs[side].measure(local, metric, 0.f);
        }
        audit.sourceWrist = original[0].world[2].getTrans();
        audit.sourceUpperLength
            = osg::Vec3f(original[0].world[1].getTrans() - original[0].world[0].getTrans()).length();
        audit.sourceForeLength = osg::Vec3f(original[0].world[2].getTrans() - original[0].world[1].getTrans()).length();
        audit.shoulder = actualWorld[0][0].getTrans();
        audit.elbow = actualWorld[0][1].getTrans();
        audit.wrist = actualWorld[0][2].getTrans();
        audit.shoulderError = (audit.shoulder - osg::Vec3f(original[0].world[0].getTrans())).length();
        audit.finalUpperLength = (audit.elbow - audit.shoulder).length();
        audit.finalForeLength = (audit.wrist - audit.elbow).length();
        audit.leftSourceWrist = original[1].world[2].getTrans();
        audit.leftShoulder = actualWorld[1][0].getTrans();
        audit.leftElbow = actualWorld[1][1].getTrans();
        audit.leftWrist = actualWorld[1][2].getTrans();
        audit.leftShoulderError = (audit.leftShoulder - osg::Vec3f(original[1].world[0].getTrans())).length();
        audit.gripError = matrixError(*actualItem * osg::Matrixf::inverse(actualWorld[0][2]), rigs[0].itemToHand);
        audit.leftGripError = matrixError(actualWorld[1][2] * osg::Matrixf::inverse(*actualItem), handFromItem[1]);
        audit.sourceUpperLengthUnscaled = rigs[0].upperLength;
        audit.sourceForeLengthUnscaled = rigs[0].foreLength;
        audit.leftSourceUpperLengthUnscaled = rigs[1].upperLength;
        audit.leftSourceForeLengthUnscaled = rigs[1].foreLength;
        audit.skeletonShoulder = audit.shoulder * actorInverse;
        audit.skeletonElbow = audit.elbow * actorInverse;
        audit.skeletonWrist = audit.wrist * actorInverse;
        audit.finalUpperLengthUnscaled = (audit.skeletonElbow - audit.skeletonShoulder).length();
        audit.finalForeLengthUnscaled = (audit.skeletonWrist - audit.skeletonElbow).length();
        audit.leftFinalUpperLengthUnscaled
            = ((audit.leftElbow * actorInverse) - (audit.leftShoulder * actorInverse)).length();
        audit.leftFinalForeLengthUnscaled
            = ((audit.leftWrist * actorInverse) - (audit.leftElbow * actorInverse)).length();
        const auto& r = metrics[0];
        const auto& l = metrics[1];
        audit.wristSwingDegrees = r.wristSwing;
        audit.wristTwistDegrees = r.wristTwist;
        audit.palmFlexDegrees = r.palmFlex;
        audit.palmSideDegrees = r.palmSide;
        audit.forearmTwistDegrees = r.foreTwist;
        audit.upperSwingDegrees = r.upperSwing;
        audit.upperTwistDegrees = r.upperTwist;
        audit.signedElbowFlexDegrees = r.signedElbowFlex;
        audit.elbowHingeErrorDegrees = r.elbowHingeError;
        audit.bodyClearance = r.clearance;
        audit.elbowFlexDegrees = r.elbowFlex;
        audit.palmForward = r.palmForward;
        audit.palmLateral = r.palmLateral;
        audit.palmNormal = r.palmNormal;
        audit.totalPronationDegrees = r.foreTwist + r.wristTwist;
        audit.leftWristSwingDegrees = l.wristSwing;
        audit.leftWristTwistDegrees = l.wristTwist;
        audit.leftPalmFlexDegrees = l.palmFlex;
        audit.leftPalmSideDegrees = l.palmSide;
        audit.leftForearmTwistDegrees = l.foreTwist;
        audit.leftUpperSwingDegrees = l.upperSwing;
        audit.leftUpperTwistDegrees = l.upperTwist;
        audit.leftSignedElbowFlexDegrees = l.signedElbowFlex;
        audit.leftElbowHingeErrorDegrees = l.elbowHingeError;
        audit.leftBodyClearance = l.clearance;
        audit.leftNatural = l.natural();
        audit.jointLimited
            = audit.jointLimited || (pose.enabled && (!r.natural() || !l.natural() || audit.leftGripError > .02f));
        audit.clamped = audit.jointLimited;
        audit.fitted = pose.enabled && !audit.jointLimited && audit.contactError < 1.f && audit.leftGripError < .02f;
    }

    bool NpcAnimation::hasNpcTwoHandGuardMeshForCombat() const
    {
        return mNpcTwoHandGuardPose && mNpcTwoHandGuardPose->snapshot && mShowWeapons && mObjectParts[ESM::PRT_Weapon]
            && mObjectParts[ESM::PRT_Weapon]->getNode().get() == mNpcTwoHandGuardPose->item.get();
    }

    std::optional<NpcAnimation::NpcGuardMeshTrace> NpcAnimation::traceNpcTwoHandGuardMeshForAudit(
        const osg::Vec3f& origin, const osg::Vec3f& end, bool combat) const
    {
        if (!mNpcTwoHandGuardPose || !mNpcTwoHandGuardPose->snapshot || !finite(origin) || !finite(end))
            return {};
        const auto& pose = *mNpcTwoHandGuardPose;
        if (pose.snapshotFrame != mNpcGuardGeometry.frame)
            return {};
        auto itemWorld = pose.snapshotItemWorld;
        if (combat)
        {
            if (!hasNpcTwoHandGuardMeshForCombat())
                return {};
            const auto actor = nodeWorld(mPtr.getRefData().getBaseNode());
            if (!actor)
                return {};
            itemWorld = pose.snapshotItemActor * *actor;
        }
        float first = std::numeric_limits<float>::infinity();
        SegmentMeshGap nearest;
        for (const auto& local : pose.triangles)
        {
            std::array<osg::Vec3f, 3> tri;
            for (unsigned i = 0; i < 3; ++i)
                tri[i] = local[i] * itemWorld;
            if (((tri[1] - tri[0]) ^ (tri[2] - tri[0])).length2() < 1e-12f)
                continue;
            if (const auto hit = segmentTriangleHit(origin, end, tri))
                first = std::min(first, *hit);
            if (std::isfinite(first))
                continue;
            for (const auto& p : { origin, end })
            {
                const auto q = closestPoint(p, tri);
                const double d = (osg::Vec3d(p) - osg::Vec3d(q)).length2();
                if (d < nearest.squared)
                    nearest = { d, q };
            }
            for (unsigned e = 0; e < 3; ++e)
            {
                const auto d = segmentEdgeGap(origin, end, tri[e], tri[(e + 1) % 3]);
                if (d.squared < nearest.squared)
                    nearest = d;
            }
        }
        if (std::isfinite(first))
            return NpcGuardMeshTrace{ pose.snapshotFrame, true, 0.f, origin + (end - origin) * first };
        if (std::isfinite(nearest.squared))
            return NpcGuardMeshTrace{ pose.snapshotFrame, false, float(std::sqrt(nearest.squared)), nearest.point };
        return {};
    }
}
