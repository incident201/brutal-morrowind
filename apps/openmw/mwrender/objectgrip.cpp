#include "npcanimation.hpp"

#include <osg/FrameStamp>

#include <components/sceneutil/skeleton.hpp>

#include "../mwphysics/physicssystem.hpp"

#include "griparms.hpp"

namespace MWRender
{
    namespace
    {
        struct Palm
        {
            osg::Vec3f center;
            osg::Vec3f forward;
            osg::Vec3f normal;
            float width;
        };

        Palm palmGeometry(const Animation::NodeMap& nodes, const char* prefix, bool shaft)
        {
            const auto finger = [&](int i) {
                const auto it = nodes.find(std::string(prefix) + std::to_string(i));
                return it == nodes.end() ? osg::Vec3f() : osg::Vec3f(it->second->getMatrix().getTrans());
            };
            const osg::Vec3f knuckles = (finger(1) + finger(2) + finger(3) + finger(4)) * .25f;
            osg::Vec3f forward = knuckles;
            if (forward.normalize() < .001f)
                forward.set(1, 0, 0);
            osg::Vec3f normal(0, 1, 0);
            normal -= forward * (normal * forward);
            normal.normalize();
            // Rest a broad object in the palm. A narrow handle crosses the
            // base of the fingers so the fingers can close over its far side.
            return { knuckles * (shaft ? .99f : .80f) + normal * .55f, forward, normal,
                std::max(2.f, (finger(4) - finger(1)).length()) };
        }

        osg::Vec3f direction(const osg::Vec3f& axis, const osg::Matrixf& matrix)
        {
            auto result = osg::Matrixf::transform3x3(axis, matrix);
            result.normalize();
            return result;
        }

        struct ShaftClearance
        {
            osg::Vec3f center, axis;
            float radius = 0.f;

            float overlap(const osg::Vec3f& start, const osg::Vec3f& along, float length) const
            {
                const auto delta = start - center;
                const auto radial = delta - axis * (delta * axis);
                const auto travel = along - axis * (along * axis);
                const float closest = std::clamp(-float(radial * travel) / std::max(.001f, travel.length2()), 0.f, length);
                return std::max(0.f, radius - (radial + travel * closest).length());
            }
        };

        template <class Apply>
        void curlGripFingers(const Animation::NodeMap& nodes, const char* prefix, const osg::Matrixf& hand,
            const Palm& palm, float curl, const ShaftClearance& shaft, const Apply& apply)
        {
            const auto forward = direction(palm.forward, hand);
            const auto normal = direction(palm.normal, hand);
            const auto lateral = forward ^ normal;
            const std::array<float, 3> open{ 9.f, 20.f, 31.f }, closed{ 63.f, 121.f, 172.f };
            for (unsigned finger = 1; finger <= 4; ++finger)
            {
                const std::string root = std::string(prefix) + std::to_string(finger);
                osg::Matrixf parent = hand;
                for (unsigned joint = 0; joint < 3; ++joint)
                {
                    const auto bone = nodes.find(joint ? root + std::to_string(joint) : root);
                    if (bone == nodes.end())
                        break;
                    const osg::Matrixf local = bone->second->getMatrix();
                    const osg::Matrixf world = local * parent;
                    const auto child = nodes.find(root + std::to_string(joint + 1));
                    const osg::Vec3f segment = child != nodes.end()
                        ? osg::Vec3f(child->second->getMatrix().getTrans()) : osg::Vec3f(1, 0, 0);
                    const auto axis = direction(segment, world);
                    const float current = std::atan2(axis * normal, axis * forward);
                    const float target = osg::DegreesToRadians(open[joint] + (closed[joint] - open[joint]) * curl);
                    const auto ideal = std::clamp(std::remainder(target - current, float(2 * osg::PI)), -.9f, .9f);
                    float delta = ideal;
                    if (shaft.radius > 0.f)
                    {
                        const float length = osg::Matrixf::transform3x3(segment, world).length() + .55f;
                        float bestScore = std::numeric_limits<float>::infinity();
                        for (int step = -6; step <= 6; ++step)
                        {
                            const float trial = std::clamp(ideal + float(step) * .12f, -.9f, .9f);
                            const auto along = osg::Quat(trial, lateral) * axis;
                            const float score = std::abs(trial - ideal)
                                + 5.f * shaft.overlap(world.getTrans(), along, length);
                            if (score < bestScore)
                            {
                                bestScore = score;
                                delta = trial;
                            }
                        }
                    }
                    osg::Matrixf bent = world * osg::Matrixf::rotate(delta, lateral);
                    bent.setTrans(world.getTrans());
                    osg::Matrixf result = bent * osg::Matrixf::inverse(parent);
                    result.setTrans(local.getTrans());
                    apply(bone->second.get(), result);
                    parent = bent;
                }
            }
        }

        template <class Apply>
        void closeGripThumb(const Animation::NodeMap& nodes, const char* prefix, const osg::Matrixf& hand,
            const Palm& palm, float thickness, const ShaftClearance& shaft, const Apply& apply)
        {
            const auto normal = direction(palm.normal, hand);
            const auto along = direction(palm.forward, hand);
            const auto axis = along ^ normal;
            const auto thumb = nodes.find(std::string(prefix) + "0");
            if (thumb == nodes.end())
                return;
            const auto side = (osg::Vec3f(thumb->second->getMatrix().getTrans()) - palm.center)
                    * (palm.forward ^ palm.normal) > 0.f ? 1.f : -1.f;
            const osg::Vec3f target = palm.center * hand + normal * (thickness * 1.56f + .45f)
                + along * (thickness * .74f) + axis * (side * 1.45f);
            osg::Matrixf parent = hand;
            const std::array<std::string, 3> names{ std::string(prefix) + "0",
                std::string(prefix) + "01", std::string(prefix) + "02" };
            for (unsigned joint = 0; joint < names.size(); ++joint)
            {
                const auto bone = nodes.find(names[joint]);
                if (bone == nodes.end())
                    break;
                const osg::Matrixf local = bone->second->getMatrix();
                const osg::Matrixf world = local * parent;
                auto desired = target - world.getTrans();
                if (desired.normalize() < .001f)
                    break;
                auto axisTurn = direction(osg::Vec3f(1, 0, 0), world);
                osg::Quat alignment, turn;
                alignment.makeRotate(axisTurn, desired);
                const float angle = std::acos(std::clamp(axisTurn * desired, -1.f, 1.f));
                const float ideal = std::min(1.f, .65f / std::max(.001f, angle));
                const auto child = joint + 1 < names.size() ? nodes.find(names[joint + 1]) : nodes.end();
                const osg::Vec3f segment = child != nodes.end()
                    ? osg::Vec3f(child->second->getMatrix().getTrans()) : osg::Vec3f(1, 0, 0);
                const float length = osg::Matrixf::transform3x3(segment, world).length() + .55f;
                float bestScore = std::numeric_limits<float>::infinity(), weight = ideal;
                for (unsigned step = 0; step <= 6; ++step)
                {
                    const float trial = ideal * (1.f - float(step) / 6.f);
                    osg::Quat sample;
                    sample.slerp(trial, osg::Quat(), alignment);
                    const float score = (ideal - trial) * angle
                        + 5.f * shaft.overlap(world.getTrans(), sample * axisTurn, length);
                    if (score < bestScore)
                    {
                        bestScore = score;
                        weight = trial;
                    }
                }
                turn.slerp(weight, osg::Quat(), alignment);
                osg::Matrixf bent = world * osg::Matrixf::rotate(turn);
                bent.setTrans(world.getTrans());
                osg::Matrixf result = bent * osg::Matrixf::inverse(parent);
                result.setTrans(local.getTrans());
                apply(bone->second.get(), result);
                parent = bent;
            }
        }

        osg::Quat alignObject(const MWPhysics::HeldObjectGrip& grip, osg::Vec3f length, osg::Vec3f depth)
        {
            length.normalize();
            depth -= length * (length * depth);
            depth.normalize();
            std::array<osg::Vec3f, 3> axes;
            const int a = grip.lengthAxis, b = grip.depthAxis, c = 3 - a - b;
            axes[a] = length;
            axes[b] = depth;
            axes[c] = (a + 1) % 3 == b ? length ^ depth : depth ^ length;
            return osg::Matrixf(axes[0].x(), axes[0].y(), axes[0].z(), 0.f,
                axes[1].x(), axes[1].y(), axes[1].z(), 0.f,
                axes[2].x(), axes[2].y(), axes[2].z(), 0.f, 0.f, 0.f, 0.f, 1.f)
                .getRotate();
        }

        osg::Matrixf limitGripWrist(osg::Matrixf hand, const osg::Matrixf& forearm,
            const osg::Matrixf& authoredHand, const Palm& palm)
        {
            const osg::Vec3f wrist = hand.getTrans();
            auto fore = wrist - osg::Vec3f(forearm.getTrans());
            if (fore.normalize() < .001f)
                return hand;
            const auto along = direction(palm.forward, hand);
            const auto normal = direction(palm.normal, hand);
            const auto side = along ^ normal;
            const float flex = std::clamp(std::atan2(fore * normal, fore * along), -.63f, .63f);
            const float sideways = std::clamp(std::asin(std::clamp(fore * side, -1.f, 1.f)), -.36f, .36f);
            const auto feasible = (along * std::cos(flex) + normal * std::sin(flex)) * std::cos(sideways)
                + side * std::sin(sideways);
            osg::Quat swing;
            swing.makeRotate(feasible, fore);
            hand *= osg::Matrixf::rotate(swing);
            auto from = direction(palm.normal, authoredHand);
            auto to = direction(palm.normal, hand);
            from -= fore * (fore * from);
            to -= fore * (fore * to);
            if (from.normalize() > .001f && to.normalize() > .001f)
            {
                const float twist = std::atan2(fore * (from ^ to), from * to);
                hand *= osg::Matrixf::rotate(std::clamp(twist, -.76f, .76f) - twist, fore);
            }
            hand.setTrans(wrist);
            return hand;
        }
    }

    bool NpcAnimation::fitHeldObjectGrip(const MWPhysics::HeldObjectGrip& grip, const osg::Vec3f& eye,
        const osg::Quat& view, const std::function<float(const osg::Vec3f&, const osg::Vec3f&, const osg::Quat&)>& sweep,
        const osg::FrameStamp& frameStamp, osg::Vec3f& position, osg::Quat& rotation)
    {
        if (mViewMode != VM_FirstPerson || !mObjectRoot)
            return false;

        const bool throwing = isPlaying(grip.dual ? "brutal_throw_dual" : "brutal_throw_solo");
        const std::string grab = grip.dual ? "brutal_grab_dual" : "brutal_grab_solo";
        const float raise = getTextKeyTime(grab + ": raise start");
        const float raised = getTextKeyTime(grab + ": raise stop");
        const float time = getCurrentTime(grab);
        const float raiseWeight = raise >= 0.f && raised > raise && time >= raise && time <= raised + .001f
            ? std::clamp((time - raise) / (raised - raise), 0.f, 1.f) : 0.f;
        const float holdWeight = throwing ? 0.f : 1.f - raiseWeight * raiseWeight * (3.f - 2.f * raiseWeight);
        const auto& nodes = getNodeMap();
        const bool wrapsShaft = grip.narrowEnd || grip.shaft;
        const std::array<Palm, 2> palms{
            palmGeometry(nodes, "Bip01 R Finger", wrapsShaft), palmGeometry(nodes, "Bip01 L Finger", wrapsShaft) };
        const auto rightHand = getBoneWorldMatrix("Bip01 R Hand");
        const auto leftHand = grip.dual ? getBoneWorldMatrix("Bip01 L Hand") : std::optional<osg::Matrixf>();
        if (!rightHand || (grip.dual && !leftHand))
            return false;

        std::array<osg::Matrixf, 2> original{ *rightHand, leftHand.value_or(osg::Matrixf()) }, goals = original;
        const osg::Vec3f up = view * osg::Vec3f(0, 0, 1);
        const osg::Vec3f ahead = view * osg::Vec3f(0, 1, 0);
        const osg::Vec3f screenRight = view * osg::Vec3f(1, 0, 0);
        osg::Vec3f lateral, depth, center;
        const auto soloPose = [&](const osg::Matrixf& hand) {
            const auto normal = direction(palms[0].normal, hand);
            const auto along = direction(palms[0].forward, hand);
            osg::Vec3f length = along;
            if (grip.narrowEnd)
            {
                length = along ^ normal;
                if (grip.contact[grip.lengthAxis] < grip.center[grip.lengthAxis])
                    length = -length;
            }
            rotation = alignObject(grip, length, normal);
            position = palms[0].center * hand + normal * (grip.narrowEnd ? grip.thickness + .35f : .35f)
                - rotation * grip.contact;
        };

        if (grip.dual)
        {
            const auto right = palms[0].center * original[0], left = palms[1].center * original[1];
            lateral = right - left;
            if (lateral.normalize() < .001f)
                lateral = screenRight;
            depth = ahead;
            if (std::abs(lateral * depth) > .94f)
                depth = up;
            rotation = alignObject(grip, lateral, depth);
            const int widthAxis = 3 - grip.lengthAxis - grip.depthAxis;
            center = (right + left) * .5f;
            center += (ahead * 12.f - up * (7.f + grip.half[widthAxis] * .24f)) * holdWeight;
            position = center + depth * (grip.shaft ? grip.thickness * .80f : 0.f) - rotation * grip.contact;
        }
        else
        {
            goals[0].setTrans(original[0].getTrans()
                + (ahead * 15.f + screenRight * 10.f + up * (grip.narrowEnd ? 1.f : -8.f)) * holdWeight);
            if (!grip.narrowEnd && holdWeight > 0.f)
            {
                const auto normal = direction(palms[0].normal, original[0]);
                osg::Quat towardUp, turn;
                towardUp.makeRotate(normal, up);
                turn.slerp(.65f * holdWeight, osg::Quat(), towardUp);
                const osg::Vec3f wrist = goals[0].getTrans();
                goals[0] = original[0] * osg::Matrixf::rotate(turn);
                goals[0].setTrans(wrist);
            }
            soloPose(goals[0]);
        }

        const auto retreat = [&] {
            const auto offset = rotation * grip.center;
            const auto from = eye - offset;
            const float fraction = sweep(from, position, rotation);
            const auto delta = from - position;
            return delta * std::min(1.f, 1.f - fraction + (fraction < 1.f ? .65f / std::max(delta.length(), 1.f) : 0.f));
        };
        const auto wallCorrection = retreat();
        position += wallCorrection;
        if (grip.dual)
        {
            center += wallCorrection;
            for (unsigned i = 0; i < 2; ++i)
            {
                const auto out = lateral * (i == 0 ? 1.f : -1.f);
                osg::Quat align, turn;
                align.makeRotate(direction(palms[i].normal, original[i]), grip.shaft ? depth : -out);
                turn.slerp(.85f, osg::Quat(), align);
                goals[i] = original[i] * osg::Matrixf::rotate(turn);
                goals[i].setTrans(center + out * grip.span
                    - osg::Matrixf::transform3x3(palms[i].center, goals[i]));
            }
        }
        else
            goals[0].setTrans(goals[0].getTrans() + wallCorrection);

        const unsigned count = grip.dual ? 2 : 1;
        std::array<osg::Matrixf, 2> corrections;
        std::array<GripArm, 2> arms;
        std::array<osg::Matrixf, 2> hands = goals;
        GripArmLimits limits;
        limits.minElbowBend = .16f + holdWeight * .36f;
        limits.maxElbowBend = 2.55f;
        limits.elbows[0] = { screenRight * .55f - up, .28f * holdWeight };
        limits.elbows[1] = { screenRight * -.55f - up, .28f * holdWeight };
        std::array<osg::Vec3f, 2> contact;
        for (unsigned i = 0; i < count; ++i)
            contact[i] = palms[i].center * goals[i];
        float clamped = 0.f;
        for (unsigned pass = 0; pass < 2; ++pass)
        {
            for (unsigned i = 0; i < count; ++i)
            {
                const auto wrist = contact[i] - osg::Matrixf::transform3x3(palms[i].center, hands[i]);
                corrections[i] = osg::Matrixf::translate(wrist - osg::Vec3f(original[i].getTrans()));
            }
            if (!solveGripArms(nodes, osg::Matrixf(), [](osg::MatrixTransform* bone) { return bone->getMatrix(); },
                    arms, clamped, limits, &corrections, count))
                return false;
            for (unsigned i = 0; i < count; ++i)
            {
                auto& arm = arms[i];
                const osg::Matrixf forearm = arm.result[1] * arm.result[0] * arm.parent;
                const osg::Matrixf authoredHand = arm.source[2] * forearm;
                auto desired = goals[i];
                desired.setTrans(authoredHand.getTrans());
                hands[i] = limitGripWrist(desired, forearm, authoredHand, palms[i]);
                arm.result[2] = hands[i] * osg::Matrixf::inverse(forearm);
                arm.result[2].setTrans(arm.source[2].getTrans());
            }
        }

        const auto setPose = [&](osg::MatrixTransform* bone, const osg::Matrixf& pose) {
            mHeldGripSourcePose.emplace_back(bone, bone->getMatrix());
            bone->setMatrix(pose);
        };
        std::array<osg::Vec3f, 2> solved;
        for (unsigned i = 0; i < count; ++i)
        {
            for (unsigned joint = 0; joint < arms[i].bones.size(); ++joint)
                setPose(arms[i].bones[joint], arms[i].result[joint]);
            const osg::Matrixf hand = osg::computeLocalToWorld(arms[i].bones[2]->getParentalNodePaths().front());
            solved[i] = palms[i].center * hand;
            const float relativeThickness = grip.thickness / palms[i].width;
            const float curl = grip.dual && !grip.shaft ? .05f : wrapsShaft
                ? std::clamp(1.37f - relativeThickness * .65f, .60f, 1.f)
                : std::clamp(.80f - .20f * relativeThickness, .27f, .64f);
            const char* prefix = i == 0 ? "Bip01 R Finger" : "Bip01 L Finger";
            ShaftClearance shaft;
            if (wrapsShaft)
            {
                const auto normal = direction(palms[i].normal, hand);
                shaft.center = solved[i] + (grip.dual ? depth * (grip.thickness * .80f) : normal * (grip.thickness + .35f));
                shaft.axis = grip.dual ? lateral : direction(palms[i].forward, hand) ^ normal;
                shaft.radius = grip.thickness + .43f;
            }
            curlGripFingers(nodes, prefix, hand, palms[i], curl, shaft, setPose);
            if (wrapsShaft)
                closeGripThumb(nodes, prefix, hand, palms[i], grip.thickness, shaft, setPose);
        }
        if (grip.dual)
        {
            auto span = solved[0] - solved[1];
            if (span.normalize() > .001f)
                rotation = alignObject(grip, span, depth);
            position = (solved[0] + solved[1]) * .5f
                + depth * (grip.shaft ? grip.thickness * .80f : 0.f) - rotation * grip.contact;
        }
        else
            soloPose(hands[0]);

        if (mSkeleton)
            mSkeleton->markDirty();
        osg::NodeVisitor bounds(osg::NodeVisitor::TRAVERSE_ALL_CHILDREN);
        bounds.setVisitorType(osg::NodeVisitor::UPDATE_VISITOR);
        bounds.setTraversalNumber(frameStamp.getFrameNumber());
        bounds.setFrameStamp(new osg::FrameStamp(frameStamp));
        bounds.setNodeMaskOverride(~0u);
        mObjectRoot->accept(bounds);
        return true;
    }
}
