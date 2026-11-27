#ifndef OPENMW_MWRENDER_GRIPARMS_H
#define OPENMW_MWRENDER_GRIPARMS_H

#include <algorithm>
#include <array>
#include <cmath>
#include <functional>
#include <string>

#include <osg/Matrixf>
#include <osg/MatrixTransform>
#include <osg/Quat>

#include "animation.hpp"

namespace MWRender
{
    // One first-person arm: UpperArm, Forearm, Hand.
    struct GripArm
    {
        std::array<osg::MatrixTransform*, 3> bones;
        std::array<osg::Matrixf, 3> source, world, result;
        osg::Matrixf parent;
        osg::MatrixTransform* clavicle = nullptr;
        osg::Matrixf clavicleSource, clavicleResult;
    };

    struct GripArmLimits
    {
        struct Elbow
        {
            osg::Vec3f guide;
            float turn = 0.f;
        };
        float clavicleTurn = 0.f;
        float minElbowBend = 0.f;
        float maxElbowBend = osg::PI;
        std::array<Elbow, 2> elbows;
    };

    // Rotate the clavicle about its own neck attachment to gain a little
    // reach, keeping that attachment and the three arm segments intact.
    inline osg::Vec3f turnGripClavicle(GripArm& arm, const osg::Vec3f& shoulder, const osg::Vec3f& wrist, float reach,
        float maximumTurn, const std::function<osg::Matrixf(osg::MatrixTransform*)>& sourceLocal)
    {
        arm.clavicle = nullptr;
        if (maximumTurn <= 0.f || (wrist - shoulder).length() <= reach || !arm.bones[0]->getNumParents())
            return shoulder;
        auto* clavicle = dynamic_cast<osg::MatrixTransform*>(arm.bones[0]->getParent(0));
        if (!clavicle)
            return shoulder;
        const osg::Vec3f pivot = arm.parent.getTrans();
        osg::Vec3f along = shoulder - pivot, toWrist = wrist - pivot;
        const float length = along.normalize(), distance = toWrist.normalize();
        if (length < .001f || distance < .001f)
            return shoulder;
        const float angle = std::acos(std::clamp(along * toWrist, -1.f, 1.f));
        const float limit = std::acos(
            std::clamp((length * length + distance * distance - reach * reach) / (2.f * length * distance), -1.f, 1.f));
        if (angle <= limit || angle < .001f)
            return shoulder;
        osg::Quat target, turn;
        target.makeRotate(along, toWrist);
        turn.slerp(std::min(maximumTurn, angle - limit) / angle, osg::Quat(), target);
        arm.clavicle = clavicle;
        arm.clavicleSource = sourceLocal(clavicle);
        const osg::Matrixf parentInverse = osg::Matrixf::inverse(arm.parent) * arm.clavicleSource;
        arm.parent = arm.parent * osg::Matrixf::translate(-pivot) * osg::Matrixf::rotate(turn)
            * osg::Matrixf::translate(pivot);
        arm.clavicleResult = arm.parent * parentInverse;
        arm.clavicleResult.setTrans(arm.clavicleSource.getTrans());
        return (arm.source[0] * arm.parent).getTrans();
    }

    // Moves both hands by `total` (applied to the evaluated hand frames) with the
    // shoulders fixed: each arm is re-solved as a two-bone chain and keeps its
    // authored elbow plane. Out-of-reach goals are clamped to full extension and
    // the same clamp is carried to the other hand so a two-handed grip holds.
    // Nothing is modified; `arms` receives the local results.
    inline bool solveGripArms(const Animation::NodeMap& nodes, const osg::Matrixf& total,
        const std::function<osg::Matrixf(osg::MatrixTransform*)>& sourceLocal, std::array<GripArm, 2>& arms,
        float& maxClamp, const GripArmLimits& limits = {},
        const std::array<osg::Matrixf, 2>* separate = nullptr, unsigned count = 2)
    {
        unsigned side = 0;
        // The shaft follows the right hand; any safety clamp moves both goals.
        for (const char* prefix : { "Bip01 R ", "Bip01 L " })
        {
            if (side == count)
                break;
            auto& arm = arms[side++];
            unsigned joint = 0;
            for (const char* suffix : { "UpperArm", "Forearm", "Hand" })
            {
                const auto found = nodes.find(std::string(prefix) + suffix);
                if (found == nodes.end() || found->second->getParentalNodePaths().empty())
                    return false;
                auto* bone = found->second.get();
                if (joint && (!bone->getNumParents() || bone->getParent(0) != arm.bones[joint - 1]))
                    return false;
                arm.bones[joint] = bone;
                arm.source[joint] = sourceLocal(bone);
                if (!joint)
                {
                    auto path = bone->getParentalNodePaths().front();
                    path.pop_back();
                    arm.parent = osg::computeLocalToWorld(path);
                }
                arm.world[joint] = arm.source[joint] * (joint ? arm.world[joint - 1] : arm.parent);
                ++joint;
            }
        }
        osg::Vec3f clampOffset;
        maxClamp = 0.f;
        for (unsigned i = 0; i < count; ++i)
        {
            auto& arm = arms[i];
            const osg::Matrixf& correction = separate ? (*separate)[i] : total;
            const osg::Quat turn = correction.getRotate();
            const osg::Vec3f oldShoulder = arm.world[0].getTrans();
            const osg::Vec3f oldElbow = arm.world[1].getTrans();
            const osg::Vec3f oldWrist = arm.world[2].getTrans();
            osg::Matrixf hand = arm.world[2] * correction * osg::Matrixf::translate(clampOffset);
            const osg::Vec3f requested = hand.getTrans();
            const float upperLength = (oldElbow - oldShoulder).length();
            const float foreLength = (oldWrist - oldElbow).length();
            if (upperLength < .001f || foreLength < .001f)
                return false;
            const osg::Vec3f shoulder = turnGripClavicle(
                arm, oldShoulder, requested, upperLength + foreLength - .05f, limits.clavicleTurn, sourceLocal);
            osg::Vec3f axis = requested - shoulder;
            const float distance = axis.normalize();
            if (distance < .001f)
                return false;
            const auto elbowReach = [&](float bend) {
                return std::sqrt(std::max(0.f, upperLength * upperLength + foreLength * foreLength
                    + 2.f * upperLength * foreLength * std::cos(bend)));
            };
            const float reach = std::clamp(distance,
                std::max(std::abs(upperLength - foreLength) + .001f, elbowReach(limits.maxElbowBend)),
                std::min(upperLength + foreLength - .001f, elbowReach(limits.minElbowBend)));
            const osg::Vec3f target = shoulder + axis * reach;
            const osg::Vec3f offset = target - requested;
            maxClamp = std::max(maxClamp, offset.length());
            if (i == 0 && !separate)
                clampOffset = offset;
            hand.setTrans(target);
            osg::Vec3f oldAxis = oldWrist - oldShoulder;
            oldAxis.normalize();
            osg::Vec3f sourceBend = oldElbow - oldShoulder;
            sourceBend -= oldAxis * (sourceBend * oldAxis);
            // A straight arm has no elbow pole. Use actor-relative down/out
            // only in that degenerate case; identity keeps every other pose.
            osg::Vec3f stable = osg::Matrixf::transform3x3(osg::Vec3f(0, 0, -1), arm.parent);
            stable -= oldAxis * (stable * oldAxis);
            if (stable.normalize() < .001f)
            {
                stable = osg::Matrixf::transform3x3(osg::Vec3f(1, 0, 0), arm.parent);
                stable -= oldAxis * (stable * oldAxis);
                stable.normalize();
            }
            const float height = sourceBend.normalize();
            if (height < .001f)
                sourceBend = stable;
            osg::Vec3f bend = turn * sourceBend;
            bend -= axis * (bend * axis);
            if (bend.normalize() < .001f)
            {
                osg::Quat transport;
                transport.makeRotate(oldAxis, axis);
                bend = transport * sourceBend;
            }
            const auto& elbowLimit = limits.elbows[i];
            if (elbowLimit.turn > 0.f)
            {
                osg::Vec3f guide = elbowLimit.guide - axis * (elbowLimit.guide * axis);
                if (guide.normalize() > .001f)
                {
                    const float swivel = std::atan2(axis * (bend ^ guide), bend * guide);
                    bend = osg::Quat(std::clamp(swivel, -elbowLimit.turn, elbowLimit.turn), axis) * bend;
                }
            }
            const float along = (upperLength * upperLength - foreLength * foreLength + reach * reach)
                / (2.f * reach);
            const osg::Vec3f elbow = shoulder + axis * along
                + bend * std::sqrt(std::max(0.f, upperLength * upperLength - along * along));
            osg::Quat upperSwing, foreSwing;
            upperSwing.makeRotate(turn * (oldElbow - oldShoulder), elbow - shoulder);
            foreSwing.makeRotate(turn * (oldWrist - oldElbow), target - elbow);
            const osg::Matrixf upper = arm.world[0] * osg::Matrixf::translate(-oldShoulder)
                * osg::Matrixf::rotate(turn * upperSwing) * osg::Matrixf::translate(shoulder);
            const osg::Matrixf fore = arm.world[1] * osg::Matrixf::translate(-oldElbow)
                * osg::Matrixf::rotate(turn * foreSwing) * osg::Matrixf::translate(elbow);
            const std::array<osg::Matrixf, 3> parents{ arm.parent, upper, fore };
            const std::array<osg::Matrixf, 3> goals{ upper, fore, hand };
            for (unsigned joint = 0; joint < arm.bones.size(); ++joint)
            {
                arm.result[joint] = goals[joint] * osg::Matrixf::inverse(parents[joint]);
                arm.result[joint].setTrans(arm.source[joint].getTrans());
            }
        }
        return true;
    }

    // Limit a transient correction as a single rigid transform so the two
    // hands do not receive different reach clamps at the end of their chains.
    inline bool solveSharedGripArms(const Animation::NodeMap& nodes, const osg::Matrixf& correction,
        const osg::Vec3f& pivot, const std::function<osg::Matrixf(osg::MatrixTransform*)>& sourceLocal,
        std::array<GripArm, 2>& arms, osg::Matrixf& applied, const GripArmLimits& limits = {})
    {
        constexpr float tolerance = .005f;
        float clamp = 0.f;
        applied = correction;
        if (!solveGripArms(nodes, correction, sourceLocal, arms, clamp, limits))
            return false;
        if (clamp <= tolerance)
            return true;
        const osg::Vec3f delta = pivot * correction - pivot;
        const auto scaled = [&](float weight) {
            osg::Quat turn;
            turn.slerp(weight, osg::Quat(), correction.getRotate());
            return osg::Matrixf::translate(-pivot) * osg::Matrixf::rotate(turn)
                * osg::Matrixf::translate(pivot + delta * weight);
        };
        float lower = 0.f, upper = 1.f;
        std::array<GripArm, 2> trial;
        for (int i = 0; i < 8; ++i)
        {
            const float weight = (lower + upper) * .5f;
            if (solveGripArms(nodes, scaled(weight), sourceLocal, trial, clamp, limits) && clamp <= tolerance)
                lower = weight;
            else
                upper = weight;
        }
        applied = scaled(lower);
        return solveGripArms(nodes, applied, sourceLocal, arms, clamp, limits) && clamp <= tolerance;
    }
}

#endif
