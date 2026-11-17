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
    };

    // Moves both hands by `total` (applied to the evaluated hand frames) with the
    // shoulders fixed: each arm is re-solved as a two-bone chain and keeps its
    // authored elbow plane. Out-of-reach goals are clamped to full extension and
    // the same clamp is carried to the other hand so a two-handed grip holds.
    // Nothing is modified; `arms` receives the local results.
    inline bool solveGripArms(const Animation::NodeMap& nodes, const osg::Matrixf& total,
        const std::function<osg::Matrixf(osg::MatrixTransform*)>& sourceLocal, std::array<GripArm, 2>& arms,
        float& maxClamp)
    {
        unsigned side = 0;
        // The shaft follows the right hand; any safety clamp moves both goals.
        for (const char* prefix : { "Bip01 R ", "Bip01 L " })
        {
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
        const osg::Quat turn = total.getRotate();
        for (auto& arm : arms)
        {
            const osg::Vec3f shoulder = arm.world[0].getTrans();
            const osg::Vec3f oldElbow = arm.world[1].getTrans();
            const osg::Vec3f oldWrist = arm.world[2].getTrans();
            osg::Matrixf hand = arm.world[2] * total * osg::Matrixf::translate(clampOffset);
            const osg::Vec3f requested = hand.getTrans();
            const float upperLength = (oldElbow - shoulder).length();
            const float foreLength = (oldWrist - oldElbow).length();
            if (upperLength < .001f || foreLength < .001f)
                return false;
            osg::Vec3f axis = requested - shoulder;
            const float distance = axis.normalize();
            if (distance < .001f)
                return false;
            const float reach = std::clamp(distance, std::abs(upperLength - foreLength) + .001f,
                upperLength + foreLength - .001f);
            const osg::Vec3f target = shoulder + axis * reach;
            const osg::Vec3f offset = target - requested;
            maxClamp = std::max(maxClamp, offset.length());
            if (&arm == &arms.front())
                clampOffset = offset;
            hand.setTrans(target);
            osg::Vec3f oldAxis = oldWrist - shoulder;
            oldAxis.normalize();
            osg::Vec3f sourceBend = oldElbow - shoulder;
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
            const float along = (upperLength * upperLength - foreLength * foreLength + reach * reach)
                / (2.f * reach);
            const osg::Vec3f elbow = shoulder + axis * along
                + bend * std::sqrt(std::max(0.f, upperLength * upperLength - along * along));
            osg::Quat upperSwing, foreSwing;
            upperSwing.makeRotate(turn * (oldElbow - shoulder), elbow - shoulder);
            foreSwing.makeRotate(turn * (oldWrist - oldElbow), target - elbow);
            const osg::Matrixf upper = arm.world[0] * osg::Matrixf::translate(-shoulder)
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

}

#endif
