#ifndef OPENMW_MWRENDER_NPCGUARDIK_H
#define OPENMW_MWRENDER_NPCGUARDIK_H

#include "griparms.hpp"
#include "vismask.hpp"
#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <optional>
#include <osg/BoundingBox>
#include <osg/Geometry>
#include <osg/NodeVisitor>
#include <osg/TriangleIndexFunctor>
#include <string>
#include <utility>
#include <vector>

// Native NPC guard math. The legacy first-person grip solver is not used.
namespace MWRender::NpcGuardIk
{
    inline bool finite(const osg::Vec3f& point)
    {
        return std::isfinite(point.x()) && std::isfinite(point.y()) && std::isfinite(point.z());
    }

    inline std::optional<osg::Matrixf> nodeWorld(osg::Node* node)
    {
        if (!node)
            return {};
        const auto paths = node->getParentalNodePaths();
        if (paths.empty())
            return {};
        return osg::Matrixf(osg::computeLocalToWorld(paths.front()));
    }

    struct MeshTriangles
    {
        std::vector<std::array<unsigned int, 3>> values;
        void operator()(unsigned int a, unsigned int b, unsigned int c) { values.push_back({ a, b, c }); }
    };

    // Guard contact comes from the actual equipped item's material. Cache it
    // in item space; only its rigid bone attachment changes between frames.
    class GuardMeshVisitor : public osg::NodeVisitor
    {
        osg::Matrixf mRootInverse;

    public:
        std::vector<std::array<osg::Vec3f, 3>> triangles;
        osg::BoundingBox bounds;

        explicit GuardMeshVisitor(osg::Node* root)
            : osg::NodeVisitor(TRAVERSE_ALL_CHILDREN)
            , mRootInverse(osg::Matrixf::inverse(osg::computeLocalToWorld(osg::NodePath{ root })))
        {
            setTraversalMask(~(Mask_UpdateVisitor | Mask_Effect | Mask_ParticleSystem));
        }

        void apply(osg::Geometry& geometry) override
        {
            const auto* vertices = dynamic_cast<const osg::Vec3Array*>(geometry.getVertexArray());
            if (!vertices)
                return;
            const osg::Matrixf local = osg::computeLocalToWorld(getNodePath()) * mRootInverse;
            osg::TriangleIndexFunctor<MeshTriangles> faces;
            geometry.accept(faces);
            for (const auto& face : faces.values)
            {
                if (face[0] >= vertices->size() || face[1] >= vertices->size() || face[2] >= vertices->size())
                    continue;
                std::array<osg::Vec3f, 3> triangle;
                for (unsigned i = 0; i < 3; ++i)
                    triangle[i] = (*vertices)[face[i]] * local;
                if (!finite(triangle[0]) || !finite(triangle[1]) || !finite(triangle[2])
                    || ((triangle[1] - triangle[0]) ^ (triangle[2] - triangle[0])).length2() < 1e-8f)
                    continue;
                triangles.push_back(triangle);
                for (const auto& point : triangle)
                    bounds.expandBy(point);
            }
        }
    };

    inline osg::Vec3f closestPoint(const osg::Vec3f& point, const std::array<osg::Vec3f, 3>& triangle)
    {
        const auto& a = triangle[0];
        const auto& b = triangle[1];
        const auto& c = triangle[2];
        const osg::Vec3f ab = b - a, ac = c - a, ap = point - a;
        if ((ab ^ ac).length2() < 1e-20f)
        {
            // Degenerate triangles are segments or points. Handle them before
            // the barycentric edge tests, which can also divide by zero.
            const auto onSegment = [&](const osg::Vec3f& start, const osg::Vec3f& end) {
                const osg::Vec3f edge = end - start;
                const float length2 = edge.length2();
                return length2 > 0.f ? start + edge * std::clamp((point - start) * edge / length2, 0.f, 1.f) : start;
            };
            osg::Vec3f closest = onSegment(a, b);
            for (const auto candidate : { onSegment(a, c), onSegment(b, c) })
                if ((point - candidate).length2() < (point - closest).length2())
                    closest = candidate;
            return closest;
        }
        const float d1 = ab * ap, d2 = ac * ap;
        if (d1 <= 0.f && d2 <= 0.f)
            return a;
        const osg::Vec3f bp = point - b;
        const float d3 = ab * bp, d4 = ac * bp;
        if (d3 >= 0.f && d4 <= d3)
            return b;
        const float vc = d1 * d4 - d3 * d2;
        if (vc <= 0.f && d1 >= 0.f && d3 <= 0.f)
            return a + ab * (d1 / (d1 - d3));
        const osg::Vec3f cp = point - c;
        const float d5 = ab * cp, d6 = ac * cp;
        if (d6 >= 0.f && d5 <= d6)
            return c;
        const float vb = d5 * d2 - d1 * d6;
        if (vb <= 0.f && d2 >= 0.f && d6 <= 0.f)
            return a + ac * (d2 / (d2 - d6));
        const float va = d3 * d6 - d5 * d4;
        if (va <= 0.f && d4 - d3 >= 0.f && d5 - d6 >= 0.f)
            return b + (c - b) * ((d4 - d3) / ((d4 - d3) + (d5 - d6)));
        const float sum = va + vb + vc;
        if (std::abs(sum) < 1e-20f)
            return a;
        const float inverse = 1.f / sum;
        return a + ab * (vb * inverse) + ac * (vc * inverse);
    }

    inline std::optional<float> segmentTriangleHit(
        const osg::Vec3f& origin, const osg::Vec3f& end, const std::array<osg::Vec3f, 3>& triangle)
    {
        const osg::Vec3d direction = osg::Vec3d(end) - osg::Vec3d(origin);
        const osg::Vec3d first = osg::Vec3d(triangle[1]) - osg::Vec3d(triangle[0]);
        const osg::Vec3d second = osg::Vec3d(triangle[2]) - osg::Vec3d(triangle[0]);
        const auto cross = direction ^ second;
        const double determinant = first * cross;
        if (std::abs(determinant) < 1e-10)
            return {};
        const osg::Vec3d offset = osg::Vec3d(origin) - osg::Vec3d(triangle[0]);
        const double u = (offset * cross) / determinant;
        const auto other = offset ^ first;
        const double v = (direction * other) / determinant;
        const double time = (second * other) / determinant;
        constexpr double epsilon = 1e-8;
        if (u < -epsilon || v < -epsilon || u + v > 1.0 + epsilon || time < -epsilon || time > 1.0 + epsilon)
            return {};
        return std::clamp(float(time), 0.f, 1.f);
    }

    struct SegmentMeshGap
    {
        double squared = std::numeric_limits<double>::infinity();
        osg::Vec3f point;
    };

    inline SegmentMeshGap segmentEdgeGap(
        const osg::Vec3f& origin, const osg::Vec3f& end, const osg::Vec3f& first, const osg::Vec3f& second)
    {
        const osg::Vec3d along = osg::Vec3d(end) - osg::Vec3d(origin);
        const osg::Vec3d edge = osg::Vec3d(second) - osg::Vec3d(first);
        const osg::Vec3d offset = osg::Vec3d(origin) - osg::Vec3d(first);
        const double a = along * along, e = edge * edge, f = edge * offset;
        double s = 0.0, t = 0.0;
        constexpr double epsilon = 1e-12;
        if (a <= epsilon)
            t = e > epsilon ? std::clamp(f / e, 0.0, 1.0) : 0.0;
        else
        {
            const double c = along * offset;
            if (e <= epsilon)
                s = std::clamp(-c / a, 0.0, 1.0);
            else
            {
                const double b = along * edge, denominator = a * e - b * b;
                if (denominator > epsilon)
                    s = std::clamp((b * f - c * e) / denominator, 0.0, 1.0);
                t = (b * s + f) / e;
                if (t < 0.0)
                {
                    t = 0.0;
                    s = std::clamp(-c / a, 0.0, 1.0);
                }
                else if (t > 1.0)
                {
                    t = 1.0;
                    s = std::clamp((b - c) / a, 0.0, 1.0);
                }
            }
        }
        const osg::Vec3d meshPoint = osg::Vec3d(first) + edge * t;
        return { (osg::Vec3d(origin) + along * s - meshPoint).length2(), osg::Vec3f(meshPoint) };
    }

    inline float matrixError(const osg::Matrixf& a, const osg::Matrixf& b)
    {
        float result = 0.f;
        for (unsigned i = 0; i < 16; ++i)
            result = std::max(result, std::abs(a.ptr()[i] - b.ptr()[i]));
        return result;
    }

    inline float orthogonalityError(const osg::Matrixf& matrix)
    {
        float result = 0.f;
        for (unsigned row = 0; row < 3; ++row)
            for (unsigned other = 0; other < 3; ++other)
            {
                float product = 0.f;
                for (unsigned column = 0; column < 3; ++column)
                    product += matrix(row, column) * matrix(other, column);
                result = std::max(result, std::abs(product - (row == other ? 1.f : 0.f)));
            }
        return result;
    }

    inline bool captureGuardArm(const Animation::NodeMap& nodes, bool left, GripArm& arm)
    {
        const std::string prefix = left ? "Bip01 L " : "Bip01 R ";
        unsigned joint = 0;
        for (const char* suffix : { "UpperArm", "Forearm", "Hand" })
        {
            const auto found = nodes.find(prefix + suffix);
            if (found == nodes.end())
                return false;
            auto* bone = found->second.get();
            if (joint && (!bone->getNumParents() || bone->getParent(0) != arm.bones[joint - 1]))
                return false;
            arm.bones[joint] = bone;
            arm.source[joint] = bone->getMatrix();
            if (!joint)
            {
                const auto paths = bone->getParentalNodePaths();
                if (paths.empty())
                    return false;
                auto path = paths.front();
                path.pop_back();
                arm.parent = osg::computeLocalToWorld(path);
            }
            arm.world[joint] = arm.source[joint] * (joint ? arm.world[joint - 1] : arm.parent);
            ++joint;
        }
        return true;
    }

    inline std::optional<osg::Vec3f> materialWitness(const std::vector<std::array<osg::Vec3f, 3>>& triangles,
        const osg::Vec3f& desired, const osg::Vec3f& axis, float minimum, float maximum)
    {
        float best = std::numeric_limits<float>::max();
        std::optional<osg::Vec3f> result;
        for (const auto& triangle : triangles)
        {
            const auto candidate = closestPoint(desired, triangle);
            const float coordinate = candidate * axis;
            if (coordinate < minimum || coordinate > maximum)
                continue;
            const float score = (candidate - desired).length2();
            if (score < best)
            {
                best = score;
                result = candidate;
            }
        }
        return result;
    }

    inline osg::Matrixf rotationOnly(const osg::Matrixf& matrix)
    {
        osg::Vec3f translation, scale;
        osg::Quat rotation, scaleOrientation;
        matrix.decompose(translation, rotation, scale, scaleOrientation);
        return osg::Matrixf::rotate(rotation);
    }

    inline float rotationDistanceDegrees(const osg::Quat& first, const osg::Quat& second)
    {
        const double dot
            = first.x() * second.x() + first.y() * second.y() + first.z() * second.z() + first.w() * second.w();
        const double firstNorm = first.length2(), secondNorm = second.length2();
        if (!std::isfinite(dot) || !std::isfinite(firstNorm) || !std::isfinite(secondNorm) || firstNorm <= 1e-20
            || secondNorm <= 1e-20)
            return std::numeric_limits<float>::infinity();
        const double normalized = dot / std::sqrt(firstNorm * secondNorm);
        return float(osg::RadiansToDegrees(2.0 * std::acos(std::clamp(std::abs(normalized), 0.0, 1.0))));
    }

    struct JointAngles
    {
        float swing = 0.f, twist = 0.f;
    };

    inline JointAngles jointAngles(
        const osg::Matrixf& authoredRotation, const osg::Matrixf& local, const osg::Vec3f& axis)
    {
        const osg::Quat delta
            = (osg::Matrixf::inverse(authoredRotation) * osg::Matrixf::rotate(local.getRotate())).getRotate();
        const float projection = delta.x() * axis.x() + delta.y() * axis.y() + delta.z() * axis.z();
        const float length = std::sqrt(projection * projection + delta.w() * delta.w());
        if (length < 1e-6f)
            return { 180.f, 0.f };
        const osg::Quat twist(axis.x() * projection / length, axis.y() * projection / length,
            axis.z() * projection / length, delta.w() / length);
        const osg::Quat swing = delta * twist.inverse();
        const float angle = 2.f * std::atan2(projection, float(delta.w()));
        return { osg::RadiansToDegrees(2.f * std::acos(std::clamp(float(std::abs(swing.w())), 0.f, 1.f))),
            osg::RadiansToDegrees(std::atan2(std::sin(angle), std::cos(angle))) };
    }

    struct GuardJointMetrics
    {
        float wristSwing = 0.f, wristTwist = 0.f, palmFlex = 0.f, palmSide = 0.f;
        float foreTwist = 0.f, upperSwing = 0.f, upperTwist = 0.f, reach = 0.f, inward = 0.f;
        float signedElbowFlex = 0.f, elbowHingeError = 0.f;
        float clearance = std::numeric_limits<float>::infinity(), elbowFlex = 0.f;
        float stanceMargin = std::numeric_limits<float>::infinity();
        float shieldHeight = 0.f, shieldForward = 0.f, shieldTilt = 0.f;
        osg::Vec3f shieldCenter;
        osg::Vec3f palmForward, palmLateral, palmNormal;

        bool natural() const
        {
            return wristSwing <= 40.f && std::abs(wristTwist) <= 45.f && std::abs(palmFlex) <= 60.f
                && std::abs(palmSide) <= 35.f && std::abs(foreTwist) <= 40.f && std::abs(foreTwist + wristTwist) <= 85.f
                && upperSwing <= 110.f && std::abs(upperTwist) <= 70.f && signedElbowFlex >= -.1f
                && signedElbowFlex <= 145.1f && elbowHingeError <= 5.f && inward <= 0.f && clearance >= 0.f
                && elbowFlex <= 145.1f;
        }

        bool working() const { return natural() && stanceMargin >= 0.f; }

        std::array<float, 14> hardViolations() const
        {
            const auto excess = [](float value, float limit) { return std::max(std::abs(value) - limit, 0.f); };
            return { excess(wristSwing, 40.f), excess(wristTwist, 45.f), excess(palmFlex, 60.f), excess(palmSide, 35.f),
                excess(foreTwist, 40.f), excess(foreTwist + wristTwist, 85.f), excess(upperSwing, 110.f),
                excess(upperTwist, 70.f), std::max(-signedElbowFlex, 0.f), std::max(signedElbowFlex - 145.1f, 0.f),
                excess(elbowHingeError, 5.f), inward * 4.f, std::max(-clearance, 0.f) * 10.f,
                excess(elbowFlex, 145.1f) };
        }

        bool improves(const GuardJointMetrics& previous) const
        {
            const auto before = previous.hardViolations(), after = hardViolations();
            float improvement = 0.f;
            for (unsigned bound = 0; bound < before.size(); ++bound)
            {
                if (after[bound] > before[bound] + .0001f)
                    return false;
                improvement += before[bound] - after[bound];
            }
            return improvement > .0001f;
        }

        float score() const
        {
            const auto excess = [](float value, float limit) { return std::max(std::abs(value) - limit, 0.f); };
            const float violation = excess(wristSwing, 40.f) + excess(wristTwist, 45.f) + excess(palmFlex, 60.f)
                + excess(palmSide, 35.f) + excess(foreTwist, 40.f) + excess(foreTwist + wristTwist, 85.f)
                + excess(upperSwing, 110.f) + excess(upperTwist, 70.f) + excess(elbowHingeError, 5.f)
                + std::max(-signedElbowFlex, 0.f) + inward * 4.f + std::max(-clearance, 0.f) * 10.f
                + std::max(-stanceMargin, 0.f) * 10.f;
            return (working() ? 0.f : 1e6f + violation * 1000.f) + reach * 100.f + wristSwing * wristSwing * .002f
                + wristTwist * wristTwist * .001f + excess(palmSide, 25.f) * .08f + std::abs(foreTwist) * .01f
                + upperSwing * .01f;
        }
    };

    struct GuardJointSource
    {
        GripArm arm;
        osg::Matrixf parentInverse, handRotation, upperRotation, foreZeroRotation;
        osg::Matrixf metricToWorld, metricToActor, itemToHand;
        osg::Vec3f wristAxis, upperAxis, upperHingeLocal, foreHingeLocal;
        osg::Vec3f palmForward, palmLateral, outward;
        float upperLength = 0.f, foreLength = 0.f;
        struct Capsule
        {
            osg::Vec3f from, to;
            float radius;
        };
        std::vector<Capsule> body;
        std::vector<osg::Vec3f> itemPoints;
        bool shield = false;
        osg::Vec3f itemCenter, torsoActor, pelvisActor, forwardActor;
        std::array<osg::Vec3f, 2> shieldTangents;
        float shieldNormalSign = 1.f, minimumShieldHeight = 0.f;

        osg::Matrixf foreLocal(float flex, float pronation) const
        {
            auto result = foreZeroRotation * osg::Matrixf::rotate(flex, upperHingeLocal);
            auto direction = osg::Matrixf::transform3x3(osg::Vec3f(arm.source[2].getTrans()), result);
            direction.normalize();
            result = result * osg::Matrixf::rotate(pronation, direction);
            result.setTrans(arm.source[1].getTrans());
            return result;
        }

        GuardJointMetrics measure(
            const std::array<osg::Matrixf, 3>& local, const std::array<osg::Matrixf, 3>& world, float reach) const
        {
            GuardJointMetrics result;
            const auto wrist = jointAngles(handRotation, local[2], wristAxis);
            result.wristSwing = wrist.swing;
            result.wristTwist = wrist.twist;
            result.upperTwist = jointAngles(upperRotation, local[0], upperAxis).twist;
            osg::Vec3f upper = world[1].getTrans() - world[0].getTrans();
            upper.normalize();
            osg::Vec3f authoredUpper = osg::Matrixf::transform3x3(osg::Vec3f(arm.source[1].getTrans()), upperRotation);
            authoredUpper.normalize();
            result.upperSwing = osg::RadiansToDegrees(std::acos(std::clamp(upper * authoredUpper, -1.f, 1.f)));
            result.inward
                = std::max(-osg::Vec3f(world[1].getTrans() - world[0].getTrans()) * outward - upperLength * .25f, 0.f);
            const auto handWorld = world[2] * metricToWorld;
            result.palmForward = osg::Matrixf::transform3x3(palmForward, handWorld);
            result.palmForward.normalize();
            result.palmLateral = osg::Matrixf::transform3x3(palmLateral, handWorld);
            result.palmLateral -= result.palmForward * (result.palmLateral * result.palmForward);
            result.palmLateral.normalize();
            result.palmNormal = result.palmForward ^ result.palmLateral;
            osg::Vec3f fore = (world[2].getTrans() * metricToWorld) - (world[1].getTrans() * metricToWorld);
            fore.normalize();
            result.palmFlex = osg::RadiansToDegrees(std::atan2(fore * result.palmNormal, fore * result.palmForward));
            result.palmSide = osg::RadiansToDegrees(std::atan2(fore * result.palmLateral, fore * result.palmForward));
            result.reach = reach;
            osg::Vec3f metricFore = world[2].getTrans() - world[1].getTrans();
            metricFore.normalize();
            result.elbowFlex = osg::RadiansToDegrees(std::acos(std::clamp(upper * metricFore, -1.f, 1.f)));
            auto hinge = osg::Matrixf::transform3x3(upperHingeLocal, world[0]);
            hinge.normalize();
            const auto bend = upper ^ metricFore;
            result.signedElbowFlex = osg::RadiansToDegrees(std::atan2(hinge * bend, upper * metricFore));
            if (bend.length2() > .0001f)
                result.elbowHingeError
                    = osg::RadiansToDegrees(std::acos(std::clamp(hinge * (bend / bend.length()), -1.f, 1.f)));
            auto forePlane = osg::Matrixf::transform3x3(foreHingeLocal, world[1]);
            forePlane -= metricFore * (forePlane * metricFore);
            forePlane.normalize();
            result.foreTwist = osg::RadiansToDegrees(std::atan2(metricFore * (hinge ^ forePlane), hinge * forePlane));
            const auto clearance = [&](const osg::Vec3f& point, float padding) {
                float best = std::numeric_limits<float>::infinity();
                for (const auto& capsule : body)
                {
                    const auto axis = capsule.to - capsule.from;
                    const float parameter
                        = std::clamp((point - capsule.from) * axis / std::max(axis.length2(), .001f), 0.f, 1.f);
                    best
                        = std::min(best, (point - capsule.from - axis * parameter).length() - capsule.radius - padding);
                }
                return best;
            };
            result.clearance = std::min(clearance(world[1].getTrans() * metricToActor, 1.5f),
                clearance(world[2].getTrans() * metricToActor, 2.f));
            const auto item = itemToHand * world[2] * metricToActor;
            for (const auto& point : itemPoints)
                result.clearance = std::min(result.clearance, clearance(point * item, 1.f));
            if (shield)
            {
                result.shieldCenter = itemCenter * item;
                auto normal = (osg::Matrixf::transform3x3(shieldTangents[0], item)
                                  ^ osg::Matrixf::transform3x3(shieldTangents[1], item))
                    * shieldNormalSign;
                normal.normalize();
                result.shieldHeight = result.shieldCenter.z() - pelvisActor.z();
                result.shieldForward = (result.shieldCenter - torsoActor) * forwardActor;
                result.shieldTilt = osg::RadiansToDegrees(std::asin(std::clamp(std::abs(normal.z()), 0.f, 1.f)));
                result.stanceMargin = std::min({ result.shieldCenter.z() - minimumShieldHeight,
                    result.shieldForward - 18.f, (25.f - result.shieldTilt) * .25f });
            }
            return result;
        }
    };

    struct GuardJointPose
    {
        std::array<osg::Matrixf, 3> local, world;
        GuardJointMetrics metrics;
    };

    // Position, elbow swivel and wrist orientation are solved together.
    // The authored Hand-local frame defines a neutral loaded grip; its
    // inverse gives the forearm frame that would keep that wrist unchanged.
    inline GuardJointPose solveGuardJoint(const GuardJointSource& source, const osg::Matrixf& requestedHand,
        float swivelDegrees, std::optional<float> pronationDegrees = {})
    {
        GuardJointPose result;
        auto hand = requestedHand;
        const osg::Vec3f shoulder = source.arm.world[0].getTrans();
        osg::Vec3f axis = hand.getTrans() - shoulder;
        const float distance = axis.normalize();
        if (distance < .001f)
            axis = osg::Vec3f(0, 0, -1);
        const float minimum = std::sqrt(std::max(0.f,
            source.upperLength * source.upperLength + source.foreLength * source.foreLength
                + 2.f * source.upperLength * source.foreLength * std::cos(osg::DegreesToRadians(145.f))));
        const float reach = std::clamp(distance, minimum, source.upperLength + source.foreLength - .01f);
        const auto wrist = shoulder + axis * reach;
        hand.setTrans(wrist);
        const float along
            = (source.upperLength * source.upperLength - source.foreLength * source.foreLength + reach * reach)
            / (2.f * reach);
        const float radius = std::sqrt(std::max(source.upperLength * source.upperLength - along * along, 0.f));
        const auto center = shoulder + axis * along;
        auto neutralUpper = source.upperRotation;
        neutralUpper.setTrans(shoulder);
        const osg::Vec3f neutralElbow = source.arm.source[1].getTrans() * neutralUpper;
        osg::Vec3f pole = neutralElbow - center;
        pole -= axis * (pole * axis);
        if (pole.normalize() < .001f)
        {
            pole = source.arm.world[1].getTrans() - center;
            pole -= axis * (pole * axis);
            if (pole.normalize() < .001f)
            {
                pole = source.outward - axis * (source.outward * axis);
                pole.normalize();
            }
        }
        const auto elbow = center + osg::Quat(osg::DegreesToRadians(swivelDegrees), axis) * pole * radius;
        osg::Quat upperSwing;
        const osg::Vec3f sourceUpperDirection = neutralElbow - shoulder;
        const osg::Vec3f desiredUpperDirection = elbow - shoulder;
        const osg::Vec3f desiredForeDirection = wrist - elbow;
        upperSwing.makeRotate(sourceUpperDirection, desiredUpperDirection);
        auto desiredUpper = desiredUpperDirection;
        desiredUpper.normalize();
        auto desiredFore = desiredForeDirection;
        desiredFore.normalize();
        auto desiredHinge = desiredUpper ^ desiredFore;
        if (desiredHinge.normalize() < .001f)
            desiredHinge = osg::Matrixf::transform3x3(source.upperHingeLocal, neutralUpper);
        auto movedHinge
            = osg::Matrixf::transform3x3(source.upperHingeLocal, neutralUpper * osg::Matrixf::rotate(upperSwing));
        movedHinge.normalize();
        const float upperRoll = std::atan2(desiredUpper * (movedHinge ^ desiredHinge), movedHinge * desiredHinge);
        const auto turn = osg::Matrixf::rotate(upperSwing) * osg::Matrixf::rotate(upperRoll, desiredUpper);
        const auto upper = neutralUpper * osg::Matrixf::translate(-shoulder) * turn * osg::Matrixf::translate(shoulder);
        const float flex = std::atan2(desiredHinge * (desiredUpper ^ desiredFore), desiredUpper * desiredFore);
        auto fore = source.foreLocal(flex, 0.f) * upper;
        const auto evaluate = [&](const osg::Matrixf& foreWorld) {
            GuardJointPose candidate;
            candidate.local = { upper * source.parentInverse, foreWorld * osg::Matrixf::inverse(upper),
                hand * osg::Matrixf::inverse(foreWorld) };
            for (unsigned i = 0; i < 3; ++i)
            {
                candidate.local[i] = osg::Matrixf::rotate(candidate.local[i].getRotate());
                candidate.local[i].setTrans(source.arm.source[i].getTrans());
                candidate.world[i] = candidate.local[i] * (i ? candidate.world[i - 1] : source.arm.parent);
            }
            candidate.metrics = source.measure(
                candidate.local, candidate.world, (requestedHand.getTrans() - candidate.world[2].getTrans()).length());
            return candidate;
        };
        if (pronationDegrees)
        {
            osg::Vec3f foreAxis = wrist - elbow;
            foreAxis.normalize();
            return evaluate(fore * osg::Matrixf::translate(-elbow)
                * osg::Matrixf::rotate(osg::DegreesToRadians(*pronationDegrees), foreAxis)
                * osg::Matrixf::translate(elbow));
        }
        result = evaluate(fore);
        if (std::abs(result.metrics.wristTwist) > 20.f)
        {
            osg::Vec3f foreAxis = wrist - elbow;
            foreAxis.normalize();
            const float amount = std::min(std::abs(result.metrics.wristTwist), 40.f);
            for (float sign : { -1.f, 1.f })
            {
                const auto shared = fore * osg::Matrixf::translate(-elbow)
                    * osg::Matrixf::rotate(osg::DegreesToRadians(amount * sign), foreAxis)
                    * osg::Matrixf::translate(elbow);
                const auto candidate = evaluate(shared);
                if (candidate.metrics.score() < result.metrics.score())
                    result = candidate;
            }
        }
        return result;
    }
}

#endif
