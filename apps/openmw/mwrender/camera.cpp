#include "camera.hpp"

#include <osg/Camera>
#include <cmath>

#include <components/misc/mathutil.hpp>
#include <components/settings/values.hpp>
#include <components/sceneutil/nodecallback.hpp>
#include <components/sceneutil/positionattitudetransform.hpp>

#include "../mwmechanics/brutaloptions.hpp"
#include "../mwmechanics/creaturestats.hpp"
#include "../mwbase/environment.hpp"
#include "../mwbase/windowmanager.hpp"
#include "../mwbase/world.hpp"

#include "../mwworld/class.hpp"
#include "../mwworld/datetimemanager.hpp"
#include "../mwworld/ptr.hpp"
#include "../mwworld/refdata.hpp"

#include "../mwmechanics/movement.hpp"

#include "../mwphysics/raycasting.hpp"

#include "npcanimation.hpp"

namespace
{
    struct BodyClearance
    {
        std::string_view start, end;
        double radius;
    };

    // Keep the near plane clear of the posed body, including the space
    // occupied by clothing over the limbs. The head itself is hidden.
    constexpr BodyClearance sDeathBodyClearance[]{
        { "Bip01 Neck", "Bip01 Head", 6. },
        { "Bip01 Spine1", "Bip01 Neck", 12. },
        { "Bip01 Pelvis", "Bip01 Spine1", 12. },
        { "Bip01 L UpperArm", "Bip01 L Forearm", 8. },
        { "Bip01 R UpperArm", "Bip01 R Forearm", 8. },
        { "Bip01 L Forearm", "Bip01 L Hand", 8. },
        { "Bip01 R Forearm", "Bip01 R Hand", 8. },
        { "Bip01 L Thigh", "Bip01 L Calf", 8. },
        { "Bip01 R Thigh", "Bip01 R Calf", 8. },
        { "Bip01 L Calf", "Bip01 L Foot", 8. },
        { "Bip01 R Calf", "Bip01 R Foot", 8. },
    };

    osg::Vec3d clearDeathBody(const MWRender::NpcAnimation& animation, const osg::Vec3d& eye,
        const osg::Vec3d& up, double scale)
    {
        osg::Vec3d result = eye;
        for (const auto& part : sDeathBodyClearance)
        {
            const auto start = animation.getBoneWorldMatrix(part.start);
            const auto end = animation.getBoneWorldMatrix(part.end);
            if (!start || !end)
                continue;
            const osg::Vec3d a(start->getTrans());
            const osg::Vec3d axis = osg::Vec3d(end->getTrans()) - a;
            const double along = std::clamp(((result - a) * axis) / std::max(axis.length2(), .001), 0., 1.);
            osg::Vec3d normal = result - (a + axis * along);
            const double distance = normal.normalize();
            const double clearance = part.radius * scale + 2.;
            if (distance < clearance)
                result += (distance > .001 ? normal : up) * (clearance - distance);
        }
        return result;
    }

    osg::Quat boneRotation(const osg::Matrixf& matrix)
    {
        osg::Vec3f position, scale;
        osg::Quat orientation, stretch;
        matrix.decompose(position, orientation, scale, stretch);
        return orientation;
    }

    double limitDeathNeckPitch(const osg::Quat& eyeOrientation, osg::Vec3d towardNeck, double droop)
    {
        if (towardNeck.normalize() <= .001)
            return 0.;
        const double across = towardNeck * (eyeOrientation * osg::Vec3d(0., 1., 0.));
        const double down = towardNeck * (eyeOrientation * osg::Vec3d(0., 0., -1.));
        const double projected = std::hypot(across, down);
        // Stop the nod before the neck enters the view cone in front of the eyes.
        const double limit = std::cos(osg::DegreesToRadians(65.));
        if (projected <= limit)
            return droop;
        const double center = std::atan2(down, across);
        const double halfSpan = std::acos(limit / projected);
        const double first = center - halfSpan, last = center + halfSpan;
        if (first <= droop && last > 0.)
            return std::max(first, osg::DegreesToRadians(-20.));
        return droop;
    }


    class UpdateRenderCameraCallback : public SceneUtil::NodeCallback<UpdateRenderCameraCallback, osg::Camera*>
    {
    public:
        UpdateRenderCameraCallback(MWRender::Camera* cam)
            : mCamera(cam)
        {
        }

        void operator()(osg::Camera* cam, osg::NodeVisitor* nv)
        {
            // traverse first to update animations, in case the camera is attached to an animated node
            traverse(cam, nv);

            mCamera->updateCamera(cam);
        }

    private:
        MWRender::Camera* mCamera;
    };

}

namespace MWRender
{

    Camera::Camera(osg::Camera* camera)
        : mHeightScale(1.f)
        , mCollisionType(
              (MWPhysics::CollisionType::CollisionType_Default & ~MWPhysics::CollisionType::CollisionType_Actor)
              | MWPhysics::CollisionType_CameraOnly)
        , mCamera(camera)
        , mAnimation(nullptr)
        , mFirstPersonView(true)
        , mMode(Mode::FirstPerson)
        , mVanityAllowed(true)
        , mDeferredRotationAllowed(true)
        , mProcessViewChange(false)
        , mHeight(124.f)
        , mPitch(0.f)
        , mYaw(0.f)
        , mRoll(0.f)
        , mCameraDistance(0.f)
        , mPreferredCameraDistance(0.f)
        , mFocalPointCurrentOffset(osg::Vec2d())
        , mFocalPointTargetOffset(osg::Vec2d())
        , mFocalPointTransitionSpeedCoef(1.f)
        , mSkipFocalPointTransition(true)
        , mPreviousTransitionInfluence(0.f)
        , mShowCrosshair(false)
        , mDeferredRotation(osg::Vec3f())
        , mDeferredRotationDisabled(false)
    {
        mUpdateCallback = new UpdateRenderCameraCallback(this);
        mCamera->addUpdateCallback(mUpdateCallback);
    }

    Camera::~Camera()
    {
        mCamera->removeUpdateCallback(mUpdateCallback);
    }

    osg::Vec3d Camera::calculateTrackedPosition() const
    {
        if (!mTrackingNode)
            return osg::Vec3d();
        osg::NodePathList nodepaths = mTrackingNode->getParentalNodePaths();
        if (nodepaths.empty())
            return osg::Vec3d();
        osg::Matrix worldMat = osg::computeLocalToWorld(nodepaths[0]);
        osg::Vec3d res = worldMat.getTrans();
        if (mMode != Mode::FirstPerson || (mAnimation && mAnimation->useFullBodyFirstPerson()))
            res.z() += mHeight * mHeightScale;
        if (mMode == Mode::FirstPerson && mAnimation && mAnimation->useFullBodyFirstPerson())
            res.z() -= mFullBodySneakOffset;
        return res;
    }

    osg::Vec3d Camera::getFocalPointOffset() const
    {
        osg::Vec3d offset;
        offset.x() = mFocalPointCurrentOffset.x() * cos(mYaw);
        offset.y() = mFocalPointCurrentOffset.x() * sin(mYaw);
        offset.z() = mFocalPointCurrentOffset.y();
        return offset;
    }

    void Camera::updateCamera(osg::Camera* cam)
    {
        osg::Quat orient = getOrient();
        osg::Vec3d forward = orient * osg::Vec3d(0, 1, 0);
        osg::Vec3d up = orient * osg::Vec3d(0, 0, 1);

        osg::Vec3d pos = mPosition;
        if (mMode == Mode::FirstPerson && !mPlayerDeathView)
        {
            // It is a hack. Camera position depends on neck animation.
            // Animations are updated in OSG cull traversal and in order to avoid 1 frame delay we
            // recalculate the position here. Note that it becomes different from mPosition that
            // is used in other parts of the code.
            // TODO: detach camera from OSG animation and get rid of this hack.
            osg::Vec3d recalculatedTrackedPosition = calculateTrackedPosition();
            pos = calculateFirstPersonPosition(recalculatedTrackedPosition);
        }
        cam->setViewMatrixAsLookAt(pos, pos + forward, up);
        mViewMatrix = cam->getViewMatrix();
        mProjectionMatrix = cam->getProjectionMatrix();
    }

    void Camera::update(float duration, bool paused)
    {
        mLockPitch = mLockYaw = false;
        MWBase::WindowManager* wm = MWBase::Environment::get().getWindowManager();
        if (mPlayerDeathView)
        {
            wm->showCrosshair(false);
            if (!paused)
                updatePlayerDeathView();
            return;
        }

        if (mQueuedMode && mAnimation->upperBodyReady())
            setMode(*mQueuedMode);
        if (mProcessViewChange)
            processViewChange();

        // only show the crosshair in game mode
        wm->showCrosshair(!wm->isGuiMode() && mShowCrosshair);

        if (!paused)
            updateFocalPointOffset(duration);
        updatePosition();
    }

    void Camera::setPlayerDeathTime(std::optional<float> elapsed)
    {
        if (!elapsed)
        {
            mProcessViewChange |= mPlayerDeathView.has_value();
            mPlayerDeathView.reset();
            return;
        }
        if (!mPlayerDeathView)
        {
            const osg::Quat orientation = getOrient();
            osg::Vec3d eyeInHead;
            osg::Quat orientationInHead = orientation;
            if (const auto head = mAnimation ? mAnimation->getBoneWorldMatrix("Bip01 Head") : std::nullopt)
            {
                osg::Vec3d eye = mPosition;
                const osg::Vec3d fromHead = eye - osg::Vec3d(head->getTrans());
                if (mMode != Mode::FirstPerson)
                {
                    const osg::Quat heading(mYaw + mExtraYaw, osg::Vec3d(0, 0, 1));
                    eye = osg::Vec3d(head->getTrans()) + heading * osg::Vec3d(0, 5, 8);
                }
                else if (fromHead.length2() > 24. * 24.)
                    eye = osg::Vec3d(head->getTrans()) + fromHead * (24. / fromHead.length());
                // Convert the existing eye, including its gameplay offsets, to the
                // death rig before Bullet advances it. The anchor moves with the head.
                eyeInHead = eye * osg::Matrixd::inverse(*head);
                orientationInHead = orientation * boneRotation(*head).inverse();
            }
            const float entryPitch = mPitch + mExtraPitch;
            const float pitchDrop = std::clamp(entryPitch + osg::DegreesToRadians(83.f),
                0.f, osg::DegreesToRadians(42.f));
            mPlayerDeathView.emplace(PlayerDeathView{ orientationInHead, orientation, eyeInHead, pitchDrop });
            mCameraDistance = 0.f;
            mQueuedMode.reset();
        }
        mPlayerDeathView->elapsed = *elapsed;
    }

    void Camera::updatePlayerDeathView()
    {
        if (mTrackingPtr.isEmpty() || !mAnimation)
            return;

        auto& death = *mPlayerDeathView;
        const float t = std::clamp(death.elapsed / .72f, 0.f, 1.f);
        double droop = death.pitchDrop * t * t * (3.f - 2.f * t);
        if (const auto head = mAnimation->getBoneWorldMatrix("Bip01 Head"))
        {
            mTrackedPosition = head->getTrans();
            const osg::Vec3d eye = death.eyeInHead * osg::Matrixd(*head);
            const osg::Vec3d up(0., 0., 1.);
            mPosition = clearDeathBody(*mAnimation, eye, up, mHeightScale);
            osg::Vec3d correction = mPosition - eye;
            constexpr double maxBodyCorrection = 10.;
            if (correction.normalize() > maxBodyCorrection)
                mPosition = eye + correction * maxBodyCorrection;

            const auto* rays = MWBase::Environment::get().getWorld()->getRayCasting();
            constexpr float radius = 3.f;
            constexpr int worldGeometry = (MWPhysics::CollisionType_Default & ~MWPhysics::CollisionType_Actor)
                | MWPhysics::CollisionType_CameraOnly;
            const auto findFloor = [&](const osg::Vec3d& point) {
                return rays->castRay(point + up * 12., point - up * 36., worldGeometry);
            };
            const auto floor = findFloor(mPosition);
            const bool hasFloor = floor.mHit && floor.mHitNormal.z() > .5f;
            if (hasFloor)
                mPosition.z() = std::max(mPosition.z(), static_cast<double>(floor.mHitPos.z()) + 6.);

            // Begin above the surface if the head bone itself has settled
            // into the floor. Sweep the final eye placement against walls.
            osg::Vec3d anchor = mTrackedPosition;
            const auto anchorFloor = findFloor(anchor);
            if (anchorFloor.mHit && anchorFloor.mHitNormal.z() > .5f)
                anchor.z() = std::max(anchor.z(), static_cast<double>(anchorFloor.mHitPos.z()) + 6.);
            const osg::Vec3d offset = mPosition - anchor;
            if (offset.length2() > .001)
            {
                const auto hit = rays->castSphere(anchor, mPosition, radius, worldGeometry);
                if (hit.mHit)
                {
                    const osg::Vec3d safe = osg::Vec3d(hit.mHitPos + hit.mHitNormal * radius) - anchor;
                    mPosition = anchor + offset * std::clamp((safe * offset) / offset.length2(), 0., 1.);
                }
            }

            const osg::Quat eyes = death.orientationInHead * boneRotation(*head);
            if (hasFloor)
            {
                const double height = mPosition.z() - floor.mHitPos.z();
                const double room = std::clamp((height - 6.) / 30., 0., 1.);
                const double limit = osg::DegreesToRadians(14. + 28. * room * room * (3. - 2. * room));
                droop = std::min(droop, limit);
            }
            if (const auto neck = mAnimation->getBoneWorldMatrix("Bip01 Neck"))
                droop = limitDeathNeckPitch(eyes, osg::Vec3d(neck->getTrans()) - mPosition, droop);

            death.orientation = osg::Quat(-droop, osg::Vec3d(1., 0., 0.)) * eyes;
        }
    }

    osg::Vec3d Camera::calculateFirstPersonPosition(const osg::Vec3d& trackedPosition) const
    {
        osg::Vec3d res = trackedPosition;
        const bool fullBody = mAnimation && mAnimation->useFullBodyFirstPerson();
        const float forward = fullBody ? Settings::game().mFirstPersonBodyForward.get() * mHeightScale : 0.f;
        osg::Vec2f horizontalOffset
            = Misc::rotateVec2f(osg::Vec2f(mFirstPersonOffset.x(), mFirstPersonOffset.y() + forward), mYaw);
        res.x() += horizontalOffset.x();
        res.y() += horizontalOffset.y();
        res.z() += mFirstPersonOffset.z();
        if (fullBody && (res - trackedPosition).length2() > 0.001)
        {
            // Keep the eye outside the chest without letting it look through nearby walls.
            const auto* rays = MWBase::Environment::get().getWorld()->getRayCasting();
            const auto hit = rays->castSphere(trackedPosition, res, 2.f, mCollisionType);
            if (hit.mHit)
            {
                const osg::Vec3d delta = res - trackedPosition;
                const osg::Vec3d safe = osg::Vec3d(hit.mHitPos + hit.mHitNormal * 2.f) - trackedPosition;
                res = trackedPosition + delta * std::clamp((safe * delta) / delta.length2(), 0.0, 1.0);
            }
        }
        return res;
    }

    void Camera::updatePosition()
    {
        mTrackedPosition = calculateTrackedPosition();
        if (mMode == Mode::Static)
            return;
        if (mMode == Mode::FirstPerson)
        {
            mPosition = calculateFirstPersonPosition(mTrackedPosition);
            mCameraDistance = 0;
            return;
        }

        constexpr float cameraObstacleLimit = 5.0f;
        constexpr float focalObstacleLimit = 10.f;

        const auto* rayCasting = MWBase::Environment::get().getWorld()->getRayCasting();

        // Adjust focal point to prevent clipping.
        osg::Vec3d focalOffset = getFocalPointOffset();
        osg::Vec3d focal = mTrackedPosition + focalOffset;
        focalOffset.z() += 10.f; // Needed to avoid camera clipping through the ceiling because
                                 // character's head can be a bit higher than the collision area.
        double offsetLen = focalOffset.length();
        if (offsetLen > 0)
        {
            MWPhysics::RayCastingResult result
                = rayCasting->castSphere(focal - focalOffset, focal, focalObstacleLimit, mCollisionType);
            if (result.mHit)
            {
                double adjustmentCoef
                    = -(result.mHitPos + result.mHitNormal * focalObstacleLimit - focal).length() / offsetLen;
                focal += focalOffset * std::max(-1.0, adjustmentCoef);
            }
        }

        // Adjust camera distance.
        mCameraDistance = mPreferredCameraDistance;
        osg::Quat orient
            = osg::Quat(mPitch + mExtraPitch, osg::Vec3d(1, 0, 0)) * osg::Quat(mYaw + mExtraYaw, osg::Vec3d(0, 0, 1));
        osg::Vec3d offset = orient * osg::Vec3d(0.f, -mCameraDistance, 0.f);
        MWPhysics::RayCastingResult result
            = rayCasting->castSphere(focal, focal + offset, cameraObstacleLimit, mCollisionType);
        if (result.mHit)
        {
            mCameraDistance = (result.mHitPos + result.mHitNormal * cameraObstacleLimit - focal).length();
            offset = orient * osg::Vec3d(0.f, -mCameraDistance, 0.f);
        }

        mPosition = focal + offset;
    }

    osg::Quat Camera::getOrient() const
    {
        if (mPlayerDeathView)
            return mPlayerDeathView->orientation;
        return osg::Quat(mRoll + mExtraRoll, osg::Vec3d(0, 1, 0)) * osg::Quat(mPitch + mExtraPitch, osg::Vec3d(1, 0, 0))
            * osg::Quat(mYaw + mExtraYaw, osg::Vec3d(0, 0, 1));
    }

    void Camera::setMode(Mode newMode, bool force)
    {
        if (mPlayerDeathView)
            return;
        // Lua camera actions call setMode directly, bypassing toggleViewMode.
        // Static remains available for explicit scripted cameras.
        if ((Settings::game().mFirstPersonFullBody || MWMechanics::brutalDmFirstPerson()) && !Settings::game().mFirstPersonBodyAllowThirdPerson
            && (newMode == Mode::ThirdPerson || newMode == Mode::Preview || newMode == Mode::Vanity))
        {
            const bool classicDeath = newMode == Mode::ThirdPerson && !MWMechanics::brutalPlayerDeathScene()
                && !mTrackingPtr.isEmpty() && mTrackingPtr == MWBase::Environment::get().getWorld()->getPlayerPtr()
                && mTrackingPtr.getClass().getCreatureStats(mTrackingPtr).isDead();
            if (!classicDeath)
                newMode = Mode::FirstPerson;
        }
        if (mMode == newMode)
        {
            mQueuedMode = std::nullopt;
            return;
        }
        Mode oldMode = mMode;
        if (!force && (newMode == Mode::FirstPerson || oldMode == Mode::FirstPerson) && mAnimation
            && !mAnimation->upperBodyReady())
        {
            // Changing the view will stop all playing animations, so if we are playing
            // anything important, queue the view change for later
            mQueuedMode = newMode;
            return;
        }
        mMode = newMode;
        mQueuedMode = std::nullopt;
        if (newMode == Mode::FirstPerson)
            mFirstPersonView = true;
        else if (newMode == Mode::ThirdPerson)
            mFirstPersonView = false;
        calculateDeferredRotation();
        if (oldMode == Mode::FirstPerson || newMode == Mode::FirstPerson)
        {
            instantTransition();
            mProcessViewChange = true;
        }
    }

    void Camera::setFocalPointTargetOffset(const osg::Vec2d& v)
    {
        mFocalPointTargetOffset = v;
        mPreviousTransitionSpeed = mFocalPointTransitionSpeed;
        mPreviousTransitionInfluence = 1.0f;
    }

    void Camera::updateFocalPointOffset(float duration)
    {
        if (duration <= 0)
            return;

        if (mSkipFocalPointTransition)
        {
            mSkipFocalPointTransition = false;
            mPreviousExtraOffset = osg::Vec2d();
            mPreviousTransitionInfluence = 0.f;
            mFocalPointCurrentOffset = mFocalPointTargetOffset;
        }

        osg::Vec2d oldOffset = mFocalPointCurrentOffset;

        if (mPreviousTransitionInfluence > 0)
        {
            mFocalPointCurrentOffset -= mPreviousExtraOffset;
            mPreviousExtraOffset
                = mPreviousExtraOffset / mPreviousTransitionInfluence + mPreviousTransitionSpeed * duration;
            mPreviousTransitionInfluence
                = std::max(0.f, mPreviousTransitionInfluence - duration * mFocalPointTransitionSpeedCoef);
            mPreviousExtraOffset *= mPreviousTransitionInfluence;
            mFocalPointCurrentOffset += mPreviousExtraOffset;
        }

        osg::Vec2d delta = mFocalPointTargetOffset - mFocalPointCurrentOffset;
        if (delta.length2() > 0)
        {
            double coef = duration * (1.0 + 5.0 / delta.length()) * mFocalPointTransitionSpeedCoef
                * (1.0f - mPreviousTransitionInfluence);
            mFocalPointCurrentOffset += delta * std::min(coef, 1.0);
        }
        else
        {
            mPreviousExtraOffset = osg::Vec2d();
            mPreviousTransitionInfluence = 0.f;
        }

        mFocalPointTransitionSpeed = (mFocalPointCurrentOffset - oldOffset) / duration;
    }

    void Camera::toggleViewMode(bool force)
    {
        if ((Settings::game().mFirstPersonFullBody || MWMechanics::brutalDmFirstPerson()) && !Settings::game().mFirstPersonBodyAllowThirdPerson)
            return;
        setMode(mFirstPersonView ? Mode::ThirdPerson : Mode::FirstPerson, force);
    }

    bool Camera::toggleVanityMode(bool enable)
    {
        if (enable && (Settings::game().mFirstPersonFullBody || MWMechanics::brutalDmFirstPerson()) && !Settings::game().mFirstPersonBodyAllowThirdPerson)
            return false;
        if (!enable)
            setMode(mFirstPersonView ? Mode::FirstPerson : Mode::ThirdPerson, false);
        else if (mVanityAllowed)
            setMode(Mode::Vanity, false);
        return (mMode == Mode::Vanity) == enable;
    }

    void Camera::setSneakOffset(float offset, float slideBlend)
    {
        mSneakTarget = offset;
        const auto world = MWBase::Environment::get().getWorld();
        // Use the same clock as NIF body fades: menus freeze both visuals, and
        // slow time cannot let the camera finish while the pose is still moving.
        const double now = world->getTimeManager()->getRenderingSimulationTime();
        const float duration = static_cast<float>(std::clamp(now - mSneakVisualTime, 0., .2));
        mSneakVisualTime = now;
        const bool smooth = mAnimation && mMode == Mode::FirstPerson && !mAnimation->useFullBodyFirstPerson()
            && MWMechanics::brutalDmFirstPerson() && slideBlend == 0.f
            && mTrackingPtr == world->getPlayerPtr()
            && mTrackingPtr.getClass().getCreatureStats(mTrackingPtr).mBrutalSlidePhase == 0
            && !mTrackingPtr.getClass().getCreatureStats(mTrackingPtr).mBrutalSlideBlocked;
        if (!smooth || !mVisualSneakInitialized)
        {
            mVisualSneakOffset = offset;
            mVisualSneakVelocity = 0.f;
            mVisualSneakInitialized = true;
        }
        else if (duration > 0.f)
        {
            // Exact critically damped response. Reversing the requested stance keeps
            // the visible position and velocity; actor stance/collision change immediately.
            constexpr float omega = 22.f;
            const float delta = mVisualSneakOffset - offset;
            const float decay = std::exp(-omega * duration);
            const float step = (mVisualSneakVelocity + omega * delta) * duration;
            mVisualSneakOffset = offset + (delta + step) * decay;
            mVisualSneakVelocity = (mVisualSneakVelocity - omega * step) * decay;
            if (std::abs(mVisualSneakOffset - offset) < .0001f && std::abs(mVisualSneakVelocity) < .001f)
            {
                mVisualSneakOffset = offset;
                mVisualSneakVelocity = 0.f;
            }
        }
        // With the visible first-person body, the eye rides on that body's torso
        // (NpcAnimation adds its displacement to the eye and arms): its crouch,
        // sneak walk and stance changes lower the view exactly as far and as fast as
        // the torso the player sees. A fixed sneak drop would leave the view
        // standing over a crouched body, or sink it into an upright one.
        const bool bodyEye = mAnimation && mMode == Mode::FirstPerson && mAnimation->hasBodyEyeFollow()
            && mTrackingPtr == world->getPlayerPtr();
        if (bodyEye)
        {
            mVisualSneakOffset = 0.f;
            mVisualSneakVelocity = 0.f;
        }
        // The authored slide eye already has its own blend. Preserve it exactly.
        mFullBodySneakOffset = mVisualSneakOffset * (1.f - slideBlend) + 70.f * slideBlend;
        if (mAnimation)
        {
            mAnimation->setBodyEyeFollowWeight(1.f - slideBlend);
            mAnimation->setFirstPersonOffset(osg::Vec3f(0, 0, -mFullBodySneakOffset));
        }
    }

    void Camera::setYaw(float angle, bool force)
    {
        if (!mLockYaw || force)
            mYaw = static_cast<float>(Misc::normalizeAngle(angle));
        if (force)
            mLockYaw = true;
    }

    void Camera::setPitch(float angle, bool force)
    {
        const float epsilon = 0.000001f;
        float limit = static_cast<float>(osg::PI_2) - epsilon;
        if (!mLockPitch || force)
            mPitch = std::clamp(angle, -limit, limit);
        if (force)
            mLockPitch = true;
    }

    void Camera::setStaticPosition(const osg::Vec3d& pos)
    {
        if (mMode != Mode::Static)
            throw std::runtime_error("setStaticPosition can be used only if camera is in Static mode");
        mPosition = pos;
    }

    void Camera::setAnimation(NpcAnimation* anim)
    {
        mAnimation = anim;
        mProcessViewChange = true;
    }

    void Camera::processViewChange()
    {
        if (mTrackingPtr.isEmpty())
            return;
        if (mMode == Mode::FirstPerson)
        {
            mAnimation->setViewMode(NpcAnimation::VM_FirstPerson);
            if (mAnimation->useFullBodyFirstPerson())
            {
                // The real skeleton can run, kick and swing independently of the player's view.
                // Tracking the actor root avoids neck-animation bobbing and attack-induced camera jumps.
                SceneUtil::PositionAttitudeTransform* transform = mTrackingPtr.getRefData().getBaseNode();
                mTrackingNode = transform;
                mHeightScale = transform ? transform->getScale().z() : 1.f;
            }
            else
            {
                mTrackingNode = mAnimation->getNode("Camera");
                if (!mTrackingNode)
                    mTrackingNode = mAnimation->getNode("Head");
                mHeightScale = 1.f;
            }
        }
        else
        {
            mAnimation->setViewMode(NpcAnimation::VM_Normal);
            SceneUtil::PositionAttitudeTransform* transform = mTrackingPtr.getRefData().getBaseNode();
            mTrackingNode = transform;
            if (transform)
                mHeightScale = transform->getScale().z();
            else
                mHeightScale = 1.f;
        }
        mProcessViewChange = false;
    }

    void Camera::applyDeferredPreviewRotationToPlayer(float dt)
    {
        if (mMode != Mode::ThirdPerson || mTrackingPtr.isEmpty())
            return;

        osg::Vec3f rot = mDeferredRotation;
        float delta = rot.normalize();
        delta = std::min(delta, (delta + 1.f) * 3 * dt);
        rot *= delta;
        mDeferredRotation -= rot;

        if (mDeferredRotationDisabled)
        {
            mDeferredRotationDisabled = delta > 0.0001;
            rotateCameraToTrackingPtr();
            return;
        }

        auto& movement = mTrackingPtr.getClass().getMovementSettings(mTrackingPtr);
        movement.mRotation[0] += rot.x();
        movement.mRotation[1] += rot.y();
        movement.mRotation[2] += rot.z();
        if (std::abs(mDeferredRotation.z()) > 0.0001)
        {
            float s = std::sin(mDeferredRotation.z());
            float c = std::cos(mDeferredRotation.z());
            float x = movement.mPosition[0];
            float y = movement.mPosition[1];
            movement.mPosition[0] = x * c + y * s;
            movement.mPosition[1] = x * -s + y * c;
        }
    }

    void Camera::rotateCameraToTrackingPtr()
    {
        if (mMode == Mode::Static || mTrackingPtr.isEmpty())
            return;
        setPitch(-mTrackingPtr.getRefData().getPosition().rot[0] - mDeferredRotation.x());
        setYaw(-mTrackingPtr.getRefData().getPosition().rot[2] - mDeferredRotation.z());
    }

    void Camera::instantTransition()
    {
        mVisualSneakOffset = mSneakTarget;
        mVisualSneakVelocity = 0.f;
        mVisualSneakInitialized = false;

        mSkipFocalPointTransition = true;
        mDeferredRotationDisabled = false;
        mDeferredRotation = osg::Vec3f();
        rotateCameraToTrackingPtr();
    }

    void Camera::calculateDeferredRotation()
    {
        if (mMode == Mode::Static)
        {
            mDeferredRotation = osg::Vec3f();
            return;
        }
        MWWorld::Ptr ptr = mTrackingPtr;
        if (mMode == Mode::Preview || mMode == Mode::Vanity || ptr.isEmpty())
            return;
        if (mFirstPersonView)
        {
            instantTransition();
            return;
        }

        mDeferredRotation.x()
            = static_cast<float>(Misc::normalizeAngle(-ptr.getRefData().getPosition().rot[0] - mPitch));
        mDeferredRotation.z() = static_cast<float>(Misc::normalizeAngle(-ptr.getRefData().getPosition().rot[2] - mYaw));
        if (!mDeferredRotationAllowed)
            mDeferredRotationDisabled = true;
    }

}
