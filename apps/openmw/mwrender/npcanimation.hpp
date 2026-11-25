#ifndef GAME_RENDER_NPCANIMATION_H
#define GAME_RENDER_NPCANIMATION_H

#include "actoranimation.hpp"
#include "animation.hpp"
#include "npcblood.hpp"
#include "weaponanimation.hpp"

#include <components/esm3/refnum.hpp>
#include <components/vfs/pathutil.hpp>

#include "../mwworld/inventorystore.hpp"

#include <array>
#include <cstdint>
#include <functional>
#include <optional>
#include <osg/observer_ptr>

namespace ESM
{
    struct NPC;
    struct BodyPart;
    struct ReleasedEquipmentState;
}

namespace MWSound
{
    class Sound;
}

namespace MWPhysics
{
    struct HeldObjectGrip;
}

namespace MWRender
{

    class RotateController;
    class HeadAnimationTime;
    struct DecapitationGeometry;
    struct BisectionGeometry;
    struct LimbSeveringGeometry;
    class ClippedRenderPrewarm;

    class NpcAnimation : public ActorAnimation, public WeaponAnimation, public MWWorld::InventoryStoreListener
    {
    public:
        void equipmentChanged() override;

    public:
        typedef std::map<ESM::PartReferenceType, std::string> PartBoneMap;

        enum ViewMode
        {
            VM_Normal,
            VM_FirstPerson,
            VM_HeadOnly
        };

        enum class FirstPersonBody
        {
            None,
            Visible,
            Shadow
        };

        struct SwordWallContact
        {
            bool armed = false, pending = false, soundPlayed = false;
            std::uint64_t count = 0, swing = 0, sounds = 0;
            unsigned int frame = 0, samples = 0;
            float sourceTime = 0.f, sweepFraction = 0.f;
            unsigned int sweepSegments = 0, sweepCandidates = 0, sweepConfirmed = 0;
            float sweepMilliseconds = 0.f;
            std::string key, kind, mount;
            osg::Vec3f position, normal, previousBase, previousTip, bladeBase, bladeTip, sweepFrom, sweepTo;
            osg::Vec3f currentBase, currentTip, currentContactPoint;
        };

        struct NpcGuardMeshTrace
        {
            unsigned int frame = 0;
            bool hit = false;
            float gap = 0.f;
            osg::Vec3f point;
        };

        struct NpcGuardRigAudit
        {
            unsigned frame = 0;
            std::map<std::string, osg::Matrixf> matrices;
            std::map<std::string, osg::Vec3f> vectors;
            std::map<std::string, float> scalars;
        };

        struct NpcGuardGeometry
        {
            bool enabled = false, active = false, shield = false, fitted = false;
            bool twoHanded = false, leftNatural = false;
            bool clamped = false, meshAvailable = false;
            bool jointLimited = false;
            bool releasing = false;
            bool recovering = false;
            bool cacheUsed = false, cacheNegative = false, fullSearch = false;
            bool sourceGroupMatches = false, sourceBlendActive = false, sourceHoldReached = false;
            bool referenceReady = false;
            bool goalNatural = false, goalWorking = false, goalFromSafe = false, pathAccepted = false;
            unsigned int frame = 0;
            unsigned int searchCandidates = 0;
            float weight = 0.f, contactError = 0.f, reachClamp = 0.f;
            float shoulderError = 0.f, gripError = 0.f;
            float sourceUpperLength = 0.f, sourceForeLength = 0.f;
            float finalUpperLength = 0.f, finalForeLength = 0.f;
            float wristSwingDegrees = 0.f, wristTwistDegrees = 0.f;
            float palmFlexDegrees = 0.f, palmSideDegrees = 0.f;
            float forearmTwistDegrees = 0.f, upperSwingDegrees = 0.f;
            float upperTwistDegrees = 0.f, signedElbowFlexDegrees = 0.f, elbowHingeErrorDegrees = 0.f;
            float totalPronationDegrees = 0.f, releaseTimeLeft = 0.f, overlayCorrectionDegrees = 0.f;
            float goalContactError = 0.f, goalTargetError = 0.f, goalReachClamp = 0.f, pathFraction = 0.f;
            float goalWristSwingDegrees = 0.f, goalWristTwistDegrees = 0.f;
            float goalPalmFlexDegrees = 0.f, goalPalmSideDegrees = 0.f;
            float goalForearmTwistDegrees = 0.f, goalUpperSwingDegrees = 0.f, goalUpperTwistDegrees = 0.f;
            float goalSignedElbowFlexDegrees = 0.f, goalElbowHingeErrorDegrees = 0.f;
            float goalBodyClearance = 0.f, goalInwardError = 0.f, goalStanceMargin = 0.f;
            float sourceTime = 0.f, sourceHoldTime = 0.f;
            float sourceUpperLengthUnscaled = 0.f, sourceForeLengthUnscaled = 0.f;
            float finalUpperLengthUnscaled = 0.f, finalForeLengthUnscaled = 0.f;
            float maxLocalOrthoError = 0.f, maxLocalTranslationError = 0.f;
            float bodyClearance = 0.f;
            float elbowFlexDegrees = 0.f;
            float upperLocalOrthoError = 0.f, foreLocalOrthoError = 0.f, handLocalOrthoError = 0.f;
            float shieldCenterAbovePelvis = 0.f, shieldForwardDistance = 0.f, shieldTiltDegrees = 0.f;
            float leftGripError = 0.f, leftShoulderError = 0.f;
            float leftSourceUpperLengthUnscaled = 0.f, leftSourceForeLengthUnscaled = 0.f;
            float leftFinalUpperLengthUnscaled = 0.f, leftFinalForeLengthUnscaled = 0.f;
            float leftMaxLocalOrthoError = 0.f, leftMaxLocalTranslationError = 0.f;
            float leftWristSwingDegrees = 0.f, leftWristTwistDegrees = 0.f;
            float leftPalmFlexDegrees = 0.f, leftPalmSideDegrees = 0.f;
            float leftForearmTwistDegrees = 0.f, leftUpperSwingDegrees = 0.f, leftUpperTwistDegrees = 0.f;
            float leftSignedElbowFlexDegrees = 0.f, leftElbowHingeErrorDegrees = 0.f, leftBodyClearance = 0.f;
            osg::Vec3f requestedTarget, interceptPoint, sourceContact, itemContact;
            osg::Vec3f handTarget, sourceWrist, wrist, shoulder, elbow;
            osg::Vec3f itemAxis, requestedAxis, materialPoint;
            osg::Vec3f palmForward, palmLateral, palmNormal;
            osg::Vec3f skeletonShoulder, skeletonElbow, skeletonWrist;
            osg::Vec3f shieldCenter;
            osg::Vec3f goalWrist, goalContact;
            osg::Vec3f guardPelvis, guardTorso, shieldBottom, shieldTop, shieldBackPoint, shieldFrontPoint;
            osg::Vec3f renderActorOrigin, renderActorRight, renderActorForward, renderActorUp;
            osg::Vec3f leftSourceWrist, leftShoulder, leftElbow, leftWrist;
        };

        // Read-only controller snapshot for comparing real input and rendered
        // weapon lifecycles. It never selects or advances an animation.
        struct WeaponCycleState
        {
            std::string_view phase = "none";
            int index = 0, upperBody = 0;
            bool queued = false, button = false, readyToHit = false;
            float held = 0.f;
            bool chargedHold = false, guardCharge = false;
            float useHeld = 0.f;
            std::uint64_t acceptedStrikes = 0;
        };

    private:
        static const PartBoneMap sPartList;

        // Opt-in private geometry audit. The callback samples after the whole
        // object update traversal, after the rendered ragdoll bones are assigned.
        struct GarmentAudit;
        std::unique_ptr<GarmentAudit> mGarmentAudit;
        void updateRagdollGarmentAudit(float timepassed);
        void dumpRagdollGarmentForAudit(osg::NodeVisitor& visitor);
        struct GarmentCoverage;
        struct BloodDecals;
        std::shared_ptr<BloodDecals> mBloodDecals;
        struct CutBloodFlow;
        std::shared_ptr<CutBloodFlow> mCutBloodFlow;
        struct WeaponBloodView;
        std::shared_ptr<WeaponBloodView> mWeaponBloodView;
        void updateWeaponBlood(float dt);
        void updateCutBloodFlow(float dt);
        void emitCutBloodFlow(osg::NodeVisitor& visitor);
        void clearCutBloodFlow();
        void rebuildBloodDecals();
        bool useBrutalStaffSet() const override;
        void prewarmBloodDecals();
        void transferBloodDecalsToHead(DecapitationGeometry& target, bool bindSources = false);
        void transferBloodDecalsToTorso(BisectionGeometry& target);
        void transferBloodDecalsToLimbs(LimbSeveringGeometry& target);
        std::unique_ptr<GarmentCoverage> mGarmentCoverage;
        bool mGarmentCoveragePrepared = false, mGarmentWasRagdoll = false;
        double mGarmentRagdollAge = 0.;
        void clearRagdollGarmentCoverage();
        void prepareRagdollGarmentCoverage();
        void updateRagdollGarmentCoverage(float timepassed);
        void evaluateRagdollGarmentCoverage(osg::NodeVisitor& visitor);

        std::unique_ptr<DecapitationGeometry> mDecapitation;
        std::unique_ptr<BisectionGeometry> mBisection;
        std::unique_ptr<LimbSeveringGeometry> mLimbSevering;
        std::array<bool, ESM::PRT_Count> mLimbReplacedParts{};
        int mLimbSeveringFailure = 0;
        bool rebuildSeveredLimbs(unsigned mask);
        int mBisectionFailure = 0;
        bool rebuildSeveredTorso();
        bool rebuildSeveredHead(bool prewarmOnly = false);
        int mDecapitationFailure = 0;
        osg::ref_ptr<osg::StateSet> mDecapitationCapState;
        osg::ref_ptr<osg::StateSet> mDecapitationHeadCapState;
        bool prewarmDecapitationMaterial();
        std::size_t mDecapitationPrewarmQueued = 0;
        std::unique_ptr<ClippedRenderPrewarm> mDecapitationRenderPrewarm;
        std::size_t mDecapitationWarmDrawn = 0, mDecapitationWarmTimedOut = 0, mDecapitationWarmDrawables = 0;

        // Bounded Parts
        PartHolderPtr mObjectParts[ESM::PRT_Count];
        std::array<std::string, ESM::PRT_Count> mPartModels;
        std::array<std::string, ESM::PRT_Count> mFirstPersonWristCoverage;
        std::array<MWSound::Sound*, ESM::PRT_Count> mSounds;

        const ESM::NPC* mNpc;
        VFS::Path::Normalized mHeadModel;
        VFS::Path::Normalized mHairModel;
        ViewMode mViewMode;
        std::optional<ViewMode> mPlayerDeathView;
        bool mShowWeapons;
        bool mShowCarriedLeft;

        enum NpcType
        {
            Type_Normal,
            Type_Werewolf,
            Type_Vampire
        };
        NpcType mNpcType;

        int mPartslots[ESM::PRT_Count]; // Each part slot is taken by clothing, armor, or is empty
        int mPartPriorities[ESM::PRT_Count];

        // Source matrices are restored before every update and blend capture. The
        // fit is applied only after the whole scene has evaluated its current pose.
        struct NpcGuardPose;
        std::shared_ptr<NpcGuardPose> mNpcGuardPose;
        NpcGuardGeometry mNpcGuardGeometry;
        void restoreNpcGuardPose();
        void fitNpcGuardPose(osg::NodeVisitor& visitor);
        void updateNpcGuardPose(float duration);
        void clearNpcGuardPose();
        struct NpcTwoHandGuardPose;
        std::shared_ptr<NpcTwoHandGuardPose> mNpcTwoHandGuardPose;
        bool mNpcGuardRigAuditRequested = false;
        std::optional<NpcGuardRigAudit> mNpcGuardRigAudit;
        bool usesNpcTwoHandGuard() const;
        void setNpcTwoHandGuardTarget(bool enabled, const osg::Vec3f& point, const osg::Vec3f& direction);
        void restoreNpcTwoHandGuardPose();
        void updateNpcTwoHandGuardPose(float duration);
        void fitNpcTwoHandGuardPose(osg::NodeVisitor& visitor);
        void clearNpcTwoHandGuardPose();
        std::optional<NpcGuardMeshTrace> traceNpcTwoHandGuardMeshForAudit(
            const osg::Vec3f& origin, const osg::Vec3f& end, bool combat = false) const;
        bool hasNpcTwoHandGuardMeshForCombat() const;
        std::optional<NpcGuardMeshTrace> traceNpcGuardMeshSnapshot(
            const osg::Vec3f& origin, const osg::Vec3f& end, bool combat, osg::Matrixf* itemToWorld = nullptr) const;

        std::vector<std::pair<osg::observer_ptr<osg::MatrixTransform>, osg::Matrixf>> mClinchSourcePose;
        std::vector<std::pair<osg::observer_ptr<osg::MatrixTransform>, osg::Matrixf>> mBackstabSourcePose;
        std::optional<float> mGroundFinisherPitch;
        float mGroundFinisherLowering = 0.f;
        std::pair<osg::observer_ptr<osg::MatrixTransform>, osg::Matrixf> mSwordWipeSourcePose;
        ESM::RefNum mSwordWipeItem;
        std::optional<std::pair<osg::Vec3f, osg::Vec3f>> mBackstabBlade;
        osg::observer_ptr<osg::Node> mClinchBladeNode;
        osg::Vec3f mClinchBladeRange;
        std::uint64_t mClinchPoseId = 0;
        bool mClinchWasContest = false;
        double mClinchExitTime = 0.;
        osg::Vec3f mClinchLastCorrection, mClinchExitCorrection;
        osg::Vec3f getClinchBladeRange();

        SwordWallContact mSwordWallContact;
        WeaponCycleState mWeaponCycleState;
        struct WallWeaponState
        {
            osg::observer_ptr<osg::Node> node;
            std::vector<osg::Vec3f> vertices;
            std::vector<std::pair<std::size_t, std::size_t>> edges;
            osg::Matrixf previousMatrix, previousCamera;
            osg::Vec3f previousRootScale{ 1.f, 1.f, 1.f }, previousRootPosition;
            osg::Quat previousRootAttitude;
            float previousFovRatio = 1.f;
            osg::Vec3f materialPoint;
            bool materialWitness = false;
            float recoilAge = -1.f;
            osg::Vec3f recoilFoldAxis;
            int recoilFoldDegrees = 0;
            bool recoilFoldValid = false;
            osg::Vec3f previousPosition;
            float base = 0.f, tip = 0.f, previousTime = -1.f;
            bool history = false;
        };
        std::array<WallWeaponState, 2> mWallWeapons;
        unsigned mWallStruckMount = 0;
        std::vector<std::pair<osg::observer_ptr<osg::MatrixTransform>, osg::Matrixf>> mHeldGripSourcePose;
        std::string mSwordWallGroup, mSwordWallKey;
        std::vector<std::pair<osg::observer_ptr<osg::MatrixTransform>, osg::Matrixf>> mSwordWallSourcePose;
        bool mSwordWallStruck = false;
        osg::Matrixf mSwordWallLastCorrection, mSwordWallAppliedTail;
        osg::Vec3f mSwordWallCorrectionOrigin;
        float mSwordWallTailAge = -1.f;
        bool mSwordWallCorrectionValid = false;
        bool applySwordWallCorrection(
            const osg::Matrixf& correction, const osg::FrameStamp& frameStamp, osg::Matrixf* applied = nullptr);
        bool sampleWeaponWallContact(unsigned mount, osg::Node* node, float duration, const osg::FrameStamp& frameStamp);

        osg::Vec3f mFirstPersonOffset;
        // Field of view to use when rendering first person meshes
        float mFirstPersonFieldOfView;

        std::shared_ptr<HeadAnimationTime> mHeadAnimationTime;
        std::shared_ptr<WeaponAnimationTime> mWeaponAnimationTime;

        bool mSoundsDisabled;

        // These secondary skeletons mirror the player. They cannot drive combat or actor motion.
        bool mBodyAwareness = false;
        bool mShadowOnly = false;
        bool mBeastRace = false; // Khajiit/Argonian anatomy: keeps the Morrowind gait.
        bool skipAdditionalAnimation(const VFS::Path::Normalized& kf) const override;
        bool mBodyAwarenessHiddenForClinch = false;
        void updateBodyAwarenessClinchVisibility();
        osg::ref_ptr<osg::MatrixTransform> mBodyAwarenessRoot;
        osg::ref_ptr<NpcAnimation> mBodyAwarenessAnimation;
        osg::ref_ptr<osg::MatrixTransform> mPlayerShadowRoot;
        osg::ref_ptr<NpcAnimation> mPlayerShadowAnimation;
        osg::Vec3f mBodyGroundNormal{ 0.f, 0.f, 1.f };
        osg::Vec3f mBodyGroundUp{ 0.f, 0.f, 1.f };
        osg::Vec3f mBodyGroundAnchor;
        float mBodyGroundOffset = 0.f;
        float mBodyGroundLift = 0.f;
        float mBodyGroundBlend = 0.f;
        void resetBodyAwareness();
        void updateBodyAwareness();
        void resetPlayerShadow();
        void updatePlayerShadow();
        void syncBodyAwareness(float duration);
        void syncFirstPersonBodyPose(NpcAnimation& body, float duration);
        void alignBodyAwareness(float duration);
        std::string bodyAwarenessGroup(const NpcAnimation& body, const std::string& group, bool sneaking) const;

        // The first-person eye rides on the visible torso: the body rig's neck
        // displacement from its settled standing pose, in object root space.
        osg::Vec3f mBodyEyeReference;
        osg::Vec3f mBodyEyeHeld;
        osg::Vec3f mBodyEyeOutput;
        osg::Vec3f mBodyEyeBlendFrom;
        float mBodyEyeStandingTime = 0.f;
        float mBodyEyeBlendTime = -1.f;
        float mBodyEyeWeight = 1.f;
        bool mBodyEyeReferenceValid = false;
        bool mBodyEyePassive = true;
        bool mBodyEyeLive = false;
        void resetBodyEyeFollow();
        void updateBodyEyeFollow(float duration);
        std::optional<osg::Vec3f> bodyNeckLocal() const;
        osg::Vec3f liveBodyEyeOffset() const;

        bool mAccurateAiming;
        enum BrutalViewmodel
        {
            BrutalViewmodel_Default,
            BrutalViewmodel_Shield,
            BrutalViewmodel_Staff,
            BrutalViewmodel_Spear,
            BrutalViewmodel_Dagger,
            BrutalViewmodel_Paired,
            BrutalViewmodel_PairedBlunt,
            BrutalViewmodel_Bow,
        };
        static constexpr std::string_view sBrutalLeftWeaponBone = "Brutal Left Weapon";
        BrutalViewmodel mViewmodelSet = BrutalViewmodel_Default; // Clip set loaded for the first-person rig.
        // Dark Messiah bow: the nock in the right hand's frame and the arrow rest in the left hand's frame.
        struct ArrowMount
        {
            osg::Vec3f mNock;
            osg::Vec3f mRest;
            osg::Vec3f mHold; // The arrow's direction in the right hand before it is nocked.
        };
        std::optional<ArrowMount> mArrowMount;
        osg::ref_ptr<osg::MatrixTransform> mNockedArrow;
        float mNockedArrowBack = 0.f;
        void loadArrowMount();
        bool hasPolearmViewmodel() const
        {
            return mViewmodelSet == BrutalViewmodel_Staff || mViewmodelSet == BrutalViewmodel_Spear;
        }
        bool hasPairedViewmodel() const
        {
            return mViewmodelSet == BrutalViewmodel_Paired || mViewmodelSet == BrutalViewmodel_PairedBlunt;
        }
        BrutalViewmodel brutalViewmodelSet() const;
        bool carriesLeftWeapon() const;
        bool carriesShield() const;
        float mAimingFactor;

        void updateNpcBase();

        NpcType getNpcType() const;

        PartHolderPtr insertBoundedPart(VFS::Path::NormalizedView model, std::string_view bonename,
            std::string_view bonefilter, bool enchantedGlow, osg::Vec4f* glowColor, bool isLight);

        void removeIndividualPart(ESM::PartReferenceType type);
        void reserveIndividualPart(ESM::PartReferenceType type, int group, int priority);

        void fitFirstPersonRobe(osg::Node& node);
        void fitFirstPersonWrists();
        bool addOrReplaceIndividualPart(ESM::PartReferenceType type, int group, int priority,
            VFS::Path::NormalizedView mesh, bool enchantedGlow = false, osg::Vec4f* glowColor = nullptr,
            bool isLight = false);
        void removePartGroup(int group);
        void addPartGroup(int group, int priority, const std::vector<ESM::PartReference>& parts,
            bool enchantedGlow = false, osg::Vec4f* glowColor = nullptr);

        void setRenderBin();

        osg::ref_ptr<RotateController> mFirstPersonNeckController;

        static bool isFemalePart(const ESM::BodyPart* bodypart);
        static NpcType getNpcType(const MWWorld::Ptr& ptr);

    protected:
        void addControllers() override;
        bool isArrowAttached() const override;
        std::string getSheathedShieldMesh(const MWWorld::ConstPtr& shield) const override;

    public:
        /**
         * @param ptr
         * @param disableListener  Don't listen for equipment changes and magic effects. InventoryStore only supports
         *                         one listener at a time, so you shouldn't do this if creating several NpcAnimations
         *                         for the same Ptr, eg preview dolls for the player.
         *                         Those need to be manually rendered anyway.
         * @param disableSounds    Same as \a disableListener but for playing items sounds
         * @param viewMode
         */
        NpcAnimation(const MWWorld::Ptr& ptr, osg::ref_ptr<osg::Group> parentNode,
            Resource::ResourceSystem* resourceSystem, bool disableSounds = false, ViewMode viewMode = VM_Normal,
            float firstPersonFieldOfView = 55.f, FirstPersonBody body = FirstPersonBody::None);
        virtual ~NpcAnimation();

        void enableHeadAnimation(bool enable) override;
        int getPartForNodePath(const osg::NodePath& path) const override;
        /// The scene node of an equipped part (for example ESM::PRT_Shield), if it is shown.
        osg::Node* getPartNode(ESM::PartReferenceType part) const;
        ProjectileAttachment attachStuckProjectileToShield(VFS::Path::NormalizedView model, const osg::Vec3f& hitPosition,
            const osg::Vec3f& flight, const std::pair<osg::Vec3f, osg::Vec3f>& path, const ESM::RefId& item);
        bool addBloodDecal(const osg::Vec3f& position, const osg::Vec3f& outward,
            std::string_view kind, bool power, int hitPart = -1);
        std::string getBloodDecalData() const;
        bool restoreBloodDecals(std::string_view data);
        std::map<std::string, double> getBloodDecalStats() const;
        // Blood left on a held melee weapon by a hit at this world point.
        bool addWeaponBlood(const osg::Vec3f& contact, std::string_view kind, bool power, bool left = false);
        // Hilt and tip of a drawn melee weapon in world space.
        std::optional<std::pair<osg::Vec3f, osg::Vec3f>> getWeaponBladeForBlood(bool left = false) const;
        auto getBackstabBlade() const { return mBackstabBlade; }
        std::map<std::string, double> getWeaponBloodStats() const;
        void startCutBloodFlow();
        void startBisectionBloodFlow();
        void startLimbSeveringBloodFlow();
        std::optional<CutPoolPoint> getCutPoolPoint(BloodPoolOutlet outlet) const;
        struct SwordClinchGeometry
        {
            bool active = false, fitted = false, clamped = false;
            unsigned int frame = 0;
            int phase = 0;
            float elapsed = 0.f, sourceElapsed = 0.f, weight = 0.f, rawGap = 0.f, finalGap = 0.f, cameraError = 0.f;
            float exposureScale = 1.f; // Share of the fit kept to keep the shoulders out of view.
            osg::Vec3f correction, requestedCorrection;
            osg::Vec3f playerHilt, playerAxis, opponentHilt, opponentAxis;
            osg::Vec3f playerRange, opponentRange, playerContact, opponentContact;
            float playerParameter = 0.f, opponentParameter = 0.f;
        };
        SwordClinchGeometry mSwordClinchGeometry;
        void restoreTransientPose() override;
        void fitSwordClinchPose(osg::NodeVisitor& visitor);
        void fitBackstabPose(osg::NodeVisitor& visitor);
        void setGroundFinisherStance(std::optional<float> pitch, float lowering)
        {
            mGroundFinisherPitch = pitch;
            mGroundFinisherLowering = lowering;
        }
        bool isPlayingGroundFinisher() const { return isPlaying("brutal_ground_finisher"); }
        std::optional<osg::Vec3f> groundFinisherStrikeOffset(std::string_view group, float pitch);
        float groundFinisherCameraDrop(std::string_view group, float time) const;
        void setSwordWipeItem(ESM::RefNum item) { mSwordWipeItem = item; }
        void fitSwordWipePose();
        /// With the Dark Messiah bow, keep a nocked arrow between the right fingers and the bow's arrow rest.
        void fitNockedArrow();
        bool canFitSwordClinchPose(NpcAnimation& opponent);
        const SwordClinchGeometry& getSwordClinchGeometryForAudit() const { return mSwordClinchGeometry; }
        // worldPoint is the threatened body point; incomingDirection points
        // from the attacker towards this NPC. Mesh contact is fitted in front
        // of that point after the authored pose has been evaluated.
        void setNpcGuardTarget(bool enabled, bool shield, const osg::Vec3f& worldPoint,
            const osg::Vec3f& incomingDirection, const osg::Vec3f& strokeDirection = {});
        const NpcGuardGeometry& getNpcGuardGeometryForAudit() const { return mNpcGuardGeometry; }
        std::optional<NpcGuardMeshTrace> traceNpcGuardMeshForAudit(
            const osg::Vec3f& origin, const osg::Vec3f& end) const;
        // Combat keeps the last actual item-to-actor pose and reprojects it
        // through the actor's current transform. The audit uses its exact frame.
        std::optional<NpcGuardMeshTrace> traceNpcGuardMeshForCombat(
            const osg::Vec3f& origin, const osg::Vec3f& end) const;
        bool hasNpcGuardMeshForCombat() const;
        // Explicit one-shot capture of the next ready post-update paired solve.
        // No diagnostic maps are allocated during ordinary guard updates.
        void requestNpcGuardRigAudit();
        std::optional<NpcGuardRigAudit> getNpcGuardRigForAudit() const;

        void configureSwordWallSweep(std::uint64_t swing, std::string_view group, std::string_view key);
        bool fitHeldObjectGrip(const MWPhysics::HeldObjectGrip& grip, const osg::Matrixf& pickupTransform,
            const osg::Vec3f& eye, const osg::Quat& view,
            const std::function<float(const osg::Vec3f&, const osg::Vec3f&, const osg::Quat&)>& sweep,
            const osg::FrameStamp& frameStamp, osg::Vec3f& position, osg::Quat& rotation, bool& lifted);
        osg::ref_ptr<osg::Callback> makeHeldObjectCullCallback() const;
        std::vector<osg::Node*> getDetachedFragmentNodes(unsigned fragment) const;
        bool sampleSwordWallContact(float duration, const osg::FrameStamp& frameStamp);
        bool consumeSwordWallContact(std::uint64_t swing, std::string& key);
        const SwordWallContact& getSwordWallContactForAudit() const { return mSwordWallContact; }
        void recordWeaponCycleForAudit(const WeaponCycleState& state) { mWeaponCycleState = state; }
        const WeaponCycleState& getWeaponCycleForAudit() const { return mWeaponCycleState; }
        void recordSwordWallSound(bool played)
        {
            mSwordWallContact.soundPlayed = played;
            if (played)
                ++mSwordWallContact.sounds;
        }
        std::map<std::string, osg::Vec3f> getShieldGeometryForAudit() const;
        std::map<std::string, double> getDeathCameraClearanceForAudit(const osg::Vec3f& eye) const;
        struct WeaponGripGeometry
        {
            std::map<std::string, std::string> models;
            std::map<std::string, osg::Matrixf> roots;
            std::map<std::string, std::vector<osg::Vec3f>> triangles;
            bool leftItemVisible = false;
        };
        WeaponGripGeometry getWeaponGripGeometryForAudit() const;
        std::shared_ptr<const ESM::ReleasedEquipmentState> captureReleasedEquipment(int slot,
            const osg::Matrixf& itemRoot, float mass, float& maxWorldVertexError) const;
        std::vector<std::pair<ESM::RefId, osg::Matrixf>> captureShieldProjectiles(
            const osg::Matrixf* releasedRoot = nullptr) const;
        bool getReleasedEquipmentOwnerIsLeft(int slot) const;
        RagdollExtremityVertices getRagdollExtremityVertices(
            UnsupportedRagdollExtremityBones* unsupportedBones = nullptr) const override;
        bool setHeadSevered(bool severed);
        bool setTorsoSevered(bool severed);
        bool setLimbsSevered(unsigned mask);
        unsigned getSeveredLimbsMask() const;
        bool canSeverLegs() const;
        std::map<std::string, double> getLimbSeveringStats() const;
        std::map<std::string, osg::Vec3f> getLimbSeveringSurfacesForAudit() const;
        bool isTorsoSevered() const { return bool(mBisection); }
        std::map<std::string, double> getBisectionStats() const;
        std::map<std::string, osg::Vec3f> getBisectionSurfacesForAudit() const;
        std::map<std::string, double> getDecapitationStats() const;
        std::map<std::string, osg::Vec3f> getDecapitationSurfacesForAudit() const;
        bool useFullBodyFirstPerson() const;
        /// Reapply a run-time animation option to this actor.
        void reloadBrutalAnimations();
        const NpcAnimation* getBodyAwarenessAnimation() const { return mBodyAwarenessAnimation.get(); }
        bool useBodyCrouchTransitions() const override;
        bool usePrimaryCrouchTransitions() const override;
        const osg::Vec3f& getBodyGroundUp() const { return mBodyGroundUp; }
        const osg::Vec3f& getBodyGroundAnchor() const { return mBodyGroundAnchor; }
        float getBodyGroundBlend() const { return mBodyGroundBlend; }
        float getBodyGroundLift() const { return mBodyGroundLift; }
        std::map<std::string, osg::Vec3f> getFootGeometryForAudit(const osg::Vec3f& normal) const;
        unsigned int getBodyAwarenessVisibilityMask() const
        { return mBodyAwarenessRoot ? mBodyAwarenessRoot->getNodeMask() : 0u; }
        bool isBodyAwarenessHiddenForClinch() const { return mBodyAwarenessHiddenForClinch; }
        bool hasPlayerShadowBody() const { return mPlayerShadowAnimation.valid(); }
        /// True when the eye follows the visible torso instead of a fixed sneak height.
        bool hasBodyEyeFollow() const;
        /// The torso displacement applied to the eye and arms (object root space).
        osg::Vec3f getBodyEyeFollow() const { return hasBodyEyeFollow() ? mBodyEyeOutput : osg::Vec3f(); }
        /// Share of the torso displacement used by the eye; a Brutal slide lowers the eye on its own.
        void setBodyEyeFollowWeight(float weight) { mBodyEyeWeight = weight; }
        std::optional<osg::Vec3f> getBodyNeckPositionForAudit() const;
        void setFirstPersonGeometryVisibleForAudit(bool visible);

        /// 1: the first person meshes follow the camera's rotation completely
        /// 0: the first person meshes follow the camera with a reduced factor, so you can look down at your own hands
        void setAccurateAiming(bool enabled) override;

        void setWeaponGroup(const std::string& group, bool relativeDuration) override;

        osg::Vec3f runAnimation(float timepassed) override;

        /// A relative factor (0-1) that decides if and how much the skeleton should be pitched
        /// to indicate the facing orientation of the character.
        void setPitchFactor(float factor) override { mPitchFactor = factor; }

        bool getWeaponsShown() const override { return mShowWeapons; }
        void showWeapons(bool showWeapon) override;

        bool updateCarriedLeftVisible(const int weaptype) const override;
        bool getCarriedLeftShown() const override { return mShowCarriedLeft; }
        void showCarriedLeft(bool show) override;

        void attachArrow() override;
        void detachArrow() override;
        void releaseArrow(float attackStrength) override;

        osg::Group* getArrowBone() override;
        osg::Node* getWeaponNode() override;
        Resource::ResourceSystem* getResourceSystem() override;

        // WeaponAnimation
        void showWeapon(bool show) override { showWeapons(show); }

        void setViewMode(ViewMode viewMode);
        void preparePlayerDeathPose();
        void restorePlayerDeathView();
        bool hasPlayerDeathView() const { return mPlayerDeathView.has_value(); }

        void updateParts();

        /// Rebuilds the NPC, updating their root model, animation sources, and equipment.
        void rebuild();

        /// Get the inventory slot that the given node path leads into, or -1 if not found.
        int getSlot(const osg::NodePath& path) const;

        void setVampire(bool vampire) override;

        /// Set a translation offset (in object root space) to apply to meshes when in first person mode.
        void setFirstPersonOffset(const osg::Vec3f& offset);

        void updatePtr(const MWWorld::Ptr& updated) override;

        /// Get a list of body parts that may be used by an NPC of given race and gender.
        /// @note This is a fixed size list, one list item for each ESM::PartReferenceType, may contain nullptr body
        /// parts.
        static const std::vector<const ESM::BodyPart*>& getBodyParts(
            const ESM::RefId& raceId, bool female, bool firstperson, bool werewolf);
    };

}

#endif
