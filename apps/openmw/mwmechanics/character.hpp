#ifndef GAME_MWMECHANICS_CHARACTER_HPP
#define GAME_MWMECHANICS_CHARACTER_HPP

#include <deque>
#include <optional>

#include <components/esm3/loadweap.hpp>

#include "../mwworld/ptr.hpp"

#include "../mwrender/animation.hpp"

namespace MWWorld
{
    class InventoryStore;
}

namespace MWRender
{
    class Animation;
}

namespace MWMechanics
{
    float calculateRagdollFallDamage(const MWWorld::Ptr& ptr, float height);

    struct Movement;
    class CreatureStats;

    enum Priority
    {
        Priority_Default,
        Priority_WeaponLowerBody,
        Priority_SneakIdleLowerBody,
        Priority_SwimIdle,
        Priority_Jump,
        Priority_Movement,
        Priority_Hit,
        Priority_Weapon,
        Priority_Block,
        Priority_Knockdown,
        Priority_Torch,
        Priority_Storm,
        Priority_Death,
        Priority_Scripted,

        Num_Priorities
    };

    enum CharacterState
    {
        CharState_None,

        CharState_SpecialIdle,
        CharState_Idle,
        CharState_IdleSwim,
        CharState_IdleSneak,

        CharState_WalkForward,
        CharState_WalkBack,
        CharState_WalkLeft,
        CharState_WalkRight,

        CharState_SwimWalkForward,
        CharState_SwimWalkBack,
        CharState_SwimWalkLeft,
        CharState_SwimWalkRight,

        CharState_RunForward,
        CharState_RunBack,
        CharState_RunLeft,
        CharState_RunRight,

        CharState_SwimRunForward,
        CharState_SwimRunBack,
        CharState_SwimRunLeft,
        CharState_SwimRunRight,

        CharState_SneakForward,
        CharState_SneakBack,
        CharState_SneakLeft,
        CharState_SneakRight,

        CharState_TurnLeft,
        CharState_TurnRight,
        CharState_SwimTurnLeft,
        CharState_SwimTurnRight,

        CharState_Death1,
        CharState_Death2,
        CharState_Death3,
        CharState_Death4,
        CharState_Death5,
        CharState_SwimDeath,
        CharState_SwimDeathKnockDown,
        CharState_SwimDeathKnockOut,
        CharState_DeathKnockDown,
        CharState_DeathKnockOut,

        CharState_Hit,
        CharState_SwimHit,
        CharState_KnockDown,
        CharState_KnockOut,
        CharState_SwimKnockDown,
        CharState_SwimKnockOut,
        CharState_Block
    };

    enum class UpperBodyState
    {
        None,
        Equipping,
        Unequipping,
        WeaponEquipped,
        AttackWindUp,
        AttackRelease,
        AttackEnd,
        Casting
    };

    enum JumpingState
    {
        JumpState_None,
        JumpState_InAir,
        JumpState_Landing
    };

    struct WeaponInfo;
    struct MeleeFollowThrough;

    class CharacterController : public MWRender::Animation::TextKeyListener
    {
        MWWorld::Ptr mPtr;
        MWWorld::Ptr mWeapon;
        MWRender::Animation* mAnimation;

        struct AnimationQueueEntry
        {
            std::string mGroup;
            uint32_t mLoopCount;
            float mTime;
            bool mLooping;
            bool mScripted;
            std::string mStartKey;
            std::string mStopKey;
            float mSpeed;
        };
        typedef std::deque<AnimationQueueEntry> AnimationQueue;
        AnimationQueue mAnimQueue;
        bool mLuaAnimations{ false };

        CharacterState mIdleState{ CharState_None };
        std::string mCurrentIdle;

        CharacterState mMovementState{ CharState_None };
        std::string mCurrentMovement;
        float mMovementAnimSpeed{ 0.f };
        bool mAdjustMovementAnimSpeed{ false };
        bool mBrutalSprinting{ false }; // kinetic player sprints forward this frame
        bool mBrutalSprintAnim{ false }; // the current movement clip is the sprint
        bool mBrutalWounded{ false }; // the NPC plays Dark Messiah's wounded clips
        // ACT_GRAB_WEAPON: the world item this NPC takes into its hand at "pickup contact".
        MWWorld::SafePtr mBrutalPickupItem;
        std::string mBrutalPickupGroup; // the crouch from the ground or the reach to a table
        MWWorld::SafePtr mBrutalBowKickTarget;
        MWWorld::SafePtr mBrutalBowKickWeapon;
        double mBrutalBowKickReadyAt = 0.;
        float mBrutalStepPhase{ 0.f }; // player footstep clock, a fraction of the leg cycle
        int mBrutalStepGait{ 0 }; // BrutalFootsteps::Gait of the last player step
        bool mMovementAnimationHasMovement{ false };

        CharacterState mDeathState{ CharState_None };
        std::string mCurrentDeath;
        bool mFloatToSurface{ true };

        CharacterState mHitState{ CharState_None };
        std::string mCurrentHit;

        UpperBodyState mUpperBodyState{ UpperBodyState::None };

        enum class BrutalBlock
        {
            None,
            Raising,
            Holding,
            Lowering
        };
        BrutalBlock mBrutalBlock{ BrutalBlock::None };
        std::string mBrutalNpcGuardGroup; // Owned shield/weapon guard layer; stock block reactions remain separate.

        // Dark Messiah player combos: Rush_Attack and Falling_Attack (In, Idle, Out).
        enum class BrutalCombo
        {
            None,
            Rush,
            FallRaise,
            FallHold,
            FallStrike
        };
        BrutalCombo mBrutalCombo{ BrutalCombo::None };
        bool mBrutalComboRelease{ false }; // The attack that started a combo must be released first.
        bool mBrutalRushLanded{ false }; // The rush's first blow struck; the charge stops at the target.
        struct GroundFinisher
        {
            MWWorld::SafePtr mVictim;
            MWWorld::SafePtr mWeapon;
            osg::Vec3f mContact;
            osg::Vec3f mTorsoOffset;
            std::optional<osg::Vec3f> mStrikeOffset;
            float mStrikePitch, mEyeHeight, mAuthoredDrop, mLowering;
            float mInitialYaw, mInitialPitch;
            float mFocusYaw, mFocusPitch;
        };
        std::optional<GroundFinisher> mGroundFinisher;
        enum class BrutalBackstabPhase
        {
            None,
            Entering,
            Ready,
            Canceling,
            Executing
        };
        struct BrutalBackstab
        {
            BrutalBackstabPhase mPhase = BrutalBackstabPhase::None;
            MWWorld::SafePtr mVictim;
            MWWorld::SafePtr mWeapon;
            MWWorld::SafePtr mOffhandWeapon;
            osg::Vec3f mPlayerOrigin;
            osg::Vec3f mVictimOrigin;
            osg::Vec3f mVictimGoal;
            float mPlayerYaw = 0.f;
            float mPlayerPitch = 0.f;
            float mVictimYaw = 0.f;
            float mPairYaw = 0.f;
            float mNeckPitch = 0.f;
            float mEyeLift = 0.f;
            float mTotalDamage = 0.f;
            float mDealtDamage = 0.f;
            bool mAttackQueued = false;
            bool mLethal = false;
            bool mReleased = false;
        };
        BrutalBackstab mBackstab;
        // Dark Messiah clicks: a release before the windup ends strikes the fixed fast attack chain.
        enum class BrutalFast
        {
            None,
            Attack,
            End,
            Transition,
            TransitionHold
        };
        BrutalFast mBrutalFast{ BrutalFast::None };
        int mBrutalFastIndex{ 0 }; // First_basic chain counter: 1 is A.
        std::string mBrutalFastClip;
        bool mBrutalFastQueued{ false };
        bool mBrutalFastButton{ false };
        float mBrutalFastHeld{ 0.f };
        // Adapted input tolerance: authored recovery may play while the next
        // fresh click can still continue the previous fast-chain letter.
        float mBrutalFastGrace{ 0.f };
        int mBrutalFastResumeIndex{ 0 };
        int mBrutalFastWindupIndex{ 0 };
        int mBrutalSpearStrafe{ 0 }; // Side sampled when the next stroke is pressed.
        int mBrutalLastSpearJab{ 0 }; // Keep the jab variation across a return to idle.
        bool mBrutalNeutralAttack{ false }; // Stationary charged swing: its own source clip.
        bool mBrutalGuardPress{ false }; // Attack pressed while the guard is up.
        std::uint64_t mBrutalWallSwing = 0;
        bool mBrutalWallBodyHit = false;
        float mBrutalGuardHeld{ 0.f };
        bool mBrutalChargedHold = false;
        bool mBrutalGuardCharge = false;
        bool mBrutalUseButton = false;
        float mBrutalUseHeld = 0.f;
        std::uint64_t mBrutalAcceptedStrikes = 0;
        float mBrutalFallPeakZ{ 0.f };
        bool mBrutalFallReleased{ false }; // A short tap still finishes the minimum raise.
        float mBrutalSlideAge{ 0.f };
        float mBrutalSlideExitAge{ 0.f };
        float mBrutalSlideTravelTime{ 0.f };
        bool mBrutalSlideStopped{ false };
        osg::Vec3f mBrutalSlideTravelStart;
        bool mResetIdleOnAttackEnd{ false };
        std::string mSwordClinchGroup;


        JumpingState mJumpState{ JumpState_None };
        std::string mCurrentJump;
        bool mInJump{ false };

        int mWeaponType{ ESM::Weapon::None };
        std::string mCurrentWeapon;
        ESM::RefNum mBrutalWipeWeapon;
        double mBrutalBowRepeatUntil = -1.;
        bool mBrutalBowRepeatQueued = false;
        bool mBrutalBowFastDraw = false;

        float mAttackStrength{ -1.f };
        bool mReadyToHit{ false };
        MWWorld::Ptr mAttackVictim;
        osg::Vec3f mAttackHitPos;
        bool mAttackSuccess{ false };
        int mAttackContactPart = -1;

        bool mSkipAnim{ false };

        // counted for skill increase
        float mSecondsOfSwimming{ 0.f };
        float mSecondsOfRunning{ 0.f };

        MWWorld::ConstPtr mHeadTrackTarget;

        float mTurnAnimationThreshold{
            0.f
        }; // how long to continue playing turning animation after actor stopped turning

        std::string mAttackType; // slash, chop or thrust

        bool mCanCast{ false };

        bool mCastingScriptedSpell{ false };

        bool mIsMovingBackward{ false };
        osg::Vec2f mSmoothedSpeed;

        std::string_view getMovementBasedAttackType() const;
        std::string getAttackAnimationType() const;

        void clearStateAnimation(std::string& anim) const;
        void resetCurrentJumpState();
        void resetCurrentMovementState();
        void resetCurrentIdleState();
        void resetCurrentHitState();
        void resetCurrentWeaponState();
        void resetCurrentDeathState();

        void refreshCurrentAnims(CharacterState idle, CharacterState movement, JumpingState jump, bool force = false);
        void refreshHitRecoilAnims();
        bool collapseIntoRagdoll(bool swimming);
        void refreshJumpAnims(JumpingState jump, bool force = false);
        void refreshMovementAnims(CharacterState movement, bool force = false);
        void refreshIdleAnims(CharacterState idle, bool force = false);

        bool updateWeaponState();
        void updateIdleStormState(bool inwater) const;

        std::string chooseRandomAttackAnimation() const;
        static bool isRandomAttackAnimation(std::string_view group);

        bool isMovementAnimationControlled() const;
        bool isUpperBodyRecovery() const;

        void updateAnimQueue();
        void playAnimQueue(bool useLoopStart = false);

        void updateHeadTracking(float duration);

        void updateMagicEffects() const;

        void playDeath(float startpoint, CharacterState death);
        CharacterState chooseRandomDeathState() const;
        void playRandomDeath(float startpoint = 0.0f);

        /// choose a random animation group with \a prefix and numeric suffix
        /// @param num if non-nullptr, the chosen animation number will be written here
        std::string chooseRandomGroup(const std::string& prefix, int* num = nullptr) const;

        bool updateCarriedLeftVisible(int weaptype) const;
        int brutalAnimationWeaponType(int weaponType) const;
        void normalizeBrutalDualDaggers();
        bool hasBrutalOffhandDagger() const;
        bool brutalStaffSecondStrike() const;
        int mBrutalOffhandType = -1; // Attack type awaiting the left dagger's strike.
        float mBrutalOffhandStrength = 0.f;

        std::string fallbackShortWeaponGroup(
            const std::string& baseGroupName, MWRender::Animation::BlendMask* blendMask = nullptr) const;

        std::string_view getWeaponAnimation(int weaponType) const;
        std::string_view getWeaponShortGroup(int weaponType) const;

        bool getAttackingOrSpell() const;
        void setAttackingOrSpell(bool attackingOrSpell) const;

        std::string_view getDesiredAttackType() const;

        void prepareHit();
        void resolveMeleeHit(std::vector<MeleeFollowThrough>* followThrough = nullptr);
        void applyMeleeImpact(int attackType);
        bool canPlayGroundFinisher() const;
        std::optional<osg::Vec3f> groundFinisherContact(const MWWorld::Ptr& target, bool lockedTarget) const;
        bool startGroundFinisher();
        bool updateGroundFinisher();
        void cancelGroundFinisher();
        void hitGroundFinisher();
        bool canPlayBackstab(bool executing = false) const;
        bool isValidBackstabTarget(const MWWorld::Ptr& target, bool maintain) const;
        MWWorld::Ptr findBackstabTarget(bool maintain) const;
        void enterBackstabStance(const MWWorld::Ptr& target);
        bool startBackstabExecution(const MWWorld::Ptr& victim);
        bool updateBackstab();
        void cancelBackstab(bool playCancelAnim = false);
        void onBackstabStab(int step);
        void onBackstabRelease();
        bool updateBackstabVictim(float duration);
        void releaseBackstabVictim(const MWWorld::Ptr& victim);

        void unpersistAnimationState();

        void playBlendedAnimation(const std::string& groupname, const MWRender::AnimPriority& priority, int blendMask,
            bool autodisable, float speedmult, std::string_view start, std::string_view stop, float startpoint,
            uint32_t loops, bool loopfallback = false) const;

    public:
        CharacterController(const MWWorld::Ptr& ptr, MWRender::Animation& anim);
        virtual ~CharacterController();

        CharacterController(const CharacterController&) = delete;
        CharacterController(CharacterController&&) = delete;

        void detachAnimation();

        const MWWorld::Ptr& getPtr() const { return mPtr; }

        void handleTextKey(std::string_view groupname, SceneUtil::TextKeyMap::ConstIterator key,
            const SceneUtil::TextKeyMap& map) override;

        // Be careful when to call this, see comment in Actors
        void updateContinuousVfx() const;

        void updatePtr(const MWWorld::Ptr& ptr);

        void update(float duration);

        bool onOpen() const;
        void onClose() const;

        void persistAnimationState() const;
        bool playGroup(std::string_view groupname, int mode, uint32_t count, bool scripted = false);
        bool playGroupLua(std::string_view groupname, float speed, std::string_view startKey, std::string_view stopKey,
            uint32_t loops, bool forceLoop);
        void enableLuaAnimations(bool enable);
        void skipAnim();
        bool isAnimPlaying(std::string_view groupName) const;
        bool isScriptedAnimPlaying() const;
        void clearAnimQueue(bool clearScriptedAnims = false);

        enum KillResult
        {
            Result_DeathAnimStarted,
            Result_DeathAnimPlaying,
            Result_DeathAnimJustFinished,
            Result_DeathAnimFinished
        };
        KillResult kill();

        void resurrect();
        bool isDead() const { return mDeathState != CharState_None; }

        void forceStateUpdate();
        // Full-body recovery; group is a preferred reaction clip (falls back to brutal_hit).
        bool stagger(std::string_view group = {});
        // Dark Messiah's ACT_GRAB_WEAPON: the NPC crouches and, at the clip's "pickup contact",
        // takes this loose weapon or shield straight into its hand without drawing it.
        bool canBrutalPickup() const;
        bool startBrutalPickup(const MWWorld::Ptr& item);
        // The pickup clip is playing, including the rise after the item was taken.
        bool isBrutalPickingUp() const;
        bool isBrutalSprinting() const { return mBrutalSprinting; }
        bool startBrutalBowKick(const MWWorld::Ptr& target);
        bool isBrutalBowKicking() const;
        bool canSwordClinch() const;
        bool hasPhysicalDeath() const;
        void playSwordClinch(std::string_view group, float speed = 1.f);
        void finishSwordClinch(bool preserveRecovery);
        bool updateSwordClinch(float duration);

        void cancelAttackForKick();
        void hitBrutalBowKick();
        void updateBrutalWounded();
        void updateBrutalPickup();
        bool takeBrutalPickup(const MWWorld::Ptr& item);
        void updateBrutalShieldBlock();
        void updateBrutalNpcGuard();
        void updateBrutalNpcGuardTarget(bool enabled, bool shield);
        void updateBrutalPlayerSteps(float duration, CharacterState movestate, bool grounded);
        ESM::RefNum getWeaponWipeItem() const;
        bool updateBrutalWeaponWipe(bool requested);
        void cancelBrutalWeaponWipe();
        bool updateBrutalCombo();
        void updateBrutalSlide(float duration);
        void cancelBrutalSlide();
        void cancelBrutalCombo();
        bool brutalFastAvailable() const;
        void chooseBrutalChargedAttack();
        bool brutalActiveKey(const std::string& key) const;
        bool isBrutalDaggerSet() const;
        bool isBrutalChainStaff() const;
        bool usesBrutalPairedWeapon() const;
        bool isBrutalWallWeapon() const;
        void startBrutalFastClip(const std::string& key, BrutalFast phase);
        void startBrutalFastNext();
        int getBrutalSpearStrafe() const;
        void resetBrutalFastContinuation();
        bool startBrutalChargedHold();
        void updateBrutalFast(bool& animPlaying, float& complete);
        void brutalFastHit(std::string_view action);
        bool brutalBounce();
        void updateBrutalWallSweep();
        void brutalGuardAttack();
        void brutalComboHit(float ratio, float push, bool falling);
        bool isBrutalComboActive() const { return mBrutalCombo != BrutalCombo::None; }
        bool isBrutalShieldBlocking() const { return mBrutalBlock != BrutalBlock::None; }

        bool isAttackPreparing() const;
        bool isCastingSpell() const;
        bool isReadyToBlock() const;
        bool isKnockedDown() const;
        bool isKnockedOut() const;
        bool isRecovery() const;
        bool isSneaking() const;
        bool isRunning() const;
        bool isTurning() const;
        bool isAttackingOrSpell() const;

        void setVisibility(float visibility) const;
        void castSpell(const ESM::RefId& spellId, bool scriptedSpell = false);
        void setAIAttackType(std::string_view attackType);
        static std::string_view getRandomAttackType();

        bool readyToPrepareAttack() const;
        bool readyToStartAttack() const;
        // Adaptive defence may replace a small upper-body flinch, never a full stagger.
        bool readyToStartBrutalGuard() const;

        float calculateWindUp() const;

        float getAttackStrength() const;

        /// @see Animation::setActive
        void setActive(int active) const;

        /// Make this character turn its head towards \a target. To turn off head tracking, pass an empty Ptr.
        void setHeadTrackTarget(const MWWorld::ConstPtr& target);

        void playSwishSound() const;

        float getAnimationMovementDirection() const;

        MWWorld::MovementDirectionFlags getSupportedMovementDirections() const;
    };
}

#endif /* GAME_MWMECHANICS_CHARACTER_HPP */
