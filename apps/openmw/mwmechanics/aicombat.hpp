#ifndef GAME_MWMECHANICS_AICOMBAT_H
#define GAME_MWMECHANICS_AICOMBAT_H

#include "aitemporarybase.hpp"

#include <vector>
#include "aitimer.hpp"
#include "movement.hpp"
#include "typedaipackage.hpp"

#include "../mwworld/cellstore.hpp" // for Doors

#include <components/esm3/loadpgrd.hpp>

namespace ESM
{
    namespace AiSequence
    {
        struct AiCombat;
    }
}

namespace MWMechanics
{
    class Action;

    /// \brief This class holds the variables AiCombat needs which are deleted if the package becomes inactive.
    struct AiCombatStorage : AiTemporaryBase
    {
        float mAttackCooldown;
        AiReactionTimer mReaction;
        float mTimerCombatMove;
        bool mReadyToAttack;
        bool mAttack;
        float mAttackRange;
        bool mCombatMove;
        bool mRotateMove;
        osg::Vec3f mLastTargetPos;
        const MWWorld::CellStore* mCell;
        std::unique_ptr<Action> mCurrentAction;
        float mActionCooldown;
        float mStrength;
        bool mForceNoShortcut;
        ESM::Position mShortcutFailPos;
        MWMechanics::Movement mMovement;

        enum FleeState
        {
            FleeState_None,
            FleeState_Idle,
            FleeState_RunBlindly,
            FleeState_RunToDestination
        };
        FleeState mFleeState;
        bool mLOS;
        float mUpdateLOSTimer;
        float mFleeBlindRunTimer;
        ESM::Pathgrid::Point mFleeDest;

        bool mUseCustomDestination;
        osg::Vec3f mCustomDestination;

        // Health at the last AI decision catches damage outside the native hit handler.
        float mBrutalLastHealth = -1.f;
        bool mBrutalFleeing = false;

        // Dark Messiah's ACT_GRAB_WEAPON (AiCombat::updateBrutalPickup): the loose item this NPC goes
        // for, whether it is a shield, the crouch, the item's last position and how long it lay
        // still, the time spent going to it, the next search and the items it gave up.
        ESM::RefNum mBrutalPickupItem;
        bool mBrutalPickupShield = false;
        bool mBrutalPickupCrouching = false;
        osg::Vec3f mBrutalPickupLastPos;
        float mBrutalPickupStill = 0.f;
        float mBrutalPickupTime = 0.f;
        float mBrutalPickupScan = 0.f;
        osg::Vec3f mBrutalPickupActorLast; // the NPC's position in the direct approach
        float mBrutalPickupStall = 0.f; // seconds the direct approach has not advanced
        std::vector<ESM::RefNum> mBrutalPickupRejected;

        // Dark Messiah guard pacing (npc_guard.qct, mm_game_settings.txt).
        bool mDmBerserk = false;
        float mDmRhythmTimer = 0.f;
        float mDmTargetPassive = 0.f;
        float mDmDodgeCooldown = 0.f;
        float mDmPreventiveTimer = 0.f;
        float mDmTargetGuardTime = 0.f;
        float mDmChargedHold = 0.f;
        // Spreading around a shared target: the side to drift (-1 left, 1 right, 0 none),
        // its next re-evaluation, and whether the strafe currently owns mMovement.
        float mBrutalSpreadSide = 0.f;
        float mBrutalSpreadTimer = 0.f;
        bool mBrutalSpreading = false;
        void updateDarkMessiah(float duration, const MWWorld::Ptr& actor, const MWWorld::Ptr& target,
            CharacterController& characterController);

        AiCombatStorage();

        void startCombatMove(bool isDistantCombat, float distToTarget, float rangeAttack, const MWWorld::Ptr& actor,
            const MWWorld::Ptr& target);
        void updateCombatMove(float duration);
        void stopCombatMove();
        void startAttackIfReady(const MWWorld::Ptr& actor, CharacterController& characterController,
            const ESM::Weapon* weapon, bool distantCombat, bool canShout);
        void updateAttack(const MWWorld::Ptr& actor, CharacterController& characterController);
        void stopAttack();

        void startFleeing();
        void stopFleeing();
        bool isFleeing() const;
    };

    /// \brief Causes the actor to fight another actor
    class AiCombat final : public TypedAiPackage<AiCombat>
    {
    public:
        /// Constructor
        /** \param actor Actor to fight **/
        explicit AiCombat(const MWWorld::Ptr& actor);

        explicit AiCombat(const ESM::AiSequence::AiCombat* combat);

        void init();
        static void clearBrutalPickupClaims();

        bool execute(const MWWorld::Ptr& actor, CharacterController& characterController, AiState& state,
            float duration) override;

        static constexpr AiPackageTypeId getTypeId() { return AiPackageTypeId::Combat; }

        static constexpr Options makeDefaultOptions()
        {
            AiPackage::Options options;
            options.mPriority = 1;
            options.mCanCancel = false;
            options.mShouldCancelPreviousAi = false;
            return options;
        }

        void writeState(ESM::AiSequence::AiSequence& sequence) const override;

    private:
        /// Returns true if combat should end
        bool attack(const MWWorld::Ptr& actor, const MWWorld::Ptr& target, AiCombatStorage& storage,
            CharacterController& characterController);

        void updateLOS(const MWWorld::Ptr& actor, const MWWorld::Ptr& target, float duration, AiCombatStorage& storage);

        /// An NPC without a weapon goes for one lying nearby, then for a shield right beside it.
        /// Returns true while this owns the actor's movement and actions.
        void updateBrutalSpread(const MWWorld::Ptr& actor, const MWWorld::Ptr& target, AiCombatStorage& storage,
            float duration);
        bool updateBrutalPickup(const MWWorld::Ptr& actor, const MWWorld::Ptr& target,
            CharacterController& characterController, AiCombatStorage& storage, float duration);

        void updateFleeing(const MWWorld::Ptr& actor, const MWWorld::Ptr& target, float duration,
            MWWorld::MovementDirectionFlags supportedMovementDirections, AiCombatStorage& storage);

        /// Transfer desired movement (from AiCombatStorage) to Actor
        void updateActorsMovement(const MWWorld::Ptr& actor, float duration, AiCombatStorage& storage);
        void rotateActorOnAxis(const MWWorld::Ptr& actor, int axis, MWMechanics::Movement& actorMovementSettings,
            AiCombatStorage& storage);
    };

}

#endif
