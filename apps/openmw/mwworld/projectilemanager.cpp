#include "projectilemanager.hpp"

#include <iomanip>
#include <memory>
#include <optional>
#include <sstream>

#include <algorithm>
#include <cmath>
#include <limits>
#include <map>

#include <osg/ComputeBoundsVisitor>
#include <osg/PositionAttitudeTransform>

#include <components/debug/debuglog.hpp>

#include <components/esm3/actoridconverter.hpp>
#include <components/esm3/esmreader.hpp>
#include <components/esm3/esmwriter.hpp>
#include <components/esm3/loadench.hpp>
#include <components/esm3/loadmgef.hpp>
#include <components/esm3/loadrace.hpp>
#include <components/esm3/projectilestate.hpp>

#include <components/esm/quaternion.hpp>
#include <components/esm/vector3.hpp>

#include <components/misc/constants.hpp>
#include <components/misc/convert.hpp>
#include <components/misc/resourcehelpers.hpp>
#include <components/misc/rng.hpp>

#include <components/resource/resourcesystem.hpp>
#include <components/resource/scenemanager.hpp>

#include <components/sceneutil/controller.hpp>
#include <components/sceneutil/lightmanager.hpp>
#include <components/sceneutil/nodecallback.hpp>
#include <components/sceneutil/visitor.hpp>

#include <components/settings/values.hpp>

#include "../mwworld/class.hpp"
#include "../mwworld/esmstore.hpp"
#include "../mwworld/inventorystore.hpp"
#include "../mwworld/manualref.hpp"
#include "../mwworld/surfacematerial.hpp"
#include "../mwworld/worldmodel.hpp"

#include "../mwbase/environment.hpp"
#include "../mwbase/luamanager.hpp"
#include "../mwbase/mechanicsmanager.hpp"
#include "../mwbase/soundmanager.hpp"
#include "../mwbase/windowmanager.hpp"
#include "../mwbase/world.hpp"

#include "../mwmechanics/actorutil.hpp"
#include "../mwmechanics/combat.hpp"
#include "../mwmechanics/creaturestats.hpp"
#include "../mwmechanics/spellcasting.hpp"
#include "../mwmechanics/weapontype.hpp"

#include "../mwrender/animation.hpp"
#include "../mwrender/camera.hpp"
#include "../mwrender/npcanimation.hpp"
#include "../mwrender/renderingmanager.hpp"
#include "../mwrender/util.hpp"
#include "../mwrender/vismask.hpp"

#include "../mwsound/sound.hpp"

#include "../mwphysics/physicssystem.hpp"
#include "../mwphysics/projectile.hpp"

namespace
{
    ESM::EffectList getMagicBoltData(std::vector<ESM::RefId>& projectileIDs, std::set<ESM::RefId>& sounds, float& speed,
        std::string& texture, std::string& sourceName, const ESM::RefId& id)
    {
        const MWWorld::ESMStore& esmStore = *MWBase::Environment::get().getESMStore();
        const ESM::EffectList* effects;
        if (const ESM::Spell* spell = esmStore.get<ESM::Spell>().search(id)) // check if it's a spell
        {
            sourceName = spell->mName;
            effects = &spell->mEffects;
        }
        else // check if it's an enchanted item
        {
            MWWorld::ManualRef ref(esmStore, id);
            MWWorld::Ptr ptr = ref.getPtr();
            const ESM::Enchantment* ench = esmStore.get<ESM::Enchantment>().find(ptr.getClass().getEnchantment(ptr));
            sourceName = ptr.getClass().getName(ptr);
            effects = &ench->mEffects;
        }

        int count = 0;
        speed = 0.0f;
        ESM::EffectList projectileEffects;
        for (const ESM::IndexedENAMstruct& effect : effects->mList)
        {
            const ESM::MagicEffect* magicEffect
                = MWBase::Environment::get().getESMStore()->get<ESM::MagicEffect>().find(effect.mData.mEffectID);

            // Speed of multi-effect projectiles should be the average of the constituent effects,
            // based on observation of the original engine.
            speed += magicEffect->mData.mSpeed;
            count++;

            if (effect.mData.mRange != ESM::RT_Target)
                continue;

            if (magicEffect->mBolt.empty())
                projectileIDs.emplace_back(ESM::RefId::stringRefId("VFX_DefaultBolt"));
            else
                projectileIDs.push_back(magicEffect->mBolt);

            if (!magicEffect->mBoltSound.empty())
                sounds.emplace(magicEffect->mBoltSound);
            else
                sounds.emplace(MWBase::Environment::get()
                                   .getESMStore()
                                   ->get<ESM::Skill>()
                                   .find(magicEffect->mData.mSchool)
                                   ->mSchool->mBoltSound);
            projectileEffects.mList.push_back(effect);
        }

        if (count != 0)
            speed /= count;

        // the particle texture is only used if there is only one projectile
        if (projectileEffects.mList.size() == 1)
        {
            const ESM::MagicEffect* magicEffect
                = MWBase::Environment::get().getESMStore()->get<ESM::MagicEffect>().find(
                    effects->mList.begin()->mData.mEffectID);
            texture = magicEffect->mParticle;
        }

        // insert a VFX_Multiple projectile if there are multiple projectile effects
        if (projectileEffects.mList.size() > 1)
        {
            const ESM::RefId projectileId
                = ESM::RefId::stringRefId("VFX_Multiple" + std::to_string(effects->mList.size()));
            projectileIDs.insert(projectileIDs.begin(), projectileId);
        }

        return projectileEffects;
    }

    osg::Vec4 getMagicBoltLightDiffuseColor(const ESM::EffectList& effects)
    {
        // Calculate combined light diffuse color from magical effects
        osg::Vec4 lightDiffuseColor;
        for (const ESM::IndexedENAMstruct& enam : effects.mList)
        {
            const ESM::MagicEffect* magicEffect
                = MWBase::Environment::get().getESMStore()->get<ESM::MagicEffect>().find(enam.mData.mEffectID);
            lightDiffuseColor += magicEffect->getColor();
        }
        size_t numberOfEffects = effects.mList.size();
        lightDiffuseColor /= static_cast<float>(numberOfEffects);

        return lightDiffuseColor;
    }

    osg::Quat lookAt(const osg::Vec3f& pos)
    {
        // Rotate the forward vector towards the position (used for gravity-affected projectiles)
        // Can't use Quat::makeRotate as the shortest angle contains undesirable local roll
        const float dist = pos.length();
        if (dist < 1e-4f)
            return {};

        const osg::Vec3f dir = pos / dist;
        osg::Vec3f right = dir ^ osg::Z_AXIS;
        if (right.normalize() < 1e-4f)
            right = osg::X_AXIS;

        const osg::Vec3f up = right ^ dir;

        osg::Matrixf mat(right.x(), right.y(), right.z(), 0.f, dir.x(), dir.y(), dir.z(), 0.f, up.x(), up.y(), up.z(),
            0.f, 0.f, 0.f, 0.f, 1.f);

        osg::Quat orient;
        orient.set(mat);
        return orient;
    }

    constexpr float sFinisherRadius = .07f; // Screen height units, measured from the crosshair.
    constexpr float sSlowMotionLead = .17f; // Simulation seconds before the predicted contact.
    constexpr int sProjectileCover
        = MWPhysics::CollisionType_Default | MWPhysics::CollisionType_Dynamic | MWPhysics::CollisionType_Water;

    std::optional<osg::Vec3f> finisherHead(const MWWorld::Ptr& target)
    {
        auto world = MWBase::Environment::get().getWorld();
        if (target.isEmpty() || !target.isInCell() || target.getCellRef().getCount() <= 0
            || !target.getRefData().isEnabled() || target.getClass().getCreatureStats(target).isDead()
            || !world->isActorCollisionEnabled(target))
            return std::nullopt;
        const auto* animation = world->getAnimation(target);
        if (animation == nullptr)
            return std::nullopt;
        const auto head = animation->getBoneWorldMatrix("Bip01 Head");
        if (!head)
            return std::nullopt;
        const auto neck = animation->getBoneWorldMatrix("Bip01 Neck");
        osg::Vec3f center = head->getTrans();
        if (neck)
            center += (center - neck->getTrans()) * .6f;
        return center;
    }

    struct FinisherTarget
    {
        MWWorld::Ptr mActor;
        osg::Vec3f mHead;
        float mContactDistance;
    };

    std::optional<FinisherTarget> findFinisherTarget(const MWWorld::Ptr& player, const osg::Vec3f& muzzle,
        const MWRender::Camera& camera, const MWPhysics::PhysicsSystem& physics)
    {
        if (camera.getMode() != MWRender::Camera::Mode::FirstPerson
            && camera.getMode() != MWRender::Camera::Mode::ThirdPerson)
            return std::nullopt;

        const osg::Vec3f eye(camera.getPosition());
        const osg::Matrixf projection = camera.getViewMatrix() * camera.getProjectionMatrix();
        const float aspect = static_cast<float>(Settings::video().mResolutionX.get())
            / std::max(1, Settings::video().mResolutionY.get());
        std::vector<MWWorld::Ptr> actors;
        MWBase::Environment::get().getMechanicsManager()->getActorsInRange(
            eye, static_cast<float>(Settings::game().mActorsProcessingRange.get()), actors);

        std::optional<FinisherTarget> best;
        float bestOffset = sFinisherRadius * sFinisherRadius;
        float bestDepth = std::numeric_limits<float>::infinity();
        for (const auto& actor : actors)
        {
            if (actor == player)
                continue;
            const auto point = finisherHead(actor);
            if (!point)
                continue;
            const osg::Vec4f clip = osg::Vec4f(*point, 1.f) * projection;
            if (clip.w() <= 0.f)
                continue;
            const osg::Vec2f offset(clip.x() * aspect, clip.y());
            const float score = (offset / (2.f * clip.w())).length2();
            if (score > bestOffset || (score == bestOffset && clip.w() >= bestDepth))
                continue;

            // Cover must be clear from both the camera and the actual launch position.
            const auto seen = physics.castRay(eye, *point, { player }, {}, sProjectileCover);
            if (seen.mHit && seen.mHitObject != actor)
                continue;
            const auto hit = physics.castRay(
                muzzle, *point, { player }, {}, sProjectileCover, MWPhysics::CollisionType_Projectile);
            if (!hit.mHit || hit.mHitObject != actor)
                continue;
            best = FinisherTarget{ actor, *point, (hit.mHitPos - muzzle).length() };
            bestOffset = score;
            bestDepth = clip.w();
        }
        return best;
    }
}

namespace MWWorld
{

    ProjectileManager::ProjectileManager(osg::Group* parent, Resource::ResourceSystem* resourceSystem,
        MWRender::RenderingManager* rendering, MWPhysics::PhysicsSystem* physics)
        : mParent(parent)
        , mResourceSystem(resourceSystem)
        , mRendering(rendering)
        , mPhysics(physics)
        , mCleanupTimer(0.0f)
    {
    }

    /// Rotates an osg::PositionAttitudeTransform over time.
    class RotateCallback : public SceneUtil::NodeCallback<RotateCallback, osg::PositionAttitudeTransform*>
    {
    public:
        RotateCallback(const osg::Vec3f& axis = osg::Vec3f(0, -1, 0), float rotateSpeed = osg::PI * 2)
            : mAxis(axis)
            , mRotateSpeed(rotateSpeed)
        {
        }

        void operator()(osg::PositionAttitudeTransform* node, osg::NodeVisitor* nv)
        {
            double time = nv->getFrameStamp()->getSimulationTime();

            osg::Quat orient = osg::Quat(time * mRotateSpeed, mAxis);
            node->setAttitude(orient);

            traverse(node, nv);
        }

    private:
        osg::Vec3f mAxis;
        float mRotateSpeed;
    };

    void ProjectileManager::createModel(State& state, VFS::Path::NormalizedView model, const osg::Vec3f& pos,
        const osg::Quat& orient, bool rotate, bool createLight, osg::Vec4 lightDiffuseColor, const std::string& texture)
    {
        state.mNode = new osg::PositionAttitudeTransform;
        state.mNode->setNodeMask(MWRender::Mask_Effect);
        state.mNode->setPosition(pos);
        state.mNode->setAttitude(orient);

        osg::Group* attachTo = state.mNode;

        if (rotate)
        {
            osg::ref_ptr<osg::PositionAttitudeTransform> rotateNode(new osg::PositionAttitudeTransform);
            rotateNode->addUpdateCallback(new RotateCallback());
            state.mNode->addChild(rotateNode);
            attachTo = rotateNode;
        }

        osg::ref_ptr<osg::Node> projectile = mResourceSystem->getSceneManager()->getInstance(model, attachTo);

        if (state.mIdMagic.size() > 1)
        {
            for (size_t iter = 1; iter != state.mIdMagic.size(); ++iter)
            {
                std::ostringstream nodeName;
                nodeName << "Dummy" << std::setw(2) << std::setfill('0') << iter;
                const ESM::Weapon* weapon
                    = MWBase::Environment::get().getESMStore()->get<ESM::Weapon>().find(state.mIdMagic.at(iter));
                std::string nameToFind = nodeName.str();
                SceneUtil::FindByNameVisitor findVisitor(nameToFind);
                attachTo->accept(findVisitor);
                if (findVisitor.mFoundNode)
                    mResourceSystem->getSceneManager()->getInstance(
                        Misc::ResourceHelpers::correctMeshPath(VFS::Path::Normalized(weapon->mModel)),
                        findVisitor.mFoundNode);
            }
        }

        if (createLight)
        {
            osg::ref_ptr<osg::Light> projectileLight(new osg::Light);
            projectileLight->setAmbient(osg::Vec4(1.0f, 1.0f, 1.0f, 1.0f));
            projectileLight->setDiffuse(lightDiffuseColor);
            projectileLight->setSpecular(osg::Vec4(0.0f, 0.0f, 0.0f, 0.0f));
            projectileLight->setConstantAttenuation(0.f);
            projectileLight->setLinearAttenuation(0.1f);
            projectileLight->setQuadraticAttenuation(0.f);
            projectileLight->setPosition(osg::Vec4(pos, 1.0));

            SceneUtil::LightSource* projectileLightSource = new SceneUtil::LightSource;
            projectileLightSource->setNodeMask(MWRender::Mask_Lighting);
            projectileLightSource->setRadius(66.f);

            state.mNode->addChild(projectileLightSource);
            projectileLightSource->setLight(projectileLight);
        }

        state.mNode->addCullCallback(new SceneUtil::LightListCallback);

        mParent->addChild(state.mNode);

        state.mEffectAnimationTime = std::make_shared<MWRender::EffectAnimationTime>();

        SceneUtil::AssignControllerSourcesVisitor assignVisitor(state.mEffectAnimationTime);
        state.mNode->accept(assignVisitor);

        MWRender::overrideFirstRootTexture(VFS::Path::toNormalized(texture), mResourceSystem, *projectile);
    }

    void ProjectileManager::update(State& state, float duration)
    {
        state.mEffectAnimationTime->addTime(duration);
    }

    void ProjectileManager::launchMagicBolt(
        const ESM::RefId& spellId, const Ptr& caster, const osg::Vec3f& fallbackDirection, ESM::RefNum item)
    {
        osg::Vec3f pos = caster.getRefData().getPosition().asVec3();
        if (caster.getClass().isActor())
        {
            // Note: we ignore the collision box offset, this is required to make some flying creatures work as
            // intended.
            pos.z() += mPhysics->getRenderingHalfExtents(caster).z() * 2 * Constants::TorsoHeight;
        }

        // Actors can't cast target spells underwater
        if (caster.getClass().isActor() && MWBase::Environment::get().getWorld()->isUnderwater(caster.getCell(), pos))
            return;

        osg::Quat orient;
        if (caster.getClass().isActor())
            orient = osg::Quat(caster.getRefData().getPosition().rot[0], osg::Vec3f(-1, 0, 0))
                * osg::Quat(caster.getRefData().getPosition().rot[2], osg::Vec3f(0, 0, -1));
        else
            orient.makeRotate(osg::Vec3f(0, 1, 0), osg::Vec3f(fallbackDirection));

        MagicBoltState state;
        state.mSpellId = spellId;
        state.mCasterHandle = caster;
        state.mItem = item;
        MWBase::Environment::get().getWorldModel()->registerPtr(caster);
        state.mCaster = caster.getCellRef().getRefNum();

        std::string texture;

        state.mEffects = getMagicBoltData(
            state.mIdMagic, state.mSoundIds, state.mSpeed, texture, state.mSourceName, state.mSpellId);

        // Non-projectile should have been removed by getMagicBoltData
        if (state.mEffects.mList.empty())
            return;

        if (!caster.getClass().isActor() && fallbackDirection.length2() <= 0)
        {
            Log(Debug::Warning) << "Unable to launch magic bolt (direction to target is empty)";
            return;
        }

        MWWorld::ManualRef ref(*MWBase::Environment::get().getESMStore(), state.mIdMagic.at(0));
        MWWorld::Ptr ptr = ref.getPtr();

        osg::Vec4 lightDiffuseColor = getMagicBoltLightDiffuseColor(state.mEffects);

        VFS::Path::Normalized model = ptr.getClass().getCorrectedModel(ptr);
        createModel(state, model, pos, orient, true, true, lightDiffuseColor, texture);

        MWBase::SoundManager* sndMgr = MWBase::Environment::get().getSoundManager();
        for (const auto& soundid : state.mSoundIds)
        {
            MWBase::Sound* sound
                = sndMgr->playSound3D(pos, soundid, 1.0f, 1.0f, MWSound::Type::Sfx, MWSound::PlayMode::Loop);
            if (sound)
                state.mSounds.push_back(sound);
        }

        // in case there are multiple effects, the model is a dummy without geometry. Use the second effect for physics
        // shape
        if (state.mIdMagic.size() > 1)
        {
            model = Misc::ResourceHelpers::correctMeshPath(VFS::Path::Normalized(
                MWBase::Environment::get().getESMStore()->get<ESM::Weapon>().find(state.mIdMagic[1])->mModel));
        }
        state.mProjectileId = mPhysics->addProjectile(caster, pos, model, true);
        state.mToDelete = false;
        mMagicBolts.push_back(std::move(state));
    }

    bool ProjectileManager::launchProjectile(const Ptr& actor, const ConstPtr& projectile, const osg::Vec3f& pos,
        const osg::Quat& orient, const Ptr& bow, float speed, float attackStrength, const Ptr& immediateTarget)
    {
        ProjectileState state;
        state.mCaster = actor.getCellRef().getRefNum();
        state.mBowId = bow.getCellRef().getRefId();
        state.mVelocity = orient * osg::Vec3f(0, 1, 0) * speed;
        state.mIdArrow = projectile.getCellRef().getRefId();
        state.mCasterHandle = actor;
        state.mAttackStrength = attackStrength;

        std::optional<FinisherTarget> selected;
        if (actor == MWMechanics::getPlayer() && MWMechanics::physicalProjectiles() && speed > 0.f && !bow.isEmpty()
            && !MWBase::Environment::get().getWindowManager()->isGuiMode()
            && MWMechanics::supportsRangedFinisher(bow.get<ESM::Weapon>()->mBase->mData.mType))
        {
            const auto& stats = actor.getClass().getCreatureStats(actor);
            if (!stats.isDead() && !stats.mPowerStrike.token.empty() && stats.mPowerStrike.weapon == bow)
                selected = findFinisherTarget(actor, pos, *mRendering->getCamera(), *mPhysics);
        }
        if (!immediateTarget.isEmpty() && (!selected || selected->mActor != immediateTarget))
            return false;
        if (selected)
        {
            // Give close shots enough travel for the existing time-scale controller to engage
            // before collision, even when the ordinary bow speed crosses the gap in one frame.
            const float guidedSpeed = std::min(speed, std::max(1.f, selected->mContactDistance) / .22f);
            osg::Vec3f direction = selected->mHead - pos;
            const float distance = direction.normalize();
            state.mVelocity = direction * guidedSpeed;
            state.mFinisher = ProjectileState::Finisher{ MWWorld::SafePtr(selected->mActor),
                MWMechanics::nextCombatAttackId(), guidedSpeed, distance / guidedSpeed + 1.f };
        }

        MWWorld::ManualRef ref(*MWBase::Environment::get().getESMStore(), projectile.getCellRef().getRefId());
        MWWorld::Ptr ptr = ref.getPtr();

        const VFS::Path::Normalized model = ptr.getClass().getCorrectedModel(ptr);
        createModel(
            state, model, pos, selected ? lookAt(state.mVelocity) : orient, false, false, osg::Vec4(0, 0, 0, 0));
        if (!ptr.getClass().getEnchantment(ptr).empty())
            SceneUtil::addEnchantedGlow(state.mNode, mResourceSystem, ptr.getClass().getEnchantmentColor(ptr));

        state.mProjectileId = mPhysics->addProjectile(actor, pos, model, false);
        state.mToDelete = false;
        if (selected)
        {
            auto& stats = actor.getClass().getCreatureStats(actor);
            stats.mLastConsumedPowerStrikeToken = std::move(stats.mPowerStrike.token);
            stats.mPowerStrike = {};
            MWBase::Environment::get().getLuaManager()->projectileFinisher(
                actor, selected->mActor, state.mFinisher->mAttackId, "launched", stats.mLastConsumedPowerStrikeToken);
        }
        mProjectiles.push_back(std::move(state));
        return true;
    }

    namespace
    {
        // The rotation that points a missile model's +Y along a direction, as an ESM rotation (no roll).
        void missileRotation(const osg::Vec3f& direction, float (&rotation)[3])
        {
            osg::Vec3f d = direction;
            d.normalize();
            const float yaw = std::atan2(d.x(), d.y());
            const float pitch = std::asin(std::clamp(d.z(), -1.f, 1.f));
            float best = -2.f;
            // Try the sign conventions and keep the one that points the model along the flight.
            for (const float pitchSign : { -1.f, 1.f })
                for (const float yawSign : { 1.f, -1.f })
                {
                    float candidate[3] = { pitchSign * pitch, 0.f, yawSign * yaw };
                    const float fit = (Misc::Convert::makeOsgQuat(candidate) * osg::Vec3f(0, 1, 0)) * d;
                    if (fit > best)
                    {
                        best = fit;
                        std::copy(std::begin(candidate), std::end(candidate), std::begin(rotation));
                    }
                }
        }

        float missileTip(Resource::ResourceSystem* resources, VFS::Path::NormalizedView model)
        {
            static std::map<std::string, float, std::less<>> tips;
            if (const auto found = tips.find(model.value()); found != tips.end())
                return found->second;
            osg::ref_ptr<osg::Node> node = resources->getSceneManager()->getInstance(model);
            osg::ComputeBoundsVisitor bounds;
            node->accept(bounds);
            const float tip = bounds.getBoundingBox().valid() ? bounds.getBoundingBox().yMax() : 0.f;
            tips.emplace(std::string(model.value()), tip);
            return tip;
        }

        void playMissileSound(const osg::Vec3f& position, std::string_view sound)
        {
            if (!sound.empty())
                MWBase::Environment::get().getSoundManager()->playSound3D(
                    position, ESM::RefId::stringRefId(sound), 1.f, 1.f);
        }
    }

    MWWorld::Ptr ProjectileManager::placeMissile(
        const ESM::RefId& id, const osg::Vec3f& tip, const osg::Vec3f& direction, float depth)
    {
        MWBase::World* world = MWBase::Environment::get().getWorld();
        MWWorld::CellStore* cell = world->getPlayerPtr().getCell();
        if (cell == nullptr)
            return {};
        osg::Vec3f d = direction;
        d.normalize();
        MWWorld::ManualRef ref(*MWBase::Environment::get().getESMStore(), id, 1);
        const float modelTip = missileTip(mResourceSystem, ref.getPtr().getClass().getCorrectedModel(ref.getPtr()));
        ESM::Position pos;
        missileRotation(d, pos.rot);
        // The model's tip lies modelTip along its +Y from the origin.
        const osg::Vec3f forward = Misc::Convert::makeOsgQuat(pos.rot) * osg::Vec3f(0, 1, 0);
        const osg::Vec3f origin = tip + d * depth - forward * modelTip;
        pos.pos[0] = origin.x();
        pos.pos[1] = origin.y();
        pos.pos[2] = origin.z();
        return world->placeObject(ref.getPtr(), cell, pos);
    }

    MWWorld::Ptr ProjectileManager::deflectMissile(const MWWorld::Ptr& projectile, const osg::Vec3f& hitPosition,
        const osg::Vec3f& normal, const osg::Vec3f& velocity, float bounce, bool breakable, std::string_view sound)
    {
        MWBase::World* world = MWBase::Environment::get().getWorld();
        if (breakable && Misc::Rng::rollProbability(world->getPrng()) < 0.35f)
        {
            playMissileSound(hitPosition, "Brutal Arrow Break");
            return {};
        }
        playMissileSound(hitPosition, sound);
        osg::Vec3f n = normal;
        if (n.normalize() < 1e-4f)
        {
            n = -velocity;
            n.normalize();
        }
        // Reflected off the surface, losing most of its speed, with some scatter.
        osg::Vec3f reflected = velocity - n * (2.f * (velocity * n));
        reflected *= bounce;
        auto& prng = world->getPrng();
        const float scatter = reflected.length() * 0.35f;
        reflected += osg::Vec3f(Misc::Rng::deviate(0.f, scatter, prng), Misc::Rng::deviate(0.f, scatter, prng),
            Misc::Rng::deviate(0.f, scatter * 0.5f, prng));
        constexpr float maximumSpeed = 650.f;
        if (reflected.length() > maximumSpeed)
            reflected *= maximumSpeed / reflected.length();
        osg::Vec3f flight = velocity;
        flight.normalize();
        const MWWorld::Ptr placed
            = placeMissile(projectile.getCellRef().getRefId(), hitPosition - flight * 6.f + n * 4.f, flight, 0.f);
        if (placed.isEmpty())
            return {};
        const float mass = std::clamp(placed.getClass().getWeight(placed), 0.1f, 5.f);
        try
        {
            world->applyObjectImpulse(placed, reflected * mass, mass);
        }
        catch (const std::exception& e)
        {
            Log(Debug::Verbose) << "Missile could not glance off: " << e.what();
        }
        return placed;
    }

    void ProjectileManager::landProjectile(const MWWorld::Ptr& caster, const MWWorld::Ptr& target,
        const MWWorld::Ptr& bow, const MWWorld::Ptr& projectile, const osg::Vec3f& hitPosition,
        const osg::Vec3f& hitNormal, const osg::Vec3f& velocity, float attackStrength, bool hitWater,
        std::uint64_t finisherId)
    {
        std::pair<osg::Vec3f, osg::Vec3f> path;
        const std::pair<osg::Vec3f, osg::Vec3f>* flightPath = nullptr;
        auto world = MWBase::Environment::get().getWorld();
        const bool physical = MWMechanics::physicalProjectiles();
        const bool actorHit = !target.isEmpty() && target.getClass().isActor();
        osg::Vec3f flight = velocity;
        MWRender::Animation* animation = nullptr;
        std::optional<MWRender::ProjectileContact> contact;
        if (physical && flight.normalize() > 0.01f)
        {
            // Far enough back to cover a shield held well out in front, on through the body.
            path = { hitPosition - flight * 140.f, hitPosition + flight * 80.f };
            flightPath = &path;
            if (actorHit)
            {
                animation = world->getAnimation(target);
                if (!hitWater && animation != nullptr && target != MWMechanics::getPlayer())
                    contact = animation->findProjectileContact(hitPosition, flight);
            }
        }
        if (finisherId != 0 && target.getClass().isNpc() && (!contact || contact->mPart != ESM::PRT_Head))
            finisherId = 0;
        const MWMechanics::ProjectileOutcome outcome = MWMechanics::projectileHit(caster, target, bow, projectile,
            hitPosition, attackStrength, flightPath, finisherId, contact ? &*contact : nullptr);
        if (!physical)
            return;

        const ESM::RefId& id = projectile.getCellRef().getRefId();
        const auto report = [&](std::string_view result, std::string_view material, const MWWorld::Ptr& item,
                                const osg::Vec3f& where) {
            MWBase::Environment::get().getLuaManager()->missileLanded(
                caster, target, item, id, result, material, where);
        };
        if (hitWater || velocity.length2() < 1e-4f)
        {
            report("water", "", {}, hitPosition);
            return;
        }

        const VFS::Path::Normalized model = projectile.getClass().getCorrectedModel(projectile);
        // An enchanted missile spends its enchantment and itself on whatever it strikes.
        const bool enchanted = !projectile.getClass().getEnchantment(projectile).empty();
        // Thrown weapons (stars, darts, knives) do not snap.
        const auto* type = MWMechanics::getWeaponType(projectile.get<ESM::Weapon>()->mBase->mData.mType);
        const bool breakable = type->mWeaponClass == ESM::WeaponType::Ammo;
        const auto glance = [&](const osg::Vec3f& where, const osg::Vec3f& normal, float bounce, bool canBreak,
                                std::string_view sound, std::string_view material) {
            if (enchanted)
            {
                report("spent", material, {}, where);
                return;
            }
            const MWWorld::Ptr loose = deflectMissile(projectile, where, normal, velocity, bounce, canBreak, sound);
            report(loose.isEmpty() ? "broken" : "glanced", material, loose, where);
        };

        if (actorHit)
        {
            switch (outcome)
            {
                case MWMechanics::ProjectileOutcome::Body:
                case MWMechanics::ProjectileOutcome::Missed:
                case MWMechanics::ProjectileOutcome::NoActor:
                {
                    // The player's own body is never shown with missiles in it (it would fill the view).
                    if (target == MWMechanics::getPlayer())
                    {
                        playMissileSound(hitPosition, "Brutal Arrow Flesh");
                        report("body", "flesh", {}, hitPosition);
                        return;
                    }
                    const ESM::RefId item = enchanted ? ESM::RefId() : id;
                    if (contact && animation->attachStuckProjectile(model, *contact, flight, item))
                    {
                        if (!item.empty())
                            target.getClass().getContainerStore(target).add(item, 1);
                        playMissileSound(contact->mPosition, "Brutal Arrow Flesh");
                        report("body", "flesh", {}, contact->mPosition);
                    }
                    else
                        // Its line passes what is seen of the body: it grazed by and falls.
                        glance(hitPosition, -flight, 0.1f, false, {}, "flesh");
                    return;
                }
                case MWMechanics::ProjectileOutcome::Shield:
                {
                    auto* npc = dynamic_cast<MWRender::NpcAnimation*>(animation);
                    if (npc != nullptr && npc->attachStuckProjectileToShield(
                            model, hitPosition, flight, path, enchanted ? ESM::RefId() : id))
                    {
                        playMissileSound(hitPosition, "Brutal Arrow Wood");
                        report("shield", "wood", {}, hitPosition);
                        return;
                    }
                    glance(hitPosition, -flight, 0.15f, breakable, {}, "wood");
                    return;
                }
                case MWMechanics::ProjectileOutcome::MetalShield:
                case MWMechanics::ProjectileOutcome::Parried:
                    glance(hitPosition, -flight, 0.25f, breakable, {}, "metal");
                    return;
            }
            return;
        }

        // The physical hit is on the collision shape, which can stand off from what is seen. Look along the
        // flight for the visible surface, a little either side of it.
        const MWRender::RenderingManager::SurfaceResult surface
            = mRendering->castRayForSurface(hitPosition - flight * 24.f, hitPosition + flight * 64.f);
        const MWWorld::Ptr struck = !surface.mObject.isEmpty() ? surface.mObject : target;
        if (!struck.isEmpty() && struck.getClass().isItem(struck))
        {
            // A loose item takes the knock and the missile drops off it.
            const float itemMass = std::clamp(struck.getClass().getWeight(struck), 0.2f, 50.f);
            const float missileMass = std::max(projectile.getClass().getWeight(projectile), 0.1f);
            try
            {
                world->applyObjectImpulse(struck, velocity * (missileMass * 0.5f), itemMass);
            }
            catch (const std::exception&)
            {
            }
            glance(surface.mHit ? surface.mPosition : hitPosition, -flight, 0.08f, false, {}, "item");
            return;
        }

        const bool terrain = surface.mTerrain || (target.isEmpty() && !surface.mHit && world->isCellExterior());
        const bool door = !struck.isEmpty() && struck.getClass().isDoor();
        const std::string structure
            = !struck.isEmpty() ? std::string(struck.getClass().getCorrectedModel(struck).value()) : std::string();
        const MWWorld::SurfaceMaterial material = MWWorld::classifySurface(surface.mTexture, structure,
            door ? MWWorld::SurfaceMaterial::Wood
                 : (terrain ? MWWorld::SurfaceMaterial::Earth : MWWorld::SurfaceMaterial::Stone));
        const std::string_view materialName = MWWorld::getSurfaceMaterialName(material);
        const osg::Vec3f point = surface.mHit ? surface.mPosition : hitPosition;
        const osg::Vec3f normal = surface.mHit ? surface.mNormal : hitNormal;
        if (enchanted)
        {
            report("spent", materialName, {}, point);
            return;
        }
        if (MWWorld::takesMissiles(material) && !door)
        {
            // Driven in a little way, deeper into earth.
            const float depth = material == MWWorld::SurfaceMaterial::Earth ? 7.f : 5.f;
            const MWWorld::Ptr placed = placeMissile(id, point, flight, depth);
            playMissileSound(point,
                material == MWWorld::SurfaceMaterial::Wood ? "Brutal Arrow Wood" : "Brutal Arrow Flesh");
            report("stuck", materialName, placed, point);
            return;
        }
        if (door)
        {
            // Doors move: the missile drops off them rather than staying in the air when one opens.
            glance(point, normal, 0.05f, false, "Brutal Arrow Wood", materialName);
            return;
        }
        glance(point, normal, 0.25f, breakable,
            material == MWWorld::SurfaceMaterial::Metal ? "Brutal Arrow Metal" : "Brutal Arrow Stone", materialName);
    }

    void ProjectileManager::updateCasters()
    {
        for (auto& state : mProjectiles)
        {
            state.mCasterHandle = state.getCaster();
            mPhysics->setCaster(state.mProjectileId, state.mCasterHandle);
        }

        for (auto& state : mMagicBolts)
        {
            if (!state.mCaster.isSet())
                continue;

            state.mCasterHandle = state.getCaster();
            if (state.mCasterHandle.isEmpty())
            {
                Log(Debug::Error) << "Couldn't find caster with ID " << state.mCaster;
                cleanupMagicBolt(state);
                continue;
            }
            mPhysics->setCaster(state.mProjectileId, state.mCasterHandle);
        }
    }

    void ProjectileManager::update(float dt)
    {
        periodicCleanup(dt);
        moveProjectiles(dt);
        moveMagicBolts(dt);
    }

    void ProjectileManager::periodicCleanup(float dt)
    {
        mCleanupTimer -= dt;
        if (mCleanupTimer <= 0.0f)
        {
            mCleanupTimer = 2.0f;

            auto isCleanable = [](const ProjectileManager::State& state) -> bool {
                const float farawayThreshold = 72000.0f;
                osg::Vec3 playerPos = MWMechanics::getPlayer().getRefData().getPosition().asVec3();
                return (state.mNode->getPosition() - playerPos).length2() >= farawayThreshold * farawayThreshold;
            };

            for (auto& projectileState : mProjectiles)
            {
                if (isCleanable(projectileState))
                    cleanupProjectile(projectileState);
            }

            for (auto& magicBoltState : mMagicBolts)
            {
                if (isCleanable(magicBoltState))
                    cleanupMagicBolt(magicBoltState);
            }
        }
    }

    void ProjectileManager::moveMagicBolts(float duration)
    {
        const bool normaliseRaceSpeed = Settings::game().mNormaliseRaceSpeed;
        for (auto& magicBoltState : mMagicBolts)
        {
            if (magicBoltState.mToDelete)
                continue;

            auto* projectile = mPhysics->getProjectile(magicBoltState.mProjectileId);
            if (!projectile->isActive())
                continue;
            // If the actor caster is gone, the magic bolt needs to be removed from the scene during the next frame.
            MWWorld::Ptr caster = magicBoltState.getCaster();
            if (!caster.isEmpty() && caster.getClass().isActor())
            {
                if (caster.getCellRef().getCount() <= 0 || caster.getClass().getCreatureStats(caster).isDead())
                {
                    cleanupMagicBolt(magicBoltState);
                    continue;
                }
            }

            const auto& store = *MWBase::Environment::get().getESMStore();
            osg::Quat orient = magicBoltState.mNode->getAttitude();
            static float fTargetSpellMaxSpeed
                = store.get<ESM::GameSetting>().find("fTargetSpellMaxSpeed")->mValue.getFloat();
            float speed = fTargetSpellMaxSpeed * magicBoltState.mSpeed;
            if (!normaliseRaceSpeed && !caster.isEmpty() && caster.getClass().isNpc())
            {
                const auto npc = caster.get<ESM::NPC>()->mBase;
                const auto race = store.get<ESM::Race>().find(npc->mRace);
                speed *= npc->isMale() ? race->mData.mMaleWeight : race->mData.mFemaleWeight;
            }
            osg::Vec3f direction = orient * osg::Vec3f(0, 1, 0);
            direction.normalize();
            projectile->setVelocity(direction * speed);

            update(magicBoltState, duration);

            for (const auto& sound : magicBoltState.mSounds)
            {
                sound->setVelocity(direction * speed);
            }

            // For AI actors, get combat targets to use in the ray cast. Only those targets will return a positive hit
            // result.
            std::vector<MWWorld::Ptr> targetActors;
            if (!caster.isEmpty() && caster.getClass().isActor() && caster != MWMechanics::getPlayer())
                caster.getClass().getCreatureStats(caster).getAiSequence().getCombatTargets(targetActors);
            projectile->setValidTargets(targetActors);
        }
    }

    bool ProjectileManager::guideFinisher(ProjectileState& state, float duration)
    {
        if (!state.mFinisher)
            return false;
        auto& finisher = *state.mFinisher;
        const auto target = finisher.mTarget.ptrOrEmpty();
        const auto player = state.getCaster();
        const auto head = finisherHead(target);
        finisher.mTimeLeft -= duration;
        if (!head || player.isEmpty() || player.getClass().getCreatureStats(player).isDead()
            || finisher.mTimeLeft <= 0.f || !MWMechanics::physicalProjectiles())
        {
            state.mFinisher.reset();
            return false;
        }
        const osg::Vec3f position(state.mNode->getPosition());
        osg::Vec3f direction = *head - position;
        const float distance = direction.normalize();
        if (distance <= 1e-4f)
        {
            state.mFinisher.reset();
            return false;
        }
        state.mVelocity = direction * finisher.mSpeed;

        // Probe once the head is close enough for the actor's collision bounds to enter the last stretch.
        const float leadDistance = finisher.mSpeed * sSlowMotionLead;
        if (!finisher.mSlowMotionRequested && distance <= leadDistance + 128.f)
        {
            const auto hit = mPhysics->castRay(
                position, *head, { player }, {}, sProjectileCover, MWPhysics::CollisionType_Projectile);
            if (hit.mHit && hit.mHitObject == target
                && (hit.mHitPos - position).length2() <= leadDistance * leadDistance)
            {
                finisher.mSlowMotionRequested = true;
                MWBase::Environment::get().getLuaManager()->projectileFinisher(
                    player, target, finisher.mAttackId, "approach");
            }
        }
        return true;
    }

    void ProjectileManager::moveProjectiles(float duration)
    {
        for (auto& projectileState : mProjectiles)
        {
            if (projectileState.mToDelete)
                continue;

            auto* projectile = mPhysics->getProjectile(projectileState.mProjectileId);
            if (!projectile->isActive())
                continue;
            // Vanilla uses a tenth of gravity, since aerodynamics are not simulated; physical missiles
            // fall on a real arc.
            if (!guideFinisher(projectileState, duration))
                projectileState.mVelocity -= osg::Vec3f(0, 0, MWMechanics::getProjectileGravity()) * duration;

            projectile->setVelocity(projectileState.mVelocity);

            projectileState.mNode->setAttitude(lookAt(projectileState.mVelocity));

            update(projectileState, duration);

            MWWorld::Ptr caster = projectileState.getCaster();

            // For AI actors, get combat targets to use in the ray cast. Only those targets will return a positive hit
            // result.
            std::vector<MWWorld::Ptr> targetActors;
            if (!caster.isEmpty() && caster.getClass().isActor() && caster != MWMechanics::getPlayer())
                caster.getClass().getCreatureStats(caster).getAiSequence().getCombatTargets(targetActors);
            projectile->setValidTargets(targetActors);
        }
    }

    void ProjectileManager::processHits()
    {
        for (auto& projectileState : mProjectiles)
        {
            if (projectileState.mToDelete)
                continue;

            auto* projectile = mPhysics->getProjectile(projectileState.mProjectileId);

            const auto pos = projectile->getSimulationPosition();
            projectileState.mNode->setPosition(pos);

            if (projectile->isActive())
                continue;

            const auto target = projectile->getTarget();
            auto caster = projectileState.getCaster();
            assert(target != caster);

            if (caster.isEmpty())
                caster = target;

            // Try to get a Ptr to the bow that was used. It might no longer exist.
            MWWorld::ManualRef projectileRef(*MWBase::Environment::get().getESMStore(), projectileState.mIdArrow);
            MWWorld::Ptr bow = projectileRef.getPtr();
            if (!caster.isEmpty() && projectileState.mIdArrow != projectileState.mBowId)
            {
                MWWorld::InventoryStore& inv = caster.getClass().getInventoryStore(caster);
                MWWorld::ContainerStoreIterator invIt = inv.getSlot(MWWorld::InventoryStore::Slot_CarriedRight);
                if (invIt != inv.end() && invIt->getCellRef().getRefId() == projectileState.mBowId)
                    bow = *invIt;
            }

            const auto hitPosition = Misc::Convert::toOsg(projectile->getHitPosition());

            if (projectile->getHitWater())
                mRendering->emitWaterRipple(hitPosition);

            std::uint64_t finisherId = 0;
            if (projectileState.mFinisher && !target.isEmpty() && !projectile->getHitWater()
                && projectileState.mFinisher->mTarget.ptrOrEmpty() == target)
            {
                const auto head = finisherHead(target);
                const float headHeight = std::clamp(mPhysics->getHalfExtents(target).z() * .25f, 8.f, 32.f);
                if (head && (target.getClass().isNpc() || std::abs(hitPosition.z() - head->z()) <= headHeight))
                    finisherId = projectileState.mFinisher->mAttackId;
            }
            landProjectile(caster, target, bow, projectileRef.getPtr(), hitPosition,
                Misc::Convert::toOsg(projectile->getHitNormal()), projectileState.mVelocity,
                projectileState.mAttackStrength, projectile->getHitWater(), finisherId);
            projectileState.mToDelete = true;
        }
        const MWWorld::ESMStore& esmStore = *MWBase::Environment::get().getESMStore();
        for (auto& magicBoltState : mMagicBolts)
        {
            if (magicBoltState.mToDelete)
                continue;

            auto* projectile = mPhysics->getProjectile(magicBoltState.mProjectileId);

            const auto pos = projectile->getSimulationPosition();
            magicBoltState.mNode->setPosition(pos);
            for (const auto& sound : magicBoltState.mSounds)
                sound->setPosition(pos);

            const Ptr caster = magicBoltState.getCaster();

            const MWBase::World& world = *MWBase::Environment::get().getWorld();
            const bool active = projectile->isActive();
            if (active && !world.isUnderwater(caster.getCell(), pos))
                continue;

            const Ptr target = !active ? projectile->getTarget() : Ptr();

            assert(target != caster);

            MWMechanics::CastSpell cast(caster, target);
            cast.mHitPosition = !active ? Misc::Convert::makeOsgVec3f(projectile->getHitPosition()) : pos;
            cast.mId = magicBoltState.mSpellId;
            cast.mSourceName = magicBoltState.mSourceName;
            cast.mItem = magicBoltState.mItem;
            // Grab original effect list so the indices are correct
            const ESM::EffectList* effects;
            if (const ESM::Spell* spell = esmStore.get<ESM::Spell>().search(magicBoltState.mSpellId))
                effects = &spell->mEffects;
            else
            {
                MWWorld::ManualRef ref(esmStore, magicBoltState.mSpellId);
                const MWWorld::Ptr& ptr = ref.getPtr();
                effects = &esmStore.get<ESM::Enchantment>().find(ptr.getClass().getEnchantment(ptr))->mEffects;
            }
            cast.inflict(target, *effects, ESM::RT_Target);

            magicBoltState.mToDelete = true;
        }

        for (auto& projectileState : mProjectiles)
        {
            if (projectileState.mToDelete)
                cleanupProjectile(projectileState);
        }

        for (auto& magicBoltState : mMagicBolts)
        {
            if (magicBoltState.mToDelete)
                cleanupMagicBolt(magicBoltState);
        }
        mProjectiles.erase(std::remove_if(mProjectiles.begin(), mProjectiles.end(),
                               [](const State& state) { return state.mToDelete; }),
            mProjectiles.end());
        mMagicBolts.erase(
            std::remove_if(mMagicBolts.begin(), mMagicBolts.end(), [](const State& state) { return state.mToDelete; }),
            mMagicBolts.end());
    }

    void ProjectileManager::cleanupProjectile(ProjectileManager::ProjectileState& state)
    {
        mParent->removeChild(state.mNode);
        mPhysics->removeProjectile(state.mProjectileId);
        state.mToDelete = true;
    }

    void ProjectileManager::cleanupMagicBolt(ProjectileManager::MagicBoltState& state)
    {
        mParent->removeChild(state.mNode);
        mPhysics->removeProjectile(state.mProjectileId);
        state.mToDelete = true;
        for (size_t soundIter = 0; soundIter != state.mSounds.size(); soundIter++)
        {
            MWBase::Environment::get().getSoundManager()->stopSound(state.mSounds.at(soundIter));
        }
    }

    void ProjectileManager::clear()
    {
        for (auto& mProjectile : mProjectiles)
            cleanupProjectile(mProjectile);
        mProjectiles.clear();

        for (auto& mMagicBolt : mMagicBolts)
            cleanupMagicBolt(mMagicBolt);
        mMagicBolts.clear();
    }

    void ProjectileManager::write(ESM::ESMWriter& writer, Loading::Listener& progress) const
    {
        for (const ProjectileState& projectile : mProjectiles)
        {
            writer.startRecord(ESM::REC_PROJ);

            ESM::ProjectileState state;
            state.mId = projectile.mIdArrow;
            state.mPosition = ESM::Vector3(osg::Vec3f(projectile.mNode->getPosition()));
            state.mOrientation = ESM::Quaternion(osg::Quat(projectile.mNode->getAttitude()));
            state.mCaster = projectile.mCaster;

            state.mBowId = projectile.mBowId;
            state.mVelocity = projectile.mVelocity;
            state.mAttackStrength = projectile.mAttackStrength;

            state.save(writer);

            writer.endRecord(ESM::REC_PROJ);
        }

        for (const MagicBoltState& bolt : mMagicBolts)
        {
            writer.startRecord(ESM::REC_MPRJ);

            ESM::MagicBoltState state;
            state.mId = bolt.mIdMagic.at(0);
            state.mPosition = ESM::Vector3(osg::Vec3f(bolt.mNode->getPosition()));
            state.mOrientation = ESM::Quaternion(osg::Quat(bolt.mNode->getAttitude()));
            state.mCaster = bolt.mCaster;
            state.mItem = bolt.mItem;
            state.mSpellId = bolt.mSpellId;
            state.mSpeed = bolt.mSpeed;

            state.save(writer);

            writer.endRecord(ESM::REC_MPRJ);
        }
    }

    bool ProjectileManager::readRecord(ESM::ESMReader& reader, uint32_t type)
    {
        if (type == ESM::REC_PROJ)
        {
            ESM::ProjectileState esm;
            esm.load(reader);

            ProjectileState state;
            state.mCaster = esm.mCaster;
            state.mBowId = esm.mBowId;
            state.mVelocity = esm.mVelocity;
            state.mIdArrow = esm.mId;
            state.mAttackStrength = esm.mAttackStrength;
            state.mToDelete = false;

            VFS::Path::Normalized model;
            try
            {
                MWWorld::ManualRef ref(*MWBase::Environment::get().getESMStore(), esm.mId);
                MWWorld::Ptr ptr = ref.getPtr();
                model = ptr.getClass().getCorrectedModel(ptr);

                state.mProjectileId
                    = mPhysics->addProjectile(state.getCaster(), osg::Vec3f(esm.mPosition), model, false);
            }
            catch (const std::exception& e)
            {
                Log(Debug::Warning) << "Failed to add projectile for " << esm.mId
                                    << " while reading projectile record: " << e.what();
                return true;
            }

            createModel(state, model, osg::Vec3f(esm.mPosition), osg::Quat(esm.mOrientation), false, false,
                osg::Vec4(0, 0, 0, 0));

            mProjectiles.push_back(std::move(state));
            return true;
        }
        if (type == ESM::REC_MPRJ)
        {
            ESM::MagicBoltState esm;
            esm.load(reader);

            MagicBoltState state;
            state.mIdMagic.push_back(esm.mId);
            state.mSpellId = esm.mSpellId;
            state.mCaster = esm.mCaster;
            state.mToDelete = false;
            state.mItem = esm.mItem;
            std::string texture;

            try
            {
                state.mEffects = getMagicBoltData(
                    state.mIdMagic, state.mSoundIds, state.mSpeed, texture, state.mSourceName, state.mSpellId);
            }
            catch (const std::exception& e)
            {
                Log(Debug::Warning) << "Failed to recreate magic projectile for " << esm.mId << " and spell "
                                    << state.mSpellId << " while reading projectile record: " << e.what();
                return true;
            }

            state.mSpeed = esm.mSpeed; // speed is derived from non-projectile effects as well as
                                       // projectile effects, so we can't calculate it from the save
                                       // file's effect list, which is already trimmed of non-projectile
                                       // effects. We need to use the stored value.

            VFS::Path::Normalized model;
            try
            {
                MWWorld::ManualRef ref(*MWBase::Environment::get().getESMStore(), state.mIdMagic.at(0));
                MWWorld::Ptr ptr = ref.getPtr();
                model = ptr.getClass().getCorrectedModel(ptr);
            }
            catch (const std::exception& e)
            {
                Log(Debug::Warning) << "Failed to get model for " << state.mIdMagic.at(0)
                                    << " while reading projectile record: " << e.what();
                return true;
            }

            osg::Vec4 lightDiffuseColor = getMagicBoltLightDiffuseColor(state.mEffects);
            createModel(state, model, osg::Vec3f(esm.mPosition), osg::Quat(esm.mOrientation), true, true,
                lightDiffuseColor, texture);
            state.mProjectileId = mPhysics->addProjectile(state.getCaster(), osg::Vec3f(esm.mPosition), model, true);

            MWBase::SoundManager* sndMgr = MWBase::Environment::get().getSoundManager();
            for (const auto& soundid : state.mSoundIds)
            {
                MWBase::Sound* sound = sndMgr->playSound3D(
                    esm.mPosition, soundid, 1.0f, 1.0f, MWSound::Type::Sfx, MWSound::PlayMode::Loop);
                if (sound)
                    state.mSounds.push_back(sound);
            }

            mMagicBolts.push_back(std::move(state));
            return true;
        }

        return false;
    }

    size_t ProjectileManager::countSavedGameRecords() const
    {
        return mMagicBolts.size() + mProjectiles.size();
    }

    void ProjectileManager::saveLoaded(const ESM::ESMReader& reader)
    {
        // Can't do this in readRecord because the vectors might get reallocated as they grow
        if (reader.mActorIdConverter)
        {
            for (ProjectileState& projectile : mProjectiles)
                reader.mActorIdConverter->convert(projectile.mCaster, projectile.mCaster.mIndex);
            for (MagicBoltState& bolt : mMagicBolts)
                reader.mActorIdConverter->convert(bolt.mCaster, bolt.mCaster.mIndex);
        }
    }

    MWWorld::Ptr ProjectileManager::State::getCaster()
    {
        if (!mCasterHandle.isEmpty())
            return mCasterHandle;

        return MWBase::Environment::get().getWorldModel()->getPtr(mCaster);
    }

}
