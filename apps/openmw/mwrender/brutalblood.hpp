#ifndef OPENMW_MWRENDER_BRUTALBLOOD_H
#define OPENMW_MWRENDER_BRUTALBLOOD_H

#include "npcblood.hpp"

#include <components/esm3/refnum.hpp>
#include <components/sceneutil/surfacedecals.hpp>

#include <osg/Group>
#include <osg/StateSet>
#include <osg/Vec3f>

#include <array>
#include <cstdint>
#include <deque>
#include <functional>
#include <map>
#include <optional>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

namespace Resource
{
    class ResourceSystem;
}

namespace MWWorld
{
    class Ptr;
}

namespace MWRender
{
    class EffectManager;

    // Blood that outlives the hit: smears on melee weapons and splatters on
    // floors and walls, including pools under bodies and transfers from wet soles.
    // Weapon smears belong to the item instance, so they survive sheathing and a change of view.
    // Blood dries over game time, and rain or immersion can wash it off.
    class BrutalBlood
    {
    public:
        // Texture styles shared by weapon smears and world splatters.
        enum Style : unsigned
        {
            Style_Slash = 0,
            Style_Small = 1,
            Style_World = 2, // Style_World + 0..5: the six floor/wall splatters
            Style_Count = Style_World + 6
        };

        struct WeaponMark
        {
            unsigned style = Style_Slash;
            std::string model;
            unsigned ordinal = 0;
            std::uint64_t fingerprint = 0;
            std::vector<SceneUtil::SurfaceDecals::Anchor> anchors;
            double hour = 0; // game hours when the blood was left
        };

        struct ItemBlood
        {
            std::vector<WeaponMark> marks;
            double wash = 0; // extra game hours of wear from rain
            unsigned revision = 0;
            float dripUntil = 0; // real seconds, not saved
        };

        enum class Mark : std::uint32_t
        {
            Imported,
            Spray,
            Pool,
            Footprint
        };

        enum class Impact
        {
            Chop,
            Slash,
            Puncture
        };

        struct Surface
        {
            osg::Vec3f position;
            osg::Vec3f normal;
            bool porous = false;
        };

        // Meshes and procedural masks are reconstructed from these parameters.
        struct Splat
        {
            std::string worldspace;
            bool exterior = false;
            bool porous = false;
            osg::Vec3f position, normal, tangent;
            float width = 32.f, height = 32.f;
            float opacity = 1.f;
            unsigned style = Style_World;
            Mark mark = Mark::Imported;
            double hour = 0;
            double wash = 0;
            float started = 0.f;
            std::uint32_t seed = 0;
            std::uint32_t id = 0;
        };

        // Provided by the renderer, which projects and adds splatter meshes. It returns the
        // world-space triangles of static surfaces around a point, or nothing
        // while that place is not loaded.
        struct SceneAccess
        {
            std::function<std::optional<Surface>(const osg::Vec3f&, const osg::Vec3f&)> castStatic;
            std::function<void(const osg::Vec3f& center, float radius, std::vector<osg::Vec3d>& positions,
                std::vector<std::array<unsigned, 3>>& triangles)>
                collectStatic;
        };

        static constexpr std::size_t MaxWeaponMarks = 24;
        static constexpr std::size_t MaxWeaponMarkVertices = 3072;
        static constexpr std::size_t MaxSplats = 320;
        static constexpr double WeaponLifetimeHours = 24.;

        BrutalBlood(Resource::ResourceSystem* resourceSystem, osg::Group* parent, EffectManager* effects,
            SceneAccess access);
        ~BrutalBlood();

        static BrutalBlood* get();

        // Immutable render states, shared by every mark of the same look.
        osg::StateSet* weaponState(unsigned style, float fade, float dry);
        bool hasStyle(unsigned style);

        double now() const;
        // Game hours that passed in the last update; waiting and resting count.
        double elapsedHours() const { return mElapsedHours; }
        void noteWeaponAdd() { ++mWeaponAdds; }
        // Unique across loads, so a rendered weapon always notices a change.
        unsigned nextRevision() { return ++mRevision; }
        float realTime() const { return mRealTime; }

        // Weapons. Keys are item instance reference numbers.
        ItemBlood* item(ESM::RefNum key);
        ItemBlood& createItem(ESM::RefNum key);
        void clearItem(ESM::RefNum key);
        bool isBloody(ESM::RefNum key) const;
        void addWear(ESM::RefNum key, double hours);
        static float markFade(double age);
        static float markDry(double age);

        void hit(const MWWorld::Ptr& victim, const MWWorld::Ptr& attacker, osg::Vec3f position, osg::Vec3f direction,
            Impact impact, bool power, bool lethal, float damage);
        void queueCutPool(const MWWorld::Ptr& actor, BloodPoolOutlet outlet);
        void drip(const MWWorld::Ptr& actor, const osg::Vec3f& tip);
        void queueStep(const MWWorld::Ptr& actor, bool left);

        void update(float dt, const osg::Vec3f& player, const std::string& worldspace, bool exterior, bool raining);
        void clear();
        void unloadSplats();

        std::string save() const;
        bool load(std::string_view data);
        std::map<std::string, double> stats() const;

        EffectManager* effects() const { return mEffects; }
        std::uint32_t random();
        float random01() { return float(random() & 0xffffff) / float(0x1000000); }

    private:
        struct PoolSource
        {
            std::string bone;
            osg::Vec3f offset, stablePosition;
            float width = 96.f;
            float expires = 0.f;
            float stableSince = -1.f;
        };
        struct Sole
        {
            float blood = 0.f, driesAt = 0.f, lastContact = -100.f;
            double stainedHour = 0.;
            osg::Vec3f lastPosition;
        };
        struct Step
        {
            ESM::RefNum actor;
            bool left;
        };

        void addSplat(Splat splat);
        void spray(const Splat& base, const osg::Vec3f& origin, osg::Vec3f direction, float range, float size);
        void startPool(const MWWorld::Ptr& actor, const osg::Vec3f& wound, float width);
        void updatePools(const osg::Vec3f& player, const std::string& worldspace);
        void updateShoes(double hour);
        void plantFoot(const MWWorld::Ptr& actor, bool left, double hour);
        float poolContact(const Surface& surface, const osg::Vec3f& forward, float size,
            const std::string& worldspace, double hour) const;
        float poolScale(const Splat& splat, double hour) const;
        void buildSplat(const Splat& splat);
        void removeBuilt(std::uint32_t id);
        osg::StateSet* state(unsigned style, float fade, float dry, bool weapon = false);
        osg::ref_ptr<osg::StateSet> createState(osg::Image* image, float fade, float dry, bool weapon);

        Resource::ResourceSystem* mResourceSystem;
        osg::ref_ptr<osg::Group> mParent;
        osg::ref_ptr<osg::Group> mSplatRoot;
        EffectManager* mEffects;
        SceneAccess mAccess;
        std::map<std::uint64_t, osg::ref_ptr<osg::StateSet>> mStates;
        std::array<int, Style_Count> mStyleAvailable{};
        std::map<ESM::RefNum, ItemBlood> mItems;
        std::deque<Splat> mSplats;
        std::map<std::pair<ESM::RefNum, BloodPoolOutlet>, PoolSource> mPoolSources;
        std::map<ESM::RefNum, std::array<Sole, 2>> mShoes;
        std::vector<Step> mSteps;
        struct Built
        {
            osg::ref_ptr<osg::Node> node;
            osg::ref_ptr<osg::Uniform> fade, dry, scale;
            int fadeLevel = -1, dryLevel = -1;
        };
        osg::ref_ptr<osg::StateSet> mProceduralState;
        std::map<std::uint32_t, Built> mBuilt;
        std::map<std::uint32_t, float> mRetryAt;
        std::uint32_t mNextId = 1;
        unsigned mRevision = 0;
        std::uint32_t mRandom = 0x51ab0d17;
        float mRealTime = 0, mNextScan = 0, mNextContactScan = 0;
        double mLastHour = -1, mElapsedHours = 0, mSlowestBuildMs = 0;
        std::uint64_t mThrown = 0, mLanded = 0, mBuiltCount = 0, mWeaponAdds = 0;
    };
}

#endif
