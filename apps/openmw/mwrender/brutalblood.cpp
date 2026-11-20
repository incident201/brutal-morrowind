#include "brutalblood.hpp"

#include "../mwbase/environment.hpp"
#include "../mwbase/world.hpp"
#include "../mwworld/timestamp.hpp"
#include "vismask.hpp"

#include <components/debug/debuglog.hpp>
#include <components/resource/imagemanager.hpp>
#include <components/resource/resourcesystem.hpp>
#include <components/resource/scenemanager.hpp>
#include <components/sceneutil/decalprojector.hpp>
#include <components/sceneutil/depth.hpp>
#include <components/sceneutil/texturetype.hpp>
#include <components/settings/values.hpp>
#include <components/vfs/manager.hpp>

#include <osg/BlendFunc>
#include <osg/ColorMask>
#include <osg/Geometry>
#include <osg/MatrixTransform>
#include <osg/PolygonOffset>
#include <osg/TexMat>
#include <osg/Texture2D>
#include <osgUtil/IncrementalCompileOperation>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstring>
#include <limits>
#include <stdexcept>

namespace MWRender
{
    namespace
    {
        BrutalBlood* sInstance = nullptr;
        constexpr int FadeLevels = 12, DryLevels = 6;
        constexpr std::size_t MaxSaveBytes = 2000000;
        // Splatters are rebuilt only near the player; farther ones stay as records.
        constexpr float BuildRange = 6000.f, UnloadRange = 7500.f;

        std::string texturePath(unsigned style)
        {
            if (style == BrutalBlood::Style_Slash)
                return "textures/brutal_dm/model_blood_slash.png";
            if (style == BrutalBlood::Style_Small)
                return "textures/brutal_dm/model_blood_small.png";
            return "textures/brutal_dm/world_blood_" + std::to_string(style - BrutalBlood::Style_World + 1) + ".png";
        }

        struct Writer
        {
            std::string data = "BWB1";
            void u32(std::uint32_t n)
            {
                for (unsigned i = 0; i < 4; ++i)
                    data.push_back(char((n >> (i * 8)) & 255));
            }
            void f32(float f)
            {
                std::uint32_t n;
                std::memcpy(&n, &f, 4);
                u32(n);
            }
            void f64(double f)
            {
                std::uint64_t n;
                std::memcpy(&n, &f, 8);
                u32(std::uint32_t(n));
                u32(std::uint32_t(n >> 32));
            }
            void vec(const osg::Vec3f& v)
            {
                for (unsigned i = 0; i < 3; ++i)
                    f32(v[i]);
            }
            void string(const std::string& s)
            {
                u32(s.size());
                data += s;
            }
        };

        struct Reader
        {
            std::string_view data;
            std::size_t p = 4;
            std::uint32_t u32()
            {
                if (p + 4 > data.size())
                    throw std::runtime_error("Truncated blood record");
                std::uint32_t n = 0;
                for (unsigned i = 0; i < 4; ++i)
                    n |= std::uint32_t(static_cast<unsigned char>(data[p++])) << (i * 8);
                return n;
            }
            float f32()
            {
                const auto n = u32();
                float f;
                std::memcpy(&f, &n, 4);
                if (!std::isfinite(f))
                    throw std::runtime_error("Nonfinite blood record");
                return f;
            }
            double f64()
            {
                std::uint64_t n = u32();
                n |= std::uint64_t(u32()) << 32;
                double f;
                std::memcpy(&f, &n, 8);
                if (!std::isfinite(f))
                    throw std::runtime_error("Nonfinite blood record");
                return f;
            }
            osg::Vec3f vec()
            {
                osg::Vec3f v;
                for (unsigned i = 0; i < 3; ++i)
                    v[i] = f32();
                return v;
            }
            std::string string()
            {
                const auto size = u32();
                if (size > 1024 || p + size > data.size())
                    throw std::runtime_error("Invalid blood record string");
                std::string s(data.substr(p, size));
                p += size;
                return s;
            }
        };

        float smooth(float x)
        {
            x = std::clamp(x, 0.f, 1.f);
            return x * x * (3.f - 2.f * x);
        }

        // Exterior splatters wash away in about an hour of rain.
        constexpr double SplatWashHours = 1.;
    }

    BrutalBlood::BrutalBlood(
        Resource::ResourceSystem* resourceSystem, osg::Group* parent, EffectManager* effects, SceneAccess access)
        : mResourceSystem(resourceSystem)
        , mParent(parent)
        , mSplatRoot(new osg::Group)
        , mEffects(effects)
        , mAccess(std::move(access))
    {
        mSplatRoot->setName("Brutal blood splatters");
        mSplatRoot->setNodeMask(Mask_Effect);
        mParent->addChild(mSplatRoot);
        mStyleAvailable.fill(-1);
        sInstance = this;
    }

    BrutalBlood::~BrutalBlood()
    {
        if (sInstance == this)
            sInstance = nullptr;
        mParent->removeChild(mSplatRoot);
    }

    BrutalBlood* BrutalBlood::get()
    {
        return sInstance;
    }

    bool BrutalBlood::hasStyle(unsigned style)
    {
        if (style >= Style_Count)
            return false;
        if (mStyleAvailable[style] < 0)
            mStyleAvailable[style] = mResourceSystem->getVFS()->exists(VFS::Path::Normalized(texturePath(style)));
        return mStyleAvailable[style] == 1;
    }

    osg::ref_ptr<osg::StateSet> BrutalBlood::createState(unsigned style, float fade, float dry, bool weapon)
    {
        // Same look as the wounds on bodies: Dark Messiah's DecalModulate,
        // multiplying the already lit surface around a neutral grey.
        osg::ref_ptr<osg::Texture2D> texture = new osg::Texture2D(
            mResourceSystem->getImageManager()->getImage(VFS::Path::Normalized(texturePath(style))));
        texture->setWrap(osg::Texture::WRAP_S, osg::Texture::CLAMP_TO_EDGE);
        texture->setWrap(osg::Texture::WRAP_T, osg::Texture::CLAMP_TO_EDGE);
        mResourceSystem->getSceneManager()->applyFilterSettings(texture);
        osg::ref_ptr<osg::Geometry> geometry
            = osg::createTexturedQuadGeometry(osg::Vec3f(), osg::Vec3f(1, 0, 0), osg::Vec3f(0, 0, 1));
        auto* state = geometry->getOrCreateStateSet();
        const auto flags = osg::StateAttribute::ON | osg::StateAttribute::OVERRIDE | osg::StateAttribute::PROTECTED;
        state->setTextureAttributeAndModes(0, texture, flags);
        state->setTextureAttribute(0, new SceneUtil::TextureType("diffuseMap"), flags);
        state->setTextureAttribute(0, new osg::TexMat, flags);
        state->setAttributeAndModes(new osg::BlendFunc(GL_DST_COLOR, GL_SRC_COLOR, GL_ZERO, GL_ONE), flags);
        state->setAttributeAndModes(new SceneUtil::AutoDepth(osg::Depth::LEQUAL, 0, 1, false), flags);
        const float offset = SceneUtil::AutoDepth::isReversed() ? 1.f : -1.f;
        state->setAttributeAndModes(new osg::PolygonOffset(offset * 2.f, offset * 2.f), flags);
        state->setMode(
            GL_CULL_FACE, osg::StateAttribute::OFF | osg::StateAttribute::OVERRIDE | osg::StateAttribute::PROTECTED);
        state->setAttribute(new osg::ColorMask(true, true, true, false), flags);
        // Protected: inside the first-person depth-clearing bin this becomes a
        // nested bin drawn right after the held weapon.
        state->setRenderBinDetails(1, "RenderBin", osg::StateSet::PROTECTED_RENDERBIN_DETAILS);
        state->addUniform(new osg::Uniform("bloodFade", fade), flags);
        state->addUniform(new osg::Uniform("bloodDry", dry), flags);
        state->addUniform(new osg::Uniform("bloodFeather", weapon), flags);
        mResourceSystem->getSceneManager()->setUpNormalsRTForStateSet(state, false);
        mResourceSystem->getSceneManager()->recreateShaders(geometry, "blooddecal");
        osg::ref_ptr<osg::StateSet> result = geometry->getStateSet();
        if (auto* program = result->getAttribute(osg::StateAttribute::PROGRAM))
            result->setAttributeAndModes(program, flags);
        if (auto* compile = mResourceSystem->getSceneManager()->getIncrementalCompileOperation())
            compile->add(geometry);
        return result;
    }

    osg::StateSet* BrutalBlood::weaponState(unsigned style, float fade, float dry)
    {
        return state(style, fade, dry, true);
    }

    osg::StateSet* BrutalBlood::state(unsigned style, float fade, float dry, bool weapon)
    {
        if (!hasStyle(style))
            return nullptr;
        const int f = std::clamp(int(std::lround(fade * FadeLevels)), 0, FadeLevels);
        const int d = std::clamp(int(std::lround(dry * DryLevels)), 0, DryLevels);
        const std::uint64_t key
            = (std::uint64_t(weapon) << 24) | (std::uint64_t(style) << 16) | (std::uint64_t(f) << 8) | std::uint64_t(d);
        auto& entry = mStates[key];
        if (!entry)
            entry = createState(style, float(f) / FadeLevels, float(d) / DryLevels, weapon);
        return entry;
    }

    double BrutalBlood::now() const
    {
        const auto stamp = MWBase::Environment::get().getWorld()->getTimeStamp();
        return double(stamp.getDay()) * 24. + double(stamp.getHour());
    }

    float BrutalBlood::markFade(double age)
    {
        // Fully visible for most of a day, then wears away.
        return smooth(float((age - 18.) / (WeaponLifetimeHours - 18.)));
    }

    float BrutalBlood::markDry(double age)
    {
        // Wet for a few minutes, dark and dry after a few hours.
        return smooth(float((age - .1) / 3.));
    }

    BrutalBlood::ItemBlood* BrutalBlood::item(ESM::RefNum key)
    {
        if (!key.isSet())
            return nullptr;
        auto found = mItems.find(key);
        return found == mItems.end() ? nullptr : &found->second;
    }

    BrutalBlood::ItemBlood& BrutalBlood::createItem(ESM::RefNum key)
    {
        return mItems[key];
    }

    bool BrutalBlood::isBloody(ESM::RefNum key) const
    {
        if (!key.isSet())
            return false;
        const auto found = mItems.find(key);
        return found != mItems.end() && !found->second.marks.empty();
    }

    void BrutalBlood::clearItem(ESM::RefNum key)
    {
        auto found = mItems.find(key);
        if (found == mItems.end())
            return;
        // Keep an empty entry with a new revision, so visible smears are removed.
        found->second.marks.clear();
        found->second.wash = 0;
        found->second.dripUntil = 0;
        found->second.revision = nextRevision();
    }

    void BrutalBlood::addWear(ESM::RefNum key, double hours)
    {
        auto* blood = item(key);
        if (!blood || blood->marks.empty() || hours <= 0)
            return;
        blood->wash += hours;
    }

    std::uint32_t BrutalBlood::random()
    {
        mRandom ^= mRandom << 13;
        mRandom ^= mRandom >> 17;
        mRandom ^= mRandom << 5;
        return mRandom;
    }

    bool BrutalBlood::throwSplat(const osg::Vec3f& origin, const osg::Vec3f& direction, float range, float size,
        const std::string& worldspace, bool exterior)
    {
        if (!Settings::game().mBrutalEnabled || !Settings::game().mBrutalWorldBlood || !hasStyle(Style_World))
            return false;
        osg::Vec3f dir = direction;
        if (dir.normalize() < 1e-3f || !mAccess.castStatic)
            return false;
        ++mThrown;
        const auto hit = mAccess.castStatic(origin, origin + dir * range);
        if (!hit)
            return false;
        Splat splat;
        splat.worldspace = worldspace;
        splat.exterior = exterior;
        splat.position = hit->first;
        splat.normal = hit->second;
        if (splat.normal.normalize() < 1e-3f)
            return false;
        // Elongate along the travel of the blood, as a thrown drop smears.
        osg::Vec3f tangent = dir - splat.normal * (dir * splat.normal);
        if (tangent.normalize() < 1e-3f)
        {
            const float angle = random01() * 6.2831853f;
            tangent = osg::Vec3f(std::cos(angle), std::sin(angle), 0.f);
            tangent -= splat.normal * (tangent * splat.normal);
            if (tangent.normalize() < 1e-3f)
                tangent = osg::Vec3f(1, 0, 0);
        }
        splat.tangent = tangent;
        splat.size = size;
        splat.hour = now();
        unsigned style = Style_World + random() % 6;
        if (!hasStyle(style))
            style = Style_World;
        splat.style = style;
        splat.id = mNextId++;
        mSplats.push_back(splat);
        ++mLanded;
        while (mSplats.size() > MaxSplats)
        {
            removeBuilt(mSplats.front().id);
            mSplats.pop_front();
        }
        buildSplat(mSplats.back(), mSplats.size() - 1);
        return true;
    }

    void BrutalBlood::dropUnder(const osg::Vec3f& point, float size, const std::string& worldspace, bool exterior)
    {
        throwSplat(point, osg::Vec3f(0, 0, -1), 400.f, size, worldspace, exterior);
    }

    void BrutalBlood::buildSplat(const Splat& splat, std::size_t)
    {
        if (!mAccess.collectStatic)
            return;
        const auto start = std::chrono::steady_clock::now();
        struct Timer
        {
            std::chrono::steady_clock::time_point start;
            double& slowest;
            ~Timer()
            {
                slowest = std::max(slowest,
                    std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - start).count());
            }
        } timer{ start, mSlowestBuildMs };
        std::vector<osg::Vec3d> positions;
        std::vector<std::array<unsigned, 3>> triangles;
        mAccess.collectStatic(splat.position, splat.size * .75f + 8.f, positions, triangles);
        if (triangles.empty())
        {
            mRetryAt[splat.id] = mRealTime + 1.f;
            return;
        }
        std::vector<SceneUtil::DecalProjector::Triangle> projected(triangles.begin(), triangles.end());
        const auto result = SceneUtil::DecalProjector::project(positions, projected,
            osg::Vec3d(splat.position + splat.normal), osg::Vec3d(splat.normal), osg::Vec3d(splat.tangent),
            splat.size, splat.size, 6.0, 8.0);
        if (result.vertices.empty())
        {
            // The surface is there but did not take a decal: never retry.
            mRetryAt[splat.id] = std::numeric_limits<float>::max();
            return;
        }
        mRetryAt.erase(splat.id);
        osg::ref_ptr<osg::Vec3Array> vertices = new osg::Vec3Array;
        osg::ref_ptr<osg::Vec2Array> uvs = new osg::Vec2Array;
        vertices->reserve(result.vertices.size());
        uvs->reserve(result.vertices.size());
        const osg::Vec3d origin(splat.position);
        for (const auto& anchor : result.vertices)
        {
            osg::Vec3d p;
            for (unsigned k = 0; k < 3; ++k)
                p += positions[anchor.indices[k]] * anchor.weights[k];
            vertices->push_back(osg::Vec3f(p - origin));
            uvs->push_back(anchor.uv);
        }
        osg::ref_ptr<osg::Geometry> geometry = new osg::Geometry;
        geometry->setUseDisplayList(false);
        geometry->setUseVertexBufferObjects(true);
        geometry->setVertexArray(vertices);
        geometry->setTexCoordArray(0, uvs, osg::Array::BIND_PER_VERTEX);
        geometry->addPrimitiveSet(new osg::DrawArrays(GL_TRIANGLES, 0, vertices->size()));
        geometry->setCullingActive(true);
        // The material goes on before the mesh enters the scene: never a plain white quad.
        const double age = now() - splat.hour;
        const float fade = smooth(float(splat.wash / SplatWashHours));
        const float dry = markDry(age);
        osg::StateSet* look = state(splat.style, fade, dry);
        if (!look)
            return;
        osg::ref_ptr<osg::MatrixTransform> transform = new osg::MatrixTransform(osg::Matrix::translate(origin));
        transform->setNodeMask(Mask_Effect);
        transform->setStateSet(look);
        transform->addChild(geometry);
        removeBuilt(splat.id);
        mSplatRoot->addChild(transform);
        mBuilt[splat.id] = Built{ transform, int(std::lround(fade * FadeLevels)), int(std::lround(dry * DryLevels)) };
        ++mBuiltCount;
    }

    void BrutalBlood::removeBuilt(std::uint32_t id)
    {
        auto found = mBuilt.find(id);
        if (found != mBuilt.end())
        {
            mSplatRoot->removeChild(found->second.node);
            mBuilt.erase(found);
        }
        mRetryAt.erase(id);
    }

    void BrutalBlood::unloadSplats()
    {
        mSplatRoot->removeChildren(0, mSplatRoot->getNumChildren());
        mBuilt.clear();
        mRetryAt.clear();
    }

    void BrutalBlood::clear()
    {
        unloadSplats();
        mSplats.clear();
        mItems.clear();
        mLastHour = -1;
    }

    void BrutalBlood::update(
        float dt, const osg::Vec3f& player, const std::string& worldspace, bool exterior, bool raining)
    {
        mRealTime += dt;
        const double hour = now();
        const double elapsed = mLastHour < 0 ? 0. : std::clamp(hour - mLastHour, 0., 24. * 30.);
        mLastHour = hour;
        mElapsedHours = elapsed;

        // Weapon smears that are gone for good. Reload is cheap: drop them.
        for (auto it = mItems.begin(); it != mItems.end();)
        {
            auto& marks = it->second.marks;
            const auto before = marks.size();
            marks.erase(std::remove_if(marks.begin(), marks.end(),
                            [&](const WeaponMark& m) {
                                return hour - m.hour + it->second.wash >= WeaponLifetimeHours;
                            }),
                marks.end());
            if (marks.size() != before)
                it->second.revision = nextRevision();
            // Rendered smears notice the missing entry and clear themselves.
            if (marks.empty() && it->second.dripUntil < mRealTime)
                it = mItems.erase(it);
            else
                ++it;
        }

        const bool enabled = Settings::game().mBrutalEnabled && Settings::game().mBrutalWorldBlood;
        if (raining && exterior && elapsed > 0)
            for (auto& splat : mSplats)
                if (splat.exterior && splat.worldspace == worldspace)
                    splat.wash += elapsed;
        for (auto it = mSplats.begin(); it != mSplats.end();)
        {
            if (it->wash >= SplatWashHours)
            {
                removeBuilt(it->id);
                it = mSplats.erase(it);
            }
            else
                ++it;
        }

        if (!enabled)
        {
            if (!mBuilt.empty())
                unloadSplats();
            return;
        }
        if (mRealTime < mNextScan)
            return;
        mNextScan = mRealTime + .5f;
        unsigned builds = 0;
        for (std::size_t i = 0; i < mSplats.size(); ++i)
        {
            const auto& splat = mSplats[i];
            const float distance = (splat.position - player).length();
            auto built = mBuilt.find(splat.id);
            if (splat.worldspace != worldspace || distance > UnloadRange)
            {
                if (built != mBuilt.end())
                    removeBuilt(splat.id);
                continue;
            }
            if (built == mBuilt.end())
            {
                if (distance > BuildRange || builds >= 8)
                    continue;
                auto retry = mRetryAt.find(splat.id);
                if (retry != mRetryAt.end() && retry->second > mRealTime)
                    continue;
                buildSplat(splat, i);
                ++builds;
                built = mBuilt.find(splat.id);
                if (built == mBuilt.end())
                    continue;
            }
            const double age = hour - splat.hour;
            const float fade = smooth(float(splat.wash / SplatWashHours));
            const float dry = markDry(age);
            const int fadeLevel = int(std::lround(fade * FadeLevels));
            const int dryLevel = int(std::lround(dry * DryLevels));
            if (fadeLevel != built->second.fadeLevel || dryLevel != built->second.dryLevel)
            {
                if (auto* stateSet = state(splat.style, fade, dry))
                    built->second.node->setStateSet(stateSet);
                built->second.fadeLevel = fadeLevel;
                built->second.dryLevel = dryLevel;
            }
        }
    }

    std::string BrutalBlood::save() const
    {
        Writer out;
        std::uint32_t items = 0;
        for (const auto& [key, blood] : mItems)
            items += !blood.marks.empty();
        out.u32(items);
        for (const auto& [key, blood] : mItems)
        {
            if (blood.marks.empty())
                continue;
            out.u32(key.mIndex);
            out.u32(std::uint32_t(key.mContentFile));
            out.f64(blood.wash);
            out.u32(blood.marks.size());
            for (const auto& m : blood.marks)
            {
                out.u32(m.style);
                out.string(m.model);
                out.u32(m.ordinal);
                out.u32(std::uint32_t(m.fingerprint));
                out.u32(std::uint32_t(m.fingerprint >> 32));
                out.f64(m.hour);
                out.u32(m.anchors.size());
                for (const auto& a : m.anchors)
                {
                    for (auto i : a.indices)
                        out.u32(i);
                    for (unsigned k = 0; k < 3; ++k)
                        out.f32(a.weights[k]);
                    out.f32(a.uv.x());
                    out.f32(a.uv.y());
                }
            }
        }
        out.u32(mSplats.size());
        for (const auto& s : mSplats)
        {
            out.string(s.worldspace);
            out.u32(s.exterior);
            out.vec(s.position);
            out.vec(s.normal);
            out.vec(s.tangent);
            out.f32(s.size);
            out.u32(s.style);
            out.f64(s.hour);
            out.f64(s.wash);
        }
        return out.data;
    }

    bool BrutalBlood::load(std::string_view data)
    {
        clear();
        if (data.empty())
            return true;
        if (data.size() > MaxSaveBytes || data.substr(0, 4) != "BWB1")
            return false;
        try
        {
            Reader in{ data };
            std::map<ESM::RefNum, ItemBlood> items;
            const auto itemCount = in.u32();
            if (itemCount > 4096)
                return false;
            for (unsigned i = 0; i < itemCount; ++i)
            {
                ESM::RefNum key;
                key.mIndex = in.u32();
                key.mContentFile = std::int32_t(in.u32());
                ItemBlood blood;
                blood.wash = in.f64();
                const auto marks = in.u32();
                if (marks > MaxWeaponMarks)
                    return false;
                for (unsigned j = 0; j < marks; ++j)
                {
                    WeaponMark m;
                    m.style = in.u32();
                    m.model = in.string();
                    m.ordinal = in.u32();
                    m.fingerprint = in.u32();
                    m.fingerprint |= std::uint64_t(in.u32()) << 32;
                    m.hour = in.f64();
                    const auto size = in.u32();
                    if (m.style >= Style_Count || m.ordinal > 4096 || !size || size % 3 || size > 3072)
                        return false;
                    m.anchors.reserve(size);
                    for (unsigned k = 0; k < size; ++k)
                    {
                        SceneUtil::SurfaceDecals::Anchor a;
                        for (auto& index : a.indices)
                        {
                            index = in.u32();
                            if (index > 65535)
                                return false;
                        }
                        for (unsigned w = 0; w < 3; ++w)
                            a.weights[w] = in.f32();
                        const float u = in.f32(), v = in.f32();
                        a.uv = { u, v };
                        m.anchors.push_back(a);
                    }
                    blood.marks.push_back(std::move(m));
                }
                if (key.isSet())
                    items[key] = std::move(blood);
            }
            std::deque<Splat> splats;
            const auto splatCount = in.u32();
            if (splatCount > MaxSplats)
                return false;
            for (unsigned i = 0; i < splatCount; ++i)
            {
                Splat s;
                s.worldspace = in.string();
                s.exterior = in.u32() != 0;
                s.position = in.vec();
                s.normal = in.vec();
                s.tangent = in.vec();
                s.size = in.f32();
                s.style = in.u32();
                s.hour = in.f64();
                s.wash = in.f64();
                if (s.style < Style_World || s.style >= Style_Count || !(s.size > 0.f && s.size < 1000.f)
                    || s.normal.length2() < .5f || s.tangent.length2() < .5f)
                    return false;
                s.id = mNextId++;
                splats.push_back(std::move(s));
            }
            if (in.p != data.size())
                return false;
            mItems = std::move(items);
            for (auto& [key, blood] : mItems)
                blood.revision = nextRevision();
            mSplats = std::move(splats);
            mNextScan = 0;
            return true;
        }
        catch (const std::exception& error)
        {
            Log(Debug::Warning) << "Brutal blood data rejected: " << error.what();
            clear();
            return false;
        }
    }

    std::map<std::string, double> BrutalBlood::stats() const
    {
        std::size_t items = 0, marks = 0;
        for (const auto& [key, blood] : mItems)
            if (!blood.marks.empty())
            {
                ++items;
                marks += blood.marks.size();
            }
        return { { "weaponItems", double(items) }, { "weaponMarks", double(marks) },
            { "weaponAdds", double(mWeaponAdds) }, { "splats", double(mSplats.size()) },
            { "splatsBuilt", double(mBuilt.size()) }, { "splatsThrown", double(mThrown) },
            { "splatsLanded", double(mLanded) }, { "splatBuilds", double(mBuiltCount) },
            { "splatSlowestMs", mSlowestBuildMs },
            { "worldAvailable", sInstance && const_cast<BrutalBlood*>(this)->hasStyle(Style_World) ? 1. : 0. },
            { "weaponAvailable", const_cast<BrutalBlood*>(this)->hasStyle(Style_Slash) ? 1. : 0. } };
    }
}
