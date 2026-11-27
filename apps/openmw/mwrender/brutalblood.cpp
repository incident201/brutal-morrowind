#include "brutalblood.hpp"

#include "../mwbase/environment.hpp"
#include "../mwbase/world.hpp"
#include "../mwmechanics/creaturestats.hpp"
#include "../mwworld/cellstore.hpp"
#include "../mwworld/class.hpp"
#include "../mwworld/inventorystore.hpp"
#include "../mwworld/timestamp.hpp"
#include "../mwworld/worldmodel.hpp"
#include "npcanimation.hpp"
#include "vismask.hpp"

#include <components/debug/debuglog.hpp>
#include <components/esm3/loadcrea.hpp>
#include <components/esm3/loadnpc.hpp>
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
#include <bitset>
#include <tuple>
#include <chrono>
#include <cmath>
#include <cstring>
#include <limits>
#include <stdexcept>
#include <utility>

namespace MWRender
{
    namespace
    {
        BrutalBlood* sInstance = nullptr;
        constexpr int FadeLevels = 12, DryLevels = 6;
        constexpr std::size_t MaxBloodString = 1024;
        constexpr std::size_t MaxSavedItems = 4096;
        // Three indices, three weights, and two UV coordinates per serialized anchor.
        constexpr std::size_t AnchorBytes = 8 * sizeof(std::uint32_t);
        // Splatters are rebuilt only near the player; farther ones stay as records.
        constexpr float BuildRange = 6000.f, UnloadRange = 7500.f;
        constexpr std::size_t MaxPools = 96, MaxPrints = 384, MaxShoeOwners = 128, MaxQueuedSteps = 96;
        constexpr float PoolGrowthSeconds = 7.f, SoleDrySeconds = 13.f, PoolSourceSeconds = 12.f;
        constexpr float CutPoolSourceSeconds = 18.f;
        constexpr double PoolWetHours = .45, SoleWetHours = .2;
        using Mark = BrutalBlood::Mark;

        enum SoleStyle : unsigned { BareFoot, Shoe, ArmouredBoot };

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
            std::string data;
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
                if (size > MaxBloodString || size > data.size() - p)
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

        double washHours(Mark mark)
        {
            return mark == Mark::Footprint ? .45 : SplatWashHours;
        }

        Mark budgetGroup(Mark mark)
        {
            return mark == Mark::Imported ? Mark::Spray : mark;
        }

        std::size_t markLimit(Mark group)
        {
            switch (group)
            {
                case Mark::Pool: return MaxPools;
                case Mark::Footprint: return MaxPrints;
                default: return BrutalBlood::MaxSplats;
            }
        }

        bool worldBloodEnabled()
        {
            return Settings::game().mBrutalEnabled && Settings::game().mBrutalWorldBlood;
        }

        float maskRandom(std::uint32_t& seed)
        {
            seed = seed * 1664525u + 1013904223u;
            return float(seed >> 8) / 16777216.f;
        }

        unsigned maskVariant(std::uint32_t seed)
        {
            return (seed >> 1) % 16;
        }

        std::uint32_t maskSeed(std::uint32_t seed, Mark mark)
        {
            // Only footprints use the low bit to distinguish left and right.
            return ((maskVariant(seed) + 1) * 0x9e3779b9u & ~1u)
                | (mark == Mark::Footprint ? seed & 1u : 0u);
        }

        unsigned footprintAmount(float opacity)
        {
            return std::clamp(int(std::lround(opacity * 4.f)), 1, 4);
        }

        std::uint64_t maskKey(const BrutalBlood::Splat& splat)
        {
            const bool footprint = splat.mark == Mark::Footprint;
            return (std::uint64_t(splat.mark) << 32)
                | (std::uint64_t(footprint ? splat.style : 0u) << 16)
                | (footprint ? (footprintAmount(splat.opacity) - 1u) << 9 : 0u)
                | (unsigned(splat.mark == Mark::Pool && splat.porous) << 8)
                | (maskVariant(splat.seed) << 1) | (footprint ? splat.seed & 1u : 0u);
        }

        float textureNoise(float x, float y, std::uint32_t seed)
        {
            const int col = int(std::floor(x)), row = int(std::floor(y));
            const float u = smooth(x - float(col)), v = smooth(y - float(row));
            const auto value = [seed](int a, int b) {
                auto key = seed ^ (std::uint32_t(a) * 73856093u) ^ (std::uint32_t(b) * 19349663u);
                return maskRandom(key);
            };
            return std::lerp(std::lerp(value(col, row), value(col + 1, row), u),
                std::lerp(value(col, row + 1), value(col + 1, row + 1), u), v);
        }

        struct Ellipse
        {
            float x, y, rx, ry;

            float distance(float sampleX, float sampleY) const
            {
                const float dx = (sampleX - x) / rx, dy = (sampleY - y) / ry;
                return dx * dx + dy * dy;
            }
        };

        struct PoolMask
        {
            std::array<Ellipse, 4> lobes;
            float phase;
            float edge;

            float coverage(float x, float y) const
            {
                // Low frequencies push out uneven tongues of fluid. Finer ripples break their edges.
                const float sx = x + .080f * std::sin(8.f * y + phase)
                    + .040f * std::sin(19.f * y + 12.f * x + phase)
                    + .017f * std::sin(42.f * y - 28.f * x - phase);
                const float sy = y + .069f * std::sin(10.f * x - phase)
                    + .036f * std::sin(22.f * x - 9.f * y)
                    + .018f * std::sin(39.f * x + 35.f * y + phase);
                float distance = 8.f;
                for (const auto& lobe : lobes)
                {
                    const float d = lobe.distance(sx, sy);
                    const float blend = std::max(.32f - std::abs(distance - d), 0.f) / .32f;
                    distance = std::min(distance, d) - .08f * blend * blend;
                }
                const float border = 1.025f + .049f * std::sin(32.f * x + 31.f * y + phase);
                const float transition = edge * (.85f + .19f * std::sin(9.f * x + 10.f * y - phase));
                return smooth((border - distance) / transition);
            }
        };

        PoolMask poolMask(std::uint32_t seed, bool porous)
        {
            const float side = maskRandom(seed) < .5f ? -1.f : 1.f;
            const float phase = maskRandom(seed) * 6.2831853f;
            const float leftY = (.09f + .19f * maskRandom(seed)) * side;
            const float rightY = (.10f + .25f * maskRandom(seed)) * -side;
            return { { Ellipse{ -.04f, 0.f, .52f, .55f + .10f * maskRandom(seed) },
                         Ellipse{ -.38f, leftY, .39f, .36f + .12f * maskRandom(seed) },
                         Ellipse{ .38f, rightY, .35f + .05f * maskRandom(seed), .33f + .13f * maskRandom(seed) },
                         Ellipse{ -.30f + .57f * maskRandom(seed), side * (.46f + .10f * maskRandom(seed)),
                             .12f + .12f * maskRandom(seed), .18f + .09f * maskRandom(seed) } },
                phase, porous ? .27f : .15f };
        }

        std::vector<Ellipse> sprayMask(std::uint32_t seed)
        {
            std::vector<Ellipse> drops;
            const unsigned count = 5 + seed % 7;
            drops.reserve(count + 2);
            drops.push_back({ -.20f + .26f * maskRandom(seed), -.23f + .46f * maskRandom(seed),
                .21f + .21f * maskRandom(seed), .18f + .22f * maskRandom(seed) });
            if (maskRandom(seed) > .42f)
                drops.push_back({ .18f + .18f * maskRandom(seed), -.25f + .50f * maskRandom(seed),
                    .13f + .15f * maskRandom(seed), .11f + .16f * maskRandom(seed) });
            for (unsigned i = 0; i < count; ++i)
            {
                const float angle = maskRandom(seed) * 6.2831853f;
                const float radius = .31f + .53f * maskRandom(seed);
                const float size = .023f + .115f * std::pow(maskRandom(seed), 2.f);
                drops.push_back({ std::cos(angle) * radius, std::sin(angle) * radius * .93f, size,
                    size * (.58f + .71f * maskRandom(seed)) });
            }
            return drops;
        }

        float footprintMask(float x, float y, unsigned style, bool left, float amount, float pressure, float grit)
        {
            y *= left ? 1.f : -1.f;
            x += .076f * (pressure - .5f) + .032f * std::sin(12.f * y);
            y += .094f * (grit - .5f) + .027f * std::sin(15.f * x + y);
            const bool bare = style == BareFoot;
            const Ellipse heel{ -.49f, -.11f, .32f, bare ? .49f : .63f };
            const Ellipse ball{ .25f, .04f, bare ? .35f : .53f, bare ? .70f : .76f };
            const Ellipse arch{ -.14f, -.25f, .32f, .36f };
            float distance = std::min(heel.distance(x, y), ball.distance(x, y));
            if (bare)
            {
                distance = std::min(distance, arch.distance(x, y));
                for (int i = 0; i < 5; ++i)
                {
                    const Ellipse toe{ .74f - .052f * float(i), .51f - .24f * float(i),
                        .15f - .016f * float(i), .135f - .011f * float(i) };
                    distance = std::min(distance, toe.distance(x, y));
                }
            }
            float alpha = smooth((.92f + .14f * grit - distance) * 4.f);
            if (style == ArmouredBoot && std::abs(y) > .15f + .13f * (grit - .5f))
            {
                const float groove = (x + .15f * std::abs(y) + .05f * (grit - .5f)) * 19.f + .45f * y;
                alpha *= .51f + .49f * smooth((std::sin(groove) + .09f) * 2.3f);
            }
            // The ball and heel carry more pressure. Drier soles transfer only in isolated patches.
            const float contact = .69f * pressure + .31f * grit + .15f * std::abs(x) + .04f * y;
            const float minimum = .65f - .24f * amount;
            alpha *= .10f + .90f * smooth((contact - minimum) * 3.85f);
            return alpha;
        }

        osg::ref_ptr<osg::Image> makeMask(const BrutalBlood::Splat& splat)
        {
            const int size = splat.mark == Mark::Pool ? 128 : 64;
            auto image = osg::ref_ptr<osg::Image>(new osg::Image);
            image->allocateImage(size, size, 1, GL_RGBA, GL_UNSIGNED_BYTE);
            const auto drops = splat.mark == Mark::Spray ? sprayMask(splat.seed) : std::vector<Ellipse>{};
            const auto pool = poolMask(splat.seed, splat.porous);
            for (int j = 1; j + 1 < size; ++j)
                for (int i = 1; i + 1 < size; ++i)
                {
                    const float x = (float(i) + .5f) * 2.f / float(size) - 1.f;
                    const float y = (float(j) + .5f) * 2.f / float(size) - 1.f;
                    auto pixelSeed = splat.seed ^ (std::uint32_t(i) * 73856093u)
                        ^ (std::uint32_t(j) * 19349663u);
                    const float grain = maskRandom(pixelSeed);
                    const bool footprint = splat.mark == Mark::Footprint;
                    const float grit = textureNoise(x * 16.f, y * 18.f, splat.seed ^ 0x9c773u);
                    const float blotches = textureNoise(x * 4.3f, y * 4.8f, splat.seed);
                    float coverage = 0.f;
                    if (splat.mark == Mark::Pool)
                        coverage = pool.coverage(x, y);
                    else if (footprint)
                        coverage = footprintMask(x, y, splat.style, (splat.seed & 1u) != 0, splat.opacity, blotches, grit);
                    else
                    {
                        const float driftX = x + .027f * std::sin(21.f * y + pool.phase)
                            + .013f * std::sin(35.f * x - 38.f * y);
                        const float driftY = y + .029f * std::sin(19.f * x - pool.phase)
                            + .015f * std::sin(31.f * x + 27.f * y);
                        for (const auto& drop : drops)
                            coverage = std::max(coverage,
                                smooth((.93f + .20f * grit - drop.distance(driftX, driftY)) * 4.2f));
                    }
                    auto* pixel = image->data(i, j);
                    if (footprint)
                    {
                        const float pigment = .58f * blotches + .34f * grit + .08f * grain;
                        pixel[0] = static_cast<unsigned char>(108.f - 53.f * pigment);
                        pixel[1] = static_cast<unsigned char>(27.f - 18.f * pigment);
                        pixel[2] = static_cast<unsigned char>(23.f - 16.f * pigment);
                        // The quantized amount shapes dry sole patches; exact
                        // per-stamp opacity is applied once by the vertex color.
                        pixel[3] = static_cast<unsigned char>(255.f * coverage * (.88f + .12f * grain));
                    }
                    else
                    {
                        // Dense patches of blood and the fine, translucent rim share the same outline.
                        const float density = .71f * blotches + .23f * grit + .06f * grain;
                        const float film = smooth((coverage - .08f) * 1.27f);
                        const float body = film * (.49f + .51f * smooth((density - .13f) * 1.5f));
                        pixel[0] = static_cast<unsigned char>(105.f - 83.f * body);
                        pixel[1] = static_cast<unsigned char>(28.f - 24.f * body);
                        pixel[2] = static_cast<unsigned char>(21.f - 17.f * body);
                        pixel[3] = static_cast<unsigned char>(255.f * coverage * splat.opacity * (.94f + .06f * grit));
                    }
                }
            // Clamp-to-edge samples a transparent border as the pool grows inside its projection.
            for (int i = 0; i < size; ++i)
                for (auto* pixel : { image->data(i, 0), image->data(i, size - 1), image->data(0, i),
                         image->data(size - 1, i) })
                    std::fill_n(pixel, 4, 0);
            return image;
        }

        osg::ref_ptr<osg::Texture2D> decalTexture(Resource::ResourceSystem& resources, osg::Image* image)
        {
            auto texture = osg::ref_ptr<osg::Texture2D>(new osg::Texture2D(image));
            texture->setWrap(osg::Texture::WRAP_S, osg::Texture::CLAMP_TO_EDGE);
            texture->setWrap(osg::Texture::WRAP_T, osg::Texture::CLAMP_TO_EDGE);
            resources.getSceneManager()->applyFilterSettings(texture);
            return texture;
        }

        bool exteriorCell(const MWWorld::Cell& cell)
        {
            return cell.isExterior() || cell.isQuasiExterior();
        }

        bool redBlood(const MWWorld::Ptr& actor)
        {
            if (actor.getType() == ESM::NPC::sRecordId)
                return actor.get<ESM::NPC>()->mBase->mBloodType == 0;
            if (actor.getType() == ESM::Creature::sRecordId)
                return actor.get<ESM::Creature>()->mBase->mBloodType == 0;
            return false;
        }

        osg::Vec3f planarDirection(osg::Vec3f direction, const osg::Vec3f& normal)
        {
            direction -= normal * (direction * normal);
            if (direction.normalize() < 1e-3f)
            {
                direction = osg::Z_AXIS ^ normal;
                if (direction.normalize() < 1e-3f)
                    direction = osg::X_AXIS;
            }
            return direction;
        }
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

    osg::ref_ptr<osg::StateSet> BrutalBlood::createState(
        osg::Image* image, float fade, float dry, DecalSurface surface)
    {
        // Same look as the wounds on bodies: Dark Messiah's DecalModulate,
        // multiplying the already lit surface around a neutral grey.
        const auto texture = decalTexture(*mResourceSystem, image);
        osg::ref_ptr<osg::Geometry> geometry
            = osg::createTexturedQuadGeometry(osg::Vec3f(), osg::Vec3f(1, 0, 0), osg::Vec3f(0, 0, 1));
        auto* state = geometry->getOrCreateStateSet();
        const auto flags = osg::StateAttribute::ON | osg::StateAttribute::OVERRIDE | osg::StateAttribute::PROTECTED;
        state->setTextureAttributeAndModes(0, texture, flags);
        state->setTextureAttribute(0, new SceneUtil::TextureType("diffuseMap"), flags);
        state->setTextureAttribute(0, new osg::TexMat, flags);
        state->setAttributeAndModes(new osg::BlendFunc(GL_DST_COLOR, GL_SRC_COLOR, GL_ZERO, GL_ONE), flags);
        state->setAttributeAndModes(new SceneUtil::AutoDepth(osg::Depth::LEQUAL, 0, 1, false), flags);
        const bool wound = surface == DecalSurface::Wound;
        const float offset = (SceneUtil::AutoDepth::isReversed() ? 1.f : -1.f) * (wound ? 1.f : 2.f);
        state->setAttributeAndModes(new osg::PolygonOffset(offset, offset), flags);
        state->setMode(
            GL_CULL_FACE, osg::StateAttribute::OFF | osg::StateAttribute::OVERRIDE | osg::StateAttribute::PROTECTED);
        state->setAttribute(new osg::ColorMask(true, true, true, false), flags);
        // Protected: inside the first-person depth-clearing bin this becomes a
        // nested bin drawn right after the held weapon.
        state->setRenderBinDetails(1, "RenderBin",
            wound ? osg::StateSet::OVERRIDE_RENDERBIN_DETAILS : osg::StateSet::PROTECTED_RENDERBIN_DETAILS);
        state->addUniform(new osg::Uniform("bloodFade", fade), flags);
        state->addUniform(new osg::Uniform("bloodDry", dry), flags);
        state->addUniform(new osg::Uniform("bloodFeather", surface == DecalSurface::Weapon), flags);
        state->addUniform(new osg::Uniform("bloodScale", 1.f), flags);
        state->addUniform(new osg::Uniform("bloodVertexData", false), flags);
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

    const std::array<osg::ref_ptr<osg::StateSet>, 4>& BrutalBlood::woundStates()
    {
        if (!mWoundStates)
        {
            constexpr std::array<std::string_view, 4> styles{ "slash", "small", "big", "tiny" };
            auto& states = mWoundStates.emplace();
            for (std::size_t i = 0; i < styles.size(); ++i)
            {
                const VFS::Path::Normalized path(std::string("textures/brutal_dm/model_blood_") + std::string(styles[i]) + ".png");
                if (mResourceSystem->getVFS()->exists(path))
                    states[i] = createState(mResourceSystem->getImageManager()->getImage(path), 0.f, 0.f, DecalSurface::Wound);
            }
        }
        return *mWoundStates;
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
        {
            const auto image = mResourceSystem->getImageManager()->getImage(VFS::Path::Normalized(texturePath(style)));
            entry = createState(image, float(f) / FadeLevels, float(d) / DryLevels,
                weapon ? DecalSurface::Weapon : DecalSurface::World);
        }
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

    void BrutalBlood::addSplat(Splat splat)
    {
        const Mark group = budgetGroup(splat.mark);
        const auto sameGroup = [group](const Splat& other) { return budgetGroup(other.mark) == group; };
        if (std::cmp_greater_equal(std::ranges::count_if(mSplats, sameGroup), markLimit(group)))
        {
            const auto oldest = std::ranges::find_if(mSplats, sameGroup);
            removeBuilt(oldest->id);
            mSplats.erase(oldest);
        }
        splat.id = mNextId++;
        mSplats.push_back(std::move(splat));
        mNextScan = mRealTime;
    }

    void BrutalBlood::hit(const MWWorld::Ptr& victim, const MWWorld::Ptr& attacker, osg::Vec3f position,
        osg::Vec3f direction, Impact impact, bool power, bool lethal, float damage)
    {
        if (!worldBloodEnabled() || !mAccess.castStatic || !victim.isInCell() || !redBlood(victim))
            return;
        const auto world = MWBase::Environment::get().getWorld();
        const auto player = world->getPlayerPtr();
        const auto* cell = victim.getCell()->getCell();
        const auto actorPosition = victim.getRefData().getPosition().asVec3();
        if (!player.isInCell()
            || (player.getRefData().getPosition().asVec3() - actorPosition).length2() > BuildRange * BuildRange
            || cell->getWorldSpace() != player.getCell()->getCell()->getWorldSpace())
            return;
        if ((position - actorPosition).length2() > 400.f * 400.f)
            position = actorPosition + osg::Vec3f(0.f, 0.f, 50.f);
        if (world->isUnderwater(victim.getCell(), position))
            return;

        const auto& health = victim.getClass().getCreatureStats(victim).getHealth();
        const float severity = std::clamp(damage / std::max(1.f, health.getBase()) * 2.8f, 0.f, 1.f);
        const float scale = 1.f + .36f * severity + (power ? .34f : 0.f);
        Splat base;
        base.worldspace = cell->getWorldSpace().serializeText();
        base.exterior = exteriorCell(*cell);
        base.mark = Mark::Spray;
        base.hour = now();
        base.opacity = .86f + .09f * severity + (power ? .04f : 0.f);

        osg::Vec3f away = attacker.isInCell() ? actorPosition - attacker.getRefData().getPosition().asVec3()
                                             : osg::Vec3f(0.f, 1.f, 0.f);
        away = planarDirection(away, osg::Z_AXIS);
        osg::Vec3f travel = away;
        if (direction.normalize() > 1e-3f)
            travel += direction * (impact == Impact::Slash ? .82f : .35f);
        travel = planarDirection(travel, osg::Z_AXIS);
        const auto right = travel ^ osg::Z_AXIS;
        const float fan = impact == Impact::Slash ? .87f : impact == Impact::Puncture ? .19f : .49f;
        const float footSize = (impact == Impact::Puncture ? 17.f : 24.f) + random01() * 12.f;
        spray(base, position, travel * .15f - osg::Z_AXIS, 320.f, footSize * scale);

        const int rays = power ? 3 : impact == Impact::Puncture ? 1 : 2;
        for (int i = 0; i < rays; ++i)
        {
            const auto path = travel + right * ((2.f * random01() - 1.f) * fan)
                - osg::Z_AXIS * (.04f + (i == 0 ? .23f : .75f) * random01());
            const float range = impact == Impact::Puncture ? 170.f : 245.f;
            const float size = (i == 0 ? 23.f + 18.f * random01() : 13.f + 15.f * random01()) * scale;
            spray(base, position, path, range * scale, size);
        }
        if (lethal)
            startPool(victim, position, 83.f + 31.f * severity + (power ? 27.f : 0.f) + 16.f * random01());
    }

    void BrutalBlood::drip(const MWWorld::Ptr& actor, const osg::Vec3f& tip)
    {
        if (!worldBloodEnabled() || !mAccess.castStatic || !actor.isInCell() || random01() >= .3f
            || MWBase::Environment::get().getWorld()->isUnderwater(actor.getCell(), tip))
            return;
        const auto* cell = actor.getCell()->getCell();
        Splat base;
        base.mark = Mark::Spray;
        base.worldspace = cell->getWorldSpace().serializeText();
        base.exterior = exteriorCell(*cell);
        base.hour = now();
        base.opacity = .89f;
        spray(base, tip, -osg::Z_AXIS, 220.f, 6.f + 6.f * random01());
    }

    void BrutalBlood::spray(
        const Splat& base, const osg::Vec3f& origin, osg::Vec3f direction, float range, float size)
    {
        if (direction.normalize() < 1e-3f)
            return;
        ++mThrown;
        const auto surface = mAccess.castStatic(origin, origin + direction * range);
        if (!surface)
            return;
        Splat splat = base;
        splat.position = surface->position;
        splat.normal = surface->normal;
        splat.porous = surface->porous;
        splat.tangent = planarDirection(direction, splat.normal);
        const float incidence = std::clamp(std::abs(direction * splat.normal), 0.f, 1.f);
        const float stretch = 1.f + 1.9f * (1.f - incidence) * (1.f - incidence);
        splat.width = size * stretch;
        splat.height = size * (.85f + .28f * random01());
        splat.seed = random();
        ++mLanded;
        addSplat(std::move(splat));
    }

    void BrutalBlood::startPool(const MWWorld::Ptr& actor, const osg::Vec3f& wound, float width)
    {
        const auto id = actor.getCellRef().getRefNum();
        if (!id.isSet() || mPoolSources.size() >= MaxPools)
            return;
        const auto* animation = MWBase::Environment::get().getWorld()->getAnimation(actor);
        PoolSource source;
        source.width = width;
        source.expires = mRealTime + PoolSourceSeconds;
        if (animation)
        {
            float nearest = std::numeric_limits<float>::max();
            for (const auto* bone : { "Bip01 Head", "Bip01 Spine1", "Bip01 Spine2", "Bip01 Pelvis",
                     "Bip01 L UpperArm", "Bip01 R UpperArm", "Bip01 L Thigh", "Bip01 R Thigh" })
            {
                const auto matrix = animation->getBoneWorldMatrix(bone);
                if (!matrix)
                    continue;
                const float distance = (wound - matrix->getTrans()).length2();
                if (distance >= nearest)
                    continue;
                nearest = distance;
                source.bone = bone;
                source.offset = wound * osg::Matrixf::inverse(*matrix);
            }
        }
        mPoolSources[{ id, BloodPoolOutlet::Impact }] = std::move(source);
    }

    void BrutalBlood::queueCutPool(const MWWorld::Ptr& actor, BloodPoolOutlet outlet)
    {
        if (!worldBloodEnabled() || !actor.isInCell() || !redBlood(actor) || mPoolSources.size() >= MaxPools)
            return;
        const auto id = actor.getCellRef().getRefNum();
        if (!id.isSet())
            return;
        PoolSource source;
        source.expires = mRealTime + CutPoolSourceSeconds;
        mPoolSources.try_emplace(std::make_pair(id, outlet), std::move(source));
    }

    void BrutalBlood::updatePools(const osg::Vec3f& player, const std::string& worldspace)
    {
        const auto& env = MWBase::Environment::get();
        const auto world = env.getWorld();
        const auto placePool = [&](const auto& key, PoolSource& source) {
            if (source.expires < mRealTime)
                return true;
            const auto [id, outlet] = key;
            const auto actor = env.getWorldModel()->getPtr(id);
            if (!actor.isInCell() || !actor.getRefData().getBaseNode())
                return false;
            const auto& stats = actor.getClass().getCreatureStats(actor);
            if (!stats.isDead())
                return true;
            const auto* cell = actor.getCell()->getCell();
            const auto center = actor.getRefData().getPosition().asVec3();
            if (cell->getWorldSpace().serializeText() != worldspace || (center - player).length2() > BuildRange * BuildRange)
                return false;
            const auto* animation = world->getAnimation(actor);
            const bool cut = outlet != BloodPoolOutlet::Impact;
            auto wound = center + osg::Vec3f(0.f, 0.f, 15.f);
            float width = source.width;
            if (cut)
            {
                const auto* npc = dynamic_cast<const NpcAnimation*>(animation);
                const auto point = npc ? npc->getCutPoolPoint(outlet) : std::nullopt;
                if (!point)
                {
                    source.stableSince = -1.f;
                    return false;
                }
                wound = point->position;
                width = std::clamp(48.f + 2.8f * point->radius, 54.f, 116.f);
                // Each exposed surface follows its own fragment. Opposing fragment velocities can cancel in the
                // whole ragdoll average, so wait for this centre to stay in place before projecting onto the ground.
                if (source.stableSince < 0.f || (wound - source.stablePosition).length2() > 3.f * 3.f)
                {
                    source.stableSince = mRealTime;
                    source.stablePosition = wound;
                }
                if (mRealTime - source.stableSince < .32f)
                    return false;
            }
            else
            {
                const auto dynamics = world->getNpcRagdollDynamics(actor);
                const bool atRest = dynamics.size() >= 2 ? dynamics[1].length2() < 27.f * 27.f
                                                        : stats.isDeathAnimationFinished();
                if (!atRest)
                {
                    source.stableSince = -1.f;
                    return false;
                }
                if (animation && !source.bone.empty())
                    if (const auto matrix = animation->getBoneWorldMatrix(source.bone))
                        wound = source.offset * *matrix;
            }
            const auto surface = mAccess.castStatic(wound + osg::Vec3f(0.f, 0.f, cut ? 4.f : 12.f),
                wound - osg::Vec3f(0.f, 0.f, cut ? 66.f : 44.f));
            if (!surface || surface->normal.z() < .68f)
            {
                source.stableSince = -1.f;
                return false;
            }
            if (world->isUnderwater(actor.getCell(), surface->position))
                return true;
            if (source.stableSince < 0.f)
                source.stableSince = mRealTime;
            if (mRealTime - source.stableSince < .20f)
                return false;

            Splat splat;
            splat.mark = Mark::Pool;
            splat.worldspace = worldspace;
            splat.exterior = exteriorCell(*cell);
            splat.porous = surface->porous;
            splat.position = surface->position;
            splat.normal = surface->normal;
            const float angle = random01() * 6.2831853f;
            const auto direction = splat.normal.z() < .965f ? -osg::Z_AXIS
                : osg::Vec3f(std::cos(angle), std::sin(angle), 0.f);
            splat.tangent = planarDirection(direction, splat.normal);
            splat.width = width;
            splat.height = width * (.69f + .21f * random01());
            splat.hour = now();
            splat.started = mRealTime;
            splat.seed = random();
            splat.opacity = .99f;
            addSplat(std::move(splat));
            return true;
        };
        for (auto it = mPoolSources.begin(); it != mPoolSources.end();)
            if (placePool(it->first, it->second))
                it = mPoolSources.erase(it);
            else
                ++it;
    }

    float BrutalBlood::poolScale(const Splat& splat, double hour) const
    {
        if (hour - splat.hour >= PoolWetHours)
            return 1.f;
        return .24f + .76f * smooth((mRealTime - splat.started) / PoolGrowthSeconds);
    }

    void BrutalBlood::queueStep(const MWWorld::Ptr& actor, bool left)
    {
        if (!worldBloodEnabled() || !actor.isInCell() || !actor.getClass().isNpc()
            || mSteps.size() >= MaxQueuedSteps)
            return;
        const auto id = actor.getCellRef().getRefNum();
        if (!id.isSet())
            return;
        if (!mShoes.contains(id))
        {
            const auto root = actor.getRefData().getPosition().asVec3();
            const auto worldspace = actor.getCell()->getCell()->getWorldSpace().serializeText();
            const bool nearbyPool = std::ranges::any_of(mSplats, [&](const Splat& splat) {
                return splat.mark == Mark::Pool && splat.worldspace == worldspace
                    && (splat.position - root).length2() < 160.f * 160.f;
            });
            if (!nearbyPool)
                return;
        }
        mSteps.push_back({ id, left });
    }

    void BrutalBlood::updateShoes(double hour)
    {
        const auto& env = MWBase::Environment::get();
        const auto world = env.getWorld();
        std::erase_if(mShoes, [&](const auto& entry) {
            const bool dry = std::ranges::all_of(entry.second, [&](const Sole& sole) {
                return sole.blood < .13f || sole.driesAt <= mRealTime || hour - sole.stainedHour > SoleWetHours;
            });
            if (dry)
                return true;
            const auto actor = env.getWorldModel()->getPtr(entry.first);
            return actor.isInCell() && actor.getRefData().getBaseNode()
                && (world->isSwimming(actor) || world->isUnderwater(
                    actor.getCell(), actor.getRefData().getPosition().asVec3() + osg::Vec3f(0.f, 0.f, 3.f)));
        });
    }

    float BrutalBlood::poolContact(const Surface& surface, const osg::Vec3f& forward, float size,
        const std::string& worldspace, double hour) const
    {
        float best = 0.f;
        for (const auto& pool : mSplats)
        {
            if (pool.mark != Mark::Pool || pool.worldspace != worldspace
                || hour - pool.hour >= PoolWetHours || pool.wash >= .22
                || pool.normal * surface.normal < .83f || !mBuilt.contains(pool.id))
                continue;
            const auto delta = surface.position - pool.position;
            if (std::abs(delta * pool.normal) > 3.5f || delta.length2() > pool.width * pool.width)
                continue;
            const float grown = poolScale(pool, hour);
            const auto across = pool.normal ^ pool.tangent;
            const auto mask = poolMask(maskSeed(pool.seed, pool.mark), pool.porous);
            float coverage = 0.f;
            for (float foot : { -.29f, 0.f, .29f })
            {
                const auto contact = delta + forward * (foot * size);
                const float x = 2.f * (contact * pool.tangent) / (pool.width * grown);
                const float y = 2.f * (contact * across) / (pool.height * grown);
                coverage += mask.coverage(x, y) / 3.f;
            }
            if (coverage <= .18f || coverage <= best)
                continue;
            // A puddle projected on a shared plane cannot transfer through a wall.
            const auto obstruction = mAccess.castStatic(pool.position + pool.normal * 4.f,
                surface.position + surface.normal * 4.f);
            if (!obstruction)
                best = coverage;
        }
        return best;
    }

    void BrutalBlood::plantFoot(const MWWorld::Ptr& actor, bool left, double hour)
    {
        const auto world = MWBase::Environment::get().getWorld();
        if (!actor.isInCell() || !actor.getRefData().getBaseNode() || !actor.getClass().isNpc()
            || actor.getClass().getCreatureStats(actor).isDead() || !world->isOnGround(actor)
            || world->isSwimming(actor) || world->isFlying(actor) || world->isWalkingOnWater(actor))
            return;
        const auto& position = actor.getRefData().getPosition();
        const auto root = position.asVec3();
        const auto* cell = actor.getCell()->getCell();
        if (world->isUnderwater(actor.getCell(), root + osg::Vec3f(0.f, 0.f, 3.f)))
            return;
        const float scale = std::clamp(world->getHalfExtents(actor, true).z() / 64.f, .54f, 1.55f);
        const float size = 27.f * scale;
        const float yaw = position.rot[2];
        osg::Vec3f forward(std::sin(yaw), std::cos(yaw), 0.f);
        osg::Vec3f foot = root + (forward ^ osg::Z_AXIS) * (scale * (left ? -9.f : 9.f));
        const auto* animation = world->getAnimation(actor);
        if (const auto* npc = dynamic_cast<const NpcAnimation*>(animation))
            if (const auto* body = npc->getBodyAwarenessAnimation())
                animation = body;
        if (animation)
        {
            const auto ankle = animation->getBoneWorldMatrix(left ? "Bip01 L Foot" : "Bip01 R Foot");
            if (ankle && (ankle->getTrans() - root).length2() < 62.f * 62.f * scale * scale)
            {
                foot = ankle->getTrans();
                if (const auto toe = animation->getBoneWorldMatrix(left ? "Bip01 L Toe0" : "Bip01 R Toe0"))
                {
                    osg::Vec3f axis = toe->getTrans() - foot;
                    axis.z() = 0.f;
                    if (axis.normalize() > 1.f)
                        forward = axis;
                }
            }
        }
        foot += forward * (4.f * scale);
        // The planted ankle in the render rig can sit above the physics support on uneven terrain.
        // Take horizontal placement from the rig and bracket the ground from the actor's base height.
        const osg::Vec3f upper(foot.x(), foot.y(), std::max(root.z(), foot.z()) + 9.f);
        const osg::Vec3f lower(foot.x(), foot.y(), root.z() - (14.f + 32.f * scale));
        const auto surface = mAccess.castStatic(upper, lower);
        if (!surface || surface->normal.z() < .68f || world->isUnderwater(actor.getCell(), surface->position))
            return;
        forward = planarDirection(forward, surface->normal);
        const auto worldspace = cell->getWorldSpace().serializeText();
        const float contact = poolContact(*surface, forward, size, worldspace, hour);
        const auto id = actor.getCellRef().getRefNum();
        auto found = mShoes.find(id);
        if (found == mShoes.end())
        {
            if (contact <= 0.f || mShoes.size() >= MaxShoeOwners)
                return;
            found = mShoes.try_emplace(id).first;
        }
        auto& sole = found->second[left ? 0 : 1];
        const auto displacement = surface->position - sole.lastPosition;
        if (mRealTime - sole.lastContact < .22f
            || (mRealTime - sole.lastContact < .8f && displacement.length2() < size * size * .08f))
            return;
        sole.lastPosition = surface->position;
        sole.lastContact = mRealTime;
        if (contact > 0.f)
        {
            sole.blood = std::max(sole.blood, .32f + .68f * contact);
            sole.driesAt = mRealTime + SoleDrySeconds;
            sole.stainedHour = hour;
            return;
        }
        const float remaining = std::clamp((sole.driesAt - mRealTime) / SoleDrySeconds, 0.f, 1.f);
        const float opacity = sole.blood * std::sqrt(remaining);
        if (opacity < .13f || hour - sole.stainedHour > SoleWetHours)
            return;

        const auto& inventory = actor.getClass().getInventoryStore(actor);
        const auto boot = inventory.getSlot(MWWorld::InventoryStore::Slot_Boots);
        Splat print;
        print.mark = Mark::Footprint;
        print.worldspace = worldspace;
        print.exterior = exteriorCell(*cell);
        print.porous = surface->porous;
        print.position = surface->position;
        print.normal = surface->normal;
        print.tangent = forward;
        print.width = size;
        print.height = size * .52f;
        print.opacity = opacity;
        print.hour = hour;
        print.style = boot == inventory.end() ? BareFoot
            : boot->getType() == ESM::Armor::sRecordId ? ArmouredBoot : Shoe;
        print.seed = (random() & ~1u) | std::uint32_t(left);
        addSplat(std::move(print));
        sole.blood *= .64f;
    }

    void BrutalBlood::buildSplat(const Splat& splat)
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
        // Wall collision can be several inches behind its visible trim or stonework.
        const bool wall = splat.mark == Mark::Spray && std::abs(splat.normal.z()) < .45f;
        const float snap = wall ? 30.f : 8.f;
        mAccess.collectStatic(splat.position, .5f * std::hypot(splat.width, splat.height) + snap, positions, triangles);
        if (triangles.empty())
        {
            mRetryAt[splat.id] = mRealTime + 1.f;
            return;
        }
        std::vector<SceneUtil::DecalProjector::Triangle> projected(triangles.begin(), triangles.end());
        const double depth = splat.mark == Mark::Footprint ? 2.5 : wall ? 8.0 : 6.0;
        const auto result = SceneUtil::DecalProjector::project(positions, projected,
            osg::Vec3d(splat.position + splat.normal), osg::Vec3d(splat.normal), osg::Vec3d(splat.tangent),
            splat.width, splat.height, depth, snap);
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
        const double hour = now();
        const double age = hour - splat.hour;
        const float fade = smooth(float(splat.wash / washHours(splat.mark)));
        const float dry = markDry(age);
        Built built;
        osg::ref_ptr<osg::StateSet> look;
        if (splat.mark == Mark::Imported)
            look = state(splat.style, fade, dry);
        else
        {
            // Sixteen deterministic masks per surface/style replace one texture
            // per stamp. Opacity and ageing are carried by the batched vertices.
            auto& cached = mProceduralLooks[maskKey(splat)];
            const auto flags = osg::StateAttribute::ON | osg::StateAttribute::OVERRIDE | osg::StateAttribute::PROTECTED;
            if (!cached)
            {
                Splat mask = splat;
                mask.seed = maskSeed(splat.seed, splat.mark);
                mask.opacity = splat.mark == Mark::Footprint ? footprintAmount(splat.opacity) / 4.f : 1.f;
                const auto image = makeMask(mask);
                if (!mProceduralState)
                    mProceduralState = createState(image, 0.f, 0.f, DecalSurface::World);
                cached = new osg::StateSet(*mProceduralState, osg::CopyOp::SHALLOW_COPY);
                cached->setTextureAttributeAndModes(0, decalTexture(*mResourceSystem, image), flags);
                cached->addUniform(new osg::Uniform("bloodVertexData", true), flags);
            }
            look = cached;
            built.procedural = true;
            built.fade = fade;
            built.dry = dry;
            built.opacity = splat.opacity;
            if (splat.mark == Mark::Pool && poolScale(splat, hour) < 1.f)
            {
                // Keep changing UV scale out of the immutable static batches.
                built.growth = new osg::Uniform("bloodScale", poolScale(splat, hour));
                osg::ref_ptr<osg::Vec4Array> color = new osg::Vec4Array;
                color->push_back(osg::Vec4f(fade, dry, splat.opacity, 1.f));
                geometry->setColorArray(color, osg::Array::BIND_OVERALL);
            }
        }
        if (!look)
            return;
        osg::ref_ptr<osg::MatrixTransform> transform = new osg::MatrixTransform(osg::Matrix::translate(origin));
        transform->setNodeMask(Mask_Effect);
        transform->setStateSet(look);
        transform->addChild(geometry);
        removeBuilt(splat.id);
        built.node = transform;
        built.look = look;
        if (built.growth)
        {
            osg::ref_ptr<osg::StateSet> growing = new osg::StateSet(*look, osg::CopyOp::SHALLOW_COPY);
            growing->addUniform(built.growth,
                osg::StateAttribute::ON | osg::StateAttribute::OVERRIDE | osg::StateAttribute::PROTECTED);
            transform->setStateSet(growing);
            mSplatRoot->addChild(transform);
        }
        else
            mSplatBatchesDirty = true;
        built.fadeLevel = int(std::lround(fade * FadeLevels));
        built.dryLevel = int(std::lround(dry * DryLevels));
        mBuilt.emplace(splat.id, std::move(built));
        ++mBuiltCount;
    }

    void BrutalBlood::rebuildSplatBatches()
    {
        if (!mSplatBatchesDirty) return;
        // Immutable replacements keep the draw thread's previous VBOs alive.
        // Spatial cells retain useful culling and precision in large exteriors.
        struct Batch
        {
            osg::ref_ptr<osg::Geometry> geometry = new osg::Geometry;
            osg::ref_ptr<osg::Vec3Array> positions = new osg::Vec3Array;
            osg::ref_ptr<osg::Vec2Array> uvs = new osg::Vec2Array;
            osg::ref_ptr<osg::Vec4Array> colors = new osg::Vec4Array;
            osg::Vec3f origin;
            osg::ref_ptr<osg::StateSet> state;
        };
        std::map<std::tuple<osg::StateSet*, int, int, int>, Batch> batches;
        for (const auto& [id, built] : mBuilt)
        {
            if (built.growth)
                continue;
            const auto* node = static_cast<const osg::MatrixTransform*>(built.node.get());
            const auto* source = static_cast<const osg::Geometry*>(node->getChild(0));
            const auto& positions = *static_cast<const osg::Vec3Array*>(source->getVertexArray());
            const auto& uvs = *static_cast<const osg::Vec2Array*>(source->getTexCoordArray(0));
            const auto origin = node->getMatrix().getTrans();
            const int x = int(std::floor(origin.x() / 1024)), y = int(std::floor(origin.y() / 1024)),
                z = int(std::floor(origin.z() / 1024));
            auto* state = built.node->getStateSet();
            auto& batch = batches[{state, x, y, z}];
            batch.origin.set(x * 1024.f, y * 1024.f, z * 1024.f);
            batch.state = state;
            for (std::size_t i = 0; i < positions.size(); ++i)
            {
                batch.positions->push_back(positions[i] + osg::Vec3f(origin) - batch.origin);
                batch.uvs->push_back(uvs[i]);
                batch.colors->push_back(osg::Vec4f(built.fade, built.dry, built.opacity, 1.f));
            }
        }
        osg::ref_ptr<osg::Group> root = new osg::Group;
        for (auto& [key, batch] : batches)
        {
            auto& geometry = batch.geometry;
            geometry->setUseDisplayList(false);
            geometry->setUseVertexBufferObjects(true);
            geometry->setVertexArray(batch.positions);
            geometry->setTexCoordArray(0, batch.uvs, osg::Array::BIND_PER_VERTEX);
            geometry->setColorArray(batch.colors, osg::Array::BIND_PER_VERTEX);
            geometry->setStateSet(batch.state);
            geometry->addPrimitiveSet(new osg::DrawArrays(GL_TRIANGLES, 0, batch.positions->size()));
            osg::ref_ptr<osg::MatrixTransform> node = new osg::MatrixTransform(osg::Matrix::translate(batch.origin));
            node->addChild(geometry);
            root->addChild(node);
        }
        root->setNodeMask(Mask_Effect);
        if (mSplatBatches) mSplatRoot->replaceChild(mSplatBatches, root);
        else mSplatRoot->addChild(root);
        mSplatBatches = root;
        mSplatBatchesDirty = false;
        ++mSplatBatchBuilds;
    }

    void BrutalBlood::removeBuilt(std::uint32_t id)
    {
        auto found = mBuilt.find(id);
        if (found != mBuilt.end())
        {
            if (found->second.growth)
                mSplatRoot->removeChild(found->second.node);
            else
                mSplatBatchesDirty = true;
            mBuilt.erase(found);
        }
        mRetryAt.erase(id);
    }

    void BrutalBlood::unloadSplats()
    {
        mSplatRoot->removeChildren(0, mSplatRoot->getNumChildren());
        mSplatBatches = nullptr;
        mSplatBatchesDirty = false;
        mBuilt.clear();
        mRetryAt.clear();
    }

    void BrutalBlood::clear()
    {
        unloadSplats();
        mSplats.clear();
        mItems.clear();
        mPoolSources.clear();
        mShoes.clear();
        mSteps.clear();
        mNextScan = mNextContactScan = mRealTime;
        mLastHour = -1;
    }

    void BrutalBlood::updatePoolGrowth(const Splat& splat, double hour)
    {
        if (splat.mark != Mark::Pool)
            return;
        const auto found = mBuilt.find(splat.id);
        if (found == mBuilt.end() || !found->second.growth)
            return;
        auto& built = found->second;
        const float scale = poolScale(splat, hour);
        built.growth->set(scale);
        if (scale >= 1.f)
        {
            mSplatRoot->removeChild(built.node);
            built.node->setStateSet(built.look);
            built.growth = nullptr;
            mSplatBatchesDirty = true;
        }
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

        const bool enabled = worldBloodEnabled();
        if (raining && exterior && elapsed > 0)
            for (auto& splat : mSplats)
                if (splat.exterior && splat.worldspace == worldspace)
                    splat.wash += elapsed;
        for (auto it = mSplats.begin(); it != mSplats.end();)
        {
            if (it->wash >= washHours(it->mark))
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
            mSteps.clear();
            return;
        }
        if (mRealTime >= mNextContactScan)
        {
            mNextContactScan = mRealTime + .12f;
            updatePools(player, worldspace);
            updateShoes(hour);
        }
        if (!mSteps.empty())
        {
            const auto model = MWBase::Environment::get().getWorldModel();
            for (const auto& step : mSteps)
                plantFoot(model->getPtr(step.actor), step.left, hour);
            mSteps.clear();
        }
        for (const auto& splat : mSplats)
            updatePoolGrowth(splat, hour);

        if (mRealTime < mNextScan)
        {
            rebuildSplatBatches();
            return;
        }
        mNextScan = mRealTime + .12f;
        unsigned builds = 0;
        const auto start = std::chrono::steady_clock::now();
        for (auto it = mSplats.rbegin(); it != mSplats.rend(); ++it)
        {
            const auto& splat = *it;
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
                if (distance > BuildRange || builds >= 4
                    || (builds > 0 && std::chrono::steady_clock::now() - start > std::chrono::milliseconds(3)))
                    continue;
                auto retry = mRetryAt.find(splat.id);
                if (retry != mRetryAt.end() && retry->second > mRealTime)
                    continue;
                buildSplat(splat);
                ++builds;
                built = mBuilt.find(splat.id);
                if (built == mBuilt.end())
                    continue;
            }
            const double age = hour - splat.hour;
            const float fade = smooth(float(splat.wash / washHours(splat.mark)));
            const float dry = markDry(age);
            const int fadeLevel = int(std::lround(fade * FadeLevels));
            const int dryLevel = int(std::lround(dry * DryLevels));
            if (fadeLevel != built->second.fadeLevel || dryLevel != built->second.dryLevel)
            {
                auto& stamp = built->second;
                if (stamp.procedural)
                {
                    stamp.fade = fade;
                    stamp.dry = dry;
                    if (stamp.growth)
                    {
                        // Ageing is infrequent. Replace the color-bearing
                        // geometry without touching the draw thread's arrays.
                        auto* node = static_cast<osg::MatrixTransform*>(stamp.node.get());
                        auto* previous = static_cast<osg::Geometry*>(node->getChild(0));
                        osg::ref_ptr<osg::Geometry> next = new osg::Geometry(*previous, osg::CopyOp::SHALLOW_COPY);
                        osg::ref_ptr<osg::Vec4Array> color = new osg::Vec4Array;
                        color->push_back(osg::Vec4f(fade, dry, stamp.opacity, 1.f));
                        next->setColorArray(color, osg::Array::BIND_OVERALL);
                        node->replaceChild(previous, next);
                    }
                }
                else if (auto* stateSet = state(splat.style, fade, dry))
                    stamp.node->setStateSet(stateSet);
                if (!stamp.growth)
                    mSplatBatchesDirty = true;
                built->second.fadeLevel = fadeLevel;
                built->second.dryLevel = dryLevel;
            }
        }
        rebuildSplatBatches();
    }

    std::string BrutalBlood::save() const
    {
        Writer world;
        // The world sections have fixed entry limits. Reserve their actual encoded size before
        // selecting weapon marks, whose meshes can vary by thousands of vertices per hit.
        world.u32(std::count_if(mSplats.begin(), mSplats.end(),
            [](const Splat& s) { return s.worldspace.size() <= MaxBloodString; }));
        for (const auto& s : mSplats)
        {
            if (s.worldspace.size() > MaxBloodString)
                continue;
            world.string(s.worldspace);
            world.u32(s.exterior);
            world.vec(s.position);
            world.vec(s.normal);
            world.vec(s.tangent);
            world.f32(s.width);
            world.u32(s.style);
            world.f64(s.hour);
            world.f64(s.wash);
            world.u32(std::uint32_t(s.mark));
            world.f32(s.height);
            world.f32(s.opacity);
            world.u32(s.seed);
            world.u32(s.porous);
            world.f32(std::clamp(mRealTime - s.started, 0.f, PoolGrowthSeconds));
        }
        world.u32(mShoes.size());
        for (const auto& [id, soles] : mShoes)
        {
            world.u32(id.mIndex);
            world.u32(std::uint32_t(id.mContentFile));
            for (const auto& sole : soles)
            {
                world.f32(sole.blood);
                world.f32(std::clamp(sole.driesAt - mRealTime, 0.f, SoleDrySeconds));
                world.f64(sole.stainedHour);
            }
        }
        world.u32(mPoolSources.size());
        for (const auto& [key, source] : mPoolSources)
        {
            const auto [id, outlet] = key;
            world.u32(id.mIndex);
            world.u32(std::uint32_t(id.mContentFile));
            world.string(source.bone);
            world.vec(source.offset);
            world.f32(source.width);
            const float lifetime = outlet == BloodPoolOutlet::Impact ? PoolSourceSeconds : CutPoolSourceSeconds;
            world.f32(std::clamp(source.expires - mRealTime, 0.f, lifetime));
            world.u32(std::uint32_t(outlet));
        }
        Writer out{ "BWB3" };
        struct Candidate
        {
            ESM::RefNum item;
            std::size_t index;
            const WeaponMark* mark;
        };
        std::vector<Candidate> candidates;
        for (const auto& [item, blood] : mItems)
            for (std::size_t i = 0; i < blood.marks.size(); ++i)
                candidates.push_back({ item, i, &blood.marks[i] });
        std::stable_sort(candidates.begin(), candidates.end(),
            [](const Candidate& a, const Candidate& b) { return a.mark->hour > b.mark->hour; });

        std::map<ESM::RefNum, std::bitset<MaxWeaponMarks>> selected;
        std::size_t remaining = MaxSaveBytes - world.data.size() - out.data.size() - sizeof(std::uint32_t);
        for (const Candidate& c : candidates)
        {
            const auto& m = *c.mark;
            if (m.model.size() > MaxBloodString)
                continue;
            // Fixed fields in an item header and a mark, plus the two variable length fields.
            const bool newItem = selected.find(c.item) == selected.end();
            const std::size_t bytes = (newItem ? 20 : 0) + 32 + m.model.size() + AnchorBytes * m.anchors->size();
            if (bytes > remaining || (newItem && selected.size() >= MaxSavedItems))
                continue;
            selected[c.item].set(c.index);
            remaining -= bytes;
        }
        out.u32(selected.size());
        for (const auto& [key, mask] : selected)
        {
            const auto& blood = mItems.at(key);
            out.u32(key.mIndex);
            out.u32(std::uint32_t(key.mContentFile));
            out.f64(blood.wash);
            out.u32(mask.count());
            for (std::size_t i = 0; i < blood.marks.size(); ++i)
            {
                if (!mask[i])
                    continue;
                const auto& m = blood.marks[i];
                out.u32(m.style);
                out.string(m.model);
                out.u32(m.ordinal);
                out.u32(std::uint32_t(m.fingerprint));
                out.u32(std::uint32_t(m.fingerprint >> 32));
                out.f64(m.hour);
                out.u32(m.anchors->size());
                for (const auto& a : *m.anchors)
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
        out.data += world.data;
        return out.data;
    }

    bool BrutalBlood::load(std::string_view data, const std::map<int, int>* contentFiles)
    {
        clear();
        if (data.empty())
            return true;
        const auto version = data.substr(0, 4);
        const bool legacy = version == "BWB1";
        const bool hasCutOutlets = version == "BWB3";
        if (!legacy && version != "BWB2" && !hasCutOutlets)
            return false;
        try
        {
            Reader in{ data };
            auto readRef = [&]() {
                ESM::RefNum id{ in.u32(), std::int32_t(in.u32()) };
                if (contentFiles && id.hasContentFile())
                {
                    const auto it = contentFiles->find(id.mContentFile);
                    if (it == contentFiles->end())
                        return ESM::RefNum{};
                    id.mContentFile = it->second;
                }
                return id;
            };
            // Older writers could exceed the size budget. Consume their complete record while
            // bounding the retained geometry, so that the following world sections still load.
            std::size_t remainingGeometry = MaxSaveBytes;
            std::map<ESM::RefNum, ItemBlood> items;
            const auto itemCount = in.u32();
            if (itemCount > (data.size() - in.p) / 20)
                return false;
            for (unsigned i = 0; i < itemCount; ++i)
            {
                const auto key = readRef();
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
                    if (m.style >= Style_Count || m.ordinal > 4096 || !size || size % 3 || size > MaxWeaponMarkVertices)
                        return false;
                    const std::size_t bytes = size * AnchorBytes;
                    if (bytes > data.size() - in.p)
                        return false;
                    if (!key.isSet() || bytes > remainingGeometry || items.size() >= MaxSavedItems)
                    {
                        in.p += bytes;
                        continue;
                    }
                    remainingGeometry -= bytes;
                    auto anchors = std::make_shared<WeaponAnchors>();
                    anchors->reserve(size);
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
                        anchors->push_back(a);
                    }
                    m.anchors = std::move(anchors);
                    blood.marks.push_back(std::move(m));
                }
                if (!blood.marks.empty())
                    items[key] = std::move(blood);
            }
            std::deque<Splat> splats;
            std::array<std::size_t, 4> markCounts{};
            const auto splatCount = in.u32();
            if (splatCount > MaxSplats + MaxPools + MaxPrints)
                return false;
            for (unsigned i = 0; i < splatCount; ++i)
            {
                Splat s;
                s.worldspace = in.string();
                s.exterior = in.u32() != 0;
                s.position = in.vec();
                s.normal = in.vec();
                s.tangent = in.vec();
                s.width = in.f32();
                s.height = s.width;
                s.style = in.u32();
                s.hour = in.f64();
                s.wash = in.f64();
                if (!legacy)
                {
                    const auto mark = in.u32();
                    if (mark > std::uint32_t(Mark::Footprint))
                        return false;
                    s.mark = Mark(mark);
                    s.height = in.f32();
                    s.opacity = in.f32();
                    s.seed = in.u32();
                    s.porous = in.u32() != 0;
                    s.started = mRealTime - std::clamp(in.f32(), 0.f, PoolGrowthSeconds);
                }
                const auto group = budgetGroup(s.mark);
                if (++markCounts[unsigned(group)] > markLimit(group) || !(s.width > 0.f && s.width < 1000.f)
                    || !(s.height > 0.f && s.height < 1000.f) || s.opacity < 0.f || s.opacity > 1.f
                    || s.normal.length2() < .5f || s.tangent.length2() < .5f
                    || (s.mark == Mark::Imported && (s.style < Style_World || s.style >= Style_Count))
                    || (s.mark == Mark::Footprint && s.style > ArmouredBoot))
                    return false;
                s.id = mNextId++;
                splats.push_back(std::move(s));
            }
            decltype(mShoes) shoes;
            decltype(mPoolSources) sources;
            if (!legacy)
            {
                const auto shoeCount = in.u32();
                if (shoeCount > MaxShoeOwners)
                    return false;
                for (unsigned i = 0; i < shoeCount; ++i)
                {
                    const auto id = readRef();
                    std::array<Sole, 2> soles{};
                    for (auto& sole : soles)
                    {
                        sole.blood = std::clamp(in.f32(), 0.f, 1.f);
                        sole.driesAt = mRealTime + std::clamp(in.f32(), 0.f, SoleDrySeconds);
                        sole.stainedHour = in.f64();
                    }
                    if (id.isSet())
                        shoes.emplace(id, soles);
                }
                const auto sourceCount = in.u32();
                if (sourceCount > MaxPools)
                    return false;
                for (unsigned i = 0; i < sourceCount; ++i)
                {
                    const auto id = readRef();
                    PoolSource source;
                    source.bone = in.string();
                    source.offset = in.vec();
                    source.width = in.f32();
                    const float remaining = in.f32();
                    const auto outlet = hasCutOutlets ? in.u32() : std::uint32_t(BloodPoolOutlet::Impact);
                    if (!(source.width > 0.f && source.width <= 200.f)
                        || outlet > std::uint32_t(BloodPoolOutlet::RightArmFragment))
                        return false;
                    const float lifetime = outlet == std::uint32_t(BloodPoolOutlet::Impact)
                        ? PoolSourceSeconds : CutPoolSourceSeconds;
                    source.expires = mRealTime + std::clamp(remaining, 0.f, lifetime);
                    if (id.isSet())
                        sources.emplace(std::make_pair(id, BloodPoolOutlet(outlet)), std::move(source));
                }
            }
            if (in.p != data.size())
                return false;
            mItems = std::move(items);
            for (auto& [key, blood] : mItems)
                blood.revision = nextRevision();
            mSplats = std::move(splats);
            mShoes = std::move(shoes);
            mPoolSources = std::move(sources);
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
        std::map<std::string, double> result{ { "weaponItems", double(items) }, { "weaponMarks", double(marks) },
            { "weaponAdds", double(mWeaponAdds) }, { "splats", double(mSplats.size()) },
            { "splatsBuilt", double(mBuilt.size()) }, { "splatsThrown", double(mThrown) },
            { "splatsLanded", double(mLanded) }, { "splatBuilds", double(mBuiltCount) },
            { "splatSlowestMs", mSlowestBuildMs },
            { "splatBatches", mSplatBatches ? double(mSplatBatches->getNumChildren()) : 0. },
            { "splatBatchBuilds", double(mSplatBatchBuilds) },
            { "growingPools", double(std::ranges::count_if(mBuilt, [](const auto& entry) { return entry.second.growth.valid(); })) },
            { "proceduralMasks", double(mProceduralLooks.size()) },
            { "worldAvailable", sInstance ? 1. : 0. },
            { "weaponAvailable", const_cast<BrutalBlood*>(this)->hasStyle(Style_Slash) ? 1. : 0. } };
        result["pools"] = result["sprayGroups"] = result["footprints"] = 0.;
        result["wallGroups"] = result["leftFootprints"] = result["rightFootprints"] = 0.;
        result["poolsBuilt"] = result["wallGroupsBuilt"] = result["footprintsBuilt"] = 0.;
        result["largestSprayWidth"] = result["largestSprayStretch"] = 0.;
        result["stainedActors"] = double(mShoes.size());
        result["pendingPools"] = double(mPoolSources.size());
        result["pendingCutPools"] = double(std::ranges::count_if(mPoolSources, [](const auto& entry) {
            return entry.first.second != BloodPoolOutlet::Impact;
        }));
        const double hour = now();
        for (const auto& splat : mSplats)
        {
            if (splat.mark == Mark::Pool)
            {
                ++result["pools"];
                result["poolsBuilt"] += mBuilt.contains(splat.id);
                result["poolX"] = splat.position.x();
                result["poolY"] = splat.position.y();
                result["poolZ"] = splat.position.z();
                result["poolWidth"] = splat.width;
                result["poolHeight"] = splat.height;
                result["poolTangentX"] = splat.tangent.x();
                result["poolTangentY"] = splat.tangent.y();
                result["poolPorous"] = splat.porous ? 1. : 0.;
                result["poolGrowth"] = poolScale(splat, hour);
            }
            else if (splat.mark == Mark::Footprint)
            {
                ++result["footprints"];
                result["footprintsBuilt"] += mBuilt.contains(splat.id);
                ++result[(splat.seed & 1u) ? "leftFootprints" : "rightFootprints"];
                result["latestFootprintOpacity"] = splat.opacity;
            }
            else if (splat.mark == Mark::Spray)
            {
                ++result["sprayGroups"];
                result["largestSprayWidth"] = std::max(result["largestSprayWidth"], double(splat.width));
                result["largestSprayStretch"] = std::max(result["largestSprayStretch"], double(splat.width / splat.height));
                if (std::abs(splat.normal.z()) < .45f)
                {
                    ++result["wallGroups"];
                    result["wallGroupsBuilt"] += mBuilt.contains(splat.id);
                }
            }
        }
        return result;
    }
}
