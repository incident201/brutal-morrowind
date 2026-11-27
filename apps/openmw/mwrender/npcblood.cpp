#include "npcblood.hpp"
#include "brutalblood.hpp"
#include "effectmanager.hpp"
#include "../mwworld/inventorystore.hpp"
#include "../mwworld/worldmodel.hpp"
#include "../mwworld/cellstore.hpp"
#include <components/esm3/loadweap.hpp>
#include "../mwbase/environment.hpp"
#include "../mwbase/world.hpp"
#include "../mwmechanics/actorutil.hpp"
#include "../mwmechanics/creaturestats.hpp"
#include "../mwworld/class.hpp"
#include "decapitation.hpp"
#include "bisection.hpp"
#include "limbsevering.hpp"
#include "npcanimation.hpp"
#include "releasedequipmentcapture.hpp"
#include "vismask.hpp"
#include <algorithm>
#include <bit>
#include <chrono>
#include <components/debug/debuglog.hpp>
#include <components/esm3/loadarmo.hpp>
#include <components/sceneutil/decalprojector.hpp>
#include <components/sceneutil/positionattitudetransform.hpp>
#include <components/sceneutil/skeleton.hpp>
#include <components/settings/values.hpp>
#include <components/vfs/manager.hpp>
#include <cstring>
#include <limits>
#include <map>
#include <optional>
#include <set>
#include <osg/FrameStamp>
#include <osg/TriangleIndexFunctor>
#include <osgUtil/CullVisitor>

namespace MWRender
{
    namespace
    {
        using Anchors = std::vector<SceneUtil::SurfaceDecals::Anchor>;
        constexpr std::size_t MaxWounds = 24, MaxActorVertices = 8192, MaxSaveBytes = 400000;

        // Ordinary Geometry is often shared between actors. Replace only this
        // actor's parent edge, keeping source arrays/materials immutable.
        class BloodMarkedGeometry final : public osg::Geometry
        {
        public:
            BloodMarkedGeometry() = default;
            explicit BloodMarkedGeometry(const osg::Geometry& source)
                : osg::Geometry(source, osg::CopyOp::SHALLOW_COPY)
            {
            }
            BloodMarkedGeometry(const BloodMarkedGeometry& source, const osg::CopyOp& op)
                : osg::Geometry(source, op)
            {
            }
            META_Object(MWRender, BloodMarkedGeometry)
            osg::ref_ptr<SceneUtil::SurfaceDecals> decals;
            void accept(osg::NodeVisitor& visitor) override
            {
                if (!visitor.validNodeMask(*this))
                    return;
                osg::Geometry::accept(visitor);
                if (decals)
                    if (auto* cull = dynamic_cast<osgUtil::CullVisitor*>(&visitor))
                    {
                        visitor.pushOntoNodePath(this);
                        decals->cull(*this, *cull);
                        visitor.popFromNodePath();
                    }
            }
        };

        struct Triangles
        {
            std::vector<SceneUtil::DecalProjector::Triangle> triangles;
            void operator()(unsigned a, unsigned b, unsigned c) { triangles.push_back({ a, b, c }); }
        };
        struct Surface
        {
            int part = 0;
            unsigned ordinal = 0;
            std::string model;
            osg::ref_ptr<osg::Drawable> drawable;
            osg::ref_ptr<osg::Geometry> source;
            osg::NodePath path;
            std::vector<SceneUtil::DecalProjector::Triangle> triangles;
            std::uint64_t fingerprint = 0;

            SceneUtil::SurfaceDecals* attach()
            {
                if (auto* rig = dynamic_cast<SceneUtil::RigGeometry*>(drawable.get()))
                    return rig->getOrCreateSurfaceDecals();
                if (auto* morph = dynamic_cast<SceneUtil::MorphGeometry*>(drawable.get()))
                    return morph->getOrCreateSurfaceDecals();
                if (auto* plain = dynamic_cast<BloodMarkedGeometry*>(drawable.get()))
                {
                    if (!plain->decals)
                        plain->decals = new SceneUtil::SurfaceDecals;
                    return plain->decals;
                }
                if (path.size() < 2 || !path[path.size() - 2]->asGroup())
                    return nullptr;
                auto* plain = drawable->asGeometry();
                if (!plain)
                    return nullptr;
                osg::ref_ptr<BloodMarkedGeometry> replacement = new BloodMarkedGeometry(*plain);
                replacement->decals = new SceneUtil::SurfaceDecals;
                auto* parent = path[path.size() - 2]->asGroup();
                if (!parent->replaceChild(drawable, replacement))
                    return nullptr;
                drawable = replacement;
                source = replacement;
                path.back() = replacement;
                return replacement->decals;
            }
        };

        struct BloodPatch
        {
            osg::observer_ptr<SceneUtil::SurfaceDecals> decals;
            SceneUtil::SurfaceDecals::PatchId id = 0;

            bool attach(Surface& surface, const Anchors& anchors, osg::StateSet* state)
            {
                if (auto* marks = surface.attach())
                {
                    id = marks->addPatch(anchors, state, Mask_Effect);
                    if (id)
                        decals = marks;
                }
                return id != 0;
            }

            void remove()
            {
                if (auto* marks = decals.get())
                    marks->removePatch(id);
                decals = nullptr;
                id = 0;
            }
        };

        struct BloodDecalRecord
        {
            int part = 0;
            unsigned ordinal = 0, style = 0;
            std::string model;
            std::uint64_t fingerprint = 0;
            Anchors anchors;
            BloodPatch patch;
        };

        struct SurfacesVisitor final : osg::NodeVisitor
        {
            std::vector<Surface>& out;
            int part;
            std::string model;
            unsigned ordinal = 0;
            SurfacesVisitor(std::vector<Surface>& output, int p, std::string m)
                : osg::NodeVisitor(TRAVERSE_ALL_CHILDREN)
                , out(output)
                , part(p)
                , model(std::move(m))
            {
            }
            void collect(osg::Node* root, osg::Node* actor)
            {
                const auto prefix = releasedEquipmentPath(root, actor);
                for (std::size_t i = 0; i + 1 < prefix.size(); ++i)
                    pushOntoNodePath(prefix[i]);
                root->accept(*this);
                for (std::size_t i = 0; i + 1 < prefix.size(); ++i)
                    popFromNodePath();
            }
            void apply(osg::Drawable& drawable) override
            {
                Surface s;
                s.part = part;
                s.model = model;
                s.ordinal = ordinal++;
                s.drawable = &drawable;
                s.path = getNodePath();
                if (auto* rig = dynamic_cast<SceneUtil::RigGeometry*>(&drawable))
                    s.source = rig->getSourceGeometry();
                else if (auto* morph = dynamic_cast<SceneUtil::MorphGeometry*>(&drawable))
                    s.source = morph->getSourceGeometry();
                else if (typeid(drawable) == typeid(osg::Geometry) || dynamic_cast<BloodMarkedGeometry*>(&drawable))
                    s.source = drawable.asGeometry();
                if (!s.source)
                    return;
                const auto* vertices = dynamic_cast<const osg::Vec3Array*>(s.source->getVertexArray());
                if (!vertices || vertices->empty() || vertices->size() > 65535)
                    return;
                osg::TriangleIndexFunctor<Triangles> functor;
                s.source->accept(functor);
                s.triangles = std::move(functor.triangles);
                std::uint64_t hash = 14695981039346656037ull;
                const auto bytes = [&](const void* data, std::size_t size) {
                    auto* p = static_cast<const unsigned char*>(data);
                    while (size--)
                    {
                        hash ^= *p++;
                        hash *= 1099511628211ull;
                    }
                };
                for (const auto& p : *vertices)
                    for (unsigned i = 0; i < 3; ++i)
                    {
                        const float value = p[i];
                        bytes(&value, sizeof(value));
                    }
                for (const auto& t : s.triangles)
                    for (auto i : t)
                        bytes(&i, sizeof(i));
                s.fingerprint = hash;
                out.push_back(std::move(s));
            }
        };

        osg::ref_ptr<osg::Geometry> posed(
            const Surface& s, const std::function<std::optional<osg::Matrixf>(std::string_view)>& bones)
        {
            if (auto* rig = dynamic_cast<SceneUtil::RigGeometry*>(s.drawable.get()))
                return rig->snapshotWorldGeometry(s.path, bones, true);
            const auto* source = dynamic_cast<const osg::Vec3Array*>(s.source->getVertexArray());
            if (!source)
                return {};
            osg::ref_ptr<osg::Geometry> result = new osg::Geometry;
            osg::ref_ptr<osg::Vec3Array> vertices = new osg::Vec3Array(*source);
            if (auto* morph = dynamic_cast<SceneUtil::MorphGeometry*>(s.drawable.get()))
            {
                const auto& targets = morph->getMorphTargetList();
                if (!targets.empty())
                {
                    if (targets[0].getOffsets()->size() != vertices->size())
                        return {};
                    *vertices = *targets[0].getOffsets();
                    for (std::size_t i = 1; i < targets.size(); ++i)
                    {
                        if (targets[i].getOffsets()->size() != vertices->size())
                            return {};
                        for (std::size_t j = 0; j < vertices->size(); ++j)
                            (*vertices)[j] += (*targets[i].getOffsets())[j] * targets[i].getWeight();
                    }
                }
            }
            const auto matrix = osg::computeLocalToWorld(s.path);
            for (auto& p : *vertices)
                p = p * matrix;
            result->setVertexArray(vertices);
            return result;
        }

        struct Writer
        {
            std::string data = "BMD1";
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
                    throw std::runtime_error("Truncated NPC blood record");
                std::uint32_t n = 0;
                for (unsigned i = 0; i < 4; ++i)
                    n |= std::uint32_t(static_cast<unsigned char>(data[p++])) << (i * 8);
                return n;
            }
            float f32()
            {
                auto n = u32();
                float f;
                std::memcpy(&f, &n, 4);
                if (!std::isfinite(f))
                    throw std::runtime_error("Nonfinite NPC blood record");
                return f;
            }
            std::string string()
            {
                const auto size = u32();
                if (size > 1024 || p + size > data.size())
                    throw std::runtime_error("Invalid NPC blood model");
                std::string s(data.substr(p, size));
                p += size;
                return s;
            }
        };

        std::string writeBloodDecalData(const std::vector<BloodDecalRecord>& records)
        {
            if (records.empty())
                return {};
            Writer out;
            out.u32(records.size());
            for (const auto& record : records)
            {
                out.u32(record.part);
                out.u32(record.ordinal);
                out.u32(record.style);
                out.string(record.model);
                out.u32(record.fingerprint & 0xffffffff);
                out.u32(record.fingerprint >> 32);
                out.u32(record.anchors.size());
                for (const auto& anchor : record.anchors)
                {
                    for (auto index : anchor.indices)
                        out.u32(index);
                    for (unsigned k = 0; k < 3; ++k)
                        out.f32(anchor.weights[k]);
                    out.f32(anchor.uv.x());
                    out.f32(anchor.uv.y());
                }
            }
            return out.data;
        }

        std::optional<std::vector<BloodDecalRecord>> readBloodDecalData(std::string_view data)
        {
            std::vector<BloodDecalRecord> records;
            if (data.empty())
                return records;
            if (data.size() > MaxSaveBytes || data.substr(0, 4) != "BMD1")
                return std::nullopt;
            try
            {
                Reader in{ data };
                const auto count = in.u32();
                if (count > MaxWounds)
                    return std::nullopt;
                std::size_t total = 0;
                for (unsigned j = 0; j < count; ++j)
                {
                    BloodDecalRecord record;
                    record.part = in.u32();
                    record.ordinal = in.u32();
                    record.style = in.u32();
                    record.model = in.string();
                    record.fingerprint = in.u32();
                    record.fingerprint |= std::uint64_t(in.u32()) << 32;
                    const auto size = in.u32();
                    if (record.part < 0 || record.part >= ESM::PRT_Count || record.ordinal > 4096 || record.style >= 4
                        || !size || size % 3 || size > 3072 || (total += size) > MaxActorVertices)
                        return std::nullopt;
                    record.anchors.reserve(size);
                    for (unsigned i = 0; i < size; ++i)
                    {
                        SceneUtil::SurfaceDecals::Anchor anchor;
                        for (auto& index : anchor.indices)
                        {
                            index = in.u32();
                            if (index > 65535)
                                return std::nullopt;
                        }
                        for (unsigned k = 0; k < 3; ++k)
                            anchor.weights[k] = in.f32();
                        const float u = in.f32(), v = in.f32();
                        anchor.uv = { u, v };
                        record.anchors.push_back(anchor);
                    }
                    records.push_back(std::move(record));
                }
                if (in.p != data.size())
                    return std::nullopt;
            }
            catch (const std::runtime_error&)
            {
                return std::nullopt;
            }
            return records;
        }

        void mergeBloodDecalRecords(std::vector<BloodDecalRecord>& saved, const std::vector<BloodDecalRecord>& recent)
        {
            // Both inputs have oldest-first order and often contain the same
            // records after re-activation or when the previous restore was retried.
            std::size_t total = 0;
            for (const auto& record : saved)
                total += record.anchors.size();
            for (const auto& record : recent)
            {
                const bool duplicate = std::ranges::any_of(saved, [&](const BloodDecalRecord& old) {
                    return old.part == record.part && old.ordinal == record.ordinal && old.style == record.style
                        && old.fingerprint == record.fingerprint && old.model == record.model
                        && std::ranges::equal(old.anchors, record.anchors, [](const auto& a, const auto& b) {
                            return a.indices == b.indices && a.weights == b.weights && a.uv == b.uv;
                        });
                });
                if (duplicate)
                    continue;
                while (!saved.empty() && (saved.size() >= MaxWounds || total + record.anchors.size() > MaxActorVertices))
                {
                    total -= saved.front().anchors.size();
                    saved.erase(saved.begin());
                }
                total += record.anchors.size();
                saved.push_back(record);
                saved.back().patch = {};
            }
        }
    }

    std::string mergeNpcBloodDecalData(std::string_view saved, std::string_view recent)
    {
        auto oldRecords = readBloodDecalData(saved);
        const auto recentRecords = readBloodDecalData(recent);
        if (!oldRecords)
            return recentRecords ? std::string(recent) : std::string();
        if (recentRecords)
            mergeBloodDecalRecords(*oldRecords, *recentRecords);
        return writeBloodDecalData(*oldRecords);
    }

    bool isRigidBloodSurface(const osg::Geometry* geometry)
    {
        return dynamic_cast<const BloodMarkedGeometry*>(geometry) != nullptr;
    }

    struct NpcAnimation::CutBloodFlow
    {
        struct Callback final : osg::NodeCallback
        {
            NpcAnimation* owner;
            explicit Callback(NpcAnimation* value)
                : owner(value)
            {
            }
            void operator()(osg::Node* node, osg::NodeVisitor* visitor) override
            {
                traverse(node, visitor);
                if (visitor->getVisitorType() == osg::NodeVisitor::UPDATE_VISITOR)
                    owner->emitCutBloodFlow(*visitor);
            }
        };
        osg::observer_ptr<osg::Group> root;
        osg::ref_ptr<Callback> callback;
        double age = 0, nextBurst = 0, nextCore = 0;
        unsigned lastTraversal = ~0u, bursts = 0, cores = 0;
        std::uint32_t random = 0x6f08ab31;
        bool active = true;
        bool torso = false;
        bool limbs = false;
        osg::Vec3f lastPosition;
        int draw(int lo, int hi)
        {
            random ^= random << 13;
            random ^= random >> 17;
            random ^= random << 5;
            return lo + int(random % unsigned(hi - lo + 1));
        }
    };

    void NpcAnimation::clearCutBloodFlow()
    {
        if (!mCutBloodFlow)
            return;
        if (mCutBloodFlow->root.valid() && mCutBloodFlow->callback)
            mCutBloodFlow->root->removeUpdateCallback(mCutBloodFlow->callback);
        mCutBloodFlow->root = nullptr;
        mCutBloodFlow->callback = nullptr;
        mCutBloodFlow->active = false;
    }

    void NpcAnimation::startCutBloodFlow()
    {
        if (!mDecapitation)
            return;
        if (auto* blood = BrutalBlood::get())
            blood->queueCutPool(mPtr, BloodPoolOutlet::Neck);
        if (!Settings::game().mNpcBloodDecals || !Settings::game().mBrutalImpactEffects)
            return;
        clearCutBloodFlow();
        mCutBloodFlow = std::make_shared<CutBloodFlow>();
        updateCutBloodFlow(0);
    }

    void NpcAnimation::startBisectionBloodFlow()
    {
        if (!mBisection)
            return;
        if (auto* blood = BrutalBlood::get())
        {
            blood->queueCutPool(mPtr, BloodPoolOutlet::LowerWaist);
            blood->queueCutPool(mPtr, BloodPoolOutlet::UpperWaist);
        }
        if (!Settings::game().mNpcBloodDecals || !Settings::game().mBrutalImpactEffects)
            return;
        clearCutBloodFlow();
        mCutBloodFlow = std::make_shared<CutBloodFlow>();
        mCutBloodFlow->torso = true;
        updateCutBloodFlow(0);
    }

    namespace
    {
        // The pairs follow LimbSeveringGeometry's mask order, with the body side first.
        constexpr std::array sLimbPoolOutlets{
            std::pair{ BloodPoolOutlet::LeftLegStump, BloodPoolOutlet::LeftLegFragment },
            std::pair{ BloodPoolOutlet::RightLegStump, BloodPoolOutlet::RightLegFragment },
            std::pair{ BloodPoolOutlet::LeftArmStump, BloodPoolOutlet::LeftArmFragment },
            std::pair{ BloodPoolOutlet::RightArmStump, BloodPoolOutlet::RightArmFragment },
        };
    }

    void NpcAnimation::startLimbSeveringBloodFlow()
    {
        if (!mLimbSevering)
            return;
        if (auto* blood = BrutalBlood::get())
            for (unsigned i = 0; i < sLimbPoolOutlets.size(); ++i)
                if (mLimbSevering->mask & (1u << i))
                {
                    const auto [stump, fragment] = sLimbPoolOutlets[i];
                    blood->queueCutPool(mPtr, stump);
                    blood->queueCutPool(mPtr, fragment);
                }
        if (!Settings::game().mNpcBloodDecals || !Settings::game().mBrutalImpactEffects)
            return;
        clearCutBloodFlow();
        mCutBloodFlow = std::make_shared<CutBloodFlow>();
        mCutBloodFlow->limbs = true;
        updateCutBloodFlow(0);
    }

    std::optional<CutPoolPoint> NpcAnimation::getCutPoolPoint(BloodPoolOutlet outlet) const
    {
        const auto sample = [&](std::string_view bone, const osg::Vec3f& center, float radius)
            -> std::optional<CutPoolPoint> {
            if (const auto matrix = getBoneWorldMatrix(bone))
                return CutPoolPoint{ center * *matrix, radius };
            return std::nullopt;
        };
        if (outlet == BloodPoolOutlet::Neck && mDecapitation)
            return sample("Bip01 Neck", mDecapitation->surfaceCenters[0], mDecapitation->surfaceRadius);
        if ((outlet == BloodPoolOutlet::LowerWaist || outlet == BloodPoolOutlet::UpperWaist) && mBisection)
        {
            const bool upper = outlet == BloodPoolOutlet::UpperWaist;
            return sample(upper ? "Bip01 Spine1" : "Bip01 Pelvis", mBisection->surfaceCenters[upper],
                mBisection->surfaceRadius);
        }
        if (mLimbSevering)
            for (const auto& cut : mLimbSevering->cuts)
            {
                const auto [stump, fragment] = sLimbPoolOutlets[std::countr_zero(cut.bit)];
                if (outlet == stump)
                    return sample(cut.parent, cut.surfaceCenters[0], cut.surfaceRadii[0]);
                if (outlet == fragment)
                    return sample(cut.child, cut.surfaceCenters[1], cut.surfaceRadii[1]);
            }
        return std::nullopt;
    }

    void NpcAnimation::updateCutBloodFlow(float dt)
    {
        if (!mCutBloodFlow || !mCutBloodFlow->active)
            return;
        auto& flow = *mCutBloodFlow;
        flow.age += std::max(0.f, dt);
        if ((flow.limbs ? !mLimbSevering : flow.torso ? !mBisection : !mDecapitation) || flow.age >= 18 || !Settings::game().mBrutalImpactEffects)
        {
            clearCutBloodFlow();
            return;
        }
        if (flow.root.get() != mObjectRoot.get())
        {
            if (flow.root.valid() && flow.callback)
                flow.root->removeUpdateCallback(flow.callback);
            flow.root = mObjectRoot;
            flow.callback = new CutBloodFlow::Callback(this);
            mObjectRoot->addUpdateCallback(flow.callback);
        }
    }

    void NpcAnimation::emitCutBloodFlow(osg::NodeVisitor& visitor)
    {
        if (!mCutBloodFlow || !mCutBloodFlow->active
            || (mCutBloodFlow->limbs ? !mLimbSevering : mCutBloodFlow->torso ? !mBisection : !mDecapitation))
            return;
        auto& flow = *mCutBloodFlow;
        if (flow.lastTraversal == visitor.getTraversalNumber())
            return;
        flow.lastTraversal = visitor.getTraversalNumber();
        const auto surfaces = flow.limbs ? getLimbSeveringSurfacesForAudit()
            : flow.torso ? getBisectionSurfacesForAudit() : getDecapitationSurfacesForAudit();
        std::vector<std::pair<osg::Vec3f, osg::Vec3f>> outlets;
        if (flow.limbs)
        {
            for (const auto& cut : mLimbSevering->cuts)
            {
                const auto center = surfaces.find(cut.name + "StumpCenter"), normal = surfaces.find(cut.name + "StumpNormal");
                if (center != surfaces.end() && normal != surfaces.end()) outlets.emplace_back(center->second, normal->second);
            }
        }
        else
        {
            const auto center = surfaces.find("lowerCenter"), normal = surfaces.find("lowerNormal");
            if (center != surfaces.end() && normal != surfaces.end()) outlets.emplace_back(center->second, normal->second);
        }
        if (outlets.empty()) return;
        flow.lastPosition = outlets.front().first;
        const double clock = visitor.getFrameStamp() ? visitor.getFrameStamp()->getSimulationTime() : flow.age;
        const double wave = std::abs(std::sin(clock * .5));
        const float envelope = flow.age <= 1 ? 1.f : std::clamp(float((8 - flow.age - wave) / 8), 0.f, 1.f);
        const auto world = MWBase::Environment::get().getWorld();
        const auto spawn = [&](std::string_view kind, float amount) {
            for (const auto& [center, normal] : outlets)
                world->spawnBrutalImpact(kind, center, normal, amount, false);
        };
        // Dark Messiah's CutBlood_Start: neck-attached body flow. The Morrowind
        // cut centre already replaces DM's model-specific Neck+4.7 offset.
        if (flow.age < 8.45 && flow.age >= flow.nextBurst)
        {
            flow.nextBurst = flow.age + .06;
            ++flow.bursts;
            spawn("cutspatter", envelope);
            spawn("cutspray", envelope);
            if (flow.age < 3.45)
                spawn("cutdrop", envelope);
        }
        if (flow.age < 14.45 && flow.age >= flow.nextCore)
        {
            flow.nextCore = flow.age + .5;
            int count = std::min(8, int(27 - 2.5 * flow.age - 2 * wave));
            if (count < 1)
                count = flow.draw(-20, 0) < count ? 1 : 0;
            if (flow.draw(0, 10) > 4)
                for (int i = 0; i < count; ++i)
                {
                    const float amount = float((.55 + .125 * (flow.draw(0, 1000) / 1000.)) * (envelope + .5) * wave);
                    spawn("cutcore", amount);
                    ++flow.cores;
                }
        }
    }

    struct NpcAnimation::BloodDecals
    {
        std::vector<BloodDecalRecord> records;
        std::array<osg::ref_ptr<osg::StateSet>, 4> states;
        std::uint64_t added = 0, rejected = 0, evicted = 0;
        double lastDistance = 0, lastMilliseconds = 0;
    };

    void NpcAnimation::prewarmBloodDecals()
    {
        if (!Settings::game().mNpcBloodDecals || mPtr == MWMechanics::getPlayer() || mViewMode != VM_Normal
            || mBodyAwareness)
            return;
        if (!mBloodDecals)
            if (auto* store = BrutalBlood::get())
            {
                mBloodDecals = std::make_shared<BloodDecals>();
                mBloodDecals->states = store->woundStates();
            }
    }

    void NpcAnimation::rebuildBloodDecals()
    {
        if (!mBloodDecals || mBloodDecals->records.empty())
            return;
        auto& blood = *mBloodDecals;
        for (auto& record : blood.records)
            record.patch.remove();
        if (!Settings::game().mNpcBloodDecals)
            return;
        std::vector<Surface> surfaces;
        for (int part = 0; part < ESM::PRT_Count; ++part)
            if (part != ESM::PRT_Weapon && part != ESM::PRT_Shield && mObjectParts[part])
            {
                SurfacesVisitor visitor(surfaces, part, mPartModels[part]);
                if (mDecapitation || mBisection || mLimbSevering)
                    visitor.setNodeMaskOverride(~0u);
                visitor.collect(mObjectParts[part]->getNode(), mPtr.getRefData().getBaseNode());
            }
        for (auto& record : blood.records)
            for (auto& s : surfaces)
                if (record.part == s.part && record.ordinal == s.ordinal && record.model == s.model
                    && record.fingerprint == s.fingerprint
                    && record.patch.attach(s, record.anchors, blood.states[record.style]))
                    break;
    }

    bool NpcAnimation::addBloodDecal(
        const osg::Vec3f& position, const osg::Vec3f& outward, std::string_view kind, bool power, int hitPart)
    {
        prewarmBloodDecals();
        if (!mBloodDecals || !Settings::game().mNpcBloodDecals || mDecapitation || mBisection || mLimbSevering)
            return false;
        auto& blood = *mBloodDecals;
        const unsigned style = kind == "slash" || kind == "chop" ? 0 : power ? 2 : 1;
        if (!blood.states[style])
            return false;
        const auto start = std::chrono::steady_clock::now();
        try
        {
            std::vector<Surface> surfaces;
            for (int part = 0; part < ESM::PRT_Count; ++part)
                if (part != ESM::PRT_Weapon && part != ESM::PRT_Shield && part != ESM::PRT_Hair && mObjectParts[part]
                    && (hitPart < 0 || hitPart == part))
                {
                    SurfacesVisitor visitor(surfaces, part, mPartModels[part]);
                    visitor.collect(mObjectParts[part]->getNode(), mPtr.getRefData().getBaseNode());
                }
            const auto bones = [this](std::string_view name) { return getBoneWorldMatrix(name); };
            SceneUtil::DecalProjector::Result best;
            Surface* selected = nullptr;
            auto tangent = osg::Vec3d(0, 0, 1) ^ osg::Vec3d(outward);
            if (kind == "chop")
                tangent = osg::Vec3d(0, 0, 1);
            // Native units: the DM slash is 64 px at .1 Source inches/px.
            const double size = kind == "projectile" ? 12. : (style == 0 ? 15. : 24.) * (power ? 1.35 : 1.);
            for (auto& s : surfaces)
                if (auto geometry = posed(s, bones))
                {
                    const auto* vertices = dynamic_cast<const osg::Vec3Array*>(geometry->getVertexArray());
                    if (!vertices)
                        continue;
                    std::vector<osg::Vec3d> world(vertices->begin(), vertices->end());
                    auto projection = SceneUtil::DecalProjector::project(
                        world, s.triangles, osg::Vec3d(position), osg::Vec3d(outward), tangent, size, size);
                    if (!projection.vertices.empty() && projection.distance < best.distance)
                    {
                        best = std::move(projection);
                        selected = &s;
                    }
                }
            if (!selected)
            {
                ++blood.rejected;
                return false;
            }
            BloodDecalRecord record{ selected->part, selected->ordinal, style, selected->model,
                selected->fingerprint, std::move(best.vertices), {} };
            if (!record.patch.attach(*selected, record.anchors, blood.states[style]))
            {
                ++blood.rejected;
                return false;
            }
            std::size_t total = 0;
            for (const auto& r : blood.records)
                total += r.anchors.size();
            while (!blood.records.empty()
                && (blood.records.size() >= MaxWounds || total + record.anchors.size() > MaxActorVertices))
            {
                total -= blood.records.front().anchors.size();
                blood.records.front().patch.remove();
                blood.records.erase(blood.records.begin());
                ++blood.evicted;
            }
            blood.records.push_back(std::move(record));
            ++blood.added;
            blood.lastDistance = best.distance;
            blood.lastMilliseconds
                = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - start).count();
            return true;
        }
        catch (const std::exception& error)
        {
            ++blood.rejected;
            Log(Debug::Warning) << "NPC blood projection rejected: " << error.what();
            return false;
        }
    }

    std::string NpcAnimation::getBloodDecalData() const
    {
        return mBloodDecals ? writeBloodDecalData(mBloodDecals->records) : std::string();
    }

    bool NpcAnimation::restoreBloodDecals(std::string_view data, bool merge)
    {
        prewarmBloodDecals();
        if (!mBloodDecals || !Settings::game().mNpcBloodDecals)
            return false;
        try
        {
            auto records = readBloodDecalData(data);
            if (!records)
                return false;
            for (const auto& record : *records)
            {
                osg::ref_ptr<SceneUtil::SurfaceDecals> validate = new SceneUtil::SurfaceDecals;
                if (!validate->addPatch(record.anchors, new osg::StateSet, Mask_Effect))
                    return false;
            }
            if (merge)
                mergeBloodDecalRecords(*records, mBloodDecals->records);
            for (auto& record : mBloodDecals->records)
                record.patch.remove();
            mBloodDecals->records = std::move(*records);
            rebuildBloodDecals();
            if (mDecapitation)
                transferBloodDecalsToHead(*mDecapitation);
            if (mBisection)
            {
                transferBloodDecalsToTorso(*mBisection);
                mBisection->prepareDrawBuffers();
            }
            if (mLimbSevering)
            {
                transferBloodDecalsToLimbs(*mLimbSevering);
                mLimbSevering->prepareDrawBuffers();
            }
            return true;
        }
        catch (const std::exception&)
        {
            return false;
        }
    }

    std::map<std::string, double> NpcAnimation::getBloodDecalStats() const
    {
        if (!mBloodDecals)
            return { { "enabled", Settings::game().mNpcBloodDecals ? 1. : 0. }, { "available", 0 }, { "records", 0 },
                { "attached", 0 } };
        const auto& b = *mBloodDecals;
        std::set<const SceneUtil::SurfaceDecals*> surfaces;
        std::size_t attached = 0, vertices = 0, evaluations = 0;
        for (const auto& record : b.records)
            if (const auto* marks = record.patch.decals.get())
            {
                ++attached;
                vertices += record.anchors.size();
                if (surfaces.insert(marks).second)
                    evaluations += marks->evaluationCount();
            }
        std::size_t frozenHead = 0, frozenNeck = 0;
        if (mDecapitation)
            for (unsigned side = 0; side < 2; ++side)
            {
                const auto* group = side ? mDecapitation->upper.get() : mDecapitation->lower.get();
                for (unsigned i = 0; i < group->getNumChildren(); ++i)
                    if (group->getChild(i)->getName() == "Frozen NPC blood decal")
                        ++(side ? frozenHead : frozenNeck);
            }
        return { { "enabled", Settings::game().mNpcBloodDecals ? 1. : 0. }, { "available", b.states[0] ? 1. : 0. },
            { "records", double(b.records.size()) }, { "attached", double(attached) },
            { "vertices", double(vertices) }, { "evaluations", double(evaluations) },
            { "added", double(b.added) }, { "rejected", double(b.rejected) },
            { "evicted", double(b.evicted) }, { "lastDistance", b.lastDistance },
            { "lastMilliseconds", b.lastMilliseconds }, { "limit", double(MaxWounds) },
            { "frozenHead", double(frozenHead) }, { "frozenNeck", double(frozenNeck) },
            { "bisectedTriangles", mBisection ? double(mBisection->decalTriangles) : 0. },
            { "limbTriangles", mLimbSevering ? double(mLimbSevering->decalTriangles) : 0. },
            { "lastPart", b.records.empty() ? -1. : double(b.records.back().part) },
            { "garmentCoverage", mGarmentCoverage ? 1. : 0. },
            { "cutFlowActive", mCutBloodFlow && mCutBloodFlow->active ? 1. : 0. },
            { "cutFlowAge", mCutBloodFlow ? mCutBloodFlow->age : 0. },
            { "cutFlowBursts", mCutBloodFlow ? double(mCutBloodFlow->bursts) : 0. },
            { "cutFlowCores", mCutBloodFlow ? double(mCutBloodFlow->cores) : 0. } };
    }

    void NpcAnimation::transferBloodDecalsToTorso(BisectionGeometry& target)
    {
        target.clearDecals();
        if (!mBloodDecals || !Settings::game().mNpcBloodDecals) return;
        std::vector<Surface> surfaces;
        for (auto part : { ESM::PRT_Cuirass, ESM::PRT_Groin, ESM::PRT_Skirt })
            if (mObjectParts[part])
            {
                SurfacesVisitor visitor(surfaces, part, mPartModels[part]);
                visitor.setNodeMaskOverride(~0u);
                visitor.collect(mObjectParts[part]->getNode(), mPtr.getRefData().getBaseNode());
            }
        for (const auto& record : mBloodDecals->records)
            for (const auto& surface : surfaces)
                if (record.part == surface.part && record.ordinal == surface.ordinal && record.model == surface.model
                    && record.fingerprint == surface.fingerprint && record.style < mBloodDecals->states.size()
                    && mBloodDecals->states[record.style])
                {
                    SceneUtil::SurfaceDecals::PatchData patch{ record.anchors, mBloodDecals->states[record.style], Mask_Effect };
                    for (auto& anchor : patch.vertices)
                        anchor.weights /= anchor.weights.x() + anchor.weights.y() + anchor.weights.z();
                    target.addDecal(surface.drawable, patch);
                    break;
                }
    }

    void NpcAnimation::transferBloodDecalsToLimbs(LimbSeveringGeometry& target)
    {
        target.clearDecals();
        if (!mBloodDecals || !Settings::game().mNpcBloodDecals) return;
        std::vector<Surface> surfaces;
        for (int part = 0; part < ESM::PRT_Count; ++part)
            if (mObjectParts[part] && part != ESM::PRT_Weapon && part != ESM::PRT_Shield)
            {
                SurfacesVisitor visitor(surfaces, part, mPartModels[part]);
                visitor.setNodeMaskOverride(~0u);
                visitor.collect(mObjectParts[part]->getNode(), mPtr.getRefData().getBaseNode());
            }
        for (const auto& record : mBloodDecals->records)
            for (const auto& surface : surfaces)
                if (record.part == surface.part && record.ordinal == surface.ordinal && record.model == surface.model
                    && record.fingerprint == surface.fingerprint && record.style < mBloodDecals->states.size()
                    && mBloodDecals->states[record.style])
                {
                    SceneUtil::SurfaceDecals::PatchData patch{ record.anchors, mBloodDecals->states[record.style], Mask_Effect };
                    for (auto& anchor : patch.vertices)
                        anchor.weights /= anchor.weights.x() + anchor.weights.y() + anchor.weights.z();
                    target.addDecal(surface.drawable, patch);
                    break;
                }
    }

    void NpcAnimation::transferBloodDecalsToHead(DecapitationGeometry& target, bool bindSources)
    {
        if (bindSources)
        {
            std::vector<Surface> surfaces;
            for (auto part : { ESM::PRT_Head, ESM::PRT_Hair, ESM::PRT_Neck })
                if (mObjectParts[part])
                {
                    SurfacesVisitor visitor(surfaces, part, mPartModels[part]);
                    visitor.setNodeMaskOverride(~0u);
                    visitor.collect(mObjectParts[part]->getNode(), mPtr.getRefData().getBaseNode());
                }
            for (auto& cached : target.bloodSources)
            {
                osg::ref_ptr<osg::Drawable> drawable;
                if (cached.sourceDrawable.lock(drawable))
                    for (const auto& surface : surfaces)
                        if (surface.drawable == drawable)
                        {
                            cached.part = surface.part;
                            cached.ordinal = surface.ordinal;
                            cached.model = surface.model;
                            cached.fingerprint = surface.fingerprint;
                            break;
                        }
                cached.sourceDrawable = nullptr;
            }
        }

        // Prepare replacements before removing the old overlays. In particular,
        // clearing an empty record must still remove all previously frozen blood.
        std::array<std::vector<osg::ref_ptr<osg::Geometry>>, 2> replacements;
        if (mBloodDecals)
            for (const auto& record : mBloodDecals->records)
            {
                if (record.style >= mBloodDecals->states.size() || !mBloodDecals->states[record.style])
                    continue;
                for (const auto& cached : target.bloodSources)
                {
                    if (cached.part != record.part || cached.ordinal != record.ordinal || cached.model != record.model
                        || cached.fingerprint != record.fingerprint || !cached.positions)
                        continue;
                    // The original head may now be detached or animated elsewhere.
                    // Only the same cut-frame vertices and inverse matrices that
                    // built target.lower/upper can reconstruct its frozen surface.
                    osg::ref_ptr<osg::Geometry> geometry = new osg::Geometry;
                    geometry->setVertexArray(cached.positions);
                    SceneUtil::SurfaceDecals::PatchData patch{ record.anchors, mBloodDecals->states[record.style],
                        Mask_Effect };
                    // Match addPatch's canonical weights, including records read
                    // from a save rather than an existing live SurfaceDecals.
                    for (auto& anchor : patch.vertices)
                        anchor.weights /= anchor.weights.x() + anchor.weights.y() + anchor.weights.z();
                    osg::ref_ptr<osg::Vec3Array> positions = new osg::Vec3Array(patch.vertices.size()),
                                                 normals = new osg::Vec3Array(patch.vertices.size());
                    if (!SceneUtil::SurfaceDecals::evaluate(*geometry, patch, *positions, *normals))
                        continue;
                    std::vector<DecapitationGeometry::Vertex> low, high;
                    for (std::size_t i = 0; i < positions->size(); i += 3)
                    {
                        std::vector<DecapitationGeometry::Vertex> triangle;
                        for (unsigned j = 0; j < 3; ++j)
                            triangle.push_back({ (*positions)[i + j], (*normals)[i + j], patch.vertices[i + j].uv,
                                osg::Vec4f(1, 1, 1, 1) });
                        if (cached.wholeHead)
                            DecapitationGeometry::appendPolygon(high, triangle);
                        else
                        {
                            const auto distance = [&](const osg::Vec3f& pos) {
                                return (pos - target.diagnosticCenter) * target.diagnosticNormal;
                            };
                            DecapitationGeometry::appendPolygon(
                                high, DecapitationGeometry::clipPolygon(triangle, distance, true));
                            DecapitationGeometry::appendPolygon(
                                low, DecapitationGeometry::clipPolygon(triangle, distance, false));
                        }
                    }
                    for (unsigned side = 0; side < 2; ++side)
                    {
                        auto& vertices = side ? high : low;
                        if (vertices.empty())
                            continue;
                        auto frozen = DecapitationGeometry::makeGeometry(
                            vertices, patch.stateSet, side ? target.bloodHeadInverse : target.bloodNeckInverse);
                        frozen->setNodeMask(Mask_Effect);
                        frozen->setName("Frozen NPC blood decal");
                        replacements[side].push_back(frozen);
                    }
                    break;
                }
            }
        for (unsigned side = 0; side < 2; ++side)
        {
            auto* group = side ? target.upper.get() : target.lower.get();
            for (unsigned i = group->getNumChildren(); i-- > 0;)
                if (group->getChild(i)->getName() == "Frozen NPC blood decal")
                    group->removeChild(i);
            for (const auto& geometry : replacements[side])
                group->addChild(geometry);
        }
    }

    namespace
    {
        // The equipped right-hand melee weapon: its instance key and type.
        struct HeldWeapon
        {
            MWWorld::Ptr item;
            int type = -1;
        };

        HeldWeapon heldMeleeWeapon(const MWWorld::Ptr& actor, bool left)
        {
            HeldWeapon result;
            if (actor.isEmpty() || !actor.getClass().hasInventoryStore(actor))
                return result;
            auto& inventory = actor.getClass().getInventoryStore(actor);
            const auto it = inventory.getSlot(
                left ? MWWorld::InventoryStore::Slot_CarriedLeft : MWWorld::InventoryStore::Slot_CarriedRight);
            if (it == inventory.end() || it->getType() != ESM::Weapon::sRecordId)
                return result;
            const int type = it->get<ESM::Weapon>()->mBase->mData.mType;
            if (type < ESM::Weapon::ShortBladeOneHand || type > ESM::Weapon::AxeTwoHand)
                return result;
            result.item = *it;
            result.type = type;
            return result;
        }

        bool bluntHead(int type)
        {
            return type == ESM::Weapon::BluntOneHand || type == ESM::Weapon::BluntTwoClose
                || type == ESM::Weapon::BluntTwoWide || type == ESM::Weapon::AxeOneHand
                || type == ESM::Weapon::AxeTwoHand;
        }

        // Where the blade is: the attached model extends along its local +Y.
        struct BladeFrame
        {
            osg::Vec3f hilt, axis;
            float length = 0;
        };

        std::optional<BladeFrame> bladeFrame(osg::Node* part, const std::vector<osg::ref_ptr<osg::Geometry>>& posed)
        {
            if (!part || part->getParentalNodePaths().empty())
                return std::nullopt;
            const osg::Matrixf matrix = osg::computeLocalToWorld(part->getParentalNodePaths().front());
            BladeFrame frame;
            frame.hilt = matrix.getTrans();
            frame.axis = osg::Matrixf::transform3x3(osg::Vec3f(0, 1, 0), matrix);
            if (frame.axis.normalize() < 1e-4f)
                return std::nullopt;
            for (const auto& geometry : posed)
                if (const auto* vertices = dynamic_cast<const osg::Vec3Array*>(geometry->getVertexArray()))
                    for (const auto& v : *vertices)
                        frame.length = std::max(frame.length, (v - frame.hilt) * frame.axis);
            if (!std::isfinite(frame.length) || frame.length < 4.f || frame.length > 400.f)
                return std::nullopt;
            return frame;
        }

        struct WipeSurface
        {
            std::string model;
            unsigned ordinal;
            std::uint64_t fingerprint;
            std::vector<osg::Vec3f> vertices;
            std::vector<SceneUtil::DecalProjector::Triangle> triangles;
        };

        struct WipeMark
        {
            BrutalBlood::WeaponMark mark;
            std::vector<float> distances;
            float minimum = 0.f, maximum = 0.f;
        };

        struct SwordWipe
        {
            ESM::RefNum item;
            osg::observer_ptr<osg::Node> part;
            std::vector<WipeSurface> surfaces;
            std::vector<WipeMark> original;
            unsigned revision = 0;
            float length = 0.f;
            float time = -1.f;
            float cleanedTo = -std::numeric_limits<float>::infinity();

            void captureMarks(const BrutalBlood::ItemBlood& blood)
            {
                original.clear();
                revision = blood.revision;
                cleanedTo = -std::numeric_limits<float>::infinity();
                for (const auto& mark : blood.marks)
                {
                    auto& sample = original.emplace_back(WipeMark{ mark, {} });
                    for (const auto& surface : surfaces)
                    {
                        if (surface.model != mark.model || surface.ordinal != mark.ordinal
                            || surface.fingerprint != mark.fingerprint)
                            continue;
                        sample.distances.reserve(mark.anchors->size());
                        for (const auto& anchor : *mark.anchors)
                        {
                            if (std::ranges::any_of(anchor.indices,
                                    [&](unsigned index) { return index >= surface.vertices.size(); }))
                            {
                                sample.distances.clear();
                                break;
                            }
                            float y = 0.f;
                            for (unsigned i = 0; i < 3; ++i)
                                y += surface.vertices[anchor.indices[i]].y() * anchor.weights[i];
                            sample.distances.push_back(y);
                        }
                        if (!sample.distances.empty())
                        {
                            const auto [minimum, maximum] = std::ranges::minmax(sample.distances);
                            sample.minimum = minimum;
                            sample.maximum = maximum;
                        }
                        break;
                    }
                }
            }

            osg::Vec3f bladeCentre(float position) const
            {
                // The mesh is usually straight, but a saber's cross-section can
                // be offset sideways. Follow its middle at the hand's position.
                const float y = std::min(position, length - .04f);
                osg::Vec3f low, high;
                bool found = false;
                for (const auto& surface : surfaces)
                    for (const auto& triangle : surface.triangles)
                        for (unsigned i = 0; i < 3; ++i)
                        {
                            const auto& a = surface.vertices[triangle[i]];
                            const auto& b = surface.vertices[triangle[(i + 1) % 3]];
                            if ((a.y() <= y) == (b.y() <= y))
                                continue;
                            const osg::Vec3f point = a + (b - a) * ((y - a.y()) / (b.y() - a.y()));
                            if (!found)
                                low = high = point;
                            found = true;
                            for (const unsigned axis : { 0u, 2u })
                            {
                                low[axis] = std::min(low[axis], point[axis]);
                                high[axis] = std::max(high[axis], point[axis]);
                            }
                        }
                return { (low.x() + high.x()) * .5f, position, (low.z() + high.z()) * .5f };
            }
        };

        Anchors clipWipedMark(const WipeMark& original, float cleanTo, bool trimEdges = true)
        {
            Anchors result;
            const auto& anchors = *original.mark.anchors;
            for (std::size_t base = 0; base < anchors.size(); base += 3)
            {
                std::array<SceneUtil::SurfaceDecals::Anchor, 4> polygon;
                std::size_t count = 0;
                const auto& y = original.distances;
                if (!trimEdges && std::min({ y[base], y[base + 1], y[base + 2] }) < cleanTo)
                    continue;
                for (unsigned k = 0; k < 3; ++k)
                {
                    const auto a = base + k, b = base + (k + 1) % 3;
                    const float da = y[a] - cleanTo, db = y[b] - cleanTo;
                    if (da >= 0.f)
                        polygon[count++] = anchors[a];
                    if ((da < 0.f && db > 0.f) || (da > 0.f && db < 0.f))
                    {
                        const float fraction = da / (da - db);
                        polygon[count++] = { anchors[a].indices,
                            anchors[a].weights + (anchors[b].weights - anchors[a].weights) * fraction,
                            anchors[a].uv + (anchors[b].uv - anchors[a].uv) * fraction };
                    }
                }
                for (std::size_t i = 1; i + 1 < count; ++i)
                    result.insert(result.end(), { polygon[0], polygon[i], polygon[i + 1] });
            }
            // Keep the save and draw limits on pathological, heavily tessellated patches.
            if (trimEdges && result.size() > BrutalBlood::MaxWeaponMarkVertices)
                return clipWipedMark(original, cleanTo, false);
            return result;
        }

        float easeWipe(float amount)
        {
            amount = std::clamp(amount, 0.f, 1.f);
            return amount * amount * (3.f - 2.f * amount);
        }
    }

    struct NpcAnimation::WeaponBloodView
    {
        struct Mark
        {
            std::shared_ptr<const Anchors> anchors;
            BloodPatch patch;
            osg::StateSet* material = nullptr;
        };
        struct Hand
        {
            ESM::RefNum key;
            unsigned revision = 0;
            osg::observer_ptr<osg::Node> part;
            std::vector<Mark> marks;
            double appearanceTime = 0.;
            std::size_t attached = 0;
            float nextDrip = 0;

            void clear()
            {
                for (auto& mark : marks)
                    mark.patch.remove();
                marks.clear();
                attached = 0;
            }
        };
        std::array<Hand, 2> hands;
        std::optional<SwordWipe> wipe;
        std::uint64_t drips = 0, rebuilds = 0, rejected = 0, materialUpdates = 0;
        double lastLength = 0, lastThickness = 0, lastWidth = 0;
        std::size_t lastSides = 0, lastMarks = 0;

        void sync(Hand& view, BrutalBlood& store, const BrutalBlood::ItemBlood* blood, ESM::RefNum key,
            int partIndex, osg::Node* part, const std::string& model, osg::Node* actor)
        {
            if (!part || !blood || blood->marks.empty())
            {
                if (!view.marks.empty())
                {
                    view.clear();
                    ++rebuilds;
                }
                return;
            }

            const bool sameWeapon = view.part.get() == part && view.key == key;
            const bool changed = !sameWeapon || view.marks.empty() || view.revision != blood->revision;
            const double appearanceTime = store.now() + blood->wash;
            if (!changed && view.appearanceTime == appearanceTime)
                return;
            const auto material = [&](const BrutalBlood::WeaponMark& mark) {
                const double age = appearanceTime - mark.hour;
                return store.weaponState(mark.style, BrutalBlood::markFade(age), BrutalBlood::markDry(age));
            };

            if (changed)
            {
                if (!sameWeapon)
                    view.clear();
                else
                    std::erase_if(view.marks, [&](Mark& visual) {
                        if (std::ranges::find(blood->marks, visual.anchors, &BrutalBlood::WeaponMark::anchors)
                            != blood->marks.end())
                            return false;
                        visual.patch.remove();
                        return true;
                    });

                std::optional<std::vector<Surface>> surfaces;
                std::vector<Mark> ordered;
                ordered.reserve(blood->marks.size());
                for (const auto& mark : blood->marks)
                {
                    // Unchanged smears keep their local mesh, including during a partial wipe.
                    const auto existing = std::ranges::find(view.marks, mark.anchors, &Mark::anchors);
                    if (existing != view.marks.end())
                    {
                        ordered.push_back(std::move(*existing));
                        continue;
                    }
                    auto& visual = ordered.emplace_back(Mark{ mark.anchors, {}, material(mark) });
                    if (!visual.material)
                        continue;
                    if (!surfaces)
                    {
                        SurfacesVisitor visitor(surfaces.emplace(), partIndex, model);
                        visitor.collect(part, actor);
                    }
                    for (auto& surface : *surfaces)
                        if (surface.ordinal == mark.ordinal && surface.model == mark.model
                            && surface.fingerprint == mark.fingerprint)
                        {
                            visual.patch.attach(surface, *mark.anchors, visual.material);
                            break;
                        }
                }
                view.marks = std::move(ordered);
                view.part = part;
                view.key = key;
                view.revision = blood->revision;
                ++rebuilds;
            }

            view.attached = 0;
            for (std::size_t i = 0; i < view.marks.size(); ++i)
            {
                auto& visual = view.marks[i];
                auto* decals = visual.patch.decals.get();
                if (!decals)
                    continue;
                auto* state = material(blood->marks[i]);
                if (state != visual.material)
                {
                    decals->setPatchState(visual.patch.id, state);
                    visual.material = state;
                    ++materialUpdates;
                }
                view.attached += visual.material != nullptr;
            }
            view.appearanceTime = appearanceTime;
        }
    };

    std::optional<std::pair<osg::Vec3f, osg::Vec3f>> NpcAnimation::getWeaponBladeForBlood(bool left) const
    {
        // A left-hand weapon is attached as the shield part.
        const int partIndex = left ? ESM::PRT_Shield : ESM::PRT_Weapon;
        if (!mObjectParts[partIndex])
            return std::nullopt;
        osg::Node* part = mObjectParts[partIndex]->getNode();
        std::vector<Surface> surfaces;
        SurfacesVisitor visitor(surfaces, partIndex, mPartModels[partIndex]);
        visitor.collect(part, mPtr.getRefData().getBaseNode());
        std::vector<osg::ref_ptr<osg::Geometry>> posedSurfaces;
        const auto bones = [](std::string_view) { return std::optional<osg::Matrixf>(); };
        for (auto& surface : surfaces)
            if (auto geometry = posed(surface, bones))
                posedSurfaces.push_back(geometry);
        const auto frame = bladeFrame(part, posedSurfaces);
        if (!frame)
            return std::nullopt;
        return std::make_pair(frame->hilt, frame->hilt + frame->axis * frame->length);
    }

    bool NpcAnimation::addWeaponBlood(const osg::Vec3f& contact, std::string_view kind, bool power, bool left)
    {
        auto* store = BrutalBlood::get();
        const int partIndex = left ? ESM::PRT_Shield : ESM::PRT_Weapon;
        if (!store || !Settings::game().mBrutalEnabled || !Settings::game().mBrutalWeaponBlood || mBodyAwareness
            || !mObjectParts[partIndex] || !store->hasStyle(BrutalBlood::Style_Slash))
            return false;
        const auto held = heldMeleeWeapon(mPtr, left);
        if (held.item.isEmpty())
            return false;
        if (!mWeaponBloodView)
            mWeaponBloodView = std::make_shared<WeaponBloodView>();
        auto& view = *mWeaponBloodView;
        try
        {
            osg::Node* part = mObjectParts[partIndex]->getNode();
            std::vector<Surface> surfaces;
            SurfacesVisitor visitor(surfaces, partIndex, mPartModels[partIndex]);
            visitor.collect(part, mPtr.getRefData().getBaseNode());
            const auto bones = [](std::string_view) { return std::optional<osg::Matrixf>(); };
            std::vector<osg::ref_ptr<osg::Geometry>> posedSurfaces(surfaces.size());
            std::vector<osg::ref_ptr<osg::Geometry>> valid;
            for (std::size_t i = 0; i < surfaces.size(); ++i)
                if ((posedSurfaces[i] = posed(surfaces[i], bones)))
                    valid.push_back(posedSurfaces[i]);
            const auto frame = bladeFrame(part, valid);
            if (!frame)
            {
                ++view.rejected;
                return false;
            }
            const float length = frame->length;
            view.lastLength = length;
            const bool thrust = kind == "thrust";
            const bool head = bluntHead(held.type);
            const bool polearm = held.type == ESM::Weapon::SpearTwoWide;
            const bool cuttingBlade = !head && !polearm && !thrust;
            const float along = cuttingBlade
                ? std::clamp((power ? .9f : .78f) * length, 5.f, 72.f)
                : std::clamp((head ? .38f : .34f) * length, 5.f, 26.f) * (power ? 1.25f : 1.f);
            float t = (contact - frame->hilt) * frame->axis;
            if (thrust || polearm)
                t = length * (.84f + .1f * store->random01());
            else if (head)
                t = length * (.8f + .1f * store->random01());
            else if (!std::isfinite(t))
                t = length * .65f;
            t += (store->random01() - .5f) * .1f * length;
            if (cuttingBlade)
                t = std::clamp(t, .5f * along + .05f * length, .84f * length);
            else
                t = std::clamp(t, .3f * length, .93f * length);
            const osg::Vec3f centre = frame->hilt + frame->axis * t;

            // The blade's flat faces: the thinnest direction across the blade here.
            osg::Vec3f e1 = frame->axis ^ osg::Vec3f(0, 0, 1);
            if (e1.normalize() < 1e-3f)
                e1 = frame->axis ^ osg::Vec3f(1, 0, 0), e1.normalize();
            osg::Vec3f e2 = frame->axis ^ e1;
            e2.normalize();
            double c11 = 0, c12 = 0, c22 = 0;
            float extentA = 0, extentB = 0;
            std::size_t count = 0;
            osg::Vec3f mean;
            // Cross-section of the blade here: where triangle edges cross the plane
            // across the blade. Low-poly blades have no vertices mid-length.
            std::vector<osg::Vec3f> nearby;
            for (std::size_t i = 0; i < surfaces.size(); ++i)
            {
                if (!posedSurfaces[i])
                    continue;
                const auto* vertices = dynamic_cast<const osg::Vec3Array*>(posedSurfaces[i]->getVertexArray());
                if (!vertices)
                    continue;
                for (const auto& triangle : surfaces[i].triangles)
                    for (unsigned e = 0; e < 3; ++e)
                    {
                        if (triangle[e] >= vertices->size() || triangle[(e + 1) % 3] >= vertices->size())
                            continue;
                        const osg::Vec3f& a = (*vertices)[triangle[e]];
                        const osg::Vec3f& b = (*vertices)[triangle[(e + 1) % 3]];
                        const float da = (a - centre) * frame->axis, db = (b - centre) * frame->axis;
                        if ((da < 0) != (db < 0) && std::abs(da - db) > 1e-6f)
                            nearby.push_back(a + (b - a) * (da / (da - db)));
                    }
            }
            for (const auto& v : nearby)
                mean += v;
            count = nearby.size();
            if (count)
                mean /= float(count);
            else
                mean = centre;
            for (const auto& v : nearby)
            {
                const double a = (v - mean) * e1, b = (v - mean) * e2;
                c11 += a * a;
                c12 += a * b;
                c22 += b * b;
            }
            // Smallest-variance direction of the 2x2 covariance.
            const double trace = c11 + c22, det = c11 * c22 - c12 * c12;
            const double small = trace / 2 - std::sqrt(std::max(0., trace * trace / 4 - det));
            osg::Vec3f flat = std::abs(c12) > 1e-9 ? e1 * float(c12) + e2 * float(small - c11)
                                                    : (c11 <= c22 ? e1 : e2);
            if (flat.normalize() < 1e-4f)
                flat = e1;
            osg::Vec3f across = frame->axis ^ flat;
            across.normalize();
            for (const auto& v : nearby)
            {
                extentA = std::max(extentA, std::abs((v - mean) * flat));
                extentB = std::max(extentB, std::abs((v - mean) * across));
            }
            const osg::Vec3f middle = mean - frame->axis * ((mean - centre) * frame->axis);

            unsigned style = BrutalBlood::Style_Slash;
            if (thrust || head)
            {
                style = BrutalBlood::Style_World + store->random() % 6;
                if (!store->hasStyle(style))
                    style = store->hasStyle(BrutalBlood::Style_Small) ? BrutalBlood::Style_Small
                                                                       : BrutalBlood::Style_Slash;
            }
            const float height = std::clamp(2.4f * extentB, 3.f, head ? 22.f : 14.f);

            // Both flat faces, and for blades both edges too: a blade seen edge-on,
            // as a first-person sword mostly is, still shows the blood.
            struct Side
            {
                osg::Vec3f outward;
                float offset, height, depth;
            };
            std::vector<Side> sides{ { flat, extentA, height, std::max(2.f, 2.f * extentA + 1.f) },
                { -flat, extentA, height, std::max(2.f, 2.f * extentA + 1.f) } };
            if (!head && extentB > 2.f * extentA)
            {
                const float edge = std::clamp(2.5f * extentA + 1.f, 1.5f, 5.f);
                sides.push_back({ across, extentB, edge, std::max(2.f, .5f * extentB) });
                sides.push_back({ -across, extentB, edge, std::max(2.f, .5f * extentB) });
            }
            view.lastThickness = extentA;
            view.lastWidth = extentB;
            view.lastSides = sides.size();
            // Weld the whole weapon for traversal so a material seam can lie
            // inside a single smear. Keep the source mapping for saved anchors.
            std::vector<osg::Vec3d> positions;
            std::vector<SceneUtil::DecalProjector::Triangle> triangles;
            std::vector<std::size_t> surfaceOfVertex;
            std::vector<unsigned> vertexOffsets(surfaces.size());
            for (std::size_t i = 0; i < surfaces.size(); ++i)
            {
                if (!posedSurfaces[i])
                    continue;
                const auto* vertices = dynamic_cast<const osg::Vec3Array*>(posedSurfaces[i]->getVertexArray());
                if (!vertices)
                    continue;
                const auto offset = static_cast<unsigned>(positions.size());
                vertexOffsets[i] = offset;
                positions.insert(positions.end(), vertices->begin(), vertices->end());
                surfaceOfVertex.resize(positions.size(), i);
                for (const auto& triangle : surfaces[i].triangles)
                    if (std::ranges::all_of(triangle, [&](unsigned index) { return index < vertices->size(); }))
                        triangles.push_back({ triangle[0] + offset, triangle[1] + offset, triangle[2] + offset });
            }
            const double hour = store->now();
            std::vector<BrutalBlood::WeaponMark> marks;
            for (const auto& side : sides)
            {
                const osg::Vec3f outward = side.outward;
                auto projection = SceneUtil::DecalProjector::project(positions, triangles,
                    osg::Vec3d(middle + outward * (side.offset + 1.f)), osg::Vec3d(outward),
                    osg::Vec3d(frame->axis), along, side.height, side.depth, side.offset + 6.0);
                std::vector<std::vector<SceneUtil::SurfaceDecals::Anchor>> anchors(surfaces.size());
                for (auto& vertex : projection.vertices)
                {
                    const auto index = surfaceOfVertex[vertex.indices[0]];
                    for (auto& sourceIndex : vertex.indices)
                        sourceIndex -= vertexOffsets[index];
                    anchors[index].push_back(vertex);
                }
                for (std::size_t i = 0; i < surfaces.size(); ++i)
                {
                    if (anchors[i].empty())
                        continue;
                    const auto& surface = surfaces[i];
                    marks.push_back({ style, surface.model, surface.ordinal, surface.fingerprint,
                        std::make_shared<const Anchors>(std::move(anchors[i])), hour });
                }
            }
            view.lastMarks = marks.size();
            if (marks.empty() || marks.size() > BrutalBlood::MaxWeaponMarks)
            {
                ++view.rejected;
                return false;
            }
            // Generated reference numbers are saved with the item, also in inventories.
            if (!held.item.getCellRef().getRefNum().isSet())
                MWBase::Environment::get().getWorldModel()->registerPtr(held.item);
            const ESM::RefNum key = held.item.getCellRef().getRefNum();
            if (!key.isSet())
                return false;
            auto& blood = store->createItem(key);
            // Retire the oldest hit as a group to keep its seam-spanning
            // patches together when multiple hits cover the same surface.
            while (blood.marks.size() + marks.size() > BrutalBlood::MaxWeaponMarks)
            {
                const double oldest = blood.marks.front().hour;
                std::erase_if(blood.marks, [oldest](const auto& mark) { return mark.hour == oldest; });
            }
            for (auto& mark : marks)
                blood.marks.push_back(std::move(mark));
            blood.revision = store->nextRevision();
            blood.dripUntil = store->realTime() + (power ? 4.f : 2.5f);
            store->noteWeaponAdd();
            return true;
        }
        catch (const std::exception& error)
        {
            ++view.rejected;
            Log(Debug::Warning) << "Weapon blood projection rejected: " << error.what();
            return false;
        }
    }

    void NpcAnimation::fitSwordWipePose()
    {
        constexpr std::string_view group = "brutal_sword_wipe";
        auto* store = BrutalBlood::get();
        if (!store || !mWeaponBloodView)
            return;
        auto& view = *mWeaponBloodView;
        if (!mSwordWipeItem.isSet() || !isPlaying(group) || mViewMode != VM_FirstPerson || useFullBodyFirstPerson()
            || mBodyAwareness || mPtr.getClass().getCreatureStats(mPtr).isDead())
        {
            view.wipe.reset();
            return;
        }

        const float align = getTextKeyTime("brutal_sword_wipe: align");
        const float start = getTextKeyTime("brutal_sword_wipe: contact start");
        const float end = getTextKeyTime("brutal_sword_wipe: contact end");
        const float release = getTextKeyTime("brutal_sword_wipe: release");
        const float t = getCurrentTime(group);
        if (align < 0.f || start <= align || end <= start || release <= end || t < align || t >= release)
        {
            view.wipe.reset();
            return;
        }
        auto* part = getWeaponNode();
        const auto held = heldMeleeWeapon(mPtr, false);
        if (!part || held.item.isEmpty() || held.item.getCellRef().getRefNum() != mSwordWipeItem)
            return;
        const auto key = mSwordWipeItem;
        auto* blood = store->item(key);
        const auto paths = part->getParentalNodePaths();
        if (paths.empty())
            return;
        const osg::Matrixf weapon = osg::computeLocalToWorld(paths.front());
        if (!view.wipe || view.wipe->part.get() != part || view.wipe->item != key || t < view.wipe->time)
        {
            if (!blood || blood->marks.empty() || t > end)
                return;
            auto& wipe = view.wipe.emplace();
            wipe.part = part;
            wipe.item = key;
            const auto inverse = osg::Matrixf::inverse(weapon);
            std::vector<Surface> surfaces;
            SurfacesVisitor visitor(surfaces, ESM::PRT_Weapon, mPartModels[ESM::PRT_Weapon]);
            visitor.collect(part, mPtr.getRefData().getBaseNode());
            for (const auto& surface : surfaces)
            {
                const auto geometry = posed(surface, [this](std::string_view bone) { return getBoneWorldMatrix(bone); });
                const auto* vertices = geometry ? dynamic_cast<osg::Vec3Array*>(geometry->getVertexArray()) : nullptr;
                if (!vertices)
                    continue;
                auto& sample = wipe.surfaces.emplace_back(WipeSurface{ surface.model, surface.ordinal, surface.fingerprint,
                    {}, {} });
                sample.vertices.reserve(vertices->size());
                for (const auto& vertex : *vertices)
                {
                    const osg::Vec3f point = vertex * inverse;
                    sample.vertices.push_back(point);
                    wipe.length = std::max(wipe.length, point.y());
                }
                for (const auto& triangle : surface.triangles)
                    if (std::ranges::all_of(triangle, [&](unsigned index) { return index < vertices->size(); }))
                        sample.triangles.push_back(triangle);
            }
            wipe.captureMarks(*blood);
        }
        auto& wipe = *view.wipe;
        wipe.time = t;
        if (wipe.length < 8.f || wipe.length > 250.f)
            return;
        // Match the reference sweep baked into the imported clip. Only the
        // difference between that reference and this mesh moves the left arm.
        constexpr float referenceLength = 60.f;
        constexpr float bladeStart = .135f;
        const float fraction = std::lerp(bladeStart, 1.f, easeWipe((t - start) / (end - start)));
        const float position = fraction * wipe.length;
        {
            const float weight = easeWipe((t - align) / (start - align)) * easeWipe((release - t) / (release - end));
            const auto node = getNodeMap().find("Bip01 L Clavicle");
            if (node != getNodeMap().end())
            {
                const auto clavicle = node->second.get();
                const auto parentPaths = clavicle->getParentalNodePaths();
                if (!parentPaths.empty())
                {
                    auto parent = parentPaths.front();
                    parent.pop_back();
                    const osg::Vec3f delta = wipe.bladeCentre(position) - osg::Vec3f(0.f, fraction * referenceLength, 0.f);
                    auto matrix = clavicle->getMatrix();
                    mSwordWipeSourcePose = { clavicle, matrix };
                    const auto local = weapon * osg::Matrixf::inverse(osg::computeLocalToWorld(parent));
                    matrix.setTrans(matrix.getTrans() + osg::Matrixf::transform3x3(delta * weight, local));
                    clavicle->setMatrix(matrix);
                    if (mSkeleton)
                        mSkeleton->markDirty();
                }
            }
        }
        if (t < start || !blood || blood->marks.empty())
            return;
        if (blood->revision != wipe.revision)
            wipe.captureMarks(*blood);
        const float cleanTo = t >= end ? wipe.length + .05f : position;
        if (cleanTo <= wipe.cleanedTo)
            return;
        const bool changed = std::ranges::any_of(wipe.original, [&](const WipeMark& mark) {
            return !mark.distances.empty() && mark.minimum < cleanTo && mark.maximum >= wipe.cleanedTo;
        });
        wipe.cleanedTo = cleanTo;
        if (!changed)
            return;
        std::vector<BrutalBlood::WeaponMark> remaining;
        remaining.reserve(wipe.original.size());
        for (const auto& original : wipe.original)
        {
            if (original.distances.empty() || cleanTo <= original.minimum)
                remaining.push_back(original.mark);
            else if (cleanTo < original.maximum)
            {
                auto anchors = clipWipedMark(original, cleanTo);
                if (!anchors.empty())
                {
                    auto& mark = remaining.emplace_back(original.mark);
                    mark.anchors = std::make_shared<const Anchors>(std::move(anchors));
                }
            }
        }
        // Write the clipped anchors to the item itself. If movement, combat or
        // saving interrupts the animation, the uncleaned side remains intact.
        blood->marks = std::move(remaining);
        blood->revision = wipe.revision = store->nextRevision();
        if (blood->marks.empty())
        {
            blood->wash = 0.;
            blood->dripUntil = 0.f;
        }
        view.sync(view.hands[0], *store, blood, key, ESM::PRT_Weapon, part, mPartModels[ESM::PRT_Weapon],
            mPtr.getRefData().getBaseNode());
    }

    void NpcAnimation::updateWeaponBlood(float dt)
    {
        auto* store = BrutalBlood::get();
        // The inventory preview shares the player's item references, but its
        // attachments belong to a separate scene and have no world blade pose.
        if (!store || mBodyAwareness || mInsert.get() != mPtr.getRefData().getBaseNode())
            return;
        const bool enabled = Settings::game().mBrutalEnabled && Settings::game().mBrutalWeaponBlood;
        const auto world = MWBase::Environment::get().getWorld();
        for (const bool left : { false, true })
        {
            const int partIndex = left ? ESM::PRT_Shield : ESM::PRT_Weapon;
            const auto held = heldMeleeWeapon(mPtr, left);
            const ESM::RefNum key = held.item.isEmpty() ? ESM::RefNum{} : held.item.getCellRef().getRefNum();
            auto* blood = enabled ? store->item(key) : nullptr;

            // Water and rain. A sheathed weapon gets as wet as a drawn one.
            if (blood && !blood->marks.empty() && mPtr.isInCell())
            {
                if (world->isSwimming(mPtr) || world->isUnderwater(MWWorld::ConstPtr(mPtr), .75f))
                    store->clearItem(key);
                else
                {
                    const auto* cell = mPtr.getCell()->getCell();
                    const int weather = world->getCurrentWeatherScriptId();
                    if ((cell->isExterior() || cell->isQuasiExterior()) && (weather == 4 || weather == 5))
                        // About an hour of rain washes a blade clean.
                        store->addWear(key, store->elapsedHours() * BrutalBlood::WeaponLifetimeHours);
                }
            }

            osg::Node* part = mObjectParts[partIndex] ? mObjectParts[partIndex]->getNode().get() : nullptr;
            if (!mWeaponBloodView)
            {
                if (!blood || !part)
                    continue;
                mWeaponBloodView = std::make_shared<WeaponBloodView>();
            }
            auto& view = mWeaponBloodView->hands[left];
            if (!part)
            {
                view.clear();
                view = WeaponBloodView::Hand{};
                continue;
            }

            mWeaponBloodView->sync(view, *store, blood, key, partIndex, part, mPartModels[partIndex],
                mPtr.getRefData().getBaseNode());

            // Fresh blood runs off the blade for a few seconds.
            if (blood && blood->dripUntil > store->realTime() && view.attached
                && Settings::game().mBrutalBloodEffects && store->effects())
            {
                view.nextDrip -= dt;
                if (view.nextDrip <= 0.f)
                {
                    view.nextDrip = .2f + .35f * store->random01();
                    if (const auto blade = getWeaponBladeForBlood(left))
                    {
                        const osg::Vec3f tip = blade->first + (blade->second - blade->first) * .95f;
                        store->effects()->queueCutBlood("cutdrop", tip, osg::Vec3f(0, 0, -1), .9f);
                        ++mWeaponBloodView->drips;
                        // Some drops reach the floor and stay there.
                        store->drip(mPtr, tip);
                    }
                }
            }
        }
    }

    std::map<std::string, double> NpcAnimation::getWeaponBloodStats() const
    {
        std::map<std::string, double> result{ { "attached", 0 }, { "rebuilds", 0 }, { "drips", 0 },
            { "rejected", 0 }, { "marks", 0 }, { "length", 0 }, { "uvArea", 0 },
            { "hasPart", mObjectParts[ESM::PRT_Weapon] ? 1. : 0. } };
        if (mWeaponBloodView)
        {
            const auto& view = *mWeaponBloodView;
            result["attached"] = double(view.hands[0].attached);
            result["attachedLeft"] = double(view.hands[1].attached);
            result["rebuilds"] = double(view.rebuilds);
            result["materialUpdates"] = double(view.materialUpdates);
            std::set<const SceneUtil::SurfaceDecals*> surfaces;
            std::size_t evaluations = 0;
            for (const auto& hand : view.hands)
                for (const auto& mark : hand.marks)
                    if (const auto* decals = mark.patch.decals.get(); decals && surfaces.insert(decals).second)
                        evaluations += decals->evaluationCount();
            result["evaluations"] = double(evaluations);
            result["drips"] = double(view.drips);
            result["rejected"] = double(view.rejected);
            result["length"] = view.lastLength;
            result["halfThickness"] = view.lastThickness;
            result["halfWidth"] = view.lastWidth;
            result["sides"] = double(view.lastSides);
            result["sideMarks"] = double(view.lastMarks);
        }
        if (auto* store = BrutalBlood::get())
        {
            const auto held = heldMeleeWeapon(mPtr, false);
            if (!held.item.isEmpty())
                if (const auto* blood = store->item(held.item.getCellRef().getRefNum()))
                {
                    result["marks"] = double(blood->marks.size());
                    result["wash"] = blood->wash;
                    double newest = -1e9;
                    double area = 0.;
                    for (const auto& mark : blood->marks)
                    {
                        newest = std::max(newest, mark.hour);
                        const auto& anchors = *mark.anchors;
                        for (std::size_t i = 0; i < anchors.size(); i += 3)
                        {
                            const auto a = anchors[i + 1].uv - anchors[i].uv;
                            const auto b = anchors[i + 2].uv - anchors[i].uv;
                            area += .5 * std::abs(a.x() * b.y() - a.y() * b.x());
                        }
                    }
                    result["uvArea"] = area;
                    if (!blood->marks.empty())
                        result["age"] = store->now() - newest + blood->wash;
                }
        }
        return result;
    }
}
