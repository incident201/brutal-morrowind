#ifndef OPENMW_MWRENDER_BRUTALPARTICLEBATCH_H
#define OPENMW_MWRENDER_BRUTALPARTICLEBATCH_H

#include <algorithm>
#include <array>
#include <map>
#include <vector>
#include <osg/Billboard>
#include <osg/Camera>
#include <osg/Geometry>
#include <osg/Group>
#include <osg/observer_ptr>
#include <osgUtil/CullVisitor>

namespace MWRender
{
    // One draw per material/animation frame, with the original POINT_ROT_EYE
    // facing and quad proportions. Each camera owns two independent VBO sets.
    class BrutalParticleBatch final : public osg::Group
    {
    public:
        struct Particle
        {
            osg::Vec3f position;
            float size, alpha;
        };

        BrutalParticleBatch(osg::Billboard* source, osg::StateSet* state)
            : mSource(source), mState(state)
        {
            setCullingActive(false);
        }

        void setParticles(std::vector<Particle> particles)
        {
            mParticles = std::move(particles);
            for (auto it = mBuffers.begin(); it != mBuffers.end();)
                it = it->second.camera.valid() ? std::next(it) : mBuffers.erase(it);
        }

        void traverse(osg::NodeVisitor& visitor) override
        {
            auto* cv = visitor.asCullVisitor();
            if (!cv || mParticles.empty()) return;
            auto& view = mBuffers[cv->getCurrentCamera()];
            view.camera = cv->getCurrentCamera();
            auto& buffer = view.frames[cv->getTraversalNumber() % 2];
            if (!buffer.geometry)
            {
                buffer.geometry = new osg::Geometry;
                buffer.geometry->setUseDisplayList(false);
                buffer.geometry->setUseVertexBufferObjects(true);
                buffer.geometry->setStateSet(mState);
                buffer.positions = new osg::Vec3Array;
                buffer.uvs = new osg::Vec2Array;
                buffer.colors = new osg::Vec4Array;
                buffer.geometry->setVertexArray(buffer.positions);
                buffer.geometry->setTexCoordArray(0, buffer.uvs, osg::Array::BIND_PER_VERTEX);
                buffer.geometry->setColorArray(buffer.colors, osg::Array::BIND_PER_VERTEX);
                buffer.draw = new osg::DrawArrays(GL_TRIANGLES);
                buffer.geometry->addPrimitiveSet(buffer.draw);
            }
            buffer.positions->clear();
            buffer.uvs->clear();
            buffer.colors->clear();
            const auto& modelView = *cv->getModelViewMatrix();
            const auto inverse = osg::Matrix::inverse(modelView);
            const osg::Vec3f eye = cv->getEyeLocal();
            auto ordered = mParticles;
            std::stable_sort(ordered.begin(), ordered.end(), [&](const auto& a, const auto& b) {
                return (a.position * modelView).z() < (b.position * modelView).z();
            });
            const auto* quad = static_cast<const osg::Geometry*>(mSource->getDrawable(0));
            const auto& corners = *static_cast<const osg::Vec3Array*>(quad->getVertexArray());
            const auto& uv = *static_cast<const osg::Vec2Array*>(quad->getTexCoordArray(0));
            for (const auto& particle : ordered)
            {
                osg::Matrix facing(modelView);
                mSource->computeMatrix(facing, eye, particle.position);
                const auto toWorld = facing * inverse;
                for (const unsigned i : { 0u, 1u, 2u, 0u, 2u, 3u })
                {
                    buffer.positions->push_back((corners[i] * particle.size) * toWorld);
                    buffer.uvs->push_back(uv[i]);
                    buffer.colors->push_back(osg::Vec4(0, 0, 0, particle.alpha));
                }
            }
            buffer.positions->dirty();
            buffer.uvs->dirty();
            buffer.colors->dirty();
            buffer.draw->setCount(buffer.positions->size());
            buffer.geometry->dirtyBound();
            buffer.geometry->accept(visitor);
        }

    private:
        struct Buffer
        {
            osg::ref_ptr<osg::Geometry> geometry;
            osg::ref_ptr<osg::Vec3Array> positions;
            osg::ref_ptr<osg::Vec2Array> uvs;
            osg::ref_ptr<osg::Vec4Array> colors;
            osg::ref_ptr<osg::DrawArrays> draw;
        };
        struct View
        {
            osg::observer_ptr<osg::Camera> camera;
            std::array<Buffer, 2> frames;
        };
        osg::ref_ptr<osg::Billboard> mSource;
        osg::ref_ptr<osg::StateSet> mState;
        std::vector<Particle> mParticles;
        std::map<osg::Camera*, View> mBuffers;
    };
}
#endif
