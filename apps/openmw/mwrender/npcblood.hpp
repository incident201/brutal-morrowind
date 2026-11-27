#ifndef OPENMW_MWRENDER_NPCBLOOD_H
#define OPENMW_MWRENDER_NPCBLOOD_H
// NPC surface-blood implementation is separate from animation/physics policy.
#include <osg/Vec3f>
#include <cstdint>
#include <string>
#include <string_view>

namespace osg
{
    class Geometry;
}
namespace MWRender
{
    enum class BloodPoolOutlet : std::uint32_t
    {
        Impact,
        Neck,
        LowerWaist,
        UpperWaist,
        LeftLegStump,
        LeftLegFragment,
        RightLegStump,
        RightLegFragment,
        LeftArmStump,
        LeftArmFragment,
        RightArmStump,
        RightArmFragment,
    };

    struct CutPoolPoint
    {
        osg::Vec3f position;
        float radius;
    };

    // This exact rigid wrapper preserves the original arrays and transforms.
    // Garment coverage may accept it wherever it accepts ordinary Geometry.
    bool isRigidBloodSurface(const osg::Geometry* geometry);
    std::string mergeNpcBloodDecalData(std::string_view saved, std::string_view recent);
}
#endif
