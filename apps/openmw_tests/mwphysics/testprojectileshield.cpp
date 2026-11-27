#include <gtest/gtest.h>

#include <apps/openmw/mwphysics/collisiontype.hpp>
#include <apps/openmw/mwphysics/projectile.hpp>

namespace
{
    void addPanel(MWPhysics::ProjectileShield& shield, float left, float right)
    {
        const osg::Vec3f lowerLeft(left, 0.f, -20.f);
        const osg::Vec3f lowerRight(right, 0.f, -20.f);
        const osg::Vec3f upperLeft(left, 0.f, 20.f);
        const osg::Vec3f upperRight(right, 0.f, 20.f);
        shield.mTriangles.push_back({ lowerLeft, lowerRight, upperRight });
        shield.mTriangles.push_back({ lowerLeft, upperRight, upperLeft });
        shield.mBounds.expandBy(lowerLeft);
        shield.mBounds.expandBy(upperRight);
    }

    TEST(ProjectileShield, InterceptsAnExposedRimOnlyWhenTheShieldExtendsToTheShot)
    {
        MWPhysics::ProjectileShield large{};
        large.mForward.set(0.f, -1.f, 0.f);
        addPanel(large, -30.f, 30.f);

        MWPhysics::ProjectileShield small{};
        small.mForward = large.mForward;
        addPanel(small, -16.f, 16.f);

        // This trajectory is already outside the defender's body collider at x = +/- 12.
        const osg::Vec3f from(24.f, -60.f, 5.f);
        const osg::Vec3f to(24.f, 40.f, 5.f);
        const auto contact = large.intersect(from, to, 1.f);
        ASSERT_TRUE(contact);
        EXPECT_FLOAT_EQ(contact->mFraction, .6f);
        EXPECT_EQ(contact->mNormal, osg::Vec3f(0.f, -1.f, 0.f));
        EXPECT_FALSE(small.intersect(from, to, 1.f));
    }

    TEST(ProjectileShield, LeavesOpenGapsAndRespectsEarlierCoverAndFacing)
    {
        MWPhysics::ProjectileShield shield{};
        shield.mForward.set(0.f, -1.f, 0.f);
        addPanel(shield, -30.f, -6.f);
        addPanel(shield, 6.f, 30.f);

        const osg::Vec3f from(15.f, -60.f, 5.f);
        const osg::Vec3f to(15.f, 40.f, 5.f);
        EXPECT_TRUE(shield.intersect(from, to, 1.f));
        EXPECT_FALSE(shield.intersect(from, to, .5f));
        EXPECT_FALSE(shield.intersect(to, from, 1.f));
        EXPECT_FALSE(shield.intersect(osg::Vec3f(0.f, -60.f, 5.f), osg::Vec3f(0.f, 40.f, 5.f), 1.f));
    }

    TEST(ProjectileImpact, SeparatesProjectileCollisionsFromTerrain)
    {
        using namespace MWPhysics;
        EXPECT_EQ(getProjectileHitType(CollisionType_Projectile), ProjectileHitType::Projectile);
        EXPECT_EQ(getProjectileHitType(CollisionType_HeightMap), ProjectileHitType::Terrain);
        EXPECT_EQ(getProjectileHitType(CollisionType_Water), ProjectileHitType::Water);
        EXPECT_EQ(getProjectileHitType(CollisionType_World | CollisionType_Dynamic), ProjectileHitType::World);
    }
}
