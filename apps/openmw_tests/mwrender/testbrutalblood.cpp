#include "apps/openmw/mwrender/brutalblood.hpp"

#include <gtest/gtest.h>

namespace
{
    using MWRender::BrutalBlood;

    class BrutalBloodSaveTest : public ::testing::Test
    {
    protected:
        osg::ref_ptr<osg::Group> mRoot = new osg::Group;
        BrutalBlood mBlood{ nullptr, mRoot, nullptr, {} };

        void addMark(ESM::RefNum key, double hour, std::size_t vertices)
        {
            BrutalBlood::WeaponMark mark;
            mark.hour = hour;
            mark.model = "meshes/w/test.nif";
            mark.anchors = std::make_shared<BrutalBlood::WeaponAnchors>(vertices);
            mBlood.createItem(key).marks.push_back(std::move(mark));
        }
    };

    TEST_F(BrutalBloodSaveTest, keepsNewestWeaponMarksWithinTheLoadBudget)
    {
        for (unsigned i = 0; i < 21; ++i)
            addMark({ 1 + i % 3, 0 }, i, 3000);

        const auto saved = mBlood.save();
        ASSERT_LE(saved.size(), BrutalBlood::MaxSaveBytes);
        ASSERT_TRUE(mBlood.load(saved));
        for (unsigned item = 1; item <= 3; ++item)
        {
            const auto* blood = mBlood.item({ item, 0 });
            ASSERT_NE(blood, nullptr);
            EXPECT_EQ(blood->marks.size(), item == 1 ? 6 : 7);
            EXPECT_EQ(blood->marks.front().hour, item == 1 ? 3 : item - 1);
        }
        EXPECT_EQ(mBlood.save(), saved);
    }

    TEST_F(BrutalBloodSaveTest, recoversOversizedSavesFromThePreviousWriter)
    {
        for (unsigned i = 0; i < 11; ++i)
            addMark({ 1, 0 }, i, 3000);
        std::string legacy = mBlood.save();
        mBlood.clear();
        for (unsigned i = 0; i < 10; ++i)
            addMark({ 2, 0 }, i + 11, 3000);
        const auto second = mBlood.save();

        // BWB3 starts with a four-byte tag and a u32 item count. Both fixtures end in
        // three empty world sections (splat, shoe, and pool-source counts).
        constexpr std::size_t headerBytes = 8;
        constexpr std::size_t emptyWorldBytes = 12;
        legacy[4] = 2;
        legacy.erase(legacy.size() - emptyWorldBytes);
        legacy.append(second, headerBytes);
        ASSERT_GT(legacy.size(), BrutalBlood::MaxSaveBytes);
        ASSERT_TRUE(mBlood.load(legacy));
        EXPECT_TRUE(mBlood.isBloody({ 1, 0 }));
        EXPECT_TRUE(mBlood.isBloody({ 2, 0 }));
        const auto bounded = mBlood.save();
        EXPECT_LE(bounded.size(), BrutalBlood::MaxSaveBytes);
        EXPECT_TRUE(mBlood.load(bounded));
    }

    TEST_F(BrutalBloodSaveTest, remapsPlacedItemsAndKeepsGeneratedItems)
    {
        addMark({ 1, 4 }, 1, 3);
        addMark({ 1, 5 }, 2, 3);
        addMark({ 2, -1 }, 3, 3);
        const auto saved = mBlood.save();
        const std::map<int, int> mapping{ { 4, 5 } };
        ASSERT_TRUE(mBlood.load(saved, &mapping));
        EXPECT_FALSE(mBlood.isBloody({ 1, 4 }));
        const auto* moved = mBlood.item({ 1, 5 });
        ASSERT_NE(moved, nullptr);
        ASSERT_EQ(moved->marks.size(), 1);
        EXPECT_EQ(moved->marks.front().hour, 1);
        EXPECT_TRUE(mBlood.isBloody({ 2, -1 }));
    }
}
