#include <algorithm>
#include <bit>
#include <cstddef>
#include <limits>
#include <memory>
#include <stdexcept>
#include <vector>

#include <gtest/gtest.h>

#include "../../../Source/Detail/BoxTreeCore.h"

namespace
{
    struct Interval
    {
        double low = std::numeric_limits<double>::infinity();
        double high = -std::numeric_limits<double>::infinity();
        bool IsValid() const { return low <= high; }
        double Center() const { return low * 0.5 + high * 0.5; }
    };

    struct Entry
    {
        std::size_t id;
        Interval box;
    };

    struct Node
    {
        Interval bounds;
        std::size_t entryCount = 0;
        std::vector<std::size_t> leafIndices;
        std::unique_ptr<Node> left;
        std::unique_ptr<Node> right;
        bool IsLeaf() const { return !left && !right; }
    };

    template <int Axes>
    struct CountedTraits
    {
        static constexpr int AxisCount = Axes;
        inline static std::size_t operations = 0;
        inline static bool failBuild = false;
        static Interval EmptyBox() { return {}; }
        static void ExpandBounds(Interval& bounds, const Interval& box)
        {
            ++operations;
            bounds.low = std::min(bounds.low, box.low);
            bounds.high = std::max(bounds.high, box.high);
        }
        static double CenterCoordinate(const Interval& box, int axis)
        {
            if (axis < 0 || axis >= Axes) throw std::logic_error("invalid split axis");
            if (failBuild) throw std::runtime_error("simulated subtree build failure");
            ++operations;
            return box.Center();
        }
        static double Distance(double point, const Interval& box)
        {
            ++operations;
            return std::max({box.low - point, 0.0, point - box.high});
        }
        static double QuarterDistance(double point, const Interval& box)
        {
            return Distance(point, box) * 0.25;
        }
    };

    std::size_t Inspect(const Node& node, std::vector<std::size_t>& indices)
    {
        if (node.IsLeaf())
        {
            EXPECT_LE(node.leafIndices.size(), 4U);
            EXPECT_EQ(node.entryCount, node.leafIndices.size());
            indices.insert(indices.end(), node.leafIndices.begin(), node.leafIndices.end());
            return 1;
        }
        EXPECT_TRUE(node.left && node.right);
        if (!node.left || !node.right) return 0;
        EXPECT_EQ(node.entryCount, node.left->entryCount + node.right->entryCount);
        return 1 + std::max(Inspect(*node.left, indices), Inspect(*node.right, indices));
    }

    template <int Axes>
    void CheckInsertionWork(bool overlap)
    {
        using Traits = CountedTraits<Axes>;
        constexpr std::size_t count = 8192;
        std::vector<Entry> entries;
        entries.reserve(count);
        std::unique_ptr<Node> root;
        Traits::operations = 0;
        for (std::size_t index = 0; index < count; ++index)
        {
            const double x = overlap ? 0.0 : static_cast<double>(index);
            entries.push_back({index, {x, x + 0.25}});
            (void)Geometry::Detail::InsertBoxTreeIndex<Node, Entry, Traits>(root, entries, index, 0, true);
        }
        // 固定规模的几何操作预算，不依赖机器计时或具体树形。
        EXPECT_LT(Traits::operations, 64 * count * std::bit_width(count));
        ASSERT_TRUE(root);
        EXPECT_EQ(root->entryCount, count);
        std::vector<std::size_t> indices;
        EXPECT_LE(Inspect(*root, indices), 4 * std::bit_width(count));
        std::sort(indices.begin(), indices.end());
        ASSERT_EQ(indices.size(), count);
        for (std::size_t index = 0; index < count; ++index) EXPECT_EQ(indices[index], index);
    }
}

TEST(BoxTreeBalanceTest, OrderedTwoAxisInsertionHasBoundedWork) { CheckInsertionWork<2>(false); }
TEST(BoxTreeBalanceTest, OverlappingTwoAxisInsertionHasBoundedWork) { CheckInsertionWork<2>(true); }
TEST(BoxTreeBalanceTest, OrderedThreeAxisInsertionHasBoundedWork) { CheckInsertionWork<3>(false); }
TEST(BoxTreeBalanceTest, OverlappingThreeAxisInsertionHasBoundedWork) { CheckInsertionWork<3>(true); }

TEST(BoxTreeBalanceTest, RebuildFailurePreservesOldSubtree)
{
    using Traits = CountedTraits<2>;
    std::vector<Entry> entries;
    std::unique_ptr<Node> root;
    for (std::size_t index = 0; index < 8; ++index)
    {
        const double x = static_cast<double>(index);
        entries.push_back({index, {x, x + 0.25}});
        (void)Geometry::Detail::InsertBoxTreeIndex<Node, Entry, Traits>(root, entries, index, 0, true);
    }
    entries.push_back({8, {8.0, 8.25}});
    Traits::failBuild = true;
    EXPECT_THROW((void)(Geometry::Detail::InsertBoxTreeIndex<Node, Entry, Traits>(root, entries, 8, 0, true)),
                 std::runtime_error);
    Traits::failBuild = false;
    entries.pop_back();
    ASSERT_TRUE(root);
    EXPECT_EQ(root->entryCount, 8U);
    EXPECT_EQ(root->bounds.low, 0.0);
    EXPECT_EQ(root->bounds.high, 7.25);
    std::vector<std::size_t> indices;
    (void)Inspect(*root, indices);
    std::sort(indices.begin(), indices.end());
    EXPECT_EQ(indices, (std::vector<std::size_t>{0, 1, 2, 3, 4, 5, 6, 7}));
}
