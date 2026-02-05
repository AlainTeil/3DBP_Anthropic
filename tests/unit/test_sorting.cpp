/**
 * @file test_sorting.cpp
 * @brief Unit tests for item sorting utilities
 */

#include "../../src/internal/sorting.hpp"
#include "bp3d/types.hpp"

#include <gtest/gtest.h>

namespace bp3d::internal::test {

class SortingTest : public ::testing::Test {
protected:
    std::vector<Item> items;

    void SetUp() override {
        // Create items with varied dimensions and weights
        items.push_back(Item{"small", {10.0, 10.0, 10.0}});   // volume: 1000
        items.push_back(Item{"medium", {20.0, 20.0, 20.0}});  // volume: 8000
        items.push_back(Item{"large", {30.0, 30.0, 30.0}});   // volume: 27000
        items.push_back(Item{"tall", {10.0, 50.0, 10.0}});    // height: 50, volume: 5000
        items.push_back(Item{"heavy", {15.0, 15.0, 15.0}});   // volume: 3375

        items[0].weight = 1.0;
        items[1].weight = 5.0;
        items[2].weight = 10.0;
        items[3].weight = 3.0;
        items[4].weight = 100.0;
    }
};

// =============================================================================
// max_dimension Tests
// =============================================================================

TEST(MaxDimensionTest, AllEqual) {
    Item item{"cube", {10.0, 10.0, 10.0}};
    EXPECT_DOUBLE_EQ(max_dimension(item), 10.0);
}

TEST(MaxDimensionTest, WidthLargest) {
    Item item{"wide", {30.0, 10.0, 20.0}};
    EXPECT_DOUBLE_EQ(max_dimension(item), 30.0);
}

TEST(MaxDimensionTest, HeightLargest) {
    Item item{"tall", {10.0, 30.0, 20.0}};
    EXPECT_DOUBLE_EQ(max_dimension(item), 30.0);
}

TEST(MaxDimensionTest, DepthLargest) {
    Item item{"deep", {10.0, 20.0, 30.0}};
    EXPECT_DOUBLE_EQ(max_dimension(item), 30.0);
}

// =============================================================================
// Comparator Tests
// =============================================================================

TEST(ComparatorTest, VolumeDescending) {
    Item a{"a", {10.0, 10.0, 10.0}};  // 1000
    Item b{"b", {20.0, 20.0, 20.0}};  // 8000

    VolumeDescendingCompare cmp;
    EXPECT_TRUE(cmp(b, a));
    EXPECT_FALSE(cmp(a, b));
}

TEST(ComparatorTest, VolumeAscending) {
    Item a{"a", {10.0, 10.0, 10.0}};  // 1000
    Item b{"b", {20.0, 20.0, 20.0}};  // 8000

    VolumeAscendingCompare cmp;
    EXPECT_TRUE(cmp(a, b));
    EXPECT_FALSE(cmp(b, a));
}

TEST(ComparatorTest, HeightDescending) {
    Item a{"a", {10.0, 50.0, 10.0}};  // height 50
    Item b{"b", {20.0, 20.0, 20.0}};  // height 20

    HeightDescendingCompare cmp;
    EXPECT_TRUE(cmp(a, b));
    EXPECT_FALSE(cmp(b, a));
}

TEST(ComparatorTest, SurfaceAreaDescending) {
    Item a{"a", {10.0, 10.0, 10.0}};  // surface: 600
    Item b{"b", {20.0, 20.0, 20.0}};  // surface: 2400

    SurfaceAreaDescendingCompare cmp;
    EXPECT_TRUE(cmp(b, a));
    EXPECT_FALSE(cmp(a, b));
}

TEST(ComparatorTest, WeightDescending) {
    Item a{"a", {10.0, 10.0, 10.0}};
    Item b{"b", {10.0, 10.0, 10.0}};
    a.weight = 5.0;
    b.weight = 10.0;

    WeightDescendingCompare cmp;
    EXPECT_TRUE(cmp(b, a));
    EXPECT_FALSE(cmp(a, b));
}

TEST(ComparatorTest, MaxDimensionDescending) {
    Item a{"a", {10.0, 10.0, 10.0}};  // max: 10
    Item b{"b", {5.0, 30.0, 5.0}};    // max: 30

    MaxDimensionDescendingCompare cmp;
    EXPECT_TRUE(cmp(b, a));
    EXPECT_FALSE(cmp(a, b));
}

// =============================================================================
// sort_items Tests
// =============================================================================

TEST_F(SortingTest, SortByVolumeDescending) {
    sort_items(items, SortCriteria::VolumeDescending);

    EXPECT_EQ(items[0].id, "large");   // 27000
    EXPECT_EQ(items[1].id, "medium");  // 8000
    EXPECT_EQ(items[2].id, "tall");    // 5000
    EXPECT_EQ(items[3].id, "heavy");   // 3375
    EXPECT_EQ(items[4].id, "small");   // 1000
}

TEST_F(SortingTest, SortByVolumeAscending) {
    sort_items(items, SortCriteria::VolumeAscending);

    EXPECT_EQ(items[0].id, "small");   // 1000
    EXPECT_EQ(items[1].id, "heavy");   // 3375
    EXPECT_EQ(items[2].id, "tall");    // 5000
    EXPECT_EQ(items[3].id, "medium");  // 8000
    EXPECT_EQ(items[4].id, "large");   // 27000
}

TEST_F(SortingTest, SortByHeightDescending) {
    sort_items(items, SortCriteria::HeightDescending);

    EXPECT_EQ(items[0].id, "tall");  // height 50
    // Others have heights 10, 20, 30, 15
}

TEST_F(SortingTest, SortBySurfaceAreaDescending) {
    sort_items(items, SortCriteria::SurfaceAreaDescending);

    EXPECT_EQ(items[0].id, "large");  // Largest surface area
}

TEST_F(SortingTest, SortByWeightDescending) {
    sort_items(items, SortCriteria::WeightDescending);

    EXPECT_EQ(items[0].id, "heavy");  // 100.0 weight
}

TEST_F(SortingTest, SortByMaxDimensionDescending) {
    sort_items(items, SortCriteria::MaxDimensionDescending);

    EXPECT_EQ(items[0].id, "tall");  // max dimension 50
}

TEST_F(SortingTest, SortEmptyList) {
    std::vector<Item> empty;
    sort_items(empty, SortCriteria::VolumeDescending);
    EXPECT_TRUE(empty.empty());
}

TEST_F(SortingTest, SortSingleItem) {
    std::vector<Item> single{Item{"only", {10.0, 10.0, 10.0}}};
    sort_items(single, SortCriteria::VolumeDescending);
    EXPECT_EQ(single.size(), 1);
    EXPECT_EQ(single[0].id, "only");
}

// =============================================================================
// expand_quantities Tests
// =============================================================================

TEST(ExpandQuantitiesTest, NoExpansionNeeded) {
    std::vector<Item> items;
    items.push_back(Item{"a", {10.0, 10.0, 10.0}});
    items.push_back(Item{"b", {20.0, 20.0, 20.0}});

    auto expanded = expand_quantities(items);
    EXPECT_EQ(expanded.size(), 2);
    EXPECT_EQ(expanded[0].id, "a");
    EXPECT_EQ(expanded[1].id, "b");
}

TEST(ExpandQuantitiesTest, ExpandSingle) {
    std::vector<Item> items;
    Item item{"box", {10.0, 10.0, 10.0}};
    item.quantity = 3;
    items.push_back(item);

    auto expanded = expand_quantities(items);
    ASSERT_EQ(expanded.size(), 3);
    EXPECT_EQ(expanded[0].id, "box_1");
    EXPECT_EQ(expanded[1].id, "box_2");
    EXPECT_EQ(expanded[2].id, "box_3");
    EXPECT_EQ(expanded[0].quantity, 1);
}

TEST(ExpandQuantitiesTest, ExpandMixed) {
    std::vector<Item> items;
    Item single{"single", {10.0, 10.0, 10.0}};
    single.quantity = 1;
    Item multi{"multi", {20.0, 20.0, 20.0}};
    multi.quantity = 2;
    items.push_back(single);
    items.push_back(multi);

    auto expanded = expand_quantities(items);
    ASSERT_EQ(expanded.size(), 3);
    EXPECT_EQ(expanded[0].id, "single");
    EXPECT_EQ(expanded[1].id, "multi_1");
    EXPECT_EQ(expanded[2].id, "multi_2");
}

TEST(ExpandQuantitiesTest, PreservesProperties) {
    std::vector<Item> items;
    Item item{"box", {10.0, 20.0, 30.0}};
    item.quantity = 2;
    item.weight = 5.0;
    item.constraints.this_side_up = true;
    items.push_back(item);

    auto expanded = expand_quantities(items);
    ASSERT_EQ(expanded.size(), 2);

    for (const auto& exp : expanded) {
        EXPECT_DOUBLE_EQ(exp.dimensions.width, 10.0);
        EXPECT_DOUBLE_EQ(exp.dimensions.height, 20.0);
        EXPECT_DOUBLE_EQ(exp.dimensions.depth, 30.0);
        EXPECT_DOUBLE_EQ(exp.weight, 5.0);
        EXPECT_TRUE(exp.constraints.this_side_up);
    }
}

TEST(ExpandQuantitiesTest, ZeroQuantity) {
    std::vector<Item> items;
    Item item{"zero", {10.0, 10.0, 10.0}};
    item.quantity = 0;
    items.push_back(item);

    auto expanded = expand_quantities(items);
    EXPECT_EQ(expanded.size(), 0);
}

TEST(ExpandQuantitiesTest, LargeQuantity) {
    std::vector<Item> items;
    Item item{"many", {10.0, 10.0, 10.0}};
    item.quantity = 100;
    items.push_back(item);

    auto expanded = expand_quantities(items);
    EXPECT_EQ(expanded.size(), 100);
}

TEST(ExpandQuantitiesTest, EmptyInput) {
    std::vector<Item> items;
    auto expanded = expand_quantities(items);
    EXPECT_TRUE(expanded.empty());
}

}  // namespace bp3d::internal::test
