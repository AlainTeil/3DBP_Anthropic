/**
 * @file test_types.cpp
 * @brief Unit tests for core types
 */

#include <bp3d/types.hpp>

#include <gtest/gtest.h>

namespace bp3d::test {

TEST(DimensionsTest, Volume) {
    Dimensions d{10.0, 20.0, 30.0};
    EXPECT_DOUBLE_EQ(d.volume(), 6000.0);
}

TEST(DimensionsTest, Comparison) {
    Dimensions d1{10.0, 20.0, 30.0};
    Dimensions d2{10.0, 20.0, 30.0};
    Dimensions d3{5.0, 20.0, 30.0};

    EXPECT_EQ(d1, d2);
    EXPECT_NE(d1, d3);
}

TEST(DimensionsTest, ContainsOrientation) {
    Dimensions bin{100.0, 100.0, 100.0};
    Dimensions item{50.0, 60.0, 70.0};
    Dimensions large{150.0, 50.0, 50.0};

    // Check if item fits in bin (all dimensions smaller or equal)
    EXPECT_TRUE(item.width <= bin.width && item.height <= bin.height && item.depth <= bin.depth);
    EXPECT_FALSE(large.width <= bin.width && large.height <= bin.height &&
                 large.depth <= bin.depth);
}

TEST(Vector3Test, Default) {
    Vector3 v{};
    EXPECT_DOUBLE_EQ(v.x, 0.0);
    EXPECT_DOUBLE_EQ(v.y, 0.0);
    EXPECT_DOUBLE_EQ(v.z, 0.0);
}

TEST(Vector3Test, Add) {
    Vector3 a{1.0, 2.0, 3.0};
    Vector3 b{4.0, 5.0, 6.0};
    Vector3 c = a + b;

    EXPECT_DOUBLE_EQ(c.x, 5.0);
    EXPECT_DOUBLE_EQ(c.y, 7.0);
    EXPECT_DOUBLE_EQ(c.z, 9.0);
}

TEST(ItemTest, Volume) {
    Item item{"test", {10.0, 20.0, 30.0}};
    EXPECT_DOUBLE_EQ(item.volume(), 6000.0);
}

TEST(ItemTest, Quantity) {
    Item item{"test", {10.0, 20.0, 30.0}};
    item.quantity = 5;
    EXPECT_EQ(item.quantity, 5);
}

TEST(ItemConstraintsTest, Defaults) {
    ItemConstraints c{};
    EXPECT_FALSE(c.this_side_up);
    EXPECT_TRUE(c.stackable);  // default stackable = true (opposite of do_not_stack)
    EXPECT_DOUBLE_EQ(c.max_stack_weight, std::numeric_limits<double>::max());
}

TEST(BinTypeTest, Volume) {
    BinType bin{"standard", {100.0, 100.0, 100.0}};
    EXPECT_DOUBLE_EQ(bin.dimensions.volume(), 1000000.0);
}

TEST(BinTypeTest, Defaults) {
    BinType bin{"test", {50.0, 50.0, 50.0}};
    EXPECT_DOUBLE_EQ(bin.max_weight, std::numeric_limits<double>::max());
    EXPECT_DOUBLE_EQ(bin.cost, 1.0);
}

}  // namespace bp3d::test
