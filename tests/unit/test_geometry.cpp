/**
 * @file test_geometry.cpp
 * @brief Unit tests for internal geometry utilities
 */

#include "../../src/internal/geometry.hpp"
#include "bp3d/result.hpp"
#include "bp3d/types.hpp"

#include <gtest/gtest.h>

namespace bp3d::internal::test {

// =============================================================================
// AABB Tests
// =============================================================================

class AABBTest : public ::testing::Test {
protected:
    AABB box1{{0.0, 0.0, 0.0}, {10.0, 10.0, 10.0}};
    AABB box2{{5.0, 5.0, 5.0}, {15.0, 15.0, 15.0}};
    AABB box3{{20.0, 20.0, 20.0}, {30.0, 30.0, 30.0}};
};

TEST_F(AABBTest, FromPlacement) {
    Placement p{"item", "bin", 0, {5.0, 10.0, 15.0}, Rotation::WHD, {10.0, 20.0, 30.0}};
    auto aabb = AABB::from_placement(p);

    EXPECT_DOUBLE_EQ(aabb.min.x, 5.0);
    EXPECT_DOUBLE_EQ(aabb.min.y, 10.0);
    EXPECT_DOUBLE_EQ(aabb.min.z, 15.0);
    EXPECT_DOUBLE_EQ(aabb.max.x, 15.0);
    EXPECT_DOUBLE_EQ(aabb.max.y, 30.0);
    EXPECT_DOUBLE_EQ(aabb.max.z, 45.0);
}

TEST_F(AABBTest, FromPosAndDims) {
    Vector3 pos{1.0, 2.0, 3.0};
    Dimensions dims{4.0, 5.0, 6.0};
    auto aabb = AABB::from_pos_dims(pos, dims);

    EXPECT_DOUBLE_EQ(aabb.min.x, 1.0);
    EXPECT_DOUBLE_EQ(aabb.min.y, 2.0);
    EXPECT_DOUBLE_EQ(aabb.min.z, 3.0);
    EXPECT_DOUBLE_EQ(aabb.max.x, 5.0);
    EXPECT_DOUBLE_EQ(aabb.max.y, 7.0);
    EXPECT_DOUBLE_EQ(aabb.max.z, 9.0);
}

TEST_F(AABBTest, Dimensions) {
    auto dims = box1.dimensions();
    EXPECT_DOUBLE_EQ(dims.width, 10.0);
    EXPECT_DOUBLE_EQ(dims.height, 10.0);
    EXPECT_DOUBLE_EQ(dims.depth, 10.0);
}

TEST_F(AABBTest, Volume) {
    EXPECT_DOUBLE_EQ(box1.volume(), 1000.0);
}

TEST_F(AABBTest, OverlapsTrue) {
    EXPECT_TRUE(box1.overlaps(box2));
    EXPECT_TRUE(box2.overlaps(box1));  // Symmetric
}

TEST_F(AABBTest, OverlapsFalse) {
    EXPECT_FALSE(box1.overlaps(box3));
    EXPECT_FALSE(box3.overlaps(box1));
}

TEST_F(AABBTest, OverlapsSelf) {
    EXPECT_TRUE(box1.overlaps(box1));
}

TEST_F(AABBTest, OverlapsTouching) {
    AABB touching{{10.0, 0.0, 0.0}, {20.0, 10.0, 10.0}};
    // Touching but not overlapping
    EXPECT_FALSE(box1.overlaps(touching));
}

TEST_F(AABBTest, ContainsPoint) {
    Vector3 inside{5.0, 5.0, 5.0};
    Vector3 outside{15.0, 15.0, 15.0};
    Vector3 corner{0.0, 0.0, 0.0};
    Vector3 edge{10.0, 5.0, 5.0};

    EXPECT_TRUE(box1.contains(inside));
    EXPECT_FALSE(box1.contains(outside));
    EXPECT_TRUE(box1.contains(corner));  // Corners included
    EXPECT_TRUE(box1.contains(edge));    // Edges included
}

TEST_F(AABBTest, ContainsAABBTrue) {
    AABB inner{{2.0, 2.0, 2.0}, {8.0, 8.0, 8.0}};
    EXPECT_TRUE(box1.contains(inner));
}

TEST_F(AABBTest, ContainsAABBFalse) {
    EXPECT_FALSE(box1.contains(box2));  // Partially overlapping
    EXPECT_FALSE(box1.contains(box3));  // Disjoint
}

TEST_F(AABBTest, ContainsSelf) {
    EXPECT_TRUE(box1.contains(box1));
}

// =============================================================================
// fits_in Tests
// =============================================================================

TEST(FitsInTest, ExactFit) {
    Dimensions box{10.0, 20.0, 30.0};
    Dimensions space{10.0, 20.0, 30.0};
    EXPECT_TRUE(fits_in(box, space));
}

TEST(FitsInTest, SmallerBox) {
    Dimensions box{5.0, 10.0, 15.0};
    Dimensions space{10.0, 20.0, 30.0};
    EXPECT_TRUE(fits_in(box, space));
}

TEST(FitsInTest, LargerBox) {
    Dimensions box{15.0, 25.0, 35.0};
    Dimensions space{10.0, 20.0, 30.0};
    EXPECT_FALSE(fits_in(box, space));
}

TEST(FitsInTest, OneDimensionTooLarge) {
    Dimensions box{10.0, 20.0, 35.0};  // depth too large
    Dimensions space{10.0, 20.0, 30.0};
    EXPECT_FALSE(fits_in(box, space));
}

TEST(FitsInTest, ZeroSize) {
    Dimensions zero{0.0, 0.0, 0.0};
    Dimensions space{10.0, 20.0, 30.0};
    EXPECT_TRUE(fits_in(zero, space));
}

// =============================================================================
// intersection Tests
// =============================================================================

TEST(IntersectionTest, Overlapping) {
    AABB a{{0.0, 0.0, 0.0}, {10.0, 10.0, 10.0}};
    AABB b{{5.0, 5.0, 5.0}, {15.0, 15.0, 15.0}};

    auto result = intersection(a, b);
    ASSERT_TRUE(result.has_value());
    EXPECT_DOUBLE_EQ(result->min.x, 5.0);
    EXPECT_DOUBLE_EQ(result->min.y, 5.0);
    EXPECT_DOUBLE_EQ(result->min.z, 5.0);
    EXPECT_DOUBLE_EQ(result->max.x, 10.0);
    EXPECT_DOUBLE_EQ(result->max.y, 10.0);
    EXPECT_DOUBLE_EQ(result->max.z, 10.0);
}

TEST(IntersectionTest, Disjoint) {
    AABB a{{0.0, 0.0, 0.0}, {10.0, 10.0, 10.0}};
    AABB b{{20.0, 20.0, 20.0}, {30.0, 30.0, 30.0}};

    auto result = intersection(a, b);
    EXPECT_FALSE(result.has_value());
}

TEST(IntersectionTest, Touching) {
    AABB a{{0.0, 0.0, 0.0}, {10.0, 10.0, 10.0}};
    AABB b{{10.0, 0.0, 0.0}, {20.0, 10.0, 10.0}};

    auto result = intersection(a, b);
    // Touching boxes have zero-volume intersection or none
    if (result.has_value()) {
        EXPECT_DOUBLE_EQ(result->volume(), 0.0);
    }
}

TEST(IntersectionTest, Contained) {
    AABB outer{{0.0, 0.0, 0.0}, {20.0, 20.0, 20.0}};
    AABB inner{{5.0, 5.0, 5.0}, {15.0, 15.0, 15.0}};

    auto result = intersection(outer, inner);
    ASSERT_TRUE(result.has_value());
    EXPECT_DOUBLE_EQ(result->min.x, 5.0);
    EXPECT_DOUBLE_EQ(result->max.x, 15.0);
    EXPECT_DOUBLE_EQ(result->volume(), 1000.0);
}

TEST(IntersectionTest, Symmetric) {
    AABB a{{0.0, 0.0, 0.0}, {10.0, 10.0, 10.0}};
    AABB b{{5.0, 5.0, 5.0}, {15.0, 15.0, 15.0}};

    auto result1 = intersection(a, b);
    auto result2 = intersection(b, a);

    ASSERT_TRUE(result1.has_value());
    ASSERT_TRUE(result2.has_value());
    EXPECT_DOUBLE_EQ(result1->volume(), result2->volume());
}

// =============================================================================
// support_ratio Tests
// =============================================================================

class SupportRatioTest : public ::testing::Test {
protected:
    Placement make_placement(const std::string& id, double x, double y, double z, double w,
                             double h, double d) {
        return {id, "bin", 0, {x, y, z}, Rotation::WHD, {w, h, d}};
    }
};

TEST_F(SupportRatioTest, OnFloor) {
    Placement proposed = make_placement("item", 0.0, 0.0, 0.0, 10.0, 10.0, 10.0);
    std::vector<Placement> existing;

    double ratio = support_ratio(proposed, existing, 0.0);
    EXPECT_DOUBLE_EQ(ratio, 1.0);  // Floor supports fully
}

TEST_F(SupportRatioTest, FullySupportedByItem) {
    Placement base = make_placement("base", 0.0, 0.0, 0.0, 20.0, 10.0, 20.0);
    Placement proposed = make_placement("top", 0.0, 10.0, 0.0, 10.0, 10.0, 10.0);
    std::vector<Placement> existing{base};

    double ratio = support_ratio(proposed, existing, 0.0);
    EXPECT_DOUBLE_EQ(ratio, 1.0);  // Base fully supports top
}

TEST_F(SupportRatioTest, PartiallySupportedByItem) {
    Placement base = make_placement("base", 0.0, 0.0, 0.0, 10.0, 10.0, 10.0);
    Placement proposed = make_placement("top", 5.0, 10.0, 5.0, 10.0, 10.0, 10.0);
    std::vector<Placement> existing{base};

    double ratio = support_ratio(proposed, existing, 0.0);
    // Only 5x5 of the 10x10 bottom is supported = 25%
    EXPECT_NEAR(ratio, 0.25, 0.01);
}

TEST_F(SupportRatioTest, Unsupported) {
    Placement base = make_placement("base", 0.0, 0.0, 0.0, 10.0, 10.0, 10.0);
    Placement proposed = make_placement("floating", 50.0, 10.0, 50.0, 10.0, 10.0, 10.0);
    std::vector<Placement> existing{base};

    double ratio = support_ratio(proposed, existing, 0.0);
    EXPECT_DOUBLE_EQ(ratio, 0.0);
}

TEST_F(SupportRatioTest, MultipleSupportingItems) {
    Placement base1 = make_placement("base1", 0.0, 0.0, 0.0, 10.0, 10.0, 10.0);
    Placement base2 = make_placement("base2", 10.0, 0.0, 0.0, 10.0, 10.0, 10.0);
    Placement proposed = make_placement("top", 5.0, 10.0, 0.0, 10.0, 10.0, 10.0);
    std::vector<Placement> existing{base1, base2};

    double ratio = support_ratio(proposed, existing, 0.0);
    EXPECT_DOUBLE_EQ(ratio, 1.0);  // Both bases together fully support
}

}  // namespace bp3d::internal::test
