/**
 * @file test_rotation.cpp
 * @brief Unit tests for rotation functions
 */

#include <bp3d/rotation.hpp>

#include <gtest/gtest.h>

namespace bp3d::test {

TEST(RotationTest, ApplyWHD) {
    Dimensions d{10.0, 20.0, 30.0};
    auto result = apply_rotation(d, Rotation::WHD);

    EXPECT_DOUBLE_EQ(result.width, 10.0);
    EXPECT_DOUBLE_EQ(result.height, 20.0);
    EXPECT_DOUBLE_EQ(result.depth, 30.0);
}

TEST(RotationTest, ApplyWDH) {
    Dimensions d{10.0, 20.0, 30.0};
    auto result = apply_rotation(d, Rotation::WDH);

    EXPECT_DOUBLE_EQ(result.width, 10.0);
    EXPECT_DOUBLE_EQ(result.height, 30.0);
    EXPECT_DOUBLE_EQ(result.depth, 20.0);
}

TEST(RotationTest, ApplyHWD) {
    Dimensions d{10.0, 20.0, 30.0};
    auto result = apply_rotation(d, Rotation::HWD);

    EXPECT_DOUBLE_EQ(result.width, 20.0);
    EXPECT_DOUBLE_EQ(result.height, 10.0);
    EXPECT_DOUBLE_EQ(result.depth, 30.0);
}

TEST(RotationTest, ApplyHDW) {
    Dimensions d{10.0, 20.0, 30.0};
    auto result = apply_rotation(d, Rotation::HDW);

    EXPECT_DOUBLE_EQ(result.width, 20.0);
    EXPECT_DOUBLE_EQ(result.height, 30.0);
    EXPECT_DOUBLE_EQ(result.depth, 10.0);
}

TEST(RotationTest, ApplyDWH) {
    Dimensions d{10.0, 20.0, 30.0};
    auto result = apply_rotation(d, Rotation::DWH);

    EXPECT_DOUBLE_EQ(result.width, 30.0);
    EXPECT_DOUBLE_EQ(result.height, 10.0);
    EXPECT_DOUBLE_EQ(result.depth, 20.0);
}

TEST(RotationTest, ApplyDHW) {
    Dimensions d{10.0, 20.0, 30.0};
    auto result = apply_rotation(d, Rotation::DHW);

    EXPECT_DOUBLE_EQ(result.width, 30.0);
    EXPECT_DOUBLE_EQ(result.height, 20.0);
    EXPECT_DOUBLE_EQ(result.depth, 10.0);
}

TEST(RotationTest, VolumePreserved) {
    Dimensions d{10.0, 20.0, 30.0};
    double original_volume = d.volume();

    for (Rotation r : kAllRotations) {
        auto rotated = apply_rotation(d, r);
        EXPECT_DOUBLE_EQ(rotated.volume(), original_volume);
    }
}

TEST(RotationTest, GetAllRotations) {
    EXPECT_EQ(kAllRotations.size(), 6);
}

TEST(RotationTest, GetAllowedRotationsNoConstraint) {
    ItemConstraints c{};
    auto allowed = get_allowed_rotations(c);
    EXPECT_EQ(allowed.count(), 6);  // bitset uses count() not size()
}

TEST(RotationTest, GetAllowedRotationsThisSideUp) {
    ItemConstraints c{};
    c.this_side_up = true;
    auto allowed = get_allowed_rotations(c);

    // Should only allow rotations where height = original height (WHD, DHW)
    EXPECT_EQ(allowed.count(), 2);
    EXPECT_TRUE(allowed.test(static_cast<size_t>(Rotation::WHD)));
    EXPECT_TRUE(allowed.test(static_cast<size_t>(Rotation::DHW)));
}

TEST(RotationTest, RotationName) {
    EXPECT_EQ(rotation_name(Rotation::WHD), "WHD");
    EXPECT_EQ(rotation_name(Rotation::WDH), "WDH");
    EXPECT_EQ(rotation_name(Rotation::HWD), "HWD");
    EXPECT_EQ(rotation_name(Rotation::HDW), "HDW");
    EXPECT_EQ(rotation_name(Rotation::DWH), "DWH");
    EXPECT_EQ(rotation_name(Rotation::DHW), "DHW");
}

}  // namespace bp3d::test
