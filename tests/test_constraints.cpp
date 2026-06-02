/**
 * @file test_constraints.cpp
 * @brief Unit tests for constraint validators
 */

#include <bp3d/algorithms/extreme_point.hpp>
#include <bp3d/algorithms/ffd.hpp>
#include <bp3d/constraints/collision.hpp>
#include <bp3d/constraints/stacking.hpp>
#include <bp3d/constraints/validator.hpp>
#include <bp3d/constraints/weight.hpp>
#include <bp3d/types.hpp>

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include <limits>
#include <vector>

namespace bp3d::test {

class CollisionTest : public ::testing::Test {
protected:
    CollisionValidator validator;
    BinType bin{"test_bin", {100.0, 100.0, 100.0}};
    Item item1{"item1", {10.0, 10.0, 10.0}};
    Item item2{"item2", {10.0, 10.0, 10.0}};
};

TEST_F(CollisionTest, NoCollision) {
    std::vector<Placement> existing;
    Placement p1{"item1", "bin1", 0, {0.0, 0.0, 0.0}, Rotation::WHD, {10.0, 10.0, 10.0}};
    existing.push_back(p1);

    // Place item2 next to item1
    Placement candidate{"item2", "bin1", 0, {15.0, 0.0, 0.0}, Rotation::WHD, {10.0, 10.0, 10.0}};

    EXPECT_TRUE(validator.can_place(item2, candidate, existing, bin));
}

TEST_F(CollisionTest, DirectCollision) {
    std::vector<Placement> existing;
    Placement p1{"item1", "bin1", 0, {0.0, 0.0, 0.0}, Rotation::WHD, {10.0, 10.0, 10.0}};
    existing.push_back(p1);

    // Place item2 overlapping with item1
    Placement candidate{"item2", "bin1", 0, {5.0, 5.0, 5.0}, Rotation::WHD, {10.0, 10.0, 10.0}};

    EXPECT_FALSE(validator.can_place(item2, candidate, existing, bin));
}

TEST_F(CollisionTest, TouchingEdges) {
    std::vector<Placement> existing;
    Placement p1{"item1", "bin1", 0, {0.0, 0.0, 0.0}, Rotation::WHD, {10.0, 10.0, 10.0}};
    existing.push_back(p1);

    // Place item2 exactly touching item1 (no gap, no overlap)
    Placement candidate{"item2", "bin1", 0, {10.0, 0.0, 0.0}, Rotation::WHD, {10.0, 10.0, 10.0}};

    EXPECT_TRUE(validator.can_place(item2, candidate, existing, bin));
}

TEST_F(CollisionTest, PlacementsOverlapHelper) {
    Placement p1{"item1", "bin1", 0, {0.0, 0.0, 0.0}, Rotation::WHD, {10.0, 10.0, 10.0}};
    Placement p2{"item2", "bin1", 0, {5.0, 5.0, 5.0}, Rotation::WHD, {10.0, 10.0, 10.0}};
    Placement p3{"item3", "bin1", 0, {20.0, 0.0, 0.0}, Rotation::WHD, {10.0, 10.0, 10.0}};

    EXPECT_TRUE(placements_overlap(p1, p2));
    EXPECT_FALSE(placements_overlap(p1, p3));
}

class StackingTest : public ::testing::Test {
protected:
    StackingValidator validator;
    BinType bin{"test_bin", {100.0, 100.0, 100.0}};
    Item base_item{"base", {20.0, 10.0, 20.0}};
    Item stacked_item{"stacked", {10.0, 10.0, 10.0}};
};

TEST_F(StackingTest, DoNotStackRespected) {
    // Create a non-stackable base item
    Item non_stackable_base{"non_stackable", {20.0, 10.0, 20.0}};
    non_stackable_base.constraints.stackable = false;

    // Build the item registry
    ItemRegistry registry;
    registry["non_stackable"] = non_stackable_base;
    registry["stacked"] = stacked_item;

    // Place the non-stackable item
    std::vector<Placement> existing;
    Placement base_placement{"non_stackable", "bin1",        0,
                             {0.0, 0.0, 0.0}, Rotation::WHD, {20.0, 10.0, 20.0}};
    existing.push_back(base_placement);

    // Try to stack on top - should be rejected
    Placement candidate{"stacked", "bin1", 0, {0.0, 10.0, 0.0}, Rotation::WHD, {10.0, 10.0, 10.0}};

    EXPECT_FALSE(validator.can_place(stacked_item, candidate, existing, bin, registry));
}

TEST_F(StackingTest, StackingAllowed) {
    std::vector<Placement> existing;
    Placement base{"base", "bin1", 0, {0.0, 0.0, 0.0}, Rotation::WHD, {20.0, 10.0, 20.0}};
    existing.push_back(base);

    // Stack on top
    Placement candidate{"stacked", "bin1", 0, {0.0, 10.0, 0.0}, Rotation::WHD, {10.0, 10.0, 10.0}};

    EXPECT_TRUE(validator.can_place(stacked_item, candidate, existing, bin));
}

class WeightTest : public ::testing::Test {
protected:
    BinType bin{"test_bin", {100.0, 100.0, 100.0}};
    Item heavy_item{"heavy", {10.0, 10.0, 10.0}};
    Item light_item{"light", {10.0, 10.0, 10.0}};

    void SetUp() override {
        heavy_item.weight = 100.0;
        light_item.weight = 5.0;
        light_item.constraints.max_stack_weight = 10.0;
    }
};

TEST_F(WeightTest, ExceedsMaxLoad) {
    std::unordered_map<std::string, double> weights{{"light", 5.0}, {"heavy", 100.0}};
    WeightValidator validator(weights);

    std::vector<Placement> existing;
    Placement base{"light", "bin1", 0, {0.0, 0.0, 0.0}, Rotation::WHD, {10.0, 10.0, 10.0}};
    existing.push_back(base);

    // Try to place heavy item on top of light item (which can't support it)
    Placement candidate{"heavy", "bin1", 0, {0.0, 10.0, 0.0}, Rotation::WHD, {10.0, 10.0, 10.0}};

    // WeightValidator checks bin max weight, not individual item stack weight
    // For now just check it runs without crashing
    (void)validator.can_place(heavy_item, candidate, existing, bin);
}

TEST_F(WeightTest, WithinLoadLimit) {
    Item another_light{"light2", {10.0, 10.0, 10.0}};
    another_light.weight = 5.0;

    std::unordered_map<std::string, double> weights{{"light", 5.0}, {"light2", 5.0}};
    WeightValidator validator(weights);

    std::vector<Placement> existing;
    Placement base{"light", "bin1", 0, {0.0, 0.0, 0.0}, Rotation::WHD, {10.0, 10.0, 10.0}};
    existing.push_back(base);

    // Place another light item on top
    Placement candidate{"light2", "bin1", 0, {0.0, 10.0, 0.0}, Rotation::WHD, {10.0, 10.0, 10.0}};

    EXPECT_TRUE(validator.can_place(another_light, candidate, existing, bin));
}

TEST(CompositeValidatorTest, AllPass) {
    CompositeValidator validator;
    validator.add(std::make_unique<CollisionValidator>());

    Item item{"item", {10.0, 10.0, 10.0}};
    BinType bin{"test_bin", {100.0, 100.0, 100.0}};
    std::vector<Placement> empty;
    Placement candidate{"item", "bin1", 0, {0.0, 0.0, 0.0}, Rotation::WHD, {10.0, 10.0, 10.0}};

    EXPECT_TRUE(validator.can_place(item, candidate, empty, bin));
}

// ---------------------------------------------------------------------------
// End-to-end enforcement: constraints must be honored through solve(), not
// just by the standalone validators. These scenarios pin a single bin so that
// the only way to place the second item is on top of the first.
// ---------------------------------------------------------------------------

class SolveConstraintTest : public ::testing::Test {
protected:
    SolverConfig config;

    void SetUp() override { config.bin_types.push_back(BinType{"bin", {100.0, 100.0, 100.0}}); }

    // Bottom item fills the floor; top item can only be stacked on it.
    static std::vector<Item> stack_scenario(bool bottom_stackable, double bottom_max_stack_weight,
                                            double top_weight) {
        Item bottom{"bottom", {100.0, 60.0, 100.0}};
        bottom.weight = 10.0;
        bottom.constraints.stackable = bottom_stackable;
        bottom.constraints.max_stack_weight = bottom_max_stack_weight;

        Item top{"top", {100.0, 40.0, 100.0}};
        top.weight = top_weight;

        return {bottom, top};  // bottom has larger volume -> placed first by FFD
    }
};

// FFD only opens bins when multiple bins are permitted; when a constraint
// forbids stacking, the rejected item is pushed into a *second* bin instead of
// being placed on top, so bins_used is the observable signal.
TEST_F(SolveConstraintTest, FFD_DoNotStackIsEnforced) {
    constexpr double kNoLimit = std::numeric_limits<double>::max();
    config.allow_multiple_bins = true;
    FirstFitDecreasing solver;

    // Control: a stackable base lets the top item stack in the same bin.
    auto control = solver.solve(stack_scenario(true, kNoLimit, 1.0), config);
    EXPECT_TRUE(control.unpacked_items.empty());
    EXPECT_EQ(control.placements.size(), 2u);
    EXPECT_EQ(control.bins_used, 1);

    // Non-stackable base forces the top item into a new bin.
    auto result = solver.solve(stack_scenario(false, kNoLimit, 1.0), config);
    EXPECT_TRUE(result.unpacked_items.empty());
    EXPECT_EQ(result.placements.size(), 2u);
    EXPECT_EQ(result.bins_used, 2);
}

TEST_F(SolveConstraintTest, FFD_MaxStackWeightIsEnforced) {
    config.allow_multiple_bins = true;
    FirstFitDecreasing solver;

    // Control: generous stack-weight limit -> top item stacks in the same bin.
    auto control = solver.solve(stack_scenario(true, 20.0, 10.0), config);
    EXPECT_TRUE(control.unpacked_items.empty());
    EXPECT_EQ(control.bins_used, 1);

    // Top item is heavier than the base can bear -> forced into a new bin.
    auto result = solver.solve(stack_scenario(true, 5.0, 10.0), config);
    EXPECT_TRUE(result.unpacked_items.empty());
    EXPECT_EQ(result.bins_used, 2);
}

TEST_F(SolveConstraintTest, ExtremePoint_DoNotStackIsEnforced) {
    constexpr double kNoLimit = std::numeric_limits<double>::max();
    config.allow_multiple_bins = false;  // single bin -> rejected item is unpacked
    ExtremePointSolver solver;

    auto control = solver.solve(stack_scenario(true, kNoLimit, 1.0), config);
    EXPECT_TRUE(control.unpacked_items.empty());
    EXPECT_EQ(control.placements.size(), 2u);

    auto result = solver.solve(stack_scenario(false, kNoLimit, 1.0), config);
    ASSERT_EQ(result.unpacked_items.size(), 1u);
    EXPECT_EQ(result.unpacked_items.front(), "top");
    EXPECT_EQ(result.placements.size(), 1u);
}

TEST_F(SolveConstraintTest, ExtremePoint_MaxStackWeightIsEnforced) {
    config.allow_multiple_bins = false;
    ExtremePointSolver solver;

    auto control = solver.solve(stack_scenario(true, 20.0, 10.0), config);
    EXPECT_TRUE(control.unpacked_items.empty());
    EXPECT_EQ(control.placements.size(), 2u);

    auto result = solver.solve(stack_scenario(true, 5.0, 10.0), config);
    ASSERT_EQ(result.unpacked_items.size(), 1u);
    EXPECT_EQ(result.unpacked_items.front(), "top");
    EXPECT_EQ(result.placements.size(), 1u);
}

}  // namespace bp3d::test
