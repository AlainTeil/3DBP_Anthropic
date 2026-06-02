/**
 * @file test_io_integration.cpp
 * @brief Integration tests for JSON I/O + solver pipeline
 */

#include "test_support.hpp"

#include <bp3d/algorithms/ffd.hpp>
#include <bp3d/bp3d.hpp>
#include <bp3d/io/json_serializer.hpp>
#include <bp3d/io/obj_exporter.hpp>

#include <gtest/gtest.h>

#include <filesystem>
#include <fstream>

namespace bp3d::test {

/**
 * @brief Integration tests for complete I/O workflows
 */
class IOIntegrationTest : public ::testing::Test {
protected:
    std::filesystem::path temp_dir;

    void SetUp() override { temp_dir = make_unique_temp_dir("io_integration"); }

    void TearDown() override { remove_temp_dir(temp_dir); }
};

TEST_F(IOIntegrationTest, LoadSolveExportJSON) {
    // Create input file
    auto input_path = temp_dir / "input.json";
    std::ofstream in_file(input_path);
    in_file << R"({
        "items": [
            {"id": "box1", "width": 10, "height": 10, "depth": 10, "quantity": 5},
            {"id": "box2", "width": 20, "height": 15, "depth": 25}
        ],
        "bins": [
            {"id": "container", "width": 100, "height": 100, "depth": 100}
        ]
    })";
    in_file.close();

    // Load input
    auto input = load_input_file(input_path.string());
    ASSERT_EQ(input.items.size(), 2);
    ASSERT_EQ(input.bins.size(), 1);

    // Configure solver
    SolverConfig config;
    config.bin_types = input.bins;
    config.allow_multiple_bins = true;

    // Solve
    FirstFitDecreasing solver;
    auto result = solver.solve(input.items, config);

    // Export result
    auto output_path = temp_dir / "output.json";
    save_result_file(result, output_path.string());

    // Verify output file exists and has content
    EXPECT_TRUE(std::filesystem::exists(output_path));

    std::ifstream out_file(output_path);
    std::string content((std::istreambuf_iterator<char>(out_file)),
                        std::istreambuf_iterator<char>());
    EXPECT_FALSE(content.empty());
    EXPECT_NE(content.find("FirstFitDecreasing"), std::string::npos);
}

TEST_F(IOIntegrationTest, LoadSolveExportOBJ) {
    // Create input file
    auto input_path = temp_dir / "input.json";
    std::ofstream in_file(input_path);
    in_file << R"({
        "items": [
            {"id": "item1", "width": 20, "height": 30, "depth": 40},
            {"id": "item2", "width": 15, "height": 25, "depth": 35}
        ],
        "bins": [
            {"id": "bin", "width": 100, "height": 100, "depth": 100}
        ]
    })";
    in_file.close();

    // Load and solve
    auto input = load_input_file(input_path.string());
    SolverConfig config;
    config.bin_types = input.bins;

    FirstFitDecreasing solver;
    auto result = solver.solve(input.items, config);

    // Export to OBJ
    auto obj_path = temp_dir / "result.obj";
    export_to_obj(result, input.bins, obj_path);

    EXPECT_TRUE(std::filesystem::exists(obj_path));
    EXPECT_GT(std::filesystem::file_size(obj_path), 0);
}

TEST_F(IOIntegrationTest, RoundTripItems) {
    std::vector<Item> original;
    Item item{"test_item", {10.5, 20.5, 30.5}};
    item.quantity = 3;
    item.weight = 5.5;
    item.constraints.this_side_up = true;
    original.push_back(item);

    // Serialize to JSON string
    auto json = items_to_json(original);

    // Deserialize back
    auto restored = items_from_json(json);

    ASSERT_EQ(restored.size(), 1);
    EXPECT_EQ(restored[0].id, original[0].id);
    EXPECT_DOUBLE_EQ(restored[0].dimensions.width, original[0].dimensions.width);
    EXPECT_DOUBLE_EQ(restored[0].dimensions.height, original[0].dimensions.height);
    EXPECT_DOUBLE_EQ(restored[0].dimensions.depth, original[0].dimensions.depth);
    EXPECT_EQ(restored[0].quantity, original[0].quantity);
    EXPECT_DOUBLE_EQ(restored[0].weight, original[0].weight);
    EXPECT_EQ(restored[0].constraints.this_side_up, original[0].constraints.this_side_up);
}

TEST_F(IOIntegrationTest, RoundTripBinTypes) {
    std::vector<BinType> original;
    BinType bin{"container", {100.0, 200.0, 300.0}, 500.0, 10.0};
    original.push_back(bin);

    auto json = bin_types_to_json(original);
    auto restored = bin_types_from_json(json);

    ASSERT_EQ(restored.size(), 1);
    EXPECT_EQ(restored[0].id, original[0].id);
    EXPECT_DOUBLE_EQ(restored[0].dimensions.width, original[0].dimensions.width);
    EXPECT_DOUBLE_EQ(restored[0].dimensions.height, original[0].dimensions.height);
    EXPECT_DOUBLE_EQ(restored[0].dimensions.depth, original[0].dimensions.depth);
    EXPECT_DOUBLE_EQ(restored[0].max_weight, original[0].max_weight);
    EXPECT_DOUBLE_EQ(restored[0].cost, original[0].cost);
}

TEST_F(IOIntegrationTest, ComplexWorkflow) {
    // Create a complex input
    auto input_path = temp_dir / "complex_input.json";
    std::ofstream in_file(input_path);
    in_file << R"({
        "items": [
            {"id": "fragile", "width": 15, "height": 30, "depth": 15, "this_side_up": true},
            {"id": "heavy", "width": 20, "height": 20, "depth": 20, "weight": 100},
            {"id": "multi", "width": 10, "height": 10, "depth": 10, "quantity": 10}
        ],
        "bins": [
            {"id": "small", "width": 50, "height": 50, "depth": 50, "max_weight": 200, "cost": 5.0},
            {"id": "large", "width": 100, "height": 100, "depth": 100, "max_weight": 500, "cost": 15.0}
        ],
        "options": {
            "allow_multiple_bins": true
        }
    })";
    in_file.close();

    // Load
    auto input = load_input_file(input_path.string());

    // Solve
    SolverConfig config;
    config.bin_types = input.bins;
    config.allow_multiple_bins = true;

    FirstFitDecreasing solver;
    auto result = solver.solve(input.items, config);

    // Export both JSON and OBJ
    auto json_output = temp_dir / "complex_output.json";
    auto obj_output = temp_dir / "complex_output.obj";

    save_result_file(result, json_output.string());
    export_to_obj(result, input.bins, obj_output);

    EXPECT_TRUE(std::filesystem::exists(json_output));
    EXPECT_TRUE(std::filesystem::exists(obj_output));

    // Verify quantities were expanded
    // 12 total items (1 fragile + 1 heavy + 10 multi)
    EXPECT_EQ(result.placements.size() + result.unpacked_items.size(), 12);
}

TEST_F(IOIntegrationTest, HandleInvalidInput) {
    auto input_path = temp_dir / "invalid.json";
    std::ofstream in_file(input_path);
    in_file << R"({ "not_valid_schema": true })";
    in_file.close();

    // The loader may return empty collections rather than throw
    // Check that it handles invalid schema gracefully
    auto input = load_input_file(input_path.string());
    // If no items or bins found, the input is effectively invalid
    EXPECT_TRUE(input.items.empty() || input.bins.empty());
}

TEST_F(IOIntegrationTest, HandleMissingFile) {
    EXPECT_THROW((void)load_input_file("/nonexistent/path/file.json"), std::exception);
}

TEST_F(IOIntegrationTest, MinimalValidInput) {
    auto input_path = temp_dir / "minimal.json";
    std::ofstream in_file(input_path);
    in_file << R"({
        "items": [{"id": "a", "width": 1, "height": 1, "depth": 1}],
        "bins": [{"id": "b", "width": 10, "height": 10, "depth": 10}]
    })";
    in_file.close();

    auto input = load_input_file(input_path.string());
    EXPECT_EQ(input.items.size(), 1);
    EXPECT_EQ(input.bins.size(), 1);
}

/**
 * @brief Test fixture-based integration tests
 */
class FixtureIntegrationTest : public ::testing::Test {
protected:
    std::filesystem::path fixture_dir;

    void SetUp() override {
        fixture_dir = std::filesystem::path(__FILE__).parent_path().parent_path() / "fixtures";
    }
};

TEST_F(FixtureIntegrationTest, LoadSimpleFixture) {
    auto fixture_path = fixture_dir / "simple.json";
    if (!std::filesystem::exists(fixture_path)) {
        GTEST_SKIP() << "Fixture file not found: " << fixture_path;
    }

    auto input = load_input_file(fixture_path.string());
    EXPECT_GT(input.items.size(), 0);
    EXPECT_GT(input.bins.size(), 0);

    SolverConfig config;
    config.bin_types = input.bins;

    FirstFitDecreasing solver;
    auto result = solver.solve(input.items, config);

    EXPECT_GT(result.placements.size(), 0);
}

TEST_F(FixtureIntegrationTest, LoadComplexFixture) {
    auto fixture_path = fixture_dir / "complex.json";
    if (!std::filesystem::exists(fixture_path)) {
        GTEST_SKIP() << "Fixture file not found: " << fixture_path;
    }

    auto input = load_input_file(fixture_path.string());
    EXPECT_GT(input.items.size(), 0);
    EXPECT_GT(input.bins.size(), 0);

    SolverConfig config;
    config.bin_types = input.bins;
    config.allow_multiple_bins = true;

    FirstFitDecreasing solver;
    auto result = solver.solve(input.items, config);

    // Complex fixture should have meaningful results
    EXPECT_GT(result.placements.size(), 0);
}

}  // namespace bp3d::test
