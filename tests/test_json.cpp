/**
 * @file test_json.cpp
 * @brief Unit tests for JSON serialization
 */

#include <bp3d/io/json_serializer.hpp>

#include <gtest/gtest.h>

#include <filesystem>
#include <fstream>

namespace bp3d::test {

class JsonTest : public ::testing::Test {
protected:
    std::filesystem::path temp_dir;

    void SetUp() override {
        temp_dir = std::filesystem::temp_directory_path() / "bp3d_test";
        std::filesystem::create_directories(temp_dir);
    }

    void TearDown() override { std::filesystem::remove_all(temp_dir); }
};

TEST_F(JsonTest, ItemsToFromJson) {
    std::vector<Item> items;
    Item item{"test_item", {10.0, 20.0, 30.0}};
    item.quantity = 3;
    item.weight = 5.0;
    item.constraints.this_side_up = true;
    items.push_back(item);

    auto json = items_to_json(items);
    auto restored = items_from_json(json);

    ASSERT_EQ(restored.size(), 1);
    EXPECT_EQ(restored[0].id, item.id);
    EXPECT_DOUBLE_EQ(restored[0].dimensions.width, item.dimensions.width);
    EXPECT_DOUBLE_EQ(restored[0].dimensions.height, item.dimensions.height);
    EXPECT_DOUBLE_EQ(restored[0].dimensions.depth, item.dimensions.depth);
    EXPECT_EQ(restored[0].quantity, item.quantity);
}

TEST_F(JsonTest, BinTypesToFromJson) {
    std::vector<BinType> bins;
    BinType bin{"standard", {100.0, 100.0, 100.0}, 500.0, 10.0};
    bins.push_back(bin);

    auto json = bin_types_to_json(bins);
    auto restored = bin_types_from_json(json);

    ASSERT_EQ(restored.size(), 1);
    EXPECT_EQ(restored[0].id, bin.id);
    EXPECT_DOUBLE_EQ(restored[0].dimensions.width, bin.dimensions.width);
    EXPECT_DOUBLE_EQ(restored[0].dimensions.height, bin.dimensions.height);
    EXPECT_DOUBLE_EQ(restored[0].dimensions.depth, bin.dimensions.depth);
    EXPECT_DOUBLE_EQ(restored[0].max_weight, bin.max_weight);
    EXPECT_DOUBLE_EQ(restored[0].cost, bin.cost);
}

TEST_F(JsonTest, ResultToFromJson) {
    PackingResult result;
    result.algorithm = "Test Algorithm";
    result.bins_used = 2;
    result.placements.push_back(
        Placement{"item1", "bin1", 0, {0.0, 0.0, 0.0}, Rotation::WHD, {10.0, 10.0, 10.0}});
    result.unpacked_items.push_back("item2");
    result.stats.total_item_volume = 1000.0;
    result.stats.total_bin_volume = 10000.0;

    auto json = result_to_json(result);

    EXPECT_FALSE(json.empty());
    EXPECT_NE(json.find("Test Algorithm"), std::string::npos);
}

TEST_F(JsonTest, LoadInputFile) {
    // Create a test input file
    auto file_path = temp_dir / "test_input.json";
    std::ofstream out(file_path);
    out << R"({
        "items": [
            {"id": "item1", "width": 10, "height": 20, "depth": 30}
        ],
        "bins": [
            {"id": "bin1", "width": 100, "height": 100, "depth": 100}
        ]
    })";
    out.close();

    auto loaded = load_input_file(file_path.string());

    EXPECT_EQ(loaded.items.size(), 1);
    EXPECT_EQ(loaded.bins.size(), 1);
    EXPECT_EQ(loaded.items[0].id, "item1");
    EXPECT_EQ(loaded.bins[0].id, "bin1");
}

TEST_F(JsonTest, SaveResultFile) {
    PackingResult result;
    result.algorithm = "FFD";
    result.bins_used = 1;
    result.placements.push_back(
        Placement{"item1", "bin1", 0, {0.0, 0.0, 0.0}, Rotation::WHD, {10.0, 10.0, 10.0}});

    auto file_path = temp_dir / "test_result.json";
    save_result_file(result, file_path.string());

    // Verify file exists and has content
    std::ifstream in(file_path);
    std::string content((std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>());
    EXPECT_FALSE(content.empty());
    EXPECT_NE(content.find("FFD"), std::string::npos);
}

TEST_F(JsonTest, ParseInputFromJson) {
    std::string json_str = R"({
        "items": [
            {"id": "box1", "width": 10, "height": 20, "depth": 30}
        ],
        "bins": [
            {"id": "container", "width": 100, "height": 100, "depth": 100}
        ]
    })";

    auto input = input_from_json(json_str);

    EXPECT_EQ(input.items.size(), 1);
    EXPECT_EQ(input.bins.size(), 1);
    EXPECT_EQ(input.items[0].id, "box1");
    EXPECT_DOUBLE_EQ(input.items[0].dimensions.width, 10.0);
}

TEST_F(JsonTest, HandleMissingOptionalFields) {
    std::string json_str = R"({
        "items": [
            {"id": "minimal", "width": 10, "height": 10, "depth": 10}
        ]
    })";

    auto input = input_from_json(json_str);

    EXPECT_EQ(input.items.size(), 1);
    EXPECT_EQ(input.items[0].quantity, 1);  // Default
}

}  // namespace bp3d::test
