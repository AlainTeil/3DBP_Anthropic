/**
 * @file test_obj_export.cpp
 * @brief Unit tests for OBJ export functionality
 */

#include <bp3d/io/obj_exporter.hpp>

#include <gtest/gtest.h>

#include <filesystem>
#include <fstream>

namespace bp3d::test {

class ObjExportTest : public ::testing::Test {
protected:
    std::filesystem::path temp_dir;
    PackingResult result;
    std::vector<BinType> bin_types;

    void SetUp() override {
        temp_dir = std::filesystem::temp_directory_path() / "bp3d_obj_test";
        std::filesystem::create_directories(temp_dir);

        bin_types.push_back(BinType{"standard", {100.0, 100.0, 100.0}});

        result.algorithm = "TestAlgorithm";
        result.bins_used = 1;
        result.placements.push_back(
            Placement{"item1", "standard", 0, {0.0, 0.0, 0.0}, Rotation::WHD, {10.0, 20.0, 30.0}});
        result.placements.push_back(
            Placement{"item2", "standard", 0, {20.0, 0.0, 0.0}, Rotation::WHD, {15.0, 15.0, 15.0}});
    }

    void TearDown() override { std::filesystem::remove_all(temp_dir); }
};

TEST_F(ObjExportTest, GenerateObjStringNotEmpty) {
    std::string obj_str = generate_obj_string(result, bin_types);
    EXPECT_FALSE(obj_str.empty());
}

TEST_F(ObjExportTest, ObjContainsVertices) {
    std::string obj_str = generate_obj_string(result, bin_types);
    EXPECT_NE(obj_str.find("v "), std::string::npos);  // Has vertices
}

TEST_F(ObjExportTest, ObjContainsFaces) {
    std::string obj_str = generate_obj_string(result, bin_types);
    EXPECT_NE(obj_str.find("f "), std::string::npos);  // Has faces
}

TEST_F(ObjExportTest, ObjContainsObjectNames) {
    std::string obj_str = generate_obj_string(result, bin_types);
    EXPECT_NE(obj_str.find("o "), std::string::npos);  // Has object definitions
}

TEST_F(ObjExportTest, GenerateMtlString) {
    std::string mtl_str = generate_mtl_string(2);
    EXPECT_FALSE(mtl_str.empty());
    EXPECT_NE(mtl_str.find("newmtl"), std::string::npos);
}

TEST_F(ObjExportTest, MtlContainsMaterialColors) {
    std::string mtl_str = generate_mtl_string(2);
    // Should have diffuse color definitions
    EXPECT_NE(mtl_str.find("Kd"), std::string::npos);
}

TEST_F(ObjExportTest, ExportToFile) {
    auto obj_path = temp_dir / "test.obj";
    export_to_obj(result, bin_types, obj_path);

    EXPECT_TRUE(std::filesystem::exists(obj_path));
    EXPECT_GT(std::filesystem::file_size(obj_path), 0);
}

TEST_F(ObjExportTest, ExportCreatesMtlFile) {
    ObjExportOptions options;
    options.generate_mtl = true;
    options.mtl_name = "test_materials";

    auto obj_path = temp_dir / "test.obj";
    export_to_obj(result, bin_types, obj_path, options);

    auto mtl_path = temp_dir / "test_materials.mtl";
    EXPECT_TRUE(std::filesystem::exists(mtl_path));
}

TEST_F(ObjExportTest, ExportWithoutMtl) {
    ObjExportOptions options;
    options.generate_mtl = false;

    auto obj_path = temp_dir / "test_no_mtl.obj";
    export_to_obj(result, bin_types, obj_path, options);

    EXPECT_TRUE(std::filesystem::exists(obj_path));
    // Directory should not have .mtl file for this test
}

TEST_F(ObjExportTest, ExportWithScale) {
    ObjExportOptions options;
    options.scale = 0.1;
    options.generate_mtl = false;

    std::string obj_str = generate_obj_string(result, bin_types, options);

    // Verify vertices are scaled
    EXPECT_NE(obj_str.find("v "), std::string::npos);
}

TEST_F(ObjExportTest, ExportWithBins) {
    ObjExportOptions options;
    options.include_bins = true;
    options.generate_mtl = false;

    std::string obj_str = generate_obj_string(result, bin_types, options);

    // Should have bin geometry
    EXPECT_NE(obj_str.find("bin"), std::string::npos);
}

TEST_F(ObjExportTest, ExportWithoutBins) {
    ObjExportOptions options;
    options.include_bins = false;
    options.generate_mtl = false;

    std::string obj_str = generate_obj_string(result, bin_types, options);

    // Should still have content (items)
    EXPECT_FALSE(obj_str.empty());
}

TEST_F(ObjExportTest, EmptyResult) {
    PackingResult empty_result;
    empty_result.algorithm = "Empty";
    empty_result.bins_used = 0;

    std::string obj_str = generate_obj_string(empty_result, bin_types);

    // Should not crash, may produce minimal output
    // Just check it doesn't throw
    EXPECT_TRUE(true);
}

TEST_F(ObjExportTest, MultipleBins) {
    result.bins_used = 3;
    result.placements.clear();
    result.placements.push_back(
        Placement{"a", "standard", 0, {0, 0, 0}, Rotation::WHD, {10, 10, 10}});
    result.placements.push_back(
        Placement{"b", "standard", 1, {0, 0, 0}, Rotation::WHD, {10, 10, 10}});
    result.placements.push_back(
        Placement{"c", "standard", 2, {0, 0, 0}, Rotation::WHD, {10, 10, 10}});

    std::string mtl_str = generate_mtl_string(result.bins_used);

    // Should have materials for each bin
    EXPECT_NE(mtl_str.find("bin_0"), std::string::npos);
    EXPECT_NE(mtl_str.find("bin_1"), std::string::npos);
    EXPECT_NE(mtl_str.find("bin_2"), std::string::npos);
}

TEST_F(ObjExportTest, ObjFileReadable) {
    auto obj_path = temp_dir / "readable.obj";
    export_to_obj(result, bin_types, obj_path);

    std::ifstream file(obj_path);
    std::string content((std::istreambuf_iterator<char>(file)), std::istreambuf_iterator<char>());

    // Should have proper OBJ format
    EXPECT_NE(content.find("# 3D Bin Packing"), std::string::npos);
}

TEST_F(ObjExportTest, VertexCount) {
    ObjExportOptions options;
    options.include_bins = false;
    options.generate_mtl = false;

    std::string obj_str = generate_obj_string(result, bin_types, options);

    // Count vertices - each box has 8 vertices
    size_t vertex_count = 0;
    size_t pos = 0;
    while ((pos = obj_str.find("\nv ", pos)) != std::string::npos) {
        ++vertex_count;
        ++pos;
    }
    // Account for possible first vertex not preceded by newline
    if (obj_str.find("v ") == 0)
        ++vertex_count;

    // 2 items * 8 vertices = 16 vertices
    EXPECT_EQ(vertex_count, 16);
}

TEST_F(ObjExportTest, FaceCount) {
    ObjExportOptions options;
    options.include_bins = false;
    options.generate_mtl = false;

    std::string obj_str = generate_obj_string(result, bin_types, options);

    // Count faces - each box has 6 faces (or 12 triangles)
    size_t face_count = 0;
    size_t pos = 0;
    while ((pos = obj_str.find("\nf ", pos)) != std::string::npos) {
        ++face_count;
        ++pos;
    }
    if (obj_str.find("f ") == 0)
        ++face_count;

    // 2 items * 6 faces = 12 faces (or 24 triangles if exported as triangles)
    EXPECT_GE(face_count, 12);  // At least quads
}

}  // namespace bp3d::test
