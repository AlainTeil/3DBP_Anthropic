/**
 * @file obj_exporter.cpp
 * @brief Wavefront OBJ export implementation
 */

#include "bp3d/io/obj_exporter.hpp"

#include "bp3d/result.hpp"
#include "bp3d/types.hpp"

#include <array>
#include <cstddef>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <ios>
#include <map>
#include <span>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace bp3d {

namespace {

/// Pre-defined colors for different bins
constexpr std::array<std::array<double, 3>, 12> kBinColors = {{
    {0.8, 0.2, 0.2},  // Red
    {0.2, 0.8, 0.2},  // Green
    {0.2, 0.2, 0.8},  // Blue
    {0.8, 0.8, 0.2},  // Yellow
    {0.8, 0.2, 0.8},  // Magenta
    {0.2, 0.8, 0.8},  // Cyan
    {0.8, 0.5, 0.2},  // Orange
    {0.5, 0.2, 0.8},  // Purple
    {0.2, 0.8, 0.5},  // Teal
    {0.8, 0.8, 0.8},  // Light gray
    {0.5, 0.5, 0.5},  // Gray
    {0.3, 0.3, 0.3},  // Dark gray
}};

/// Get color for a bin index
std::array<double, 3> get_bin_color(int bin_index) {
    return kBinColors[static_cast<std::size_t>(bin_index) % kBinColors.size()];
}

/// Write a box to OBJ format
void write_box_obj(std::ostringstream& oss, const Vector3& min, const Vector3& max,
                   int& vertex_offset, double scale) {
    // 8 vertices of a box
    std::array<std::array<double, 3>, 8> const vertices = {{
        {min.x * scale, min.y * scale, min.z * scale},
        {max.x * scale, min.y * scale, min.z * scale},
        {max.x * scale, max.y * scale, min.z * scale},
        {min.x * scale, max.y * scale, min.z * scale},
        {min.x * scale, min.y * scale, max.z * scale},
        {max.x * scale, min.y * scale, max.z * scale},
        {max.x * scale, max.y * scale, max.z * scale},
        {min.x * scale, max.y * scale, max.z * scale},
    }};

    for (const auto& v : vertices) {
        oss << "v " << std::fixed << std::setprecision(6) << v[0] << " " << v[1] << " " << v[2]
            << "\n";
    }

    // 6 faces (quads)
    int const o = vertex_offset;
    // Front face
    oss << "f " << o + 1 << " " << o + 2 << " " << o + 3 << " " << o + 4 << "\n";
    // Back face
    oss << "f " << o + 5 << " " << o + 8 << " " << o + 7 << " " << o + 6 << "\n";
    // Left face
    oss << "f " << o + 1 << " " << o + 4 << " " << o + 8 << " " << o + 5 << "\n";
    // Right face
    oss << "f " << o + 2 << " " << o + 6 << " " << o + 7 << " " << o + 3 << "\n";
    // Top face
    oss << "f " << o + 4 << " " << o + 3 << " " << o + 7 << " " << o + 8 << "\n";
    // Bottom face
    oss << "f " << o + 1 << " " << o + 5 << " " << o + 6 << " " << o + 2 << "\n";

    vertex_offset += 8;
}

/// Write a wireframe box (for bins)
void write_wireframe_obj(std::ostringstream& oss, const Vector3& min, const Vector3& max,
                         int& vertex_offset, double scale) {
    // Just vertices for wireframe
    std::array<std::array<double, 3>, 8> const vertices = {{
        {min.x * scale, min.y * scale, min.z * scale},
        {max.x * scale, min.y * scale, min.z * scale},
        {max.x * scale, max.y * scale, min.z * scale},
        {min.x * scale, max.y * scale, min.z * scale},
        {min.x * scale, min.y * scale, max.z * scale},
        {max.x * scale, min.y * scale, max.z * scale},
        {max.x * scale, max.y * scale, max.z * scale},
        {min.x * scale, max.y * scale, max.z * scale},
    }};

    for (const auto& v : vertices) {
        oss << "v " << std::fixed << std::setprecision(6) << v[0] << " " << v[1] << " " << v[2]
            << "\n";
    }

    // Edges as line elements
    int const o = vertex_offset;
    // Bottom edges
    oss << "l " << o + 1 << " " << o + 2 << "\n";
    oss << "l " << o + 2 << " " << o + 6 << "\n";
    oss << "l " << o + 6 << " " << o + 5 << "\n";
    oss << "l " << o + 5 << " " << o + 1 << "\n";
    // Top edges
    oss << "l " << o + 4 << " " << o + 3 << "\n";
    oss << "l " << o + 3 << " " << o + 7 << "\n";
    oss << "l " << o + 7 << " " << o + 8 << "\n";
    oss << "l " << o + 8 << " " << o + 4 << "\n";
    // Vertical edges
    oss << "l " << o + 1 << " " << o + 4 << "\n";
    oss << "l " << o + 2 << " " << o + 3 << "\n";
    oss << "l " << o + 6 << " " << o + 7 << "\n";
    oss << "l " << o + 5 << " " << o + 8 << "\n";

    vertex_offset += 8;
}

}  // namespace

std::string generate_mtl_string(int num_bins) {
    std::ostringstream oss;
    oss << "# Material file for 3D bin packing visualization\n\n";

    // Bin wireframe material
    oss << "newmtl bin_wireframe\n";
    oss << "Kd 0.3 0.3 0.3\n";
    oss << "Ka 0.1 0.1 0.1\n";
    oss << "Ks 0.0 0.0 0.0\n";
    oss << "d 0.5\n\n";

    // Materials for each bin
    for (int i = 0; i < num_bins; ++i) {
        auto color = get_bin_color(i);
        oss << "newmtl bin_" << i << "\n";
        oss << "Kd " << color[0] << " " << color[1] << " " << color[2] << "\n";
        oss << "Ka " << color[0] * 0.3 << " " << color[1] * 0.3 << " " << color[2] * 0.3 << "\n";
        oss << "Ks 0.2 0.2 0.2\n";
        oss << "Ns 50\n";
        oss << "d 0.9\n\n";
    }

    return oss.str();
}

std::string generate_obj_string(const PackingResult& result, std::span<const BinType> bin_types,
                                const ObjExportOptions& options) {
    std::ostringstream oss;
    oss << "# 3D Bin Packing Visualization\n";
    oss << "# Algorithm: " << result.algorithm << "\n";
    oss << "# Bins used: " << result.bins_used << "\n";
    oss << "# Items packed: " << result.placements.size() << "\n";
    oss << "# Utilization: " << std::fixed << std::setprecision(2) << result.stats.utilization * 100
        << "%\n\n";

    if (options.generate_mtl) {
        oss << "mtllib " << options.mtl_name << ".mtl\n\n";
    }

    int vertex_offset = 1;

    // Build map of bin types by ID
    std::map<std::string, const BinType*> bin_type_map;
    for (const auto& bt : bin_types) {
        bin_type_map[bt.id] = &bt;
    }

    // Group placements by bin
    std::map<std::pair<std::string, int>, std::vector<const Placement*>> bins_map;
    for (const auto& p : result.placements) {
        bins_map[{p.bin_id, p.bin_index}].push_back(&p);
    }

    // Offset for multiple bins (place them side by side)
    double bin_offset_x = 0.0;
    double const spacing = 10.0;

    int bin_num = 0;
    for (const auto& [key, placements] : bins_map) {
        const auto& [bin_id, bin_index] = key;

        oss << "# Bin " << bin_id << " (index " << bin_index << ")\n";
        oss << "g bin_" << bin_id << "_" << bin_index << "\n";

        // Find bin dimensions
        Dimensions bin_dims{100, 100, 100};  // Default
        auto it = bin_type_map.find(bin_id);
        if (it != bin_type_map.end()) {
            bin_dims = it->second->dimensions;
        }

        // Draw bin wireframe
        if (options.include_bins) {
            oss << "o bin_" << bin_id << "_" << bin_index << "_frame\n";
            if (options.generate_mtl) {
                oss << "usemtl bin_wireframe\n";
            }
            Vector3 const bin_min{bin_offset_x, 0, 0};
            Vector3 const bin_max{bin_offset_x + bin_dims.width, bin_dims.height, bin_dims.depth};
            write_wireframe_obj(oss, bin_min, bin_max, vertex_offset, options.scale);
        }

        // Draw items
        if (options.generate_mtl) {
            oss << "usemtl bin_" << bin_num << "\n";
        }

        for (const auto* p : placements) {
            oss << "o " << p->item_id << "\n";
            Vector3 const item_min{bin_offset_x + p->position.x, p->position.y, p->position.z};
            Vector3 const item_max{bin_offset_x + p->position.x + p->rotated_dimensions.width,
                                   p->position.y + p->rotated_dimensions.height,
                                   p->position.z + p->rotated_dimensions.depth};
            write_box_obj(oss, item_min, item_max, vertex_offset, options.scale);
        }

        oss << "\n";
        bin_offset_x += bin_dims.width + spacing;
        bin_num++;
    }

    return oss.str();
}

void export_to_obj(const PackingResult& result, std::span<const BinType> bin_types,
                   const std::filesystem::path& path, const ObjExportOptions& options) {
    // Write OBJ file
    std::ofstream obj_file(path);
    if (!obj_file) {
        throw std::runtime_error("Cannot create OBJ file: " + path.string());
    }
    obj_file << generate_obj_string(result, bin_types, options);

    // Write MTL file if requested
    if (options.generate_mtl) {
        auto mtl_path = path.parent_path() / (options.mtl_name + ".mtl");
        std::ofstream mtl_file(mtl_path);
        if (!mtl_file) {
            throw std::runtime_error("Cannot create MTL file: " + mtl_path.string());
        }
        mtl_file << generate_mtl_string(result.bins_used);
    }
}

}  // namespace bp3d
