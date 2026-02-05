/**
 * @file obj_exporter.hpp
 * @brief Wavefront OBJ export for visualization
 */

#ifndef BP3D_IO_OBJ_EXPORTER_HPP
#define BP3D_IO_OBJ_EXPORTER_HPP

#include "bp3d/result.hpp"
#include "bp3d/types.hpp"

#include <filesystem>
#include <string>

namespace bp3d {

/**
 * @brief Options for OBJ export
 */
struct ObjExportOptions {
    /// Include bin wireframe boxes
    bool include_bins = true;

    /// Scale factor for coordinates
    double scale = 1.0;

    /// Generate MTL file with colors
    bool generate_mtl = true;

    /// Base name for MTL file (without extension)
    std::string mtl_name = "packing";
};

/**
 * @brief Export packing result to Wavefront OBJ format
 *
 * Creates an OBJ file with boxes for each placed item.
 * Different bins are colored differently for visualization.
 *
 * @param result Packing result to export
 * @param bin_types Bin type definitions (for dimensions)
 * @param path Output file path
 * @param options Export options
 */
void export_to_obj(const PackingResult& result, std::span<const BinType> bin_types,
                   const std::filesystem::path& path, const ObjExportOptions& options = {});

/**
 * @brief Generate OBJ content as string
 */
[[nodiscard]] std::string generate_obj_string(const PackingResult& result,
                                              std::span<const BinType> bin_types,
                                              const ObjExportOptions& options = {});

/**
 * @brief Generate MTL (material) content as string
 *
 * Creates different colored materials for each bin.
 */
[[nodiscard]] std::string generate_mtl_string(int num_bins);

}  // namespace bp3d

#endif  // BP3D_IO_OBJ_EXPORTER_HPP
