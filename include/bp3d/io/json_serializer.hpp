/**
 * @file json_serializer.hpp
 * @brief JSON serialization for bp3d types
 */

#ifndef BP3D_IO_JSON_SERIALIZER_HPP
#define BP3D_IO_JSON_SERIALIZER_HPP

#include "bp3d/result.hpp"
#include "bp3d/types.hpp"

#include <filesystem>
#include <string>
#include <vector>

namespace bp3d {

/**
 * @brief Input data structure for JSON parsing
 */
struct PackingInput {
    std::vector<Item> items;
    std::vector<BinType> bins;
};

/**
 * @brief Serialize items to JSON string
 */
[[nodiscard]] std::string items_to_json(std::span<const Item> items);

/**
 * @brief Serialize bin types to JSON string
 */
[[nodiscard]] std::string bin_types_to_json(std::span<const BinType> bins);

/**
 * @brief Serialize packing result to JSON string
 */
[[nodiscard]] std::string result_to_json(const PackingResult& result);

/**
 * @brief Parse items from JSON string
 */
[[nodiscard]] std::vector<Item> items_from_json(std::string_view json);

/**
 * @brief Parse bin types from JSON string
 */
[[nodiscard]] std::vector<BinType> bin_types_from_json(std::string_view json);

/**
 * @brief Parse packing input from JSON string
 *
 * Expects JSON with "items" and optionally "bins" arrays.
 */
[[nodiscard]] PackingInput input_from_json(std::string_view json);

/**
 * @brief Load packing input from JSON file
 */
[[nodiscard]] PackingInput load_input_file(const std::filesystem::path& path);

/**
 * @brief Save packing result to JSON file
 */
void save_result_file(const PackingResult& result, const std::filesystem::path& path);

}  // namespace bp3d

#endif  // BP3D_IO_JSON_SERIALIZER_HPP
