/**
 * @file result.hpp
 * @brief Packing result types: Placement, PackingResult
 */

#ifndef BP3D_RESULT_HPP
#define BP3D_RESULT_HPP

#include "bp3d/rotation.hpp"
#include "bp3d/types.hpp"

#include <chrono>
#include <string>
#include <vector>

namespace bp3d {

/**
 * @brief A placement of an item within a bin
 */
struct Placement {
    std::string item_id;            ///< ID of the placed item
    std::string bin_id;             ///< ID of the bin containing this item
    int bin_index = 0;              ///< Index of this bin instance (0-based)
    Vector3 position;               ///< Position of the item's corner (min x, y, z)
    Rotation rotation;              ///< Applied rotation
    Dimensions rotated_dimensions;  ///< Dimensions after rotation

    /// Get the maximum corner position
    [[nodiscard]] Vector3 max_corner() const noexcept {
        return {position.x + rotated_dimensions.width, position.y + rotated_dimensions.height,
                position.z + rotated_dimensions.depth};
    }
};

/**
 * @brief Statistics for a packing result
 */
struct PackingStats {
    double utilization = 0.0;                     ///< Volume utilization ratio (0-1)
    double total_item_volume = 0.0;               ///< Total volume of packed items
    double total_bin_volume = 0.0;                ///< Total volume of used bins
    std::chrono::microseconds execution_time{0};  ///< Time taken to compute
};

/**
 * @brief Result of a packing operation
 */
struct PackingResult {
    std::vector<Placement> placements;        ///< All item placements
    std::vector<std::string> unpacked_items;  ///< IDs of items that couldn't be packed
    int bins_used = 0;                        ///< Number of bins used
    PackingStats stats;                       ///< Packing statistics
    std::string algorithm;                    ///< Name of algorithm used

    /// Check if packing was completely successful
    [[nodiscard]] bool is_complete() const noexcept { return unpacked_items.empty(); }

    /// Get utilization as a percentage
    [[nodiscard]] double utilization_percent() const noexcept { return stats.utilization * 100.0; }
};

/**
 * @brief Information about a bin instance used in packing
 */
struct UsedBin {
    std::string type_id;           ///< ID of the bin type
    int index = 0;                 ///< Instance index
    Dimensions dimensions;         ///< Bin dimensions
    double used_volume = 0.0;      ///< Volume occupied by items
    double used_weight = 0.0;      ///< Total weight of items
    std::vector<Placement> items;  ///< Items in this bin

    /// Get utilization ratio for this bin
    [[nodiscard]] double utilization() const noexcept {
        double total = dimensions.volume();
        return total > 0.0 ? used_volume / total : 0.0;
    }
};

}  // namespace bp3d

#endif  // BP3D_RESULT_HPP
