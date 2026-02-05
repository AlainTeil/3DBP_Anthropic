/**
 * @file ffd.cpp
 * @brief First Fit Decreasing algorithm implementation
 */

#include "bp3d/algorithms/ffd.hpp"

#include "bp3d/constraints/collision.hpp"
#include "bp3d/rotation.hpp"
#include "internal/sorting.hpp"

#include <algorithm>
#include <chrono>
#include <vector>

namespace bp3d {

namespace {

/// State of a single bin during packing
struct BinState {
    BinType type;
    int index;
    std::vector<Placement> placements;
    double used_weight;
    double used_volume;
};

/// Try to find a position for the item in the bin
std::optional<Placement> find_placement(const Item& item, BinState& bin) {
    const auto allowed = get_allowed_rotations(item.constraints);

    // Check weight limit
    if (bin.used_weight + item.weight > bin.type.max_weight) {
        return std::nullopt;
    }

    // Try each allowed rotation
    for (std::size_t ri = 0; ri < kRotationCount; ++ri) {
        if (!allowed.test(ri))
            continue;

        Rotation rotation = rotation_from_index(ri);
        Dimensions rotated = apply_rotation(item.dimensions, rotation);

        // Quick check: does it fit in the bin at all?
        if (rotated.width > bin.type.dimensions.width ||
            rotated.height > bin.type.dimensions.height ||
            rotated.depth > bin.type.dimensions.depth) {
            continue;
        }

        // Try corner positions for efficient placement
        // (Grid-based fallback could be added with configurable step size)

        // First try corner positions (optimization)
        std::vector<Vector3> candidates;
        candidates.push_back({0, 0, 0});

        // Add positions at corners of existing items
        for (const auto& p : bin.placements) {
            // Right of item
            candidates.push_back({p.position.x + p.rotated_dimensions.width, 0, 0});
            candidates.push_back({p.position.x + p.rotated_dimensions.width, p.position.y, 0});
            candidates.push_back(
                {p.position.x + p.rotated_dimensions.width, p.position.y, p.position.z});

            // On top of item
            candidates.push_back(
                {p.position.x, p.position.y + p.rotated_dimensions.height, p.position.z});
            candidates.push_back({0, p.position.y + p.rotated_dimensions.height, 0});

            // Behind item
            candidates.push_back(
                {p.position.x, p.position.y, p.position.z + p.rotated_dimensions.depth});
            candidates.push_back({0, 0, p.position.z + p.rotated_dimensions.depth});
        }

        // Sort by bottom-left-back preference (y, then z, then x)
        std::sort(candidates.begin(), candidates.end(), [](const Vector3& a, const Vector3& b) {
            if (a.y != b.y)
                return a.y < b.y;
            if (a.z != b.z)
                return a.z < b.z;
            return a.x < b.x;
        });

        // Try each candidate position
        for (const auto& pos : candidates) {
            // Check bounds
            if (pos.x + rotated.width > bin.type.dimensions.width ||
                pos.y + rotated.height > bin.type.dimensions.height ||
                pos.z + rotated.depth > bin.type.dimensions.depth) {
                continue;
            }

            Placement proposed{item.id, bin.type.id, bin.index, pos, rotation, rotated};

            // Check for collisions
            bool valid = true;
            for (const auto& existing : bin.placements) {
                if (placements_overlap(proposed, existing)) {
                    valid = false;
                    break;
                }
            }

            if (valid) {
                // Check support (must be on floor or supported by another item)
                bool supported = (pos.y < 1e-6);  // On floor
                if (!supported) {
                    for (const auto& existing : bin.placements) {
                        double existing_top =
                            existing.position.y + existing.rotated_dimensions.height;
                        if (std::abs(existing_top - pos.y) < 1e-6) {
                            // Check XZ overlap
                            if (existing.position.x < pos.x + rotated.width &&
                                pos.x < existing.position.x + existing.rotated_dimensions.width &&
                                existing.position.z < pos.z + rotated.depth &&
                                pos.z < existing.position.z + existing.rotated_dimensions.depth) {
                                supported = true;
                                break;
                            }
                        }
                    }
                }

                if (supported) {
                    return proposed;
                }
            }
        }
    }

    return std::nullopt;
}

}  // namespace

PackingResult FirstFitDecreasing::solve(std::span<const Item> items, const SolverConfig& config) {
    auto start_time = std::chrono::steady_clock::now();

    PackingResult result;
    result.algorithm = std::string(name());

    if (items.empty() || config.bin_types.empty()) {
        return result;
    }

    // Expand quantities and sort by volume descending
    auto expanded = internal::expand_quantities(items);
    internal::sort_items(expanded, internal::SortCriteria::VolumeDescending);

    // Track bins
    std::vector<BinState> bins;
    std::vector<std::string> unpacked;

    // Process each item
    for (const auto& item : expanded) {
        bool placed = false;

        // Try existing bins (first fit)
        for (auto& bin : bins) {
            auto placement = find_placement(item, bin);
            if (placement) {
                bin.placements.push_back(*placement);
                bin.used_weight += item.weight;
                bin.used_volume += item.volume();
                result.placements.push_back(*placement);
                placed = true;
                break;
            }
        }

        // If not placed, try a new bin
        if (!placed && config.allow_multiple_bins) {
            // Find smallest bin that can fit the item
            for (const auto& bin_type : config.bin_types) {
                BinState new_bin{bin_type, static_cast<int>(bins.size()), {}, 0.0, 0.0};

                auto placement = find_placement(item, new_bin);
                if (placement) {
                    new_bin.placements.push_back(*placement);
                    new_bin.used_weight += item.weight;
                    new_bin.used_volume += item.volume();
                    bins.push_back(std::move(new_bin));
                    result.placements.push_back(*placement);
                    placed = true;
                    break;
                }
            }
        }

        if (!placed) {
            unpacked.push_back(item.id);
        }
    }

    // Compute statistics
    result.bins_used = static_cast<int>(bins.size());
    result.unpacked_items = std::move(unpacked);

    double total_item_volume = 0.0;
    double total_bin_volume = 0.0;
    for (const auto& bin : bins) {
        total_item_volume += bin.used_volume;
        total_bin_volume += bin.type.volume();
    }

    result.stats.total_item_volume = total_item_volume;
    result.stats.total_bin_volume = total_bin_volume;
    result.stats.utilization = total_bin_volume > 0 ? total_item_volume / total_bin_volume : 0.0;

    auto end_time = std::chrono::steady_clock::now();
    result.stats.execution_time =
        std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time);

    return result;
}

}  // namespace bp3d
