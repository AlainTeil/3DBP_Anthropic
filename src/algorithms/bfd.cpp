/**
 * @file bfd.cpp
 * @brief Best Fit Decreasing algorithm implementation
 */

#include "bp3d/algorithms/bfd.hpp"

#include "bp3d/constraints/collision.hpp"
#include "bp3d/rotation.hpp"
#include "internal/sorting.hpp"

#include <algorithm>
#include <chrono>
#include <limits>
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

    double remaining_volume() const { return type.volume() - used_volume; }
};

/// Try to find best placement for the item in the bin
/// Returns placement and score (lower is better)
std::pair<std::optional<Placement>, double> find_best_placement(const Item& item, BinState& bin) {
    const auto allowed = get_allowed_rotations(item.constraints);

    // Check weight limit
    if (bin.used_weight + item.weight > bin.type.max_weight) {
        return {std::nullopt, std::numeric_limits<double>::max()};
    }

    std::optional<Placement> best_placement;
    double best_score = std::numeric_limits<double>::max();

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

        // Generate candidate positions
        std::vector<Vector3> candidates;
        candidates.push_back({0, 0, 0});

        for (const auto& p : bin.placements) {
            // Right of item
            candidates.push_back(
                {p.position.x + p.rotated_dimensions.width, p.position.y, p.position.z});
            candidates.push_back({p.position.x + p.rotated_dimensions.width, 0, 0});

            // On top of item
            candidates.push_back(
                {p.position.x, p.position.y + p.rotated_dimensions.height, p.position.z});
            candidates.push_back({0, p.position.y + p.rotated_dimensions.height, 0});

            // Behind item
            candidates.push_back(
                {p.position.x, p.position.y, p.position.z + p.rotated_dimensions.depth});
            candidates.push_back({0, 0, p.position.z + p.rotated_dimensions.depth});
        }

        for (const auto& pos : candidates) {
            // Check bounds
            if (pos.x < 0 || pos.y < 0 || pos.z < 0 ||
                pos.x + rotated.width > bin.type.dimensions.width ||
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

            if (!valid)
                continue;

            // Check support
            bool supported = (pos.y < 1e-6);
            if (!supported) {
                for (const auto& existing : bin.placements) {
                    double existing_top = existing.position.y + existing.rotated_dimensions.height;
                    if (std::abs(existing_top - pos.y) < 1e-6) {
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

            if (!supported)
                continue;

            // Score this placement: prefer lower positions, then less wasted space
            // Score = y + small penalty for higher x and z
            double score = pos.y * 1000.0 + pos.x + pos.z * 0.1;

            if (score < best_score) {
                best_score = score;
                best_placement = proposed;
            }
        }
    }

    return {best_placement, best_score};
}

}  // namespace

PackingResult BestFitDecreasing::solve(std::span<const Item> items, const SolverConfig& config) {
    auto start_time = std::chrono::steady_clock::now();

    PackingResult result;
    result.algorithm = std::string(name());

    if (items.empty() || config.bin_types.empty()) {
        return result;
    }

    // Expand quantities and sort by volume descending
    auto expanded = internal::expand_quantities(items);
    internal::sort_items(expanded, internal::SortCriteria::VolumeDescending);

    std::vector<BinState> bins;
    std::vector<std::string> unpacked;

    for (const auto& item : expanded) {
        // Find best bin for this item (one with least remaining space after placement)
        std::optional<Placement> best_placement;
        double best_remaining = std::numeric_limits<double>::max();
        std::size_t best_bin_index = 0;

        for (std::size_t i = 0; i < bins.size(); ++i) {
            auto [placement, score] = find_best_placement(item, bins[i]);
            if (placement) {
                double remaining_after = bins[i].remaining_volume() - item.volume();
                if (remaining_after < best_remaining) {
                    best_remaining = remaining_after;
                    best_placement = placement;
                    best_bin_index = i;
                }
            }
        }

        if (best_placement) {
            bins[best_bin_index].placements.push_back(*best_placement);
            bins[best_bin_index].used_weight += item.weight;
            bins[best_bin_index].used_volume += item.volume();
            result.placements.push_back(*best_placement);
        } else if (config.allow_multiple_bins) {
            // Try new bin - find smallest bin type that works
            bool placed = false;
            auto sorted_types = config.bin_types;
            std::sort(sorted_types.begin(), sorted_types.end(),
                      [](const BinType& a, const BinType& b) { return a.volume() < b.volume(); });

            for (const auto& bin_type : sorted_types) {
                BinState new_bin{bin_type, static_cast<int>(bins.size()), {}, 0.0, 0.0};

                auto [placement, score] = find_best_placement(item, new_bin);
                if (placement) {
                    new_bin.placements.push_back(*placement);
                    new_bin.used_weight += item.weight;
                    new_bin.used_volume += item.volume();
                    result.placements.push_back(*placement);
                    bins.push_back(std::move(new_bin));
                    placed = true;
                    break;
                }
            }

            if (!placed) {
                unpacked.push_back(item.id);
            }
        } else {
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
