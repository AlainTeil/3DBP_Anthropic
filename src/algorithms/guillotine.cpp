/**
 * @file guillotine.cpp
 * @brief Guillotine algorithm implementation
 */

#include "bp3d/algorithms/guillotine.hpp"

#include "bp3d/rotation.hpp"
#include "internal/free_space.hpp"
#include "internal/sorting.hpp"

#include <algorithm>
#include <chrono>
#include <limits>
#include <vector>

namespace bp3d {

namespace {

/// Represents a 3D free rectangle for guillotine packing
struct FreeRect3D {
    Vector3 position;
    Dimensions size;
};

/// Split a free rectangle after placing an item
void guillotine_split(std::vector<FreeRect3D>& free_rects, const FreeRect3D& rect,
                      const Vector3& item_pos, const Dimensions& item_size,
                      GuillotineSplitRule /*split_rule*/) {
    // After placing an item, we create up to 3 new free rectangles
    // The split rule determines how we divide the remaining space
    // TODO: Implement different splitting strategies based on split_rule

    double remaining_x = rect.size.width - item_size.width;
    double remaining_y = rect.size.height - item_size.height;
    double remaining_z = rect.size.depth - item_size.depth;

    // Simple splitting: create rectangles for remaining space in each direction
    // Right of item (X direction)
    if (remaining_x > 0) {
        free_rects.push_back({{item_pos.x + item_size.width, rect.position.y, rect.position.z},
                              {remaining_x, rect.size.height, rect.size.depth}});
    }

    // Above item (Y direction) - only the space directly above
    if (remaining_y > 0) {
        free_rects.push_back({{rect.position.x, item_pos.y + item_size.height, rect.position.z},
                              {item_size.width, remaining_y, rect.size.depth}});
    }

    // Behind item (Z direction) - only the space directly behind
    if (remaining_z > 0) {
        free_rects.push_back({{rect.position.x, rect.position.y, item_pos.z + item_size.depth},
                              {item_size.width, item_size.height, remaining_z}});
    }
}

/// Find best free rectangle for item
std::optional<std::pair<std::size_t, Rotation>>
find_best_rect(const std::vector<FreeRect3D>& free_rects, const Item& item,
               GuillotineFreeRectChoice choice_rule) {
    const auto allowed = get_allowed_rotations(item.constraints);
    std::optional<std::pair<std::size_t, Rotation>> best;
    double best_score = std::numeric_limits<double>::max();

    for (std::size_t ri = 0; ri < kRotationCount; ++ri) {
        if (!allowed.test(ri))
            continue;
        Rotation rot = rotation_from_index(ri);
        Dimensions rotated = apply_rotation(item.dimensions, rot);

        for (std::size_t i = 0; i < free_rects.size(); ++i) {
            const auto& rect = free_rects[i];

            if (rotated.width <= rect.size.width && rotated.height <= rect.size.height &&
                rotated.depth <= rect.size.depth) {
                double score = 0;

                switch (choice_rule) {
                    case GuillotineFreeRectChoice::BestShortSideFit: {
                        double leftover_w = rect.size.width - rotated.width;
                        double leftover_d = rect.size.depth - rotated.depth;
                        score = std::min(leftover_w, leftover_d);
                        break;
                    }
                    case GuillotineFreeRectChoice::BestLongSideFit: {
                        double leftover_w = rect.size.width - rotated.width;
                        double leftover_d = rect.size.depth - rotated.depth;
                        score = std::max(leftover_w, leftover_d);
                        break;
                    }
                    case GuillotineFreeRectChoice::BestAreaFit:
                        score = rect.size.volume() - rotated.volume();
                        break;
                    case GuillotineFreeRectChoice::WorstAreaFit:
                        score = -(rect.size.volume() - rotated.volume());
                        break;
                }

                // Prefer lower positions
                score += rect.position.y * 1e6;

                if (score < best_score) {
                    best_score = score;
                    best = std::make_pair(i, rot);
                }
            }
        }
    }

    return best;
}

struct BinState {
    BinType type;
    int index;
    std::vector<Placement> placements;
    std::vector<FreeRect3D> free_rects;
    double used_weight;
    double used_volume;
};

}  // namespace

GuillotineSolver::GuillotineSolver(GuillotineSplitRule split_rule,
                                   GuillotineFreeRectChoice choice_rule)
    : split_rule_(split_rule),
      choice_rule_(choice_rule) {}

PackingResult GuillotineSolver::solve(std::span<const Item> items, const SolverConfig& config) {
    auto start_time = std::chrono::steady_clock::now();

    PackingResult result;
    result.algorithm = std::string(name());

    if (items.empty() || config.bin_types.empty()) {
        return result;
    }

    auto expanded = internal::expand_quantities(items);
    internal::sort_items(expanded, internal::SortCriteria::VolumeDescending);

    std::vector<BinState> bins;
    std::vector<std::string> unpacked;

    for (const auto& item : expanded) {
        bool placed = false;

        // Try existing bins
        for (auto& bin : bins) {
            if (bin.used_weight + item.weight > bin.type.max_weight)
                continue;

            auto best = find_best_rect(bin.free_rects, item, choice_rule_);
            if (best) {
                auto [rect_idx, rotation] = *best;
                const auto& rect = bin.free_rects[rect_idx];
                Dimensions rotated = apply_rotation(item.dimensions, rotation);

                Placement placement{item.id,       bin.type.id, bin.index,
                                    rect.position, rotation,    rotated};

                // Remove used rect and add splits
                FreeRect3D used_rect = rect;
                Vector3 item_position = rect.position;  // Copy before erase
                bin.free_rects.erase(bin.free_rects.begin() +
                                     static_cast<std::ptrdiff_t>(rect_idx));
                guillotine_split(bin.free_rects, used_rect, item_position, rotated, split_rule_);

                bin.placements.push_back(placement);
                bin.used_weight += item.weight;
                bin.used_volume += item.volume();
                result.placements.push_back(placement);
                placed = true;
                break;
            }
        }

        // Try new bin
        if (!placed && config.allow_multiple_bins) {
            for (const auto& bin_type : config.bin_types) {
                BinState new_bin{bin_type, static_cast<int>(bins.size()),
                                 {},       {{{0, 0, 0}, bin_type.dimensions}},
                                 0.0,      0.0};

                if (new_bin.used_weight + item.weight > bin_type.max_weight)
                    continue;

                auto best = find_best_rect(new_bin.free_rects, item, choice_rule_);
                if (best) {
                    auto [rect_idx, rotation] = *best;
                    const auto& rect = new_bin.free_rects[rect_idx];
                    Dimensions rotated = apply_rotation(item.dimensions, rotation);

                    Placement placement{item.id,       bin_type.id, new_bin.index,
                                        rect.position, rotation,    rotated};

                    FreeRect3D used_rect = rect;
                    Vector3 item_position = rect.position;  // Copy before erase
                    new_bin.free_rects.erase(new_bin.free_rects.begin());
                    guillotine_split(new_bin.free_rects, used_rect, item_position, rotated,
                                     split_rule_);

                    new_bin.placements.push_back(placement);
                    new_bin.used_weight += item.weight;
                    new_bin.used_volume += item.volume();
                    result.placements.push_back(placement);
                    bins.push_back(std::move(new_bin));
                    placed = true;
                    break;
                }
            }
        }

        if (!placed) {
            unpacked.push_back(item.id);
        }
    }

    // Statistics
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
