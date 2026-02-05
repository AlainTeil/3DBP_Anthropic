/**
 * @file shelf.cpp
 * @brief Shelf algorithm implementation
 */

#include "bp3d/algorithms/shelf.hpp"

#include "bp3d/rotation.hpp"

#include <algorithm>
#include <chrono>
#include <limits>

namespace bp3d {

ShelfSolver::ShelfSolver(ShelfHeuristic heuristic) : heuristic_(heuristic) {}

void ShelfSolver::reset(const SolverConfig& config) {
    config_ = config;
    bins_.clear();
    unpacked_.clear();
    start_time_ = std::chrono::steady_clock::now();

    if (!config.bin_types.empty()) {
        add_bin();
    }
}

void ShelfSolver::add_bin() {
    if (config_.bin_types.empty())
        return;

    const auto& bin_type = config_.bin_types.front();

    BinState bin;
    bin.type = bin_type;
    bin.index = static_cast<int>(bins_.size());
    bin.used_weight = 0.0;

    bins_.push_back(std::move(bin));
}

std::optional<Placement> ShelfSolver::try_place_on_shelf(BinState& bin, Shelf& shelf,
                                                         const Item& item,
                                                         const Dimensions& rotated) {
    // Check if item fits on this shelf
    if (rotated.height > shelf.height) {
        return std::nullopt;
    }

    if (shelf.used_width + rotated.width > bin.type.dimensions.width) {
        return std::nullopt;
    }

    if (rotated.depth > bin.type.dimensions.depth) {
        return std::nullopt;
    }

    // Check weight
    if (bin.used_weight + item.weight > bin.type.max_weight) {
        return std::nullopt;
    }

    // Find the rotation that produces these dimensions
    const auto allowed = get_allowed_rotations(item.constraints);
    Rotation used_rotation = Rotation::WHD;

    for (std::size_t ri = 0; ri < kRotationCount; ++ri) {
        if (!allowed.test(ri))
            continue;
        Rotation rot = rotation_from_index(ri);
        Dimensions r = apply_rotation(item.dimensions, rot);
        if (r.width == rotated.width && r.height == rotated.height && r.depth == rotated.depth) {
            used_rotation = rot;
            break;
        }
    }

    Vector3 pos{shelf.used_width, shelf.y_position, 0.0};

    return Placement{item.id, bin.type.id, bin.index, pos, used_rotation, rotated};
}

std::optional<Placement> ShelfSolver::try_new_shelf(BinState& bin, const Item& item,
                                                    const Dimensions& rotated) {
    // Calculate current shelf height
    double current_height = 0.0;
    for (const auto& shelf : bin.shelves) {
        current_height = std::max(current_height, shelf.y_position + shelf.height);
    }

    // Check if there's room for a new shelf
    if (current_height + rotated.height > bin.type.dimensions.height) {
        return std::nullopt;
    }

    if (rotated.width > bin.type.dimensions.width || rotated.depth > bin.type.dimensions.depth) {
        return std::nullopt;
    }

    // Check weight
    if (bin.used_weight + item.weight > bin.type.max_weight) {
        return std::nullopt;
    }

    // Create new shelf
    Shelf new_shelf{current_height, rotated.height, 0.0};

    // Find the rotation
    const auto allowed = get_allowed_rotations(item.constraints);
    Rotation used_rotation = Rotation::WHD;

    for (std::size_t ri = 0; ri < kRotationCount; ++ri) {
        if (!allowed.test(ri))
            continue;
        Rotation rot = rotation_from_index(ri);
        Dimensions r = apply_rotation(item.dimensions, rot);
        if (r.width == rotated.width && r.height == rotated.height && r.depth == rotated.depth) {
            used_rotation = rot;
            break;
        }
    }

    Vector3 pos{0.0, current_height, 0.0};

    bin.shelves.push_back(new_shelf);

    return Placement{item.id, bin.type.id, bin.index, pos, used_rotation, rotated};
}

std::optional<Placement> ShelfSolver::pack_one(const Item& item) {
    const auto allowed = get_allowed_rotations(item.constraints);

    // Collect all valid rotations
    std::vector<Dimensions> rotations;
    for (std::size_t ri = 0; ri < kRotationCount; ++ri) {
        if (!allowed.test(ri))
            continue;
        Dimensions r = apply_rotation(item.dimensions, rotation_from_index(ri));
        // Avoid duplicates
        bool found = false;
        for (const auto& existing : rotations) {
            if (existing.width == r.width && existing.height == r.height &&
                existing.depth == r.depth) {
                found = true;
                break;
            }
        }
        if (!found) {
            rotations.push_back(r);
        }
    }

    // Sort rotations by height (prefer shorter items for shelves)
    std::sort(rotations.begin(), rotations.end(),
              [](const Dimensions& a, const Dimensions& b) { return a.height < b.height; });

    // Try each bin
    for (auto& bin : bins_) {
        for (const auto& rotated : rotations) {
            // Based on heuristic, try different shelf selection strategies
            switch (heuristic_) {
                case ShelfHeuristic::NextFit: {
                    // Only try the last shelf
                    if (!bin.shelves.empty()) {
                        auto placement = try_place_on_shelf(bin, bin.shelves.back(), item, rotated);
                        if (placement) {
                            bin.shelves.back().used_width += rotated.width;
                            bin.placements.push_back(*placement);
                            bin.used_weight += item.weight;
                            return placement;
                        }
                    }
                    break;
                }

                case ShelfHeuristic::FirstFit: {
                    // Try each shelf in order
                    for (auto& shelf : bin.shelves) {
                        auto placement = try_place_on_shelf(bin, shelf, item, rotated);
                        if (placement) {
                            shelf.used_width += rotated.width;
                            bin.placements.push_back(*placement);
                            bin.used_weight += item.weight;
                            return placement;
                        }
                    }
                    break;
                }

                case ShelfHeuristic::BestWidthFit: {
                    // Find shelf with least remaining width after placement
                    Shelf* best_shelf = nullptr;
                    double best_remaining = std::numeric_limits<double>::max();

                    for (auto& shelf : bin.shelves) {
                        if (rotated.height <= shelf.height &&
                            shelf.used_width + rotated.width <= bin.type.dimensions.width) {
                            double remaining =
                                bin.type.dimensions.width - shelf.used_width - rotated.width;
                            if (remaining < best_remaining) {
                                best_remaining = remaining;
                                best_shelf = &shelf;
                            }
                        }
                    }

                    if (best_shelf) {
                        auto placement = try_place_on_shelf(bin, *best_shelf, item, rotated);
                        if (placement) {
                            best_shelf->used_width += rotated.width;
                            bin.placements.push_back(*placement);
                            bin.used_weight += item.weight;
                            return placement;
                        }
                    }
                    break;
                }

                case ShelfHeuristic::BestHeightFit: {
                    // Find shelf with closest height match
                    Shelf* best_shelf = nullptr;
                    double best_diff = std::numeric_limits<double>::max();

                    for (auto& shelf : bin.shelves) {
                        if (rotated.height <= shelf.height &&
                            shelf.used_width + rotated.width <= bin.type.dimensions.width) {
                            double diff = shelf.height - rotated.height;
                            if (diff < best_diff) {
                                best_diff = diff;
                                best_shelf = &shelf;
                            }
                        }
                    }

                    if (best_shelf) {
                        auto placement = try_place_on_shelf(bin, *best_shelf, item, rotated);
                        if (placement) {
                            best_shelf->used_width += rotated.width;
                            bin.placements.push_back(*placement);
                            bin.used_weight += item.weight;
                            return placement;
                        }
                    }
                    break;
                }
            }

            // Try creating a new shelf
            auto placement = try_new_shelf(bin, item, rotated);
            if (placement) {
                bin.shelves.back().used_width = rotated.width;
                bin.placements.push_back(*placement);
                bin.used_weight += item.weight;
                return placement;
            }
        }
    }

    // Try adding a new bin
    if (config_.allow_multiple_bins) {
        add_bin();
        if (!bins_.empty()) {
            auto& new_bin = bins_.back();
            for (const auto& rotated : rotations) {
                auto placement = try_new_shelf(new_bin, item, rotated);
                if (placement) {
                    new_bin.shelves.back().used_width = rotated.width;
                    new_bin.placements.push_back(*placement);
                    new_bin.used_weight += item.weight;
                    return placement;
                }
            }
        }
    }

    unpacked_.push_back(item.id);
    return std::nullopt;
}

PackingResult ShelfSolver::finalize() {
    auto end_time = std::chrono::steady_clock::now();

    PackingResult result;
    result.algorithm = std::string(name());
    result.bins_used = static_cast<int>(bins_.size());
    result.unpacked_items = unpacked_;

    double total_item_volume = 0.0;
    double total_bin_volume = 0.0;

    for (const auto& bin : bins_) {
        for (const auto& p : bin.placements) {
            result.placements.push_back(p);
            total_item_volume += p.rotated_dimensions.volume();
        }
        if (!bin.placements.empty()) {
            total_bin_volume += bin.type.volume();
        }
    }

    // Don't count empty bins
    result.bins_used = 0;
    for (const auto& bin : bins_) {
        if (!bin.placements.empty()) {
            result.bins_used++;
        }
    }

    result.stats.total_item_volume = total_item_volume;
    result.stats.total_bin_volume = total_bin_volume;
    result.stats.utilization = total_bin_volume > 0 ? total_item_volume / total_bin_volume : 0.0;
    result.stats.execution_time =
        std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time_);

    return result;
}

PackingResult ShelfSolver::solve(std::span<const Item> items, const SolverConfig& config) {
    reset(config);

    for (const auto& item : items) {
        for (int i = 0; i < item.quantity; ++i) {
            Item single = item;
            single.quantity = 1;
            if (item.quantity > 1) {
                single.id = item.id + "_" + std::to_string(i + 1);
            }
            (void)pack_one(single);
        }
    }

    return finalize();
}

}  // namespace bp3d
