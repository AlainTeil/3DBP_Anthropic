/**
 * @file shelf.cpp
 * @brief Shelf algorithm implementation
 */

#include "bp3d/algorithms/shelf.hpp"

#include "bp3d/config.hpp"
#include "bp3d/result.hpp"
#include "bp3d/rotation.hpp"
#include "bp3d/types.hpp"
#include "internal/placement_engine.hpp"

#include <algorithm>
#include <chrono>
#include <cstddef>
#include <limits>
#include <memory>
#include <optional>
#include <span>
#include <string>
#include <vector>

namespace bp3d {

ShelfSolver::ShelfSolver(ShelfHeuristic heuristic) : heuristic_(heuristic) {}

ShelfSolver::~ShelfSolver() = default;

void ShelfSolver::reset(const SolverConfig& config) {
    config_ = config;
    // The shared engine owns bin bookkeeping, the item registry, the spatial
    // index and the constraint pipeline (collision, do-not-stack, weight
    // limits). Shelves provide their own support, so support is disabled.
    engine_ = std::make_unique<internal::PlacementEngine>(config, /*require_support=*/false);
    shelves_.clear();
    unpacked_.clear();
    start_time_ = std::chrono::steady_clock::now();

    if (!config.bin_types.empty()) {
        add_bin();
    }
}

bool ShelfSolver::accept(std::size_t bin_index, const Item& item, const Placement& placement) {
    auto& bin = engine_->bins()[bin_index];
    if (!engine_->is_valid(item, placement, bin)) {
        return false;
    }
    engine_->commit(bin, item, placement);
    return true;
}

void ShelfSolver::add_bin() {
    if (config_.bin_types.empty()) {
        return;
    }

    engine_->open_bin(config_.bin_types.front());
    // Keep the parallel shelf list in sync with the engine's bins.
    shelves_.emplace_back();
}

std::optional<Placement> ShelfSolver::try_place_on_shelf(std::size_t bin_index, Shelf& shelf,
                                                         const Item& item,
                                                         const Dimensions& rotated) {
    const auto& bin = engine_->bins()[bin_index];

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
        if (!allowed.test(ri)) {
            continue;
        }
        Rotation const rot = rotation_from_index(ri);
        Dimensions const r = apply_rotation(item.dimensions, rot);
        if (r.width == rotated.width && r.height == rotated.height && r.depth == rotated.depth) {
            used_rotation = rot;
            break;
        }
    }

    Vector3 const pos{shelf.used_width, shelf.y_position, 0.0};

    return Placement{item.id, bin.type.id, bin.index, pos, used_rotation, rotated};
}

std::optional<Placement> ShelfSolver::try_new_shelf(std::size_t bin_index, const Item& item,
                                                    const Dimensions& rotated) {
    const auto& bin = engine_->bins()[bin_index];
    auto& shelves = shelves_[bin_index];

    // Calculate current shelf height
    double current_height = 0.0;
    for (const auto& shelf : shelves) {
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
    Shelf const new_shelf{current_height, rotated.height, 0.0};

    // Find the rotation
    const auto allowed = get_allowed_rotations(item.constraints);
    Rotation used_rotation = Rotation::WHD;

    for (std::size_t ri = 0; ri < kRotationCount; ++ri) {
        if (!allowed.test(ri)) {
            continue;
        }
        Rotation const rot = rotation_from_index(ri);
        Dimensions const r = apply_rotation(item.dimensions, rot);
        if (r.width == rotated.width && r.height == rotated.height && r.depth == rotated.depth) {
            used_rotation = rot;
            break;
        }
    }

    Vector3 const pos{0.0, current_height, 0.0};

    shelves.push_back(new_shelf);

    return Placement{item.id, bin.type.id, bin.index, pos, used_rotation, rotated};
}

std::optional<Placement> ShelfSolver::pack_one(const Item& item) {
    const auto allowed = get_allowed_rotations(item.constraints);

    // Collect all valid rotations
    std::vector<Dimensions> rotations;
    for (std::size_t ri = 0; ri < kRotationCount; ++ri) {
        if (!allowed.test(ri)) {
            continue;
        }
        Dimensions const r = apply_rotation(item.dimensions, rotation_from_index(ri));
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
    auto& bins = engine_->bins();
    for (std::size_t bi = 0; bi < bins.size(); ++bi) {
        auto& shelves = shelves_[bi];
        const auto& bin_type = bins[bi].type;
        for (const auto& rotated : rotations) {
            // Based on heuristic, try different shelf selection strategies
            switch (heuristic_) {
                case ShelfHeuristic::NextFit: {
                    // Only try the last shelf
                    if (!shelves.empty()) {
                        auto placement = try_place_on_shelf(bi, shelves.back(), item, rotated);
                        if (placement && accept(bi, item, *placement)) {
                            shelves.back().used_width += rotated.width;
                            return placement;
                        }
                    }
                    break;
                }

                case ShelfHeuristic::FirstFit: {
                    // Try each shelf in order
                    for (auto& shelf : shelves) {
                        auto placement = try_place_on_shelf(bi, shelf, item, rotated);
                        if (placement && accept(bi, item, *placement)) {
                            shelf.used_width += rotated.width;
                            return placement;
                        }
                    }
                    break;
                }

                case ShelfHeuristic::BestWidthFit: {
                    // Find shelf with least remaining width after placement
                    Shelf* best_shelf = nullptr;
                    double best_remaining = std::numeric_limits<double>::max();

                    for (auto& shelf : shelves) {
                        if (rotated.height <= shelf.height &&
                            shelf.used_width + rotated.width <= bin_type.dimensions.width) {
                            double const remaining =
                                bin_type.dimensions.width - shelf.used_width - rotated.width;
                            if (remaining < best_remaining) {
                                best_remaining = remaining;
                                best_shelf = &shelf;
                            }
                        }
                    }

                    if (best_shelf != nullptr) {
                        auto placement = try_place_on_shelf(bi, *best_shelf, item, rotated);
                        if (placement && accept(bi, item, *placement)) {
                            best_shelf->used_width += rotated.width;
                            return placement;
                        }
                    }
                    break;
                }

                case ShelfHeuristic::BestHeightFit: {
                    // Find shelf with closest height match
                    Shelf* best_shelf = nullptr;
                    double best_diff = std::numeric_limits<double>::max();

                    for (auto& shelf : shelves) {
                        if (rotated.height <= shelf.height &&
                            shelf.used_width + rotated.width <= bin_type.dimensions.width) {
                            double const diff = shelf.height - rotated.height;
                            if (diff < best_diff) {
                                best_diff = diff;
                                best_shelf = &shelf;
                            }
                        }
                    }

                    if (best_shelf != nullptr) {
                        auto placement = try_place_on_shelf(bi, *best_shelf, item, rotated);
                        if (placement && accept(bi, item, *placement)) {
                            best_shelf->used_width += rotated.width;
                            return placement;
                        }
                    }
                    break;
                }
            }

            // Try creating a new shelf
            auto placement = try_new_shelf(bi, item, rotated);
            if (placement) {
                if (accept(bi, item, *placement)) {
                    shelves.back().used_width = rotated.width;
                    return placement;
                }
                shelves.pop_back();
            }
        }
    }

    // Try adding a new bin
    if (config_.allow_multiple_bins) {
        add_bin();
        if (!bins.empty()) {
            const std::size_t bi = bins.size() - 1;
            for (const auto& rotated : rotations) {
                auto placement = try_new_shelf(bi, item, rotated);
                if (placement) {
                    if (accept(bi, item, *placement)) {
                        shelves_[bi].back().used_width = rotated.width;
                        return placement;
                    }
                    shelves_[bi].pop_back();
                }
            }
        }
    }

    unpacked_.push_back(item.id);
    return std::nullopt;
}

PackingResult ShelfSolver::finalize() {
    return internal::make_result(std::string(name()), engine_->bins(), unpacked_, start_time_);
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
            // Honour the solving time budget: once exceeded, remaining items
            // are reported as unpacked rather than packed.
            if (internal::deadline_reached(start_time_, config.timeout)) {
                unpacked_.push_back(single.id);
                continue;
            }
            (void)pack_one(single);
        }
    }

    return finalize();
}

}  // namespace bp3d
