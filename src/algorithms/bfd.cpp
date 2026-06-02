/**
 * @file bfd.cpp
 * @brief Best Fit Decreasing algorithm implementation
 */

#include "bp3d/algorithms/bfd.hpp"

#include "bp3d/config.hpp"
#include "bp3d/result.hpp"
#include "bp3d/types.hpp"
#include "internal/placement_engine.hpp"
#include "internal/sorting.hpp"

#include <algorithm>
#include <chrono>
#include <cstddef>
#include <limits>
#include <optional>
#include <span>
#include <string>
#include <utility>
#include <vector>

namespace bp3d {

namespace {

/// Score a candidate placement: prefer lower positions, then less wasted space.
double placement_score(const Placement& p) {
    return p.position.y * 1000.0 + p.position.x + p.position.z * 0.1;
}

}  // namespace

PackingResult BestFitDecreasing::solve(std::span<const Item> items, const SolverConfig& config) {
    const auto start_time = std::chrono::steady_clock::now();

    if (items.empty() || config.bin_types.empty()) {
        PackingResult empty;
        empty.algorithm = std::string(name());
        return empty;
    }

    // Expand quantities and sort by volume descending.
    auto expanded = internal::expand_quantities(items);
    internal::sort_items(expanded, internal::SortCriteria::VolumeDescending);

    internal::PlacementEngine engine(config, /*require_support=*/true);
    std::vector<std::string> unpacked;

    for (const auto& item : expanded) {
        // Honour the solving time budget: once exceeded, remaining items are
        // reported as unpacked rather than packed.
        if (internal::deadline_reached(start_time, config.timeout)) {
            unpacked.push_back(item.id);
            continue;
        }

        // Find the best existing bin (least remaining volume after placement).
        std::optional<Placement> best_placement;
        double best_remaining = std::numeric_limits<double>::max();
        std::size_t best_bin_index = 0;

        auto& bins = engine.bins();
        for (std::size_t i = 0; i < bins.size(); ++i) {
            if (auto placement = engine.find_best_in_bin(item, bins[i], placement_score)) {
                const double remaining_after =
                    bins[i].type.volume() - bins[i].used_volume - item.volume();
                if (remaining_after < best_remaining) {
                    best_remaining = remaining_after;
                    best_placement = placement;
                    best_bin_index = i;
                }
            }
        }

        if (best_placement) {
            engine.commit(bins[best_bin_index], item, *best_placement);
            continue;
        }

        if (!config.allow_multiple_bins && !engine.bins().empty()) {
            unpacked.push_back(item.id);
            continue;
        }

        // Open a new bin: smallest bin type (by volume) that fits. Always permit
        // the first bin; additional bins require allow_multiple_bins.
        auto sorted_types = config.bin_types;
        std::sort(sorted_types.begin(), sorted_types.end(),
                  [](const BinType& a, const BinType& b) { return a.volume() < b.volume(); });

        bool placed = false;
        for (const auto& bin_type : sorted_types) {
            const int idx = engine.open_bin(bin_type);
            auto& bin = engine.bins()[static_cast<std::size_t>(idx)];
            if (auto placement = engine.find_best_in_bin(item, bin, placement_score)) {
                engine.commit(bin, item, *placement);
                placed = true;
                break;
            }
            engine.discard_last_bin_if_empty();
        }

        if (!placed) {
            unpacked.push_back(item.id);
        }
    }

    return internal::make_result(std::string(name()), engine.bins(), std::move(unpacked),
                                 start_time);
}

}  // namespace bp3d
