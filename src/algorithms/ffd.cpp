/**
 * @file ffd.cpp
 * @brief First Fit Decreasing algorithm implementation
 */

#include "bp3d/algorithms/ffd.hpp"

#include "bp3d/config.hpp"
#include "bp3d/result.hpp"
#include "bp3d/types.hpp"
#include "internal/placement_engine.hpp"
#include "internal/sorting.hpp"

#include <chrono>
#include <cstddef>
#include <span>
#include <string>
#include <utility>
#include <vector>

namespace bp3d {

namespace {

/// Bottom-left-back ordering: prefer low y, then low z, then low x.
bool bottom_left_back(const Vector3& a, const Vector3& b) {
    if (a.y != b.y) {
        return a.y < b.y;
    }
    if (a.z != b.z) {
        return a.z < b.z;
    }
    return a.x < b.x;
}

}  // namespace

PackingResult FirstFitDecreasing::solve(std::span<const Item> items, const SolverConfig& config) {
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

        bool placed = false;
        for (auto& bin : engine.bins()) {
            if (auto placement = engine.find_first_fit(item, bin, bottom_left_back)) {
                engine.commit(bin, item, *placement);
                placed = true;
                break;
            }
        }

        // Otherwise open a new bin (smallest bin type that fits, in input order).
        // Always permit the first bin; additional bins require allow_multiple_bins.
        if (!placed && (config.allow_multiple_bins || engine.bins().empty())) {
            for (const auto& bin_type : config.bin_types) {
                const int idx = engine.open_bin(bin_type);
                auto& bin = engine.bins()[static_cast<std::size_t>(idx)];
                if (auto placement = engine.find_first_fit(item, bin, bottom_left_back)) {
                    engine.commit(bin, item, *placement);
                    placed = true;
                    break;
                }
                engine.discard_last_bin_if_empty();
            }
        }

        if (!placed) {
            unpacked.push_back(item.id);
        }
    }

    return internal::make_result(std::string(name()), engine.bins(), std::move(unpacked),
                                 start_time);
}

}  // namespace bp3d
