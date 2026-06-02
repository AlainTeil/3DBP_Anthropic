/**
 * @file guillotine.cpp
 * @brief Guillotine algorithm implementation
 */

#include "bp3d/algorithms/guillotine.hpp"

#include "bp3d/config.hpp"
#include "bp3d/result.hpp"
#include "bp3d/rotation.hpp"
#include "bp3d/types.hpp"
#include "internal/placement_engine.hpp"
#include "internal/sorting.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cstddef>
#include <limits>
#include <optional>
#include <span>
#include <utility>
#include <vector>

namespace bp3d {

namespace {

/// Represents a 3D free rectangle for guillotine packing
struct FreeRect3D {
    Vector3 position;
    Dimensions size;
};

/// Split a free rectangle after placing an item.
///
/// The item is placed at the rectangle's origin corner, leaving free space
/// along each axis. The remaining volume is partitioned into up to three
/// disjoint, axis-aligned free rectangles whose union exactly covers the
/// original rectangle minus the item (a guillotine-style tiling).
///
/// The @p split_rule determines the order in which the axes are cut. The axis
/// cut first receives a free rectangle spanning the full cross-section of the
/// other two axes (the "big" slab); the second axis is constrained to the
/// item's footprint along the first axis; the third is constrained to the
/// item's footprint along both prior axes. Choosing which axis is cut first
/// yields genuinely different free-space shapes and therefore different packing
/// behaviour:
/// - ShorterLeftoverAxis: cut axes ordered by ascending leftover length.
/// - LongerLeftoverAxis: cut axes ordered by descending leftover length.
/// - MinimizeArea: cut the axis whose full-span slab has the smallest volume
///   first, keeping large contiguous free space for later (minimizes
///   fragmentation of big regions).
/// - MaximizeArea: cut the axis whose full-span slab has the largest volume
///   first.
void guillotine_split(std::vector<FreeRect3D>& free_rects, const FreeRect3D& rect,
                      const Vector3& item_pos, const Dimensions& item_size,
                      GuillotineSplitRule split_rule) {
    // Per-axis geometry: index 0 = X (width), 1 = Y (height), 2 = Z (depth).
    const std::array<double, 3> rect_pos{rect.position.x, rect.position.y, rect.position.z};
    const std::array<double, 3> rect_size{rect.size.width, rect.size.height, rect.size.depth};
    const std::array<double, 3> item_p{item_pos.x, item_pos.y, item_pos.z};
    const std::array<double, 3> item_s{item_size.width, item_size.height, item_size.depth};

    const std::array<double, 3> leftover{rect_size[0] - item_s[0], rect_size[1] - item_s[1],
                                         rect_size[2] - item_s[2]};

    // Volume of the full-cross-section slab if a given axis is cut first.
    const auto slab_volume = [&](int axis) {
        double v = leftover[static_cast<std::size_t>(axis)];
        for (int other = 0; other < 3; ++other) {
            if (other != axis) {
                v *= rect_size[static_cast<std::size_t>(other)];
            }
        }
        return v;
    };

    // Determine the axis cut order from the split rule.
    std::array<int, 3> order{0, 1, 2};
    switch (split_rule) {
        case GuillotineSplitRule::ShorterLeftoverAxis:
            std::sort(order.begin(), order.end(), [&](int a, int b) {
                return leftover[static_cast<std::size_t>(a)] <
                       leftover[static_cast<std::size_t>(b)];
            });
            break;
        case GuillotineSplitRule::LongerLeftoverAxis:
            std::sort(order.begin(), order.end(), [&](int a, int b) {
                return leftover[static_cast<std::size_t>(a)] >
                       leftover[static_cast<std::size_t>(b)];
            });
            break;
        case GuillotineSplitRule::MinimizeArea:
            std::sort(order.begin(), order.end(),
                      [&](int a, int b) { return slab_volume(a) < slab_volume(b); });
            break;
        case GuillotineSplitRule::MaximizeArea:
            std::sort(order.begin(), order.end(),
                      [&](int a, int b) { return slab_volume(a) > slab_volume(b); });
            break;
    }

    const int a = order[0];
    const int b = order[1];
    const int c = order[2];

    const auto au = static_cast<std::size_t>(a);
    const auto bu = static_cast<std::size_t>(b);
    const auto cu = static_cast<std::size_t>(c);

    // Build a free rectangle from per-axis position/size arrays.
    const auto make_rect = [](const std::array<double, 3>& pos, const std::array<double, 3>& size) {
        return FreeRect3D{{pos[0], pos[1], pos[2]}, {size[0], size[1], size[2]}};
    };

    // Big slab: beyond the item along axis a, spanning the full cross-section.
    if (leftover[au] > 0) {
        std::array<double, 3> pos = rect_pos;
        std::array<double, 3> size = rect_size;
        pos[au] = item_p[au] + item_s[au];
        size[au] = leftover[au];
        free_rects.push_back(make_rect(pos, size));
    }

    // Medium slab: beyond the item along axis b, within the item's extent on a,
    // spanning the full extent on c.
    if (leftover[bu] > 0) {
        std::array<double, 3> pos = rect_pos;
        std::array<double, 3> size = rect_size;
        pos[au] = rect_pos[au];
        size[au] = item_s[au];
        pos[bu] = item_p[bu] + item_s[bu];
        size[bu] = leftover[bu];
        free_rects.push_back(make_rect(pos, size));
    }

    // Small slab: beyond the item along axis c, within the item's extent on a and b.
    if (leftover[cu] > 0) {
        std::array<double, 3> pos = rect_pos;
        std::array<double, 3> size = rect_size;
        pos[au] = rect_pos[au];
        size[au] = item_s[au];
        pos[bu] = rect_pos[bu];
        size[bu] = item_s[bu];
        pos[cu] = item_p[cu] + item_s[cu];
        size[cu] = leftover[cu];
        free_rects.push_back(make_rect(pos, size));
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
        if (!allowed.test(ri)) {
            continue;
        }
        Rotation const rot = rotation_from_index(ri);
        Dimensions const rotated = apply_rotation(item.dimensions, rot);

        for (std::size_t i = 0; i < free_rects.size(); ++i) {
            const auto& rect = free_rects[i];

            if (rotated.width <= rect.size.width && rotated.height <= rect.size.height &&
                rotated.depth <= rect.size.depth) {
                double score = 0;

                switch (choice_rule) {
                    case GuillotineFreeRectChoice::BestShortSideFit: {
                        double const leftover_w = rect.size.width - rotated.width;
                        double const leftover_d = rect.size.depth - rotated.depth;
                        score = std::min(leftover_w, leftover_d);
                        break;
                    }
                    case GuillotineFreeRectChoice::BestLongSideFit: {
                        double const leftover_w = rect.size.width - rotated.width;
                        double const leftover_d = rect.size.depth - rotated.depth;
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

}  // namespace

GuillotineSolver::GuillotineSolver(GuillotineSplitRule split_rule,
                                   GuillotineFreeRectChoice choice_rule)
    : split_rule_(split_rule),
      choice_rule_(choice_rule) {}

PackingResult GuillotineSolver::solve(std::span<const Item> items, const SolverConfig& config) {
    const auto start_time = std::chrono::steady_clock::now();

    if (items.empty() || config.bin_types.empty()) {
        PackingResult empty;
        empty.algorithm = std::string(name());
        return empty;
    }

    auto expanded = internal::expand_quantities(items);
    internal::sort_items(expanded, internal::SortCriteria::VolumeDescending);

    // The shared engine owns bin bookkeeping, the item registry and the
    // constraint pipeline (collision, do-not-stack, weight limits) plus the
    // spatial index that narrows neighbour checks. Guillotine free-rects
    // guarantee non-overlap and provide their own support, so support is not
    // required. The per-bin free-rectangle partition is the only algorithm-
    // specific state and is kept in a vector parallel to engine.bins().
    internal::PlacementEngine engine(config, /*require_support=*/false);
    std::vector<std::vector<FreeRect3D>> free_rects;
    std::vector<std::string> unpacked;

    auto& bins = engine.bins();

    for (const auto& item : expanded) {
        // Honour the solving time budget: once exceeded, remaining items are
        // reported as unpacked rather than packed.
        if (internal::deadline_reached(start_time, config.timeout)) {
            unpacked.push_back(item.id);
            continue;
        }

        bool placed = false;

        // Try existing bins
        for (std::size_t bi = 0; bi < bins.size(); ++bi) {
            auto& bin = bins[bi];
            if (bin.used_weight + item.weight > bin.type.max_weight) {
                continue;
            }

            auto best = find_best_rect(free_rects[bi], item, choice_rule_);
            if (best) {
                auto [rect_idx, rotation] = *best;
                const FreeRect3D used_rect = free_rects[bi][rect_idx];
                const Vector3 item_position = used_rect.position;  // Copy before erase
                Dimensions const rotated = apply_rotation(item.dimensions, rotation);

                Placement const placement{item.id,       bin.type.id, bin.index,
                                          item_position, rotation,    rotated};

                if (!engine.is_valid(item, placement, bin)) {
                    continue;
                }

                // Remove used rect and add splits
                free_rects[bi].erase(free_rects[bi].begin() +
                                     static_cast<std::ptrdiff_t>(rect_idx));
                guillotine_split(free_rects[bi], used_rect, item_position, rotated, split_rule_);

                engine.commit(bin, item, placement);
                placed = true;
                break;
            }
        }

        // Try new bin. Always permit the first bin; additional bins require
        // allow_multiple_bins.
        if (!placed && (config.allow_multiple_bins || bins.empty())) {
            for (const auto& bin_type : config.bin_types) {
                if (item.weight > bin_type.max_weight) {
                    continue;
                }

                const int index = engine.open_bin(bin_type);
                const auto bi = static_cast<std::size_t>(index);
                free_rects.push_back({FreeRect3D{{0, 0, 0}, bin_type.dimensions}});
                auto& new_bin = bins[bi];

                auto best = find_best_rect(free_rects[bi], item, choice_rule_);
                if (best) {
                    auto [rect_idx, rotation] = *best;
                    const FreeRect3D used_rect = free_rects[bi][rect_idx];
                    const Vector3 item_position = used_rect.position;  // Copy before erase
                    Dimensions const rotated = apply_rotation(item.dimensions, rotation);

                    Placement const placement{item.id,       bin_type.id, new_bin.index,
                                              item_position, rotation,    rotated};

                    if (engine.is_valid(item, placement, new_bin)) {
                        free_rects[bi].erase(free_rects[bi].begin() +
                                             static_cast<std::ptrdiff_t>(rect_idx));
                        guillotine_split(free_rects[bi], used_rect, item_position, rotated,
                                         split_rule_);
                        engine.commit(new_bin, item, placement);
                        placed = true;
                        break;
                    }
                }

                // This bin type did not work out; drop the empty trial bin and
                // its parallel free-rect list before trying the next type.
                engine.discard_last_bin_if_empty();
                free_rects.pop_back();
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
