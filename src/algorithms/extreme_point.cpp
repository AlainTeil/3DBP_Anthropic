/**
 * @file extreme_point.cpp
 * @brief Extreme Point algorithm implementation
 */

#include "bp3d/algorithms/extreme_point.hpp"

#include "bp3d/constraints/collision.hpp"
#include "bp3d/rotation.hpp"

#include <algorithm>
#include <chrono>

namespace bp3d {

ExtremePointSolver::ExtremePointSolver(ExtremePointHeuristic heuristic) : heuristic_(heuristic) {}

void ExtremePointSolver::reset(const SolverConfig& config) {
    config_ = config;
    bins_.clear();
    unpacked_.clear();
    start_time_ = std::chrono::steady_clock::now();

    if (!config.bin_types.empty()) {
        add_bin();
    }
}

void ExtremePointSolver::add_bin() {
    if (config_.bin_types.empty())
        return;

    // Use first bin type by default (could be made configurable)
    const auto& bin_type = config_.bin_types.front();

    BinState bin;
    bin.type = bin_type;
    bin.index = static_cast<int>(bins_.size());
    bin.used_weight = 0.0;
    bin.extreme_points.push_back({0, 0, 0});  // Origin point

    bins_.push_back(std::move(bin));
}

std::optional<Placement> ExtremePointSolver::try_place_at(BinState& bin, const Item& item,
                                                          const Vector3& point) {
    const auto allowed = get_allowed_rotations(item.constraints);

    // Check weight
    if (bin.used_weight + item.weight > bin.type.max_weight) {
        return std::nullopt;
    }

    // Try each rotation
    for (std::size_t ri = 0; ri < kRotationCount; ++ri) {
        if (!allowed.test(ri))
            continue;

        Rotation rot = rotation_from_index(ri);
        Dimensions rotated = apply_rotation(item.dimensions, rot);

        // Check bounds
        if (point.x + rotated.width > bin.type.dimensions.width ||
            point.y + rotated.height > bin.type.dimensions.height ||
            point.z + rotated.depth > bin.type.dimensions.depth) {
            continue;
        }

        Placement proposed{item.id, bin.type.id, bin.index, point, rot, rotated};

        // Check collisions
        bool valid = true;
        for (const auto& existing : bin.placements) {
            if (placements_overlap(proposed, existing)) {
                valid = false;
                break;
            }
        }

        if (!valid)
            continue;

        // Check support (on floor or on another item)
        bool supported = (point.y < 1e-6);
        if (!supported) {
            for (const auto& existing : bin.placements) {
                double top = existing.position.y + existing.rotated_dimensions.height;
                if (std::abs(top - point.y) < 1e-6) {
                    // Check XZ overlap for support
                    if (existing.position.x < point.x + rotated.width &&
                        point.x < existing.position.x + existing.rotated_dimensions.width &&
                        existing.position.z < point.z + rotated.depth &&
                        point.z < existing.position.z + existing.rotated_dimensions.depth) {
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

    return std::nullopt;
}

void ExtremePointSolver::update_extreme_points(BinState& bin, const Placement& placement) {
    const auto& pos = placement.position;
    const auto& dims = placement.rotated_dimensions;

    // Generate new extreme points at corners of placed item
    std::vector<Vector3> new_points;

    // Point to the right (+X)
    new_points.push_back({pos.x + dims.width, pos.y, pos.z});

    // Point on top (+Y)
    new_points.push_back({pos.x, pos.y + dims.height, pos.z});

    // Point behind (+Z)
    new_points.push_back({pos.x, pos.y, pos.z + dims.depth});

    // Also add corner combinations
    new_points.push_back({pos.x + dims.width, pos.y, pos.z + dims.depth});
    new_points.push_back({pos.x + dims.width, pos.y + dims.height, pos.z});
    new_points.push_back({pos.x, pos.y + dims.height, pos.z + dims.depth});

    // Filter valid points (within bin, not inside existing items)
    for (const auto& pt : new_points) {
        if (pt.x < 0 || pt.y < 0 || pt.z < 0 || pt.x >= bin.type.dimensions.width ||
            pt.y >= bin.type.dimensions.height || pt.z >= bin.type.dimensions.depth) {
            continue;
        }

        // Check if point is inside any existing placement
        bool inside = false;
        for (const auto& p : bin.placements) {
            if (pt.x >= p.position.x && pt.x < p.position.x + p.rotated_dimensions.width &&
                pt.y >= p.position.y && pt.y < p.position.y + p.rotated_dimensions.height &&
                pt.z >= p.position.z && pt.z < p.position.z + p.rotated_dimensions.depth) {
                inside = true;
                break;
            }
        }

        if (!inside) {
            // Check if point already exists
            bool exists = false;
            for (const auto& ep : bin.extreme_points) {
                if (std::abs(ep.x - pt.x) < 1e-9 && std::abs(ep.y - pt.y) < 1e-9 &&
                    std::abs(ep.z - pt.z) < 1e-9) {
                    exists = true;
                    break;
                }
            }
            if (!exists) {
                bin.extreme_points.push_back(pt);
            }
        }
    }

    // Remove the point that was used
    auto it = std::remove_if(
        bin.extreme_points.begin(), bin.extreme_points.end(), [&pos](const Vector3& p) {
            return std::abs(p.x - pos.x) < 1e-9 && std::abs(p.y - pos.y) < 1e-9 &&
                   std::abs(p.z - pos.z) < 1e-9;
        });
    bin.extreme_points.erase(it, bin.extreme_points.end());
}

std::optional<Placement> ExtremePointSolver::pack_one(const Item& item) {
    // Try each bin
    for (auto& bin : bins_) {
        // Sort extreme points based on heuristic
        std::vector<Vector3> sorted_points = bin.extreme_points;

        switch (heuristic_) {
            case ExtremePointHeuristic::BottomLeft:
            case ExtremePointHeuristic::BottomLeftFill:
                std::sort(sorted_points.begin(), sorted_points.end(),
                          [](const Vector3& a, const Vector3& b) {
                              if (a.y != b.y)
                                  return a.y < b.y;
                              if (a.x != b.x)
                                  return a.x < b.x;
                              return a.z < b.z;
                          });
                break;

            case ExtremePointHeuristic::TouchingPerimeter:
            case ExtremePointHeuristic::MinWastedSpace:
                // For simplicity, use same ordering as BottomLeft
                std::sort(sorted_points.begin(), sorted_points.end(),
                          [](const Vector3& a, const Vector3& b) {
                              if (a.y != b.y)
                                  return a.y < b.y;
                              if (a.x != b.x)
                                  return a.x < b.x;
                              return a.z < b.z;
                          });
                break;
        }

        // Try each point
        for (const auto& point : sorted_points) {
            auto placement = try_place_at(bin, item, point);
            if (placement) {
                bin.placements.push_back(*placement);
                bin.used_weight += item.weight;
                update_extreme_points(bin, *placement);
                return placement;
            }
        }
    }

    // Try adding a new bin
    if (config_.allow_multiple_bins) {
        add_bin();
        if (!bins_.empty()) {
            auto& new_bin = bins_.back();
            if (!new_bin.extreme_points.empty()) {
                auto placement = try_place_at(new_bin, item, new_bin.extreme_points.front());
                if (placement) {
                    new_bin.placements.push_back(*placement);
                    new_bin.used_weight += item.weight;
                    update_extreme_points(new_bin, *placement);
                    return placement;
                }
            }
        }
    }

    unpacked_.push_back(item.id);
    return std::nullopt;
}

PackingResult ExtremePointSolver::finalize() {
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
        total_bin_volume += bin.type.volume();
    }

    result.stats.total_item_volume = total_item_volume;
    result.stats.total_bin_volume = total_bin_volume;
    result.stats.utilization = total_bin_volume > 0 ? total_item_volume / total_bin_volume : 0.0;
    result.stats.execution_time =
        std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time_);

    return result;
}

PackingResult ExtremePointSolver::solve(std::span<const Item> items, const SolverConfig& config) {
    reset(config);

    for (const auto& item : items) {
        // Handle quantity
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
