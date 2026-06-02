/**
 * @file extreme_point.cpp
 * @brief Extreme Point algorithm implementation
 */

#include "bp3d/algorithms/extreme_point.hpp"

#include "bp3d/config.hpp"
#include "bp3d/result.hpp"
#include "bp3d/rotation.hpp"
#include "bp3d/types.hpp"
#include "internal/placement_engine.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <memory>
#include <optional>
#include <span>
#include <string>
#include <vector>

namespace bp3d {

namespace {

/// 1-D overlap length of intervals [a0, a1] and [b0, b1] (0 if disjoint).
double overlap_1d(double a0, double a1, double b0, double b1) {
    return std::max(0.0, std::min(a1, b1) - std::max(a0, b0));
}

/// Area of the shared face between two axis-aligned boxes that touch on one
/// axis; 0 when the boxes do not share a face.
double face_contact(const Vector3& pa, const Dimensions& da, const Vector3& pb,
                    const Dimensions& db) {
    const double ax1 = pa.x + da.width;
    const double ay1 = pa.y + da.height;
    const double az1 = pa.z + da.depth;
    const double bx1 = pb.x + db.width;
    const double by1 = pb.y + db.height;
    const double bz1 = pb.z + db.depth;
    double area = 0.0;
    if (std::abs(ax1 - pb.x) < kContactTolerance || std::abs(bx1 - pa.x) < kContactTolerance) {
        area += overlap_1d(pa.y, ay1, pb.y, by1) * overlap_1d(pa.z, az1, pb.z, bz1);
    }
    if (std::abs(ay1 - pb.y) < kContactTolerance || std::abs(by1 - pa.y) < kContactTolerance) {
        area += overlap_1d(pa.x, ax1, pb.x, bx1) * overlap_1d(pa.z, az1, pb.z, bz1);
    }
    if (std::abs(az1 - pb.z) < kContactTolerance || std::abs(bz1 - pa.z) < kContactTolerance) {
        area += overlap_1d(pa.x, ax1, pb.x, bx1) * overlap_1d(pa.y, ay1, pb.y, by1);
    }
    return area;
}

}  // namespace

ExtremePointSolver::ExtremePointSolver(ExtremePointHeuristic heuristic) : heuristic_(heuristic) {}

ExtremePointSolver::~ExtremePointSolver() = default;

void ExtremePointSolver::reset(const SolverConfig& config) {
    config_ = config;
    // The shared engine owns bin bookkeeping, the item registry, the spatial
    // index and the constraint pipeline (collision, support, do-not-stack,
    // weight limits). Support is required for the extreme-point heuristic.
    engine_ = std::make_unique<internal::PlacementEngine>(config, /*require_support=*/true);
    extreme_points_.clear();
    unpacked_.clear();
    start_time_ = std::chrono::steady_clock::now();
    // Bins are opened lazily as items arrive so the bin type can be chosen to
    // fit (and minimise the cost of) the first item placed in it.
}

void ExtremePointSolver::add_bin(const Item* item) {
    if (config_.bin_types.empty()) {
        return;
    }

    const auto& bin_type = select_bin_type(item);
    engine_->open_bin(bin_type);
    // Keep the parallel extreme-point set in sync, seeded with the origin.
    extreme_points_.push_back({{0, 0, 0}});
}

const BinType& ExtremePointSolver::select_bin_type(const Item* item) const {
    const auto& types = config_.bin_types;
    if (item == nullptr) {
        return types.front();
    }

    // Pick the lowest-cost bin type that can accommodate the item in some
    // allowed rotation and within the weight limit.
    const auto allowed = get_allowed_rotations(item->constraints);
    const BinType* best = nullptr;
    for (const auto& bin_type : types) {
        if (item->weight > bin_type.max_weight) {
            continue;
        }
        bool fits = false;
        for (std::size_t ri = 0; ri < kRotationCount; ++ri) {
            if (!allowed.test(ri)) {
                continue;
            }
            const Dimensions rotated = apply_rotation(item->dimensions, rotation_from_index(ri));
            if (rotated.width <= bin_type.dimensions.width &&
                rotated.height <= bin_type.dimensions.height &&
                rotated.depth <= bin_type.dimensions.depth) {
                fits = true;
                break;
            }
        }
        if (fits && (best == nullptr || bin_type.cost < best->cost)) {
            best = &bin_type;
        }
    }

    // None clearly fit; fall back to the first type.
    return best != nullptr ? *best : types.front();
}

std::optional<Placement> ExtremePointSolver::select_best_placement(std::size_t bin_index,
                                                                   const Item& item) {
    auto& bin = engine_->bins()[bin_index];
    const auto& points = extreme_points_[bin_index];
    const auto allowed = get_allowed_rotations(item.constraints);

    std::optional<Placement> best;
    std::array<double, 3> best_key{};

    // Consider every extreme point and allowed rotation, keeping the candidate
    // with the smallest heuristic key. The constraint pipeline enforces bounds,
    // collisions, support, do-not-stack and weight limits.
    for (const auto& point : points) {
        for (std::size_t ri = 0; ri < kRotationCount; ++ri) {
            if (!allowed.test(ri)) {
                continue;
            }

            const Rotation rot = rotation_from_index(ri);
            const Dimensions rotated = apply_rotation(item.dimensions, rot);
            const Placement proposed{item.id, bin.type.id, bin.index, point, rot, rotated};

            if (!engine_->is_valid(item, proposed, bin)) {
                continue;
            }

            const std::array<double, 3> key = score_placement(bin_index, proposed);
            if (!best || key < best_key) {
                best = proposed;
                best_key = key;
            }
        }
    }

    return best;
}

std::array<double, 3> ExtremePointSolver::score_placement(std::size_t bin_index,
                                                          const Placement& placement) {
    const auto& bin = engine_->bins()[bin_index];
    const auto& pos = placement.position;
    const auto& d = placement.rotated_dimensions;

    switch (heuristic_) {
        case ExtremePointHeuristic::BottomLeft:
            // Lowest point wins: Y, then X, then Z.
            return {pos.y, pos.x, pos.z};

        case ExtremePointHeuristic::BottomLeftFill:
            // Favour low placements, then those nearest the origin corner so
            // gaps closest to the seed corner are filled first.
            return {pos.y, pos.x + pos.z, pos.x};

        case ExtremePointHeuristic::TouchingPerimeter: {
            // Maximise the contact area with bin walls and already-placed items.
            double contact = 0.0;
            if (pos.x <= kContactTolerance) {
                contact += d.height * d.depth;
            }
            if (pos.x + d.width >= bin.type.dimensions.width - kContactTolerance) {
                contact += d.height * d.depth;
            }
            if (pos.y <= kContactTolerance) {
                contact += d.width * d.depth;
            }
            if (pos.y + d.height >= bin.type.dimensions.height - kContactTolerance) {
                contact += d.width * d.depth;
            }
            if (pos.z <= kContactTolerance) {
                contact += d.width * d.height;
            }
            if (pos.z + d.depth >= bin.type.dimensions.depth - kContactTolerance) {
                contact += d.width * d.height;
            }
            for (const auto& q : bin.placements) {
                contact += face_contact(pos, d, q.position, q.rotated_dimensions);
            }
            // Negate so that larger contact sorts first; break ties bottom-left.
            return {-contact, pos.y, pos.x};
        }

        case ExtremePointHeuristic::MinWastedSpace: {
            // Keep the axis-aligned bounding box of all placed items as small as
            // possible, leaving larger contiguous free regions for later items.
            double max_x = pos.x + d.width;
            double max_y = pos.y + d.height;
            double max_z = pos.z + d.depth;
            for (const auto& q : bin.placements) {
                max_x = std::max(max_x, q.position.x + q.rotated_dimensions.width);
                max_y = std::max(max_y, q.position.y + q.rotated_dimensions.height);
                max_z = std::max(max_z, q.position.z + q.rotated_dimensions.depth);
            }
            return {max_x * max_y * max_z, pos.y, pos.x};
        }
    }

    return {pos.y, pos.x, pos.z};
}

void ExtremePointSolver::update_extreme_points(std::size_t bin_index, const Placement& placement) {
    const auto& bin = engine_->bins()[bin_index];
    auto& extreme_points = extreme_points_[bin_index];
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
            for (const auto& ep : extreme_points) {
                if (std::abs(ep.x - pt.x) < kEpsilon && std::abs(ep.y - pt.y) < kEpsilon &&
                    std::abs(ep.z - pt.z) < kEpsilon) {
                    exists = true;
                    break;
                }
            }
            if (!exists) {
                extreme_points.push_back(pt);
            }
        }
    }

    // Remove the point that was used
    auto it =
        std::remove_if(extreme_points.begin(), extreme_points.end(), [&pos](const Vector3& p) {
            return std::abs(p.x - pos.x) < kEpsilon && std::abs(p.y - pos.y) < kEpsilon &&
                   std::abs(p.z - pos.z) < kEpsilon;
        });
    extreme_points.erase(it, extreme_points.end());
}

std::optional<Placement> ExtremePointSolver::pack_one(const Item& item) {
    auto& bins = engine_->bins();

    // Try existing bins, choosing the placement that best fits the heuristic.
    for (std::size_t bi = 0; bi < bins.size(); ++bi) {
        if (auto placement = select_best_placement(bi, item)) {
            engine_->commit(bins[bi], item, *placement);
            update_extreme_points(bi, *placement);
            return placement;
        }
    }

    // Open a new bin when allowed (or for the very first bin even in
    // single-bin mode), choosing the cheapest bin type that fits the item.
    if (config_.allow_multiple_bins || bins.empty()) {
        add_bin(&item);
        if (!bins.empty()) {
            const std::size_t bi = bins.size() - 1;
            if (auto placement = select_best_placement(bi, item)) {
                engine_->commit(bins[bi], item, *placement);
                update_extreme_points(bi, *placement);
                return placement;
            }
        }
    }

    unpacked_.push_back(item.id);
    return std::nullopt;
}

PackingResult ExtremePointSolver::finalize() {
    return internal::make_result(std::string(name()), engine_->bins(), unpacked_, start_time_);
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
