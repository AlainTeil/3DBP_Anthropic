/**
 * @file placement_engine.cpp
 * @brief Implementation of the shared PlacementEngine.
 */

#include "internal/placement_engine.hpp"

#include "bp3d/config.hpp"
#include "bp3d/constraints/validator.hpp"
#include "bp3d/types.hpp"
#include "internal/geometry.hpp"
#include "spatial_grid.hpp"

#include <algorithm>
#include <cstddef>
#include <utility>
#include <vector>

namespace bp3d::internal {

namespace {

/// Choose a cell edge length for a bin's spatial grid.
///
/// Sized to a fraction of the bin's largest dimension so a typical item spans
/// only a couple of cells: small enough to prune effectively, large enough to
/// keep the cells-per-box count low.
[[nodiscard]] double grid_cell_size(const BinType& type) {
    const double largest =
        std::max({type.dimensions.width, type.dimensions.height, type.dimensions.depth});
    constexpr double kDivisions = 16.0;
    const double cell = largest / kDivisions;
    return cell > 0.0 ? cell : 1.0;
}

}  // namespace

PlacementEngine::PlacementEngine(SolverConfig config, bool require_support)
    : config_(std::move(config)),
      validator_(create_default_validator(require_support)) {}

int PlacementEngine::open_bin(const BinType& type) {
    EngineBin bin;
    bin.type = type;
    bin.index = static_cast<int>(bins_.size());
    bins_.push_back(std::move(bin));
    grids_.emplace_back(grid_cell_size(type));
    return bins_.back().index;
}

void PlacementEngine::discard_last_bin_if_empty() {
    if (!bins_.empty() && bins_.back().placements.empty()) {
        bins_.pop_back();
        grids_.pop_back();
    }
}

std::vector<Vector3> PlacementEngine::candidates(const EngineBin& bin) const {
    std::vector<Vector3> result;
    result.reserve(bin.placements.size() * 6 + 1);
    result.push_back({0.0, 0.0, 0.0});

    for (const auto& p : bin.placements) {
        const auto max = p.max_corner();
        // Right of the item.
        result.push_back({max.x, p.position.y, p.position.z});
        result.push_back({max.x, 0.0, 0.0});
        // On top of the item.
        result.push_back({p.position.x, max.y, p.position.z});
        result.push_back({0.0, max.y, 0.0});
        // Behind the item.
        result.push_back({p.position.x, p.position.y, max.z});
        result.push_back({0.0, 0.0, max.z});
    }

    std::sort(result.begin(), result.end(), [](const Vector3& a, const Vector3& b) {
        if (a.x != b.x) {
            return a.x < b.x;
        }
        if (a.y != b.y) {
            return a.y < b.y;
        }
        return a.z < b.z;
    });
    result.erase(std::unique(result.begin(), result.end()), result.end());
    return result;
}

bool PlacementEngine::is_valid(const Item& item, const Placement& placement,
                               const EngineBin& bin) const {
    // Enforce the bin-wide weight limit from the running accumulator. The
    // narrowed neighbour set below only covers items near the candidate, which
    // is insufficient for this global sum, so the engine owns this check
    // (mirroring WeightValidator's bin-total semantics exactly).
    if (bin.used_weight + item.weight > bin.type.max_weight) {
        return false;
    }

    // Gather only the placements spatially near the candidate. Collision needs
    // overlapping boxes; support / do-not-stack / stack-weight need boxes
    // directly beneath. Both lie within the candidate's AABB, expanded by the
    // contact tolerance so boxes merely *touching* it are also captured.
    const SpatialGrid& grid = grids_[static_cast<std::size_t>(bin.index)];
    AABB query = AABB::from_placement(placement);
    query.min.x -= kContactTolerance;
    query.min.y -= kContactTolerance;
    query.min.z -= kContactTolerance;
    query.max.x += kContactTolerance;
    query.max.y += kContactTolerance;
    query.max.z += kContactTolerance;

    neighbor_indices_.clear();
    grid.query(query, neighbor_indices_);

    neighbor_scratch_.clear();
    neighbor_scratch_.reserve(neighbor_indices_.size());
    for (const std::size_t idx : neighbor_indices_) {
        neighbor_scratch_.push_back(bin.placements[idx]);
    }

    return validator_->can_place(item, placement, neighbor_scratch_, bin.type, registry_);
}

void PlacementEngine::commit(EngineBin& bin, const Item& item, const Placement& placement) {
    const std::size_t new_index = bin.placements.size();
    bin.placements.push_back(placement);
    bin.used_weight += item.weight;
    bin.used_volume += item.volume();
    registry_[item.id] = item;
    grids_[static_cast<std::size_t>(bin.index)].insert(new_index, AABB::from_placement(placement));
}

}  // namespace bp3d::internal
