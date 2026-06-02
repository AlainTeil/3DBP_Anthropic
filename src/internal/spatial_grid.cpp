/**
 * @file spatial_grid.cpp
 * @brief Implementation of the uniform spatial-hash grid.
 */

#include "internal/spatial_grid.hpp"

#include "geometry.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <vector>

namespace bp3d::internal {

namespace {

/// Fallback cell edge length used when callers request a non-positive size.
///
/// Must stay a sane, coarse value: an extremely small cell would make a single
/// box span an astronomical number of cells in @ref SpatialGrid::insert. The
/// engine always supplies a positive, dimension-derived size, so this only
/// guards the defensive misuse path.
constexpr double kFallbackCellSize = 1.0;

}  // namespace

SpatialGrid::SpatialGrid(double cell_size)
    : cell_size_(cell_size > 0.0 ? cell_size : kFallbackCellSize) {}

std::size_t SpatialGrid::CellKeyHash::operator()(const CellKey& key) const noexcept {
    // Mix the three cell coordinates with a 64-bit golden-ratio constant.
    constexpr std::size_t kGolden = 0x9e3779b97f4a7c15ULL;
    auto mix = [](std::size_t seed, int value) noexcept {
        return seed ^ (static_cast<std::size_t>(static_cast<std::uint32_t>(value)) + kGolden +
                       (seed << 6) + (seed >> 2));
    };
    auto h = static_cast<std::size_t>(static_cast<std::uint32_t>(key.x));
    h = mix(h, key.y);
    h = mix(h, key.z);
    return h;
}

int SpatialGrid::cell_index(double coord) const noexcept {
    return static_cast<int>(std::floor(coord / cell_size_));
}

void SpatialGrid::insert(std::size_t index, const AABB& box) {
    if (index >= seen_.size()) {
        seen_.resize(index + 1, 0);
    }

    const int x0 = cell_index(box.min.x);
    const int x1 = cell_index(box.max.x);
    const int y0 = cell_index(box.min.y);
    const int y1 = cell_index(box.max.y);
    const int z0 = cell_index(box.min.z);
    const int z1 = cell_index(box.max.z);

    if (has_cells_) {
        min_cell_x_ = std::min(min_cell_x_, x0);
        min_cell_y_ = std::min(min_cell_y_, y0);
        min_cell_z_ = std::min(min_cell_z_, z0);
        max_cell_x_ = std::max(max_cell_x_, x1);
        max_cell_y_ = std::max(max_cell_y_, y1);
        max_cell_z_ = std::max(max_cell_z_, z1);
    } else {
        min_cell_x_ = x0;
        min_cell_y_ = y0;
        min_cell_z_ = z0;
        max_cell_x_ = x1;
        max_cell_y_ = y1;
        max_cell_z_ = z1;
        has_cells_ = true;
    }

    for (int x = x0; x <= x1; ++x) {
        for (int y = y0; y <= y1; ++y) {
            for (int z = z0; z <= z1; ++z) {
                cells_[CellKey{x, y, z}].push_back(index);
            }
        }
    }
    ++count_;
}

void SpatialGrid::query(const AABB& box, std::vector<std::size_t>& out) const {
    if (!has_cells_) {
        return;
    }

    // Advance the epoch; on wrap-around reset the stamps so stale values from a
    // previous cycle cannot be mistaken for the current epoch.
    if (++epoch_ == 0) {
        std::fill(seen_.begin(), seen_.end(), 0);
        epoch_ = 1;
    }

    // Clamp the query range to the populated cell bounds. Without this a box far
    // larger than the occupied region (e.g. an oversized candidate item) would
    // make the loops iterate an astronomical number of empty cells.
    const int x0 = std::max(cell_index(box.min.x), min_cell_x_);
    const int x1 = std::min(cell_index(box.max.x), max_cell_x_);
    const int y0 = std::max(cell_index(box.min.y), min_cell_y_);
    const int y1 = std::min(cell_index(box.max.y), max_cell_y_);
    const int z0 = std::max(cell_index(box.min.z), min_cell_z_);
    const int z1 = std::min(cell_index(box.max.z), max_cell_z_);

    for (int x = x0; x <= x1; ++x) {
        for (int y = y0; y <= y1; ++y) {
            for (int z = z0; z <= z1; ++z) {
                const auto it = cells_.find(CellKey{x, y, z});
                if (it == cells_.end()) {
                    continue;
                }
                for (const std::size_t index : it->second) {
                    if (seen_[index] != epoch_) {
                        seen_[index] = epoch_;
                        out.push_back(index);
                    }
                }
            }
        }
    }
}

void SpatialGrid::clear() noexcept {
    cells_.clear();
    count_ = 0;
    has_cells_ = false;
    // seen_/epoch_ remain valid; stamps are naturally invalidated by the epoch.
}

}  // namespace bp3d::internal
