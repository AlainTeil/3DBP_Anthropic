/**
 * @file free_space.cpp
 * @brief Free space tracking implementations
 */

#include "free_space.hpp"

#include "bp3d/result.hpp"
#include "bp3d/types.hpp"
#include "geometry.hpp"

#include <algorithm>
#include <cstddef>
#include <cstdlib>
#include <limits>
#include <optional>
#include <span>
#include <utility>
#include <vector>

namespace bp3d::internal {

// ============================================================================
// MaximalSpaceManager
// ============================================================================

MaximalSpaceManager::MaximalSpaceManager(const Dimensions& bin_dims) : bin_dims_(bin_dims) {}

void MaximalSpaceManager::initialize() {
    spaces_.clear();
    spaces_.push_back(FreeSpace{{0.0, 0.0, 0.0}, bin_dims_});
}

std::optional<std::size_t> MaximalSpaceManager::find_best_fit(const Dimensions& dims) const {
    std::optional<std::size_t> best_index;
    double best_score = std::numeric_limits<double>::max();

    for (std::size_t i = 0; i < spaces_.size(); ++i) {
        if (spaces_[i].fits(dims)) {
            // Score by remaining volume after placement (lower is better)
            double const remaining = spaces_[i].volume() - dims.volume();
            if (remaining < best_score) {
                best_score = remaining;
                best_index = i;
            }
        }
    }

    return best_index;
}

void MaximalSpaceManager::place_item(const Vector3& position, const Dimensions& dims) {
    AABB const item_box = AABB::from_pos_dims(position, dims);
    std::vector<FreeSpace> new_spaces;

    for (const auto& space : spaces_) {
        AABB const space_box = space.to_aabb();

        if (!item_box.overlaps(space_box)) {
            // Space is not affected
            new_spaces.push_back(space);
        } else {
            // Split this space around the item
            split_space(space, position, dims);
            // The split pieces are added to spaces_ temporarily,
            // but we need a different approach

            // Generate up to 6 new spaces from splitting
            // Right side (positive X)
            if (position.x + dims.width < space.position.x + space.dimensions.width) {
                new_spaces.push_back(
                    FreeSpace{{position.x + dims.width, space.position.y, space.position.z},
                              {space.position.x + space.dimensions.width - position.x - dims.width,
                               space.dimensions.height, space.dimensions.depth}});
            }
            // Left side (negative X)
            if (position.x > space.position.x) {
                new_spaces.push_back(FreeSpace{space.position,
                                               {position.x - space.position.x,
                                                space.dimensions.height, space.dimensions.depth}});
            }
            // Top (positive Y)
            if (position.y + dims.height < space.position.y + space.dimensions.height) {
                new_spaces.push_back(FreeSpace{
                    {space.position.x, position.y + dims.height, space.position.z},
                    {space.dimensions.width,
                     space.position.y + space.dimensions.height - position.y - dims.height,
                     space.dimensions.depth}});
            }
            // Bottom (negative Y)
            if (position.y > space.position.y) {
                new_spaces.push_back(
                    FreeSpace{space.position,
                              {space.dimensions.width, position.y - space.position.y,
                               space.dimensions.depth}});
            }
            // Back (positive Z)
            if (position.z + dims.depth < space.position.z + space.dimensions.depth) {
                new_spaces.push_back(FreeSpace{
                    {space.position.x, space.position.y, position.z + dims.depth},
                    {space.dimensions.width, space.dimensions.height,
                     space.position.z + space.dimensions.depth - position.z - dims.depth}});
            }
            // Front (negative Z)
            if (position.z > space.position.z) {
                new_spaces.push_back(FreeSpace{space.position,
                                               {space.dimensions.width, space.dimensions.height,
                                                position.z - space.position.z}});
            }
        }
    }

    // Filter out invalid spaces (negative or zero dimensions)
    spaces_.clear();
    for (const auto& space : new_spaces) {
        if (space.dimensions.width > 0 && space.dimensions.height > 0 &&
            space.dimensions.depth > 0) {
            spaces_.push_back(space);
        }
    }

    remove_redundant_spaces();
}

void MaximalSpaceManager::split_space(const FreeSpace& /*space*/, const Vector3& /*pos*/,
                                      const Dimensions& /*dims*/) {
    // Implementation moved inline to place_item for clarity
}

void MaximalSpaceManager::remove_redundant_spaces() {
    // Remove spaces that are fully contained within other spaces
    std::vector<bool> to_remove(spaces_.size(), false);

    for (std::size_t i = 0; i < spaces_.size(); ++i) {
        if (to_remove[i]) {
            continue;
        }
        AABB const box_i = spaces_[i].to_aabb();

        for (std::size_t j = i + 1; j < spaces_.size(); ++j) {
            if (to_remove[j]) {
                continue;
            }
            AABB const box_j = spaces_[j].to_aabb();

            if (box_i.contains(box_j)) {
                to_remove[j] = true;
            } else if (box_j.contains(box_i)) {
                to_remove[i] = true;
                break;
            }
        }
    }

    std::vector<FreeSpace> filtered;
    for (std::size_t i = 0; i < spaces_.size(); ++i) {
        if (!to_remove[i]) {
            filtered.push_back(spaces_[i]);
        }
    }
    spaces_ = std::move(filtered);
}

double MaximalSpaceManager::free_volume() const {
    // Sum of all free space volumes (may have overlaps, so approximate)
    double total = 0.0;
    for (const auto& space : spaces_) {
        total += space.volume();
    }
    return total;
}

// ============================================================================
// ExtremePointManager
// ============================================================================

ExtremePointManager::ExtremePointManager(const Dimensions& bin_dims) : bin_dims_(bin_dims) {}

void ExtremePointManager::initialize() {
    points_.clear();
    // Start with the origin point
    points_.push_back(ExtremePoint{{0.0, 0.0, 0.0}, bin_dims_});
}

void ExtremePointManager::update(const Vector3& position, const Dimensions& dims) {
    // When an item is placed, create new extreme points at its corners
    // that are not inside other items

    // Point at back-top-right of placed item
    Vector3 const p1 = {position.x + dims.width, position.y, position.z};
    Vector3 const p2 = {position.x, position.y + dims.height, position.z};
    Vector3 const p3 = {position.x, position.y, position.z + dims.depth};

    // Calculate residual space for each new point
    auto add_point_if_valid = [this](const Vector3& p) {
        // Check if point is within bin bounds
        if (p.x >= 0 && p.x < bin_dims_.width && p.y >= 0 && p.y < bin_dims_.height && p.z >= 0 &&
            p.z < bin_dims_.depth) {
            // Calculate residual space (simplified: distance to bin walls)
            Dimensions const residual = {bin_dims_.width - p.x, bin_dims_.height - p.y,
                                         bin_dims_.depth - p.z};
            if (residual.width > 0 && residual.height > 0 && residual.depth > 0) {
                points_.push_back(ExtremePoint{p, residual});
            }
        }
    };

    add_point_if_valid(p1);
    add_point_if_valid(p2);
    add_point_if_valid(p3);

    // Remove the point that was used for placement
    auto it = std::remove_if(points_.begin(), points_.end(), [&position](const ExtremePoint& ep) {
        return ep.position.x == position.x && ep.position.y == position.y &&
               ep.position.z == position.z;
    });
    points_.erase(it, points_.end());
}

void ExtremePointManager::update_residual_space(ExtremePoint& point,
                                                std::span<const Placement> placements) const {
    // Start with maximum residual space (to bin walls)
    Dimensions residual = {bin_dims_.width - point.position.x, bin_dims_.height - point.position.y,
                           bin_dims_.depth - point.position.z};

    // Reduce residual space based on nearby items
    for (const auto& p : placements) {
        // Check if placement blocks in X direction
        if (p.position.y < point.position.y + residual.height &&
            p.position.y + p.rotated_dimensions.height > point.position.y &&
            p.position.z < point.position.z + residual.depth &&
            p.position.z + p.rotated_dimensions.depth > point.position.z) {
            if (p.position.x > point.position.x &&
                p.position.x < point.position.x + residual.width) {
                residual.width = p.position.x - point.position.x;
            }
        }

        // Check if placement blocks in Y direction
        if (p.position.x < point.position.x + residual.width &&
            p.position.x + p.rotated_dimensions.width > point.position.x &&
            p.position.z < point.position.z + residual.depth &&
            p.position.z + p.rotated_dimensions.depth > point.position.z) {
            if (p.position.y > point.position.y &&
                p.position.y < point.position.y + residual.height) {
                residual.height = p.position.y - point.position.y;
            }
        }

        // Check if placement blocks in Z direction
        if (p.position.x < point.position.x + residual.width &&
            p.position.x + p.rotated_dimensions.width > point.position.x &&
            p.position.y < point.position.y + residual.height &&
            p.position.y + p.rotated_dimensions.height > point.position.y) {
            if (p.position.z > point.position.z &&
                p.position.z < point.position.z + residual.depth) {
                residual.depth = p.position.z - point.position.z;
            }
        }
    }

    point.residual_space = residual;
}

// ============================================================================
// ShelfManager
// ============================================================================

ShelfManager::ShelfManager(const Dimensions& bin_dims) : bin_dims_(bin_dims) {}

std::optional<Vector3> ShelfManager::find_position(const Dimensions& dims) {
    // Try to fit on existing shelves
    for (auto& shelf : shelves_) {
        // Check if item fits on this shelf
        if (dims.height <= shelf.height) {
            // Try to place at current position on shelf
            if (shelf.used_width + dims.width <= bin_dims_.width && dims.depth <= bin_dims_.depth) {
                return Vector3{shelf.used_width, shelf.y_position, 0.0};
            }
        }
    }

    return std::nullopt;
}

bool ShelfManager::create_shelf(double height) {
    // Calculate current total height used
    double current_height = 0.0;
    for (const auto& shelf : shelves_) {
        current_height = std::max(current_height, shelf.y_position + shelf.height);
    }

    // Check if there's room for a new shelf
    if (current_height + height > bin_dims_.height) {
        return false;
    }

    shelves_.push_back(Shelf{current_height, height, 0.0, 0.0, {}});
    return true;
}

void ShelfManager::place_item(const Vector3& position, const Dimensions& dims,
                              const Placement& placement) {
    // Find the shelf this placement belongs to
    for (auto& shelf : shelves_) {
        if (std::abs(shelf.y_position - position.y) < kContactTolerance) {
            shelf.used_width = position.x + dims.width;
            shelf.used_depth = std::max(shelf.used_depth, dims.depth);
            shelf.items.push_back(placement);
            return;
        }
    }
}

}  // namespace bp3d::internal
