/**
 * @file sorting.hpp
 * @brief Item sorting utilities for offline algorithms
 */

#ifndef BP3D_INTERNAL_SORTING_HPP
#define BP3D_INTERNAL_SORTING_HPP

#include "bp3d/types.hpp"

#include <algorithm>
#include <span>
#include <vector>

namespace bp3d::internal {

/**
 * @brief Sorting criteria for items
 */
enum class SortCriteria {
    VolumeDescending,       ///< Largest volume first
    VolumeAscending,        ///< Smallest volume first
    HeightDescending,       ///< Tallest items first
    SurfaceAreaDescending,  ///< Largest surface area first
    WeightDescending,       ///< Heaviest items first
    MaxDimensionDescending  ///< Items with largest dimension first
};

/**
 * @brief Get the maximum dimension of an item
 */
[[nodiscard]] inline double max_dimension(const Item& item) {
    return std::max({item.dimensions.width, item.dimensions.height, item.dimensions.depth});
}

/**
 * @brief Compare items by volume (descending)
 */
struct VolumeDescendingCompare {
    bool operator()(const Item& a, const Item& b) const { return a.volume() > b.volume(); }
};

/**
 * @brief Compare items by volume (ascending)
 */
struct VolumeAscendingCompare {
    bool operator()(const Item& a, const Item& b) const { return a.volume() < b.volume(); }
};

/**
 * @brief Compare items by height (descending)
 */
struct HeightDescendingCompare {
    bool operator()(const Item& a, const Item& b) const {
        return a.dimensions.height > b.dimensions.height;
    }
};

/**
 * @brief Compare items by surface area (descending)
 */
struct SurfaceAreaDescendingCompare {
    bool operator()(const Item& a, const Item& b) const {
        return a.dimensions.surface_area() > b.dimensions.surface_area();
    }
};

/**
 * @brief Compare items by weight (descending)
 */
struct WeightDescendingCompare {
    bool operator()(const Item& a, const Item& b) const { return a.weight > b.weight; }
};

/**
 * @brief Compare items by max dimension (descending)
 */
struct MaxDimensionDescendingCompare {
    bool operator()(const Item& a, const Item& b) const {
        return max_dimension(a) > max_dimension(b);
    }
};

/**
 * @brief Sort items by the specified criteria
 *
 * @param items Items to sort (will be modified in place)
 * @param criteria Sorting criteria
 */
inline void sort_items(std::vector<Item>& items, SortCriteria criteria) {
    switch (criteria) {
        case SortCriteria::VolumeDescending:
            std::sort(items.begin(), items.end(), VolumeDescendingCompare{});
            break;
        case SortCriteria::VolumeAscending:
            std::sort(items.begin(), items.end(), VolumeAscendingCompare{});
            break;
        case SortCriteria::HeightDescending:
            std::sort(items.begin(), items.end(), HeightDescendingCompare{});
            break;
        case SortCriteria::SurfaceAreaDescending:
            std::sort(items.begin(), items.end(), SurfaceAreaDescendingCompare{});
            break;
        case SortCriteria::WeightDescending:
            std::sort(items.begin(), items.end(), WeightDescendingCompare{});
            break;
        case SortCriteria::MaxDimensionDescending:
            std::sort(items.begin(), items.end(), MaxDimensionDescendingCompare{});
            break;
    }
}

/**
 * @brief Expand items with quantity > 1 into separate items
 *
 * @param items Original items (may have quantity > 1)
 * @return Expanded list with one entry per item instance
 */
[[nodiscard]] inline std::vector<Item> expand_quantities(std::span<const Item> items) {
    std::vector<Item> expanded;

    for (const auto& item : items) {
        for (int i = 0; i < item.quantity; ++i) {
            Item copy = item;
            copy.quantity = 1;
            if (item.quantity > 1) {
                copy.id = item.id + "_" + std::to_string(i + 1);
            }
            expanded.push_back(std::move(copy));
        }
    }

    return expanded;
}

}  // namespace bp3d::internal

#endif  // BP3D_INTERNAL_SORTING_HPP
