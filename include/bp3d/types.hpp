/**
 * @file types.hpp
 * @brief Core types for 3D bin packing: Dimensions, Vector3, Item, BinType
 */

#ifndef BP3D_TYPES_HPP
#define BP3D_TYPES_HPP

#include <bitset>
#include <cmath>
#include <limits>
#include <string>

namespace bp3d {

/**
 * @brief General geometric equality tolerance.
 *
 * Used when comparing coordinates/dimensions for exact coincidence
 * (e.g. deduplicating points, equality with rounding noise).
 */
inline constexpr double kEpsilon = 1e-9;

/**
 * @brief Surface-contact tolerance.
 *
 * Used for "touching" relationships such as support and stacking, where a
 * slightly looser tolerance than @ref kEpsilon is appropriate to absorb
 * accumulated floating-point error along stacked faces.
 */
inline constexpr double kContactTolerance = 1e-6;

/**
 * @brief 3D dimensions with named axes
 *
 * Convention: Y is the vertical axis (height), X is width, Z is depth.
 */
struct Dimensions {
    double width = 0.0;   ///< X-axis dimension
    double height = 0.0;  ///< Y-axis dimension (vertical)
    double depth = 0.0;   ///< Z-axis dimension

    /// Calculate volume
    [[nodiscard]] constexpr double volume() const noexcept { return width * height * depth; }

    /// Calculate surface area
    [[nodiscard]] constexpr double surface_area() const noexcept {
        return 2.0 * (width * height + height * depth + width * depth);
    }

    /// Check if all dimensions are positive
    [[nodiscard]] constexpr bool is_valid() const noexcept {
        return width > 0.0 && height > 0.0 && depth > 0.0;
    }

    /// Equality comparison with tolerance
    [[nodiscard]] bool equals(const Dimensions& other, double epsilon = kEpsilon) const noexcept {
        return std::abs(width - other.width) < epsilon &&
               std::abs(height - other.height) < epsilon && std::abs(depth - other.depth) < epsilon;
    }

    constexpr bool operator==(const Dimensions& other) const noexcept = default;
};

/**
 * @brief 3D position vector
 */
struct Vector3 {
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;

    constexpr Vector3 operator+(const Vector3& other) const noexcept {
        return {x + other.x, y + other.y, z + other.z};
    }

    constexpr Vector3 operator-(const Vector3& other) const noexcept {
        return {x - other.x, y - other.y, z - other.z};
    }

    constexpr bool operator==(const Vector3& other) const noexcept = default;

    /// Equality comparison with tolerance
    [[nodiscard]] bool equals(const Vector3& other, double epsilon = kEpsilon) const noexcept {
        return std::abs(x - other.x) < epsilon && std::abs(y - other.y) < epsilon &&
               std::abs(z - other.z) < epsilon;
    }
};

/**
 * @brief Constraints that can be applied to an item
 */
struct ItemConstraints {
    /**
     * @brief Allowed rotations as a bitset
     *
     * Bit positions correspond to Rotation enum values (0-5).
     * Default: all 6 rotations allowed.
     */
    std::bitset<6> allowed_rotations{0b111111};

    /// Whether other items can be stacked on top of this one
    bool stackable = true;

    /// Maximum weight that can be placed on top of this item
    double max_stack_weight = std::numeric_limits<double>::max();

    /// Whether the item must maintain its vertical orientation
    /// When true, only rotations that keep height as Y-axis are allowed
    bool this_side_up = false;

    bool operator==(const ItemConstraints& other) const noexcept = default;
};

/**
 * @brief An item (box) to be packed
 */
struct Item {
    std::string id;               ///< Unique identifier
    Dimensions dimensions;        ///< Width, height, depth
    double weight = 0.0;          ///< Weight of the item
    int quantity = 1;             ///< Number of copies of this item
    ItemConstraints constraints;  ///< Packing constraints

    /// Get the volume of this item
    [[nodiscard]] double volume() const noexcept { return dimensions.volume(); }

    /// Check if this item is valid
    [[nodiscard]] bool is_valid() const noexcept {
        return !id.empty() && dimensions.is_valid() && weight >= 0.0 && quantity > 0;
    }
};

/**
 * @brief A bin (container) type that items can be packed into
 */
struct BinType {
    std::string id;                                          ///< Unique identifier
    Dimensions dimensions;                                   ///< Inner dimensions
    double max_weight = std::numeric_limits<double>::max();  ///< Maximum total weight
    double cost = 1.0;  ///< Relative cost of using this bin. The Extreme Point
                        ///< solver opens the cheapest bin type that fits an item;
                        ///< other solvers select bin types in configuration order.

    /// Get the volume of this bin
    [[nodiscard]] double volume() const noexcept { return dimensions.volume(); }

    /// Check if this bin type is valid
    [[nodiscard]] bool is_valid() const noexcept {
        return !id.empty() && dimensions.is_valid() && max_weight > 0.0 && cost > 0.0;
    }
};

}  // namespace bp3d

#endif  // BP3D_TYPES_HPP
