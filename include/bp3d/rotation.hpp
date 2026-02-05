/**
 * @file rotation.hpp
 * @brief Rotation enumeration and utilities for 3D box orientations
 */

#ifndef BP3D_ROTATION_HPP
#define BP3D_ROTATION_HPP

#include "bp3d/types.hpp"

#include <array>
#include <bitset>
#include <cstdint>
#include <string_view>

namespace bp3d {

/**
 * @brief 6 possible orthogonal rotations for a 3D box
 *
 * Named by which original dimension maps to each axis (X, Y, Z).
 * For example, WHD means: Width->X, Height->Y, Depth->Z (original orientation).
 */
enum class Rotation : std::uint8_t {
    WHD = 0,  ///< Original: width-X, height-Y, depth-Z
    WDH = 1,  ///< width-X, depth-Y, height-Z
    HWD = 2,  ///< height-X, width-Y, depth-Z
    HDW = 3,  ///< height-X, depth-Y, width-Z
    DWH = 4,  ///< depth-X, width-Y, height-Z
    DHW = 5   ///< depth-X, height-Y, width-Z
};

/// Number of possible rotations
inline constexpr std::size_t kRotationCount = 6;

/// All rotation values in order
inline constexpr std::array<Rotation, kRotationCount> kAllRotations = {
    Rotation::WHD, Rotation::WDH, Rotation::HWD, Rotation::HDW, Rotation::DWH, Rotation::DHW};

/**
 * @brief Rotations that keep the original height on the Y-axis
 *
 * Used for "this side up" constraint - only WHD, WDH, DWH keep height vertical.
 * Actually, for "this side up", we keep the original Y dimension on Y axis,
 * which means rotations where the second letter is H: WHD, DHW.
 * And also rotations where height stays as height on Y.
 *
 * When this_side_up is true, we want rotations where the original height
 * dimension remains on the Y-axis. Looking at the naming:
 * - WHD: Height on Y ✓
 * - WDH: Depth on Y ✗
 * - HWD: Width on Y ✗
 * - HDW: Depth on Y ✗
 * - DWH: Width on Y ✗
 * - DHW: Height on Y ✓
 *
 * Wait, let me reconsider. The naming convention XYZ means:
 * - First letter: which dimension goes to X
 * - Second letter: which dimension goes to Y
 * - Third letter: which dimension goes to Z
 *
 * So for "this side up" (keep Height on Y):
 * - WHD: W->X, H->Y, D->Z ✓
 * - DHW: D->X, H->Y, W->Z ✓
 */
inline constexpr std::bitset<6> kThisSideUpRotations{0b100001};  // WHD and DHW

/**
 * @brief Get the name of a rotation (for debugging/display)
 */
[[nodiscard]] constexpr std::string_view rotation_name(Rotation rot) noexcept {
    switch (rot) {
        case Rotation::WHD:
            return "WHD";
        case Rotation::WDH:
            return "WDH";
        case Rotation::HWD:
            return "HWD";
        case Rotation::HDW:
            return "HDW";
        case Rotation::DWH:
            return "DWH";
        case Rotation::DHW:
            return "DHW";
    }
    return "Unknown";
}

/**
 * @brief Apply a rotation to dimensions
 *
 * Transforms the original dimensions to the rotated coordinate system.
 *
 * @param dims Original dimensions
 * @param rot Rotation to apply
 * @return Rotated dimensions
 */
[[nodiscard]] constexpr Dimensions apply_rotation(const Dimensions& dims, Rotation rot) noexcept {
    switch (rot) {
        case Rotation::WHD:
            return {dims.width, dims.height, dims.depth};
        case Rotation::WDH:
            return {dims.width, dims.depth, dims.height};
        case Rotation::HWD:
            return {dims.height, dims.width, dims.depth};
        case Rotation::HDW:
            return {dims.height, dims.depth, dims.width};
        case Rotation::DWH:
            return {dims.depth, dims.width, dims.height};
        case Rotation::DHW:
            return {dims.depth, dims.height, dims.width};
    }
    return dims;
}

/**
 * @brief Get allowed rotations for an item, considering its constraints
 *
 * @param constraints Item constraints
 * @return Bitset of allowed rotations
 */
[[nodiscard]] inline std::bitset<6>
get_allowed_rotations(const ItemConstraints& constraints) noexcept {
    auto allowed = constraints.allowed_rotations;
    if (constraints.this_side_up) {
        allowed &= kThisSideUpRotations;
    }
    return allowed;
}

/**
 * @brief Convert rotation enum to index
 */
[[nodiscard]] constexpr std::size_t rotation_index(Rotation rot) noexcept {
    return static_cast<std::size_t>(rot);
}

/**
 * @brief Convert index to rotation enum
 */
[[nodiscard]] constexpr Rotation rotation_from_index(std::size_t index) noexcept {
    return static_cast<Rotation>(index);
}

}  // namespace bp3d

#endif  // BP3D_ROTATION_HPP
