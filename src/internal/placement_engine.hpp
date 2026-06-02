/**
 * @file placement_engine.hpp
 * @brief Shared placement core for corner/point-based packing algorithms.
 *
 * Centralizes bin management, candidate generation, constraint validation
 * (collision, support, do-not-stack, weight / stack-weight limits) and the
 * book-keeping (item registry, used weight/volume) that the First-Fit,
 * Best-Fit and Extreme-Point families previously duplicated.
 */

#ifndef BP3D_INTERNAL_PLACEMENT_ENGINE_HPP
#define BP3D_INTERNAL_PLACEMENT_ENGINE_HPP

#include "bp3d/config.hpp"
#include "bp3d/constraints/validator.hpp"
#include "bp3d/result.hpp"
#include "bp3d/rotation.hpp"
#include "bp3d/types.hpp"
#include "internal/spatial_grid.hpp"

#include <algorithm>
#include <chrono>
#include <cstddef>
#include <limits>
#include <memory>
#include <optional>
#include <span>
#include <string>
#include <vector>

namespace bp3d::internal {

/**
 * @brief Whether a solving deadline has been reached.
 *
 * Returns true once the elapsed time since @p start is at least @p timeout.
 * A non-positive @p timeout disables the limit and always returns false, so
 * callers can pass the configured timeout unconditionally.
 *
 * @param start  Time point captured at the start of solving.
 * @param timeout Maximum allowed duration (<= 0 means "no limit").
 */
[[nodiscard]] inline bool deadline_reached(std::chrono::steady_clock::time_point start,
                                           std::chrono::milliseconds timeout) noexcept {
    if (timeout <= std::chrono::milliseconds::zero()) {
        return false;
    }
    return (std::chrono::steady_clock::now() - start) >= timeout;
}

/// State of a single bin during packing.
struct EngineBin {
    BinType type;                       ///< The bin type backing this instance
    int index = 0;                      ///< 0-based instance index
    std::vector<Placement> placements;  ///< Items already committed to this bin
    double used_weight = 0.0;           ///< Sum of committed item weights
    double used_volume = 0.0;           ///< Sum of committed item volumes
};

/**
 * @brief Shared placement engine for corner-based packing algorithms.
 *
 * The engine owns the bins, the constraint pipeline and the item registry.
 * Algorithms supply only their candidate ordering / scoring policy through the
 * @ref find_first_fit and @ref find_best_in_bin templates.
 */
class PlacementEngine {
public:
    /**
     * @param config Solver configuration (bin types, options).
     * @param require_support When true, every placement must rest on the floor
     *        or another item (gravity); when false the support requirement is
     *        skipped while collision / do-not-stack / weight limits still apply.
     */
    PlacementEngine(SolverConfig config, bool require_support);

    [[nodiscard]] const SolverConfig& config() const noexcept { return config_; }

    [[nodiscard]] std::vector<EngineBin>& bins() noexcept { return bins_; }
    [[nodiscard]] const std::vector<EngineBin>& bins() const noexcept { return bins_; }

    /// Open a new (empty) bin of the given type. Returns its instance index.
    int open_bin(const BinType& type);

    /// Drop the most recently opened bin if it received no placement.
    void discard_last_bin_if_empty();

    /// Generate deduplicated corner candidate positions for a bin.
    [[nodiscard]] std::vector<Vector3> candidates(const EngineBin& bin) const;

    /// Validate a fully specified placement against the constraint pipeline.
    [[nodiscard]] bool is_valid(const Item& item, const Placement& placement,
                                const EngineBin& bin) const;

    /// Commit a validated placement: updates the bin, registry and accumulators.
    void commit(EngineBin& bin, const Item& item, const Placement& placement);

    /**
     * @brief Find the first valid placement for an item in a bin.
     *
     * Candidate positions are ordered by @p position_less (a strict-weak
     * ordering over Vector3); the first position/rotation pair that satisfies
     * every constraint is returned.
     */
    template <class PositionLess>
    [[nodiscard]] std::optional<Placement> find_first_fit(const Item& item, const EngineBin& bin,
                                                          PositionLess position_less) const {
        auto positions = candidates(bin);
        std::sort(positions.begin(), positions.end(), position_less);
        const auto allowed = get_allowed_rotations(item.constraints);
        for (const auto& pos : positions) {
            for (std::size_t ri = 0; ri < kRotationCount; ++ri) {
                if (!allowed.test(ri)) {
                    continue;
                }
                const Rotation rotation = rotation_from_index(ri);
                const Dimensions rotated = apply_rotation(item.dimensions, rotation);
                Placement candidate{item.id, bin.type.id, bin.index, pos, rotation, rotated};
                if (is_valid(item, candidate, bin)) {
                    return candidate;
                }
            }
        }
        return std::nullopt;
    }

    /**
     * @brief Find the lowest-scoring valid placement for an item in a bin.
     *
     * @p score maps a candidate Placement to a double; the valid candidate with
     * the minimum score across all positions/rotations is returned.
     */
    template <class Score>
    [[nodiscard]] std::optional<Placement> find_best_in_bin(const Item& item, const EngineBin& bin,
                                                            Score score) const {
        std::optional<Placement> best;
        double best_score = std::numeric_limits<double>::max();
        const auto positions = candidates(bin);
        const auto allowed = get_allowed_rotations(item.constraints);
        for (const auto& pos : positions) {
            for (std::size_t ri = 0; ri < kRotationCount; ++ri) {
                if (!allowed.test(ri)) {
                    continue;
                }
                const Rotation rotation = rotation_from_index(ri);
                const Dimensions rotated = apply_rotation(item.dimensions, rotation);
                const Placement candidate{item.id, bin.type.id, bin.index, pos, rotation, rotated};
                if (!is_valid(item, candidate, bin)) {
                    continue;
                }
                const double s = score(candidate);
                if (s < best_score) {
                    best_score = s;
                    best = candidate;
                }
            }
        }
        return best;
    }

private:
    SolverConfig config_;
    std::unique_ptr<CompositeValidator> validator_;
    ItemRegistry registry_;
    std::vector<EngineBin> bins_;

    // One spatial index per bin (parallel to bins_), accelerating the
    // collision / support neighbour lookups inside is_valid().
    std::vector<SpatialGrid> grids_;

    // Reusable scratch buffers for gathering a candidate's nearby placements
    // (mutable: is_valid() is logically const but reuses these to avoid
    // per-call allocation). The engine is never shared across threads.
    mutable std::vector<std::size_t> neighbor_indices_;
    mutable std::vector<Placement> neighbor_scratch_;
};

/**
 * @brief Build a packing result from a range of bins.
 *
 * Works with any bin type exposing a `type` (BinType) and `placements`
 * (range of Placement) — the engine's @ref EngineBin as well as the
 * specialized per-bin states of the extreme-point, shelf and guillotine
 * solvers. Empty bins are ignored; item volume is summed from the placed
 * (rotated) dimensions, which is rotation-invariant.
 *
 * @tparam Bins Range of bins, each with `.type` and `.placements` members.
 */
template <class Bins>
[[nodiscard]] PackingResult make_result(std::string algorithm, const Bins& bins,
                                        std::vector<std::string> unpacked,
                                        std::chrono::steady_clock::time_point start) {
    PackingResult result;
    result.algorithm = std::move(algorithm);
    result.unpacked_items = std::move(unpacked);

    int bins_used = 0;
    double total_item_volume = 0.0;
    double total_bin_volume = 0.0;
    double total_cost = 0.0;
    for (const auto& bin : bins) {
        if (bin.placements.empty()) {
            continue;
        }
        ++bins_used;
        total_bin_volume += bin.type.volume();
        total_cost += bin.type.cost;
        for (const auto& p : bin.placements) {
            total_item_volume += p.rotated_dimensions.volume();
            result.placements.push_back(p);
        }
    }

    result.bins_used = bins_used;
    result.stats.total_item_volume = total_item_volume;
    result.stats.total_bin_volume = total_bin_volume;
    result.stats.total_cost = total_cost;
    result.stats.utilization = total_bin_volume > 0.0 ? total_item_volume / total_bin_volume : 0.0;

    const auto end = std::chrono::steady_clock::now();
    result.stats.execution_time =
        std::chrono::duration_cast<std::chrono::microseconds>(end - start);
    return result;
}

}  // namespace bp3d::internal

#endif  // BP3D_INTERNAL_PLACEMENT_ENGINE_HPP
