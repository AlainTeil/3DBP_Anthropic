/**
 * @file parallel_solver.hpp
 * @brief Multi-threaded parallel solver that runs multiple algorithms
 */

#ifndef BP3D_ALGORITHMS_PARALLEL_SOLVER_HPP
#define BP3D_ALGORITHMS_PARALLEL_SOLVER_HPP

#include "bp3d/solver.hpp"

#include <memory>
#include <vector>

namespace bp3d {

/**
 * @brief Parallel solver that runs multiple algorithms concurrently
 *
 * Runs several packing algorithms in parallel and returns the best result as
 * ranked by the configured @ref PackingObjective (minimize bins, minimize cost,
 * or maximize utilization), always preferring a more complete packing first.
 */
class ParallelSolver : public ISolver {
public:
    /**
     * @brief Constructor with list of algorithms to run
     *
     * @param algorithms List of algorithms to run in parallel
     */
    explicit ParallelSolver(std::vector<Algorithm> algorithms);

    /**
     * @brief Default constructor - runs all offline algorithms
     */
    ParallelSolver();

    [[nodiscard]] PackingResult solve(std::span<const Item> items,
                                      const SolverConfig& config) override;

    [[nodiscard]] std::string_view name() const override { return "Parallel"; }

    [[nodiscard]] bool supports_online() const override { return false; }

private:
    std::vector<Algorithm> algorithms_;
};

/**
 * @brief Compare two packing results under the default objective.
 *
 * Equivalent to comparing under @ref PackingObjective::MinimizeBins.
 *
 * @return true if a is better than b
 */
[[nodiscard]] bool is_better_result(const PackingResult& a, const PackingResult& b);

/**
 * @brief Compare two packing results under a specific objective.
 *
 * Regardless of objective, a more complete packing (fewer unpacked items) is
 * always preferred first. Among equally complete results the objective decides:
 * fewest bins, lowest total bin cost, or highest utilization.
 *
 * @return true if a is better than b under @p objective
 */
[[nodiscard]] bool is_better_result(const PackingResult& a, const PackingResult& b,
                                    PackingObjective objective);

}  // namespace bp3d

#endif  // BP3D_ALGORITHMS_PARALLEL_SOLVER_HPP
