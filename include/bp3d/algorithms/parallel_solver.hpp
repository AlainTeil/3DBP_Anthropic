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
 * Runs several packing algorithms in parallel and returns the best result
 * (by number of bins used, then by utilization).
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
 * @brief Compare two packing results to determine which is better
 *
 * @return true if a is better than b
 */
[[nodiscard]] bool is_better_result(const PackingResult& a, const PackingResult& b);

}  // namespace bp3d

#endif  // BP3D_ALGORITHMS_PARALLEL_SOLVER_HPP
