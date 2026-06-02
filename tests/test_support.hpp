/**
 * @file test_support.hpp
 * @brief Shared test utilities for filesystem isolation.
 *
 * Provides per-test unique temporary directories so that tests can run
 * concurrently (e.g. `ctest -j`) without colliding on shared paths.
 */

#ifndef BP3D_TEST_SUPPORT_HPP
#define BP3D_TEST_SUPPORT_HPP

#include <atomic>
#include <filesystem>
#include <random>
#include <string>
#include <string_view>
#include <system_error>

namespace bp3d::test {

/**
 * @brief Create a unique temporary directory for a test.
 *
 * The path is randomized so it is unique both across parallel test
 * processes and across fixtures within a single process. The directory
 * is created before returning.
 *
 * @param label Short human-readable tag embedded in the directory name.
 * @return Path to the freshly created unique directory.
 */
inline std::filesystem::path make_unique_temp_dir(std::string_view label) {
    static std::atomic<unsigned long long> counter{0};

    std::random_device rd;
    const auto token = (static_cast<unsigned long long>(rd()) << 32) ^ rd();
    const auto seq = counter.fetch_add(1, std::memory_order_relaxed);

    std::filesystem::path dir =
        std::filesystem::temp_directory_path() /
        ("bp3d_" + std::string(label) + "_" + std::to_string(token) + "_" + std::to_string(seq));
    std::filesystem::create_directories(dir);
    return dir;
}

/**
 * @brief Remove a temporary directory, ignoring errors.
 *
 * Safe to call when the directory no longer exists; never throws, so it
 * is suitable for use in test teardown.
 */
inline void remove_temp_dir(const std::filesystem::path& dir) noexcept {
    std::error_code ec;
    std::filesystem::remove_all(dir, ec);
}

}  // namespace bp3d::test

#endif  // BP3D_TEST_SUPPORT_HPP
