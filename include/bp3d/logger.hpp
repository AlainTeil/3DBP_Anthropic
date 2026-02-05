/**
 * @file logger.hpp
 * @brief Custom callback-based logging interface
 */

#ifndef BP3D_LOGGER_HPP
#define BP3D_LOGGER_HPP

#include <functional>
#include <string>
#include <string_view>

namespace bp3d {

/**
 * @brief Log severity levels
 */
enum class LogLevel {
    Debug,    ///< Detailed debugging information
    Info,     ///< General information
    Warning,  ///< Warning messages
    Error     ///< Error messages
};

/**
 * @brief Get string name of a log level
 */
[[nodiscard]] constexpr std::string_view log_level_name(LogLevel level) noexcept {
    switch (level) {
        case LogLevel::Debug:
            return "DEBUG";
        case LogLevel::Info:
            return "INFO";
        case LogLevel::Warning:
            return "WARNING";
        case LogLevel::Error:
            return "ERROR";
    }
    return "UNKNOWN";
}

/**
 * @brief Callback type for log messages
 *
 * @param level Log severity level
 * @param message Log message
 */
using LogCallback = std::function<void(LogLevel level, std::string_view message)>;

/**
 * @brief Logger class with callback-based output
 *
 * Allows users to plug in their own logging infrastructure.
 */
class Logger {
public:
    /**
     * @brief Set the log callback
     *
     * @param callback Function to call for each log message
     */
    static void set_callback(LogCallback callback);

    /**
     * @brief Set the minimum log level
     *
     * Messages below this level will be filtered out.
     *
     * @param level Minimum level to log
     */
    static void set_level(LogLevel level);

    /**
     * @brief Get the current minimum log level
     */
    [[nodiscard]] static LogLevel get_level() noexcept;

    /**
     * @brief Log a debug message
     */
    static void debug(std::string_view message);

    /**
     * @brief Log an info message
     */
    static void info(std::string_view message);

    /**
     * @brief Log a warning message
     */
    static void warning(std::string_view message);

    /**
     * @brief Log an error message
     */
    static void error(std::string_view message);

    /**
     * @brief Log a message at the specified level
     */
    static void log(LogLevel level, std::string_view message);

private:
    static LogCallback callback_;
    static LogLevel min_level_;
};

}  // namespace bp3d

#endif  // BP3D_LOGGER_HPP
