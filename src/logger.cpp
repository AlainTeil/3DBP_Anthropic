/**
 * @file logger.cpp
 * @brief Logger implementation
 */

#include "bp3d/logger.hpp"

#include <iostream>

namespace bp3d {

// Static member definitions
LogCallback Logger::callback_ = nullptr;
LogLevel Logger::min_level_ = LogLevel::Info;

void Logger::set_callback(LogCallback callback) {
    callback_ = std::move(callback);
}

void Logger::set_level(LogLevel level) {
    min_level_ = level;
}

LogLevel Logger::get_level() noexcept {
    return min_level_;
}

void Logger::log(LogLevel level, std::string_view message) {
    if (level < min_level_) {
        return;
    }

    if (callback_) {
        callback_(level, message);
    } else {
        // Default: print to stderr
        std::cerr << "[" << log_level_name(level) << "] " << message << "\n";
    }
}

void Logger::debug(std::string_view message) {
    log(LogLevel::Debug, message);
}

void Logger::info(std::string_view message) {
    log(LogLevel::Info, message);
}

void Logger::warning(std::string_view message) {
    log(LogLevel::Warning, message);
}

void Logger::error(std::string_view message) {
    log(LogLevel::Error, message);
}

}  // namespace bp3d
