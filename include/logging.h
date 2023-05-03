#pragma once

#include <chrono>
#include <mutex>
#include <source_location>
#include <sstream>
#include <string>
#include <type_traits>
#include <utility>

#include "esp_err.h"
#include "esp_log.h"

class Logging {
   public:
    using Ms = std::chrono::milliseconds;          ///< Typename alias
    using source_location = std::source_location;  ///< Typename alias

   private:
    using mutx_t = std::recursive_timed_mutex;  ///< Typename alias for the mutex used

    constexpr static esp_log_level_t default_level{ESP_LOG_INFO};  ///< Default logging level if none is set
    constexpr static Ms default_mutex_wait{100};                   ///< Default log timeout if none is set
    esp_log_level_t instance_level{default_level};                 ///< Minimum log level for this instance
    Ms instance_mutex_wait{default_mutex_wait};                    ///< Maximum log lock wait time for this instance

    static mutx_t mutx;  ///< API access mutex

    constexpr static size_t logf_buf_len{1024};  ///< snprintf logbuf length

    template <typename... Args>
    class GetLast;  ///< Forward declaration of helper class

   public:
    Logging(void) noexcept = default;  ///< Trivially constructable

    /*!
     * @brief  Construct a logging instance.
     *
     * @param  level             minimum log level for this instance;
     * @param  mutex_wait_ms     maximum log lock wait time for this instance;
     */
    constexpr Logging(const esp_log_level_t level, const Ms mutex_wait_ms) noexcept : instance_level{level}, instance_mutex_wait{mutex_wait_ms} {}

    /*!
     * @brief  Construct a logging instance.
     *
     * @param  level             minimum log level for this instance;
     */
    constexpr Logging(const esp_log_level_t level) noexcept : Logging{level, default_mutex_wait} {}

    /*!
     * @brief  Construct a logging instance.
     *
     * @param  mutex_wait_ms     maximum log lock wait time for this instance;
     */
    constexpr Logging(const Ms mutex_wait_ms) noexcept : Logging{default_level, mutex_wait_ms} {}

    /*!
     * @brief  Log a message at a given level. Will only log if above the default logging level.
     * Times out if the log is busy.
     *
     * @param  level        level to log this message at;
     * @param  msg          message to log;
     * @param  location     (optional) source location where the log originates from;
     *
     * @return  - ESP_OK if message logged;
     *          - ESP_ERR_INVALID_STATE if requested level is below our default minimum;
     *          - ESP_ERR_TIMEOUT if timed out waiting to log the message;
     */
    static esp_err_t log(const esp_log_level_t level,
                         const std::string_view& msg,
                         const source_location& location = source_location::current()) {
        if (default_level < level) return ESP_ERR_INVALID_STATE;

        if (mutx.try_lock_for(default_mutex_wait)) {
            ESP_LOG_LEVEL(level,
                          location.file_name(),
                          "[%d:%s]: %.*s",
                          location.line(),
                          location.function_name(),
                          msg.length(),
                          msg.data());
            mutx.unlock();

            return ESP_OK;
        }

        return ESP_ERR_TIMEOUT;
    }

    /*!
     * @brief  Log a message at a given level. Will only log if above the default logging level.
     * Times out if the log is busy.
     *
     * @param  level    level to log this message at;
     * @param  args     (variadic) args to be passed to the std::stringstream;
     *
     * @return  - ESP_OK if message logged;
     *          - ESP_ERR_INVALID_STATE if requested level is below our default minimum;
     *          - ESP_ERR_TIMEOUT if timed out waiting to log the message;
     */
    template <typename... Args>
    static esp_err_t log(const esp_log_level_t level, const Args&... args) {
        std::ostringstream strem;
        args_to_strem(stream, args...);
        return log(level, stream.str());
    }

    /*!
     * @brief  Log a printf style message at a given level. Will only log if above the default logging level.
     * Times out if the log is busy.
     *
     * @note  This is the version when source_location is the last arg.
     *
     * @param  level    level to log this message at;
     * @param  format   printf format cstring;
     * @param  args     (variadic) args to be passed to the std::stringstream. Last arg can be a source_location;
     *
     * @return  - ESP_OK if message logged;
     *          - ESP_ERR_INVALID_STATE if requested level is below our default minimum;
     *          - ESP_ERR_TIMEOUT if timed out waiting to log the message;
     */
    template <typename... Args, std::enable_if_t<std::is_same_v<typename GetLast<Args...>::type, source_location>, bool> = true>  // valid only if last arg is source_location
    static esp_err_t logf(const esp_log_level_t level, const char* const format, const Args&... args) {
        char buf[logf_buf_len]{};
        snprintf(buf, sizeof(buf), format, args...);  // FIXME
        return log(level, buf, GetLast(args...).value);
    }

    /*!
     * @brief  Log a printf style message at a given level. Will only log if above the default logging level.
     * Times out if the log is busy.
     *
     * @note  This is the version when source_location is not included as the last arg.
     *
     * @param  level    level to log this message at;
     * @param  format   printf format cstring;
     * @param  args     (variadic) args to be passed to the std::stringstream;
     *
     * @return  - ESP_OK if message logged;
     *          - ESP_ERR_INVALID_STATE if requested level is below our default minimum;
     *          - ESP_ERR_TIMEOUT if timed out waiting to log the message;
     */
    template <typename... Args, std::enable_if_t<!std::is_same_v<typename GetLast<Args...>::type, source_location>, bool> = true>  // valid only if last arg is not source_location
    static esp_err_t logf(const esp_log_level_t level, const char* const format, const Args&... args) {
        char buf[logf_buf_len]{};
        snprintf(buf, sizeof(buf), format, args...);
        return log(level, buf);
    }

    /*!
     * @brief  Log a buffer of hex bytes at specified level, separated into 16 bytes each line.
     * Will only log if above the default logging level. Times out if the log is busy.
     *
     * @note  This is the version when source_location is not included as the last arg.
     *
     * @param  level        level to log this message at;
     * @param  buf          buffer to log;
     * @param  buf_len      buffer length in bytes
     * @param  location     (optional) source location where the log originates from;
     *
     * @return  - ESP_OK if message logged;
     *          - ESP_ERR_INVALID_ARG if buffer is nullptr or length is zero;
     *          - ESP_ERR_INVALID_STATE if requested level is below our default minimum;
     *          - ESP_ERR_TIMEOUT if timed out waiting to log the message;
     */
    static esp_err_t hex(const esp_log_level_t level,
                         const void* buf,
                         const size_t buf_len,
                         const source_location& location = source_location::current()) {
        if (default_level < level) return ESP_ERR_INVALID_STATE;

        if (buf && 0 < buf_len) {
            if (mutx.try_lock_for(default_mutex_wait)) {
                ESP_LOG_BUFFER_HEX_LEVEL(location.file_name(), buf, buf_len, level);
                mutx.unlock();

                return ESP_OK;
            }
            return ESP_ERR_TIMEOUT;
        }
        return ESP_ERR_INVALID_ARG;
    }

    /*!
     * @brief  Dump a buffer to the log at specified level.
     * Will only log if above the default logging level. Times out if the log is busy.
     *
     * @note  This is the version when source_location is not included as the last arg.
     *
     * @param  level        level to log this message at;
     * @param  buf          buffer to log;
     * @param  buf_len      buffer length in bytes
     * @param  location     (optional) source location where the log originates from;
     *
     * @return  - ESP_OK if message logged;
     *          - ESP_ERR_INVALID_ARG if buffer is nullptr or length is zero;
     *          - ESP_ERR_INVALID_STATE if requested level is below our default minimum;
     *          - ESP_ERR_TIMEOUT if timed out waiting to log the message;
     */
    static esp_err_t hexdump(const esp_log_level_t level,
                             const void* buf,
                             const size_t buf_len,
                             const source_location& location = source_location::current()) {
        if (default_level < level) return ESP_ERR_INVALID_STATE;

        if (buf && 0 < buf_len) {
            if (mutx.try_lock_for(default_mutex_wait)) {
                ESP_LOG_BUFFER_HEXDUMP(location.file_name(), buf, buf_len, level);
                mutx.unlock();

                return ESP_OK;
            }
            return ESP_ERR_TIMEOUT;
        }
        return ESP_ERR_INVALID_ARG;
    }

    /*!
     * @brief  Log a message at a given level. Will only log if above the default logging level.
     * Times out if the log is busy.
     *
     * @note  operator overload so we can just call the class name as a function.
     *
     * @param  level        level to log this message at;
     * @param  msg          message to log;
     * @param  location     (optional) source location where the log originates from;
     *
     * @return  - ESP_OK if message logged;
     *          - ESP_ERR_INVALID_STATE if requested level is below our default minimum;
     *          - ESP_ERR_TIMEOUT if timed out waiting to log the message;
     */
    esp_err_t operator()(const esp_log_level_t level,
                         const std::string_view& msg,
                         const source_location& location = source_location.current()) {
        return log(level, msg, location);
    }
};