#pragma once
#include <glog/logging.h>
#include <ros/ros.h>
#include <spdlog/async.h>
#include <spdlog/common.h>
#include <spdlog/fmt/ostr.h>
#include <spdlog/sinks/daily_file_sink.h>
#include <spdlog/sinks/stdout_color_sinks.h>
#include <spdlog/spdlog.h>
#include <chrono>
#include <ctime>
#include <sstream>
#include <string>
#include <unordered_map>

#define LOG_TRACE(...) SPDLOG_LOGGER_TRACE(spdlog::default_logger_raw(), __VA_ARGS__)
#define LOG_DEBUG(...) SPDLOG_LOGGER_DEBUG(spdlog::default_logger_raw(), __VA_ARGS__)
#define LOG_INFO(...) SPDLOG_LOGGER_INFO(spdlog::default_logger_raw(), __VA_ARGS__)
#define LOG_WARN(...) SPDLOG_LOGGER_WARN(spdlog::default_logger_raw(), __VA_ARGS__)
#define LOG_ERROR(...) SPDLOG_LOGGER_ERROR(spdlog::default_logger_raw(), __VA_ARGS__)
#define LOG_FATAL(...) SPDLOG_LOGGER_CRITICAL(spdlog::default_logger_raw(), __VA_ARGS__)

class SpdLogger {
    struct Config {
        std::string name;
        std::string dir;
        size_t queue_size;
        std::string pattern;
        int flush_interval;
        spdlog::level::level_enum console_level;
        spdlog::level::level_enum file_level;

        Config()
            : name("slam"),
              dir("/home/kilox/rcs/logs/"),
              queue_size(8192),
              pattern("[%Y-%m-%d %H:%M:%S.%e] [%^%l%$] [%s:%#] %v"),
              flush_interval(3.0),
              console_level(spdlog::level::info),
              file_level(spdlog::level::info) {
        }
    };

   public:
    explicit SpdLogger(const Config config = Config()) : config_(config) {
        init_thread_pool();
        setup_logger();
        // 打印日志设置信息

        LOG_INFO("Logger initialized with name: {}", config_.name);
        LOG_INFO("Logger directory: {}", config_.dir);
        LOG_INFO("Logger console level: {}", spdlog::level::to_string_view(config_.console_level));
        LOG_INFO("Logger file level: {}", spdlog::level::to_string_view(config_.file_level));
    }

    ~SpdLogger() {
        spdlog::drop_all();
    }

   private:
    void init_thread_pool() {
        // Initialize the thread pool if not already initialized
        if (!spdlog::details::registry::instance().get_tp()) {
            spdlog::init_thread_pool(config_.queue_size, 1);
        }
    }

    void setup_logger() {
        auto sinks = create_sinks();
        logger_ =
            std::make_shared<spdlog::async_logger>(config_.name, sinks.begin(), sinks.end(), spdlog::thread_pool(),
                                                   spdlog::async_overflow_policy::overrun_oldest);

        logger_->set_level(spdlog::level::trace);
        // logger_->set_pattern(config_.pattern);
        logger_->flush_on(spdlog::level::info);
        // 设置刷新频率

        spdlog::register_logger(logger_);
        spdlog::set_default_logger(logger_);
        spdlog::flush_every(std::chrono::seconds(config_.flush_interval));
    }

    std::vector<spdlog::sink_ptr> create_sinks() {
        std::vector<spdlog::sink_ptr> sinks;

        // 控制台输出
        auto console_sink = std::make_shared<spdlog::sinks::stdout_color_sink_mt>();
        get_env_level("LOG_CONSOLE_LEVEL", config_.console_level);
        console_sink->set_level(config_.console_level);
        console_sink->set_pattern(config_.pattern);
        sinks.push_back(console_sink);

        // 每日自动轮转文件
        // 午夜自动轮转
        auto file_sink = std::make_shared<spdlog::sinks::daily_file_sink_mt>(format_filename(), 0, 0);
        get_env_level("LOG_SAVE_LEVEL", config_.file_level);
        file_sink->set_level(config_.file_level);
        file_sink->set_pattern(config_.pattern);
        sinks.push_back(file_sink);

        return sinks;
    }

    std::string format_filename() const {
        std::string result = config_.dir;
        if (!result.empty() && result.back() != '/') {
            result += '/';
        }
        result += config_.name;
        result += ".log";
        return result;
    }

    void get_env_level(const char* env_var, spdlog::level::level_enum& default_level) {
        //"INFO" "DEBUG" "TRACE" "ERROR" "WARN" "CRITICAL"
        const char* level_str = std::getenv(env_var);
        // 变为小写
        std::unordered_map<std::string, spdlog::level::level_enum> level_map = {
            {"INFO", spdlog::level::info}, {"DEBUG", spdlog::level::debug}, {"TRACE", spdlog::level::trace},
            {"ERROR", spdlog::level::err}, {"WARN", spdlog::level::warn},   {"CRITICAL", spdlog::level::critical},
        };
        if (!level_str) {
            return;
        }
        auto it = level_map.find(level_str);
        if (it != level_map.end()) {
            default_level = it->second;
        } else {
            return;
        }
    }

    // 成员变量
    Config config_;
    std::shared_ptr<spdlog::async_logger> logger_;
};
