#include "system.hh"
#include "system_config.hh"

namespace slam {
System::System(const std::string& config_path) : config_path_(config_path) {
    InitConfigParams();
}

void System::InitConfigParams() {
    LOG_INFO("config_path: {}", config_path_);
    system_config_ptr_ = std::make_shared<SystemConfig>();
    if (!system_config_ptr_->LoadAndPrintConfig(config_path_)) {
        LOG_ERROR("Load config file failed, Program will exit!");
        std::exit(1);
    }
    LOG_INFO("System init done!");
}

System::~System() {
}
}  // namespace slam