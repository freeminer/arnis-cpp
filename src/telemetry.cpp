#include "telemetry.h"
#include <atomic>
namespace arnis::telemetry { static std::atomic<bool> enabled{false}; void set_consent(bool v){enabled.store(v,std::memory_order_relaxed);} bool consent(){return enabled.load(std::memory_order_relaxed);} }
namespace arnis::telemetry { const char *platform(){
#if defined(_WIN32)
return "windows";
#elif defined(__APPLE__)
return "macos";
#elif defined(__linux__)
return "linux";
#else
return "unknown";
#endif
} const char *event_name_generation_click(){return "generation_click";} }
