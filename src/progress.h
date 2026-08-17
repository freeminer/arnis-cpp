#pragma once
#include <string>
namespace arnis::progress { void reset_progress_floor(); void set_progress_suppressed(bool); double clamp_progress(double); bool emits_suppressed(double,const std::string&); }
