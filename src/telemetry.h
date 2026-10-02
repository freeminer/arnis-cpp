#pragma once
#pragma once

#include <string>

namespace arnis::telemetry
{
enum class LogLevel
{
	Debug,
	Info,
	Warning,
	Error
};

void set_consent(bool);
bool consent();
const char *platform();
const char *event_name_generation_click();
std::string redact_url_queries(const std::string &message);
void send_log(LogLevel level, const std::string &message);
void send_generation_click();
void send_crash_report(const std::string &message);
} // namespace arnis::telemetry
