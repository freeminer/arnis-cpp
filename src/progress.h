#pragma once

#include <functional>
#include <optional>
#include <string>
#include <utility>
#include "transfer.h"

namespace arnis::progress
{
// Negative progress updates change only the status text and leave the bar alone.
inline constexpr double MESSAGE_ONLY = -1.0;

struct Event
{
	enum class Kind
	{
		Progress,
		WorldName,
		MapPreviewReady,
		ShowInFolder,
	};

	Kind kind = Kind::Progress;
	double progress = MESSAGE_ONLY;
	std::string message;
	std::optional<bool> streaming;
	std::optional<transfer::Transfer> transfer;
};

// Backend-neutral replacement for the Rust GUI emitter. Applications using
// Arnis as a library install a callback; generation itself has no GUI dependency.
using EventHandler = std::function<void(const Event &)>;
void set_event_handler(EventHandler handler);
bool has_event_handler();

void reset_progress_floor();
void set_progress_suppressed(bool suppressed);
double clamp_progress(double progress);
bool emits_suppressed(double progress, const std::string &message);
void emit_gui_progress_update(double progress, const std::string &message);
void emit_gui_progress_update_ex(
		double progress, const std::string &message, bool streaming);
void emit_gui_transfer(
		double progress, const std::string &message, const transfer::Transfer &transfer);
void emit_gui_error(const std::string &message);
void emit_world_name_update(const std::string &name);
void emit_map_preview_ready();
void emit_show_in_folder(const std::string &path);

class WaitingStatus
{
public:
	explicit WaitingStatus(const std::string &message);
	~WaitingStatus();
	WaitingStatus(const WaitingStatus &) = delete;
	WaitingStatus &operator=(const WaitingStatus &) = delete;

private:
	std::string previous_;
	bool active_{};
};

template <typename F>
decltype(auto) while_waiting(const std::string &message, F &&wait)
{
	WaitingStatus status(message);
	return std::forward<F>(wait)();
}
} // namespace arnis::progress
