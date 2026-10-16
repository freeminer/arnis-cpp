#include "progress.h"

#include <algorithm>
#include <atomic>
#include <cmath>
#include <cstdint>
#include <exception>
#include <iostream>
#include <limits>
#include <mutex>
#include <utility>

namespace arnis::progress
{
namespace
{
std::atomic<std::uint32_t> progress_floor{0};
std::atomic<std::uint32_t> suppress_count{0};
std::mutex handler_mutex;
std::recursive_mutex emit_mutex;
EventHandler event_handler;

EventHandler current_handler()
{
	std::lock_guard<std::mutex> lock(handler_mutex);
	return event_handler;
}

void emit(Event event)
{
	auto handler = current_handler();
	if (!handler)
		return;
	try {
		std::lock_guard<std::recursive_mutex> lock(emit_mutex);
		handler(event);
	} catch (const std::exception &error) {
		std::cerr << "Failed to emit Arnis event: " << error.what() << '\n';
	} catch (...) {
		std::cerr << "Failed to emit Arnis event: unknown callback exception\n";
	}
}

std::string truncate_utf8(const std::string &text, std::size_t max_characters)
{
	std::size_t characters = 0;
	std::size_t end = 0;
	while (end < text.size() && characters < max_characters) {
		const auto byte = static_cast<unsigned char>(text[end]);
		// UTF-8 continuation bytes do not begin a new character. Advancing by
		// bytes this way also guarantees that truncation never splits a codepoint.
		if ((byte & 0xc0U) != 0x80U)
			++characters;
		++end;
	}
	return text.substr(0, end);
}
} // namespace

void set_event_handler(EventHandler handler)
{
	std::lock_guard<std::mutex> lock(handler_mutex);
	event_handler = std::move(handler);
}

bool has_event_handler()
{
	std::lock_guard<std::mutex> lock(handler_mutex);
	return static_cast<bool>(event_handler);
}

void reset_progress_floor()
{
	progress_floor.store(0, std::memory_order_relaxed);
}

void set_progress_suppressed(bool suppressed)
{
	if (suppressed) {
		suppress_count.fetch_add(1, std::memory_order_relaxed);
		return;
	}
	auto count = suppress_count.load(std::memory_order_relaxed);
	while (!suppress_count.compare_exchange_weak(count, count == 0 ? 0 : count - 1,
			std::memory_order_relaxed, std::memory_order_relaxed)) {
	}
}

bool emits_suppressed(double progress, const std::string &message)
{
	return suppress_count.load(std::memory_order_relaxed) > 0 && progress < 100.0 &&
		   message.rfind("Error!", 0) != 0;
}

double clamp_progress(double progress)
{
	if (progress <= 0.0)
		return progress;
	std::uint32_t value = 0;
	if (progress >=
			static_cast<double>(std::numeric_limits<std::uint32_t>::max()) / 100.0)
		value = std::numeric_limits<std::uint32_t>::max();
	else if (!std::isnan(progress)) // Rust's float-to-int cast maps NaN to zero.
		value = static_cast<std::uint32_t>(progress * 100.0);
	auto previous = progress_floor.load(std::memory_order_relaxed);
	while (previous < value &&
			!progress_floor.compare_exchange_weak(previous, value,
					std::memory_order_relaxed, std::memory_order_relaxed)) {
	}
	return static_cast<double>(std::max(previous, value)) / 100.0;
}

void emit_gui_progress_update(double progress, const std::string &message)
{
	if (emits_suppressed(progress, message) || !has_event_handler())
		return;
	Event event;
	event.kind = Event::Kind::Progress;
	event.progress = clamp_progress(progress);
	event.message = message;
	emit(std::move(event));
}

void emit_gui_progress_update_ex(
		double progress, const std::string &message, bool streaming)
{
	if (emits_suppressed(progress, message) || !has_event_handler())
		return;
	Event event;
	event.kind = Event::Kind::Progress;
	event.progress = clamp_progress(progress);
	event.message = message;
	event.streaming = streaming;
	emit(std::move(event));
}

void emit_gui_error(const std::string &message)
{
	const auto truncated = truncate_utf8(message, 35);
	emit_gui_progress_update(0.0, "Error! " + truncated);
}

void emit_world_name_update(const std::string &name)
{
	Event event;
	event.kind = Event::Kind::WorldName;
	event.message = name;
	emit(std::move(event));
}

void emit_map_preview_ready()
{
	Event event;
	event.kind = Event::Kind::MapPreviewReady;
	emit(std::move(event));
}

void emit_show_in_folder(const std::string &path)
{
	Event event;
	event.kind = Event::Kind::ShowInFolder;
	event.message = path;
	emit(std::move(event));
}
} // namespace arnis::progress
