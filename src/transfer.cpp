#include "transfer.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <thread>

namespace arnis::transfer
{
namespace
{
constexpr double DOWNLOAD_END = 40.0;
constexpr double BAKE_END = 85.0;
constexpr double RATE_WINDOW_S = 8.0;
} // namespace

double job_percent(std::size_t index, std::size_t items, Stage stage, double fraction)
{
	if (!std::isfinite(fraction))
		fraction = 0.0;
	fraction = std::clamp(fraction, 0.0, 1.0);
	double within = 0.0;
	switch (stage) {
	case Stage::Download:
		within = DOWNLOAD_END * fraction;
		break;
	case Stage::Bake:
		within = DOWNLOAD_END + (BAKE_END - DOWNLOAD_END) * fraction;
		break;
	case Stage::Finalize:
		within = BAKE_END + (100.0 - BAKE_END) * fraction;
		break;
	}
	const double pct = (100.0 * static_cast<double>(index) + within) /
					   static_cast<double>(std::max<std::size_t>(items, 1));
	return std::min(pct, 100.0);
}

std::size_t cores()
{
	const auto count = std::thread::hardware_concurrency();
	return count ? count : 4;
}

std::uint32_t cpu_pct(std::size_t threads, std::size_t logical_cores)
{
	logical_cores = std::max<std::size_t>(logical_cores, 1);
	const auto rounded = (threads * 100 + logical_cores / 2) / logical_cores;
	return static_cast<std::uint32_t>(std::min<std::size_t>(rounded, 100));
}

void RateMeter::reset()
{
	samples_.clear();
}

double RateMeter::add(double seconds, std::uint64_t bytes)
{
	if (!std::isfinite(seconds))
		return 0.0;
	if (!samples_.empty() && bytes < samples_.back().second)
		samples_.clear();
	samples_.emplace_back(seconds, bytes);
	while (samples_.size() > 2 && samples_[1].first <= seconds - RATE_WINDOW_S)
		samples_.pop_front();
	const auto [start_time, start_bytes] = samples_.front();
	const double elapsed = seconds - start_time;
	if (elapsed < 0.25 || bytes < start_bytes)
		return 0.0;
	return static_cast<double>(bytes - start_bytes) / elapsed;
}
} // namespace arnis::transfer
