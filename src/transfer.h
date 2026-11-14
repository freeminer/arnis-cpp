#pragma once

#include <cstddef>
#include <cstdint>
#include <deque>
#include <string>
#include <utility>

namespace arnis::transfer
{
enum class Stage
{
	Download,
	Bake,
	Finalize,
};

struct Transfer
{
	Stage stage = Stage::Download;
	std::string name;
	std::size_t item = 0;
	std::size_t items = 0;
	std::uint64_t done_bytes = 0;
	std::uint64_t total_bytes = 0;
	double rate_bps = 0.0;
	std::size_t threads = 0;
	std::uint32_t cpu_pct = 0;
	std::uint32_t downloads = 0;
	double percent = 0.0;
};

double job_percent(std::size_t index, std::size_t items, Stage stage, double fraction);
std::size_t cores();
std::uint32_t cpu_pct(std::size_t threads, std::size_t logical_cores);

class RateMeter
{
	std::deque<std::pair<double, std::uint64_t>> samples_;

public:
	void reset();
	double add(double seconds, std::uint64_t bytes);
};
} // namespace arnis::transfer
