#pragma once

#include <chrono>
#include <iostream>

namespace arnis
{

// Small phase reporter matching the Rust generator's --benchmark output.
class Bench
{
public:
	using Clock = std::chrono::steady_clock;

	explicit Bench(bool enabled) : enabled_(enabled), last_(Clock::now()) {}

	void mark(const char *label)
	{
		if (!enabled_)
			return;
		report(label, Clock::now() - last_);
		last_ = Clock::now();
	}

	void reset() { last_ = Clock::now(); }

	void exclude(Clock::duration duration) { last_ += duration; }

	template <typename Rep, typename Period>
	void report(const char *label, std::chrono::duration<Rep, Period> duration) const
	{
		if (enabled_)
			std::cerr << "[BENCHMARK] " << label << "_ms="
					  << std::chrono::duration_cast<std::chrono::milliseconds>(duration)
								 .count()
					  << '\n';
	}

private:
	bool enabled_;
	Clock::time_point last_;
};

} // namespace arnis
