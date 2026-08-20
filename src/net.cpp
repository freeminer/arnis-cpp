#include "net.h"
#include <condition_variable>
#include <mutex>
namespace arnis::net
{
namespace
{
std::mutex mutex;
std::condition_variable freed;
std::size_t in_flight = 0;
}
RequestPermit::RequestPermit()
{
	std::unique_lock lock(mutex);
	freed.wait(lock, [] { return in_flight < MAX_CONCURRENT_REQUESTS; });
	++in_flight;
	held_ = true;
}
RequestPermit::~RequestPermit()
{
	if (!held_)
		return;
	{
		std::lock_guard lock(mutex);
		if (in_flight)
			--in_flight;
	}
	freed.notify_one();
}
RequestPermit::RequestPermit(RequestPermit &&other) noexcept : held_(other.held_)
{
	other.held_ = false;
}
RequestPermit request_permit()
{
	return RequestPermit{};
}
std::size_t in_flight_requests()
{
	std::lock_guard lock(mutex);
	return in_flight;
}
}
