#pragma once
#include <cstddef>
namespace arnis::net
{
inline constexpr std::size_t MAX_CONCURRENT_REQUESTS = 16;
class RequestPermit
{
	bool held_{false};

public:
	RequestPermit();
	~RequestPermit();
	RequestPermit(const RequestPermit &) = delete;
	RequestPermit &operator=(const RequestPermit &) = delete;
	RequestPermit(RequestPermit &&other) noexcept;
	RequestPermit &operator=(RequestPermit &&) = delete;
};
RequestPermit request_permit();
std::size_t in_flight_requests();
}
