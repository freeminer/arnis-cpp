#pragma once
#include <cstddef>
#include <map>
#include <mutex>
#include <optional>
#include <set>
#include <string>
#include <vector>
namespace arnis::net
{
inline constexpr std::size_t MAX_CONCURRENT_REQUESTS = 16;
// Runtime ceiling corresponding to Rust's --max-downloads setting.
void set_max_requests(std::size_t count);
std::size_t max_requests();
class RequestPermit
{
	bool held_{false};

public:
	explicit RequestPermit(const std::string &resource = "network request");
	~RequestPermit();
	RequestPermit(const RequestPermit &) = delete;
	RequestPermit &operator=(const RequestPermit &) = delete;
	RequestPermit(RequestPermit &&other) noexcept;
	RequestPermit &operator=(RequestPermit &&) = delete;
};
RequestPermit request_permit(const std::string &resource = "network request");
std::size_t in_flight_requests();
// A generation run can disable only network requests that are reached after a
// cache miss. Providers should acquire a permit only after checking their cache.
void set_offline(bool enabled);
bool offline();
std::vector<std::pair<std::string, std::size_t>> offline_misses();
void clear_offline_misses();
// A prewarm permits network access but records source data that cannot be
// retained in the cache, so callers do not report an incomplete warmup as ready.
void set_prewarm(bool enabled);
bool prewarm();
void note_uncached(const std::string &resource);
std::vector<std::string> uncached();
// Persistent front-end setting; unlike `offline()`, this does not itself
// prevent requests and is combined only for telemetry/update policy.
void set_offline_mode(bool enabled);
bool offline_mode();
class RunScope
{
	bool active_{false};
	std::optional<std::size_t> previous_request_limit_;

public:
	explicit RunScope(bool offline_run, bool prewarm_run = false,
			std::optional<std::size_t> request_limit = std::nullopt);
	~RunScope();
	RunScope(const RunScope &) = delete;
	RunScope &operator=(const RunScope &) = delete;
};
// Coordinate-free category matching Rust's reqwest failure tags.
std::string failure_kind(int transport_error, long http_status);
bool is_failure_kind(const std::string &tag);
}
