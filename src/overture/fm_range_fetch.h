#pragma once

#include <cstdint>
#include <future>
#include <map>
#include <mutex>
#include <optional>
#include <string>
#include <tuple>
#include <vector>

namespace arnis::overture
{
// Deduplicate only concurrent requests; completed results live in the disk cache.
class RangeFetches
{
	using Key = std::tuple<std::string, std::uint64_t, std::uint64_t>;
	using Result = std::optional<std::vector<std::uint8_t>>;
	std::mutex mutex_;
	std::map<Key, std::shared_future<Result>> pending_;

public:
	template <typename F>
	Result fetch(
			const std::string &url, std::uint64_t offset, std::uint64_t length, F &&load)
	{
		Key key{url, offset, length};
		std::shared_future<Result> future;
		std::promise<Result> promise;
		bool owner = false;
		{
			std::lock_guard lock(mutex_);
			const auto it = pending_.find(key);
			if (it != pending_.end()) {
				future = it->second;
			} else {
				future = promise.get_future().share();
				pending_.emplace(key, future);
				owner = true;
			}
		}
		if (owner) {
			try {
				promise.set_value(load());
			} catch (...) {
				promise.set_exception(std::current_exception());
			}
			std::lock_guard lock(mutex_);
			pending_.erase(key);
		}
		return future.get();
	}
};
} // namespace arnis::overture
