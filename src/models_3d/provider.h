#pragma once
#include "model_asset.h"
#include <optional>
#include <string>
#include <unordered_map>
namespace arnis::models_3d
{
class ModelProvider
{
public:
	virtual ~ModelProvider() = default;
	virtual std::optional<ModelAsset> fetch(const std::string &key) = 0;
};
inline bool provider_has(ModelProvider &provider, const std::string &key)
{
	return provider.fetch(key).has_value();
}
class MemoProvider : public ModelProvider
{
	ModelProvider &inner_;
	std::unordered_map<std::string, std::optional<ModelAsset>> cache_;
	std::size_t limit_ = 256;
	std::size_t hits_ = 0, misses_ = 0;
	bool cache_failures_ = true;
	std::size_t max_key_length_ = 512;
	bool enabled_ = true;

public:
	explicit MemoProvider(ModelProvider &inner) : inner_(inner) {}
	void set_limit(std::size_t limit)
	{
		limit_ = limit;
		if (cache_.size() > limit_)
			cache_.clear();
	}
	void cache_failures(bool enabled) { cache_failures_ = enabled; }
	void max_key_length(std::size_t length) { max_key_length_ = length; }
	void enabled(bool value) { enabled_ = value; }
	std::optional<ModelAsset> fetch(const std::string &key) override
	{
		if (key.size() > max_key_length_)
			return std::nullopt;
		if (!enabled_)
			return inner_.fetch(key);
		auto it = cache_.find(key);
		if (it != cache_.end()) {
			++hits_;
			return it->second;
		}
		++misses_;
		auto a = inner_.fetch(key);
		if (cache_failures_ || a) {
			if (cache_.size() >= limit_)
				cache_.clear();
			cache_.emplace(key, a);
		}
		return a;
	}
	std::size_t hits() const { return hits_; }
	std::size_t misses() const { return misses_; }
	void reset_stats() { hits_ = misses_ = 0; }
	void clear() { cache_.clear(); }
	void invalidate(const std::string &key) { cache_.erase(key); }
};
}
