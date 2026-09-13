#pragma once
#include "choose.h"
#include <filesystem>
#include <optional>
namespace arnis::building_facades
{
class FacadeSet
{
	std::vector<Entry> entries_;

public:
	static std::optional<FacadeSet> validate(std::vector<Entry>);
	// Rust's replacement-set path: manifest.json and every listed image live in
	// one directory.  Image decoding belongs to the embedding renderer; this
	// loader owns the schema, category validation, and safe file membership.
	static std::optional<FacadeSet> load_directory(
			const std::filesystem::path &, std::vector<std::string> *rejected = nullptr);
	const std::vector<Entry> &entries() const { return entries_; }
};
}
