#pragma once
#include "choose.h"
#include <optional>
namespace arnis::building_facades
{
class FacadeSet
{
	std::vector<Entry> entries_;

public:
	static std::optional<FacadeSet> validate(std::vector<Entry>);
	const std::vector<Entry> &entries() const { return entries_; }
};
}
