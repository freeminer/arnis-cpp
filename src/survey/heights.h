#pragma once

#include "survey.h"

#include <cstddef>
#include <vector>

struct XZBBox;

namespace arnis
{
class ProcessedElement;
struct Ground;
}

namespace arnis::survey::heights
{

struct Deferred
{
	std::size_t index = 0;
	float height = 0.0f;
	bool flat = false;
};

struct ApplyResult
{
	std::size_t filled = 0;
	std::vector<Deferred> deferred;
};

ApplyResult apply(std::vector<ProcessedElement> &elements, const Ground &ground,
		const ::XZBBox &bbox, double scale);
std::size_t apply_deferred(
		std::vector<ProcessedElement> &elements, const std::vector<Deferred> &deferred);

} // namespace arnis::survey::heights
