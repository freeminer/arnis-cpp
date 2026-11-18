#pragma once

#include "survey.h"

#include "../coordinate_system/geographic/llbbox.h"

#include <filesystem>
#include <optional>

namespace arnis::survey
{
// Assemble and classify cached scan tiles on the source grid, fetching photos
// only when scans are complete and the estimated transfer fits max_megabytes.
std::optional<SurveyData> fetch_cached_survey_data(const geographic::LLBBox &bbox,
		std::size_t grid_width, std::size_t grid_height, Features features,
		std::uint32_t max_megabytes, const std::filesystem::path &cache_root);
} // namespace arnis::survey
