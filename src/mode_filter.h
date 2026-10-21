#pragma once

#include "../../arnis_adapter.h"
#include "args.h"

#include <vector>

namespace arnis::mode_filter
{
void retain_for_mode(std::vector<ProcessedElement> &elements, GenerationMode mode);

template <typename OutlineSuppression>
OutlineSuppression outline_suppression_for_mode(
		OutlineSuppression suppression, GenerationMode mode)
{
	if (mode == GenerationMode::Blueprint)
		suppression.clear();
	return suppression;
}
}
