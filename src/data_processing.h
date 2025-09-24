#pragma once
#include "../../arnis_adapter.h"

namespace arnis
{

bool generate_world(
		WorldEditor &editor, const std::vector<ProcessedElement> &elements,
		const Args &args = {});

bool generate_world(WorldEditor &editor,
		const ProcessedElement &elements,
		const Args &args = {});
}