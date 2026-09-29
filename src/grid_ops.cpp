#include "grid_ops.h"
#include "elevation/providers.h"

namespace arnis::grid_ops
{
double mercator_source_row(double top, double bottom, std::size_t rows, std::size_t gz)
{
	if (rows < 2)
		return 0.0;
	const double yt = elevation::providers::lat_to_mercator_y(top),
				 yb = elevation::providers::lat_to_mercator_y(bottom);
	const double lat = elevation::providers::mercator_y_to_lat(
			yt + (yb - yt) * double(gz) / double(rows - 1));
	const double span = top - bottom;
	if (std::abs(span) < 1e-15)
		return double(gz);
	return std::clamp((top - lat) / span * double(rows - 1), 0.0, double(rows - 1));
}
}
