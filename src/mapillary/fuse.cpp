#include "fuse.h"

namespace arnis::mapillary::fuse
{
FusedTexture fuse(const std::string &wall_key, const std::vector<ViewTexture> &views,
		unsigned pixels_per_block, const Params &params)
{
	(void)params; // Rust's current fuse() also keeps Params for API consistency.
	FusedTexture result;
	result.wall_key = wall_key;
	if (views.empty() || !pixels_per_block)
		return result;
	const auto width = views.front().rgb.width;
	const auto height = views.front().rgb.height;
	const auto count = std::size_t(width) * height;
	if (!width || !height || views.front().rgb.pixels.size() < count)
		return result;
	for (const auto &view : views)
		if (view.rgb.width != width || view.rgb.height != height ||
				view.rgb.pixels.size() < count || view.valid.size() != count)
			return result;

	std::vector<std::size_t> order(views.size());
	for (std::size_t i = 0; i < order.size(); ++i)
		order[i] = i;
	std::stable_sort(order.begin(), order.end(), [&](std::size_t a, std::size_t b) {
		if (std::isfinite(views[a].score) && std::isfinite(views[b].score) &&
				views[a].score != views[b].score)
			return views[a].score > views[b].score;
		return views[a].pano_id < views[b].pano_id;
	});
	const std::size_t best = order.front();
	result.best_index = best;
	result.ppm = pixels_per_block;
	std::vector<double> scores;
	scores.reserve(views.size());
	for (const auto &view : views)
		scores.push_back(std::max(view.score, 1e-3));

	if (views.size() == 1) {
		const auto holes = fill_holes(
				views.front().rgb, views.front().valid, pixels_per_block, hole_max_m);
		result.rgb = holes.rgb;
		result.valid = holes.valid;
		result.shifts = {{0.0, 0.0}};
		result.mode = FuseMode::Single;
		result.hole_fraction = holes.fraction;
		result.filled = holes.filled;
		if (holes.fraction > 0.0)
			result.flags.emplace_back("HOLES_FILLED");
		return result;
	}

	const auto aligned = align_views(views, pixels_per_block, max_shift_m, best);
	if (aligned.images.size() != views.size() || aligned.masks.size() != views.size())
		return result;
	std::vector<std::size_t> kept;
	for (std::size_t i = 0; i < aligned.usable.size(); ++i)
		if (aligned.usable[i])
			kept.push_back(i);
	result.shifts = aligned.shifts;
	result.agreement_m = aligned.agreement_m;
	if (kept.size() < 2 || result.agreement_m > agreement_max_m) {
		const auto holes = fill_holes(
				views[best].rgb, views[best].valid, pixels_per_block, hole_max_m);
		result.rgb = holes.rgb;
		result.valid = holes.valid;
		result.mode = FuseMode::BestView;
		result.hole_fraction = holes.fraction;
		result.filled = holes.filled;
		result.flags.emplace_back("VIEWS_DISAGREE");
		if (holes.fraction > 0.0)
			result.flags.emplace_back("HOLES_FILLED");
		return result;
	}

	std::vector<projection::Image> matched;
	std::vector<std::vector<double>> weights;
	matched.reserve(kept.size());
	weights.reserve(kept.size());
	for (const auto i : kept) {
		if (i == best) {
			matched.push_back(aligned.images[i]);
		} else {
			std::vector<bool> overlap(count);
			for (std::size_t k = 0; k < count; ++k)
				overlap[k] = aligned.masks[best][k] && aligned.masks[i][k];
			matched.push_back(
					match_exposure(aligned.images[best], aligned.images[i], overlap));
		}
		const auto border =
				border_weight(aligned.masks[i], width, height, pixels_per_block);
		auto view_weights = border;
		for (auto &weight : view_weights)
			weight *= scores[i];
		weights.push_back(std::move(view_weights));
	}
	const auto median = weighted_median(matched, weights, min_weight_sum);
	const auto holes =
			fill_holes(median.first, median.second, pixels_per_block, hole_max_m);
	result.rgb = holes.rgb;
	result.valid = holes.valid;
	result.mode = FuseMode::Fused;
	result.hole_fraction = holes.fraction;
	result.filled = holes.filled;
	if (holes.fraction > 0.0)
		result.flags.emplace_back("HOLES_FILLED");
	return result;
}
} // namespace arnis::mapillary::fuse
