#pragma once

#include <cstddef>
#include <future>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <utility>
#include <vector>

namespace arnis
{
struct tags_t;
class ProcessedElement;
}

namespace arnis::geographic
{
class LLBBox;
}

namespace arnis::osm_parser
{
struct RawOsmDocument;
}

namespace arnis::overture
{
class OvertureHints;
struct OvertureData;
}

namespace arnis::elevation
{
struct PeakHint
{
	double lat{};
	double lon{};
	double ele_m{};
};

struct TallModel
{
	double lat{};
	double lon{};
	double height_m{};
};

struct TerrainHints
{
	std::vector<PeakHint> peaks;
	std::vector<TallModel> tall_models;
	std::vector<TallModel> tall_structures;
	bool overture_follows{};
};

struct PeakChannel;

class PeakSender
{
public:
	PeakSender() = default;
	PeakSender(const PeakSender &) = delete;
	PeakSender &operator=(const PeakSender &) = delete;
	PeakSender(PeakSender &&) noexcept = default;
	PeakSender &operator=(PeakSender &&) noexcept = default;
	bool send(TerrainHints hints);
	bool send_overture(TerrainHints hints);

private:
	explicit PeakSender(std::shared_ptr<std::promise<TerrainHints>> promise,
			std::shared_ptr<std::promise<TerrainHints>> overture_promise) :
			promise_(std::move(promise)), overture_promise_(std::move(overture_promise))
	{
	}
	std::shared_ptr<std::promise<TerrainHints>> promise_;
	std::shared_ptr<std::promise<TerrainHints>> overture_promise_;
	friend class PeakSource;
	friend struct PeakChannel;
};

class PeakSource
{
public:
	PeakSource() = default;
	static PeakChannel channel();
	static PeakSource none();
	TerrainHints take() const;
	std::vector<TallModel> take_overture() const;

private:
	explicit PeakSource(std::shared_future<TerrainHints> future,
			std::shared_future<TerrainHints> overture_future) :
			future_(std::move(future)), overture_future_(std::move(overture_future))
	{
	}
	std::shared_future<TerrainHints> future_;
	std::shared_future<TerrainHints> overture_future_;
	mutable std::mutex take_mutex_;
	mutable bool taken_{};
	mutable bool overture_allowed_{};
	mutable bool overture_taken_{};
	friend struct PeakChannel;
};

struct PeakChannel
{
	PeakSender sender;
	PeakSource source;
};

std::optional<PeakHint> peak_from_tags(
		double latitude, double longitude, const tags_t &tags);
std::vector<PeakHint> peaks_in(const osm_parser::RawOsmDocument &data);
std::vector<PeakHint> peaks_in(const std::vector<ProcessedElement> &elements);
std::vector<TallModel> tall_models_in(const osm_parser::RawOsmDocument &data);
// Tall OSM buildings and masts that can exceed the vanilla build ceiling.
std::vector<TallModel> tall_structures_in(const osm_parser::RawOsmDocument &data,
		double scale, const overture::OvertureHints *overture_hints = nullptr);
TerrainHints terrain_hints_in(const osm_parser::RawOsmDocument &data);
TerrainHints terrain_hints_in(const osm_parser::RawOsmDocument &data, double scale,
		bool include_models, bool include_structures, bool overture_follows = false);
TerrainHints overture_terrain_hints_in(const osm_parser::RawOsmDocument &data,
		double scale, const ::arnis::overture::OvertureData &overture_data);
bool may_pass_vanilla(double height_m, double scale);

void raise_peaks(std::vector<std::vector<double>> &grid, const geographic::LLBBox &bbox,
		const std::vector<PeakHint> &peaks, double meters_per_cell,
		std::optional<std::size_t> seam_pad = std::nullopt);

std::vector<std::pair<double, double>> model_cells(
		const std::vector<std::vector<double>> &grid, const geographic::LLBBox &bbox,
		const std::vector<TallModel> &models);
}
