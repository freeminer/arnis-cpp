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

private:
	explicit PeakSender(std::shared_ptr<std::promise<TerrainHints>> promise) :
			promise_(std::move(promise))
	{
	}
	std::shared_ptr<std::promise<TerrainHints>> promise_;
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

private:
	explicit PeakSource(std::shared_future<TerrainHints> future) :
			future_(std::move(future))
	{
	}
	std::shared_future<TerrainHints> future_;
	mutable std::mutex take_mutex_;
	mutable bool taken_{};
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
TerrainHints terrain_hints_in(const osm_parser::RawOsmDocument &data);

void raise_peaks(std::vector<std::vector<double>> &grid, const geographic::LLBBox &bbox,
		const std::vector<PeakHint> &peaks, double meters_per_cell);

std::vector<std::pair<double, double>> model_cells(
		const std::vector<std::vector<double>> &grid, const geographic::LLBBox &bbox,
		const std::vector<TallModel> &models);
}
