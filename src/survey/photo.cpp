#include "photo.h"

#include "fetch.h"
#include "../net.h"
#include "config.h"

#include <stb_image.h>
#include <zstd.h>
#if USE_TIFF
#include <tiffio.h>
#endif

#include <algorithm>
#include <array>
#include <atomic>
#include <cmath>
#include <chrono>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <functional>
#include <iostream>
#include <iterator>
#include <limits>
#include <list>
#include <map>
#include <memory>
#include <string>
#include <thread>
#include <unordered_map>

namespace arnis::survey::photo
{
namespace
{
constexpr std::array<std::uint8_t, 8> MAGIC{'A', 'R', 'N', 'S', 'D', 'O', 'P', '1'};
constexpr std::size_t CELL_BYTES = 8;
constexpr std::size_t CELL_COUNT = TILE * TILE;
constexpr std::size_t HEADER_BYTES = 12;
std::atomic<std::uint64_t> temp_sequence{0};

bool decompress(const std::vector<std::uint8_t> &packed, std::vector<std::uint8_t> &raw)
{
	using Stream = std::unique_ptr<ZSTD_DStream, decltype(&ZSTD_freeDStream)>;
	Stream stream(ZSTD_createDStream(), &ZSTD_freeDStream);
	if (!stream || ZSTD_isError(ZSTD_initDStream(stream.get())))
		return false;
	ZSTD_inBuffer input{packed.data(), packed.size(), 0};
	std::array<std::uint8_t, 32768> output{};
	std::size_t remaining = 1;
	while (input.pos < input.size || remaining != 0) {
		ZSTD_outBuffer out{output.data(), output.size(), 0};
		const auto before = input.pos;
		remaining = ZSTD_decompressStream(stream.get(), &out, &input);
		if (ZSTD_isError(remaining))
			return false;
		constexpr auto MAX_RAW = HEADER_BYTES + TILE * TILE * CELL_BYTES + TILE * TILE;
		if (raw.size() > MAX_RAW || out.pos > MAX_RAW - raw.size())
			return false;
		raw.insert(raw.end(), output.begin(), output.begin() + out.pos);
		if (remaining == 0)
			return input.pos == input.size;
		if (input.pos == before && out.pos == 0)
			return false;
	}
	return false;
}

void put_u32_le(std::vector<std::uint8_t> &out, std::uint32_t value)
{
	for (unsigned shift = 0; shift < 32; shift += 8)
		out.push_back(static_cast<std::uint8_t>(value >> shift));
}

std::uint32_t get_u32_le(const std::uint8_t *p)
{
	return std::uint32_t(p[0]) | (std::uint32_t(p[1]) << 8) |
		   (std::uint32_t(p[2]) << 16) | (std::uint32_t(p[3]) << 24);
}

std::vector<std::size_t> needed_for(std::size_t across,
		const std::vector<bool> &blocks_in, std::size_t c0, std::size_t r0,
		std::size_t c1, std::size_t r1)
{
	std::vector<std::size_t> result;
	if (across == 0 || c0 >= c1 || r0 >= r1)
		return result;
	const auto px0 = c0 * PIXELS_PER_METRE / PIXELS_PER_BLOCK;
	const auto px1 = (c1 * PIXELS_PER_METRE - 1) / PIXELS_PER_BLOCK;
	const auto py0 = r0 * PIXELS_PER_METRE / PIXELS_PER_BLOCK;
	const auto py1 = (r1 * PIXELS_PER_METRE - 1) / PIXELS_PER_BLOCK;
	for (auto by = py0; by <= std::min(py1, across - 1); ++by) {
		for (auto bx = px0; bx <= std::min(px1, across - 1); ++bx) {
			const auto i = by * across + bx;
			if (i >= blocks_in.size() || !blocks_in[i])
				result.push_back(i);
		}
	}
	return result;
}

struct Drawn
{
	std::size_t width = 0, height = 0;
	std::vector<std::uint8_t> rgb;
};

std::optional<Drawn> get_wms_image(
		const std::string &url, std::size_t width, std::size_t height, bool &host_down)
{
	const auto response = fetch::get_bytes(url);
	if (const auto *error = std::get_if<fetch::Failure>(&response)) {
		host_down = fetch::unreachable(error->message);
		return std::nullopt;
	}
	const auto &bytes = std::get<std::vector<std::uint8_t>>(response);
	if (bytes.empty() ||
			bytes.size() > static_cast<std::size_t>(std::numeric_limits<int>::max()))
		return std::nullopt;
	int w = 0, h = 0, channels = 0;
	stbi_uc *decoded = stbi_load_from_memory(
			bytes.data(), static_cast<int>(bytes.size()), &w, &h, &channels, 3);
	if (!decoded)
		return std::nullopt;
	std::unique_ptr<stbi_uc, decltype(&stbi_image_free)> pixels(
			decoded, &stbi_image_free);
	if (w <= 0 || h <= 0 || static_cast<std::size_t>(w) != width ||
			static_cast<std::size_t>(h) != height)
		return std::nullopt;
	Drawn image{width, height, {}};
	image.rgb.assign(decoded, decoded + width * height * 3);
	return image;
}

std::string wms_url(const source::Sheet &sheet, std::int32_t east, std::int32_t north,
		std::size_t px0, std::size_t py0, std::size_t px1, std::size_t py1)
{
	const double west = static_cast<double>(east) * TILE +
						static_cast<double>(px0) / PIXELS_PER_METRE;
	const double top = static_cast<double>(north + 1) * TILE -
					   static_cast<double>(py0) / PIXELS_PER_METRE;
	const double right = static_cast<double>(east) * TILE +
						 static_cast<double>(px1) / PIXELS_PER_METRE;
	const double south = static_cast<double>(north + 1) * TILE -
						 static_cast<double>(py1) / PIXELS_PER_METRE;
	const auto &photo = sheet.source->photo;
	const std::string bbox =
			sheet.projection.north_first()
					? std::to_string(south) + "," + std::to_string(west) + "," +
							  std::to_string(top) + "," + std::to_string(right)
					: std::to_string(west) + "," + std::to_string(south) + "," +
							  std::to_string(right) + "," + std::to_string(top);
	return photo.url + "?SERVICE=WMS&VERSION=1.3.0&REQUEST=GetMap&LAYERS=" + photo.layer +
		   "&STYLES=&CRS=EPSG:" + std::to_string(sheet.projection.epsg()) +
		   "&BBOX=" + bbox + "&WIDTH=" + std::to_string(px1 - px0) +
		   "&HEIGHT=" + std::to_string(py1 - py0) + "&FORMAT=image/jpeg";
}

#if USE_TIFF
struct RemoteTiff
{
	std::string url;
	std::uint64_t size = 0;
	std::uint64_t position = 0;
	std::string error;
	bool missing = false;
	std::atomic_bool *down = nullptr;
};

tmsize_t tiff_read(thandle_t opaque, void *output, tmsize_t requested)
{
	auto &remote = *static_cast<RemoteTiff *>(opaque);
	if (requested <= 0 || remote.position >= remote.size)
		return 0;
	const auto length = std::min<std::uint64_t>(
			static_cast<std::uint64_t>(requested), remote.size - remote.position);
	auto bytes = fetch::get_range(remote.url, remote.position, length);
	if (const auto *failure = std::get_if<fetch::Failure>(&bytes)) {
		remote.error = failure->message;
		remote.missing = failure->kind == fetch::Failure::Kind::Missing;
		if (fetch::unreachable(failure->message) && remote.down)
			remote.down->store(true, std::memory_order_relaxed);
		return -1;
	}
	auto &buffer = std::get<std::vector<std::uint8_t>>(bytes);
	if (buffer.empty())
		return 0;
	std::memcpy(output, buffer.data(), buffer.size());
	remote.position += buffer.size();
	return static_cast<tmsize_t>(buffer.size());
}

toff_t tiff_seek(thandle_t opaque, toff_t offset, int whence)
{
	auto &remote = *static_cast<RemoteTiff *>(opaque);
	std::uint64_t next = 0;
	switch (whence) {
	case SEEK_SET:
		next = offset;
		break;
	case SEEK_CUR:
		if (offset > std::numeric_limits<std::uint64_t>::max() - remote.position)
			return static_cast<toff_t>(-1);
		next = remote.position + offset;
		break;
	case SEEK_END:
		if (offset > remote.size)
			return static_cast<toff_t>(-1);
		next = remote.size - offset;
		break;
	default:
		return static_cast<toff_t>(-1);
	}
	if (next > remote.size)
		return static_cast<toff_t>(-1);
	remote.position = next;
	return static_cast<toff_t>(next);
}

tmsize_t tiff_write(thandle_t, void *, tmsize_t)
{
	return 0;
}

toff_t tiff_size(thandle_t opaque)
{
	return static_cast<toff_t>(static_cast<RemoteTiff *>(opaque)->size);
}

int tiff_close(thandle_t)
{
	return 0;
}

int tiff_map(thandle_t, void **, toff_t *)
{
	return 0;
}

void tiff_unmap(thandle_t, void *, toff_t)
{
}

std::size_t fetch_tiff_blocks(const std::string &url,
		const std::vector<std::size_t> &todo, Tile &tile, std::atomic_bool &down)
{
	const auto size = fetch::content_length(url);
	if (!size || *size == 0)
		return todo.size();
	RemoteTiff remote{url, *size, 0, {}, false, &down};
	TIFF *image = TIFFClientOpen(url.c_str(), "r", &remote, tiff_read, tiff_write,
			tiff_seek, tiff_close, tiff_size, tiff_map, tiff_unmap);
	if (!image)
		return todo.size();
	std::uint32_t width = 0, height = 0, tile_width = 0, tile_height = 0;
	std::uint16_t samples = 0, bits = 0, compression = 0, predictor = 1;
	std::uint16_t planar = PLANARCONFIG_CONTIG, format = SAMPLEFORMAT_UINT;
	std::uint16_t photometric = 0;
	const bool layout_ok = TIFFGetField(image, TIFFTAG_IMAGEWIDTH, &width) &&
						   TIFFGetField(image, TIFFTAG_IMAGELENGTH, &height) &&
						   TIFFGetField(image, TIFFTAG_TILEWIDTH, &tile_width) &&
						   TIFFGetField(image, TIFFTAG_TILELENGTH, &tile_height) &&
						   TIFFGetField(image, TIFFTAG_SAMPLESPERPIXEL, &samples) &&
						   TIFFGetField(image, TIFFTAG_BITSPERSAMPLE, &bits) &&
						   TIFFGetField(image, TIFFTAG_PHOTOMETRIC, &photometric);
	TIFFGetFieldDefaulted(image, TIFFTAG_COMPRESSION, &compression);
	TIFFGetFieldDefaulted(image, TIFFTAG_PREDICTOR, &predictor);
	TIFFGetFieldDefaulted(image, TIFFTAG_PLANARCONFIG, &planar);
	TIFFGetFieldDefaulted(image, TIFFTAG_SAMPLEFORMAT, &format);
	const bool supported_compression = compression == COMPRESSION_NONE ||
									   compression == COMPRESSION_LZW ||
									   compression == COMPRESSION_ADOBE_DEFLATE ||
									   compression == COMPRESSION_DEFLATE;
	if (!layout_ok || width != TILE * PIXELS_PER_METRE ||
			height != TILE * PIXELS_PER_METRE || tile_width != PIXELS_PER_BLOCK ||
			tile_height != PIXELS_PER_BLOCK || samples < 3 || samples > 4 || bits != 8 ||
			planar != PLANARCONFIG_CONTIG || format != SAMPLEFORMAT_UINT ||
			photometric != PHOTOMETRIC_RGB || !supported_compression ||
			(predictor != 1 && predictor != 2)) {
		TIFFClose(image);
		return todo.size();
	}
	const auto tile_size = TIFFTileSize64(image);
	if (tile_size == 0 || tile_size > 256ULL * 256 * 4 + 4096) {
		TIFFClose(image);
		return todo.size();
	}
	std::vector<std::uint8_t> pixels(static_cast<std::size_t>(tile_size));
	std::size_t failed = 0;
	for (const auto block : todo) {
		if (down.load(std::memory_order_relaxed)) {
			failed = todo.size();
			break;
		}
		const auto bx = block % tile.blocks_across, by = block / tile.blocks_across;
		const auto x = static_cast<std::uint32_t>(bx * PIXELS_PER_BLOCK);
		const auto y = static_cast<std::uint32_t>(by * PIXELS_PER_BLOCK);
		const auto tiff_tile = TIFFComputeTile(image, x, y, 0, 0);
		remote.error.clear();
		const auto got = TIFFReadEncodedTile(
				image, tiff_tile, pixels.data(), static_cast<tmsize_t>(pixels.size()));
		if (got < static_cast<tmsize_t>(PIXELS_PER_BLOCK * PIXELS_PER_BLOCK * samples) ||
				!remote.error.empty()) {
			++failed;
			continue;
		}
		fill_cells(tile, block, pixels.data(), samples);
		tile.blocks_in[block] = true;
	}
	TIFFClose(image);
	return failed;
}

struct SheetTiff
{
	enum class State
	{
		Ready,
		Absent,
		Failed
	};

	RemoteTiff remote;
	TIFF *image = nullptr;
	State state = State::Failed;
	std::size_t width = 0, height = 0, tile_width = 0, tile_height = 0;
	std::size_t samples = 0, across = 0, down_blocks = 0;
	std::size_t tile_bytes = 0;

	SheetTiff(const std::string &url, std::atomic_bool &network_down) :
			remote{url, 0, 0, {}, false, &network_down}
	{
		const auto length = fetch::content_length(url);
		if (!length || *length == 0) {
			state = length ? State::Absent : State::Failed;
			return;
		}
		remote.size = *length;
		image = TIFFClientOpen(url.c_str(), "r", &remote, tiff_read, tiff_write,
				tiff_seek, tiff_close, tiff_size, tiff_map, tiff_unmap);
		if (!image) {
			state = remote.missing ? State::Absent : State::Failed;
			return;
		}
		std::uint32_t w = 0, h = 0, tw = 0, th = 0;
		std::uint16_t spp = 0, bps = 0, compression = 0, predictor = 1;
		std::uint16_t planar = PLANARCONFIG_CONTIG, format = SAMPLEFORMAT_UINT;
		std::uint16_t photometric = 0;
		const bool tags = TIFFGetField(image, TIFFTAG_IMAGEWIDTH, &w) &&
						  TIFFGetField(image, TIFFTAG_IMAGELENGTH, &h) &&
						  TIFFGetField(image, TIFFTAG_TILEWIDTH, &tw) &&
						  TIFFGetField(image, TIFFTAG_TILELENGTH, &th) &&
						  TIFFGetField(image, TIFFTAG_SAMPLESPERPIXEL, &spp) &&
						  TIFFGetField(image, TIFFTAG_BITSPERSAMPLE, &bps) &&
						  TIFFGetField(image, TIFFTAG_PHOTOMETRIC, &photometric);
		TIFFGetFieldDefaulted(image, TIFFTAG_COMPRESSION, &compression);
		TIFFGetFieldDefaulted(image, TIFFTAG_PREDICTOR, &predictor);
		TIFFGetFieldDefaulted(image, TIFFTAG_PLANARCONFIG, &planar);
		TIFFGetFieldDefaulted(image, TIFFTAG_SAMPLEFORMAT, &format);
		const bool codec = compression == COMPRESSION_NONE ||
						   compression == COMPRESSION_LZW ||
						   compression == COMPRESSION_ADOBE_DEFLATE ||
						   compression == COMPRESSION_DEFLATE;
		if (!tags || w == 0 || h == 0 || w > (1U << 16) || h > (1U << 16) || tw < 16 ||
				tw > 1024 || th != tw || spp < 3 || spp > 4 || bps != 8 ||
				planar != PLANARCONFIG_CONTIG || format != SAMPLEFORMAT_UINT ||
				photometric != PHOTOMETRIC_RGB || !codec ||
				(predictor != 1 && predictor != 2)) {
			state = State::Failed;
			return;
		}
		width = w;
		height = h;
		tile_width = tw;
		tile_height = th;
		samples = spp;
		across = (width + tile_width - 1) / tile_width;
		down_blocks = (height + tile_height - 1) / tile_height;
		const auto bytes = TIFFTileSize64(image);
		if (!bytes || bytes > 4ULL * 1024 * 1024) {
			state = State::Failed;
			return;
		}
		tile_bytes = static_cast<std::size_t>(bytes);
		state = State::Ready;
	}

	~SheetTiff()
	{
		if (image)
			TIFFClose(image);
	}

	SheetTiff(const SheetTiff &) = delete;
	SheetTiff &operator=(const SheetTiff &) = delete;

	std::optional<std::vector<std::uint8_t>> block(std::size_t index)
	{
		if (state != State::Ready || index >= across * down_blocks)
			return std::nullopt;
		const auto x = static_cast<std::uint32_t>((index % across) * tile_width);
		const auto y = static_cast<std::uint32_t>((index / across) * tile_height);
		std::vector<std::uint8_t> output(tile_bytes);
		remote.error.clear();
		const auto got = TIFFReadEncodedTile(image, TIFFComputeTile(image, x, y, 0, 0),
				output.data(), static_cast<tmsize_t>(output.size()));
		const auto need = tile_width * tile_height * samples;
		if (got < static_cast<tmsize_t>(need) || !remote.error.empty()) {
			if (remote.missing)
				state = State::Absent;
			else
				state = State::Failed;
			return std::nullopt;
		}
		output.resize(need);
		return output;
	}
};

struct SheetBlockKey
{
	std::string url;
	std::size_t index = 0;
	bool operator==(const SheetBlockKey &) const = default;
};

struct SheetBlockHash
{
	std::size_t operator()(const SheetBlockKey &key) const
	{
		return std::hash<std::string>{}(key.url) ^
			   (std::hash<std::size_t>{}(key.index) + 0x9e3779b9 + (key.index << 6) +
					   (key.index >> 2));
	}
};

class SheetBlockCache
{
	struct Entry
	{
		SheetBlockKey key;
		std::vector<std::uint8_t> pixels;
	};
	using List = std::list<Entry>;
	List recent_;
	std::unordered_map<SheetBlockKey, List::iterator, SheetBlockHash> index_;
	std::size_t bytes_ = 0;
	static constexpr std::size_t LIMIT = 48U << 20;

public:
	const std::vector<std::uint8_t> *get(
			const std::string &url, std::size_t block, SheetTiff &reader)
	{
		SheetBlockKey key{url, block};
		if (const auto found = index_.find(key); found != index_.end()) {
			recent_.splice(recent_.end(), recent_, found->second);
			return &found->second->pixels;
		}
		auto pixels = reader.block(block);
		if (!pixels)
			return nullptr;
		bytes_ += pixels->size();
		recent_.push_back({std::move(key), std::move(*pixels)});
		auto last = std::prev(recent_.end());
		index_.emplace(last->key, last);
		while (bytes_ > LIMIT && !recent_.empty()) {
			auto first = recent_.begin();
			bytes_ -= first->pixels.size();
			index_.erase(first->key);
			recent_.erase(first);
		}
		return &last->pixels;
	}
};

std::size_t fetch_sheet_blocks(const source::Sheet &sheet, std::int32_t east,
		std::int32_t north, const std::vector<std::size_t> &todo, Tile &output,
		std::atomic_bool &network_down)
{
	const auto &photo = sheet.source->photo;
	if (!photo.sheet_at || photo.pixels_per_metre == 0)
		return todo.size();
	const double resolution = static_cast<double>(photo.pixels_per_metre);
	const double west = static_cast<double>(east) * TILE;
	const double top = static_cast<double>(north + 1) * TILE;
	std::unordered_map<std::string, std::unique_ptr<SheetTiff>> readers;
	SheetBlockCache cache;
	std::size_t failed = 0;
	for (const auto block : todo) {
		if (network_down.load(std::memory_order_relaxed)) {
			++failed;
			continue;
		}
		std::vector<std::uint8_t> rgb(PIXELS_PER_BLOCK * PIXELS_PER_BLOCK * 3);
		std::vector<bool> absent(PIXELS_PER_BLOCK * PIXELS_PER_BLOCK, false);
		bool lost = false;
		const auto bx = block % BLOCKS_ACROSS, by = block / BLOCKS_ACROSS;
		const auto px0 = bx * PIXELS_PER_BLOCK, py0 = by * PIXELS_PER_BLOCK;
		for (std::size_t y = 0; y < PIXELS_PER_BLOCK && py0 + y < TILE * PIXELS_PER_METRE;
				++y) {
			for (std::size_t x = 0;
					x < PIXELS_PER_BLOCK && px0 + x < TILE * PIXELS_PER_METRE; ++x) {
				const double e = west + (static_cast<double>(px0 + x) + 0.5) /
												static_cast<double>(PIXELS_PER_METRE);
				const double n = top - (static_cast<double>(py0 + y) + 0.5) /
											   static_cast<double>(PIXELS_PER_METRE);
				const auto location = photo.sheet_at(e, n);
				const auto local = y * PIXELS_PER_BLOCK + x;
				if (!location) {
					absent[local] = true;
					continue;
				}
				const auto &[url, sheet_west, sheet_north] = *location;
				auto it = readers.find(url);
				if (it == readers.end()) {
					auto reader = std::make_unique<SheetTiff>(url, network_down);
					it = readers.emplace(url, std::move(reader)).first;
				}
				auto &reader = *it->second;
				if (reader.state == SheetTiff::State::Absent) {
					absent[local] = true;
					continue;
				}
				if (reader.state != SheetTiff::State::Ready) {
					lost = true;
					continue;
				}
				if (e < sheet_west || e >= sheet_west + reader.width / resolution ||
						n > sheet_north ||
						n <= sheet_north - reader.height / resolution) {
					lost = true;
					continue;
				}
				const auto sx =
						std::min(static_cast<std::size_t>((e - sheet_west) * resolution),
								reader.width - 1);
				const auto sy =
						std::min(static_cast<std::size_t>((sheet_north - n) * resolution),
								reader.height - 1);
				const auto source_block = (sy / reader.tile_height) * reader.across +
										  sx / reader.tile_width;
				const auto *pixels = cache.get(url, source_block, reader);
				if (!pixels) {
					if (reader.state == SheetTiff::State::Absent)
						absent[local] = true;
					else
						lost = true;
					continue;
				}
				const auto offset = ((sy % reader.tile_height) * reader.tile_width +
											sx % reader.tile_width) *
									reader.samples;
				const auto target = local * 3;
				std::copy_n(pixels->data() + offset, 3, rgb.data() + target);
			}
		}
		if (lost) {
			++failed;
			continue;
		}
		fill_cells(output, block, rgb.data(), 3, &absent);
		output.blocks_in[block] = true;
	}
	return failed;
}
#endif
} // namespace

Tile::Tile(std::size_t across) :
		cells(CELL_COUNT), blocks_in(across * across, false), blocks_across(across)
{
}

bool Tile::valid() const
{
	return cells.size() == CELL_COUNT && blocks_across > 0 &&
		   blocks_in.size() == blocks_across * blocks_across;
}

std::filesystem::path cache_path(const std::filesystem::path &root,
		const source::Sheet &sheet, std::int32_t east, std::int32_t north)
{
	return root / sheet.cache_dir() /
		   ("photo_" + std::to_string(east) + "_" + std::to_string(north) + ".bin");
}

std::optional<Tile> read_cache(const std::filesystem::path &path)
{
	std::ifstream file(path, std::ios::binary);
	if (!file)
		return std::nullopt;
	std::vector<std::uint8_t> packed(
			(std::istreambuf_iterator<char>(file)), std::istreambuf_iterator<char>());
	if (packed.empty())
		return std::nullopt;
	std::vector<std::uint8_t> raw;
	if (!decompress(packed, raw) || raw.size() < HEADER_BYTES ||
			!std::equal(MAGIC.begin(), MAGIC.end(), raw.begin()))
		return std::nullopt;
	const auto across = static_cast<std::size_t>(get_u32_le(raw.data() + MAGIC.size()));
	if (across > TILE)
		return std::nullopt;
	const auto blocks = across * across;
	const auto expected = HEADER_BYTES + blocks + CELL_COUNT * CELL_BYTES;
	if (raw.size() != expected)
		return std::nullopt;
	Tile tile(across);
	for (std::size_t i = 0; i < blocks; ++i)
		tile.blocks_in[i] = raw[HEADER_BYTES + i] != 0;
	auto p = raw.data() + HEADER_BYTES + blocks;
	for (auto &cell : tile.cells) {
		cell.exg = static_cast<std::int8_t>(p[0]);
		cell.green = p[1];
		cell.lum = p[2];
		cell.blue = static_cast<std::int8_t>(p[3]);
		cell.rgb = {p[4], p[5], p[6]};
		cell.present = p[7] != 0;
		p += CELL_BYTES;
	}
	return tile;
}

bool write_cache(const std::filesystem::path &path, const Tile &tile)
{
	if (!tile.valid() || tile.blocks_across > TILE ||
			tile.blocks_across > std::numeric_limits<std::uint32_t>::max())
		return false;
	std::vector<std::uint8_t> raw;
	raw.reserve(HEADER_BYTES + tile.blocks_in.size() + CELL_COUNT * CELL_BYTES);
	raw.insert(raw.end(), MAGIC.begin(), MAGIC.end());
	put_u32_le(raw, static_cast<std::uint32_t>(tile.blocks_across));
	for (bool present : tile.blocks_in)
		raw.push_back(present ? 1 : 0);
	for (const auto &cell : tile.cells) {
		raw.push_back(static_cast<std::uint8_t>(cell.exg));
		raw.push_back(cell.green);
		raw.push_back(cell.lum);
		raw.push_back(static_cast<std::uint8_t>(cell.blue));
		raw.insert(raw.end(), cell.rgb.begin(), cell.rgb.end());
		raw.push_back(cell.present ? 1 : 0);
	}
	std::vector<std::uint8_t> packed(ZSTD_compressBound(raw.size()));
	const auto packed_size =
			ZSTD_compress(packed.data(), packed.size(), raw.data(), raw.size(), 9);
	if (ZSTD_isError(packed_size))
		return false;
	packed.resize(packed_size);
	std::error_code ec;
	std::filesystem::create_directories(path.parent_path(), ec);
	if (ec)
		return false;
	const auto suffix =
			".tmp-" +
			std::to_string(std::hash<std::thread::id>{}(std::this_thread::get_id())) +
			"-" + std::to_string(temp_sequence.fetch_add(1, std::memory_order_relaxed));
	auto temporary = path;
	temporary += suffix;
	{
		std::ofstream file(temporary, std::ios::binary | std::ios::trunc);
		if (!file || !file.write(reinterpret_cast<const char *>(packed.data()),
							 static_cast<std::streamsize>(packed.size()))) {
			std::filesystem::remove(temporary, ec);
			return false;
		}
		file.flush();
		if (!file) {
			std::filesystem::remove(temporary, ec);
			return false;
		}
	}
	std::filesystem::rename(temporary, path, ec);
	if (!ec)
		return true;
	std::error_code ignored;
	std::filesystem::remove(temporary, ignored);
	return false;
}

std::vector<std::size_t> needed_blocks(
		const Tile &tile, std::size_t c0, std::size_t r0, std::size_t c1, std::size_t r1)
{
	if (!tile.valid())
		return {};
	return needed_for(tile.blocks_across, tile.blocks_in, c0, r0, c1, r1);
}

void fill_cells(Tile &tile, std::size_t block_index, const std::uint8_t *pixels,
		std::size_t pixel_stride, const std::vector<bool> *absent)
{
	if (!tile.valid() || !pixels || pixel_stride < 3 ||
			block_index >= tile.blocks_in.size() ||
			(absent && absent->size() < PIXELS_PER_BLOCK * PIXELS_PER_BLOCK))
		return;
	const auto bx = block_index % tile.blocks_across;
	const auto by = block_index / tile.blocks_across;
	const auto x0 = bx * PIXELS_PER_BLOCK;
	const auto y0 = by * PIXELS_PER_BLOCK;
	const auto x1 = x0 + PIXELS_PER_BLOCK;
	const auto y1 = y0 + PIXELS_PER_BLOCK;
	const auto span = [](std::size_t cell, std::size_t lo, std::size_t hi) {
		return std::pair{std::max(cell * PIXELS_PER_METRE, lo),
				std::min(cell * PIXELS_PER_METRE + PIXELS_PER_METRE, hi)};
	};
	const auto rows_end = std::min((y1 + PIXELS_PER_METRE - 1) / PIXELS_PER_METRE, TILE);
	const auto cols_end = std::min((x1 + PIXELS_PER_METRE - 1) / PIXELS_PER_METRE, TILE);
	for (std::size_t r = y0 / PIXELS_PER_METRE; r < rows_end; ++r) {
		const auto [py0, py1] = span(r, y0, y1);
		if (py1 <= py0 || py1 - py0 < 3)
			continue;
		for (std::size_t c = x0 / PIXELS_PER_METRE; c < cols_end; ++c) {
			const auto [px0, px1] = span(c, x0, x1);
			if (px1 <= px0 || px1 - px0 < 3)
				continue;
			bool missing = false;
			if (absent) {
				for (auto py = py0; py < py1 && !missing; ++py) {
					for (auto px = px0; px < px1; ++px) {
						if ((*absent)[(py - y0) * PIXELS_PER_BLOCK + px - x0]) {
							missing = true;
							break;
						}
					}
				}
			}
			auto &cell = tile.cells[r * TILE + c];
			if (missing) {
				cell = {};
				continue;
			}
			float exg = 0.0f, lum = 0.0f, blue = 0.0f;
			std::uint32_t plant = 0, n = 0;
			std::array<float, 3> rgb{};
			for (auto py = py0; py < py1; ++py) {
				for (auto px = px0; px < px1; ++px) {
					const auto offset =
							((py - y0) * PIXELS_PER_BLOCK + px - x0) * pixel_stride;
					const float red = pixels[offset];
					const float green = pixels[offset + 1];
					const float blue_pixel = pixels[offset + 2];
					const float total = red + green + blue_pixel + 1e-3f;
					const float green_index = (2.0f * green - red - blue_pixel) / total;
					exg += green_index;
					plant += green_index > 0.06f;
					lum += total / 3.0f;
					blue += (blue_pixel - std::max(red, green)) / 255.0f;
					rgb[0] += red;
					rgb[1] += green;
					rgb[2] += blue_pixel;
					++n;
				}
			}
			const float count = static_cast<float>(n);
			const auto quantize = [](float value) {
				return static_cast<std::uint8_t>(
						std::clamp(std::lround(value), 0L, 255L));
			};
			cell.exg = static_cast<std::int8_t>(
					std::clamp(std::lround(exg / count * 200.0f), -127L, 127L));
			cell.green = quantize(static_cast<float>(plant) / count * 25.0f);
			cell.lum = quantize(lum / count);
			cell.blue = static_cast<std::int8_t>(
					std::clamp(std::lround(blue / count * 200.0f), -127L, 127L));
			for (std::size_t channel = 0; channel < cell.rgb.size(); ++channel)
				cell.rgb[channel] = quantize(rgb[channel] / count);
			cell.present = true;
		}
	}
}

std::optional<Tile> tile(const std::filesystem::path &root, const source::Sheet &sheet,
		std::int32_t east, std::int32_t north, std::size_t c0, std::size_t r0,
		std::size_t c1, std::size_t r1, std::atomic_bool &down)
{
	if (!sheet.source || sheet.source->photo.kind == source::Photo::Kind::None)
		return std::nullopt;
	const auto path = cache_path(root, sheet, east, north);
	auto cached = read_cache(path);
	if (sheet.source->photo.kind == source::Photo::Kind::TiledTiff) {
		const bool compatible = cached && cached->blocks_across == BLOCKS_ACROSS;
		const std::vector<bool> empty_blocks;
		const auto todo = needed_for(BLOCKS_ACROSS,
				compatible ? cached->blocks_in : empty_blocks, c0, r0, c1, r1);
		if (todo.empty())
			return cached;
#if USE_TIFF
		if (down.load(std::memory_order_relaxed))
			return cached;
		Tile current = compatible ? std::move(*cached) : Tile(BLOCKS_ACROSS);
		[[maybe_unused]] fetch::PhotoRequestScope photo_requests;
		const auto url = sheet.source->photo.tile_url
								 ? sheet.source->photo.tile_url(east, north)
								 : std::string{};
		const auto failed =
				url.empty() ? todo.size() : fetch_tiff_blocks(url, todo, current, down);
		if (failed) {
			std::cerr << "Warning: " << failed << " aerial photo blocks of " << east
					  << '_' << north << " failed\n";
			net::note_uncached("aerial photos");
		}
		if (!write_cache(path, current))
			std::cerr << "Warning: aerial photo " << east << '_' << north
					  << " not cached\n";
		return current;
#else
		return cached;
#endif
	}
	if (sheet.source->photo.kind == source::Photo::Kind::Sheets) {
		const bool compatible = cached && cached->blocks_across == BLOCKS_ACROSS;
		const std::vector<bool> empty_blocks;
		const auto todo = needed_for(BLOCKS_ACROSS,
				compatible ? cached->blocks_in : empty_blocks, c0, r0, c1, r1);
		if (todo.empty())
			return cached;
#if USE_TIFF
		if (down.load(std::memory_order_relaxed))
			return cached;
		Tile current = compatible ? std::move(*cached) : Tile(BLOCKS_ACROSS);
		[[maybe_unused]] fetch::PhotoRequestScope photo_requests;
		const auto failed = fetch_sheet_blocks(sheet, east, north, todo, current, down);
		if (failed) {
			std::cerr << "Warning: " << failed << " aerial photo blocks of " << east
					  << '_' << north << " failed\n";
			net::note_uncached("aerial photos");
		}
		if (!write_cache(path, current))
			std::cerr << "Warning: aerial photo " << east << '_' << north
					  << " not cached\n";
		return current;
#else
		return cached;
#endif
	}
	if (sheet.source->photo.kind != source::Photo::Kind::Wms)
		return cached;
	const bool compatible = cached && cached->blocks_across == BLOCKS_ACROSS;
	const std::vector<bool> empty_blocks;
	const auto todo = needed_for(
			BLOCKS_ACROSS, compatible ? cached->blocks_in : empty_blocks, c0, r0, c1, r1);
	if (todo.empty())
		return cached;
	if (down.load(std::memory_order_relaxed))
		return cached;
	Tile current = compatible ? std::move(*cached) : Tile(BLOCKS_ACROSS);
	[[maybe_unused]] fetch::PhotoRequestScope photo_requests;
	std::map<std::pair<std::size_t, std::size_t>, std::vector<std::size_t>> chunks;
	for (const auto i : todo) {
		const auto bx = i % BLOCKS_ACROSS, by = i / BLOCKS_ACROSS;
		chunks[{by / 8, bx / 8}].push_back(i);
	}
	std::size_t failed = 0;
	constexpr std::size_t tile_pixels = TILE * PIXELS_PER_METRE;
	for (const auto &[chunk, blocks] : chunks) {
		(void)chunk;
		std::size_t bx0 = BLOCKS_ACROSS, by0 = BLOCKS_ACROSS, bx1 = 0, by1 = 0;
		for (const auto i : blocks) {
			const auto bx = i % BLOCKS_ACROSS, by = i / BLOCKS_ACROSS;
			bx0 = std::min(bx0, bx);
			bx1 = std::max(bx1, bx + 1);
			by0 = std::min(by0, by);
			by1 = std::max(by1, by + 1);
		}
		const auto px0 = bx0 * PIXELS_PER_BLOCK;
		const auto px1 = std::min(bx1 * PIXELS_PER_BLOCK, tile_pixels);
		const auto py0 = by0 * PIXELS_PER_BLOCK;
		const auto py1 = std::min(by1 * PIXELS_PER_BLOCK, tile_pixels);
		std::optional<Drawn> image;
		for (unsigned attempt = 0; attempt < 3 && !image; ++attempt) {
			if (down.load(std::memory_order_relaxed))
				break;
			if (attempt)
				std::this_thread::sleep_for(std::chrono::milliseconds(800 * attempt));
			bool host_down = false;
			image = get_wms_image(wms_url(sheet, east, north, px0, py0, px1, py1),
					px1 - px0, py1 - py0, host_down);
			if (host_down) {
				down.store(true, std::memory_order_relaxed);
				break;
			}
		}
		if (!image) {
			failed += blocks.size();
			continue;
		}
		for (const auto i : blocks) {
			const auto bx = i % BLOCKS_ACROSS, by = i / BLOCKS_ACROSS;
			std::vector<std::uint8_t> padded(PIXELS_PER_BLOCK * PIXELS_PER_BLOCK * 3);
			const auto block_x = bx * PIXELS_PER_BLOCK;
			const auto block_y = by * PIXELS_PER_BLOCK;
			const auto copy_w = std::min(PIXELS_PER_BLOCK, tile_pixels - block_x);
			const auto copy_h = std::min(PIXELS_PER_BLOCK, tile_pixels - block_y);
			for (std::size_t y = 0; y < copy_h; ++y) {
				const auto source_y = block_y + y - py0;
				const auto source_x = block_x - px0;
				const auto *src =
						image->rgb.data() + (source_y * image->width + source_x) * 3;
				auto *dst = padded.data() + y * PIXELS_PER_BLOCK * 3;
				std::copy_n(src, copy_w * 3, dst);
			}
			fill_cells(current, i, padded.data(), 3);
			current.blocks_in[i] = true;
		}
	}
	if (failed) {
		std::cerr << "Warning: " << failed << " aerial photo blocks of " << east << '_'
				  << north << " failed\n";
		net::note_uncached("aerial photos");
	}
	if (!write_cache(path, current))
		std::cerr << "Warning: aerial photo " << east << '_' << north << " not cached\n";
	return current;
}
} // namespace arnis::survey::photo
