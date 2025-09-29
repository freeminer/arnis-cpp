#if 0

#if 0
#define STB_IMAGE_IMPLEMENTATION
#include "stb_image.h"
#endif

#include <cmath>
#include <cstdint>
#include <curl/curl.h>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <limits>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <tuple>
#include <utility>
#include <vector>
#include <algorithm>

namespace crate {
namespace coordinate_system {
namespace geographic {

class Coordinate {
public:
    Coordinate() : lat_(0.0), lng_(0.0) {}
    Coordinate(double lat, double lng) : lat_(lat), lng_(lng) {}
    double lat() const { return lat_; }
    double lng() const { return lng_; }
private:
    double lat_;
    double lng_;
};

class LLBBox {
public:
    LLBBox() = default;
    LLBBox(const Coordinate& a, const Coordinate& b) : a_(a), b_(b) {}
    Coordinate min() const {
        double minlat = std::min(a_.lat(), b_.lat());
        double minlng = std::min(a_.lng(), b_.lng());
        return Coordinate(minlat, minlng);
    }
    Coordinate max() const {
        double maxlat = std::max(a_.lat(), b_.lat());
        double maxlng = std::max(a_.lng(), b_.lng());
        return Coordinate(maxlat, maxlng);
    }
private:
    Coordinate a_;
    Coordinate b_;
};

} // namespace geographic

namespace transformation {

// Returns approximate distances in degrees? To mimic original signature returning two doubles (z,x).
// For compatibility we compute rough meter estimates for lat and lng edges and return as pair (z,x).
inline std::pair<double, double> geo_distance(const geographic::Coordinate& a, const geographic::Coordinate& b) {
    constexpr double R = 6371000.0; // Earth radius meters
    double lat1 = a.lat() * M_PI / 180.0;
    double lat2 = b.lat() * M_PI / 180.0;
    double dlat = lat2 - lat1;
    double dlng = (b.lng() - a.lng()) * M_PI / 180.0;
    double hav = std::sin(dlat/2.0)*std::sin(dlat/2.0) + std::cos(lat1)*std::cos(lat2)*std::sin(dlng/2.0)*std::sin(dlng/2.0);
    double distance = 2.0 * R * std::asin(std::sqrt(hav));
    // Return (z-distance, x-distance) in meters approximated via lat and lng separate components:
    double avg_lat = (lat1 + lat2) / 2.0;
    double lat_m = std::abs((b.lat() - a.lat()) * M_PI / 180.0) * R;
    double lng_m = std::abs((b.lng() - a.lng()) * M_PI / 180.0) * R * std::cos(avg_lat);
    return std::make_pair(lat_m, lng_m);
}

} // namespace transformation
} // namespace coordinate_system
} // namespace crate

// Constants
constexpr int32_t MAX_Y = 319;
constexpr double BASE_HEIGHT_SCALE = 0.7;
const std::string AWS_TERRARIUM_URL = "https://s3.amazonaws.com/elevation-tiles-prod/terrarium/{z}/{x}/{y}.png";
constexpr double TERRARIUM_OFFSET = 32768.0;
constexpr uint8_t MIN_ZOOM = 10;
constexpr uint8_t MAX_ZOOM = 15;

// ElevationData struct
struct ElevationData {
    std::vector<std::vector<int32_t>> heights;
    std::size_t width;
    std::size_t height;
};

// CURL write callback
size_t curl_write_callback(void* contents, size_t size, size_t nmemb, void* userp) {
    size_t total = size * nmemb;
    std::vector<uint8_t>* buffer = reinterpret_cast<std::vector<uint8_t>*>(userp);
    buffer->insert(buffer->end(), reinterpret_cast<uint8_t*>(contents), reinterpret_cast<uint8_t*>(contents) + total);
    return total;
}

uint8_t calculate_zoom_level(const crate::coordinate_system::geographic::LLBBox& bbox) {
    double lat_diff = std::abs(bbox.max().lat() - bbox.min().lat());
    double lng_diff = std::abs(bbox.max().lng() - bbox.min().lng());
    double max_diff = std::max(lat_diff, lng_diff);
    double raw = -std::log2(max_diff) + 20.0;
    int z = static_cast<int>(std::floor(raw));
    if (z < MIN_ZOOM) z = MIN_ZOOM;
    if (z > MAX_ZOOM) z = MAX_ZOOM;
    return static_cast<uint8_t>(z);
}

std::pair<uint32_t, uint32_t> lat_lng_to_tile(double lat, double lng, uint8_t zoom) {
    double lat_rad = lat * M_PI / 180.0;
    double n = std::pow(2.0, static_cast<int>(zoom));
    uint32_t x = static_cast<uint32_t>(std::floor((lng + 180.0) / 360.0 * n));
    uint32_t y = static_cast<uint32_t>(std::floor((1.0 - std::asinh(std::tan(lat_rad)) / M_PI) / 2.0 * n));
    return std::make_pair(x, y);
}

std::vector<std::pair<uint32_t, uint32_t>> get_tile_coordinates(const crate::coordinate_system::geographic::LLBBox& bbox, uint8_t zoom) {
    auto p1 = lat_lng_to_tile(bbox.min().lat(), bbox.min().lng(), zoom);
    auto p2 = lat_lng_to_tile(bbox.max().lat(), bbox.max().lng(), zoom);
    uint32_t x1 = p1.first;
    uint32_t y1 = p1.second;
    uint32_t x2 = p2.first;
    uint32_t y2 = p2.second;
    std::vector<std::pair<uint32_t, uint32_t>> tiles;
    uint32_t xmin = std::min(x1, x2);
    uint32_t xmax = std::max(x1, x2);
    uint32_t ymin = std::min(y1, y2);
    uint32_t ymax = std::max(y1, y2);
    for (uint32_t x = xmin; x <= xmax; ++x) {
        for (uint32_t y = ymin; y <= ymax; ++y) {
            tiles.emplace_back(x, y);
        }
    }
    return tiles;
}

// Download tile and decode PNG into RGB vector (row-major)
std::optional<std::tuple<std::vector<uint8_t>, int, int>> download_tile_and_decode(uint32_t tile_x, uint32_t tile_y, uint8_t zoom, const std::filesystem::path& tile_path) {
    std::string url = AWS_TERRARIUM_URL;
    // replace placeholders
    {
        std::ostringstream ss;
        ss << static_cast<int>(zoom);
        std::string s = ss.str();
        size_t pos = url.find("{z}");
        if (pos != std::string::npos) url.replace(pos, 3, s);
    }
    {
        std::ostringstream ss;
        ss << tile_x;
        std::string s = ss.str();
        size_t pos = url.find("{x}");
        if (pos != std::string::npos) url.replace(pos, 3, s);
    }
    {
        std::ostringstream ss;
        ss << tile_y;
        std::string s = ss.str();
        size_t pos = url.find("{y}");
        if (pos != std::string::npos) url.replace(pos, 3, s);
    }

    CURL* curl = curl_easy_init();
    if (!curl) {
        return std::nullopt;
    }
    std::vector<uint8_t> buffer;
    curl_easy_setopt(curl, CURLOPT_URL, url.c_str());
    curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, curl_write_callback);
    curl_easy_setopt(curl, CURLOPT_WRITEDATA, &buffer);
    curl_easy_setopt(curl, CURLOPT_FOLLOWLOCATION, 1L);
    CURLcode res = curl_easy_perform(curl);
    long http_code = 0;
    curl_easy_getinfo(curl, CURLINFO_RESPONSE_CODE, &http_code);
    curl_easy_cleanup(curl);
    if (res != CURLE_OK || http_code < 200 || http_code >= 300) {
        return std::nullopt;
    }

    // write to cache file
    try {
        std::ofstream ofs(tile_path, std::ios::binary);
        ofs.write(reinterpret_cast<const char*>(buffer.data()), static_cast<std::streamsize>(buffer.size()));
    } catch (...) {
        // ignore write failures but continue with in-memory buffer
    }

    int width = 0;
    int height = 0;
    int channels = 0;
    unsigned char* img = stbi_load_from_memory(buffer.data(), static_cast<int>(buffer.size()), &width, &height, &channels, 3);
    if (!img) {
        return std::nullopt;
    }
    std::vector<uint8_t> pixels;
    pixels.assign(img, img + (width * height * 3));
    stbi_image_free(img);
    return std::make_tuple(std::move(pixels), width, height);
}

std::vector<double> create_gaussian_kernel(std::size_t size, double sigma) {
    std::vector<double> kernel(size, 0.0);
    double center = static_cast<double>(size) / 2.0;
    for (std::size_t i = 0; i < size; ++i) {
        double x = static_cast<double>(i) - center;
        kernel[i] = std::exp(-(x * x) / (2.0 * sigma * sigma));
    }
    double sum = 0.0;
    for (double v : kernel) sum += v;
    if (sum != 0.0) {
        for (double& v : kernel) v /= sum;
    }
    return kernel;
}

std::vector<std::vector<double>> apply_gaussian_blur(const std::vector<std::vector<double>>& heights, double sigma) {
    std::size_t kernel_size = static_cast<std::size_t>(std::ceil(sigma * 3.0)) * 2 + 1;
    std::vector<double> kernel = create_gaussian_kernel(kernel_size, sigma);
    std::vector<std::vector<double>> blurred = heights;
    std::size_t h = blurred.size();
    if (h == 0) return blurred;
    std::size_t w = blurred[0].size();

    // horizontal pass
    for (std::size_t y = 0; y < h; ++y) {
        std::vector<double> temp = blurred[y];
        for (std::size_t i = 0; i < w; ++i) {
            double sum = 0.0;
            double weight = 0.0;
            for (std::size_t j = 0; j < kernel.size(); ++j) {
                int idx = static_cast<int>(i) + static_cast<int>(j) - static_cast<int>(kernel_size / 2);
                if (idx >= 0 && idx < static_cast<int>(w)) {
                    double val = blurred[y][static_cast<std::size_t>(idx)];
                    sum += val * kernel[j];
                    weight += kernel[j];
                }
            }
            temp[i] = (weight == 0.0) ? 0.0 : sum / weight;
        }
        blurred[y] = std::move(temp);
    }

    // vertical pass
    for (std::size_t x = 0; x < w; ++x) {
        std::vector<double> column(h);
        for (std::size_t y = 0; y < h; ++y) column[y] = blurred[y][x];
        for (std::size_t y = 0; y < h; ++y) {
            double sum = 0.0;
            double weight = 0.0;
            for (std::size_t j = 0; j < kernel.size(); ++j) {
                int idx = static_cast<int>(y) + static_cast<int>(j) - static_cast<int>(kernel_size / 2);
                if (idx >= 0 && idx < static_cast<int>(h)) {
                    sum += column[static_cast<std::size_t>(idx)] * kernel[j];
                    weight += kernel[j];
                }
            }
            blurred[y][x] = (weight == 0.0) ? 0.0 : sum / weight;
        }
    }

    return blurred;
}

void fill_nan_values(std::vector<std::vector<double>>& grid) {
    std::size_t height = grid.size();
    if (height == 0) return;
    std::size_t width = grid[0].size();
    bool changes = true;
    while (changes) {
        changes = false;
        for (std::size_t y = 0; y < height; ++y) {
            for (std::size_t x = 0; x < width; ++x) {
                if (std::isnan(grid[y][x])) {
                    double sum = 0.0;
                    int count = 0;
                    for (int dy = -1; dy <= 1; ++dy) {
                        for (int dx = -1; dx <= 1; ++dx) {
                            int ny = static_cast<int>(y) + dy;
                            int nx = static_cast<int>(x) + dx;
                            if (ny >= 0 && ny < static_cast<int>(height) && nx >= 0 && nx < static_cast<int>(width)) {
                                double v = grid[ny][nx];
                                if (!std::isnan(v)) {
                                    sum += v;
                                    ++count;
                                }
                            }
                        }
                    }
                    if (count > 0) {
                        grid[y][x] = sum / static_cast<double>(count);
                        changes = true;
                    }
                }
            }
        }
    }
}

void filter_elevation_outliers(std::vector<std::vector<double>>& grid) {
    std::vector<double> all;
    for (auto const& row : grid) {
        for (double v : row) {
            if (!std::isnan(v) && std::isfinite(v)) all.push_back(v);
        }
    }
    if (all.empty()) return;
    std::sort(all.begin(), all.end());
    std::size_t len = all.size();
    std::size_t p1_idx = static_cast<std::size_t>(len * 0.01);
    std::size_t p99_idx = static_cast<std::size_t>(len * 0.99);
    if (p1_idx >= len) p1_idx = 0;
    if (p99_idx >= len) p99_idx = len - 1;
    double min_reasonable = all[p1_idx];
    double max_reasonable = all[p99_idx];
    std::size_t filtered = 0;
    for (auto& row : grid) {
        for (double& v : row) {
            if (!std::isnan(v) && (v < min_reasonable || v > max_reasonable)) {
                v = std::numeric_limits<double>::quiet_NaN();
                ++filtered;
            }
        }
    }
    if (filtered > 0) {
        fill_nan_values(grid);
    }
}

std::optional<ElevationData> fetch_elevation_data(const crate::coordinate_system::geographic::LLBBox& bbox, double scale, int32_t ground_level) {
    using crate::coordinate_system::transformation::geo_distance;
    auto [base_scale_z, base_scale_x] = geo_distance(bbox.min(), bbox.max());
    double scale_factor_z = std::floor(base_scale_z) * scale;
    double scale_factor_x = std::floor(base_scale_x) * scale;
    uint8_t zoom = calculate_zoom_level(bbox);
    std::vector<std::pair<uint32_t, uint32_t>> tiles = get_tile_coordinates(bbox, zoom);

    std::size_t grid_width = static_cast<std::size_t>(std::max(1.0, scale_factor_x));
    std::size_t grid_height = static_cast<std::size_t>(std::max(1.0, scale_factor_z));

    std::vector<std::vector<double>> height_grid(grid_height, std::vector<double>(grid_width, std::numeric_limits<double>::quiet_NaN()));
    std::vector<std::tuple<uint32_t, uint32_t, int, int, uint8_t, uint8_t, uint8_t, double>> extreme_values_found;

    std::filesystem::path tile_cache_dir = std::filesystem::path("./arnis-tile-cache");
    if (!std::filesystem::exists(tile_cache_dir)) {
        std::error_code ec;
        std::filesystem::create_directories(tile_cache_dir, ec);
    }

    for (auto const& tile : tiles) {
        uint32_t tile_x = tile.first;
        uint32_t tile_y = tile.second;
        std::ostringstream fname;
        fname << "z" << static_cast<int>(zoom) << "_x" << tile_x << "_y" << tile_y << ".png";
        std::filesystem::path tile_path = tile_cache_dir / fname.str();

        std::vector<uint8_t> pixels;
        int width = 0;
        int height = 0;
        bool ok = false;

        if (std::filesystem::exists(tile_path)) {
            std::error_code ec;
            auto file_size = std::filesystem::file_size(tile_path, ec);
            if (!ec && file_size >= 1000) {
                // try to load from disk
                std::ifstream ifs(tile_path, std::ios::binary);
                std::vector<uint8_t> data((std::istreambuf_iterator<char>(ifs)), std::istreambuf_iterator<char>());
                int channels = 0;
                unsigned char* img = stbi_load_from_memory(data.data(), static_cast<int>(data.size()), &width, &height, &channels, 3);
                if (img) {
                    pixels.assign(img, img + (width * height * 3));
                    stbi_image_free(img);
                    ok = true;
                } else {
                    // fallthrough to re-download
                    std::filesystem::remove(tile_path, ec);
                }
            } else {
                std::filesystem::remove(tile_path, std::error_code{});
            }
        }

        if (!ok) {
            auto decoded = download_tile_and_decode(tile_x, tile_y, zoom, tile_path);
            if (!decoded.has_value()) {
                continue;
            }
            auto tuple = decoded.value();
            pixels = std::move(std::get<0>(tuple));
            width = std::get<1>(tuple);
            height = std::get<2>(tuple);
        }

        if (width <= 0 || height <= 0) continue;

        for (int py = 0; py < height; ++py) {
            for (int px = 0; px < width; ++px) {
                int idx = (py * width + px) * 3;
                uint8_t r = pixels[idx + 0];
                uint8_t g = pixels[idx + 1];
                uint8_t b = pixels[idx + 2];

                double pixel_lng = ((static_cast<double>(tile_x) + static_cast<double>(px) / static_cast<double>(width)) / std::pow(2.0, static_cast<int>(zoom))) * 360.0 - 180.0;
                double pixel_lat_rad = M_PI * (1.0 - 2.0 * (static_cast<double>(tile_y) + static_cast<double>(py) / static_cast<double>(height)) / std::pow(2.0, static_cast<int>(zoom)));
                double pixel_lat = std::atan(std::sinh(pixel_lat_rad)) * 180.0 / M_PI;

                if (pixel_lat < bbox.min().lat() || pixel_lat > bbox.max().lat() || pixel_lng < bbox.min().lng() || pixel_lng > bbox.max().lng()) {
                    continue;
                }

                double rel_x = (pixel_lng - bbox.min().lng()) / (bbox.max().lng() - bbox.min().lng());
                double rel_y = 1.0 - (pixel_lat - bbox.min().lat()) / (bbox.max().lat() - bbox.min().lat());

                std::size_t scaled_x = static_cast<std::size_t>(std::round(rel_x * static_cast<double>(grid_width)));
                std::size_t scaled_y = static_cast<std::size_t>(std::round(rel_y * static_cast<double>(grid_height)));

                if (scaled_y >= grid_height || scaled_x >= grid_width) {
                    continue;
                }

                double height_m = (static_cast<double>(r) * 256.0 + static_cast<double>(g) + static_cast<double>(b) / 256.0) - TERRARIUM_OFFSET;

                if (!(height_m >= -1000.0 && height_m <= 10000.0)) {
                    if (extreme_values_found.size() <= 5) {
                        std::cerr << "Extreme value found: tile(" << tile_x << "," << tile_y << ") pixel(" << px << "," << py << ") RGB(" << static_cast<int>(r) << "," << static_cast<int>(g) << "," << static_cast<int>(b) << ") = " << height_m << "m\n";
                    }
                    extreme_values_found.emplace_back(tile_x, tile_y, px, py, r, g, b, height_m);
                }

                height_grid[scaled_y][scaled_x] = height_m;
            }
        }
    }

    if (!extreme_values_found.empty()) {
        std::cerr << "Found " << extreme_values_found.size() << " total extreme elevation values during tile processing\n";
        std::cerr << "This may indicate corrupted tile data or areas with invalid elevation data\n";
    }

    fill_nan_values(height_grid);
    filter_elevation_outliers(height_grid);

    constexpr double SMALL_GRID_REF = 100.0;
    constexpr double SMALL_SIGMA_REF = 15.0;
    constexpr double LARGE_GRID_REF = 1000.0;
    constexpr double LARGE_SIGMA_REF = 7.0;

    double grid_size = static_cast<double>(std::min(grid_width, grid_height));
    if (grid_size < 1.0) grid_size = 1.0;

    double sigma = 0.0;
    if (grid_size <= SMALL_GRID_REF) {
        sigma = SMALL_SIGMA_REF * (grid_size / SMALL_GRID_REF);
    } else {
        double ln_small = std::log(SMALL_GRID_REF);
        double ln_large = std::log(LARGE_GRID_REF);
        double log_grid_size = std::log(grid_size);
        double t = (log_grid_size - ln_small) / (ln_large - ln_small);
        sigma = SMALL_SIGMA_REF + t * (LARGE_SIGMA_REF - SMALL_SIGMA_REF);
    }

    std::vector<std::vector<double>> blurred = apply_gaussian_blur(height_grid, sigma);

    std::vector<std::vector<int32_t>> mc_heights;
    mc_heights.reserve(blurred.size());

    double min_height = std::numeric_limits<double>::infinity();
    double max_height = -std::numeric_limits<double>::infinity();
    int extreme_low_count = 0;
    int extreme_high_count = 0;

    for (auto const& row : blurred) {
        for (double val : row) {
            if (val < min_height) min_height = val;
            if (val > max_height) max_height = val;
            if (val < -1000.0) ++extreme_low_count;
            if (val > 10000.0) ++extreme_high_count;
        }
    }

    std::cerr << "Height data range: " << min_height << " to " << max_height << " m\n";
    if (extreme_low_count > 0) {
        std::cerr << "WARNING: Found " << extreme_low_count << " pixels with extremely low elevations (< -1000m)\n";
    }
    if (extreme_high_count > 0) {
        std::cerr << "WARNING: Found " << extreme_high_count << " pixels with extremely high elevations (> 10000m)\n";
    }

    double height_range = max_height - min_height;
    if (height_range == 0.0) height_range = 1.0;

    double height_scale = BASE_HEIGHT_SCALE * std::sqrt(scale);
    double scaled_range = height_range * height_scale;

    double available_y_range = static_cast<double>(MAX_Y - ground_level);
    double safety_margin = 0.9;
    double max_allowed_range = available_y_range * safety_margin;

    if (scaled_range > max_allowed_range) {
        double adjustment_factor = max_allowed_range / scaled_range;
        height_scale *= adjustment_factor;
        scaled_range = height_range * height_scale;
        std::cerr << "Height range too large, applying scaling adjustment factor: " << adjustment_factor << "\n";
        std::cerr << "Adjusted scaled range: " << scaled_range << " blocks\n";
    }

    for (auto const& row : blurred) {
        std::vector<int32_t> mc_row;
        mc_row.reserve(row.size());
        for (double h : row) {
            double relative_height = (h - min_height) / height_range;
            double scaled_height = relative_height * scaled_range;
            int32_t block_y = static_cast<int32_t>(std::round(static_cast<double>(ground_level) + scaled_height));
            if (block_y < ground_level) block_y = ground_level;
            if (block_y > MAX_Y) block_y = MAX_Y;
            mc_row.push_back(block_y);
        }
        mc_heights.push_back(std::move(mc_row));
    }

    int32_t min_block_height = std::numeric_limits<int32_t>::max();
    int32_t max_block_height = std::numeric_limits<int32_t>::min();
    for (auto const& row : mc_heights) {
        for (int32_t v : row) {
            if (v < min_block_height) min_block_height = v;
            if (v > max_block_height) max_block_height = v;
        }
    }
    std::cerr << "Minecraft height data range: " << min_block_height << " to " << max_block_height << " blocks\n";

    ElevationData result;
    result.heights = std::move(mc_heights);
    result.width = grid_width;
    result.height = grid_height;
    return std::make_optional(result);
}

#endif
