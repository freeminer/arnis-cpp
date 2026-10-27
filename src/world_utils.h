#pragma once
#include <cstddef>
#include <filesystem>
#include <cstdint>
#include <string>
#include <string_view>
#include <utility>
#include <vector>
#include <optional>
#include <memory>
#include "coordinate_system/geographic/llbbox.h"
#include "celestial.h"
namespace arnis
{
struct Args;
enum class WorldFormat;
namespace world_utils
{
inline constexpr std::int32_t DIMENSION_FOLDERS_DATA_VERSION = 4772;
inline constexpr std::uint64_t DISK_HEADROOM_BYTES = 3ull * 1024 * 1024 * 1024;
inline constexpr std::size_t MAX_CUSTOM_WORLD_NAME_CHARS = 48;
inline constexpr std::string_view TALL_DATAPACK_NAME = "arnis_tall";

struct WorldSizeOptions
{
	bool interior = false;
	bool bake_lighting = false;
	bool voxy_lod = false;
	bool caves = false;
	bool fillground = false;
	bool tall = false;
	static WorldSizeOptions of(const Args &args);
};

struct RunSizeError
{
	bool area = false;
	std::string detail;
	const char *short_message() const;
};

std::uint64_t estimated_world_bytes(
		double blocks_km2, WorldFormat format, const WorldSizeOptions &options);
double blocks_km2(const Args &args, const geographic::LLBBox &bbox);
bool check_disk_space(const std::filesystem::path &dir, std::uint64_t needed,
		std::string *error = nullptr);
bool check_run_size(const Args &args, const geographic::LLBBox &bbox, WorldFormat format,
		const std::filesystem::path &dir, RunSizeError *error = nullptr);
bool check_disk_for_extended_height(const Args &args, const geographic::LLBBox &bbox,
		WorldFormat format, const std::filesystem::path &dir,
		std::string *error = nullptr);

struct WorldLayout
{
	enum class Kind
	{
		Legacy,
		Dimensions
	};

	Kind kind{Kind::Legacy};
	static WorldLayout of(const std::filesystem::path &world);
	std::filesystem::path overworld_dir(const std::filesystem::path &world) const;
	std::filesystem::path maps_dir(const std::filesystem::path &world) const;
};

// Holds Java's session.lock while a One World area is being written. The lock
// is compatible with Java's whole-file fcntl lock on Unix and LockFileEx on
// Windows; destruction releases it and removes the marker file.
class SessionLock
{
public:
	~SessionLock();
	SessionLock(const SessionLock &) = delete;
	SessionLock &operator=(const SessionLock &) = delete;
	static std::unique_ptr<SessionLock> acquire(
			const std::filesystem::path &world, std::string *error = nullptr);

private:
	SessionLock(std::filesystem::path path, std::intptr_t handle,
			std::uint64_t identity_a, std::uint64_t identity_b);
	std::filesystem::path path_;
	std::intptr_t handle_ = -1;
	std::uint64_t identity_a_ = 0, identity_b_ = 0;
};

// True only when another process currently holds session.lock. A lock held by
// this process is reported as available, matching Rust's One World behavior.
bool world_is_locked(const std::filesystem::path &world);

std::optional<std::int32_t> level_data_version(const std::filesystem::path &world);
// Read Data.LevelName from Java level.dat; absent or invalid NBT returns no value.
std::optional<std::string> level_name(const std::filesystem::path &world);
// Read the world spawn X/Z from legacy SpawnX/SpawnZ or the modern spawn.pos
// int array in level.dat; return no value for absent or invalid NBT.
std::optional<std::pair<std::int32_t, std::int32_t>> level_spawn_xz(
		const std::filesystem::path &world);
std::optional<std::int32_t> map_id_counter(const std::filesystem::path &path);
// Update the Java world spawn and, for post-26.1 worlds, the separately stored
// single-player position. Returns false when the NBT files cannot be updated.
bool set_spawn_in_level_dat(const std::filesystem::path &world, std::int32_t x,
		std::int32_t y, std::int32_t z);
// Rust-equivalent best-effort cleanup: remove only the untouched packaged
// template region, never chunks changed by generation or the game.
void remove_untouched_template_region(const std::filesystem::path &world);
// Update Java level metadata after generation; generator edits apply only when
// the existing overworld is flat, matching Rust's conservative behavior.
bool apply_java_world_settings(const std::filesystem::path &world, int game_type,
		std::int64_t world_time, bool void_world, std::int32_t base_y, std::int32_t min_y,
		std::string *error = nullptr);
// Configure an outbreak world so its zombies survive and the game starts on Normal.
bool start_in_survival(const std::filesystem::path &world, std::string *error = nullptr);
// Whether the world already contains the bundled extended-height datapack.
bool has_tall_datapack(const std::filesystem::path &world);
// Install the bundled dimension override required for worlds generated beyond
// vanilla's build-height limits, then enable the pack in level.dat.
bool install_tall_datapack(
		const std::filesystem::path &world, std::string *error = nullptr);
bool enable_datapack_in_level_dat(const std::filesystem::path &world,
		std::string_view pack_dir_name, std::string *error = nullptr);
bool touch_last_played(const std::filesystem::path &world, std::string *error = nullptr);
// Remove Anvil chunks whose embedded xPos/zPos do not match their region slot.
// Used once when upgrading legacy One World manifests.
std::optional<std::size_t> drop_misplaced_chunks(
		const std::filesystem::path &world, std::string *error = nullptr);
bool replace_file_atomically(
		const std::filesystem::path &, const std::vector<std::uint8_t> &);
std::filesystem::path default_bedrock_output_directory();
std::filesystem::path get_bedrock_output_directory();
std::filesystem::path get_luanti_worlds_directory();
std::string sanitize_for_filename(const std::string &name);
std::string world_folder_name(const std::string &raw);
// Select a Rust-compatible unique world directory name under base_path.
// Empty/invalid custom names use the automatic "Arnis World N" scheme.
std::string unique_world_folder_name(
		const std::filesystem::path &base_path, const std::string &custom_name = {});
std::string get_area_name_for_bedrock(const geographic::LLBBox &bbox);
std::pair<std::filesystem::path, std::string> build_bedrock_output(
		const geographic::LLBBox &bbox, const std::filesystem::path &output_dir,
		CelestialBody body = CelestialBody::Earth);
}
}
