#pragma once

#include <array>
#include <cstdint>
#include <filesystem>
#include <optional>
#include <system_error>
#include <vector>

namespace arnis::live::region
{
constexpr std::size_t SLOT_COUNT = 1024;

struct Bits
{
	std::array<std::uint64_t, 16> words{};
	static Bits all();
	bool get(std::size_t index) const;
	void set(std::size_t index);
	std::uint32_t count() const;
	bool is_full() const;
	Bits united(const Bits &other) const;
};

struct Slot
{
	enum class Kind : std::uint8_t
	{
		Empty,
		Inline,
		External
	};
	Kind kind = Kind::Empty;
	std::uint8_t version = 0;
	std::vector<std::uint8_t> data;
	static Slot inline_chunk(std::uint8_t compression, std::vector<std::uint8_t> payload);
	static Slot external_chunk(std::uint8_t compression);
};

std::size_t slot_index(std::int32_t cx, std::int32_t cz);
std::filesystem::path region_file(
		const std::filesystem::path &dir, std::int32_t rx, std::int32_t rz);
std::filesystem::path mcc_file(
		const std::filesystem::path &dir, std::int32_t cx, std::int32_t cz);
std::optional<Bits> occupied(const std::filesystem::path &path, std::error_code &ec);
Bits external_slots(const std::filesystem::path &path, std::error_code &ec);
std::vector<Slot> read_slots(const std::filesystem::path &path, std::error_code &ec);
bool write_slots(const std::filesystem::path &path, const std::vector<Slot> &slots,
		std::error_code &ec);
Bits add_stubs(const std::filesystem::path &path, std::error_code &ec);
bool write_mcc(const std::filesystem::path &dir, std::int32_t cx, std::int32_t cz,
		const std::vector<std::uint8_t> &compressed, std::error_code &ec);
bool link_no_replace(const std::filesystem::path &src, const std::filesystem::path &dst,
		std::error_code &ec);
std::size_t normalize(const std::filesystem::path &path,
		const std::filesystem::path &region_dir, std::int32_t rx, std::int32_t rz,
		std::error_code &ec);
std::size_t fill_from(const std::filesystem::path &live,
		const std::filesystem::path &staged, const std::filesystem::path &region_dir,
		std::int32_t rx, std::int32_t rz, std::error_code &ec);
std::size_t clear_void(const std::filesystem::path &path, std::error_code &ec);
bool is_void_chunk(std::uint8_t compression, const std::vector<std::uint8_t> &payload);
std::optional<std::pair<std::int32_t, std::int32_t>> parse_region_name(
		const std::string &name);
bool replace_atomically(const std::filesystem::path &path,
		const std::vector<std::uint8_t> &bytes, std::error_code &ec);
} // namespace arnis::live::region
