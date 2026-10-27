#include "fm_datapack_disasters.h"

#include "world_editor/floor_state.h"

#include <nlohmann/json.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <iomanip>
#include <sstream>
#include <string_view>

namespace arnis::datapack::disasters
{
namespace
{
struct KindText
{
	int id;
	const char *name_earth;
	const char *name_moon;
	const char *name_mars;
	const char *advice;
	const char *over_earth;
	const char *over_moon;
	const char *over_mars;
	const char *phrase_earth;
	const char *phrase_moon;
	const char *phrase_mars;
};

constexpr std::array<KindText, 11> KINDS{{
		{1, "Meteor Shower", "Meteor Shower", "Meteor Shower",
				"Keep out of the red rings", "The meteor shower is over.",
				"The meteor shower is over.", "The meteor shower is over.",
				"a meteor shower", "a meteor shower", "a meteor shower"},
		{2, "Thunderstorm", "Thunderstorm", "Thunderstorm", "Stay off the rooftops",
				"The storm has passed.", "The storm has passed.", "The storm has passed.",
				"a thunderstorm", "a thunderstorm", "a thunderstorm"},
		{3, "Hailstorm", "Hailstorm", "Hailstorm", "Get under a roof",
				"The hail has stopped.", "The hail has stopped.", "The hail has stopped.",
				"a hailstorm", "a hailstorm", "a hailstorm"},
		{4, "Sinkholes", "Sinkholes", "Sinkholes", "Move when the ground cracks",
				"The ground has settled.", "The ground has settled.",
				"The ground has settled.", "sinkholes", "sinkholes", "sinkholes"},
		{5, "Tornado", "Tornado", "Dust Devil", "Stay out of its path",
				"The tornado is gone.", "The tornado is gone.", "The dust devil is gone.",
				"a tornado", "a tornado", "a dust devil"},
		{6, "Earthquake", "Moonquake", "Marsquake", "Get out of the buildings",
				"The shaking has stopped.", "The shaking has stopped.",
				"The shaking has stopped.", "an earthquake", "a moonquake",
				"a marsquake"},
		{7, "Flood", "Flood", "Flood", "Get to high ground", "The water has gone down.",
				"The water has gone down.", "The water has gone down.", "a flood",
				"a flood", "a flood"},
		{8, "Wildfire", "Wildfire", "Wildfire", "Keep ahead of the fire front",
				"The fire front has passed.", "The fire front has passed.",
				"The fire front has passed.", "a wildfire", "a wildfire", "a wildfire"},
		{9, "Tsunami", "Tsunami", "Tsunami", "Get away from the coast",
				"The sea has pulled back.", "The sea has pulled back.",
				"The sea has pulled back.", "a tsunami", "a tsunami", "a tsunami"},
		{10, "Kaiju", "Kaiju", "Kaiju", "Run, it can't be stopped",
				"The kaiju went back underground.", "The kaiju went back underground.",
				"The kaiju went back underground.", "a kaiju", "a kaiju", "a kaiju"},
		{11, "Snowstorm", "Snowstorm", "Snowstorm", "Get inside and keep warm",
				"The snow has stopped.", "The snow has stopped.", "The snow has stopped.",
				"a snowstorm", "a snowstorm", "a snowstorm"},
}};

bool has_kind(const std::vector<int> &kinds, int id)
{
	return std::find(kinds.begin(), kinds.end(), id) != kinds.end();
}

const char *body_text(
		CelestialBody body, const char *earth, const char *moon, const char *mars)
{
	return body == CelestialBody::Moon	 ? moon
		   : body == CelestialBody::Mars ? mars
										 : earth;
}

std::string json_quote(const std::string &value)
{
	return nlohmann::json(value).dump();
}

std::string number_word(std::size_t n)
{
	static constexpr std::array<const char *, 13> words{{"zero", "one", "two", "three",
			"four", "five", "six", "seven", "eight", "nine", "ten", "eleven", "twelve"}};
	return n < words.size() ? words[n] : std::to_string(n);
}

std::string count_text(std::size_t n)
{
	return n == 1 ? "one disaster" : number_word(n) + " disasters";
}

std::string ring(double radius, int points)
{
	std::ostringstream out;
	for (int p = 0; p < points; ++p)
		out << "execute rotated ~" << p * 360 / points
			<< " 0 run particle minecraft:dust{color:[1.0,0.25,0.05],scale:1.6} ^"
			<< radius << " ^0.2 ^ 0 0 0 0 1 force\n";
	out << "particle minecraft:dust{color:[1.0,0.55,0.1],scale:1.2} ~ ~0.3 ~ "
		   "0.3 0.1 0.3 0 3 force\n";
	return out.str();
}

int ring_of(int dx, int dz)
{
	return static_cast<int>(std::lround(std::sqrt(double(dx * dx + dz * dz))));
}

std::string sink_ring(int r)
{
	std::ostringstream out;
	for (int dz = -r; dz <= r; ++dz) {
		std::vector<std::pair<int, int>> runs;
		for (int dx = -r; dx <= r; ++dx) {
			if (ring_of(dx, dz) != r)
				continue;
			if (!runs.empty() && runs.back().second == dx - 1)
				runs.back().second = dx;
			else
				runs.emplace_back(dx, dx);
		}
		for (const auto &[x0, x1] : runs) {
			out << "$clone ~" << x0 << " ~-2 ~" << dz << " ~" << x1 << " ~30 ~" << dz
				<< " ~" << x0 << " ~-$(dd) ~" << dz << " replace force\n";
			out << "$fill ~" << x0 << " ~$(cl) ~" << dz << " ~" << x1 << " ~30 ~" << dz
				<< " minecraft:air\n";
		}
	}
	return out.str();
}

std::string funnel(CelestialBody body)
{
	const bool mars = body == CelestialBody::Mars;
	const char *color = mars ? "0.72,0.42,0.25" : "0.45,0.45,0.48";
	const char *cloud = mars ? "0.62,0.36,0.22" : "0.32,0.32,0.36";
	const char *ground = mars ? "minecraft:red_sand" : "minecraft:dirt";
	std::ostringstream out;
	for (int level = 0; level < 15; ++level) {
		const double h = level * 3.0;
		const double radius = 1.0 + .1 * h + .0028 * h * h;
		const int arms = h < 24.0 ? 3 : 4;
		for (int arm = 0; arm < arms; ++arm) {
			const int angle = (level * 17 + arm * 360 / arms) % 360;
			out << "execute rotated ~" << angle
				<< " 0 run particle minecraft:dust{color:[" << color << "],scale:2.5} ^"
				<< std::fixed << std::setprecision(1) << radius << " ^" << h
				<< " ^ 0.15 0.2 0.15 0 1 force\n";
		}
		if (level % 2 == 1)
			out << "execute rotated ~" << (level * 17 + 60) % 360
				<< " 0 run particle minecraft:large_smoke ^" << std::fixed
				<< std::setprecision(1) << radius * .8 << " ^" << h
				<< " ^ 0.2 0.2 0.2 0.01 1 force\n";
	}
	for (int arm = 0; arm < 8; ++arm)
		out << "execute rotated ~" << arm * 45 << " 0 run particle minecraft:dust{color:["
			<< cloud << "],scale:4.0} ^12 ^45 ^ 1 0.6 1 0 2 force\n";
	out << "particle minecraft:block{block_state:\"" << ground
		<< "\"} ~ ~0.3 ~ 2.5 0.2 2.5 0.15 6 force\n"
		   "particle minecraft:cloud ~ ~0.5 ~ 2 0.3 2 0.05 3 force\n";
	return out.str();
}

std::string json_text(const nlohmann::json &value)
{
	return value.dump();
}
} // namespace

Plan::Plan(CelestialBody body, int floor, int world_max_y) :
		body_(body), floor_(floor), snow_top_(std::min(floor + 383, world_max_y))
{
	if (body == CelestialBody::Earth)
		kinds_ = {1, 2, 3, 4, 5, 6, 10, 11};
	else if (body == CelestialBody::Moon)
		kinds_ = {1, 4, 6};
	else
		kinds_ = {1, 4, 5, 6};
}

std::string Plan::coming() const
{
	std::vector<std::string> phrases;
	for (const int id : kinds_)
		phrases.emplace_back(body_text(body_, KINDS[id - 1].phrase_earth,
				KINDS[id - 1].phrase_moon, KINDS[id - 1].phrase_mars));
	if (phrases.empty())
		return {};
	if (phrases.size() == 1)
		return phrases.front();
	std::string result;
	for (std::size_t i = 0; i + 1 < phrases.size(); ++i) {
		if (i)
			result += ", ";
		result += phrases[i];
	}
	return result + " and " + phrases.back();
}

std::vector<std::pair<std::string, std::string>> Plan::files(
		const std::string &area) const
{
	constexpr const char *root = "ov80/data/arnis/function/disaster/";
	std::vector<std::pair<std::string, std::string>> out;
	auto add = [&](std::string path, std::string content) {
		out.emplace_back(root + path, std::move(content));
	};
	std::ostringstream area_fn;
	area_fn << "data modify storage arnis:ds pool set value [";
	for (std::size_t i = 0; i < kinds_.size(); ++i)
		area_fn << (i ? "," : "") << kinds_[i];
	area_fn << "]\nscoreboard players set #floor arnis.ds " << floor_
			<< "\nscoreboard players set #has_snow arnis.ds 0\n"
			   "data modify storage arnis:ds wind_p set value \""
			<< (body_ == CelestialBody::Mars ? "minecraft:dust_plume" : "minecraft:cloud")
			<< "\"\n";
	// Body-specific labels: the table stores each variant separately.
	for (const char *field : {"names", "subs", "ends"}) {
		nlohmann::json values = nlohmann::json::array({""});
		for (const auto &kind : KINDS) {
			if (std::string_view(field) == "names")
				values.push_back(body_text(
						body_, kind.name_earth, kind.name_moon, kind.name_mars));
			else if (std::string_view(field) == "subs")
				values.push_back(kind.advice);
			else
				values.push_back(body_text(
						body_, kind.over_earth, kind.over_moon, kind.over_mars));
		}
		area_fn << "data modify storage arnis:ds " << field << " set value "
				<< values.dump() << '\n';
	}
	for (const char *name : {"flood", "tsunami"})
		area_fn << "scoreboard players set #has_" << name << " arnis.ds 0\n"
				<< "scoreboard players set #k_" << name << " arnis.ds 0\n"
				<< "data modify storage arnis:ds " << name << ".cells set value {}\n";
	add("area.mcfunction", area_fn.str());

	const std::string n = number_word(kinds_.size());
	add("text/intro.mcfunction",
			"title @s subtitle " +
					json_text({{"text", "Survive " + count_text(kinds_.size()) + " in " +
												area},
							{"color", "gray"}}) +
					"\n");
	add("text/coming.mcfunction",
			"tellraw @s " +
					json_text({{"text", "Coming to " + area +
												", in no set order: " + coming() + "."},
							{"color", "gray"}}) +
					"\n");
	add("text/victory.mcfunction",
			"title @a subtitle " +
					json_text({{"text", area + " survived all " + n},
							{"color", "yellow"}}) +
					"\n");
	add("tornado/funnel.mcfunction", funnel(body_));
	add("meteor/ring.mcfunction", ring(3.0, 12));
	add("meteor/ring_big.mcfunction", ring(5.0, 18));
	for (int r = 0; r <= 15; ++r)
		add("sinkhole/ring/" + std::to_string(r) + ".mcfunction", sink_ring(r));
	const std::string snow = "# It snows here as it is.\n";
	add("snow/swap.mcfunction", snow);
	add("snow/thaw.mcfunction", snow);
	add("flood/tops.mcfunction", "# No water here to rise.\n");
	add("tsunami/tops.mcfunction", "# No water here to rise.\n");
	return out;
}

nlohmann::json Plan::dialog(const std::string &area, bool has_pause_menu) const
{
	auto text = [](std::string value, const char *color) {
		return nlohmann::json{{"text", std::move(value)}, {"color", color}};
	};
	auto run = [](const char *command) {
		return nlohmann::json{{"type", "run_command"}, {"command", command}};
	};
	nlohmann::json exit = has_pause_menu
								  ? nlohmann::json{{"label", "Back"},
											{"action", {{"type", "show_dialog"},
															   {"dialog", "arnis:menu"}}}}
								  : nlohmann::json{{"label", "Close"}};
	return {{"type", "minecraft:multi_action"}, {"title", text("Disasters", "red")},
			{"body",
					nlohmann::json::array(
							{{{"type", "minecraft:plain_message"},
									 {"contents",
											 nlohmann::json::array({text("A disaster hits ",
																			"gray"),
													 text(area, "white"),
													 text(" every few minutes, in random order, each with a short warning first.",
															 "gray")})},
									 {"width", 300}},
									{{"type", "minecraft:plain_message"},
											{"contents", text("Coming: " + coming() + ".",
																 "gray")},
											{"width", 300}}})},
			{"columns", 3},
			{"actions",
					nlohmann::json::array(
							{{{"label", "Restart"},
									 {"tooltip", "Start again from the first disaster"},
									 {"action", run("trigger arnis.ds.ctl set 1")}},
									{{"label", "Skip the Wait"},
											{"tooltip", "Bring on the next disaster now"},
											{"action",
													run("trigger arnis.ds.ctl set 2")}},
									{{"label", "Stop"}, {"tooltip", "End the disasters"},
											{"action",
													run("trigger arnis.ds.ctl set 3")}}})},
			{"exit_action", std::move(exit)}, {"pause", false}};
}
} // namespace arnis::datapack::disasters
