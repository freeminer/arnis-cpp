#include "config.h"
#if BUILD_UNITTESTS
#include "advtrains.h"
#include "unittest/test.h"

#include <map>
#include <random>
#include <set>

namespace
{
using namespace arnis;
using namespace arnis::block_definitions;
namespace at = arnis::railways::advtrains;
using Connections = std::vector<std::pair<int, int>>;

// Connection heights in hundredths, copied from track_reg_helper.lua. These
// assertions exercise the same reciprocal-port rule as get_adjacent_rail().
struct TrackDefinitions
{
	std::vector<std::pair<Block *, Block>> saved;
	std::map<content_t, Connections> definitions;
	std::array<bool, 6> flags{ADVTRAINS_AVAILABLE, ADVTRAINS_JUNCTIONS_AVAILABLE,
			ADVTRAINS_CROSSINGS_AVAILABLE, ADVTRAINS_SLOPES_AVAILABLE,
			ADVTRAINS_GENTLE_SLOPES_AVAILABLE, ADVTRAINS_DIAGONAL_SLOPES_AVAILABLE};
	void add(Block &block, Connections ports)
	{
		saved.emplace_back(&block, block);
		block = Block(1000 + saved.size());
		definitions.emplace(block.id(), std::move(ports));
	}
	TrackDefinitions()
	{
		ADVTRAINS_AVAILABLE = ADVTRAINS_JUNCTIONS_AVAILABLE = ADVTRAINS_SLOPES_AVAILABLE =
				ADVTRAINS_GENTLE_SLOPES_AVAILABLE = true;
		ADVTRAINS_CROSSINGS_AVAILABLE = false;
		ADVTRAINS_DIAGONAL_SLOPES_AVAILABLE = true;
		std::array<Block *, 4> straight{&ADV_RAIL_STRAIGHT_0, &ADV_RAIL_STRAIGHT_30,
				&ADV_RAIL_STRAIGHT_45, &ADV_RAIL_STRAIGHT_60};
		std::array<Block *, 4> curve{&ADV_RAIL_CURVE_0, &ADV_RAIL_CURVE_30,
				&ADV_RAIL_CURVE_45, &ADV_RAIL_CURVE_60};
		for (int d = 0; d < 4; ++d) {
			add(*straight[d], {{d, 0}, {d + 8, 0}});
			add(*curve[d], {{d, 0}, {d + 7, 0}});
			add(ADV_RAIL_SWITCH_LEFT_STRAIGHT[d], {{d, 0}, {d + 8, 0}, {d + 7, 0}});
			add(ADV_RAIL_SWITCH_RIGHT_STRAIGHT[d], {{d, 0}, {d + 8, 0}, {d + 9, 0}});
			add(ADV_RAIL_Y_TURNOUT[d], {{d, 0}, {d + 7, 0}, {d + 9, 0}});
			add(ADV_RAIL_THREE_WAY_STRAIGHT[d],
					{{d, 0}, {d + 7, 0}, {d + 8, 0}, {d + 9, 0}});
			add(ADV_RAIL_PERP_CROSSING[d], {{d, 0}, {d + 8, 0}, {d + 4, 0}, {d + 12, 0}});
		}
		add(ADV_RAIL_SLOPE_UP, {{8, 0}, {0, 50}});
		add(ADV_RAIL_SLOPE_DOWN, {{8, 50}, {0, 100}});
		add(ADV_RAIL_GENTLE_SLOPE[0], {{8, 0}, {0, 33}});
		add(ADV_RAIL_GENTLE_SLOPE[1], {{8, 33}, {0, 66}});
		add(ADV_RAIL_GENTLE_SLOPE[2], {{8, 66}, {0, 100}});
		add(ADV_RAIL_DIAGONAL_SLOPE[0], {{10, 0}, {2, 50}});
		add(ADV_RAIL_DIAGONAL_SLOPE[1], {{10, 50}, {2, 100}});
	}
	~TrackDefinitions()
	{
		for (auto &[ptr, value] : saved)
			*ptr = value;
		ADVTRAINS_AVAILABLE = flags[0];
		ADVTRAINS_JUNCTIONS_AVAILABLE = flags[1];
		ADVTRAINS_CROSSINGS_AVAILABLE = flags[2];
		ADVTRAINS_SLOPES_AVAILABLE = flags[3];
		ADVTRAINS_GENTLE_SLOPES_AVAILABLE = flags[4];
		ADVTRAINS_DIAGONAL_SLOPES_AVAILABLE = flags[5];
		world_editor::WorldEditor editor;
		at::prepare_network({}, editor);
	}
	std::optional<int> port(Block block, int direction) const
	{
		for (const auto &[d, height] : definitions.at(block.id()))
			if ((d + block.getParam2() * 4) % 16 == direction)
				return height;
		return {};
	}
	void check(const at::RailPlan &plan) const
	{
		UASSERT(plan.line.size() >= 2);
		UASSERT(plan.line.size() == plan.heights.size());
		UASSERT(plan.line.size() == plan.rails.size());
		for (std::size_t i = 1; i < plan.line.size(); ++i) {
			const auto direction = at::direction_between(plan.line[i - 1], plan.line[i]);
			UASSERT(direction.has_value());
			const auto a = port(plan.rails[i - 1], *direction);
			const auto b = port(plan.rails[i], (*direction + 8) % 16);
			UASSERT(a.has_value());
			UASSERT(b.has_value());
			UASSERT(plan.heights[i - 1] * 100 + *a == plan.heights[i] * 100 + *b);
		}
	}
};

ProcessedWay way(std::uint64_t id, std::initializer_list<ProcessedNode> nodes,
		std::initializer_list<std::pair<const std::string, std::string>> tags = {
				{"railway", "rail"}})
{
	ProcessedWay result{id, nodes, {}};
	result.tags.insert(tags);
	return result;
}

class TestFmAdvtrains : public TestBase
{
public:
	TestFmAdvtrains() { TestManager::registerTestModule(this); }
	const char *getName() override { return "TestFmAdvtrains"; }
	void runTests(IGameDef *) override
	{
		TEST(testProfiles);
		TEST(testRoutes);
		TEST(testNetwork);
	}
	void testProfiles()
	{
		TrackDefinitions definitions;
		std::mt19937 random(913);
		for (int family = 0; family < 3; ++family) {
			ADVTRAINS_GENTLE_SLOPES_AVAILABLE = family == 0;
			ADVTRAINS_SLOPES_AVAILABLE = family != 2;
			ADVTRAINS_DIAGONAL_SLOPES_AVAILABLE = family != 2;
			for (int sample = 0; sample < 100; ++sample) {
				at::RailPlan plan;
				for (pos_t z = 0; z < 48; ++z) {
					plan.line.emplace_back(sample % 3 == 0 ? z : 0, z);
					plan.heights.push_back(sample < 50 ? z / 9 : random() % 12);
				}
				if (sample % 2)
					std::reverse(plan.heights.begin(), plan.heights.end());
				std::vector<bool> flat(plan.line.size(), false);
				flat.front() = flat.back() = flat[23] = true;
				at::fit_profile(plan.line, plan.heights, flat);
				for (std::size_t i = 0; i < plan.line.size(); ++i) {
					const auto slope = at::slope_rail(plan.line, plan.heights, i);
					if (flat[i])
						UASSERT(!slope.has_value());
					plan.rails.push_back(
							slope.value_or(*at::connected_rail(plan.line, i)));
				}
				definitions.check(plan);
				const auto before = plan.heights;
				at::fit_profile(plan.line, plan.heights, flat);
				UASSERT(before == plan.heights);
			}
		}
	}
	void testRoutes()
	{
		TrackDefinitions definitions;
		for (const auto target :
				{at::XZ{11, 31}, at::XZ{-17, 9}, at::XZ{40, 0}, at::XZ{2, 3}}) {
			const int d = at::closest_direction(target.first, target.second);
			at::RailPlan plan;
			plan.line = at::route({0, 0}, target, d, d);
			UASSERT(!plan.line.empty());
			UASSERT(plan.line.front() == at::XZ(0, 0));
			UASSERT(plan.line.back() == target);
			for (std::size_t i = 0; i < plan.line.size(); ++i) {
				const auto rail = at::connected_rail(plan.line, i);
				UASSERT(rail.has_value());
				plan.rails.push_back(*rail);
				plan.heights.push_back(0);
			}
			definitions.check(plan);
		}
		UASSERT(!at::two_connection_rail(0, 4));
	}
	void testNetwork()
	{
		TrackDefinitions definitions;
		world_editor::WorldEditor editor;
		std::vector<ProcessedElement> elements{way(10, {{1, {}, 0, -60}, {2, {}, 0, 0}}),
				way(20, {{2, {}, 0, 0}, {3, {}, 0, 20}},
						{{"railway", "rail"}, {"bridge", "yes"}, {"layer", "1"}}),
				way(30, {{3, {}, 0, 20}, {4, {}, 0, 80}}),
				way(40, {{4, {}, 0, 80}, {5, {}, 0, 140}},
						{{"railway", "rail"}, {"tunnel", "yes"}, {"layer", "-1"}}),
				way(50, {{6, {}, -20, 10}, {7, {}, 20, 10}}),
				way(60, {{3, {}, 0, 20}, {8, {}, 30, 70}})};
		at::prepare_network(elements, editor);
		std::map<std::uint64_t, at::RailPlan> first;
		for (const auto &element : elements) {
			const auto &w = element.as_way();
			const auto *plan = at::get_plan(w);
			UASSERT(plan);
			definitions.check(*plan);
			first.emplace(w.id, *plan);
		}
		UASSERT(first.at(10).heights.back() == first.at(20).heights.front());
		UASSERT(first.at(20).heights.back() == first.at(30).heights.front());
		UASSERT(first.at(20).heights.back() == first.at(60).heights.front());
		UASSERT(first.at(30).heights.back() == first.at(40).heights.front());
		UASSERT(first.at(50).heights.front() == 0);
		UASSERT(first.at(20).heights.front() > 0);
		std::reverse(elements.begin(), elements.end());
		at::prepare_network(elements, editor);
		for (const auto &element : elements) {
			const auto &w = element.as_way();
			const auto &a = first.at(w.id);
			const auto &b = *at::get_plan(w);
			UASSERT(a.line == b.line && a.heights == b.heights);
			for (std::size_t i = 0; i < a.rails.size(); ++i)
				UASSERT(a.rails[i].id() == b.rails[i].id() &&
						a.rails[i].getParam2() == b.rails[i].getParam2());
		}
	}
};
TestFmAdvtrains g_test_instance;
} // namespace
#endif
