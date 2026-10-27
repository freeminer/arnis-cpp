#include "config.h"
#if BUILD_UNITTESTS
#include "advtrains.h"
#include "../clipping.h"
#include "rail_track.h"
#include "unittest/test.h"

#include <map>
#include <random>
#include <set>
#include <future>

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
		TEST(testRustAtGradeProfileParity);
		TEST(testRustRailJoinSelectionParity);
		TEST(testRustAreaEdgeCrossingOrder);
		TEST(testRoutes);
		TEST(testNetwork);
		TEST(testChunkSeamsKeepSourceTopology);
		TEST(testParallelWorkersKeepNativeNetworkAndCompletion);
		TEST(testCrossoverAvoidsBothConnectedLayers);
		TEST(testFlyoverKeepsItsSeparateLayer);
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
	void testRustAtGradeProfileParity()
	{
		// A one-block terrain dip is cheaper to fill than to put a step in the
		// track, matching rail_track.rs::a_one_block_dip_is_filled_rather_than_stepped_into.
		std::vector<int> ground(20, 5);
		ground[10] = 4;
		const std::vector<bool> straight(ground.size(), true);
		const auto beds = railways::rail_profile(
				straight,
				[&](std::size_t i) { return std::pair{ground[i] - 10, ground[i] + 10}; },
				[&](std::size_t i, int y) {
					return static_cast<double>(std::abs(y - ground[i]));
				},
				2.5);
		UASSERT(beds == std::vector<int>(ground.size(), 5));

		// A straight segment can climb at an endpoint only when the joined way
		// continues through it. A horizontal-to-diagonal join closes that ramp.
		const std::vector<std::pair<int, int>> corner{{0, 0}, {1, 0}};
		const auto open_ends = railways::straight_cells(corner);
		const auto joined_corner =
				railways::straight_cells(corner, std::pair{-1, 0}, std::pair{1, 1});
		UASSERT(open_ends[1]);
		UASSERT(!joined_corner[1]);
	}
	void testRustRailJoinSelectionParity()
	{
		const auto surface = [](const ProcessedWay &) {
			return std::optional<railways::TrackLevel>(railways::TrackLevel::Surface);
		};
		const std::vector<ProcessedElement> branch{
				ProcessedElement(way(1, {{1, {}, 0, 0}, {2, {}, 20, 0}})),
				ProcessedElement(way(2, {{2, {}, 20, 0}, {3, {}, 40, 0}})),
				ProcessedElement(way(3, {{2, {}, 20, 0}, {4, {}, 40, 10}}))};
		const auto joins = railways::RailJoins::build(branch, surface);
		UASSERT(joins.ends(1)[1].owns);
		UASSERT(joins.ends(2)[0].owns);
		UASSERT(!joins.ends(3)[0].owns);
		UASSERT(!joins.ends(1)[1].beyond.empty());
		UASSERT(joins.ends(1)[1].beyond.front() == std::pair(21, 0));

		// A way passing through a node owns the shared rail cell, even when a
		// spur terminates on that interior node.
		const std::vector<ProcessedElement> through{
				ProcessedElement(way(4, {{1, {}, 0, 0}, {2, {}, 20, 0}, {3, {}, 40, 0}})),
				ProcessedElement(way(5, {{2, {}, 20, 0}, {5, {}, 40, 8}}))};
		const auto through_joins = railways::RailJoins::build(through, surface);
		UASSERT(!through_joins.ends(5)[0].owns);
		UASSERT(through_joins.ends(5)[0].shared);
	}
	void testRustAreaEdgeCrossingOrder()
	{
		railways::RailJoins joins;
		const std::vector<std::pair<int, int>> cells{{-1, 5}, {0, 5}, {10, 5}, {11, 5}};
		UASSERT(joins.edge_crossings(cells).empty());
		joins.meet_at_edge_of(cartesian::XZBBox::rect_from_min_max(0, 0, 10, 10));
		const auto crossings = joins.edge_crossings(cells);
		UASSERT(crossings.size() == 2);
		// Rust's contract is [inside, outside], including when the path enters
		// the area from outside rather than leaving it.
		UASSERT((crossings[0] == std::array<std::size_t, 2>{1, 0}));
		UASSERT((crossings[1] == std::array<std::size_t, 2>{2, 3}));
	}
	void testRoutes()
	{
		TrackDefinitions definitions;
		for (const auto &target :
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
		// Repeated visits to a position with different headings must consult the
		// stable obstacle map only once, while still routing around obstacles.
		std::map<at::XZ, unsigned> probes;
		const auto detour = at::route({0, 0}, {0, 30}, 0, 0, [&](const at::XZ &p) {
			++probes[p];
			return p == at::XZ(0, 15);
		});
		UASSERT(!detour.empty());
		UASSERT(detour.front() == at::XZ(0, 0));
		UASSERT(detour.back() == at::XZ(0, 30));
		for (const auto &p : detour)
			UASSERT(p != at::XZ(0, 15));
		for (const auto &[p, count] : probes)
			UASSERT(count == 1);
	}
	void testChunkSeamsKeepSourceTopology()
	{
		TrackDefinitions definitions;
		WorldEditor source_editor;
		const auto trunk = way(10, {{1, {}, 0, -120}, {2, {}, 0, 0}, {3, {}, 48, 160}});
		const std::vector<ProcessedElement> source{
				trunk, way(20, {{2, {}, 0, 0}, {4, {}, -48, 160}})};
		const auto shared = at::build_network(source, source_editor);
		const auto &expected = shared->plans.at(trunk.id);
		definitions.check(expected);
		// Adjacent chunk halos clip different endpoints and can omit a turnout
		// branch. Rebuilding from either view must never replace the source plan.
		for (bool reverse : {false, true}) {
			for (int n = 0; n < 2; ++n) {
				const int chunk = reverse ? 1 - n : n;
				const XZBBox context(-80, chunk ? 32 : -160, 80, chunk ? 192 : 95);
				auto clipped = trunk;
				clipped.nodes = clipping::clip_way_to_bbox(trunk.nodes, context);
				UASSERT(!clipped.nodes.empty());
				UASSERT(clipped.nodes.front().z != trunk.nodes.front().z ||
						clipped.nodes.back().z != trunk.nodes.back().z);
				WorldEditor editor;
				editor.native_rail_network = shared;
				// Chunk-local terrain and previously written roads must not change
				// an already resolved route's grade at the shared seam.
				for (const auto &[x, z] : expected.line)
					editor.register_road_surface_y(x, z, 100 + chunk);
				auto worker = editor.make_tile_worker();
				UASSERT(worker.native_rail_network == shared);
				at::prepare_network({ProcessedElement(clipped)}, worker);
				const auto *plan = at::get_plan(clipped);
				UASSERT(plan == &expected);
				definitions.check(*plan);
				// The branch is outside this local input but its turnout survives.
				UASSERT(at::get_plan(source[1].as_way()) == &shared->plans.at(20));
				WorldEditor detached;
				at::prepare_network({}, detached);
				UASSERT(!at::get_plan(trunk));
			}
		}
	}

	void testParallelWorkersKeepNativeNetworkAndCompletion()
	{
		TrackDefinitions definitions;
		WorldEditor editor;
		const auto rail = way(42, {{1, {}, 0, 0}, {2, {}, 32, 96}});
		editor.native_rail_network =
				at::prepare_network({ProcessedElement(rail)}, editor);
		const auto *expected = at::get_plan(rail);
		UASSERT(expected && !expected->line.empty());
		std::vector<std::future<WorldEditor>> jobs;
		for (int i = 0; i < 2; ++i) {
			auto worker = editor.make_tile_worker();
			jobs.push_back(std::async(std::launch::async,
					[worker = std::move(worker), rail, expected]() mutable {
						// Mirrors begin_generation_tile on a new async thread.
						at::use_network(worker);
						if (at::get_plan(rail) == expected)
							at::mark_generated(rail, worker);
						return std::move(worker);
					}));
		}
		for (auto &job : jobs) {
			auto worker = job.get();
			UASSERT(worker.native_generated_rails.contains(rail.id));
			UASSERT(editor.merge_tile_worker(std::move(worker), 0, 0, 32, 96));
		}
		UASSERT(editor.native_generated_rails == std::set<std::uint64_t>{rail.id});
		UASSERT(at::get_plan(rail) == expected);
		definitions.check(*expected);
	}

	void testCrossoverAvoidsBothConnectedLayers()
	{
		TrackDefinitions definitions;
		WorldEditor editor;
		// Port Washington Branch, ny2: the untagged crossover joins a bridge
		// and a surface approach. Previously it overwrote a mainline rail cell.
		std::vector<ProcessedElement> elements{
				way(49175623,
						{{3263868996, {}, 12924, -3444}, {13571301748, {}, 12886, -3457},
								{13571301749, {}, 12878, -3460},
								{3263868992, {}, 12855, -3468}},
						{{"railway", "rail"}, {"bridge", "yes"}, {"layer", "1"}}),
				way(49175625,
						{{3263868992, {}, 12855, -3468}, {2819593657, {}, 12834, -3476},
								{13571558876, {}, 12711, -3517},
								{3263868982, {}, 12678, -3529},
								{3263868980, {}, 12485, -3587}}),
				way(319903734,
						{{3263868989, {}, 12857, -3472}, {3263868995, {}, 12926, -3448}},
						{{"railway", "rail"}, {"bridge", "yes"}, {"layer", "1"}}),
				way(319903754,
						{{3263868979, {}, 12487, -3591}, {3263868981, {}, 12680, -3533},
								{13571558877, {}, 12713, -3522},
								{13571301750, {}, 12740, -3512},
								{3263868984, {}, 12836, -3480},
								{3263868989, {}, 12857, -3472}}),
				way(1479433697,
						{{13571301749, {}, 12878, -3460},
								{13571301750, {}, 12740, -3512}},
						{{"railway", "rail"}, {"service", "crossover"}})};
		const auto network = at::build_network(elements, editor);
		std::map<std::tuple<pos_t, pos_t, pos_t>, Block> cells;
		for (const auto &[id, plan] : network->plans) {
			definitions.check(plan);
			for (std::size_t i = 0; i < plan.line.size(); ++i) {
				const auto &[x, z] = plan.line[i];
				const auto [it, inserted] = cells.emplace(
						std::tuple<pos_t, pos_t, pos_t>{x, plan.heights[i], z},
						plan.rails[i]);
				UASSERT(inserted || it->second == plan.rails[i]);
			}
		}
		UASSERT(network->plans.size() == elements.size());
	}

	void testFlyoverKeepsItsSeparateLayer()
	{
		TrackDefinitions definitions;
		WorldEditor editor;
		const auto network = at::build_network(
				{way(1, {{1, {}, 0, -30}, {2, {}, 0, 30}},
						 {{"railway", "rail"}, {"bridge", "yes"}, {"layer", "1"}}),
						way(2, {{3, {}, -30, 0}, {4, {}, 30, 0}})},
				editor);
		std::vector<int> crossing_heights;
		for (const auto &[id, plan] : network->plans) {
			definitions.check(plan);
			for (std::size_t i = 0; i < plan.line.size(); ++i)
				if (plan.line[i] == at::XZ{0, 0})
					crossing_heights.push_back(plan.heights[i]);
		}
		UASSERT(crossing_heights.size() == 2);
		UASSERT(std::abs(crossing_heights[0] - crossing_heights[1]) >= 4);
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
