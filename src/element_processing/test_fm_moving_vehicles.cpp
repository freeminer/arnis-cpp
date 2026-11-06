#include "config.h"
#if BUILD_UNITTESTS
#include "cars.h"
#include "traffic_lights.h"
#include "train_bodies.h"
#include "unittest/test.h"

namespace
{

class TestFmMovingVehicles : public TestBase
{
public:
	TestFmMovingVehicles() { TestManager::registerTestModule(this); }
	const char *getName() override { return "TestFmMovingVehicles"; }
	void runTests(IGameDef *) override
	{
		TEST(testCarPayloads);
		TEST(testTrainDoors);
		TEST(testJunctionApproaches);
		TEST(testClippedLanes);
	}

	void testCarPayloads()
	{
		const auto net = arnis::highways::cars::CarNetwork();
		const auto &[kind_path, kind] = net.kind_function();
		UASSERT(kind_path == "data/arnis/function/cars/respawn_kind.mcfunction");
		UASSERT(kind.find("kind:\"sedan\"") != std::string::npos);
		UASSERT(kind.find("kind:\"bus\",long:1b") != std::string::npos);
		const auto &[seat_path, seat] = net.seat_function();
		UASSERT(seat_path == "data/arnis/function/cars/seat_kind.mcfunction");
		UASSERT(seat.find("tag=arnis.k_bus") != std::string::npos);
		const auto files = net.datapack_functions();
		UASSERT(!files.empty());
		for (const auto &[path, contents] : files) {
			UASSERT(contents.find("return run run") == std::string::npos);
			if (path == "data/arnis/function/cars/look/l0.mcfunction")
				UASSERT(contents.find("arnis.car.long") != std::string::npos);
			if (path == "data/arnis/function/cars/clear.mcfunction") {
				UASSERT(contents.find("tag=!arnis.riding") != std::string::npos);
				UASSERT(contents.find("tag=arnis.car.long") != std::string::npos);
			}
			if (path == "data/arnis/function/cars/network.mcfunction")
				UASSERT(contents.find("#near_cap arnis.car 120") != std::string::npos);
		}
	}

	void testTrainDoors()
	{
		const arnis::train_bodies::Coach wagon = arnis::train_bodies::Container{0};
		UASSERT(arnis::train_bodies::doors(wagon).empty());
		UASSERT(arnis::train_bodies::door_key(wagon) == arnis::train_bodies::NO_DOORS);
		const arnis::train_bodies::Coach coach = arnis::train_bodies::MetroMid{};
		const auto spots = arnis::train_bodies::doors(coach);
		const auto &[keep, make] =
				arnis::train_bodies::door_functions("m160", spots, 1.0);
		UASSERT(!spots.empty());
		UASSERT(make.find("summon interaction") != std::string::npos);
		UASSERT(make.find("arnis.seat") != std::string::npos);
		UASSERT(keep.find("tag=arnis.ts$(c)") != std::string::npos);
	}

	void testJunctionApproaches()
	{
		const auto node = [](std::uint64_t id, int x, int z, bool signal = false) {
			arnis::ProcessedNode result{id, {}, x, z};
			if (signal)
				result.tags["highway"] = "traffic_signals";
			return result;
		};
		arnis::ProcessedWay a{1,
				{node(1, -40, 0), node(11, -8, 0, true), node(9, 0, 0),
						node(12, 8, 0, true), node(2, 40, 0)},
				{{"highway", "residential"}}};
		arnis::ProcessedWay b{2,
				{node(3, 0, -40), node(13, 0, -8, true), node(9, 0, 0),
						node(14, 0, 8, true), node(4, 0, 40)},
				{{"highway", "residential"}}};
		const std::vector<arnis::ProcessedElement> elements{
				arnis::ProcessedElement(std::move(a)),
				arnis::ProcessedElement(std::move(b))};
		const auto junctions = arnis::highways::Junctions::build(elements, 1.0);
		UASSERT(junctions.is_junction(9));
		UASSERT(!junctions.passed(elements[0].as_way().nodes, 1, -1));
		UASSERT(junctions.passed(elements[0].as_way().nodes, 1, 1));
		UASSERT(junctions.passed(elements[0].as_way().nodes, 3, -1));
		UASSERT(!junctions.passed(elements[0].as_way().nodes, 3, 1));
	}
	void testClippedLanes()
	{
		const auto n1 = arnis::ProcessedNode{1, {}, -30, 15};
		const auto n2 = arnis::ProcessedNode{2, {}, 40, 15};
		std::vector<arnis::ProcessedElement> elements{arnis::ProcessedWay{
				1, {n1, n2}, {{"highway", "residential"}, {"lanes", "2"}}}};
		const ::XZBBox world(0, 10, 20, 20);
		const auto net = arnis::highways::cars::CarNetwork::build(
				elements, 1.0, true, true, world,
				[](const auto &, int, int) { return 0; }, [](int, int) { return false; });
		// Both opposite-direction lanes are clipped to this tiny area. Building
		// their continuation map must not dereference an empty lane's end.
		UASSERT(net.segments().size() == 2);
		for (const auto &segment : net.segments())
			UASSERT(segment.points.empty() ||
					segment.arcs.size() == segment.points.size());
	}
};

TestFmMovingVehicles instance;

} // namespace
#endif
