#include "bedrock_addon.h"

#include "element_processing/cars.h"
#include "element_processing/trains.h"

#include <iterator>

namespace arnis::bedrock_addon
{

std::optional<VehiclePayload> vehicles(const highways::cars::CarNetwork *cars,
		const trains::TrainRouteNetwork *trains,
		const std::function<std::optional<int>(std::uint64_t, int, int)> &rail_y)
{
	const bool has_cars = cars && cars->car_count() > 0;
	nlohmann::json data{{"cars", nullptr}, {"trains", nullptr}};
	std::vector<Body> bodies;
	bool has_trains = false;
	if (has_cars) {
		data["cars"] = cars->bedrock_data();
		bodies = cars->bedrock_bodies();
	}
	if (trains && !trains->is_empty()) {
		auto [train_data, train_bodies] = trains->bedrock_data(rail_y);
		const auto &services = train_data["services"];
		has_trains = services.is_array() && !services.empty();
		if (has_trains) {
			data["trains"] = std::move(train_data);
			bodies.insert(bodies.end(), std::make_move_iterator(train_bodies.begin()),
					std::make_move_iterator(train_bodies.end()));
		}
	}
	if (!has_cars && !has_trains)
		return std::nullopt;
	return VehiclePayload{std::move(data), std::move(bodies)};
}

} // namespace arnis::bedrock_addon
