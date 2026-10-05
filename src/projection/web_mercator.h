#pragma once
#include <memory>
#include <string>
#include <utility>
namespace arnis::projection
{
enum class ProjectionKind
{
	WebMercator,
	Local
};
std::string to_string(ProjectionKind);
ProjectionKind projection_kind_from_string(const std::string &);

class Projection
{
public:
	virtual ~Projection() = default;
	virtual std::pair<double, double> forward(double lat, double lon) const = 0;
	virtual std::pair<double, double> inverse(double x, double z) const = 0;
};

class WebMercatorProjection : public Projection
{
	double origin_lat_, origin_lon_, scale_, z_offset_;

public:
	WebMercatorProjection(double origin_lat, double origin_lon, double scale = 1.0);
	std::pair<double, double> forward(double lat, double lon) const override;
	std::pair<double, double> inverse(double x, double z) const override;
	double x_for_lon(double longitude) const;
	double z_for_lat(double latitude) const;
	double lon_for_x(double x) const;
	double lat_for_z(double z) const;
};

// Round near-integral projected edges consistently before applying floor/ceil.
int snap_edge(double value, bool ceil);
}
