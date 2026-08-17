#pragma once
#include "geographic/llbbox.h"
#include "cartesian/xzbbox/rectangle.h"
#include <utility>
#include <stdexcept>
namespace arnis::coordinate_system
{
double lon_to_mercator_x(double);
double lat_to_mercator_y(double);
double mercator_x_to_lon(double);
double mercator_y_to_lat(double);
class CoordTransformer
{
	double len_lat_, len_lng_, sx_, sz_, min_lat_, min_lng_;
	bool mercator_ = false;
	double origin_lat_ = 0, origin_lng_ = 0, scale_ = 1, cos_ref_ = 1, z_offset_ = 0;

public:
	CoordTransformer(double la, double ln, double sx, double sz, double mla, double mln) :
			len_lat_(la), len_lng_(ln), sx_(sx), sz_(sz), min_lat_(mla), min_lng_(mln)
	{
	}
	double scale_factor_x() const { return sx_; }
	double scale_factor_z() const { return sz_; }
	::arnis::cartesian::XZPoint transform_point(::arnis::geographic::LLPoint p) const;
	static std::pair<CoordTransformer, ::arnis::cartesian::XZBBoxRect> llbbox_to_xzbbox(
			const ::arnis::geographic::LLBBox &, double scale);
	static std::pair<CoordTransformer, ::arnis::cartesian::XZBBoxRect> with_web_mercator(
			const ::arnis::geographic::LLBBox &, double scale);
};
double lat_distance(double a, double b);
double lon_distance(double lat, double a, double b);
std::pair<double, double> geo_distance(
		::arnis::geographic::LLPoint, ::arnis::geographic::LLPoint);
}
