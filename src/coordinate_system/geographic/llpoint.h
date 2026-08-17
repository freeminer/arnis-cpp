#pragma once
namespace arnis::geographic
{
class LLPoint
{
	double lat_, lng_;

public:
	LLPoint(double lat, double lng);
	double lat() const { return lat_; }
	double lng() const { return lng_; }
};
}
