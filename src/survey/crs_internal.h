#pragma once

#include <utility>

namespace arnis::survey::crs::detail
{
std::pair<double, double> lambert93_forward(double lat, double lon);
std::pair<double, double> lambert93_inverse(double x, double y);
std::pair<double, double> lest97_forward(double lat, double lon);
std::pair<double, double> lest97_inverse(double x, double y);
std::pair<double, double> rd_forward(double lat, double lon);
std::pair<double, double> rd_inverse(double x, double y);
std::pair<double, double> lv95_forward(double lat, double lon);
std::pair<double, double> lv95_inverse(double x, double y);
std::pair<double, double> luref_forward(double lat, double lon);
std::pair<double, double> luref_inverse(double x, double y);
std::pair<double, double> mercator_forward(double lat, double lon);
std::pair<double, double> mercator_inverse(double x, double y);
} // namespace arnis::survey::crs::detail
