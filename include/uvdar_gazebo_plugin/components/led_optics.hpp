#pragma once

#include <gz/sim/components/Component.hh>
#include <gz/sim/components/Factory.hh>
#include <gz/sim/config.hh>

#include <istream>
#include <ostream>

// Optical properties are stored on each LED link so every camera observes the
// same emitter radiant power, direction and radiation pattern. The emission
// axis is expressed in the LED link frame; cameras transform it to world frame.
namespace uvdar_gazebo_plugin
{
namespace components
{

struct LedOpticsData
{
  double power_w = 1.0;
  double lambertian_order = 1.0;
  double axis_x = 0.0;
  double axis_y = 0.0;
  double axis_z = 1.0;
};

inline std::ostream &operator<<(std::ostream &_out, const LedOpticsData &_data)
{
  _out << _data.power_w << " " << _data.lambertian_order << " "
       << _data.axis_x << " " << _data.axis_y << " " << _data.axis_z;
  return _out;
}

inline std::istream &operator>>(std::istream &_in, LedOpticsData &_data)
{
  _in >> _data.power_w >> _data.lambertian_order
      >> _data.axis_x >> _data.axis_y >> _data.axis_z;
  return _in;
}

using LedOptics = gz::sim::components::Component<
    LedOpticsData, class LedOpticsTag>;

GZ_SIM_REGISTER_COMPONENT(
    "uvdar_gazebo_plugin.components.LedOptics", LedOptics)

}  // namespace components
}  // namespace uvdar_gazebo_plugin
