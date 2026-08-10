#pragma once

#include <gz/sim/components/Component.hh>
#include <gz/sim/components/Factory.hh>
#include <gz/sim/config.hh>

#include <ostream>
#include <istream>

// Shared between the uvdar_led and uvdar_cam plugin libraries: UvLed writes
// its current blink state onto its own entity each step, and OcclusionCheck
// reads it back from the ECM when deciding whether to include an LED in the
// published points. This keeps the two plugins in lockstep with the physics
// update without needing a transport link between them.
namespace uvdar_gazebo_plugin
{
namespace components
{

struct LedBlinkStateData
{
  bool on = false;
  int signal_id = -1;
};

inline std::ostream &operator<<(std::ostream &_out, const LedBlinkStateData &_data)
{
  _out << _data.on << " " << _data.signal_id;
  return _out;
}

inline std::istream &operator>>(std::istream &_in, LedBlinkStateData &_data)
{
  _in >> _data.on >> _data.signal_id;
  return _in;
}

using LedBlinkState = gz::sim::components::Component<
    LedBlinkStateData, class LedBlinkStateTag>;

GZ_SIM_REGISTER_COMPONENT(
    "uvdar_gazebo_plugin.components.LedBlinkState", LedBlinkState)

}  // namespace components
}  // namespace uvdar_gazebo_plugin
