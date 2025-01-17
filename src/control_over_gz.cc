#include <gz_gep_tools/control_over_gz.hh>
#include <gz/msgs.hh>
#include <gz/transport.hh>

namespace gz_transport_hw_tools {

ControlOverGz::ControlOverGz(std::string &world_prefix)
    : world_prefix_(world_prefix) {
  control_gzsim_ = world_prefix+std::string("/control");  
}

bool ControlOverGz::Reset()
{
  gz::msgs::WorldControl req_world_ctrl;
  gz::msgs::WorldReset req_world_reset;
  gz::msgs::Boolean rep_bool;
  unsigned int timeout = 5000;
  
  /// Set reset for everything
  req_world_reset.set_all(true);
  /// Set the world reset inside the world control reset.
  req_world_ctrl.set_allocated_reset(&req_world_reset);
  /// Do request.
  bool result;
  node_.Request( control_gzsim_, req_world_ctrl, timeout, rep_bool, result);

  return result;
}

bool ControlOverGz::SendPauseRequest(bool abool)
{
  gz::msgs::WorldControl req_world_ctrl;
  gz::msgs::Boolean rep_bool;
  unsigned int timeout = 5000;
  
  /// Set the world reset inside the world control reset.
  req_world_ctrl.set_pause(abool);
  
  /// Do request.
  bool result;
  node_.Request( control_gzsim_, req_world_ctrl, timeout, rep_bool, result);
  return result;
}

bool ControlOverGz::Pause() { return SendPauseRequest(true); }

bool ControlOverGz::Start() { return SendPauseRequest(false); }



}
