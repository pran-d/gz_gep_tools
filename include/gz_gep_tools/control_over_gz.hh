/// Standard includes
#include <vector>
#include <string>

/// GZ includes
#include <gz/msgs.hh>
#include <gz/transport.hh>

namespace gz_transport_hw_tools {

class ControlOverGz {
 public:

  /// Provides the gazebo name.
  ControlOverGz(std::string &world_prefix);

  /// Pause Gazebosim
  bool Pause();

  /// Start Gazebo
  bool Start();
  
  /// Reset Gazebosim
  bool Reset();
  
 protected:

  /// Send a Pause request
  bool SendPauseRequest(bool abool);
  
  /// World prefix 
  std::string world_prefix_;

  /// list of services
  std::string control_gzsim_;

  /// GZ node
  gz::transport::Node node_;

  
};

}

