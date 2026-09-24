#pragma once
#include "pool_viewer/panels/composition.hpp"

namespace pool::panels {
// Application boundary: the composition, panels and capability contracts have
// no dependency on the ROS implementation hidden behind this owner.
class RosProviders {
  public:
    RosProviders();
    ~RosProviders();
    void registerFactories(Registry &);
    void start();
    void stop();

  private:
    struct Impl;
    std::shared_ptr<Impl> impl;
};
} // namespace pool::panels
