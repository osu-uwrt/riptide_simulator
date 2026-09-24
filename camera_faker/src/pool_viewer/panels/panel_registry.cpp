#include "pool_viewer/panels/composition.hpp"
namespace pool::panels {
void registerMotionPanel(Registry &);
void registerAutonomyPanel(Registry &);
void registerPoseGizmo(Registry &);
void registerMappingPanel(Registry &);
void registerActuatorPanel(Registry &);
void registerRunPanel(Registry &);
void registerSimulationPanel(Registry &);
void registerPanels(Registry &registry) {
    registerSimulationPanel(registry);
    registerRunPanel(registry);
    registerActuatorPanel(registry);
    registerMappingPanel(registry);
    registerMotionPanel(registry);
    registerAutonomyPanel(registry);
    registerPoseGizmo(registry);
}
} // namespace pool::panels
