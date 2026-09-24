#include "pool_viewer/panels/composition.hpp"
#include <imgui.h>
namespace pool::panels {
namespace {
class ActuatorPanel final : public Panel {
    std::shared_ptr<Actuators> actuators;

  public:
    explicit ActuatorPanel(const Binding &b) : actuators(std::dynamic_pointer_cast<Actuators>(b.provider)) {}
    void draw() override {
        auto s = actuators ? actuators->state() : ActuatorState{};
        ImGui::TextWrapped("%s", actuators ? s.message.c_str() : "Preview / actuators disconnected");
        for (const auto &action : s.actions) {
            ImGui::PushID(action.id.c_str());
            ImGui::BeginDisabled(!action.available);
            if (ImGui::Button(action.label.c_str(), {-1, 36}))
                actuators->command(action.id);
            ImGui::EndDisabled();
            ImGui::PopID();
        }
        if (!s.readings.empty())
            ImGui::SeparatorText("Status");
        for (const auto &reading : s.readings)
            ImGui::TextWrapped("%s: %s", reading.first.c_str(), reading.second.c_str());
    }
};
} // namespace
void registerActuatorPanel(Registry &r) {
    r.panels.emplace("actuators",
                     ViewFactory<Panel>{Kind::Actuators, [](const YAML::Node &n) { keys(n, {}, "actuators panel"); },
                                        [](const Binding &b) { return std::make_unique<ActuatorPanel>(b); }});
}
} // namespace pool::panels
