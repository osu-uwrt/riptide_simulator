#include "pool_viewer/panels/composition.hpp"
#include "pool_viewer/panel_layout.hpp"
#include <imgui.h>
#include <cmath>
#include <algorithm>
namespace pool::panels {
namespace {
class SimulationPanel final : public Panel {
    std::shared_ptr<Simulation> simulation;
    float draft = 1;
    bool dirty = false;

  public:
    explicit SimulationPanel(const Binding &b) : simulation(std::dynamic_pointer_cast<Simulation>(b.provider)) {}
    void toolbar() override {
        pool::sameLineIfFits(ImGui::CalcTextSize("Simulation settings").x + 2 * ImGui::GetStyle().FramePadding.x);
        if (ImGui::Button("Simulation settings"))
            ImGui::OpenPopup("simulation_settings");
        // A known width and anchor prevent wrapped text from causing a first-frame
        // auto-fit/reposition jump. Keep the popup inside the application viewport.
        const auto *viewport = ImGui::GetMainViewport();
        const auto button = ImGui::GetItemRectMin();
        const auto bottom = ImGui::GetItemRectMax().y;
        const float width = std::min(340.f, viewport->WorkSize.x - 16.f);
        const float x =
            std::clamp(button.x, viewport->WorkPos.x + 8.f, viewport->WorkPos.x + viewport->WorkSize.x - width - 8.f);
        const float y = bottom + ImGui::GetStyle().ItemSpacing.y;
        ImGui::SetNextWindowPos({x, y});
        ImGui::SetNextWindowSize({width, 0});
        ImGui::SetNextWindowSizeConstraints(
            {width, 0}, {width, std::max(1.f, viewport->WorkPos.y + viewport->WorkSize.y - y - 8.f)});
        if (ImGui::BeginPopup("simulation_settings")) {
            draw();
            ImGui::EndPopup();
        }
    }
    void draw() override {
        const auto s = simulation ? simulation->state() : SimulationState{};
        const bool paused = s.rate == 0;
        if (!dirty)
            draft = paused ? s.resumeRate : s.rate;
        ImGui::TextWrapped("%s", simulation ? s.message.c_str() : "Preview / simulator disconnected");
        if (s.connected) {
            ImGui::BeginDisabled(paused);
            ImGui::Text("Selected speed: %.2fx", paused ? s.resumeRate : s.rate);
            ImGui::EndDisabled();
        }
        ImGui::BeginDisabled(!simulation || !s.connected || s.pending);
        ImGui::BeginDisabled(paused);
        ImGui::SetNextItemWidth(180);
        if (ImGui::InputFloat("Speed", &draft, .25f, 1.f, "%.2fx"))
            dirty = true;
        ImGui::BeginDisabled(!std::isfinite(draft) || draft <= 0 || draft > s.maxRate);
        if (ImGui::Button(s.pending ? "Applying..." : "Apply", {90, 36})) {
            simulation->setRate(draft);
            dirty = false;
        }
        ImGui::EndDisabled();
        ImGui::EndDisabled();
        ImGui::SameLine();
        if (ImGui::Button(paused ? "Resume" : "Pause", {80, 36})) {
            simulation->setPaused(!paused);
            dirty = false;
        }
        ImGui::SameLine();
        ImGui::BeginDisabled(paused || s.maxRate < 1);
        if (ImGui::Button("1x", {60, 36})) {
            simulation->setRate(1);
            dirty = false;
        }
        ImGui::EndDisabled();
        ImGui::EndDisabled();
        ImGui::Text("Speed range: >0 to %.0fx", s.maxRate);
        ImGui::TextDisabled("Pause preserves the selected speed.");
        ImGui::Separator();
        ImGui::BeginDisabled(!simulation || !s.syncReady || s.operationPending);
        if (ImGui::Button("Sync sim", {130, 36}))
            simulation->sync();
        if (ImGui::IsItemHovered())
            ImGui::SetTooltip(
                "Move simulation to the estimated robot pose. Keep velocities and the estimate unchanged.");
        ImGui::EndDisabled();
        ImGui::SameLine();
        ImGui::BeginDisabled(!simulation || !s.resetReady || s.operationPending);
        if (ImGui::Button("Reset sim", {130, 36}))
            simulation->reset();
        if (ImGui::IsItemHovered())
            ImGui::SetTooltip(
                "Reset simulation to its start pose, at rest, with thrusters cleared; re-seed the estimator.");
        ImGui::EndDisabled();
        if (!s.operationMessage.empty())
            ImGui::TextWrapped("%s", s.operationMessage.c_str());
    }
};
} // namespace
void registerSimulationPanel(Registry &registry) {
    registry.panels.emplace(
        "simulation", ViewFactory<Panel>{Kind::Simulation, [](const YAML::Node &n) { keys(n, {}, "simulation tool"); },
                                         [](const Binding &b) { return std::make_unique<SimulationPanel>(b); }});
}
} // namespace pool::panels
