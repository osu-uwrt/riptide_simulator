#include "pool_viewer/panels/composition.hpp"
#include <imgui.h>
#include <cstdio>
namespace pool::panels {
namespace {
class MappingPanel final : public Panel {
    std::shared_ptr<Mapping> mapping;
    char parent[256]{}, child[256]{}, target[256]{};
    int samples = 10;
    bool locked = false, initialized = false;

  public:
    explicit MappingPanel(const Binding &b) : mapping(std::dynamic_pointer_cast<Mapping>(b.provider)) {
        std::snprintf(parent, sizeof(parent), "%s", b.options["parent_frame"].as<std::string>("").c_str());
        std::snprintf(child, sizeof(child), "%s", b.options["tag_frame"].as<std::string>("").c_str());
        samples = b.options["samples"].as<int>(10);
    }
    void draw() override {
        auto s = mapping ? mapping->state() : MappingState{};
        ImGui::SeparatorText("Tag calibration");
        ImGui::BeginDisabled(s.calibrating);
        ImGui::TextUnformatted("Parent frame");
        ImGui::SetNextItemWidth(-1);
        ImGui::InputText("##parent", parent, sizeof(parent));
        ImGui::TextUnformatted("Tag frame");
        ImGui::SetNextItemWidth(-1);
        ImGui::InputText("##child", child, sizeof(child));
        ImGui::TextUnformatted("Samples");
        ImGui::SetNextItemWidth(-1);
        ImGui::InputInt("##samples", &samples);
        ImGui::EndDisabled();
        ImGui::BeginDisabled(!mapping || (!s.calibrating && (!s.calibrationReady || samples < 1 || samples > 65535 ||
                                                             !parent[0] || !child[0] || std::string(parent) == child)));
        if (ImGui::Button(s.calibrating ? "Cancel tag cal" : "Tag cal", {-1, 36})) {
            if (s.calibrating)
                mapping->cancelCalibration();
            else
                mapping->calibrate(parent, child, samples);
        }
        ImGui::EndDisabled();
        if (s.calibrating)
            ImGui::Text("Samples: %u / %d", s.samples, samples);
        ImGui::TextWrapped("%s", mapping ? s.calibrationMessage.c_str() : "Preview / mapping disconnected");
        ImGui::SeparatorText("Mapping target");
        if (s.fresh) {
            ImGui::TextWrapped("Current: %s", s.target.empty() ? "Automatic" : s.target.c_str());
            ImGui::TextUnformatted(s.locked ? "Map locked" : "Map unlocked");
            if (!initialized) {
                std::snprintf(target, sizeof(target), "%s", s.target.c_str());
                locked = s.locked;
                initialized = true;
            }
        } else
            ImGui::TextDisabled("Mapping status unavailable / stale");
        ImGui::SetNextItemWidth(-1);
        ImGui::InputTextWithHint("##target", "Target object (empty = automatic)", target, sizeof(target));
        ImGui::Checkbox("Lock map", &locked);
        ImGui::BeginDisabled(!mapping || !s.targetReady || s.settingTarget);
        if (ImGui::Button(s.settingTarget ? "Setting target..." : "Set mapping target", {-1, 36}))
            mapping->setTarget(target, locked);
        ImGui::EndDisabled();
        if (!s.targetMessage.empty())
            ImGui::TextWrapped("%s", s.targetMessage.c_str());
        ImGui::BeginDisabled(!mapping || !s.resetReady || s.resetting);
        if (ImGui::Button(s.resetting ? "Resetting..." : "Reset mapping", {-1, 36}))
            mapping->reset();
        ImGui::EndDisabled();
        if (!s.resetMessage.empty())
            ImGui::TextWrapped("%s", s.resetMessage.c_str());
    }
};
} // namespace
void registerMappingPanel(Registry &r) {
    r.panels.emplace("mapping", ViewFactory<Panel>{Kind::Mapping,
                                                   [](const YAML::Node &n) {
                                                       keys(n, {"parent_frame", "tag_frame", "samples"},
                                                            "mapping panel");
                                                       positive(n, "samples", 10, 65535);
                                                       (void)n["samples"].as<int>(10);
                                                   },
                                                   [](const Binding &b) { return std::make_unique<MappingPanel>(b); }});
}
} // namespace pool::panels
