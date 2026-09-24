#include "pool_viewer/panels/composition.hpp"
#include <imgui.h>
#include <filesystem>
#include <cfloat>
namespace pool::panels {
namespace {
class AutonomyPanel final : public Panel {
    std::shared_ptr<Autonomy> mission;
    std::function<bool()> mayStart;
    std::string selected;
    ImGuiTextFilter filter;

  public:
    explicit AutonomyPanel(const Binding &b)
        : mission(std::dynamic_pointer_cast<Autonomy>(b.provider)), mayStart(b.mayStart) {}
    void draw() override {
        const auto s = mission ? mission->state() : MissionState{};
        ImGui::TextWrapped("%s", mission ? s.message.c_str() : "Preview / autonomy disconnected");
        ImGui::BeginDisabled(!mission || !s.connected || s.busy || s.pending);
        float labelWidth = ImGui::CalcTextSize("Select a tree").x;
        for (const auto &tree : s.trees)
            labelWidth = std::max(labelWidth, ImGui::CalcTextSize(std::filesystem::path(tree).filename().c_str()).x);
        const float popupWidth =
            std::min(ImGui::GetIO().DisplaySize.x - 36,
                     labelWidth + std::max(2 * ImGui::GetStyle().WindowPadding.x,
                                           2 * ImGui::GetStyle().FramePadding.x + ImGui::GetFrameHeight()));
        ImGui::SetNextItemWidth(std::min(ImGui::GetContentRegionAvail().x, popupWidth));
        ImGui::SetNextWindowSizeConstraints({popupWidth, 0}, {popupWidth, FLT_MAX});
        if (ImGui::BeginCombo("##tree", selected.empty() ? "Select a tree"
                                                         : std::filesystem::path(selected).filename().c_str())) {
            ImGui::SetNextItemWidth(-1);
            if (ImGui::InputTextWithHint("##tree_search", "Search trees", filter.InputBuf,
                                         IM_ARRAYSIZE(filter.InputBuf)))
                filter.Build();
            for (const auto &tree : s.trees)
                if (filter.PassFilter(tree.c_str())) {
                    ImGui::PushID(tree.c_str());
                    if (ImGui::Selectable(std::filesystem::path(tree).filename().c_str(), selected == tree))
                        selected = tree;
                    if (ImGui::IsItemHovered())
                        ImGui::SetTooltip("%s", tree.c_str());
                    ImGui::PopID();
                }
            ImGui::EndCombo();
        }
        if (ImGui::Button(s.refreshing ? "Refreshing..." : "Refresh"))
            mission->refresh();
        ImGui::SameLine();
        ImGui::BeginDisabled(selected.empty() || !mayStart());
        if (ImGui::Button("Start"))
            mission->start(selected);
        ImGui::EndDisabled();
        ImGui::EndDisabled();
        ImGui::SameLine();
        ImGui::BeginDisabled(!mission || !s.busy);
        if (ImGui::Button("Stop"))
            mission->stop();
        ImGui::EndDisabled();
        if (!s.activeTree.empty())
            ImGui::TextWrapped("Tree: %s", std::filesystem::path(s.activeTree).filename().c_str());
        ImGui::SeparatorText(s.stackStale ? "Execution stack / stale"
                             : s.busy     ? "Execution stack"
                                          : "Last execution stack");
        if (s.stack.empty())
            ImGui::TextDisabled(s.busy ? "Waiting for stack..." : "No stack received");
        ImGui::BeginChild("stack",
                          {0, std::max(80.f, ImGui::GetContentRegionAvail().y - ImGui::GetStyle().ItemSpacing.y)},
                          ImGuiChildFlags_None);
        for (size_t i = 0; i < s.stack.size(); ++i) {
            ImGui::TextColored(i + 1 == s.stack.size() ? ImVec4(.3, .9, .8, 1) : ImVec4(.65, .72, .77, 1), "%02zu",
                               i + 1);
            ImGui::SameLine();
            ImGui::TextWrapped("%s", s.stack[i].c_str());
        }
        ImGui::EndChild();
    }
};
} // namespace
void registerAutonomyPanel(Registry &r) {
    r.panels.emplace("autonomy",
                     ViewFactory<Panel>{Kind::Autonomy, [](const YAML::Node &n) { keys(n, {}, "autonomy panel"); },
                                        [](const Binding &b) { return std::make_unique<AutonomyPanel>(b); }});
}
} // namespace pool::panels
