#include "pool_viewer/panels/composition.hpp"
#include "pool_viewer/panel_layout.hpp"
#include "pool_viewer/panels/pose_math.hpp"
#include <imgui.h>
#include <imgui_internal.h>
#include <cstdio>
#include <cmath>

namespace pool::panels {
namespace {
// Numeric displays align to the right edge of their column. Editable values use
// normal text editing while active and the same right alignment at rest.
void numericText(const char *text, bool available = true) {
    ImGui::AlignTextToFramePadding();
    ImGui::SetCursorPosX(ImGui::GetCursorPosX() +
                         std::max(0.f, ImGui::GetContentRegionAvail().x - ImGui::CalcTextSize(text).x));
    if (available)
        ImGui::TextUnformatted(text);
    else
        ImGui::TextDisabled("%s", text);
}
void numericValue(float value, bool available) {
    char text[64];
    std::snprintf(text, sizeof(text), "%.2f", value);
    numericText(available ? text : "--", available);
}
bool targetInput(float *value) {
    const bool editing = ImGui::GetActiveID() == ImGui::GetID("##target");
    const auto color = ImGui::GetColorU32(ImGuiCol_Text);
    if (!editing)
        ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(0, 0, 0, 0));
    const bool changed = ImGui::InputFloat("##target", value, 0, 0, "%.2f");
    if (!editing) {
        ImGui::PopStyleColor();
        char text[64];
        std::snprintf(text, sizeof(text), "%.2f", *value);
        const auto lo = ImGui::GetItemRectMin(), hi = ImGui::GetItemRectMax();
        const auto padding = ImGui::GetStyle().FramePadding;
        auto *draw = ImGui::GetWindowDrawList();
        draw->PushClipRect(lo, hi, true);
        draw->AddText({std::max(lo.x + padding.x, hi.x - padding.x - ImGui::CalcTextSize(text).x), lo.y + padding.y},
                      color, text);
        draw->PopClipRect();
    }
    return changed;
}
class MotionPanel final : public Panel {
    std::shared_ptr<Motion> motion;
    std::function<void()> kill, drawOverlayControls;
    glm::vec3 position{0}, degrees{0};
    bool initialized = false, dirty = false, hasDive;
    Mode selected = Mode::Position;
    float diveZ;
    uint64_t revision = 0;
    void copy(const Pose &p) {
        position = glm::vec3(p[3]);
        degrees = glm::degrees(glm::eulerAngles(glm::quat_cast(p)));
        initialized = true;
    }

  public:
    explicit MotionPanel(const Binding &b)
        : motion(std::dynamic_pointer_cast<Motion>(b.provider)), kill(b.kill),
          drawOverlayControls(b.drawOverlayControls), hasDive(bool(b.options["dive_z"])),
          diveZ(b.options["dive_z"].as<float>(0)) {}
    void enableKillButton(ImVec2 size) {
        const auto s = motion ? motion->state() : MotionState{};
        const bool canKill =
            s.enabled || s.pending || s.blocked || s.competing || (s.observedKilled && !*s.observedKilled);
        ImGui::BeginDisabled(!motion || (!canKill && (!s.fresh || s.blocked || s.competing)));
        ImGui::PushStyleColor(ImGuiCol_Button, canKill ? ImVec4(.65f, .16f, .19f, 1) : ImVec4(.12f, .48f, .46f, 1));
        ImGui::PushStyleColor(ImGuiCol_ButtonHovered,
                              canKill ? ImVec4(.8f, .22f, .25f, 1) : ImVec4(.16f, .6f, .56f, 1));
        ImGui::PushStyleColor(ImGuiCol_ButtonActive, canKill ? ImVec4(.55f, .12f, .15f, 1) : ImVec4(.1f, .4f, .38f, 1));
        if (ImGui::Button(canKill ? "KILL###enable_kill" : "Enable###enable_kill", size)) {
            if (canKill)
                kill();
            else
                motion->enable();
        }
        ImGui::PopStyleColor(3);
        ImGui::EndDisabled();
    }
    void toolbar() override {
        pool::sameLineIfFits(90);
        enableKillButton({90, ImGui::GetFrameHeight()});
    }
    void pinned() override {
        const auto s = motion ? motion->state() : MotionState{};
        enableKillButton({ImGui::GetContentRegionAvail().x, std::max(36.f, ImGui::GetFrameHeight())});
        if (!motion)
            ImGui::TextDisabled("Preview / controls disconnected");
        else
            ImGui::TextColored(s.observedKilled.value_or(true) ? ImVec4(1, .65, .4, 1) : ImVec4(.3, 1, .65, 1),
                               "Robot: %s",
                               s.observedKilled ? (*s.observedKilled ? "killed" : "enabled") : "state unknown");
    }
    void draw() override {
        const auto s = motion ? motion->state() : MotionState{};
        if (s.fresh && (!initialized || (!dirty && !s.hasCommand)))
            copy(s.actual);
        if (s.revision != revision) {
            if (!dirty && s.hasCommand)
                copy(s.commanded);
            revision = s.revision;
        }
        if (!s.pending)
            selected = s.mode == Mode::Feedforward ? Mode::Feedforward : Mode::Position;
        ImGui::TextDisabled("Frame: %s / m, deg", s.frame.c_str());
        ImGui::TextWrapped("%s", s.message.c_str());
        const bool unavailable = !motion || !s.enabled || !s.fresh || s.pending || s.blocked || s.competing;
        ImGui::BeginDisabled(unavailable);
        const float modeWidth = (ImGui::GetContentRegionAvail().x - ImGui::GetStyle().ItemSpacing.x) * .5f;
        for (const auto choice : {Mode::Position, Mode::Feedforward}) {
            if (choice == Mode::Feedforward)
                ImGui::SameLine();
            ImGui::BeginDisabled(choice == Mode::Feedforward && !s.supportsFeedforward);
            if (s.mode == choice)
                ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(.12f, .48f, .46f, 1));
            if (ImGui::Button(choice == Mode::Position ? "Position" : "Feedforward", {modeWidth, 0})) {
                selected = choice;
                motion->activate(choice, s.actual);
                copy(s.actual);
                dirty = false;
            }
            if (s.mode == choice)
                ImGui::PopStyleColor();
            ImGui::EndDisabled();
        }
        if (s.mode == Mode::Feedforward)
            ImGui::TextWrapped("Feedforward controller mode; pose feedback is disabled.");
        ImGui::EndDisabled();
        ImGui::PushStyleVar(ImGuiStyleVar_CellPadding, ImVec2(2, 2));
        if (ImGui::BeginTable("pose", 5, ImGuiTableFlags_SizingStretchProp | ImGuiTableFlags_RowBg)) {
            ImGui::TableSetupColumn("", ImGuiTableColumnFlags_WidthFixed, 36);
            ImGui::TableSetupColumn("Actual");
            ImGui::TableSetupColumn("Commanded");
            ImGui::TableSetupColumn("Error");
            ImGui::TableSetupColumn("Target", ImGuiTableColumnFlags_WidthFixed, 66);
            ImGui::TableNextRow();
            for (const char *heading : {"", "Actual", "Commanded", "Error", "Target"}) {
                ImGui::TableNextColumn();
                ImGui::TextDisabled("%s", heading);
                if (std::string(heading) == "Error" && ImGui::IsItemHovered())
                    ImGui::SetTooltip("Commanded minus actual / angles wrapped to [-180, 180] degrees");
            }
            const auto actualAngles = glm::degrees(glm::eulerAngles(glm::quat_cast(s.actual)));
            const auto sentAngles = glm::degrees(glm::eulerAngles(glm::quat_cast(s.commanded)));
            const char *names[] = {"X", "Y", "Z", "Roll", "Pitch", "Yaw"};
            for (int i = 0; i < 6; ++i) {
                ImGui::PushID(i);
                ImGui::TableNextRow();
                ImGui::TableNextColumn();
                ImGui::AlignTextToFramePadding();
                ImGui::TextUnformatted(names[i]);
                ImGui::TableNextColumn();
                numericValue(i < 3 ? s.actual[3][i] : actualAngles[i - 3], s.fresh);
                ImGui::TableNextColumn();
                numericValue(i < 3 ? s.commanded[3][i] : sentAngles[i - 3], s.hasCommand);
                ImGui::TableNextColumn();
                const float error = i < 3 ? s.commanded[3][i] - s.actual[3][i]
                                          : std::remainder(sentAngles[i - 3] - actualAngles[i - 3], 360.f);
                numericValue(error, s.fresh && s.hasCommand);
                ImGui::TableNextColumn();
                ImGui::SetNextItemWidth(-1);
                ImGui::BeginDisabled(!motion || !s.fresh || s.pending || s.blocked);
                if (targetInput(i < 3 ? &position[i] : &degrees[i - 3]))
                    dirty = true;
                ImGui::EndDisabled();
                ImGui::PopID();
            }
            ImGui::EndTable();
        }
        ImGui::PopStyleVar();
        ImGui::PushStyleVar(ImGuiStyleVar_FramePadding, ImVec2(4, ImGui::GetStyle().FramePadding.y));
        const float actionsWidth = ImGui::CalcTextSize("Current").x + ImGui::CalcTextSize("Command").x +
                                   4 * ImGui::GetStyle().FramePadding.x + ImGui::GetStyle().ItemSpacing.x +
                                   (hasDive ? ImGui::CalcTextSize("Dive in place").x +
                                                  2 * ImGui::GetStyle().FramePadding.x + ImGui::GetStyle().ItemSpacing.x
                                            : 0);
        const float extraWidth = std::max(0.f, ImGui::GetContentRegionAvail().x - actionsWidth) / (hasDive ? 3 : 2);
        const auto actionSize = [&](const char *label) {
            return ImVec2(ImGui::CalcTextSize(label).x + 2 * ImGui::GetStyle().FramePadding.x + extraWidth,
                          std::max(40.f, ImGui::GetFrameHeight()));
        };
        ImGui::BeginDisabled(!motion || !s.fresh || s.pending || s.blocked);
        if (ImGui::Button("Current", actionSize("Current"))) {
            copy(s.actual);
            dirty = true;
        }
        ImGui::EndDisabled();
        ImGui::SameLine();
        ImGui::BeginDisabled(unavailable);
        const bool finite = std::isfinite(position.x + position.y + position.z + degrees.x + degrees.y + degrees.z);
        ImGui::BeginDisabled(!finite);
        if (ImGui::Button("Command", actionSize("Command"))) {
            motion->activate(selected, pool::rpyPose(position, glm::radians(degrees)));
            dirty = false;
        }
        ImGui::EndDisabled();
        if (hasDive)
            ImGui::SameLine();
        if (hasDive && ImGui::Button("Dive in place", actionSize("Dive in place"))) {
            copy(s.actual);
            position.z = diveZ;
            degrees.x = degrees.y = 0;
            selected = Mode::Position;
            motion->activate(selected, pool::rpyPose(position, glm::radians(degrees)));
            dirty = false;
        }
        ImGui::EndDisabled();
        ImGui::PopStyleVar();
        ImGui::TextDisabled("%s", dirty ? "Target edited / not sent" : "Drag an axis or ring in the pool view");
        if (drawOverlayControls)
            drawOverlayControls();
    }
};
} // namespace
void registerMotionPanel(Registry &r) {
    r.panels.emplace("motion", ViewFactory<Panel>{Kind::Motion,
                                                  [](const YAML::Node &n) {
                                                      keys(n, {"dive_z", "dive_max_depth_z"}, "motion panel");
                                                      if (n["dive_max_depth_z"])
                                                          required(n, {"dive_z"});
                                                      for (const char *key : {"dive_z", "dive_max_depth_z"})
                                                          if (n[key] && !std::isfinite(n[key].as<float>()))
                                                              throw std::invalid_argument("nonfinite dive setting");
                                                  },
                                                  [](const Binding &b) { return std::make_unique<MotionPanel>(b); }});
}
} // namespace pool::panels
