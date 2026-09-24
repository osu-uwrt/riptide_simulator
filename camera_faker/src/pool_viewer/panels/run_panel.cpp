#include "pool_viewer/panels/composition.hpp"
#include <imgui.h>
#include "pool_viewer/panel_layout.hpp"
#include <cmath>
#include <algorithm>
#include <cstdio>
namespace pool::panels {
namespace {
class RunPanel final : public Panel {
    std::shared_ptr<Run> run;
    YAML::Node schema, options;
    std::function<void(const std::string &)> focus;
    bool details = false, focusDetails = false, enabled = true;
    float manualPoints = 0;
    std::string title;

  public:
    explicit RunPanel(const Binding &b)
        : run(std::dynamic_pointer_cast<Run>(b.provider)), focus(b.focus), details(b.showWindow) {
        auto profile = b.options["profile"].as<std::string>("");
        if (b.documents.count(profile)) {
            const auto document = b.documents.at(profile);
            enabled = document["scoring_enabled"].as<bool>(true);
            if (document["ui"])
                schema = YAML::Clone(document["ui"]);
        }
        title = schema["title"].as<std::string>("Run scorecard");
        for (const auto &option : schema["run_options"])
            options[option["key"].as<std::string>()] = YAML::Clone(option["default"]);
    }
    void toolbar() override {
        const float width = ImGui::CalcTextSize("Run tracking").x + 2 * ImGui::GetStyle().FramePadding.x;
        pool::sameLineIfFits(width);
        if (ImGui::Button("Run tracking")) {
            details = !details;
            focusDetails = details;
        }
    }
    void draw() override {
        controls(true);
    }
    void drawWindows() override {
        if (!details)
            return;
        const auto *viewport = ImGui::GetMainViewport();
        const ImVec2 available(std::max(1.f, viewport->WorkSize.x - 24.f), std::max(1.f, viewport->WorkSize.y - 24.f));
        // Measure the complete scorecard on opening, then allow normal resizing.
        ImGui::SetNextWindowSize({std::min(720.f, available.x), 0}, ImGuiCond_Appearing);
        ImGui::SetNextWindowSizeConstraints({std::min(360.f, available.x), std::min(120.f, available.y)}, available);
        ImGui::SetNextWindowPos(
            {viewport->WorkPos.x + viewport->WorkSize.x * .5f, viewport->WorkPos.y + viewport->WorkSize.y * .5f},
            ImGuiCond_Appearing, {.5f, .5f});
        if (focusDetails) {
            ImGui::SetNextWindowFocus();
            ImGui::SetNextWindowCollapsed(false);
            focusDetails = false;
        }
        // Include the instance's ID so multiple run panels can have separate windows.
        const auto name = title + "###scorecard_" + std::to_string(ImGui::GetID("scorecard"));
        if (ImGui::Begin(name.c_str(), &details)) {
            const auto pos = ImGui::GetWindowPos(), size = ImGui::GetWindowSize();
            if (!ImGui::IsWindowAppearing())
                ImGui::SetWindowPos({std::clamp(pos.x, viewport->WorkPos.x + 12.f,
                                                viewport->WorkPos.x + 12.f + std::max(0.f, available.x - size.x)),
                                     std::clamp(pos.y, viewport->WorkPos.y + 12.f,
                                                viewport->WorkPos.y + 12.f + std::max(0.f, available.y - size.y))});
            controls(false);
            taskStatus();
            scorecard();
        }
        ImGui::End();
    }

  private:
    void controls(bool showDetails) {
        auto s = run ? run->state() : RunState{};
        const auto score = s.score;
        const bool running = score["running"].as<bool>(false);
        const double seconds = score["elapsed"].as<double>(0);
        ImGui::Text("%02d:%04.1f  |  %.1f points", int(seconds) / 60, std::fmod(seconds, 60.),
                    score["total"].as<double>(0));
        if (showDetails && ImGui::Button("Detailed scorecard", {-1, 36}))
            details = focusDetails = true;
        ImGui::BeginDisabled(!run || !s.fresh || !enabled);
        ImGui::BeginDisabled(running);
        for (const auto &option : schema["run_options"]) {
            auto key = option["key"].as<std::string>(), label = option["label"].as<std::string>(),
                 type = option["type"].as<std::string>();
            ImGui::PushID(key.c_str());
            if (type == "bool") {
                bool value = options[key].as<bool>();
                if (ImGui::Checkbox(label.c_str(), &value))
                    options[key] = value;
            } else {
                ImGui::TextUnformatted(label.c_str());
                ImGui::SetNextItemWidth(-1);
                if (type == "number") {
                    float value = options[key].as<float>();
                    if (ImGui::InputFloat("##value", &value) && std::isfinite(value))
                        options[key] = std::clamp(value, option["min"].as<float>(-1e9f), option["max"].as<float>(1e9f));
                } else if (type == "choice") {
                    std::string selected = options[key].as<std::string>(), preview = selected;
                    for (const auto &choice : option["choices"])
                        if (choice["value"].as<std::string>() == selected)
                            preview = choice["label"].as<std::string>();
                    if (ImGui::BeginCombo("##value", preview.c_str())) {
                        for (const auto &choice : option["choices"])
                            if (ImGui::Selectable(choice["label"].as<std::string>().c_str(),
                                                  choice["value"].as<std::string>() == selected))
                                options[key] = YAML::Clone(choice["value"]);
                        ImGui::EndCombo();
                    }
                }
            }
            ImGui::PopID();
        }
        const float half = (ImGui::GetContentRegionAvail().x - ImGui::GetStyle().ItemSpacing.x) * .5f;
        if (ImGui::Button("Start run", {half, 36})) {
            auto cmd = YAML::Clone(options);
            cmd["action"] = "start";
            run->command(cmd);
        }
        if (ImGui::IsItemHovered())
            ImGui::SetTooltip(
                "Clear tasks and score, reload/disarm, and start a fresh timer. Vehicle position stays unchanged.");
        ImGui::EndDisabled();
        ImGui::SameLine();
        ImGui::BeginDisabled(!running);
        if (ImGui::Button("Stop run", {half, 36})) {
            YAML::Node cmd;
            cmd["action"] = "stop";
            run->command(cmd);
        }
        ImGui::EndDisabled();
        if (ImGui::Button("Reset run & tasks", {-1, 36}))
            run->reset();
        for (const auto &action : schema["actions"])
            if (ImGui::Button(action["label"].as<std::string>().c_str(), {-1, 36}))
                run->command(action["command"]);
        ImGui::EndDisabled();
        if (!run)
            ImGui::TextWrapped("Preview / run tracking disconnected");
        else if (!s.message.empty())
            ImGui::TextWrapped("%s", s.message.c_str());
        for (const auto &field : schema["status_fields"]) {
            auto value = score[field["key"].as<std::string>()];
            ImGui::TextWrapped("%s: %s", field["label"].as<std::string>().c_str(),
                               value && value.IsScalar() ? value.as<std::string>().c_str() : "Pending");
        }
        for (auto field : {"message", "ended_reason"})
            if (score[field] && score[field].IsScalar())
                ImGui::TextWrapped("%s", score[field].as<std::string>().c_str());
        ImGui::TextWrapped("Timer uses simulation time; pauses with physics. Stop is manual.");
    }
    void taskStatus() {
        const auto s = run ? run->state() : RunState{};
        if (!s.taskSummary.empty()) {
            ImGui::SeparatorText("Task status");
            ImGui::TextWrapped("%s", s.taskSummary.c_str());
        }
        if (!s.magnetTargets.empty())
            ImGui::SeparatorText("Magnet targets");
        for (const auto &target : s.magnetTargets)
            ImGui::TextColored(target.second ? ImVec4(.2f, 1.f, .3f, 1.f) : ImVec4(1.f, .3f, .25f, 1.f), "%s: %s",
                               target.first.c_str(), target.second ? "GREEN" : "RED");
        for (const auto &reading : s.simulationReadings)
            ImGui::TextWrapped("%s: %s", reading.first.c_str(), reading.second.c_str());
        if (!s.events.empty())
            ImGui::SeparatorText("Recent events");
        for (const auto &event : s.events)
            ImGui::TextWrapped("%s", event.c_str());
        if (schema["run_inspections"] && ImGui::TreeNode("Inspect scene")) {
            ImGui::BeginDisabled(!focus);
            for (const auto &item : schema["run_inspections"])
                if (ImGui::Button(item["label"].as<std::string>().c_str(), {-1, 36}))
                    focus(item["target"].as<std::string>());
            ImGui::EndDisabled();
            ImGui::TreePop();
        }
    }
    void scorecard() {
        auto s = run ? run->state() : RunState{};
        const auto score = s.score;
        ImGui::SeparatorText("Awards");
        if (ImGui::BeginTable("awards", 2, ImGuiTableFlags_RowBg | ImGuiTableFlags_BordersInnerH)) {
            ImGui::TableSetupColumn("Award", ImGuiTableColumnFlags_WidthStretch);
            ImGui::TableSetupColumn("Points", ImGuiTableColumnFlags_WidthFixed, 80);
            ImGui::TableHeadersRow();
            for (const auto &row : score["rows"]) {
                ImGui::TableNextRow(0, 30);
                ImGui::TableNextColumn();
                ImGui::AlignTextToFramePadding();
                ImGui::TextWrapped("%s", row["label"].as<std::string>().c_str());
                ImGui::TableNextColumn();
                ImGui::AlignTextToFramePadding();
                char number[64];
                std::snprintf(number, sizeof(number), "%.1f", row["points"].as<double>());
                ImGui::SetCursorPosX(ImGui::GetCursorPosX() +
                                     std::max(0.f, ImGui::GetContentRegionAvail().x - ImGui::CalcTextSize(number).x));
                ImGui::TextUnformatted(number);
            }
            ImGui::EndTable();
        }
        ImGui::Text("Manual adjustment: %.1f", score["adjustment"].as<double>(0));
        ImGui::Text("TOTAL: %.1f", score["total"].as<double>(0));
        for (const auto &field : schema["score_fields"]) {
            auto value = score[field["key"].as<std::string>()];
            if (value && value.IsScalar())
                ImGui::TextWrapped("%s: %s", field["label"].as<std::string>().c_str(), value.as<std::string>().c_str());
        }
        if (schema["manual_adjustment"].as<bool>(false)) {
            ImGui::Separator();
            ImGui::TextWrapped("Manual adjustment (signed points; replaces prior adjustment)");
            ImGui::SetNextItemWidth(-1);
            ImGui::InputFloat("##points", &manualPoints, 50, 100, "%.1f");
            ImGui::BeginDisabled(!run || !s.fresh || !std::isfinite(manualPoints));
            if (ImGui::Button("Apply adjustment", {-1, 36})) {
                YAML::Node cmd;
                cmd["action"] = "adjustment";
                cmd["points"] = manualPoints;
                run->command(cmd);
            }
            ImGui::EndDisabled();
        }
        if (schema["score_note"])
            ImGui::TextWrapped("%s", schema["score_note"].as<std::string>().c_str());
    }
};
} // namespace
void registerRunPanel(Registry &r) {
    r.panels.emplace("run",
                     ViewFactory<Panel>{Kind::Run, [](const YAML::Node &n) { keys(n, {"profile"}, "run panel"); },
                                        [](const Binding &b) { return std::make_unique<RunPanel>(b); }});
}
} // namespace pool::panels
