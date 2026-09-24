#pragma once
#include <imgui.h>
#include <algorithm>
namespace pool {
inline void sameLineIfFits(float width) {
    const float right = ImGui::GetWindowPos().x + ImGui::GetWindowContentRegionMax().x;
    if (ImGui::GetItemRectMax().x + ImGui::GetStyle().ItemSpacing.x + width <= right)
        ImGui::SameLine();
}

inline bool disclosureHeader(const char *label, bool open) {
    const auto origin = ImGui::GetCursorScreenPos();
    const auto &style = ImGui::GetStyle();
    ImGui::PushStyleColor(ImGuiCol_Button, style.Colors[ImGuiCol_Header]);
    ImGui::PushStyleColor(ImGuiCol_ButtonHovered, style.Colors[ImGuiCol_HeaderHovered]);
    ImGui::PushStyleColor(ImGuiCol_ButtonActive, style.Colors[ImGuiCol_HeaderActive]);
    if (ImGui::Button("##disclosure", {ImGui::GetContentRegionAvail().x, ImGui::GetFrameHeight()}))
        open = !open;
    ImGui::PopStyleColor(3);
    auto *draw = ImGui::GetWindowDrawList();
    const auto color = ImGui::GetColorU32(ImGuiCol_Text);
    draw->AddText({origin.x + ImGui::GetFontSize() + style.FramePadding.x * 3, origin.y + style.FramePadding.y}, color,
                  label);
    if (ImGui::IsItemHovered() || (ImGui::IsItemFocused() && ImGui::GetIO().NavVisible)) {
        const ImVec2 center(origin.x + style.FramePadding.x + 7, origin.y + ImGui::GetFrameHeight() * .5f);
        if (open)
            draw->AddTriangleFilled({center.x - 5, center.y - 3}, {center.x + 5, center.y - 3},
                                    {center.x, center.y + 5}, color);
        else
            draw->AddTriangleFilled({center.x - 3, center.y - 5}, {center.x - 3, center.y + 5},
                                    {center.x + 5, center.y}, color);
    }
    return open;
}
// Retain drag ownership across a snap so the user can pull the panel back open
// before releasing. Only the restored width is clamped; the gesture is not.
class PanelEdge {
    bool dragging = false, startedVisible = false;
    float startX = 0, startWidth = 0;

  public:
    // Origin anchors the sidebar at its outer window edge; right-side drags mirror left-side drags.
    bool draw(ImVec2 origin, float height, bool visible, float &width, float maximum, bool rightSide = false) {
        constexpr float minimum = 300, snapRatio = .8f;
        const float direction = rightSide ? -1.f : 1.f;
        const auto edgeX = [&] { return origin.x + direction * (visible ? width : 0) - (rightSide ? 16 : 0); };
        const ImVec2 edge(edgeX(), origin.y);
        ImGui::SetCursorScreenPos(edge);
        ImGui::InvisibleButton("divider", {16, std::max(1.f, height)});
        if (ImGui::IsItemActivated()) {
            dragging = true;
            startedVisible = visible;
            startX = ImGui::GetIO().MousePos.x;
            startWidth = visible ? width : 0;
        }
        if (dragging && ImGui::IsMouseDown(ImGuiMouseButton_Left)) {
            const float requested = startWidth + direction * (ImGui::GetIO().MousePos.x - startX);
            const float threshold = minimum * (startedVisible ? snapRatio : 1 - snapRatio);
            visible = requested >= threshold;
            width = std::clamp(requested, minimum, maximum);
        } else
            dragging = false;
        const bool hovered = ImGui::IsItemHovered();
        if (hovered || dragging)
            ImGui::SetMouseCursor(ImGuiMouseCursor_ResizeEW);
        if (hovered || dragging) {
            const float x = edgeX() + 8;
            const auto tint = IM_COL32(70, 200, 190, 255);
            ImGui::GetWindowDrawList()->AddLine({x, origin.y}, {x, origin.y + height}, tint, 2);
            const float middle = origin.y + height * .5f;
            ImGui::GetWindowDrawList()->AddRectFilled({x - 2, middle - 20}, {x + 2, middle + 20}, tint, 2);
        }
        if (hovered && !dragging)
            ImGui::SetTooltip(visible     ? "Drag to resize / pull toward the window edge to hide"
                              : rightSide ? "Drag left to show cameras"
                                          : "Drag right to show panels");
        return visible;
    }
};
} // namespace pool
