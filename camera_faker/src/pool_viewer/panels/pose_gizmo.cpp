#include "pool_viewer/panels/composition.hpp"
#include "pool_viewer/panels/pose_math.hpp"
#include <imgui.h>
#include <glm/gtc/constants.hpp>
namespace pool::panels {
namespace {
class PoseGizmo final : public Overlay {
    std::shared_ptr<Motion> control;
    int targetDrag = -1;
    Pose dragStart{1};
    glm::mat4 dragProjectionView{1};
    glm::vec2 dragOrigin{0}, dragSize{1};
    glm::vec3 dragRpy{0}, dragPoint{0}, dragDirection{0}, dragNormal{0};
    glm::vec2 dragMouse{0}, dragTangent{0};
    float dragAxis = 0, dragAngle = 0, sizeMetres, hitPixels;
    glm::vec3 dragRadial{0};
    bool dragInPlane = false, dragRadialValid = false;

  public:
    explicit PoseGizmo(const Binding &b)
        : control(std::dynamic_pointer_cast<Motion>(b.provider)), sizeMetres(b.options["size_metres"].as<float>(.3f)),
          hitPixels(b.options["hit_pixels"].as<float>(20)) {}
    void cancelInteraction() override {
        targetDrag = -1;
    }
    bool input(const Viewport &v) override {
        return update(v, false);
    }
    void draw(const Viewport &v) override {
        update(v, true);
    }
    bool update(const Viewport &view, bool draw) {
        const ImVec2 origin(view.origin.x, view.origin.y);
        const float width = view.size.x, height = view.size.y;
        const bool hovered = view.interactive;
        if (!control)
            return false;
        const auto state = control->state();
        if (state.mode != Mode::Position || !state.enabled || !state.fresh || state.blocked || state.pending) {
            targetDrag = -1;
            return false;
        }
        const auto vp = view.projection * view.view;
        const glm::vec3 p(state.commanded[3]);
        const float length = sizeMetres;
        const glm::vec3 axes[] = {glm::normalize(glm::vec3(state.commanded[0])),
                                  glm::normalize(glm::vec3(state.commanded[1])),
                                  glm::normalize(glm::vec3(state.commanded[2]))};
        const ImU32 colors[] = {IM_COL32(90, 235, 230, 255), IM_COL32(235, 75, 75, 255), IM_COL32(90, 215, 110, 255),
                                IM_COL32(80, 145, 255, 255), IM_COL32(235, 75, 75, 255), IM_COL32(90, 215, 110, 255),
                                IM_COL32(80, 145, 255, 255)};
        const auto angles = glm::eulerAngles(glm::quat_cast(state.commanded));
        const auto project = [&](const glm::vec3 &point, ImVec2 &pixel) {
            const auto clip = vp * glm::vec4(point, 1);
            if (clip.w <= 0)
                return false;
            pixel = {origin.x + (clip.x / clip.w * .5f + .5f) * width,
                     origin.y + (.5f - clip.y / clip.w * .5f) * height};
            return std::abs(clip.x) <= clip.w && std::abs(clip.y) <= clip.w && std::abs(clip.z) <= clip.w;
        };
        struct Segment {
            ImVec2 first, second;
            glm::vec3 a, b;
            int handle;
        };
        std::vector<Segment> rings, arrows;
        ImVec2 center;
        const bool centerVisible = project(p, center);
        for (int axis = 0; axis < 3; ++axis) {
            const auto normal = pool::rpyAxis(angles, axis);
            const auto a = pool::rpyReference(angles, axis) * length * 1.15f;
            const auto b = glm::cross(normal, a);
            for (int step = 0; step < 96; ++step) {
                const float start = step * glm::two_pi<float>() / 96.f, end = (step + 1) * glm::two_pi<float>() / 96.f;
                Segment segment;
                segment.a = std::cos(start) * a + std::sin(start) * b;
                segment.b = std::cos(end) * a + std::sin(end) * b;
                segment.handle = axis + 4;
                if (project(p + segment.a, segment.first) && project(p + segment.b, segment.second))
                    rings.push_back(segment);
            }
            for (float sign : {-1.f, 1.f}) {
                Segment segment;
                segment.a = axes[axis] * length * .2f * sign;
                segment.b = axes[axis] * length * 1.5f * sign;
                segment.handle = axis + 1;
                if (project(p + segment.a, segment.first) && project(p + segment.b, segment.second))
                    arrows.push_back(segment);
            }
        }
        auto &io = ImGui::GetIO();
        const glm::vec2 mouse(io.MousePos.x, io.MousePos.y);
        int hit = -1;
        float best = hitPixels * .6f;
        glm::vec3 selectedRadial{0};
        // Every visible ring segment and arrow shaft is a handle. No grip dots.
        for (const auto &segment : rings) {
            float fraction;
            const auto distance = pool::segmentDistance(mouse, {segment.first.x, segment.first.y},
                                                        {segment.second.x, segment.second.y}, fraction);
            if (distance < best) {
                best = distance;
                hit = segment.handle;
                selectedRadial = glm::mix(segment.a, segment.b, fraction);
            }
        }
        for (const auto &segment : arrows) {
            float fraction;
            const auto distance = pool::segmentDistance(mouse, {segment.first.x, segment.first.y},
                                                        {segment.second.x, segment.second.y}, fraction);
            // Arrows take priority at crossings, matching their draw order.
            if (distance <= best + 1.f) {
                best = distance;
                hit = segment.handle;
            }
        }
        if (centerVisible && glm::length(mouse - glm::vec2(center.x, center.y)) < 10)
            hit = 0;
        if (!hovered)
            hit = -1;
        if (draw) {
            auto *list = ImGui::GetWindowDrawList();
            list->PushClipRect(origin, {origin.x + width, origin.y + height}, true);
            const int active = targetDrag >= 0 ? targetDrag : hit;
            const auto tint = [&](int handle, int alpha = 255) {
                return active == handle ? IM_COL32(255, 235, 90, 255)
                                        : (colors[handle] & 0x00ffffff) | IM_COL32(0, 0, 0, alpha);
            };
            for (const auto &segment : rings) {
                const bool front = glm::dot((segment.a + segment.b) * .5f, view.eye - p) > 0;
                list->AddLine(segment.first, segment.second, tint(segment.handle, front ? 210 : 105),
                              active == segment.handle ? 8 : 5);
            }
            for (const auto &segment : arrows) {
                const glm::vec2 delta(segment.second.x - segment.first.x, segment.second.y - segment.first.y);
                const auto distance = glm::length(delta);
                if (distance < 3)
                    continue;
                const auto direction = delta / distance, perpendicular = glm::vec2(-direction.y, direction.x);
                const auto end = glm::vec2(segment.second.x, segment.second.y);
                const auto base = end - direction * std::min(17.f, distance * .5f);
                const auto left = base + perpendicular * 7.f, right = base - perpendicular * 7.f;
                list->AddLine(segment.first, {base.x, base.y}, tint(segment.handle), active == segment.handle ? 7 : 5);
                list->AddTriangleFilled(segment.second, {left.x, left.y}, {right.x, right.y}, tint(segment.handle));
            }
            if (centerVisible)
                list->AddRectFilled({center.x - 5, center.y - 5}, {center.x + 5, center.y + 5}, tint(0), 1);
            list->PopClipRect();
            if (active >= 0) {
                const char *names[] = {"Body XY position",
                                       "Body X position",
                                       "Body Y position",
                                       "Body Z position",
                                       "Roll",
                                       "Pitch",
                                       "Yaw"};
                ImGui::SetMouseCursor(ImGuiMouseCursor_Hand);
                ImGui::SetTooltip("%s / drag commands immediately / Esc restores start", names[active]);
            }
            return false;
        }
        // Keep cursor-to-world conversion stable while the Follow camera moves.
        const auto ray = targetDrag >= 0 ? pool::screenRay(dragProjectionView, mouse - dragOrigin, dragSize)
                                         : pool::screenRay(vp, mouse - view.origin, view.size);
        bool consumed = targetDrag >= 0;
        if (targetDrag < 0 && hovered && ImGui::IsMouseClicked(ImGuiMouseButton_Left)) {
            if (hit >= 0) {
                consumed = true;
                bool valid = false;
                if (hit >= 4) {
                    dragNormal = pool::rpyAxis(angles, hit - 4);
                    dragRpy = angles;
                    dragAngle = 0;
                    // Use the cursor angle in the ring plane when it is visible.
                    // Freeze this choice for the gesture, as with its projection.
                    glm::vec3 point;
                    dragInPlane = std::abs(glm::dot(ray.direction, dragNormal)) >= .15f &&
                                  pool::planeHit(ray, p, dragNormal, point);
                    if (dragInPlane) {
                        dragRadial = point - p;
                        valid = glm::length(dragRadial) > sizeMetres * .1f;
                        if (valid)
                            dragRadial = glm::normalize(dragRadial);
                        dragRadialValid = valid;
                    } else {
                        // Edge-on rings collapse to a line; keep straight dragging
                        // usable instead of dividing by a near-parallel ray.
                        const auto tangent = glm::cross(dragNormal, selectedRadial);
                        ImVec2 base, tip;
                        valid = project(p + selectedRadial, base) && project(p + selectedRadial + tangent * .1f, tip);
                        dragTangent = glm::vec2(tip.x - base.x, tip.y - base.y) * 10.f;
                        valid = valid && glm::length(dragTangent) > 5;
                        dragMouse = mouse;
                    }
                } else if (hit == 0) {
                    dragNormal = axes[2];
                    valid = pool::planeHit(ray, p, dragNormal, dragPoint);
                } else {
                    dragDirection = axes[hit - 1];
                    valid = pool::axisHit(ray, p, dragDirection, dragAxis);
                }
                if (valid) {
                    targetDrag = hit;
                    dragStart = state.commanded;
                    dragProjectionView = vp;
                    dragOrigin = view.origin;
                    dragSize = view.size;
                }
            }
        }
        if (targetDrag >= 0) {
            if (ImGui::IsKeyPressed(ImGuiKey_Escape)) {
                control->drag(dragStart);
                targetDrag = -1;
            } else if (!ImGui::IsMouseDown(ImGuiMouseButton_Left) || !view.focused) {
                targetDrag = -1;
            } else {
                glm::mat4 next = dragStart;
                glm::vec3 point;
                float along;
                if (targetDrag >= 4) {
                    const bool moved = glm::length(glm::vec2(io.MouseDelta.x, io.MouseDelta.y)) > 0;
                    bool changed = false;
                    if (dragInPlane) {
                        if (pool::planeHit(ray, glm::vec3(dragStart[3]), dragNormal, point) &&
                            glm::length(point - glm::vec3(dragStart[3])) > sizeMetres * .1f) {
                            const auto radial = glm::normalize(point - glm::vec3(dragStart[3]));
                            if (dragRadialValid && moved) {
                                // Accumulate signed steps so crossing +/-pi never
                                // reverses direction or limits the number of turns.
                                dragAngle += std::atan2(glm::dot(dragNormal, glm::cross(dragRadial, radial)),
                                                        glm::dot(dragRadial, radial));
                                changed = true;
                            }
                            dragRadial = radial;
                            dragRadialValid = true;
                        } else {
                            // Angle is undefined at the center. Re-anchor on
                            // re-entry rather than issuing a half-turn jump.
                            dragRadialValid = false;
                        }
                    } else if (moved) {
                        dragAngle = glm::dot(mouse - dragMouse, dragTangent) / glm::dot(dragTangent, dragTangent);
                        changed = true;
                    }
                    if (changed) {
                        auto nextAngles = dragRpy;
                        nextAngles[targetDrag - 4] += dragAngle;
                        control->drag(pool::rpyPose(glm::vec3(dragStart[3]), nextAngles));
                    }
                } else if (targetDrag == 0 && pool::planeHit(ray, glm::vec3(dragStart[3]), dragNormal, point)) {
                    next[3] += glm::vec4(point - dragPoint, 0);
                    if (glm::length(glm::vec2(io.MouseDelta.x, io.MouseDelta.y)) > 0)
                        control->drag(next);
                } else if (targetDrag > 0 && targetDrag < 4 &&
                           pool::axisHit(ray, glm::vec3(dragStart[3]), dragDirection, along)) {
                    next[3] += glm::vec4(dragDirection * (along - dragAxis), 0);
                    if (glm::length(glm::vec2(io.MouseDelta.x, io.MouseDelta.y)) > 0)
                        control->drag(next);
                }
            }
        }
        return consumed;
    }
};
} // namespace
void registerPoseGizmo(Registry &r) {
    r.overlays.emplace("pose_gizmo",
                       ViewFactory<Overlay>{Kind::Motion,
                                            [](const YAML::Node &n) {
                                                keys(n, {"size_metres", "hit_pixels"}, "pose_gizmo");
                                                positive(n, "size_metres", .3, 10);
                                                positive(n, "hit_pixels", 20, 40);
                                            },
                                            [](const Binding &b) { return std::make_unique<PoseGizmo>(b); }});
}
} // namespace pool::panels
