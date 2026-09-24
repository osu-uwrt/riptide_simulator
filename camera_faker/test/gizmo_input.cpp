#include "pool_viewer/panels/composition.hpp"
#include "pool_viewer/panels/pose_math.hpp"
#include <imgui.h>
#include <cassert>
using namespace pool::panels;
namespace {
struct MotionProbe : Motion {
    MotionState s;
    int commands = 0;
    MotionState state() override {
        return s;
    }
    void enable() override {}
    void kill() override {}
    void activate(Mode, const Pose &) override {}
    void block(bool) override {}
    void drag(const Pose &p) override {
        s.commanded = p;
        ++commands;
    }
};
glm::vec2 project(const Viewport &v, glm::vec3 point) {
    auto clip = v.projection * v.view * glm::vec4(point, 1);
    return v.origin + glm::vec2(clip.x / clip.w * .5f + .5f, .5f - clip.y / clip.w * .5f) * v.size;
}
glm::vec3 dragAxis(Registry &registry, int axis, bool cameraMoves) {
    auto motion = std::make_shared<MotionProbe>();
    motion->s.enabled = motion->s.fresh = motion->s.hasCommand = true;
    motion->s.mode = Mode::Position;
    const auto initial = pool::rpyPose({0, 0, 0}, glm::radians(glm::vec3(15, 30, 90)));
    motion->s.commanded = initial;
    const glm::vec3 direction(initial[axis]);
    auto overlay = registry.overlays.at("pose_gizmo").create({motion, YAML::Node(), {}, {}});
    Viewport view;
    view.eye = {2, -3, 2};
    view.view = glm::lookAt(view.eye, glm::vec3(0), glm::vec3(0, 0, 1));
    view.projection = glm::perspective(glm::radians(53.f), 4.f / 3.f, .05f, 100.f);
    view.size = {800, 600};
    view.interactive = view.focused = true;
    const auto start = project(view, direction * .43f), end = project(view, direction * .53f);
    auto frame = [&](glm::vec2 cursor, bool pressed) {
        auto &io = ImGui::GetIO();
        io.AddMousePosEvent(cursor.x, cursor.y);
        io.AddMouseButtonEvent(0, pressed);
        ImGui::NewFrame();
        ImGui::SetNextWindowPos({0, 0});
        ImGui::SetNextWindowSize({800, 600});
        ImGui::Begin("viewport", nullptr, ImGuiWindowFlags_NoDecoration);
        overlay->input(view);
        ImGui::End();
        ImGui::Render();
    };
    frame(start, false);
    frame(start, true);
    if (cameraMoves) {
        const glm::vec3 offset(.4, .1, .2);
        view.eye += offset;
        view.view = glm::lookAt(view.eye, offset, glm::vec3(0, 0, 1));
    }
    frame(end, true);
    frame(end, false);
    assert(motion->commands > 0);
    // A 10 cm body-axis drag must transform into the corresponding world
    // displacement, even while Follow translates the viewing camera.
    const auto local = glm::transpose(glm::mat3(initial)) * glm::vec3(motion->s.commanded[3]);
    // ImGui rounds cursor coordinates to pixels, giving up to a few mm here.
    assert(std::abs(local[axis] - .1f) < .006f);
    for (int i = 0; i < 3; ++i)
        if (i != axis)
            assert(std::abs(local[i]) < 1e-5f);
    for (int i = 0; i < 3; ++i)
        assert(glm::length(motion->s.commanded[i] - initial[i]) < 1e-5f);
    return glm::vec3(motion->s.commanded[3]);
}
void rotateRing(Registry &registry, int axis, float direction, bool cameraMoves) {
    auto motion = std::make_shared<MotionProbe>();
    motion->s.enabled = motion->s.fresh = motion->s.hasCommand = true;
    motion->s.mode = Mode::Position;
    const auto angles = glm::radians(glm::vec3(15, 20, 25));
    const auto initial = pool::rpyPose({0, 0, 0}, angles);
    motion->s.commanded = initial;
    const auto normal = pool::rpyAxis(angles, axis);
    const auto reference = pool::rpyReference(angles, axis);
    const auto side = glm::cross(normal, reference);
    auto overlay = registry.overlays.at("pose_gizmo").create({motion, YAML::Node(), {}, {}});
    Viewport view;
    view.eye = normal * 2.f + (cameraMoves ? reference * .8f : glm::vec3(0));
    view.view = glm::lookAt(view.eye, glm::vec3(0), reference);
    view.projection = glm::perspective(glm::radians(53.f), 4.f / 3.f, .05f, 100.f);
    view.size = {800, 600};
    view.interactive = view.focused = true;
    const auto frozenView = view;
    const auto cursor = [&](float angle) {
        const float phase = .65f + angle;
        return project(frozenView, .345f * (std::cos(phase) * reference + std::sin(phase) * side));
    };
    const auto frame = [&](glm::vec2 position, bool pressed, bool escape = false) {
        auto &io = ImGui::GetIO();
        io.AddMousePosEvent(position.x, position.y);
        io.AddMouseButtonEvent(0, pressed);
        io.AddKeyEvent(ImGuiKey_Escape, escape);
        ImGui::NewFrame();
        ImGui::SetNextWindowPos({0, 0});
        ImGui::SetNextWindowSize({800, 600});
        ImGui::Begin("viewport", nullptr, ImGuiWindowFlags_NoDecoration);
        overlay->input(view);
        overlay->draw(view);
        ImGui::End();
        ImGui::Render();
    };
    frame(cursor(0), false);
    frame(cursor(0), true);
    auto previous = glm::mat3(initial);
    // Two complete turns cross the atan2 seam and, for pitch, Euler singularities.
    for (int step = 1; step <= 144; ++step) {
        const float angle = direction * step * glm::two_pi<float>() / 72.f;
        if (cameraMoves) {
            const auto offset = glm::vec3(.2f, .1f, .05f) * (step / 144.f);
            view.eye = frozenView.eye + offset;
            view.view = glm::lookAt(view.eye, offset, reference);
        }
        frame(cursor(angle), true);
        const auto expected = glm::rotate(glm::mat4(1), angle, normal) * initial;
        const auto actual = glm::mat3(motion->s.commanded);
        for (int col = 0; col < 3; ++col)
            assert(glm::length(actual[col] - glm::vec3(expected[col])) < .025f);
        assert(glm::length(glm::vec3(motion->s.commanded[3])) < 1e-5f);
        // Every step must keep turning in the requested direction, not reverse
        // at a quadrant or jump at a wrap boundary.
        const auto moved = actual * glm::transpose(previous) * reference;
        const float increment = std::atan2(glm::dot(normal, glm::cross(reference, moved)), glm::dot(reference, moved));
        assert(direction * increment > .04f && direction * increment < .14f);
        previous = actual;
    }
    assert(motion->commands >= 144);
    // Moving through the undefined center pauses and re-anchors the angle.
    const auto held = motion->s.commanded;
    const auto commands = motion->commands;
    frame(project(frozenView, glm::vec3(0)), true);
    frame(cursor(direction * 13.f), true);
    assert(motion->commands == commands);
    for (int col = 0; col < 4; ++col)
        assert(glm::length(motion->s.commanded[col] - held[col]) < 1e-5f);
    frame(cursor(direction * 13.1f), true);
    assert(motion->commands > commands);
    // Escape restores the exact starting pose, including after multiple turns.
    frame(cursor(direction * 13.f), true, true);
    for (int col = 0; col < 4; ++col)
        assert(glm::length(motion->s.commanded[col] - initial[col]) < 1e-5f);
    frame(cursor(0), false);
}
} // namespace
int main() {
    ImGui::CreateContext();
    auto &io = ImGui::GetIO();
    io.IniFilename = nullptr;
    io.DisplaySize = {800, 600};
    io.DeltaTime = 1.f / 60;
    io.ConfigInputTrickleEventQueue = false;
    unsigned char *pixels;
    int width, height;
    io.Fonts->GetTexDataAsRGBA32(&pixels, &width, &height);
    Registry registry;
    registerPanels(registry);
    for (int axis = 0; axis < 3; ++axis) {
        const auto stationary = dragAxis(registry, axis, false);
        const auto following = dragAxis(registry, axis, true);
        assert(glm::length(stationary - following) < 1e-5f);
    }
    for (int axis = 0; axis < 3; ++axis)
        for (float direction : {-1.f, 1.f})
            for (bool cameraMoves : {false, true})
                rotateRing(registry, axis, direction, cameraMoves);
    ImGui::DestroyContext();
}
