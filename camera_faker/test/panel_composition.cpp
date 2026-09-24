#include "pool_viewer/panels/composition.hpp"
#include <cassert>
#include <iostream>
using namespace pool::panels;
struct FakeMotion : Motion {
    MotionState s;
    MotionState state() override {
        return s;
    }
    void enable() override {
        s.enabled = true;
    }
    void kill() override {
        s.enabled = false;
    }
    void activate(Mode m, const Pose &p) override {
        s.mode = m;
        s.commanded = p;
    }
    void drag(const Pose &p) override {
        s.commanded = p;
    }
    void block(bool b) override {
        s.blocked = b;
    }
};
struct FakeMission : Autonomy {
    MissionState s;
    MissionState state() override {
        return s;
    }
    void refresh() override {}
    void start(const std::string &) override {
        s.busy = true;
    }
    void stop() override {
        s.busy = false;
    }
};
int main() {
    Registry r;
    registerPanels(r);
    int created = 0;
    r.providers.emplace("fake.motion", ProviderFactory{Kind::Motion, [](auto) {},
                                                       [&](auto, auto) {
                                                           ++created;
                                                           return std::make_shared<FakeMotion>();
                                                       }});
    r.providers.emplace("fake.mission", ProviderFactory{Kind::Autonomy, [](auto) {},
                                                        [&](auto, auto) {
                                                            ++created;
                                                            return std::make_shared<FakeMission>();
                                                        }});
    const auto text = R"(schema_version: 1
providers:
  motion: {type: fake.motion, options: {}}
  mission: {type: fake.mission, options: {}}
panels:
  - {id: control, type: motion, provider: motion}
  - {id: autonomy, type: autonomy, provider: mission}
overlays:
  - {id: target, type: pose_gizmo, provider: motion}
ownership:
  - {motion: motion, autonomy: mission}
)";
    Context ctx{"some_robot", "some_frame", false, false};
    Composition good(YAML::Load(text), ctx, r);
    assert(created == 2);
    assert(good.sidebarVisible());
    assert(std::abs(good.width(1000) - 290) < .01f);
    assert(std::abs(good.width(1500) - 435) < .01f);
    good.setWidth(400, true);
    assert(good.width(1500) == 400);
    good.toggleSidebar();
    assert(!good.sidebarVisible());
    good.toggleSidebar();
    auto motion = std::dynamic_pointer_cast<Motion>(good.providers().at("motion"));
    auto mission = std::dynamic_pointer_cast<Autonomy>(good.providers().at("mission"));
    mission->start("test");
    good.touch();
    assert(motion->state().blocked);
    mission->stop();
    good.touch();
    assert(!motion->state().blocked);
    ctx.preview = true;
    Composition preview(YAML::Load(text), ctx, r);
    assert(created == 2 && preview.providers().empty());
    Composition empty(YAML::Load("schema_version: 1\nproviders: {}"), ctx, r);
    assert(empty.empty());
    ctx.preview = false;
    auto fails = [&](const YAML::Node &cfg) {
        bool threw = false;
        try {
            Composition bad(cfg, ctx, r);
        } catch (const std::exception &) {
            threw = true;
        }
        assert(threw && created == 2);
    };
    auto cfg = YAML::Load(text);
    cfg["panels"][0]["provider"] = "missing";
    fails(cfg);
    cfg = YAML::Load(text);
    cfg["panels"][0]["provider"] = "mission";
    fails(cfg);
    cfg = YAML::Load(text);
    cfg["panels"][1]["id"] = "control";
    fails(cfg);
    cfg = YAML::Load(text);
    cfg["sidebar_width"] = 10;
    fails(cfg);
    cfg = YAML::Load(text);
    cfg["sidebar_width_fraction"] = 1;
    fails(cfg);
    cfg["sidebar_width_fraction"] = .29;
    cfg["sidebar_width"] = 350;
    fails(cfg);
    cfg = YAML::Load(text);
    cfg["providers"]["motion"]["type"] = "missing";
    fails(cfg);
    cfg = YAML::Load(text);
    cfg["overlays"][0]["options"]["size_metres"] = -5;
    fails(cfg);
    cfg = YAML::Load(text);
    cfg["schema_version"] = 2;
    fails(cfg);
    cfg = YAML::Load(text);
    cfg["typo"] = true;
    fails(cfg);
    cfg = YAML::Load(text);
    cfg["tools"] = YAML::Load("[{id: tool, type: motion, provider: motion, slot: typo}]");
    fails(cfg);
    cfg = YAML::Load(text);
    cfg["panels"][0]["slot"] = "settings";
    fails(cfg);
    assert(expand("/{namespace}/{fixed_frame}", ctx) == "/some_robot/some_frame");
    bool threw = false;
    try {
        expand("{typo}", ctx);
    } catch (const std::exception &) {
        threw = true;
    }
    assert(threw);
    std::cout << "PASS: composition bindings, validation, preview passivity, ownership, and empty layout\n";
}
