#pragma once
#include "pool_viewer/panels/capabilities.hpp"
#include <yaml-cpp/yaml.h>
#include <functional>
#include <map>

namespace pool::panels {
struct Context {
    std::string robotNamespace, fixedFrame;
    bool preview = false, useSimTime = false;
    // Immutable host documents and optional navigation; no robot/task types.
    std::map<std::string, YAML::Node> documents{};
    std::function<void(const std::string &)> focus{};
    std::vector<std::string> initialWindows{};
};
struct Viewport {
    glm::mat4 projection{1}, view{1};
    glm::vec3 eye{0};
    glm::vec2 origin{0}, size{1};
    bool interactive = false, focused = true;
};
struct Panel {
    virtual ~Panel() = default;
    virtual void toolbar() {} // compact actions when the sidebar is hidden
    virtual void pinned() {}  // critical controls remain accessible when collapsed
    virtual void draw() = 0;
    virtual void drawWindows() {}
};
struct Overlay {
    virtual ~Overlay() = default;
    virtual void cancelInteraction() {}
    virtual bool input(const Viewport &) = 0;
    virtual void draw(const Viewport &) = 0;
};
using Providers = std::map<std::string, std::shared_ptr<Provider>>;
struct Binding {
    std::shared_ptr<Provider> provider;
    YAML::Node options;
    std::function<bool()> mayStart;
    std::function<void()> kill;
    std::map<std::string, YAML::Node> documents{};
    std::function<void(const std::string &)> focus{};
    bool showWindow = false;
    std::function<void()> drawOverlayControls{};
};
struct ProviderFactory {
    Kind kind;
    std::function<void(const YAML::Node &)> validate;
    std::function<std::shared_ptr<Provider>(const YAML::Node &, const Context &)> create;
};
template <class T> struct ViewFactory {
    Kind kind;
    std::function<void(const YAML::Node &)> validate;
    std::function<std::unique_ptr<T>(const Binding &)> create;
};
struct Registry {
    std::map<std::string, ProviderFactory> providers;
    std::map<std::string, ViewFactory<Panel>> panels;
    std::map<std::string, ViewFactory<Overlay>> overlays;
};
void keys(const YAML::Node &, std::initializer_list<const char *> allowed, const std::string &where);
void required(const YAML::Node &, std::initializer_list<const char *> names);
void positive(const YAML::Node &, const char *key, double fallback, double maximum = 60.);
std::string expand(std::string value, const Context &context);

class Composition {
  public:
    Composition(const YAML::Node &, const Context &, const Registry &);
    void touch();
    void drawSidebar(float height);
    void drawToolbar();
    void drawToolsToolbar(const std::string &slot = "overlays");
    void drawWindows();
    bool sidebarVisible() const {
        return sidebarShown;
    }
    void toggleSidebar() {
        sidebarShown = !sidebarShown;
    }
    bool input(const Viewport &);
    void drawOverlays(const Viewport &);
    void setWidth(float value, bool manuallyResized = false) {
        sidebarWidth = value;
        sidebarResized = sidebarResized || manuallyResized;
    }
    float width(float windowWidth = 0) const {
        return windowWidth > 0 && sidebarFraction > 0 && !sidebarResized ? windowWidth * sidebarFraction : sidebarWidth;
    }
    bool empty() const {
        return panelInstances.empty();
    }
    const Providers &providers() const {
        return sources;
    }

  private:
    struct PanelInstance {
        std::string id, title;
        bool visible, open;
        std::unique_ptr<Panel> panel;
        std::string slot = "overlays";
    };
    struct Ownership {
        std::shared_ptr<Motion> motion;
        std::shared_ptr<Autonomy> mission;
    };
    void syncOwnership();
    void drawOverlayControls(const std::string &provider);
    float sidebarWidth = 350, sidebarFraction = .29f;
    bool sidebarResized = false;
    bool sidebarShown = true;
    Providers sources;
    std::vector<PanelInstance> panelInstances, toolInstances;
    struct OverlayInstance {
        std::string id, title;
        bool visible;
        std::unique_ptr<Overlay> overlay;
        std::string provider;
    };
    std::vector<OverlayInstance> overlays;
    std::vector<Ownership> ownership;
};
void registerPanels(Registry &);
} // namespace pool::panels
