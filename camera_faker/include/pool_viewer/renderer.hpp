#pragma once
#include "pool_viewer/camera.hpp"
#include "pool_viewer/frustum.hpp"
#include "pool_viewer/status_lights.hpp"
#include <glad/glad.h>
#include <array>
#include <map>
#include <memory>
#include <stdexcept>
#include <vector>

namespace pool {
struct Vertex {
    glm::vec3 p, n;
    glm::vec2 uv;
};
struct Mesh {
    GLuint vao = 0, vbo = 0, ebo = 0, texture = 0;
    int count = 0;
    Bounds bounds;
    glm::vec4 color{1};
    std::vector<glm::vec3> holes; // texture-space center/radius for real openings
    Mesh(const std::vector<Vertex> &v, const std::vector<unsigned> &i);
    ~Mesh();
    Mesh(const Mesh &) = delete;
    Mesh &operator=(const Mesh &) = delete;
    void draw() const;
};
struct Object {
    std::string name;
    std::vector<std::shared_ptr<Mesh>> meshes;
    glm::mat4 transform{1};
    int material = 0; // 0 asset, 1 tiles, 2 deck, 3 lamp, 4 liner, 5 clear cover, 6 LED
    bool robot = false, tag = false, castsShadow = true;
    glm::vec4 tint{1};
    glm::mat4 robotMount{1};
    float radiance = -1; // Negative uses the existing task LED radiance.
};
struct Target {
    GLuint fbo = 0, color = 0, depth = 0;
    int width = 0, height = 0;
    Target() = default;
    ~Target();
    Target(const Target &) = delete;
    Target &operator=(const Target &) = delete;
    void resize(int w, int h, bool hdr = true, bool depthOnly = false);
    void release();
};
struct Frame {
    Target opaque, composite, final;
    std::array<Target, 2> bloom;
    void resize(int w, int h) {
        opaque.resize(w, h);
        composite.resize(w, h);
        final.resize(w, h, false);
        for (auto &target : bloom)
            target.resize(std::max(1, w / 4), std::max(1, h / 4));
    }
    void release() {
        opaque.release();
        composite.release();
        final.release();
        for (auto &target : bloom)
            target.release();
    }
};
struct WaterOptics {
    glm::vec3 tint{.025f, .22f, .29f}, absorption{.095f, .035f, .025f};
    float scattering = .10f, distanceScale = 1.f, distancePower = 1.f, clearDistance = 0.f;
    void validate() const {
        for (int i = 0; i < 3; ++i)
            if (!std::isfinite(tint[i]) || tint[i] < 0 || tint[i] > 1 || !std::isfinite(absorption[i]) ||
                absorption[i] < 0 || absorption[i] > 5)
                throw std::invalid_argument("Water tint must be in [0,1], absorption in [0,5]");
        if (!std::isfinite(scattering) || scattering < 0 || scattering > 5 || !std::isfinite(distanceScale) ||
            distanceScale < 0 || distanceScale > 5 || !std::isfinite(distancePower) || distancePower < .25f ||
            distancePower > 3 || !std::isfinite(clearDistance) || clearDistance < 0 || clearDistance > 20)
            throw std::invalid_argument("Invalid water distance or scattering settings");
    }
};
struct PointCloud {
    GLuint vao = 0, vbo = 0;
    int count = 0;
    glm::mat4 transform{1};
    glm::vec3 highlight{1};
};
struct Look {
    WaterOptics water;
    float caustics = 1.f, exposure = 1.f;
    bool surface = true, shadows = true, tag = true;
    bool outdoor = false;
    float sunAzimuth = 225.f, sunElevation = 55.f;
    float directLight = 1.f, ambientLight = .7f, glare = .5f;
    glm::vec3 sunDirection() const {
        float a = glm::radians(sunAzimuth), e = glm::radians(sunElevation);
        return {cos(e) * cos(a), cos(e) * sin(a), sin(e)};
    }
};
class Renderer {
  public:
    Renderer(const std::string &shaders, const std::string &meshes, const std::string &textures,
             const std::string &mapping, const std::string &markers, const std::string &scene, const std::string &robot,
             const std::string &robotAsset = "", const std::string &taskConfig = "",
             const std::string &payloadAsset = "", const std::string &launcherAsset = "",
             const std::string &clawAsset = "", const std::vector<StatusLight> &statusLights = {});
    ~Renderer();
    void robotPose(const glm::mat4 &p);
    void payloadPoses(const std::vector<glm::mat4> &poses);
    void objectPose(const std::string &name, const glm::mat4 &pose);
    void clawPose(const glm::mat4 &mount, float left, float right);
    void magnetPose(const glm::mat4 &mount);
    void magnetLight(const std::string &name, bool green);
    void statusLight(const std::string &id, const glm::vec3 &color);
    void shadows(const Look &look);
    // Overlays are drawn only when asked, so sensor renders never contain them.
    void render(Frame &frame, const View &camera, const Look &look, float time, bool showRobot = true,
                bool reflect = false, bool overlays = false);
    // Overlay point cloud: six floats per point (xyz metres, rgb in [0,1]) in
    // the frame that `transform` maps into the map frame.
    void pointCloud(int slot, const std::vector<float> &xyzrgb, const glm::mat4 &transform, const glm::vec3 &highlight);
    void clearPointCloud(int slot);
    float pointSize = 3.f, pointHighlight = 0.f; // 0 = sensor RGB, 1 = highlight color
    std::map<std::string, glm::mat4> landmarks;
    glm::mat4 poolToMap{1}, mapToPool{1};
    size_t triangles = 0;

  private:
    std::vector<Object> objects;
    std::map<std::string, std::vector<std::shared_ptr<Mesh>>> cache;
    std::map<std::string, GLuint> textures;
    GLuint sceneProgram = 0, waterProgram = 0, postProgram = 0, shadowProgram = 0, pointProgram = 0, quad = 0;
    GLuint bloomProgram = 0;
    std::vector<PointCloud> pointClouds;
    YAML::Node world;
    Target shadow, reflection;
    glm::mat4 lightMatrix{1};
    Object water;
    std::string meshRoot, textureRoot;
    std::vector<glm::vec3> torpedoHoles;
    std::vector<std::shared_ptr<Mesh>> payloadMeshes;
    float ledRadiance = 60.f;
    float clawMinGap = .008f;
    std::map<std::string, glm::mat4> clawParts;
    void buildCrates(const YAML::Node &config);
    void buildOctagon(const YAML::Node &config);
    std::vector<std::shared_ptr<Mesh>> load(const std::string &name, const std::string &overrideTexture = "");
    GLuint texture(const std::string &file);
    void box(const std::string &name, const glm::mat4 &matrix, const glm::vec3 &size, const glm::vec3 &color,
             int material = 0, bool castsShadow = true);
    void buildPool();
    void drawScene(const View &camera, const Look &look, float time, bool showRobot, bool clip = false);
    void drawPoints(const View &camera, const Look &look);
};
} // namespace pool
