#include "pool_viewer/renderer.hpp"
#include <algorithm>
#include <array>
#include <assimp/Importer.hpp>
#include <assimp/config.h>
#include <assimp/postprocess.h>
#include <assimp/scene.h>
#include <fstream>
#include <functional>
#include <glm/gtc/type_ptr.hpp>
#include <iostream>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <set>

namespace pool {
namespace {
GLuint program(const std::string &root, const std::string &name) {
    GLuint p = glCreateProgram();
    for (auto type : {GL_VERTEX_SHADER, GL_FRAGMENT_SHADER}) {
        const std::string path = root + "/" + name + (type == GL_VERTEX_SHADER ? ".vert" : ".frag");
        std::ifstream f(path);
        if (!f)
            throw std::runtime_error("Missing shader: " + path);
        std::string source((std::istreambuf_iterator<char>(f)), {});
        const char *s = source.c_str();
        GLuint shader = glCreateShader(type);
        glShaderSource(shader, 1, &s, nullptr);
        glCompileShader(shader);
        GLint ok;
        glGetShaderiv(shader, GL_COMPILE_STATUS, &ok);
        if (!ok) {
            char log[4096];
            glGetShaderInfoLog(shader, sizeof(log), nullptr, log);
            throw std::runtime_error(path + ": " + log);
        }
        glAttachShader(p, shader);
        glDeleteShader(shader);
    }
    glLinkProgram(p);
    GLint ok;
    glGetProgramiv(p, GL_LINK_STATUS, &ok);
    if (!ok) {
        char log[4096];
        glGetProgramInfoLog(p, sizeof(log), nullptr, log);
        throw std::runtime_error(log);
    }
    return p;
}
void uniform(GLuint p, const char *n, const glm::mat4 &v) {
    glUniformMatrix4fv(glGetUniformLocation(p, n), 1, GL_FALSE, glm::value_ptr(v));
}
void uniform(GLuint p, const char *n, const glm::vec3 &v) {
    glUniform3fv(glGetUniformLocation(p, n), 1, glm::value_ptr(v));
}
void uniform(GLuint p, const char *n, float v) {
    glUniform1f(glGetUniformLocation(p, n), v);
}
void integer(GLuint p, const char *n, int v) {
    glUniform1i(glGetUniformLocation(p, n), v);
}
void bindTexture(GLuint id, int unit) {
    glActiveTexture(GL_TEXTURE0 + unit);
    glBindTexture(GL_TEXTURE_2D, id);
}
glm::mat4 aiMatrix(const aiMatrix4x4 &m) {
    return glm::transpose(glm::make_mat4(&m.a1));
}
std::shared_ptr<Mesh> cube() {
    std::vector<Vertex> v;
    std::vector<unsigned> idx;
    for (int axis = 0; axis < 3; ++axis)
        for (int sign : {-1, 1}) {
            glm::vec3 n(0), u(0), w(0);
            n[axis] = sign;
            u[(axis + 1) % 3] = 1;
            w[(axis + 2) % 3] = sign;
            unsigned start = v.size();
            for (glm::vec2 uv : {glm::vec2(0, 0), glm::vec2(1, 0), glm::vec2(1, 1), glm::vec2(0, 1)})
                v.push_back({n * .5f + (uv.x - .5f) * u + (uv.y - .5f) * w, n, uv});
            for (unsigned i : {0u, 1u, 2u, 0u, 2u, 3u})
                idx.push_back(start + i);
        }
    return std::make_shared<Mesh>(v, idx);
}
} // namespace
Mesh::Mesh(const std::vector<Vertex> &v, const std::vector<unsigned> &i) : count(i.size()) {
    for (const auto index : i)
        bounds.include(v[index].p);
    glGenVertexArrays(1, &vao);
    glGenBuffers(1, &vbo);
    glGenBuffers(1, &ebo);
    glBindVertexArray(vao);
    glBindBuffer(GL_ARRAY_BUFFER, vbo);
    glBufferData(GL_ARRAY_BUFFER, v.size() * sizeof(Vertex), v.data(), GL_STATIC_DRAW);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, ebo);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, i.size() * sizeof(unsigned), i.data(), GL_STATIC_DRAW);
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex), (void *)offsetof(Vertex, p));
    glEnableVertexAttribArray(1);
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex), (void *)offsetof(Vertex, n));
    glEnableVertexAttribArray(2);
    glVertexAttribPointer(2, 2, GL_FLOAT, GL_FALSE, sizeof(Vertex), (void *)offsetof(Vertex, uv));
    glBindVertexArray(0);
}
Mesh::~Mesh() {
    glDeleteVertexArrays(1, &vao);
    glDeleteBuffers(1, &vbo);
    glDeleteBuffers(1, &ebo);
}
void Mesh::draw() const {
    glBindVertexArray(vao);
    glDrawElements(GL_TRIANGLES, count, GL_UNSIGNED_INT, nullptr);
}
Target::~Target() {
    release();
}
void Target::release() {
    if (fbo)
        glDeleteFramebuffers(1, &fbo);
    if (color)
        glDeleteTextures(1, &color);
    if (depth)
        glDeleteTextures(1, &depth);
    fbo = color = depth = 0;
    width = height = 0;
}
void Target::resize(int w, int h, bool hdr, bool depthOnly) {
    if (w == width && h == height && depthOnly == (color == 0))
        return;
    release();
    width = w;
    height = h;
    glGenFramebuffers(1, &fbo);
    glBindFramebuffer(GL_FRAMEBUFFER, fbo);
    if (depthOnly) {
        glDrawBuffer(GL_NONE);
        glReadBuffer(GL_NONE);
    } else {
        glGenTextures(1, &color);
        glBindTexture(GL_TEXTURE_2D, color);
        glTexImage2D(GL_TEXTURE_2D, 0, hdr ? GL_RGBA16F : GL_RGBA8, w, h, 0, GL_RGBA, hdr ? GL_FLOAT : GL_UNSIGNED_BYTE,
                     nullptr);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
        glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, color, 0);
    }
    glGenTextures(1, &depth);
    glBindTexture(GL_TEXTURE_2D, depth);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_DEPTH_COMPONENT24, w, h, 0, GL_DEPTH_COMPONENT, GL_FLOAT, nullptr);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
    glFramebufferTexture2D(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_TEXTURE_2D, depth, 0);
    if (glCheckFramebufferStatus(GL_FRAMEBUFFER) != GL_FRAMEBUFFER_COMPLETE)
        throw std::runtime_error("OpenGL framebuffer incomplete");
    glBindFramebuffer(GL_FRAMEBUFFER, 0);
}
GLuint Renderer::texture(const std::string &file) {
    if (textures.count(file))
        return textures.at(file);
    cv::Mat pixels = cv::imread(file, cv::IMREAD_UNCHANGED);
    if (pixels.empty())
        throw std::runtime_error("Cannot load scene texture: " + file);
    if (pixels.channels() == 3)
        cv::cvtColor(pixels, pixels, cv::COLOR_BGR2RGBA);
    else if (pixels.channels() == 4)
        cv::cvtColor(pixels, pixels, cv::COLOR_BGRA2RGBA);
    else
        cv::cvtColor(pixels, pixels, cv::COLOR_GRAY2RGBA);
    cv::flip(pixels, pixels, 0);
    GLuint id;
    glGenTextures(1, &id);
    glBindTexture(GL_TEXTURE_2D, id);
    glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_SRGB8_ALPHA8, pixels.cols, pixels.rows, 0, GL_RGBA, GL_UNSIGNED_BYTE,
                 pixels.data);
    glGenerateMipmap(GL_TEXTURE_2D);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    textures[file] = id;
    return id;
}
std::vector<std::shared_ptr<Mesh>> Renderer::load(const std::string &name, const std::string &overrideTexture) {
    const std::string key = name + overrideTexture;
    if (cache.count(key))
        return cache.at(key);
    Assimp::Importer importer;
    // These assets are authored in ROS Z-up. Do not apply Assimp's Z-up -> Y-up
    // conversion.
    importer.SetPropertyBool(AI_CONFIG_IMPORT_COLLADA_IGNORE_UP_DIRECTION, true);
    const auto path = std::filesystem::path(name).is_absolute() ? std::filesystem::path(name)
                                                                : std::filesystem::path(meshRoot) / name / "model.dae";
    const aiScene *scene = importer.ReadFile(path.string(), aiProcess_Triangulate | aiProcess_GenSmoothNormals |
                                                                aiProcess_JoinIdenticalVertices);
    if (!scene || !scene->mRootNode)
        throw std::runtime_error(path.string() + ": " + importer.GetErrorString());
    std::vector<std::shared_ptr<Mesh>> result;
    std::function<void(const aiNode *, glm::mat4)> visit = [&](const aiNode *node, glm::mat4 parent) {
        const glm::mat4 transform = parent * aiMatrix(node->mTransformation);
        const glm::mat3 normal = glm::transpose(glm::inverse(glm::mat3(transform)));
        for (unsigned j = 0; j < node->mNumMeshes; ++j) {
            auto *input = scene->mMeshes[node->mMeshes[j]];
            std::vector<Vertex> v;
            std::vector<unsigned> idx;
            v.reserve(input->mNumVertices);
            for (unsigned k = 0; k < input->mNumVertices; ++k) {
                const auto &a = input->mVertices[k];
                const auto &n = input->mNormals[k];
                glm::vec2 uv(0);
                if (input->HasTextureCoords(0))
                    uv = {input->mTextureCoords[0][k].x, input->mTextureCoords[0][k].y};
                v.push_back({glm::vec3(transform * glm::vec4(a.x, a.y, a.z, 1)),
                             glm::normalize(normal * glm::vec3(n.x, n.y, n.z)), uv});
            }
            for (unsigned k = 0; k < input->mNumFaces; ++k)
                for (unsigned l = 0; l < input->mFaces[k].mNumIndices; ++l)
                    idx.push_back(input->mFaces[k].mIndices[l]);
            if (idx.empty())
                continue;
            // These exports assign the vinyl material to the entire prop, with
            // every body UV at (0, 0). That transparent texel erases the solid body.
            // Keep the printed faces textured and restore the CAD's dark plastic
            // material on the unmapped sides and bottom.
            if (name == "table_pill" || name == "table_nut_and_bolt") {
                std::vector<unsigned> bodyIndices, vinylIndices;
                for (size_t k = 0; k < idx.size(); k += 3) {
                    bool unmapped = true;
                    for (size_t corner = 0; corner < 3; ++corner)
                        unmapped &= glm::length(v[idx[k + corner]].uv) < 1e-7f;
                    auto &destination = unmapped ? bodyIndices : vinylIndices;
                    destination.insert(destination.end(), idx.begin() + k, idx.begin() + k + 3);
                }
                if (!bodyIndices.empty()) {
                    auto bodyMesh = std::make_shared<Mesh>(v, bodyIndices);
                    bodyMesh->color = {.003442196f, .003442196f, .003442196f, 1};
                    triangles += bodyIndices.size() / 3;
                    result.push_back(bodyMesh);
                }
                idx = std::move(vinylIndices);
                if (idx.empty())
                    continue;
            }
            // The CAD merges an untextured backing sheet (x ~= -4 mm) into the
            // support-frame mesh. Cutting only the textured front leaves solid depth
            // behind every opening. Separate that sheet and give it the same UVs.
            std::vector<Vertex> backingVertices;
            std::vector<unsigned> backingIndices;
            if (name == "torpedo" && idx.size() > 6) {
                std::vector<unsigned> supportIndices;
                for (size_t k = 0; k < idx.size(); k += 3) {
                    bool panel = true;
                    for (size_t corner = 0; corner < 3; ++corner) {
                        const auto p = v[idx[k + corner]].p;
                        panel &= std::abs(p.x) < .01f && std::abs(p.y) <= .30481f && std::abs(p.z) <= .30481f;
                    }
                    for (size_t corner = 0; corner < 3; ++corner) {
                        if (panel) {
                            auto vertex = v[idx[k + corner]];
                            vertex.uv = glm::vec2(vertex.p.y, vertex.p.z) / .6096f + .5f;
                            backingIndices.push_back(backingVertices.size());
                            backingVertices.push_back(vertex);
                        } else
                            supportIndices.push_back(idx[k + corner]);
                    }
                }
                idx = std::move(supportIndices);
            }
            auto mesh = std::make_shared<Mesh>(v, idx);
            if (name == "torpedo" && idx.size() == 6)
                mesh->holes = torpedoHoles;
            triangles += idx.size() / 3;
            aiColor4D color(1, 1, 1, 1);
            auto *material = scene->mMaterials[input->mMaterialIndex];
            aiGetMaterialColor(material, AI_MATKEY_COLOR_DIFFUSE, &color);
            float opacity = 1.f;
            material->Get(AI_MATKEY_OPACITY, opacity);
            color.a = std::min(color.a, opacity);
            mesh->color = {color.r, color.g, color.b, color.a};
            if (!backingIndices.empty()) {
                auto backing = std::make_shared<Mesh>(backingVertices, backingIndices);
                backing->holes = torpedoHoles;
                backing->color = mesh->color;
                triangles += backingIndices.size() / 3;
                result.push_back(backing);
            }
            aiString tex;
            if (material->GetTexture(aiTextureType_DIFFUSE, 0, &tex) == AI_SUCCESS) {
                std::filesystem::path t = overrideTexture.empty() ? std::filesystem::path(tex.C_Str())
                                                                  : std::filesystem::path(overrideTexture);
                // Exporters sometimes leave absolute author-machine paths in Collada.
                auto local = path.parent_path() / t.filename();
                mesh->texture = texture(local.string());
                mesh->color = {1, 1, 1, 1};
            }
            result.push_back(mesh);
        }
        for (unsigned k = 0; k < node->mNumChildren; ++k)
            visit(node->mChildren[k], transform);
    };
    visit(scene->mRootNode, glm::mat4(1));
    cache[key] = result;
    return result;
}
void Renderer::box(const std::string &name, const glm::mat4 &matrix, const glm::vec3 &size, const glm::vec3 &color,
                   int material, bool castsShadow) {
    auto mesh = cube();
    mesh->color = glm::vec4(color, 1);
    objects.push_back({name, {mesh}, glm::scale(matrix, size), material, false, false, castsShadow});
    triangles += 12;
}
void Renderer::buildPool() {
    // Dimensions and lane markings match the legacy pool (metres, water z=0).
    const float length = world["length"].as<float>(50), width = world["width"].as<float>(22.86f),
                depth = world["depth"].as<float>(2.1336f), deck = world["deck_height"].as<float>(.305288888f);
    auto at = [&](float x, float y, float z) {
        return poolToMap * pose({x, y, z + world["water_level"].as<float>(0)});
    };
    box("Pool floor", at(length / 2, width / 2, -depth - .12f), {length, width, .24f}, {.68, .85, .87}, 1);
    box("Near wall", at(length / 2, -.15f, (deck - depth) / 2), {length, .3f, depth + deck}, {.68, .85, .87}, 1);
    box("Far wall", at(length / 2, width + .15f, (deck - depth) / 2), {length, .3f, depth + deck}, {.68, .85, .87}, 1);
    box("End wall", at(-.15f, width / 2, (deck - depth) / 2), {.3f, width, depth + deck}, {.68, .85, .87}, 1);
    box("End wall", at(length + .15f, width / 2, (deck - depth) / 2), {.3f, width, depth + deck}, {.68, .85, .87}, 1);
    for (float y : {-1.5f, width + 1.5f})
        box("Deck", at(length / 2, y < 0 ? -1.8f : width + 1.8f, deck - .15f), {length + 6.6f, 3, .3f}, {.73, .76, .73},
            2);
    for (float x : {-1.5f, length + 1.5f})
        box("Deck", at(x < 0 ? -1.8f : length + 1.8f, width / 2, deck - .15f), {3, width + .6f, .3f}, {.73, .76, .73},
            2);
    for (float y : {-.10f, width + .10f})
        box("Coping", at(length / 2, y, deck + .02f), {length, .22f, .055f}, {.9, .91, .86});
    for (float x : {-.10f, length + .10f})
        box("Coping", at(x, width / 2, deck + .02f), {.22f, width, .055f}, {.9, .91, .86});
    // Lighting is procedural; no building geometry obstructs pool or sensor
    // views.
    auto surface = std::make_shared<Mesh>(std::vector<Vertex>{{{0, 0, 0}, {0, 0, 1}, {0, 0}},
                                                              {{length, 0, 0}, {0, 0, 1}, {1, 0}},
                                                              {{length, width, 0}, {0, 0, 1}, {1, 1}},
                                                              {{0, width, 0}, {0, 0, 1}, {0, 1}}},
                                          std::vector<unsigned>{0, 1, 2, 0, 2, 3});
    water = {"Water", {surface}, poolToMap * pose({0, 0, world["water_level"].as<float>(0)})};
}
Renderer::Renderer(const std::string &shaders, const std::string &meshes, const std::string &textureFolder,
                   const std::string &mapping, const std::string &markers, const std::string &sceneFile,
                   const std::string &robot, const std::string &robotAsset, const std::string &taskConfig,
                   const std::string &payloadAsset, const std::string &launcherAsset, const std::string &clawAsset,
                   const std::vector<StatusLight> &statusLights)
    : meshRoot(meshes), textureRoot(textureFolder) {
    sceneProgram = program(shaders, "scene");
    waterProgram = program(shaders, "water");
    shadowProgram = program(shaders, "shadow");
    postProgram = program(shaders, "post");
    bloomProgram = program(shaders, "bloom");
    pointProgram = program(shaders, "points");
    YAML::Node task;
    if (!taskConfig.empty()) {
        task = YAML::LoadFile(taskConfig);
        if (task["magnet_lights"]) {
            ledRadiance = task["magnet_lights"]["led_radiance"].as<float>(60.f);
            if (!std::isfinite(ledRadiance) || ledRadiance <= 0 || ledRadiance > 200)
                throw std::runtime_error("LED radiance must be in (0,200]");
        }
        if (task["torpedo"])
            for (const auto &h : task["torpedo"]["holes"])
                torpedoHoles.push_back({h["uv"][0].as<float>(), h["uv"][1].as<float>(), h["radius_uv"].as<float>()});
        if (torpedoHoles.size() > 4)
            throw std::runtime_error("The torpedo shader supports up to four holes");
    }
    glGenVertexArrays(1, &quad);
    const auto map = YAML::LoadFile(mapping);
    const auto frame = map["/**/zed_faker"]["ros__parameters"];
    if (frame && frame["config_frame"].as<std::string>("") == "tag") {
        auto origin = vector3(frame["map_origin_pool"]);
        mapToPool = pose({origin.x, origin.y, 0}, {0, 0, glm::radians(origin.z)});
        poolToMap = glm::inverse(mapToPool);
    }
    world = YAML::LoadFile(sceneFile)["world"];
    buildPool();
    const auto data = map["/" + robot + "/riptide_mapping2"]["ros__parameters"]["init_data"];
    if (!data)
        throw std::runtime_error("Mapping configuration has no init_data for " + robot);
    std::set<std::string> visiting;
    std::function<glm::mat4(std::string)> resolve = [&](std::string key) {
        if (key == "map" || key == "world")
            return glm::mat4(1);
        if (key.size() > 6 && key.substr(key.size() - 6) == "_frame")
            key.resize(key.size() - 6);
        if (landmarks.count(key))
            return landmarks.at(key);
        if (!visiting.insert(key).second)
            throw std::runtime_error("Cycle in mapping frames: " + key);
        auto entry = data[key];
        if (!entry)
            throw std::runtime_error("Unknown mapping frame: " + key);
        auto p = entry["pose"];
        auto t = resolve(entry["parent"].as<std::string>()) *
                 pose({p["x"].as<float>(0), p["y"].as<float>(0), p["z"].as<float>(0)},
                      {0, 0, glm::radians(p["yaw"].as<float>(0))});
        visiting.erase(key);
        landmarks[key] = t;
        return t;
    };
    const auto config = YAML::LoadFile(markers)["/**/marker_publisher"]["ros__parameters"]["markers"];
    for (const auto &item : config) {
        auto m = item.second;
        std::string name = m["mesh"].as<std::string>();
        if (name == "cube" || name == "sphere" || name == "arrow")
            continue;
        std::string key = m["frame"].as<std::string>();
        glm::mat4 t = resolve(key) * yamlPose(m["pose"]);
        if (m["scale"])
            t = glm::scale(t, vector3(m["scale"]));
        if (name == "bin_magnet" && task["magnet_lights"]) {
            const auto lights = task["magnet_lights"];
            const std::string target = key.substr(0, key.size() - 6);
            if (lights["targets"][target]) {
                t *= yamlPose(lights["face_pose"]);
                const auto folder = std::filesystem::path(textureRoot).parent_path() / "models/magnet_lights";
                objects.push_back({target + "/housing", load((folder / "housing.glb").string()), t});
                objects.push_back(
                    {target + "/cover", load((folder / "cover.glb").string()), t, 5, false, false, false});
                objects.push_back({target + "/LEDs", load((folder / "leds.glb").string()), t, 6, false, false, false});
                magnetLight(target, lights["targets"][target].as<std::string>() == "green");
                continue;
            }
        }
        std::string overrideTex;
        if (name == "bin_vinyl") {
            key = key.substr(0, key.size() - 6);
            const auto cls = data[key]["class"].as<std::string>("");
            if (cls == "blood")
                overrideTex = "Task3_Blood_Fixed.png";
            if (cls == "fire")
                overrideTex = "Task3_Fire_Fixed.png";
        }
        objects.push_back({key, load(name, overrideTex), t});
    }
    auto scene = YAML::LoadFile(sceneFile);
    for (const auto &entity : scene["entities"]) {
        const auto name = entity["id"].as<std::string>();
        auto transform = resolve(entity["frame"].as<std::string>("map"));
        if (entity["pose"])
            transform *= yamlPose(entity["pose"]);
        landmarks[name] = transform;
        const auto size = vector3(entity["size"]);
        if (entity["mesh"])
            objects.push_back({name, load(entity["mesh"].as<std::string>()), glm::scale(transform, size)});
        else
            box(name, transform, size, entity["color"] ? vector3(entity["color"]) : glm::vec3(.3f, .7f, .8f));
    }
    if (task["octagon"]) {
        resolve("octagon");
        buildOctagon(task["octagon"]);
    }
    if (task["crate"])
        buildCrates(task["crate"]);
    if (!payloadAsset.empty())
        payloadMeshes = load(payloadAsset);
    if (!launcherAsset.empty())
        objects.push_back({"Payload launcher", load(launcherAsset), glm::mat4(1), 0, true});
    if (!clawAsset.empty() && task["claw"]) {
        const auto cfg = task["claw"];
        clawMinGap = cfg["min_gap"].as<float>();
        objects.push_back({"Claw left pad", load(clawAsset), glm::mat4(1)});
        objects.push_back({"Claw right pad",
                           load((std::filesystem::path(clawAsset).parent_path() / "gripper_right.glb").string()),
                           glm::mat4(1)});
        const auto folder = std::filesystem::path(clawAsset).parent_path();
        objects.push_back({"Claw assembly", load((folder / "assembly_static.glb").string()), glm::mat4(1)});
        objects.push_back({"Claw left carrier", load((folder / "assembly_left.glb").string()), glm::mat4(1)});
        objects.push_back({"Claw right carrier", load((folder / "assembly_right.glb").string()), glm::mat4(1)});
        for (const auto &o : objects)
            if (o.name.rfind("Claw ", 0) == 0)
                clawParts[o.name] = o.transform;
    }
    auto robotInfo = scene["robot"]["model"];
    std::string robotMesh = robotInfo["riptide_mesh"].as<std::string>(robot);
    objects.push_back({"Vehicle", load(robotAsset.empty() ? robotMesh : robotAsset), glm::mat4(1), 0, true});
    for (const auto &light : statusLights) {
        box("Status light/" + light.id, light.mount, light.size, glm::vec3(1), 6, false);
        auto &emitter = objects.back();
        emitter.robot = true;
        emitter.robotMount = emitter.transform;
        emitter.radiance = light.radiance;
        emitter.tint = glm::vec4(0, 0, 0, 1);
    }
    if (task["magnet_lights"]) {
        const auto file = std::filesystem::path(textureRoot).parent_path() / "models/magnet_lights/robot_magnet.glb";
        objects.push_back({"Robot magnet", load(file.string()), glm::mat4(1), 0, true});
    }
    // Real calibration board: local +X is the printed face normal; UVs match
    // legacy.
    auto board = std::make_shared<Mesh>(std::vector<Vertex>{{{0, -.3048f, -.4572f}, {1, 0, 0}, {0, 0}},
                                                            {{0, .3048f, -.4572f}, {1, 0, 0}, {1, 0}},
                                                            {{0, .3048f, .4572f}, {1, 0, 0}, {1, 1}},
                                                            {{0, -.3048f, .4572f}, {1, 0, 0}, {0, 1}}},
                                        std::vector<unsigned>{0, 1, 2, 0, 2, 3});
    board->texture = texture(textureRoot + "/objects/April Tag.jpg");
    objects.push_back({"Calibration", {board}, pose({.002f, 0, -.345f}), 0, false, true});
    // Double the spatial detail without allocating an unused HDR color target.
    shadow.resize(4096, 4096, false, true);
    bindTexture(shadow.depth, 0);
    // Filter depth comparisons, not raw depths, for continuous shadow edges.
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_COMPARE_MODE, GL_COMPARE_REF_TO_TEXTURE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_COMPARE_FUNC, GL_LEQUAL);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_BORDER);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_BORDER);
    const float litBorder[] = {1.f, 1.f, 1.f, 1.f};
    glTexParameterfv(GL_TEXTURE_2D, GL_TEXTURE_BORDER_COLOR, litBorder);
    reflection.resize(640, 400);
}
void Renderer::buildOctagon(const YAML::Node &config) {
    const float apothem = config["apothem"].as<float>();
    const float radius = config["pipe_radius"].as<float>();
    auto mount = landmarks.at("octagon");
    // The existing octagon frame locates the hanging signs below the water.
    mount[3].z = config["surface_z"].as<float>();
    auto tube = [&](const std::string &name, glm::vec3 a, glm::vec3 b, float r) {
        const glm::vec3 axis = glm::normalize(b - a);
        const glm::vec3 u =
            glm::normalize(glm::cross(axis, std::abs(axis.z) < .9f ? glm::vec3(0, 0, 1) : glm::vec3(0, 1, 0)));
        const glm::vec3 v = glm::cross(axis, u);
        std::vector<Vertex> vertices;
        std::vector<unsigned> indices;
        for (int i = 0; i <= 16; ++i) {
            const float angle = 2 * glm::pi<float>() * i / 16;
            const auto normal = u * std::cos(angle) + v * std::sin(angle);
            vertices.push_back({a + r * normal, normal, {float(i) / 16, 0}});
            vertices.push_back({b + r * normal, normal, {float(i) / 16, 1}});
            if (i < 16)
                for (unsigned j : {0u, 1u, 3u, 0u, 3u, 2u})
                    indices.push_back(2 * i + j);
        }
        auto mesh = std::make_shared<Mesh>(vertices, indices);
        mesh->color = {.94f, .95f, .91f, 1};
        triangles += indices.size() / 3;
        objects.push_back({name, {mesh}, mount});
    };
    const float outer = apothem / std::cos(glm::pi<float>() / 8);
    for (int i = 0; i < 8; ++i) {
        const float a = (i + .5f) * glm::pi<float>() / 4, b = (i + 1.5f) * glm::pi<float>() / 4;
        tube("Octagon PVC " + std::to_string(i), {outer * std::cos(a), outer * std::sin(a), 0},
             {outer * std::cos(b), outer * std::sin(b), 0}, radius);
    }
    // Four inward-facing 12-inch signs already come from the marker assets.
    for (const auto &key : {"compass", "hammer_and_wrench", "buoy", "sos"}) {
        if (!landmarks.count(key))
            continue;
        auto sign = glm::inverse(mount) * landmarks.at(key);
        glm::vec3 bottom(sign[3]);
        bottom.z += .1524f;
        tube(std::string("Octagon sign hanger ") + key, {bottom.x, bottom.y, 0}, bottom, .002f);
    }
}

void Renderer::buildCrates(const YAML::Node &config) {
    const float outer = config["outer_width"].as<float>(), inner = config["inner_width"].as<float>();
    const float base = config["base_thickness"].as<float>();
    const float height = config["outer_height"].as<float>() - base;
    const float liner = config["liner_thickness"].as<float>();
    const float linerHeight = height * config["liner_height_fraction"].as<float>();
    std::array<std::vector<Vertex>, 2> vertices;
    std::array<std::vector<unsigned>, 2> indices;
    auto add = [&](int group, const glm::mat4 &t, glm::vec3 size) {
        auto &v = vertices[group];
        auto &idx = indices[group];
        for (int axis = 0; axis < 3; ++axis)
            for (int sign : {-1, 1}) {
                glm::vec3 n(0), u(0), w(0);
                n[axis] = sign;
                u[(axis + 1) % 3] = 1;
                w[(axis + 2) % 3] = sign;
                unsigned start = v.size();
                for (glm::vec2 uv : {glm::vec2(0, 0), glm::vec2(1, 0), glm::vec2(1, 1), glm::vec2(0, 1)})
                    v.push_back({glm::vec3(t * glm::vec4((n * .5f + (uv.x - .5f) * u + (uv.y - .5f) * w) * size, 1)),
                                 glm::mat3(t) * n, uv});
                for (unsigned i : {0u, 1u, 2u, 0u, 2u, 3u})
                    idx.push_back(start + i);
            }
    };
    for (int i = 1; i <= 4; ++i) {
        const auto key = "bin_vinyl" + std::to_string(i);
        if (!landmarks.count(key))
            continue;
        const auto root = landmarks.at(key);
        auto part = [&](int material, glm::vec3 p, glm::vec3 size) { add(material, root * pose(p), size); };
        part(0, {0, 0, -base / 2 - .0005f}, {outer, outer, base});
        for (int axis = 0; axis < 2; ++axis)
            for (int sign : {-1, 1}) {
                auto wall = [&](int material, float across, float along, float z, float thick, float length, float h) {
                    glm::vec3 p(0, 0, z), size(thick, thick, h);
                    p[axis] = across;
                    p[1 - axis] = along;
                    size[1 - axis] = length;
                    part(material, p, size);
                };
                const float edge = sign * (outer / 2 - .007f);
                // Navy lattice, horizontal rails and sturdy corner posts.
                for (int bar = 0; bar <= 8; ++bar)
                    wall(0, edge, -outer / 2 + .007f + bar * (outer - .014f) / 8, height / 2, .008f, .009f, height);
                for (int bar = 0; bar <= 5; ++bar)
                    wall(0, edge, 0, .01f + bar * (height - .02f) / 5, .012f, outer, .012f);
                wall(0, edge, 0, height - .008f, .016f, outer, .016f);
                // White corrugated liner covers the lower half only, inside the
                // lattice.
                wall(1, sign * (inner / 2 - liner / 2), 0, linerHeight / 2, liner, inner, linerHeight);
            }
    }
    for (int group = 0; group < 2; ++group) {
        auto mesh = std::make_shared<Mesh>(vertices[group], indices[group]);
        mesh->color = group ? glm::vec4(.92, .94, .91, 1) : glm::vec4(.025, .055, .13, 1);
        triangles += indices[group].size() / 3;
        objects.push_back({group ? "Crate liners" : "CleverMade crates", {mesh}, glm::mat4(1), group ? 4 : 0});
    }
}
void Renderer::objectPose(const std::string &name, const glm::mat4 &p) {
    for (auto &o : objects)
        if (o.name == name)
            o.transform = p;
}
void Renderer::clawPose(const glm::mat4 &mount, float left, float right) {
    for (auto &o : objects) {
        auto part = clawParts.find(o.name);
        if (part == clawParts.end())
            continue;
        glm::mat4 local(1);
        if (o.name.find("left") != std::string::npos)
            local = pose({0, left, 0});
        else if (o.name.find("right") != std::string::npos)
            local = pose({0, -right, 0});
        o.transform = mount * local * part->second;
    }
}
void Renderer::payloadPoses(const std::vector<glm::mat4> &poses) {
    objects.erase(std::remove_if(objects.begin(), objects.end(), [](const Object &o) { return o.name == "Payload"; }),
                  objects.end());
    for (const auto &p : poses)
        objects.push_back({"Payload", payloadMeshes, p});
}

Renderer::~Renderer() {
    objects.clear();
    cache.clear();
    water.meshes.clear();
    for (auto &t : textures)
        glDeleteTextures(1, &t.second);
    for (GLuint p : {sceneProgram, waterProgram, shadowProgram, postProgram, pointProgram, bloomProgram})
        glDeleteProgram(p);
    for (auto &pc : pointClouds) {
        if (pc.vbo)
            glDeleteBuffers(1, &pc.vbo);
        if (pc.vao)
            glDeleteVertexArrays(1, &pc.vao);
    }
    glDeleteVertexArrays(1, &quad);
}
void Renderer::robotPose(const glm::mat4 &p) {
    for (auto &o : objects)
        if (o.robot)
            o.transform = p * o.robotMount;
}
void Renderer::statusLight(const std::string &id, const glm::vec3 &color) {
    for (auto &o : objects)
        if (o.name == "Status light/" + id)
            o.tint = glm::vec4(color, 1);
}
void Renderer::magnetPose(const glm::mat4 &mount) {
    for (auto &o : objects)
        if (o.name == "Robot magnet")
            o.transform = mount;
}
void Renderer::magnetLight(const std::string &name, bool green) {
    for (auto &o : objects)
        if (o.name == name + "/LEDs")
            o.tint = green ? glm::vec4(.002f, 1.f, .004f, 1.f) : glm::vec4(1.f, .001f, .002f, 1.f);
}
void Renderer::shadows(const Look &look) {
    glm::vec3 center(poolToMap * glm::vec4(world["length"].as<float>(50) / 2, world["width"].as<float>(22.86f) / 2,
                                           world["water_level"].as<float>(0), 1));
    auto sun = look.outdoor ? look.sunDirection() : glm::normalize(glm::vec3(-.2f, -.1f, 1));
    lightMatrix =
        glm::ortho(-33.f, 33.f, -33.f, 33.f, .1f, 140.f) * glm::lookAt(center + sun * 60.f, center, glm::vec3(0, 1, 0));
    glBindFramebuffer(GL_FRAMEBUFFER, shadow.fbo);
    glViewport(0, 0, shadow.width, shadow.height);
    glDepthMask(GL_TRUE);
    glClear(GL_DEPTH_BUFFER_BIT);
    if (!look.shadows)
        return;
    glEnable(GL_DEPTH_TEST);
    glDisable(GL_BLEND);
    glDisable(GL_CULL_FACE);
    glEnable(GL_POLYGON_OFFSET_FILL);
    glPolygonOffset(2.f, 4.f);
    glUseProgram(shadowProgram);
    uniform(shadowProgram, "lightMatrix", lightMatrix);
    for (const auto &o : objects) {
        if (!o.castsShadow || (o.tag && !look.tag))
            continue;
        uniform(shadowProgram, "model", o.transform);
        // Off-screen objects can still cast visible shadows. Cull against the
        // light's volume, independently of every viewer/sensor camera.
        const Frustum frustum(lightMatrix * o.transform);
        for (const auto &m : o.meshes) {
            // Clear CAD panels must not cast opaque shadows onto internal LEDs.
            if (m->color.a < .999f)
                continue;
            if (!frustum.intersects(m->bounds))
                continue;
            bindTexture(m->texture, 0);
            integer(shadowProgram, "hasTexture", m->texture != 0);
            integer(shadowProgram, "holeCount", int(m->holes.size()));
            if (!m->holes.empty())
                glUniform3fv(glGetUniformLocation(shadowProgram, "holes"), m->holes.size(),
                             glm::value_ptr(m->holes[0]));
            m->draw();
        }
    }
    glDisable(GL_POLYGON_OFFSET_FILL);
}
void Renderer::drawScene(const View &camera, const Look &look, float time, bool showRobot, bool clip) {
    glUseProgram(sceneProgram);
    uniform(sceneProgram, "view", camera.view);
    uniform(sceneProgram, "projection", camera.projection);
    uniform(sceneProgram, "eye", camera.eye);
    uniform(sceneProgram, "waterLevel", world["water_level"].as<float>(0));
    uniform(
        sceneProgram, "poolSize",
        glm::vec3(world["length"].as<float>(50), world["width"].as<float>(22.86f), world["depth"].as<float>(2.1336f)));
    uniform(sceneProgram, "lightMatrix", lightMatrix);
    uniform(sceneProgram, "mapToPool", mapToPool);
    uniform(sceneProgram, "time", time);
    uniform(sceneProgram, "ledRadiance", ledRadiance);
    uniform(sceneProgram, "waterTint", look.water.tint);
    uniform(sceneProgram, "waterAbsorption", look.water.absorption);
    uniform(sceneProgram, "waterScattering", look.water.scattering);
    uniform(sceneProgram, "waterDistanceScale", look.water.distanceScale);
    uniform(sceneProgram, "waterDistancePower", look.water.distancePower);
    uniform(sceneProgram, "waterClearDistance", look.water.clearDistance);
    uniform(sceneProgram, "caustics", look.caustics);
    uniform(sceneProgram, "sunDirection",
            look.outdoor ? look.sunDirection() : glm::normalize(glm::vec3(-.2f, -.1f, 1)));
    uniform(sceneProgram, "directLight", look.directLight);
    uniform(sceneProgram, "ambientLight", look.ambientLight);
    integer(sceneProgram, "outdoor", look.outdoor);
    integer(sceneProgram, "useShadow", look.shadows);
    integer(sceneProgram, "clipWater", clip);
    integer(sceneProgram, "albedo", 0);
    integer(sceneProgram, "shadowMap", 1);
    bindTexture(shadow.depth, 1);
    const auto viewProjection = camera.projection * camera.view;
    // Clear polycarbonate is blended after opaque electronics and LED lenses.
    // It writes the front-cover depth, so RGB/depth still describe one enclosure.
    for (int transparent = 0; transparent < 2; ++transparent) {
        if (transparent) {
            glEnable(GL_BLEND);
            glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
        }
        for (const auto &o : objects) {
            if ((o.robot && !showRobot) || (o.tag && !look.tag))
                continue;
            // Keep overhead spectator views clear; the ceiling remains in sensor
            // views/reflections.
            if (!clip && camera.eye.z > 4.5f && (o.name == "Roof beam" || o.name == "Lighting"))
                continue;
            const Frustum frustum(viewProjection * o.transform);
            bool modelBound = false;
            for (const auto &m : o.meshes) {
                const int material = o.material == 0 && m->color.a < .999f ? 5 : o.material;
                if ((material == 5) != bool(transparent))
                    continue;
                if (!frustum.intersects(m->bounds))
                    continue;
                if (!modelBound) {
                    uniform(sceneProgram, "model", o.transform);
                    uniform(sceneProgram, "ledRadiance", o.radiance < 0 ? ledRadiance : o.radiance);
                    modelBound = true;
                }
                integer(sceneProgram, "material", material);
                const auto tint = m->color * o.tint;
                glUniform4fv(glGetUniformLocation(sceneProgram, "tint"), 1, glm::value_ptr(tint));
                integer(sceneProgram, "hasTexture", m->texture != 0);
                bindTexture(m->texture, 0);
                integer(sceneProgram, "holeCount", int(m->holes.size()));
                if (!m->holes.empty())
                    glUniform3fv(glGetUniformLocation(sceneProgram, "holes"), m->holes.size(),
                                 glm::value_ptr(m->holes[0]));
                m->draw();
            }
        }
    }
    glDisable(GL_BLEND);
}
void Renderer::pointCloud(int slot, const std::vector<float> &xyzrgb, const glm::mat4 &transform,
                          const glm::vec3 &highlight) {
    if (slot < 0)
        return;
    if (size_t(slot) >= pointClouds.size())
        pointClouds.resize(slot + 1);
    auto &pc = pointClouds.at(slot);
    if (!pc.vao) {
        glGenVertexArrays(1, &pc.vao);
        glGenBuffers(1, &pc.vbo);
        glBindVertexArray(pc.vao);
        glBindBuffer(GL_ARRAY_BUFFER, pc.vbo);
        glEnableVertexAttribArray(0);
        glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), nullptr);
        glEnableVertexAttribArray(1);
        glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), reinterpret_cast<void *>(3 * sizeof(float)));
        glBindVertexArray(0);
    }
    glBindBuffer(GL_ARRAY_BUFFER, pc.vbo);
    glBufferData(GL_ARRAY_BUFFER, xyzrgb.size() * sizeof(float), xyzrgb.data(), GL_DYNAMIC_DRAW);
    glBindBuffer(GL_ARRAY_BUFFER, 0);
    pc.count = int(xyzrgb.size() / 6);
    pc.transform = transform;
    pc.highlight = highlight;
}
void Renderer::clearPointCloud(int slot) {
    if (slot >= 0 && size_t(slot) < pointClouds.size())
        pointClouds[slot].count = 0;
}
void Renderer::drawPoints(const View &camera, const Look &look) {
    bool any = false;
    for (const auto &pc : pointClouds)
        any = any || pc.count > 0;
    if (!any)
        return;
    glUseProgram(pointProgram);
    uniform(pointProgram, "view", camera.view);
    uniform(pointProgram, "projection", camera.projection);
    uniform(pointProgram, "pointSize", pointSize);
    uniform(pointProgram, "exposure", look.exposure);
    uniform(pointProgram, "highlight", pointHighlight);
    glEnable(GL_PROGRAM_POINT_SIZE);
    glDepthFunc(GL_LEQUAL);
    for (const auto &pc : pointClouds) {
        if (!pc.count)
            continue;
        uniform(pointProgram, "model", pc.transform);
        uniform(pointProgram, "highlightColor", pc.highlight);
        glBindVertexArray(pc.vao);
        glDrawArrays(GL_POINTS, 0, pc.count);
    }
    glBindVertexArray(0);
    glDepthFunc(GL_LESS);
    glDisable(GL_PROGRAM_POINT_SIZE);
}
void Renderer::render(Frame &f, const View &camera, const Look &look, float time, bool showRobot, bool reflect,
                      bool overlays) {
    glEnable(GL_DEPTH_TEST);
    glDepthMask(GL_TRUE);
    glDisable(GL_BLEND);
    glDisable(GL_CULL_FACE);
    const bool surfaceVisible =
        look.surface &&
        Frustum(camera.projection * camera.view * water.transform).intersects(water.meshes.front()->bounds);
    const bool reflectionActive = reflect && surfaceVisible && camera.eye.z > world["water_level"].as<float>(0);
    if (reflectionActive) {
        reflection.resize(std::max(160, f.opaque.width / 2), std::max(100, f.opaque.height / 2));
        glBindFramebuffer(GL_FRAMEBUFFER, reflection.fbo);
        glViewport(0, 0, reflection.width, reflection.height);
        glClearColor(look.outdoor ? .30f : .13f, look.outdoor ? .48f : .16f, look.outdoor ? .68f : .18f, 1);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        glm::mat4 mirror =
            pool::pose({0, 0, 2 * world["water_level"].as<float>(0)}) * glm::scale(glm::mat4(1), glm::vec3(1, 1, -1));
        View reflected = camera;
        reflected.eye.z = 2 * world["water_level"].as<float>(0) - reflected.eye.z;
        reflected.view = camera.view * mirror;
        drawScene(reflected, look, time, showRobot, true);
    }
    glBindFramebuffer(GL_FRAMEBUFFER, f.opaque.fbo);
    glViewport(0, 0, f.opaque.width, f.opaque.height);
    glClearColor(look.outdoor ? .30f : .13f, look.outdoor ? .48f : .16f, look.outdoor ? .68f : .18f, 1);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    drawScene(camera, look, time, showRobot);
    // Drawn into the opaque pass so the water surface and post effects treat the
    // samples like scene geometry, but never for sensor or reflection renders.
    if (overlays)
        drawPoints(camera, look);
    glBindFramebuffer(GL_READ_FRAMEBUFFER, f.opaque.fbo);
    glBindFramebuffer(GL_DRAW_FRAMEBUFFER, f.composite.fbo);
    glBlitFramebuffer(0, 0, f.opaque.width, f.opaque.height, 0, 0, f.opaque.width, f.opaque.height,
                      GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT, GL_NEAREST);
    glBindFramebuffer(GL_FRAMEBUFFER, f.composite.fbo);
    if (surfaceVisible) {
        glUseProgram(waterProgram);
        uniform(waterProgram, "model", water.transform);
        uniform(waterProgram, "view", camera.view);
        uniform(waterProgram, "projection", camera.projection);
        uniform(waterProgram, "eye", camera.eye);
        uniform(waterProgram, "waterLevel", world["water_level"].as<float>(0));
        uniform(waterProgram, "poolSize",
                glm::vec3(world["length"].as<float>(50), world["width"].as<float>(22.86f),
                          world["depth"].as<float>(2.1336f)));
        uniform(waterProgram, "time", time);
        uniform(waterProgram, "sunDirection", look.sunDirection());
        uniform(waterProgram, "directLight", look.directLight);
        uniform(waterProgram, "glare", look.outdoor ? look.glare : 0.f);
        uniform(waterProgram, "ambientLight", look.ambientLight);
        uniform(waterProgram, "waterTint", look.water.tint);
        integer(waterProgram, "outdoor", look.outdoor);
        uniform(waterProgram, "mapToPool", mapToPool);
        integer(waterProgram, "sceneColor", 0);
        integer(waterProgram, "reflectionColor", 1);
        integer(waterProgram, "sceneDepth", 2);
        integer(waterProgram, "hasReflection", reflectionActive);
        bindTexture(f.opaque.color, 0);
        bindTexture(reflection.color, 1);
        bindTexture(f.opaque.depth, 2);
        glUniform2f(glGetUniformLocation(waterProgram, "resolution"), f.opaque.width, f.opaque.height);
        glDepthMask(GL_FALSE);
        water.meshes.front()->draw();
        glDepthMask(GL_TRUE);
    }
    // Filter the HDR bright pass before tone mapping. A continuous low-resolution
    // blur avoids the replicated bars produced by sparse full-resolution rings.
    glDisable(GL_DEPTH_TEST);
    glUseProgram(bloomProgram);
    integer(bloomProgram, "source", 0);
    glBindVertexArray(quad);
    glViewport(0, 0, f.bloom[0].width, f.bloom[0].height);
    for (int pass = 0; pass < 3; ++pass) {
        glBindFramebuffer(GL_FRAMEBUFFER, f.bloom[pass % 2].fbo);
        integer(bloomProgram, "extractBright", pass == 0);
        bindTexture(pass == 0 ? f.composite.color : f.bloom[(pass - 1) % 2].color, 0);
        glUniform2f(glGetUniformLocation(bloomProgram, "stepSize"),
                    pass == 0 ? 1.f / f.composite.width : (pass == 1 ? 1.f / f.bloom[0].width : 0.f),
                    pass == 0 ? 1.f / f.composite.height : (pass == 2 ? 1.f / f.bloom[0].height : 0.f));
        glDrawArrays(GL_TRIANGLES, 0, 3);
    }
    glBindFramebuffer(GL_FRAMEBUFFER, f.final.fbo);
    glViewport(0, 0, f.final.width, f.final.height);
    glUseProgram(postProgram);
    integer(postProgram, "sceneColor", 0);
    uniform(postProgram, "exposure", look.exposure);
    bindTexture(f.composite.color, 0);
    bindTexture(f.composite.depth, 1);
    bindTexture(f.bloom[0].color, 2);
    integer(postProgram, "bloomColor", 2);
    integer(postProgram, "sceneDepth", 1);
    uniform(postProgram, "inverseViewProjection", glm::inverse(camera.projection * camera.view));
    uniform(postProgram, "eye", camera.eye);
    uniform(postProgram, "sunDirection", look.sunDirection());
    uniform(postProgram, "glare", look.outdoor ? look.glare * look.directLight : 0.f);
    glUniform2f(glGetUniformLocation(postProgram, "texel"), 1.f / f.opaque.width, 1.f / f.opaque.height);
    glBindVertexArray(quad);
    glDrawArrays(GL_TRIANGLES, 0, 3);
    glBindVertexArray(0);
    glBindFramebuffer(GL_FRAMEBUFFER, 0);
}
} // namespace pool
