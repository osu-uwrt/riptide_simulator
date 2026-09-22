#include "c_simulator/TaskContacts.h"
#include <algorithm>
#include <fstream>
#include <sstream>
#include <stdexcept>
using V = Eigen::Vector3d;
using Q = Eigen::Quaterniond;
namespace {
btVector3 b(const V &v) {
    return {btScalar(v.x()), btScalar(v.y()), btScalar(v.z())};
}
V e(const btVector3 &v) {
    return {v.x(), v.y(), v.z()};
}
btTransform transform(const TaskContacts::Pose &t) {
    Q q(t.rotation());
    return btTransform(btQuaternion(q.x(), q.y(), q.z(), q.w()), b(t.translation()));
}
V vec(const YAML::Node &n) {
    return {n[0].as<double>(), n[1].as<double>(), n[2].as<double>()};
}
} // namespace
TaskContacts::TaskContacts(const YAML::Node &cfg, const std::string &folder, const V &com,
                           const std::function<Pose(std::string)> &frame) {
    // Free props use the solid tabletop for support queries here. Their full
    // mesh/table dynamics run in ClawWorld. Re-testing every triangle of the
    // decorative table hardware at 1 kHz is both redundant and expensive.
    // Vehicle parts and carried objects still collide with the complete table.
    dispatcher.setNearCallback([](btBroadphasePair &pair, btCollisionDispatcher &d, const btDispatcherInfo &info) {
        auto *a =
            static_cast<Entry *>(static_cast<btCollisionObject *>(pair.m_pProxy0->m_clientObject)->getUserPointer());
        auto *b =
            static_cast<Entry *>(static_cast<btCollisionObject *>(pair.m_pProxy1->m_clientObject)->getUserPointer());
        if ((a->name == "table" && b->kind == 4 && !b->attached) ||
            (b->name == "table" && a->kind == 4 && !a->attached))
            return;
        btCollisionDispatcher::defaultNearCallback(pair, d, info);
    });
    const auto claw = cfg["claw"];
    auto p = claw["pose"];
    mount.translate(vec(p) - com);
    mount.rotate(Eigen::AngleAxisd(p[5].as<double>(), V::UnitZ()) * Eigen::AngleAxisd(p[4].as<double>(), V::UnitY()) *
                 Eigen::AngleAxisd(p[3].as<double>(), V::UnitX()));
    travel = (claw["max_gap"].as<double>() - claw["min_gap"].as<double>()) / 2;
    addMesh("left", folder + "/claw_pad.obj", mount, 1);
    addMesh("right", folder + "/claw_pad_right.obj", mount, 1);
    setJaws(0, 0);
    for (const auto &pair : std::vector<std::pair<std::string, std::string>>{
             {"table", "table"}, {"helmet", "table_basket_helmet"}, {"warning", "table_basket_warning"}})
        addMesh(pair.first, folder + "/" + pair.second + ".obj", frame(pair.first), 2);
    const auto solid = cfg["table_collision"];
    Pose slab = frame("table");
    slab.translate(solid ? vec(solid["slab_center"]) : V(0, 0, -.009525));
    addBox("table_solid", solid ? vec(solid["slab_size"]) : V(.635, .635, .01905), slab);
    for (const auto &pair : claw["props"]) {
        const auto name = pair.first.as<std::string>();
        addMesh(name, folder + "/" + pair.second["mesh"].as<std::string>() + ".obj", frame(name), 4);
    }
}
TaskContacts::~TaskContacts() {
    for (auto &x : entries)
        world.removeCollisionObject(&x->object);
}
void TaskContacts::insert(std::unique_ptr<Entry> x) {
    x->shape->setMargin(.0005);
    x->object.setCollisionShape(x->shape.get());
    x->object.setWorldTransform(transform(x->local));
    x->object.setUserPointer(x.get());
    // Dynamic proxies are moved explicitly before each detection pass.
    x->object.setCollisionFlags(x->kind == 2 ? btCollisionObject::CF_STATIC_OBJECT
                                             : btCollisionObject::CF_KINEMATIC_OBJECT);
    world.addCollisionObject(&x->object, x->kind, x->kind == 1 ? 6 : x->kind == 2 ? 5 : 3);
    entries.push_back(std::move(x));
}
void TaskContacts::addBox(const std::string &name, const V &size, const Pose &t, bool vehicle) {
    auto x = std::make_unique<Entry>();
    x->name = name;
    x->kind = vehicle ? 1 : 2;
    x->local = t;
    x->shape = std::make_unique<btBoxShape>(b(size / 2));
    insert(std::move(x));
}
void TaskContacts::addMesh(const std::string &name, const std::string &file, const Pose &t, int kind) {
    std::ifstream stream(file);
    if (!stream)
        throw std::runtime_error("Missing task collision mesh " + file);
    std::vector<V> vertices;
    std::vector<Eigen::Vector3i> faces;
    std::string line;
    while (std::getline(stream, line)) {
        std::istringstream in(line);
        std::string type;
        in >> type;
        if (type == "v") {
            V v;
            in >> v.x() >> v.y() >> v.z();
            vertices.push_back(v);
        } else if (type == "f") {
            std::string a, c, d;
            in >> a >> c >> d;
            faces.emplace_back(std::stoi(a) - 1, std::stoi(c) - 1, std::stoi(d) - 1);
        }
    }
    auto x = std::make_unique<Entry>();
    x->name = name;
    x->kind = kind;
    x->local = t;
    if (kind == 2) {
        x->triangles = std::make_unique<btTriangleMesh>();
        for (const auto &f : faces)
            x->triangles->addTriangle(b(vertices.at(f[0])), b(vertices.at(f[1])), b(vertices.at(f[2])));
        x->shape = std::make_unique<btBvhTriangleMeshShape>(x->triangles.get(), true);
    } else {
        auto shape = std::make_unique<btConvexHullShape>();
        for (const auto &v : vertices)
            shape->addPoint(b(v), false);
        shape->recalcLocalAabb();
        shape->optimizeConvexHull();
        x->shape = std::move(shape);
    }
    insert(std::move(x));
}
void TaskContacts::setJaws(double left, double right) {
    for (auto &x : entries)
        if (x->name == "left" || x->name == "right") {
            double q = x->name == "left" ? left : right;
            if (!std::isfinite(q))
                continue;
            x->local = mount;
            x->local.translate(V(0, (x->name == "left" ? 1 : -1) * std::clamp(q, 0., travel), 0));
        }
}
void TaskContacts::setProp(const std::string &name, const Pose &pose, bool attached) {
    for (auto &x : entries)
        if (x->kind == 4 && x->name == name) {
            x->local = pose;
            x->attached = attached;
            break;
        }
}
std::vector<TaskContacts::Contact> TaskContacts::contacts(const Eigen::VectorXd &state) {
    Pose body = Pose::Identity();
    body.translate(state.head<3>());
    body.rotate(Q(state[3], state[4], state[5], state[6]).normalized());
    for (auto &x : entries) {
        x->supported = false;
        if (x->kind != 2) {
            x->object.setWorldTransform(transform(x->kind == 1 || x->attached ? body * x->local : x->local));
            world.updateSingleAabb(&x->object);
        }
    }
    world.performDiscreteCollisionDetection();
    // A free prop transmits downward pressure to the vehicle only when supported
    // by scenery. Side pushes remain in the dynamic prop world.
    for (int i = 0; i < dispatcher.getNumManifolds(); ++i) {
        auto *m = dispatcher.getManifoldByIndexInternal(i);
        auto *a = static_cast<Entry *>(m->getBody0()->getUserPointer());
        auto *c = static_cast<Entry *>(m->getBody1()->getUserPointer());
        for (int j = 0; j < m->getNumContacts(); ++j) {
            const auto &p = m->getContactPoint(j);
            if (p.getDistance() > .003)
                continue;
            if (a->kind == 4 && c->kind == 2 && c->name != "table" && p.m_normalWorldOnB.z() > .5)
                a->supported = true;
            if (c->kind == 4 && a->kind == 2 && a->name != "table" && p.m_normalWorldOnB.z() < -.5)
                c->supported = true;
        }
    }
    std::vector<Contact> out;
    for (int i = 0; i < dispatcher.getNumManifolds(); ++i) {
        auto *m = dispatcher.getManifoldByIndexInternal(i);
        auto *a = static_cast<Entry *>(m->getBody0()->getUserPointer());
        auto *c = static_cast<Entry *>(m->getBody1()->getUserPointer());
        bool av = a->kind == 1 || a->attached, cv = c->kind == 1 || c->attached;
        if (av == cv)
            continue;
        auto *obstacle = av ? c : a;
        for (int j = 0; j < m->getNumContacts(); ++j) {
            const auto &p = m->getContactPoint(j);
            if (p.getDistance() > .0005)
                continue;
            V normal = e(p.m_normalWorldOnB) * (av ? 1 : -1);
            if (obstacle->kind == 4 && (!obstacle->supported || normal.z() < .7))
                continue;
            out.push_back({e(av ? p.getPositionWorldOnA() : p.getPositionWorldOnB()), normal,
                           std::max(0., -double(p.getDistance()))});
        }
    }
    return out;
}
Eigen::VectorXd TaskContacts::resolve(Eigen::VectorXd state, const c_simulator::Matrix6d &inverseMass,
                                      double friction) {
    Q q(state[3], state[4], state[5], state[6]);
    q.normalize();
    for (int pass = 0; pass < 8; ++pass) {
        auto cs = contacts(state);
        if (cs.empty())
            break;
        const auto deepest = std::max_element(cs.begin(), cs.end(),
                                              [](const Contact &a, const Contact &c) { return a.depth < c.depth; });
        bool changed = deepest->depth > 1e-5;
        if (changed)
            state.head<3>() += deepest->normal * (deepest->depth + .00005);
        for (const auto &c : cs) {
            V n = q.conjugate() * c.normal, r = q.conjugate() * (c.point - state.head<3>());
            c_simulator::Vector6d j;
            j << n, r.cross(n);
            double speed = j.dot(state.segment<6>(7)), den = j.dot(inverseMass * j);
            if (speed >= -1e-5 || den < 1e-12)
                continue;
            changed = true;
            double impulse = -speed / den;
            state.segment<6>(7) += inverseMass * j * impulse;
            V v = state.segment<3>(7) + state.segment<3>(10).cross(r);
            V tangent = v - n * v.dot(n);
            if (tangent.norm() > 1e-9) {
                tangent.normalize();
                c_simulator::Vector6d jt;
                jt << tangent, r.cross(tangent);
                double d = jt.dot(inverseMass * jt);
                if (d > 1e-12)
                    state.segment<6>(7) -=
                        inverseMass * jt * std::min(friction * impulse, jt.dot(state.segment<6>(7)) / d);
            }
        }
        if (!changed)
            break;
    }
    return state;
}
