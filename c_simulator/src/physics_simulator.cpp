// ROS adapter for the COM-centered Fossen plant. See docs/PHYSICS.md for
// equations, frame conventions, parameter provenance and model limits.
#include "c_simulator/RobotClass.h"
#include "c_simulator/TaskContacts.h"
#include "c_simulator/collisionBox.h"
#include "c_simulator/settings.h"
#include <algorithm>
#include <chrono>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <filesystem>
#include <functional>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <memory>
#include <nav_msgs/msg/odometry.hpp>
#include <random>
#include <rclcpp/rclcpp.hpp>
#include <riptide_msgs2/msg/dshot_partial_telemetry.hpp>
#include <riptide_msgs2/msg/kill_switch_report.hpp>
#include <robot_localization/srv/set_pose.hpp>
#include <rosgraph_msgs/msg/clock.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <set>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <string>
#include <tf2_ros/transform_broadcaster.h>
#include <thread>
#include <urdf_model/model.h>
#include <urdf_parser/urdf_parser.h>
#include <visualization_msgs/msg/marker_array.hpp>

using std::placeholders::_1;
using std::placeholders::_2;
typedef Eigen::Vector3d v3d;
typedef Eigen::Vector4d v4d;
typedef Eigen::VectorXd vXd;
typedef Eigen::Matrix3d m3d;
typedef Eigen::MatrixXd mXd;
namespace fs = std::filesystem;
typedef Eigen::Quaterniond quat;
using namespace std::chrono_literals;
using std::string, std::cout, std::endl;

class PhysicsSimNode : public rclcpp::Node {
  public:
    //================================//
    //         SIM START UP           //
    //================================//
    PhysicsSimNode() : Node("physics_simulator") {
        enabled = false;
        physicsStep = declare_parameter<double>("physics_step", .002);
        contactFriction = declare_parameter<double>("contact_friction", .4);
        restitution = declare_parameter<double>("restitution", .1);
        collisionsEnabled = declare_parameter<bool>("collisions", true);
        noiseEnabled = declare_parameter<bool>("sensor_noise", true);
        // Simulated time advances at real_time_factor x wall time. Every stamp
        // this node emits comes from that clock, which it also publishes on
        // /clock, so a stack running with use_sim_time follows the plant exactly
        // even when physics cannot keep up with wall time or is deliberately
        // sped up/slowed down.
        realTimeFactor = declare_parameter<double>("real_time_factor", 1.0);
        clockPublishRate = declare_parameter<double>("clock_publish_rate", 500.0);
        if (!std::isfinite(realTimeFactor) || realTimeFactor < 0 || !std::isfinite(clockPublishRate) ||
            clockPublishRate <= 0)
            throw std::invalid_argument("Invalid real_time_factor/clock_publish_rate");
        gyroRate = declare_parameter<double>("gyro_rate", 500.0);
        // Sensor-noise prior, in rad/s; independently configurable from the IMU.
        gyroSigma = declare_parameter<double>("gyro_noise_stddev", 0.01 * M_PI / 180.0);
        gyroVariance = declare_parameter<double>("gyro_variance", std::max(1e-9, gyroSigma * gyroSigma));
        if (!std::isfinite(gyroRate) || gyroRate <= 0 || !std::isfinite(gyroSigma) || gyroSigma < 0 ||
            !std::isfinite(gyroVariance) || gyroVariance <= 0)
            throw std::invalid_argument("Invalid gyro sensor parameters");
        randomGenerator.seed(declare_parameter<int>("random_seed", 7));
        if (!std::isfinite(physicsStep) || physicsStep <= 0 || physicsStep > .02 || !std::isfinite(contactFriction) ||
            contactFriction < 0 || !std::isfinite(restitution) || restitution < 0 || restitution > 1)
            throw std::invalid_argument("Invalid physics step/contact parameters");
        stepNs = std::llround(physicsStep * 1e9);
        // Start simulated time at the wall epoch so stamps, logs and bags stay
        // human-readable and a wall-clock stack sees no jump at rtf 1.
        simNs = rclcpp::Clock(RCL_SYSTEM_TIME).now().nanoseconds();
        transformBuffer = std::make_unique<tf2_ros::Buffer>(get_clock());
        transformListener = std::make_shared<tf2_ros::TransformListener>(*transformBuffer);
        string name = this->get_namespace();
        robot = Robot(name.substr(1)); // Removing the starting '/' with substring
        // Pose the vehicle launched at. An explicit set_sim_pose replaces it; the
        // viewer's "Reset to start" button returns here via reset_sim_to_start.
        startState = robot.getState();

        // Create publishers and subscribers
        imuPub = this->create_publisher<sensor_msgs::msg::Imu>("vectornav/imu", 10);
        // Absolute topic: /clock is global, not per vehicle namespace. Reliable so
        // both ClockQoS (rclcpp TimeSource) and default subscribers can match.
        clockPub = create_publisher<rosgraph_msgs::msg::Clock>("/clock", rclcpp::QoS(rclcpp::KeepLast(10)));
        gyroPub = create_publisher<geometry_msgs::msg::TwistWithCovarianceStamped>("gyro/twist", 10);
        firmwareKillPub = this->create_publisher<std_msgs::msg::Bool>("state/kill", 10);
        truthPub = create_publisher<nav_msgs::msg::Odometry>("simulator/ground_truth", 10);
        actualThrusterPub = create_publisher<std_msgs::msg::Float32MultiArray>("simulator/actual_thruster_forces", 10);
        timePub = create_publisher<std_msgs::msg::Float64>("simulator/time", 10);
        statePub = this->create_publisher<geometry_msgs::msg::Pose>("simulator/state", 10);
        dvlPub = this->create_publisher<geometry_msgs::msg::TwistWithCovarianceStamped>("dvl_twist", 10);
        depthPub = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("depth/pose", 10);
        acousticsPub = this->create_publisher<geometry_msgs::msg::Vector3Stamped>("acoustics/delta_t", 10);
        collisionBoxPub =
            this->create_publisher<visualization_msgs::msg::MarkerArray>("simulator/collisionMarkers", 10);
        thrusterTelemetryPub =
            this->create_publisher<riptide_msgs2::msg::DshotPartialTelemetry>("state/thrusters/telemetry", 10);
        thrusterSub = this->create_subscription<std_msgs::msg::Float32MultiArray>(
            "thruster_forces", 10, std::bind(&PhysicsSimNode::forceCallback, this, _1));
        softwareKillSub = this->create_subscription<riptide_msgs2::msg::KillSwitchReport>(
            "command/software_kill", 10, std::bind(&PhysicsSimNode::killSwitchCallback, this, _1));

        genericEnableSub =
            create_subscription<std_msgs::msg::Bool>("simulator/enable", 10, [this](const std_msgs::msg::Bool &msg) {
                if (vehicleProfile["sim_adapter"].as<std::string>("uwrt") == "generic") {
                    enabled = msg.data;
                    if (!enabled)
                        robot.stopThrusters();
                }
            });
        clawJointsSub = create_subscription<std_msgs::msg::Float64MultiArray>(
            "simulator/claw_joints", 10, [this](const std_msgs::msg::Float64MultiArray &m) {
                if (taskContacts && m.data.size() == 2)
                    taskContacts->setJaws(m.data[0], m.data[1]);
            });
        taskObjectsSub = create_subscription<visualization_msgs::msg::MarkerArray>(
            "simulator/task_objects", 10, [this](const visualization_msgs::msg::MarkerArray &m) {
                if (!taskContacts)
                    return;
                for (const auto &marker : m.markers) {
                    if (marker.action != visualization_msgs::msg::Marker::ADD)
                        continue;
                    const auto &p = marker.pose.position;
                    const auto &r = marker.pose.orientation;
                    Eigen::Isometry3d t = Eigen::Isometry3d::Identity();
                    t.translate(v3d(p.x, p.y, p.z));
                    t.rotate(quat(r.w, r.x, r.y, r.z).normalized());
                    const bool attached = marker.header.frame_id == robot.getName() + "/base_link";
                    if (attached)
                        t.pretranslate(robot.getBaseLinkOffset());
                    taskContacts->setProp(marker.ns, t, attached);
                }
            });

        // Periodic outputs run from the physics loop on simulated time (see
        // schedule()); only housekeeping stays on wall timers. /clock goes first so
        // subscribers' clocks reach a stamp before data carrying it.
        schedule(1.0 / clockPublishRate, [this] { publishClock(); });
        schedule(STATE_PUB_TIME, [this] { publishState(); });
        schedule(0.5, [this] { pubThrusterTelemetry(); });
        paramRefreshTimer =
            this->create_wall_timer(5.0s, std::bind(&PhysicsSimNode::refreshSimulationParameters, this));

        // Services for setting odom and simulator
        poseClient = this->create_client<robot_localization::srv::SetPose>("/" + robot.getName() + "/set_pose");
        poseService = this->create_service<robot_localization::srv::SetPose>(
            "set_sim_pose", std::bind(&PhysicsSimNode::setSim, this, _1));
        ekfOdomSub = create_subscription<nav_msgs::msg::Odometry>("odometry/filtered", 10,
                                                                  [this](const nav_msgs::msg::Odometry &odom) {
                                                                      ekfFrame = odom.header.frame_id;
                                                                      latestEkfOdom = odom;
                                                                      haveEkfOdom = true;
                                                                  });
        // Opposite of alignEkf: teleport the plant to where navigation thinks the
        // vehicle is, leaving the EKF untouched. Used by the viewer's sync button.
        syncService = create_service<std_srvs::srv::Trigger>(
            "sync_sim_to_estimate", std::bind(&PhysicsSimNode::syncSimToEstimate, this, _1, _2));
        resetStartService = create_service<std_srvs::srv::Trigger>(
            "reset_sim_to_start", std::bind(&PhysicsSimNode::resetSimToStart, this, _1, _2));
        ekfAlignmentTimer = create_wall_timer(100ms, std::bind(&PhysicsSimNode::alignEkf, this));

        // TF broadcaster
        tf_broadcaster = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    }

    /**
   * @brief Loads parameters from the robot's yaml file into the Robot object
   * @returns wether the parameters were loaded successfully
   */
    bool init() {
        // Tries to load robot parameter data
        RCLCPP_INFO(this->get_logger(), "Loading robot's parameter data from YAML..");
        bool paramsLoaded = robot.loadParams(shared_from_this());
        if (!paramsLoaded)
            return false;
        vehicleProfile = YAML::LoadFile(get_parameter("vehicle_config").as_string());
        if (vehicleProfile["sim_start_pose"]) {
            const auto p = vehicleProfile["sim_start_pose"].as<std::vector<double>>();
            vXd x = robot.getState();
            tf2::Quaternion q;
            q.setRPY(p.at(3), p.at(4), p.at(5));
            x.head<3>() = v3d(p.at(0), p.at(1), p.at(2));
            x.segment<4>(3) = v4d(q.w(), q.x(), q.y(), q.z());
            robot.setState(x);
            startState = x;
        }
        dvlSigma = declare_parameter<double>("dvl_noise_stddev", robot.getDVLSigma());
        dvlVariance = declare_parameter<double>("dvl_variance", std::max(1e-9, dvlSigma * dvlSigma));
        imuGravity = declare_parameter<double>("imu_gravity", GRAVITY);
        imuYawDrift = declare_parameter<double>("imu_yaw_drift", robot.getIMUDrift());
        if (!std::isfinite(dvlSigma) || dvlSigma < 0 || !std::isfinite(dvlVariance) || dvlVariance <= 0 ||
            !std::isfinite(imuGravity) || imuGravity <= 0 || !std::isfinite(imuYawDrift))
            throw std::invalid_argument("Invalid DVL noise/covariance or IMU gravity/drift");
        RCLCPP_INFO(this->get_logger(), "Loading collision files...");
        bool collisionBoxesLoaded = (COLLISION_TOGGLE ? loadCollisionFiles() : true);

        // wether or not to synchronize odometry
        this->declare_parameter("sync_odom", false);
        this->get_parameter("sync_odom", this->sync_odom);

        // Loaded successfully if true
        if (paramsLoaded && collisionBoxesLoaded) {
            //  Once YAML file params have been loaded, schedule the fake sensors.
            //  They fire from the physics loop at simulated-time intervals, so their
            //  rates and stamps are locked to the plant rather than to wall timers.
            RCLCPP_INFO(this->get_logger(), "Initization successful. Sensors scheduled on simulated time");
            if (sensorEnabled("imu"))
                schedule(robot.getIMURate(), [this] { publishFakeIMUData(); });
            if (sensorEnabled("fog"))
                schedule(1.0 / gyroRate, [this] { publishFakeGyroData(); });
            if (sensorEnabled("dvl"))
                schedule(robot.getDVLRate(), [this] { publishFakeDVLData(); });
            if (sensorEnabled("depth"))
                schedule(robot.getDepthRate(), [this] { publishFakeDepthData(); });
            if (sensorEnabled("acoustics"))
                schedule(2.0, [this] { publishFakeAcousticsData(); });
        } else {
            // Report error
            if (!paramsLoaded)
                RCLCPP_INFO(this->get_logger(), "Failed to read YAML parameter data");
            else
                RCLCPP_INFO(this->get_logger(), "Failed to read collision URDF data");
        }
        return paramsLoaded && collisionBoxesLoaded;
    }

    //================================//
    //      PHYSICS FUNCTIONS         //
    //================================//

    /**
   * @brief Solves system of ODEs representing robot physics using fourth order
   * Runge-Kutta numerical method. This function is runs in an infinite loop to
   * run the simulator, which also spins the ROS node
   *
   * Physics equations have been set up as a system of first order equations so
   * that: dx/dt = f(x)
   *
   * Where x is the robot's state as a column vector:
   * x = [x y z q_w q_x q_y q_z u v w p q r]^T                   <- parameter
   *      0 1 2 3   4   5   6   7   8   9   10  11  12         <- index
   * Position and quaternion are world-referenced; linear and angular velocities
   * are body-frame quantities following Fossen's marine-craft convention.
   */
    void rungeKutta4() {
        using steady_clock = std::chrono::steady_clock;
        auto previousTime = steady_clock::now();
        // Simulated seconds owed to the plant. Wall time is the pacing source;
        // simulated time (simNs) only ever advances by whole physics steps.
        double accumulator = 0.0;
        publishClock();

        while (rclcpp::ok()) {
            rclcpp::spin_some(shared_from_this());
            refreshRealTimeFactor();

            const auto currentTime = steady_clock::now();
            accumulator += realTimeFactor * std::chrono::duration<double>(currentTime - previousTime).count();
            previousTime = currentTime;

            int steps = 0;
            while (accumulator >= physicsStep && steps < MAX_CATCHUP_STEPS) {
                robot.updateThrusters(physicsStep / 2);

                vXd state = robot.getState();
                if (collisionsEnabled)
                    state = handleCollisions(state);

                const vXd K1 = calcStateDot(state, -physicsStep / 2);
                const vXd K2 = calcStateDot(state + physicsStep / 2.0 * K1);
                const vXd K3 = calcStateDot(state + physicsStep / 2.0 * K2);
                const vXd K4 = calcStateDot(state + physicsStep * K3, physicsStep / 2);
                const vXd stateDelta = physicsStep / 6.0 * (K1 + 2 * K2 + 2 * K3 + K4);
                vXd advanced = state + stateDelta;
                if (collisionsEnabled)
                    advanced = handleCollisions(advanced);
                robot.setState(advanced);
                robot.updateThrusters(physicsStep / 2);
                robot.setAccel(robot.stateDerivative(robot.getState()));
                simNs += stepNs;
                // Sensors sample the state just integrated, stamped with its time.
                runScheduled();

                accumulator -= physicsStep;
                ++steps;
            }

            if (steps == MAX_CATCHUP_STEPS && accumulator >= physicsStep) {
                // Simulated time simply falls behind wall time here. Nodes on
                // use_sim_time follow /clock, so this costs speed, not accuracy.
                RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                                     "Physics cannot keep up at real_time_factor %.2f; simulated time "
                                     "falls behind wall time (dropping %.1f ms of backlog)",
                                     realTimeFactor, accumulator * 1000.0);
                accumulator = 0.0;
            }

            // Leave CPU time for rendering and sensor nodes between fixed physics
            // steps.
            const double remaining = realTimeFactor > 0 ? (physicsStep - accumulator) / realTimeFactor : .002;
            if (remaining > 0.0002)
                std::this_thread::sleep_for(std::chrono::duration<double>(remaining - 0.0001));
        }
        RCLCPP_INFO(this->get_logger(), "Shutting down simulator");
    }

  private:
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr genericEnableSub;
    YAML::Node vehicleProfile;
    bool sensorEnabled(const std::string &name) const {
        if (!vehicleProfile["sim_enabled_sensors"])
            return true;
        for (const auto &s : vehicleProfile["sim_enabled_sensors"])
            if (s.as<std::string>() == name)
                return true;
        return false;
    }
    /**
   * @param state Robot's state vector where derivative is to be evaluated at
   * @return The rate of change of the robot's state vector
   */
    vXd calcStateDot(const vXd &state, double stageOffset = 0) {
        return robot.stateDerivative(state, stageOffset);
    }

    //================================//
    //      COLLISION FUNCTIONS       //
    //================================//

    /**
   * @brief Loops through all collision elements and adds impulses based on
   * collision information
   * @param state Robot's state vector where collisions are evaluated at
   * @returns Modified state vector with added collision impulses
   */
    vXd handleCollisions(vXd state) {
        // Get state information
        quat q = state2quat(state);

        // Loop through all combination of boxes
        for (collisionBox &robotBox : robotBoxes) {
            // Need to update robot collision boxes to robot's current position
            robotBox.updateLocation(state);
            for (collisionBox obstacleBox : obstacleBoxes) {
                // Compute the collision on selected boxes
                robotBox.updateLocation(state);
                collisionResult collision = computeCollision(robotBox, obstacleBox);

                // A collision occoured, calculate how to change state in response to
                // collision
                if (collision.collided) {
                    RCLCPP_DEBUG(this->get_logger(), "%s has collided with %s", robot.getName().c_str(),
                                 obstacleBox.getName().c_str());
                    // Compute impulse
                    // Using impulse method described here:
                    // https://research.ncl.ac.uk/game/mastersdegree/gametechnologies/physicstutorials/5collisionresponse/Physics%20-%20Collision%20Response.pdf
                    // And also here: https://hitokageproduction.com/article/11
                    v3d r_a = collision.collisionPoint - state.segment(0, 3);
                    double v_rel =
                        (q * state.segment<3>(7) + (q * state.segment<3>(10)).cross(r_a)).dot(collision.unitDirection);
                    // Seperate the objects
                    state.segment(0, 3) = collision.unitDirection * collision.depth + state.segment(0, 3);
                    // Apply restitution only while approaching. Applying an impulse while
                    // already separating incorrectly pulls the vehicle back into the
                    // obstacle.
                    if (v_rel < 0.0) {
                        const v3d normalBody = q.conjugate() * collision.unitDirection;
                        const v3d contactOffsetBody = q.conjugate() * r_a;
                        c_simulator::Vector6d contactJacobian;
                        contactJacobian << normalBody, contactOffsetBody.cross(normalBody);
                        const c_simulator::Matrix6d inverseMass = robot.getInverseMass();
                        const double effectiveInverseMass = contactJacobian.dot(inverseMass * contactJacobian);
                        if (effectiveInverseMass > 1e-12) {
                            const double impulse = -(1.0 + restitution) * v_rel / effectiveInverseMass;
                            state.segment(7, 6) += inverseMass * contactJacobian * impulse;
                            const v3d contactVelocity =
                                state.segment<3>(7) + state.segment<3>(10).cross(contactOffsetBody);
                            v3d tangent = contactVelocity - normalBody * contactVelocity.dot(normalBody);
                            if (tangent.norm() > 1e-9) {
                                tangent.normalize();
                                c_simulator::Vector6d jt;
                                jt << tangent, contactOffsetBody.cross(tangent);
                                const double jtInv = jt.dot(inverseMass * jt);
                                const double friction =
                                    std::min(contactFriction * impulse, jt.dot(state.segment<6>(7)) / jtInv);
                                state.segment<6>(7) -= inverseMass * jt * friction;
                            }
                        }
                    }
                }
            }
        }
        if (taskContacts)
            state = taskContacts->resolve(state, robot.getInverseMass(), contactFriction);
        return state;
    }

    /**
   * @brief Determines collision information for two collision boxes using the
   * Seperating Axis Theorem (SAT) in 3D
   * @param box1 Collision box of the robot (must make sure to update position
   * beforehand)
   * @param box2 Collision box of the obstacle
   * @returns Struct containing wether there was a collision, collision point,
   * depth, and collision normal
   */
    collisionResult computeCollision(collisionBox &box1, collisionBox &box2) {
        // For more on Seperating Axis Theorem:
        // https://research.ncl.ac.uk/game/mastersdegree/gametechnologies/previousinformation/physics4collisiondetection/2017%20Tutorial%204%20-%20Collision%20Detection.pdf
        // Create axes to check projections on
        v3d axes[] = {box1.getAxis(0),
                      box1.getAxis(1),
                      box1.getAxis(2),
                      box2.getAxis(0),
                      box2.getAxis(1),
                      box2.getAxis(2),
                      box1.getAxis(0).cross(box2.getAxis(0)),
                      box1.getAxis(0).cross(box2.getAxis(1)),
                      box1.getAxis(0).cross(box2.getAxis(2)),
                      box1.getAxis(1).cross(box2.getAxis(0)),
                      box1.getAxis(1).cross(box2.getAxis(1)),
                      box1.getAxis(1).cross(box2.getAxis(2)),
                      box1.getAxis(2).cross(box2.getAxis(0)),
                      box1.getAxis(2).cross(box2.getAxis(1)),
                      box1.getAxis(2).cross(box2.getAxis(2))};

        // Variable set up
        v3d minAxis;
        double overlap;
        double minOverlap = std::numeric_limits<double>::infinity();

        // Loop through all axes
        for (v3d axis : axes) {
            // If axis is zero vector, don't compute it. (Can occour because vectors
            // from cross products could be alligned)
            const double axisNorm = axis.norm();
            if (axisNorm < 1e-9)
                continue;
            axis /= axisNorm;
            // If the center of box1 is to the right of box2, flip axis so it becomes
            // to the left
            if (box1.getCenter().dot(axis) > box2.getCenter().dot(axis))
                axis = -axis;

            // Calculate the overlap between the projections of the two boxes onto the
            // axis Think about this like shining a flashlight onto the shape and
            // looking at it's shadow This finds the overlap between the shape's
            // "shadows" along the axis to be tested
            overlap = box1.maxProjection(axis) - box2.minProjection(axis);

            // New smallest overlap found
            if (overlap < minOverlap) {
                minOverlap = overlap;
                minAxis = axis;
                if (minOverlap <= 0)
                    //  *gasp!* there is a way to look at the boxes without any overlap
                    //  The shapes can't be colliding then, return standard no-collision
                    //  result
                    return collisionResult();
            }
        }

        // Since the function hasn't returned yet, there is no axis without overlap
        // COLLISION HAS OCCOURED!
        // Now finding collision point
        v3d point;
        // Create a 3x6 matrix containing all the box axis
        mXd boxAxes(3, 6);
        boxAxes.block(0, 0, 3, 3) = box1.rotationMatrix();
        boxAxes.block(0, 3, 3, 3) = box2.rotationMatrix();
        int index;
        // Ok, we have the axis that has the smallest overlap, but need to find if
        // it belongs to first or second box Need to know which box it belongs to
        // because it determines which box's vertex to use as the collision point
        // Note that the minAxis might not be one of the three axes alligned with a
        // box edge since the cross product of the axes could be the minAxes The dot
        // product is taken with each of the box axes, the one with the largest
        // matches the box the most. This informs us on what box's vertex to use for
        // the collision point
        (boxAxes.transpose() * minAxis).cwiseAbs().maxCoeff(&index);
        if (index < 3)
            // A face of the first box is getting intersected, thus a vertex from the
            // second box must be intersecting it
            point = box2.minVertex(minAxis);
        else
            // A face of the second box is getting intersected, thus a vertex from the
            // first box must be intersecting it
            point = box1.maxVertex(minAxis);

        // It's an edge on edge collision if true
        if (!box1.isInBox(point) || !box2.isInBox(point)) {
            // The collision point needs a bit of persuasion to get to a reasonable
            // location :P Shuffle the point back and forth between the two boxes to
            // iteratively move the vertex closer to the edge collision point
            point = box1.moveInBox(point);
            point = box2.moveInBox(point);
            point = box1.moveInBox(point);
            point = box2.moveInBox(point);
            point = box1.moveInBox(point);
        }
        // Store results and return
        collisionResult result;
        result.collided = true;
        result.depth = minOverlap;
        result.unitDirection = -minAxis;
        result.collisionPoint = point;
        return result;
    }

    /**
   * @brief Goes through collsion .urdf files in "scene_info.yaml" and adds
   * their collisionBoxes
   * @returns Whether the files could be read succesfully
   */
    bool loadCollisionFiles() {
        // Get folder path
        string _collisionFolder, _sceneFile;
        this->declare_parameter("collision_folder", "");
        this->declare_parameter("scene_config", "");
        this->get_parameter("scene_config", _sceneFile);
        this->get_parameter("collision_folder", _collisionFolder);
        std::filesystem::path collisionFolder(_collisionFolder);

        RCLCPP_INFO(get_logger(), "Loading scene from file: %s", _sceneFile.c_str());
        YAML::Node sceneFile = YAML::LoadFile(_sceneFile);
        if (!sceneFile) {
            RCLCPP_ERROR(this->get_logger(), "Failed to load scene file: %s", _sceneFile.c_str());
            return false;
        }

        const auto mapping = YAML::LoadFile(declare_parameter<std::string>("mapping_config", ""));
        const auto data = mapping["/" + robot.getName() + "/riptide_mapping2"]["ros__parameters"]["init_data"];
        if (!data)
            throw std::runtime_error("Mapping config has no robot init_data");
        std::map<std::string, Eigen::Isometry3d> frames;
        std::set<std::string> visiting;
        std::function<Eigen::Isometry3d(std::string)> resolve = [&](std::string key) -> Eigen::Isometry3d {
            if (key == "map" || key == "world")
                return Eigen::Isometry3d::Identity();
            if (key.size() > 6 && key.substr(key.size() - 6) == "_frame")
                key.resize(key.size() - 6);
            if (frames.count(key))
                return frames.at(key);
            if (!visiting.insert(key).second)
                throw std::runtime_error("Cycle in mapping frames");
            auto entry = data[key];
            if (!entry)
                throw std::runtime_error("Unknown mapping frame " + key);
            auto pose = entry["pose"];
            Eigen::Isometry3d local = Eigen::Isometry3d::Identity();
            local.translate(v3d(pose["x"].as<double>(0), pose["y"].as<double>(0), pose["z"].as<double>(0)));
            local.rotate(Eigen::AngleAxisd(deg2rad(pose["yaw"].as<double>(0)), v3d::UnitZ()));
            Eigen::Isometry3d result = resolve(entry["parent"].as<std::string>()) * local;
            visiting.erase(key);
            frames[key] = result;
            return result;
        };
        const auto origin = mapping["/**/zed_faker"]["ros__parameters"]["map_origin_pool"].as<std::vector<double>>();
        if (origin.size() != 3)
            throw std::runtime_error("Expected map_origin_pool [x,y,yaw]");
        Eigen::Isometry3d mapToPool = Eigen::Isometry3d::Identity();
        mapToPool.translate(v3d(origin[0], origin[1], 0));
        mapToPool.rotate(Eigen::AngleAxisd(deg2rad(origin[2]), v3d::UnitZ()));
        const Eigen::Isometry3d poolToMap = mapToPool.inverse();
        // Add the robot collision boxes
        if (sceneFile["robot"]["collision"].IsDefined()) {
            std::string robotCollisionPath = collisionFolder / "robots" / sceneFile["robot"]["collision"].as<string>();
            urdf::ModelInterfaceSharedPtr robotModel = urdf::parseURDFFile(robotCollisionPath);
            if (!robotModel) {
                RCLCPP_ERROR(this->get_logger(), "Failed to load robot model: %s", robotCollisionPath.c_str());
                return false;
            }
            robotBoxes = unpackURDF(robotModel, v3d(0, 0, 0), quat(1, 0, 0, 0));
        }

        // Configured box proxies share their frame and pose with the viewer.
        for (const auto &entity : sceneFile["entities"]) {
            if (!entity["collision"].as<bool>(false))
                continue;
            auto transform = resolve(entity["frame"].as<std::string>("map"));
            if (entity["pose"]) {
                const auto pose = entity["pose"].as<std::vector<double>>();
                transform.translate(v3d(pose[0], pose[1], pose[2]));
                transform.rotate(Eigen::AngleAxisd(pose[5], v3d::UnitZ()) * Eigen::AngleAxisd(pose[4], v3d::UnitY()) *
                                 Eigen::AngleAxisd(pose[3], v3d::UnitX()));
            }
            const auto size = entity["size"].as<std::vector<double>>();
            obstacleBoxes.emplace_back(entity["id"].as<std::string>(), size[0], size[1], size[2],
                                       transform.translation(), v3d::Zero(), quat(transform.rotation()));
        }
        // Add the objects collision boxes
        for (auto const &entryPair : sceneFile["objects"]) {
            // Get the .urdf file with some error checking
            YAML::Node entry = entryPair.second;
            if (!entry["collision"].IsDefined()) {
                RCLCPP_WARN(this->get_logger(), "Skipping entry  %s without collision info.",
                            entryPair.first.as<string>().c_str());
                continue;
            }
            std::string objectCollisionPath = collisionFolder / "objects" / entry["collision"].as<string>();
            urdf::ModelInterfaceSharedPtr objectModel = urdf::parseURDFFile(objectCollisionPath);
            if (!objectModel) {
                RCLCPP_ERROR(this->get_logger(), "Failed to load object model: %s, continuing...",
                             objectCollisionPath.c_str());
                continue;
            }
            const std::string key = entryPair.first.as<std::string>();
            if (!data[key]) {
                RCLCPP_WARN(get_logger(), "No mapping pose for %s; skipping legacy collision proxy", key.c_str());
                continue;
            }
            const auto pose = resolve(key);
            const v3d position = pose.translation();
            const quat objQuat(pose.rotation());
            // Add all the new boxes to the vector
            std::vector<collisionBox> newBoxes = unpackURDF(objectModel, position, objQuat);
            for (collisionBox newBox : newBoxes)
                obstacleBoxes.push_back(newBox);
        }
        // Crates share their dimensions and individual vinyl frames with the
        // viewer.
        const auto taskFile = declare_parameter<std::string>("task_config", "");
        if (!taskFile.empty() && YAML::LoadFile(taskFile)["crate"]) {
            const auto crate = YAML::LoadFile(taskFile)["crate"];
            const double outer = crate["outer_width"].as<double>();
            const double inner = crate["inner_width"].as<double>() - 2 * crate["liner_thickness"].as<double>();
            const double base = crate["base_thickness"].as<double>();
            const double height = crate["outer_height"].as<double>() - base;
            const double wall = (outer - inner) / 2;
            for (int i = 1; i <= 4; ++i) {
                const auto key = "bin_vinyl" + std::to_string(i);
                if (!data[key])
                    continue;
                const auto t = resolve(key);
                auto add = [&](const v3d &p, const v3d &size) {
                    obstacleBoxes.emplace_back(key + "_crate", size.x(), size.y(), size.z(), t * p, v3d::Zero(),
                                               quat(t.rotation()));
                };
                add({0, 0, -base / 2}, {outer, outer, base});
                for (int sign : {-1, 1}) {
                    add({sign * (outer - wall) / 2, 0, height / 2}, {wall, outer, height});
                    add({0, sign * (outer - wall) / 2, height / 2}, {outer, wall, height});
                }
            }
        }
        // Pool planes share the exact corner-origin transform used by the renderer.
        auto poolBox = [&](const std::string &name, const v3d &size, const v3d &center) {
            obstacleBoxes.emplace_back(name, size.x(), size.y(), size.z(), poolToMap * center, v3d::Zero(),
                                       quat(poolToMap.rotation()));
        };
        const auto world = sceneFile["world"];
        const double length = world["length"].as<double>(50), width = world["width"].as<double>(22.86),
                     depth = world["depth"].as<double>(2.1336), surface = world["water_level"].as<double>(0);
        const double height = depth + world["deck_height"].as<double>(.305288888) + 1.,
                     middle = surface - depth - .5 + height / 2;
        poolBox("floor", v3d(length, width, 1), v3d(length / 2, width / 2, surface - depth - .5));
        poolBox("pool_west_wall", v3d(1, width + 2, height), v3d(-.5, width / 2, middle));
        poolBox("pool_east_wall", v3d(1, width + 2, height), v3d(length + .5, width / 2, middle));
        poolBox("pool_south_wall", v3d(length, 1, height), v3d(length / 2, -.5, middle));
        poolBox("pool_north_wall", v3d(length, 1, height), v3d(length / 2, width + .5, middle));
        if (!taskFile.empty()) {
            const auto task = YAML::LoadFile(taskFile);
            if (task["octagon"] && data["octagon"]) {
                const auto cfg = task["octagon"];
                auto mount = resolve("octagon");
                mount.translation().z() = cfg["surface_z"].as<double>();
                const double a = cfg["apothem"].as<double>(), r = cfg["pipe_radius"].as<double>();
                const double length = 2 * a * std::tan(M_PI / 8);
                for (int i = 0; i < 8; ++i) {
                    const double angle = i * M_PI / 4;
                    Eigen::Isometry3d t = mount;
                    t.translate(v3d(a * std::cos(angle), a * std::sin(angle), 0));
                    t.rotate(Eigen::AngleAxisd(angle + M_PI / 2, v3d::UnitZ()));
                    obstacleBoxes.emplace_back("octagon_pvc", length, 2 * r, 2 * r, t.translation(), v3d::Zero(),
                                               quat(t.rotation()));
                }
            }
            if (task["claw"]) {
                const auto vehicle = YAML::LoadFile(get_parameter("vehicle_config").as_string());
                const auto c = vehicle["com"].as<std::vector<double>>();
                taskContacts = std::make_unique<TaskContacts>(task, (collisionFolder / "tasks").string(),
                                                              v3d(c[0], c[1], c[2]), resolve);
                auto addBox = [&](collisionBox &box, bool attached) {
                    Eigen::Isometry3d t = Eigen::Isometry3d::Identity();
                    t.translate(box.getCenter());
                    t.rotate(box.getOrientation());
                    taskContacts->addBox(box.getName(), v3d(box.getLength(), box.getWidth(), box.getHeight()), t,
                                         attached);
                };
                for (auto &box : obstacleBoxes)
                    addBox(box, false);
                for (auto &box : robotBoxes)
                    addBox(box, true);
            }
        }
        return true;
    }

    /**
   * @brief Unpacks collision information from URDF model and stores it's
   * content into a collisionBox object. Note this only works for boxes
   * @param model URDF model
   * @param basePosition Location of orgin of obstacle in world frame
   * @param baseOrientation Orientation of obstacle represented by quaternion.
   * @returns vector containing collision boxes for each object of URDF
   */
    std::vector<collisionBox> unpackURDF(urdf::ModelInterfaceSharedPtr model, v3d basePosition, quat baseOrientation) {
        std::vector<collisionBox> newBoxes;
        // Loop through each link in the URDF
        for (const auto &link_pair : model->links_) {
            const urdf::LinkSharedPtr &link = link_pair.second;
            // Loop through each collision element in each link
            for (const auto &collision : link->collision_array) {
                const urdf::CollisionSharedPtr &collision_ptr = collision;
                // Check if it's a or not box (current collision implementation only
                // works with boxes)
                if (collision_ptr->geometry->type != urdf::Geometry::BOX) {
                    RCLCPP_ERROR(this->get_logger(),
                                 "Link %s uses a non-box geometry, only boxes are "
                                 "supportede. Skipping element",
                                 link->name.c_str());
                    continue;
                }
                const urdf::BoxSharedPtr &box = std::static_pointer_cast<urdf::Box>(collision_ptr->geometry);
                // Get information from box element
                double length = box->dim.x;
                double width = box->dim.y;
                double height = box->dim.z;
                // Get box posititon relative to obstacle's base
                v3d baseOffset(collision_ptr->origin.position.x, collision_ptr->origin.position.y,
                               collision_ptr->origin.position.z);
                // Get box orientation relative to obstacle's base
                quat baseOrientationOffset(collision_ptr->origin.rotation.w, collision_ptr->origin.rotation.x,
                                           collision_ptr->origin.rotation.y, collision_ptr->origin.rotation.z);

                // Add box to list
                collisionBox newBox = collisionBox(link->name, length, width, height, basePosition, baseOffset,
                                                   baseOrientation, baseOrientationOffset);
                newBoxes.push_back(newBox);
            }
        }
        return newBoxes;
    }

    /**
   * @brief Publishes a marker array containing each collision box for robot and
   * obstacle. This allows RViz to visualize the collision elements for
   * information and debugging
   */
    void publishCollisionMarkers() {
        if (collisionBoxPub->get_subscription_count() == 0)
            return;
        // Combine obstacleBoxes and robotBoxes into a single list to loop through
        std::vector<collisionBox> combinedBoxList = obstacleBoxes;
        combinedBoxList.insert(combinedBoxList.begin(), robotBoxes.begin(), robotBoxes.end());
        // Loop through each collision box and add to marker array
        visualization_msgs::msg::MarkerArray markerArray;
        int markerID = 0;
        rclcpp::Time stampTime = simNow();
        for (collisionBox boxInfo : combinedBoxList) {
            // Fill in generic marker info
            visualization_msgs::msg::Marker collisionMarker;
            collisionMarker.ns = "Collision Boxes";
            collisionMarker.id = markerID;
            collisionMarker.type = visualization_msgs::msg::Marker::CUBE;
            collisionMarker.action = visualization_msgs::msg::Marker::ADD;
            collisionMarker.header.stamp = stampTime;
            collisionMarker.header.frame_id = "map";
            // Set box size
            collisionMarker.scale.x = boxInfo.getLength();
            collisionMarker.scale.y = boxInfo.getWidth();
            collisionMarker.scale.z = boxInfo.getHeight();
            // Set box position
            v3d boxPosition = boxInfo.getCenter();
            collisionMarker.pose.position.x = boxPosition.x();
            collisionMarker.pose.position.y = boxPosition.y();
            collisionMarker.pose.position.z = boxPosition.z();
            //  Set box orientation
            quat boxOrientation = boxInfo.getOrientation();
            collisionMarker.pose.orientation.w = boxOrientation.w();
            collisionMarker.pose.orientation.x = boxOrientation.x();
            collisionMarker.pose.orientation.y = boxOrientation.y();
            collisionMarker.pose.orientation.z = boxOrientation.z();
            // Set color to red
            collisionMarker.color.r = 1;
            collisionMarker.color.g = 0;
            collisionMarker.color.b = 0;
            collisionMarker.color.a = 0.5;

            // Add marker to array
            markerArray.markers.push_back(collisionMarker);
            markerID++;
        }
        // Publish marker array to update boxes
        collisionBoxPub->publish(markerArray);
    }

    //================================//
    //       FAKING SENSOR DATA       //
    //================================//

    // Fabricates fake depth sensor data from robot's state and publishes it to
    // topic. Called on a timer
    void publishFakeDepthData() {
        // Get depth and add noise to data
        const auto state = robot.getState();
        // Actual pressure sensor position, including the rotating lever arm.
        double depthData = (state.head<3>() + state2quat(state) * robot.getDepthOffset()).z();
        if (noiseEnabled && !this->sync_odom)
            depthData = randomNorm(depthData, robot.getDepthSigma());

        // Send message with depth sensor info
        geometry_msgs::msg::PoseWithCovarianceStamped depthMsg;
        // depth/pose is the converted base_link altitude, not the raw pressure
        // point.
        depthMsg.pose.pose.position.z =
            depthData + (state2quat(state) * (robot.getBaseLinkOffset() - robot.getDepthOffset())).z();
        depthMsg.pose.pose.orientation.w = 1.;
        depthMsg.pose.covariance[14] = robot.getDepthSigma() * robot.getDepthSigma();
        depthMsg.header.stamp = simNow();
        depthMsg.header.frame_id = "map";
        depthPub->publish(depthMsg);
    }

    // Fabricates fake DVL data from robot's state and publishes it to topic.
    // Called on a timer
    void publishFakeDVLData() {
        if (robot.dvlTransformAvailable()) {
            vXd state = robot.getState();
            const v3d angularVel = state.segment(10, 3);
            const v3d linearVel = state.segment(7, 3);

            // V_dvl = V_robot + w x r
            v3d dvlData = linearVel + angularVel.cross(robot.getDVLOffset());
            // Transform velocity from robot frame to DVL frame.
            dvlData = robot.getDVLQuat().conjugate() * dvlData;

            // Add nonise to sensor data if enabled, otherwise don't
            if (noiseEnabled && !this->sync_odom)
                dvlData = randomNorm(dvlData, dvlSigma);
            // Send message with DVL sensor info
            geometry_msgs::msg::TwistWithCovarianceStamped dvlMsg;
            dvlMsg.twist.twist.linear.x = dvlData.x();
            dvlMsg.twist.twist.linear.y = dvlData.y();
            dvlMsg.twist.twist.linear.z = dvlData.z();
            // Add covariance to message
            dvlMsg.twist.covariance[0] = dvlVariance;
            dvlMsg.twist.covariance[7] = dvlVariance;
            dvlMsg.twist.covariance[14] = dvlVariance;
            // Send message
            dvlMsg.header.stamp = simNow();
            dvlMsg.header.frame_id = robot.getName() + "/dvl_link";
            dvlPub->publish(dvlMsg);
        }
    }

    // Match the separate fiber-optic gyro used for yaw by the vehicle EKF.
    // Without this stream, yaw rate and the DVL lever-arm correction are unobserved.
    void publishFakeGyroData() {
        const v3d omega = robot.getGyroQuat().conjugate() * robot.getState().segment<3>(10);
        geometry_msgs::msg::TwistWithCovarianceStamped msg;
        msg.header.stamp = simNow();
        msg.header.frame_id = robot.getName() + "/fog_link";
        msg.twist.twist.angular.z = noiseEnabled && !sync_odom ? randomNorm(omega.z(), gyroSigma) : omega.z();
        msg.twist.covariance[35] = gyroVariance;
        gyroPub->publish(msg);
    }

    // Fabricates fake IMU data from robot's state and publishes it to topic.
    // Called on a timer
    void publishFakeIMUData() {
        if (robot.imuTransformAvailable()) {
            vXd state = robot.getState();
            // Get quaternion from state
            quat q = state2quat(state);
            v3d angularVel = q * state.segment(10, 3);

            // a_imu = a_body + alpha x r + w x w x r
            // All vectors are in world frame
            // No Coriolis force is needed since the IMU is not moving relative to the
            // robot
            v3d imuAccel =
                // The plant uses physical gravity. The sensor output must match the
                // calibrated gravity magnitude removed by the unchanged vehicle EKF.
                robot.getLatestLinAccel() + v3d(0, 0, imuGravity - GRAVITY) +
                robot.getLatestAngAccel().cross(q * robot.getIMUOffset()) +
                angularVel.cross(angularVel.cross(q * robot.getIMUOffset()));

            // Transform vectors from world -> robot -> IMU
            angularVel = robot.getIMUQuat().conjugate() * (q.conjugate() * angularVel);
            imuAccel = robot.getIMUQuat().conjugate() * (q.conjugate() * imuAccel);
            // Transform orientation from robot -> IMU
            q = q * robot.getIMUQuat();
            // Add noise to sensor data if enabled, otherwise don't
            v3d imu_sigma = robot.getIMUSigma(); // [imu_sigmaAccel, imu_sigmaOmega,
                                                 // imu_sigmaAngle]
            if (noiseEnabled && !this->sync_odom) {
                imuAccel = randomNorm(imuAccel, imu_sigma[0]);
                angularVel = randomNorm(angularVel, imu_sigma[1]);
                // Gaussian small-angle orientation noise about an isotropic random
                // axis.
                std::normal_distribution<double> unitNormal(0.0, 1.0);
                v3d randomAxis(unitNormal(randomGenerator), unitNormal(randomGenerator), unitNormal(randomGenerator));
                randomAxis.normalize();
                std::normal_distribution<double> angleNoise(0.0, imu_sigma[2]);
                Eigen::AngleAxisd randomAngleNoise(angleNoise(randomGenerator), randomAxis);
                // Drift = drift_speed * elapsed_time
                double elapsedTime = robot.simulationTime() / 60.0;
                double driftAngle = elapsedTime * imuYawDrift * M_PI / 180;
                Eigen::AngleAxisd yawDrift(driftAngle, v3d::UnitZ());
                // Add rotation noise to orientation
                q = yawDrift * randomAngleNoise * q;
            }
            // Send message with IMU sensor info
            sensor_msgs::msg::Imu imuMsg;
            // Setting linear acceleration message info
            imuMsg.linear_acceleration.x = imuAccel.x();
            imuMsg.linear_acceleration.y = imuAccel.y();
            imuMsg.linear_acceleration.z = imuAccel.z();
            // Setting angular velocity message info
            imuMsg.angular_velocity.x = angularVel.x();
            imuMsg.angular_velocity.y = angularVel.y();
            imuMsg.angular_velocity.z = angularVel.z();
            // Setting orientation message info
            imuMsg.orientation.w = q.w();
            imuMsg.orientation.x = q.x();
            imuMsg.orientation.y = q.y();
            imuMsg.orientation.z = q.z();
            // Add covariance to message
            const v3d orientationVariance = robot.getIMUOrientationVariance();
            imuMsg.orientation_covariance[0] = orientationVariance[0];
            imuMsg.orientation_covariance[4] = orientationVariance[1];
            imuMsg.orientation_covariance[8] = orientationVariance[2];
            const v3d angVelVariance = robot.getIMUAngularVelocityVariance();
            imuMsg.angular_velocity_covariance[0] = angVelVariance[0];
            imuMsg.angular_velocity_covariance[4] = angVelVariance[1];
            imuMsg.angular_velocity_covariance[8] = angVelVariance[2];
            const v3d accelVariance = robot.getIMULinearAccelerationVariance();
            imuMsg.linear_acceleration_covariance[0] = accelVariance[0];
            imuMsg.linear_acceleration_covariance[4] = accelVariance[1];
            imuMsg.linear_acceleration_covariance[8] = accelVariance[2];
            // Publish message
            imuMsg.header.stamp = simNow();
            imuMsg.header.frame_id = robot.getName() + "/imu_link";
            imuPub->publish(imuMsg);
        }
    }

    void publishFakeAcousticsData() {
        // if the acoustics transform is availbe
        if (robot.acousticsTransformAvailable() && robot.mapAvailable() && ACOUSTIC_DATA) {
            auto msg = geometry_msgs::msg::Vector3Stamped();
            msg.header.stamp = simNow();
            msg.vector.x = robot.getAcousticsPingTime();

            acousticsPub->publish(msg);
        }
    }

    //================================//
    //       CALLBACK FUNCTIONS       //
    //================================//

    // Publishes fake thruster telemtry messages to make controller work
    void pubThrusterTelemetry() {
        if (vehicleProfile["sim_adapter"].as<std::string>("uwrt") != "uwrt")
            return;
        for (int offset = 0; offset < robot.getThrusterCount(); offset += 4) {
            riptide_msgs2::msg::DshotPartialTelemetry msg;
            msg.start_thruster_num = offset;
            msg.disabled_flags = 0;
            for (int i = 0; i < 4 && offset + i < robot.getThrusterCount(); ++i)
                msg.esc_telemetry[i].thruster_ready = true;
            thrusterTelemetryPub->publish(msg);
        }
    }

    // Once new thruster forces are available, add them to thruster que
    void forceCallback(const std_msgs::msg::Float32MultiArray &thrusterForces) {
        if (static_cast<int>(thrusterForces.data.size()) != robot.getThrusterCount()) {
            RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 2000,
                                  "Ignoring %zu thruster forces; vehicle has %d thrusters", thrusterForces.data.size(),
                                  robot.getThrusterCount());
            return;
        }
        for (float force : thrusterForces.data)
            if (!std::isfinite(force)) {
                RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 2000, "Ignoring nonfinite thruster command");
                return;
            }
        // Create a zero vector with the current time
        thrusterForcesStamped commandedThrust(vXd::Zero(robot.getThrusterCount()), simNow().seconds());
        // Only set forces if robot's enabled
        if (enabled)
            commandedThrust.thrusterForces = convert2eigen(thrusterForces.data);
        robot.addToThrusterQue(commandedThrust);
    }

    // Mirrors software kill to firmware kill. This is done to make the
    // enable/disable button in RViz work
    void killSwitchCallback(const riptide_msgs2::msg::KillSwitchReport &softwareKillMsg) {
        // Make sure the correct kill switch is being called
        if (softwareKillMsg.kill_switch_id == 1) {
            std_msgs::msg::Bool message;
            message.data = softwareKillMsg.switch_asserting_kill;
            enabled = !message.data;
            firmwareKillPub->publish(message);
            // Turn off thrusters if disabled
            if (!enabled)
                robot.stopThrusters();
        }
    }

    // Function called by "/set_sim_pose" service. Updates simulator and EKF to
    // requested position
    void setSim(const std::shared_ptr<robot_localization::srv::SetPose::Request> poseRequest) {
        auto desired = poseRequest->pose;
        if (desired.header.frame_id.empty())
            desired.header.frame_id = "map";
        auto &pose = desired.pose.pose;
        quat q(pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z);
        v3d position(pose.position.x, pose.position.y, pose.position.z);
        if (!q.coeffs().allFinite() || q.norm() < 1e-10 || !position.allFinite()) {
            RCLCPP_ERROR(get_logger(), "Rejecting invalid simulator reset pose");
            return;
        }
        q.normalize();
        if (desired.header.frame_id != "map") {
            try {
                auto t = transformBuffer->lookupTransform("map", desired.header.frame_id, tf2::TimePointZero).transform;
                quat rotation(t.rotation.w, t.rotation.x, t.rotation.y, t.rotation.z);
                position = rotation * position + v3d(t.translation.x, t.translation.y, t.translation.z);
                q = rotation * q;
            } catch (const tf2::TransformException &e) {
                RCLCPP_ERROR(get_logger(), "Cannot transform reset pose: %s", e.what());
                return;
            }
        }
        vXd x = vXd::Zero(13);
        x.head<3>() = position - q * robot.getBaseLinkOffset();
        x.segment<4>(3) << q.w(), q.x(), q.y(), q.z();
        robot.setState(x);
        robot.resetDynamics();
        startState = x; // A deliberate placement becomes the new start pose.
        // Keep the reset pending if navigation has not started yet. Align to the
        // current plant pose when it is ready, rather than an obsolete reset pose.
        ekfAlignmentPending = true;
        alignEkf();
    }

    // Function called by the "reset_sim_to_start" service. Returns the plant to
    // its start pose at rest with thrusters cleared, then reseeds the EKF there
    // exactly like set_sim_pose does.
    void resetSimToStart(const std::shared_ptr<std_srvs::srv::Trigger::Request>,
                         std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
        vXd x = startState;
        x.tail<6>().setZero();
        robot.setState(x);
        robot.resetDynamics();
        ekfAlignmentPending = true;
        alignEkf();
        const quat q = state2quat(x);
        const v3d position = x.head<3>() + q * robot.getBaseLinkOffset();
        const double yaw = std::atan2(2 * (q.w() * q.z() + q.x() * q.y()), 1 - 2 * (q.y() * q.y() + q.z() * q.z()));
        char text[160];
        std::snprintf(text, sizeof text, "Simulator reset to start base_link (%.2f, %.2f, %.2f) m, yaw %.1f deg",
                      position.x(), position.y(), position.z(), yaw * 180 / M_PI);
        response->message = text;
        response->success = true;
        RCLCPP_INFO(get_logger(), "%s", text);
    }

    // Moves the simulated vehicle to the EKF's latest base_link pose (mapped
    // into the simulator's map frame). Velocities, thrusters and the EKF state
    // are all left alone, so this only removes accumulated estimation error
    // from the plant's point of view.
    void syncSimToEstimate(const std::shared_ptr<std_srvs::srv::Trigger::Request>,
                           std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
        response->success = false;
        if (!haveEkfOdom) {
            response->message = "No odometry/filtered received yet";
            return;
        }
        const auto &pose = latestEkfOdom.pose.pose;
        quat q(pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z);
        v3d position(pose.position.x, pose.position.y, pose.position.z);
        if (!q.coeffs().allFinite() || q.norm() < 1e-10 || !position.allFinite()) {
            response->message = "EKF pose is not finite";
            return;
        }
        q.normalize();
        const string frame = latestEkfOdom.header.frame_id.empty() ? ekfFrame : latestEkfOdom.header.frame_id;
        if (frame != "map") {
            try {
                const auto t = transformBuffer->lookupTransform("map", frame, tf2::TimePointZero).transform;
                const quat rotation(t.rotation.w, t.rotation.x, t.rotation.y, t.rotation.z);
                position = rotation * position + v3d(t.translation.x, t.translation.y, t.translation.z);
                q = rotation * q;
            } catch (const tf2::TransformException &e) {
                response->message = string("Cannot transform ") + frame + " to map: " + e.what();
                return;
            }
        }
        // EKF pose is base_link; the plant state is the COM. Keep body velocities.
        vXd x = robot.getState();
        x.head<3>() = position - q * robot.getBaseLinkOffset();
        x.segment<4>(3) << q.w(), q.x(), q.y(), q.z();
        robot.setState(x);
        const double yaw = std::atan2(2 * (q.w() * q.z() + q.x() * q.y()), 1 - 2 * (q.y() * q.y() + q.z() * q.z()));
        char text[160];
        std::snprintf(text, sizeof text, "Simulator moved to EKF base_link (%.2f, %.2f, %.2f) m, yaw %.1f deg",
                      position.x(), position.y(), position.z(), yaw * 180 / M_PI);
        response->message = text;
        response->success = true;
        RCLCPP_INFO(get_logger(), "%s", text);
    }

    void alignEkf() {
        // Only startup and explicit simulator resets seed navigation. Between
        // resets the EKF remains independent, including its sensor noise/drift.
        if (!ekfAlignmentPending || ekfFrame.empty() || !poseClient->service_is_ready())
            return;
        const auto state = robot.getState();
        quat q = state2quat(state);
        v3d position = state.head<3>() + q * robot.getBaseLinkOffset();
        if (ekfFrame != "map") {
            try {
                const auto t = transformBuffer->lookupTransform(ekfFrame, "map", tf2::TimePointZero).transform;
                const quat rotation(t.rotation.w, t.rotation.x, t.rotation.y, t.rotation.z);
                position = rotation * position + v3d(t.translation.x, t.translation.y, t.translation.z);
                q = rotation * q;
            } catch (const tf2::TransformException &) {
                return;
            }
        }
        auto request = std::make_shared<robot_localization::srv::SetPose::Request>();
        request->pose.header.frame_id = ekfFrame;
        request->pose.header.stamp = simNow();
        auto &pose = request->pose.pose.pose;
        pose.position.x = position.x();
        pose.position.y = position.y();
        pose.position.z = position.z();
        pose.orientation.w = q.w();
        pose.orientation.x = q.x();
        pose.orientation.y = q.y();
        pose.orientation.z = q.z();
        for (int i = 0; i < 6; ++i)
            request->pose.pose.covariance[7 * i] = 1e-6;
        // Sending in the EKF's own world frame avoids a startup race with its TF
        // listener. Both poses refer to base_link, not the plant's COM/CAD origin.
        poseClient->async_send_request(request);
        ekfAlignmentPending = false;
        RCLCPP_INFO(get_logger(), "Initialized EKF from simulator base_link in %s", ekfFrame.c_str());
    }

    /**
   * @brief Called on a timer, publishes the following simulator information:
   * 1) Robot's pose for RViz
   * 2) Collision boxes marker array for RViz
   * 3) TF frames for simulated robot and cameras for Zed SDK image faking
   */
    void publishState() {
        {
            // Update collision box markers for RViz display
            publishCollisionMarkers();

            // Getting state information from simulator
            vXd state = robot.getState();
            quat q = state2quat(state);
            v3d baseLinkOffset = q * robot.getBaseLinkOffset(); // Converting baseLink to world frame

            // Getting position
            // Sim uses COM for robot position, need to add offset for position
            // relative to base link
            geometry_msgs::msg::Pose poseMsg;
            poseMsg.position.x = state.x() + baseLinkOffset.x();
            poseMsg.position.y = state.y() + baseLinkOffset.y();
            poseMsg.position.z = state.z() + baseLinkOffset.z();
            // Setting orientation
            poseMsg.orientation.w = q.w();
            poseMsg.orientation.x = q.x();
            poseMsg.orientation.y = q.y();
            poseMsg.orientation.z = q.z();

            // Setting up TF messages
            geometry_msgs::msg::TransformStamped robotFrame;
            geometry_msgs::msg::TransformStamped cameraFrameL;
            geometry_msgs::msg::TransformStamped cameraOpticalFrameL;
            rclcpp::Time clockTime = simNow();
            robotFrame.header.stamp = clockTime;
            cameraFrameL.header.stamp = clockTime;
            cameraOpticalFrameL.header.stamp = clockTime;
            // Frame names
            robotFrame.header.frame_id = "map";
            robotFrame.child_frame_id = "simulator/" + robot.getName() + "/base_link";
            cameraFrameL.header.frame_id = robotFrame.child_frame_id;
            cameraFrameL.child_frame_id = "simulator/" + robot.getName() + "/ffc_camera_link";
            cameraOpticalFrameL.header.frame_id = cameraFrameL.child_frame_id;
            cameraOpticalFrameL.child_frame_id = "simulator/" + robot.getName() + "/ffc_left_camera_optical_frame";
            // Set message position
            robotFrame.transform.translation.x = poseMsg.position.x;
            robotFrame.transform.translation.y = poseMsg.position.y;
            robotFrame.transform.translation.z = poseMsg.position.z;
            if (!vehicleProfile["sim_cameras"])
                cameraFrameL.transform = robot.getLCameraTransform();
            tf2::Quaternion opticalRotation;
            opticalRotation.setRPY(-M_PI_2, 0.0, -M_PI_2);
            cameraOpticalFrameL.transform.rotation.x = opticalRotation.x();
            cameraOpticalFrameL.transform.rotation.y = opticalRotation.y();
            cameraOpticalFrameL.transform.rotation.z = opticalRotation.z();
            cameraOpticalFrameL.transform.rotation.w = opticalRotation.w();
            // Set message orientation
            robotFrame.transform.rotation.w = q.w();
            robotFrame.transform.rotation.x = q.x();
            robotFrame.transform.rotation.y = q.y();
            robotFrame.transform.rotation.z = q.z();

            // Broadcast and publish messages
            statePub->publish(poseMsg);
            nav_msgs::msg::Odometry truth;
            truth.header.stamp = clockTime;
            truth.header.frame_id = "map";
            truth.child_frame_id = robotFrame.child_frame_id;
            truth.pose.pose = poseMsg;
            const v3d baseVelocity = state.segment<3>(7) + state.segment<3>(10).cross(robot.getBaseLinkOffset());
            truth.twist.twist.linear.x = baseVelocity.x();
            truth.twist.twist.linear.y = baseVelocity.y();
            truth.twist.twist.linear.z = baseVelocity.z();
            truth.twist.twist.angular.x = state[10];
            truth.twist.twist.angular.y = state[11];
            truth.twist.twist.angular.z = state[12];
            truthPub->publish(truth);
            std_msgs::msg::Float32MultiArray actual;
            const auto realized = robot.realizedThrusters();
            actual.data.assign(realized.data(), realized.data() + realized.size());
            actualThrusterPub->publish(actual);
            std_msgs::msg::Float64 timeMessage;
            timeMessage.data = robot.simulationTime();
            timePub->publish(timeMessage);
            tf_broadcaster->sendTransform(robotFrame);
            if (!vehicleProfile["sim_cameras"]) {
                tf_broadcaster->sendTransform(cameraFrameL);
                tf_broadcaster->sendTransform(cameraOpticalFrameL);
            } else
                for (const auto &camera : vehicleProfile["sim_cameras"]) {
                    if (camera["truth_tf_owner"].as<std::string>("viewer") != "physics")
                        continue;
                    const auto name = camera["name"].as<std::string>();
                    for (const auto &mount : vehicleProfile["cameras"])
                        if (mount["name"].as<std::string>() == name) {
                            const auto p = mount["pose"].as<std::vector<double>>();
                            const auto base = vehicleProfile["base_link"].as<std::vector<double>>();
                            cameraFrameL.child_frame_id = "simulator/" + robot.getName() + "/" + name + "_camera_link";
                            cameraFrameL.transform.translation.x = p[0] - base[0];
                            cameraFrameL.transform.translation.y = p[1] - base[1];
                            cameraFrameL.transform.translation.z = p[2] - base[2];
                            tf2::Quaternion q;
                            q.setRPY(p[3], p[4], p[5]);
                            cameraFrameL.transform.rotation.x = q.x();
                            cameraFrameL.transform.rotation.y = q.y();
                            cameraFrameL.transform.rotation.z = q.z();
                            cameraFrameL.transform.rotation.w = q.w();
                            cameraOpticalFrameL.header.frame_id = cameraFrameL.child_frame_id;
                            cameraOpticalFrameL.child_frame_id =
                                "simulator/" + robot.getName() + "/" + name + "_left_camera_optical_frame";
                            tf_broadcaster->sendTransform(cameraFrameL);
                            tf_broadcaster->sendTransform(cameraOpticalFrameL);
                        }
                }
        }
    }

    // use this function to refresh any ROS parameters pertaining to simulation
    void refreshSimulationParameters() {
        // reload the sync odom parameter
        this->get_parameter("sync_odom", this->sync_odom);
    }

    //================================//
    //        SIMULATED TIME          //
    //================================//

    // Current simulated time as a ROS time; the only stamp source in this node.
    rclcpp::Time simNow() const {
        return rclcpp::Time(simNs, RCL_ROS_TIME);
    }

    void publishClock() {
        rosgraph_msgs::msg::Clock msg;
        msg.clock = simNow();
        clockPub->publish(msg);
    }

    // Register a periodic job on simulated time. Periods shorter than one
    // physics step run once per step.
    void schedule(double periodSeconds, std::function<void()> job) {
        const int64_t period = std::max<int64_t>(stepNs, std::llround(periodSeconds * 1e9));
        scheduled.push_back({period, simNs + period, std::move(job)});
    }

    // Runs every job that has come due at the current simulated time. Called
    // after each physics step, so a job fires at most once per step.
    void runScheduled() {
        for (auto &job : scheduled) {
            if (simNs < job.nextNs)
                continue;
            job.run();
            job.nextNs += job.periodNs;
            if (job.nextNs <= simNs) // never burst to catch up after a stall
                job.nextNs = simNs + job.periodNs;
        }
    }

    // Allows `ros2 param set .../real_time_factor` to speed up or slow down the
    // plant and, through /clock, the whole stack.
    void refreshRealTimeFactor() {
        const double requested = get_parameter("real_time_factor").as_double();
        if (requested == realTimeFactor)
            return;
        if (!std::isfinite(requested) || requested < 0) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "Ignoring invalid real_time_factor %g", requested);
            return;
        }
        RCLCPP_INFO(get_logger(), "real_time_factor %.3g -> %.3g", realTimeFactor, requested);
        realTimeFactor = requested;
    }

    //================================//
    //       UTILITY FUNCTIONS        //
    //================================//

    // Returns a normalied quaternion from state vector
    quat state2quat(const vXd &state) {
        quat q;
        q.w() = state[3];
        q.vec() = state.segment(4, 3);
        return q.normalized();
    }
    // Returns a vector with added normal distribution noise added to each element
    v3d randomNorm(const v3d &vector, const double &stdv) {
        v3d noisyVector;
        noisyVector[0] = randomNorm(vector[0], stdv);
        noisyVector[1] = randomNorm(vector[1], stdv);
        noisyVector[2] = randomNorm(vector[2], stdv);
        return noisyVector;
    }
    // Returns a number with added normal distribution noise
    double randomNorm(const double &mean, const double &stdv) {
        std::normal_distribution<double> distribution(mean, stdv);
        return distribution(randomGenerator);
    }
    // Converts a std type vector into and Eigen vector
    vXd convert2eigen(std::vector<float> stdVector) {
        Eigen::VectorXf eigenVector = Eigen::Map<Eigen::VectorXf>(stdVector.data(), stdVector.size());
        return eigenVector.cast<double>();
    }

    // Tries to convert a YAML node to a double, if it doesn't exist return
    // default of 0.0 Example: toDouble(config["robot"]["position"]["x"])
    double toDouble(YAML::Node node) {
        try {
            return node.as<double>();
        } catch (const std::exception &e) {
            return 0.0;
        }
    }

    double deg2rad(double degrees) {
        return degrees * M_PI / 180;
    }

    //================================//
    //          VARIABLES             //
    //================================//
    string name;
    Robot robot;
    double physicsStep = .002, contactFriction = .4, restitution = .1;
    bool collisionsEnabled = true, noiseEnabled = true;
    std::unique_ptr<tf2_ros::Buffer> transformBuffer;
    std::shared_ptr<tf2_ros::TransformListener> transformListener;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr truthPub;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr actualThrusterPub;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr timePub;
    bool enabled;
    bool sync_odom;
    std::mt19937 randomGenerator{std::random_device{}()};
    std::vector<collisionBox> robotBoxes;
    std::unique_ptr<TaskContacts> taskContacts;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr clawJointsSub;
    rclcpp::Subscription<visualization_msgs::msg::MarkerArray>::SharedPtr taskObjectsSub;
    struct ScheduledJob {
        int64_t periodNs, nextNs;
        std::function<void()> run;
    };
    std::vector<ScheduledJob> scheduled;
    int64_t simNs = 0, stepNs = 2000000;
    double realTimeFactor = 1.0, clockPublishRate = 500.0;
    rclcpp::Publisher<rosgraph_msgs::msg::Clock>::SharedPtr clockPub;
    double gyroRate, gyroSigma, gyroVariance;
    double dvlSigma, dvlVariance, imuGravity, imuYawDrift;
    rclcpp::Publisher<geometry_msgs::msg::TwistWithCovarianceStamped>::SharedPtr gyroPub;
    std::vector<collisionBox> obstacleBoxes;
    rclcpp::TimerBase::SharedPtr killSwitchTimer;
    rclcpp::TimerBase::SharedPtr paramRefreshTimer;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imuPub;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster;
    rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr statePub;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr firmwareKillPub;
    rclcpp::Client<robot_localization::srv::SetPose>::SharedPtr poseClient;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr ekfOdomSub;
    rclcpp::TimerBase::SharedPtr ekfAlignmentTimer;
    string ekfFrame;
    bool ekfAlignmentPending = true;
    rclcpp::Service<robot_localization::srv::SetPose>::SharedPtr poseService;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr syncService;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr resetStartService;
    vXd startState; // 13-state at launch or last explicit set_sim_pose
    nav_msgs::msg::Odometry latestEkfOdom;
    bool haveEkfOdom = false;
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr thrusterSub;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr collisionBoxPub;
    rclcpp::Publisher<geometry_msgs::msg::TwistWithCovarianceStamped>::SharedPtr dvlPub;
    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr depthPub;
    rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr acousticsPub;
    rclcpp::Subscription<riptide_msgs2::msg::KillSwitchReport>::SharedPtr softwareKillSub;
    rclcpp::Publisher<riptide_msgs2::msg::DshotPartialTelemetry>::SharedPtr thrusterTelemetryPub;
};

//=========================//
//          MAIN           //
//=========================//
int main(int argc, char *argv[]) {
    // Create PhysicsSimNode
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PhysicsSimNode>();
    // Load parameters into Robot class, don't start if unseccessful
    bool startUpSuccess = node->init();
    if (startUpSuccess) {
        // Parameters loaded sucessfully, start node
        RCLCPP_INFO(node->get_logger(), "Simulator node starting...");
        node->rungeKutta4();
        rclcpp::shutdown();
    } else {
        // Parameters failed to load, report error
        RCLCPP_FATAL(node->get_logger(), "SIM FAILED TO START: Could not initialize");
        rclcpp::shutdown();
        return 1;
    }
    return 0;
}
