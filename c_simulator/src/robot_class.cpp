#include "c_simulator/RobotClass.h"
#include <algorithm>

//================================//
//     OBJECT INITIALIZATION      //
//================================//

thrusterForcesStamped::thrusterForcesStamped(vXd thrusterForces_,
                                             double time_) {
  thrusterForces = thrusterForces_;
  time = time_;
}

Robot::Robot() : Robot("talos") {}

Robot::Robot(string name_) {
  name = name_;
  state.resize(13);

  // INITIAL CONDITION
  state << 0, 0, -1, // Position, X Y Z
      1, 0, 0, 0,    // Orientation, W X Y Z
      0, 0, 0,       // Body linear velocity: surge, sway, heave
      0, 0, 0;       // Body angular velocity: roll, pitch, yaw rates

  hasLCameraTransform = false;
  hasDVLTransform = false;
  hasIMUTransform = false;
  hasAcousticsTransform = false;
  hasMapFrame = false;
}

/**
 * @brief Loads vechicle information from robot's config file
 * @param node ROS2 physics simulator node
 * @return whether the results were loaded successfully
 */
bool Robot::loadParams(rclcpp::Node::SharedPtr node) {
  // Creating tf2 buffer and listener
  this->node = node;
  tf_buffer = std::make_unique<tf2_ros::Buffer>(node->get_clock());
  tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);

  // Retrieve the path to the YAML config file from the parameters
  string vehicle_config_file, simulator_config_file;
  node->declare_parameter("vehicle_config", "");
  node->declare_parameter("simulator_config", "");
  if (node->get_parameter("vehicle_config", vehicle_config_file) &&
      node->get_parameter("simulator_config", simulator_config_file)) {
    try {
      RCLCPP_INFO(
          node->get_logger(),
          "Opening vehicle config file: %s and simulator config file %s",
          vehicle_config_file.c_str(), simulator_config_file.c_str());
      // Loading YAML file for parsing
      YAML::Node vehicle_config = YAML::LoadFile(vehicle_config_file);
      YAML::Node simulator_config = YAML::LoadFile(simulator_config_file);
      Robot::storeConfigData(vehicle_config, simulator_config);

      // Loading params successeeded
      return true;
    } catch (const std::exception &e) {
      RCLCPP_ERROR(node->get_logger(), "Failed to parse the config file: %s",
                   e.what());
    }
  } else {
    RCLCPP_ERROR(node->get_logger(),
                 "Failed to retrieve the config file ROS parameter.");
  }

  // Loading params failed
  return false;
}

/**
 * @brief Unpacks YAML contents and stores them into class variables
 */
void Robot::storeConfigData(YAML::Node vehicle_config,
                            YAML::Node simulator_config) {

  const std::string hydroFile =
      node->declare_parameter<std::string>("hydrodynamics_config", "");
  if (hydroFile.empty())
    throw std::invalid_argument(
        "hydrodynamics_config is required; use the physics launch or supply a "
        "validated model YAML");
  const auto hydro = YAML::LoadFile(hydroFile);
  if (hydro["robot"].as<std::string>() != name)
    throw std::invalid_argument(
        "Hydrodynamic model robot does not match namespace");
  if (hydro["schema_version"].as<int>() != 1)
    throw std::invalid_argument("Unsupported hydrodynamic schema");
  if (hydro["parameter_status"].as<std::string>("unvalidated_prior") !=
      "identified")
    RCLCPP_WARN(node->get_logger(),
                "Hydrodynamic coefficients are UNVALIDATED PRIORS; pool "
                "identification is required before gain transfer");
  auto matrix6 = [](const YAML::Node &n) {
    c_simulator::Matrix6d m;
    if (n.size() == 36) {
      for (int i = 0; i < 36; ++i)
        m(i / 6, i % 6) = n[i].as<double>();
    } else if (n.size() == 6) {
      for (int r = 0; r < 6; ++r) {
        if (n[r].size() != 6)
          throw std::invalid_argument("Expected 6x6 matrix");
        for (int c = 0; c < 6; ++c)
          m(r, c) = n[r][c].as<double>();
      }
    } else
      throw std::invalid_argument(
          "Expected flat 36-element or nested 6x6 matrix");
    return m;
  };
  mass = vehicle_config["mass"].as<double>();
  const v3d r_com = std2v3d(vehicle_config["com"].as<std::vector<double>>());
  const auto inertia = hydro["rigid_body_inertia3x3"].as<std::vector<double>>();
  if (inertia.size() != 9)
    throw std::invalid_argument("Expected row-major 3x3 rigid inertia");
  m3d bodyInertia;
  for (int r = 0; r < 3; ++r)
    for (int c = 0; c < 3; ++c)
      bodyInertia(r, c) = inertia[r * 3 + c];
  marineDynamics.configure(mass, bodyInertia, matrix6(hydro["added_mass6x6"]));
  invBodyInertia = marineDynamics.mass().bottomRightCorner<3, 3>().inverse();
  const auto quadratic = hydro["quadratic_damping"].as<std::vector<double>>();
  if (quadratic.size() != 6)
    throw std::invalid_argument("Expected six quadratic damping coefficients");
  c_simulator::Vector6d qd;
  for (int i = 0; i < 6; ++i)
    qd[i] = quadratic[i];
  marineDynamics.configureDamping(
      matrix6(hydro["linear_damping6x6"]), qd,
      std2v3d(hydro["damping_center_relative"].as<std::vector<double>>()));
  marineDynamics.configureHydrostatics(
      hydro["water_density"].as<double>(),
      hydro["displaced_volume"].as<double>(),
      std2v3d(hydro["cob_relative"].as<std::vector<double>>()),
      std2v3d(hydro["buoyancy_radii"].as<std::vector<double>>()), GRAVITY,
      hydro["water_level"].as<double>(0));
  waterCurrentWorld =
      std2v3d(hydro["current_velocity"].as<std::vector<double>>());
  currentAmplitude =
      std2v3d(hydro["current_oscillation_amplitude"].as<std::vector<double>>());
  currentFrequency = hydro["current_oscillation_frequency"].as<double>(0);
  if (!std::isfinite(currentFrequency) || currentFrequency < 0)
    throw std::invalid_argument("Invalid current frequency");
  // Every pose in vehicle YAML is relative to the CAD origin, including
  // base_link.
  r_baseLink =
      std2v3d(vehicle_config["base_link"].as<std::vector<double>>()) - r_com;
  for (const auto &camera : vehicle_config["cameras"]) {
    if (camera["name"].as<std::string>() != "ffc")
      continue;
    const auto pose = camera["pose"].as<std::vector<double>>();
    if (pose.size() != 6)
      throw std::invalid_argument("Expected six camera pose elements");
    const v3d mount =
        std2v3d(pose) -
        std2v3d(vehicle_config["base_link"].as<std::vector<double>>());
    const quat q = rpy2quat(pose[3], pose[4], pose[5]);
    lCameraTransform.translation.x = mount.x();
    lCameraTransform.translation.y = mount.y();
    lCameraTransform.translation.z = mount.z();
    lCameraTransform.rotation.w = q.w();
    lCameraTransform.rotation.x = q.x();
    lCameraTransform.rotation.y = q.y();
    lCameraTransform.rotation.z = q.z();
  }
  // Getting IMU information
  std::vector<double> imu_pose =
      vehicle_config["imu"]["pose"].as<std::vector<double>>();
  r_imu = v3d(imu_pose[0], imu_pose[1], imu_pose[2]) - r_com;
  q_imu = rpy2quat(imu_pose[3], imu_pose[4], imu_pose[5]);
  if (vehicle_config["fog"]) {
    const auto pose = vehicle_config["fog"]["pose"].as<std::vector<double>>();
    q_gyro = rpy2quat(pose.at(3), pose.at(4), pose.at(5));
  }
  const YAML::Node sensorProperties = simulator_config["sensor_properties"];
  imu_rate = 1.0 / sensorProperties["imu_rate"].as<double>(
                       vehicle_config["imu"]["rate"].as<double>());
  imu_yawDrift = vehicle_config["imu"]["yaw_drift"].as<double>();
  imu_sigmaAccel = vehicle_config["imu"]["sigma_accel"].as<double>();
  imu_sigmaOmega =
      vehicle_config["imu"]["sigma_omega"].as<double>() * M_PI / 180;
  imu_sigmaAngle =
      vehicle_config["imu"]["sigma_angle"].as<double>() * M_PI / 180;
  imu_orientationVariance =
      sensorProperties["imu_orientation_variance"]
          ? std2v3d(sensorProperties["imu_orientation_variance"]
                        .as<std::vector<double>>())
          : v3d::Constant(imu_sigmaAngle * imu_sigmaAngle);
  imu_angularVelocityVariance =
      sensorProperties["imu_angular_velocity_variance"]
          ? std2v3d(sensorProperties["imu_angular_velocity_variance"]
                        .as<std::vector<double>>())
          : v3d::Constant(imu_sigmaOmega * imu_sigmaOmega);
  imu_linearAccelerationVariance =
      sensorProperties["imu_linear_acceleration_variance"]
          ? std2v3d(sensorProperties["imu_linear_acceleration_variance"]
                        .as<std::vector<double>>())
          : v3d::Constant(imu_sigmaAccel * imu_sigmaAccel);
  // Getting depth sensor information
  r_depth = std2v3d(vehicle_config["depth"]["pose"].as<std::vector<double>>()) -
            r_com;
  depth_rate = 1.0 / vehicle_config["depth"]["rate"].as<double>();
  depth_sigma = vehicle_config["depth"]["sigma"].as<double>();
  // Getting dvl sensor information
  std::vector<double> dvl_pose =
      vehicle_config["dvl"]["pose"].as<std::vector<double>>();
  r_dvl = v3d(dvl_pose[0], dvl_pose[1], dvl_pose[2]) - r_com;
  q_dvl = rpy2quat(dvl_pose[3], dvl_pose[4], dvl_pose[5]);
  dvl_rate = 1.0 / vehicle_config["dvl"]["rate"].as<double>();
  dvl_sigma = vehicle_config["dvl"]["sigma"].as<double>();

  // Creating thruster forces -> body forces & torques matrix by looping through
  // each thruster
  YAML::Node thrusters = vehicle_config["thrusters"];
  thrusterCount = thrusters.size();
  const YAML::Node dynamics = hydro["thruster_dynamics"];
  maxThrust = dynamics["forward_max_force"].as<double>();
  thrusterDelay = dynamics["delay"].as<double>(0.1);
  thrusterRiseTime = dynamics["rise_time_constant"].as<double>(0.08);
  thrusterFallTime = dynamics["fall_time_constant"].as<double>(0.06);
  thrusterSlewRate = dynamics["slew_rate"].as<double>(0.0);
  thrusterDeadband = dynamics["force_deadband"].as<double>(0.0);
  thrusterForwardScale = dynamics["forward_scale"].as<double>(1.0);
  thrusterReverseScale = dynamics["reverse_scale"].as<double>(1.0);
  thrusterMatrix.resize(thrusterCount, 6);
  thrusterPositions.clear();
  thrusterDirections.clear();
  propellerRadius = dynamics["propeller_radius"].as<double>(.05);
  if (!std::isfinite(propellerRadius) || propellerRadius <= 0)
    throw std::invalid_argument("Invalid propeller radius");
  std::vector<c_simulator::ThrusterParameters> actuatorParameters(
      thrusterCount);
  const auto efficiencies =
      hydro["thruster_efficiencies"].as<std::vector<double>>();
  if (int(efficiencies.size()) != thrusterCount)
    throw std::invalid_argument("Expected one efficiency per thruster");
  for (int i = 0; i < thrusterCount; ++i) {
    auto &p = actuatorParameters[i];
    p.delay = thrusterDelay;
    p.rise = thrusterRiseTime;
    p.fall = thrusterFallTime;
    p.slew = thrusterSlewRate;
    p.deadband = thrusterDeadband;
    p.forwardLimit = maxThrust;
    p.reverseLimit = dynamics["reverse_max_force"].as<double>(maxThrust);
    p.forwardScale = thrusterForwardScale;
    p.reverseScale = thrusterReverseScale;
    p.efficiency = efficiencies[i];
  }
  actuator.configure(actuatorParameters,
                     dynamics["command_timeout"].as<double>(.5));
  // Thruster info
  int row = 0;
  for (auto thruster : thrusters) {
    // Get thruster position and orientation
    std::vector<double> pose = thruster["pose"].as<std::vector<double>>();
    v3d thrusterPos = std2v3d(pose);
    quat thrusterDirection = rpy2quat(pose[3], pose[4], pose[5]);
    // Body force caused by unit thrust vector:
    v3d bodyForce = thrusterDirection * v3d(1, 0, 0);
    thrusterPositions.push_back(thrusterPos - r_com);
    thrusterDirections.push_back(bodyForce);
    // Body torque caused by unit thrust vector (T = r x F):
    v3d bodyTorque = (thrusterPos - r_com).cross(bodyForce);
    // Storing results into matrix
    thrusterMatrix.block(row, 0, 1, 3) = bodyForce.transpose();
    thrusterMatrix.block(row, 3, 1, 3) = bodyTorque.transpose();
    row++;
  }
  // Initialize thruster forces and torques to zero
  forces.setZero();
  torques.setZero();
  linAccel.setZero();
  angAccel.setZero();
  for (double period : {imu_rate, dvl_rate, depth_rate})
    if (!std::isfinite(period) || period <= 0)
      throw std::invalid_argument("Sensor rates must be positive/finite");
  setAccel(stateDerivative(state));

  if (ACOUSTIC_DATA) {
    // acoustics stuff
    speedOfSound = vehicle_config["acoustics"]["speed_of_sound"].as<double>();
    std::vector<double> fakePingerPose =
        simulator_config["acoustics"]["fake_pinger"]["pose"]
            .as<std::vector<double>>();
    fakePingerPosition =
        v3d(fakePingerPose[0], fakePingerPose[1], fakePingerPose[2]);
  }
}

// The ROS adapter delegates all hydrodynamics to the reusable model library.
vXd Robot::stateDerivative(const vXd &x, double stageOffset) const {
  const double phase =
      2 * M_PI * currentFrequency * (actuator.time() + stageOffset);
  return marineDynamics.derivative(
      x, propulsionWrench(x), waterCurrentWorld + currentAmplitude * sin(phase),
      currentAmplitude * (2 * M_PI * currentFrequency * cos(phase)));
}
c_simulator::Vector6d Robot::propulsionWrench(const vXd &x) const {
  const quat q = quat(x[3], x[4], x[5], x[6]).normalized();
  auto thrust = actuator.forces();
  for (int i = 0; i < thrusterCount; ++i) {
    double z = (x.head<3>() + q * thrusterPositions[i]).z();
    double axisZ = (q * thrusterDirections[i]).z();
    double extent =
        std::max(.001, propellerRadius * sqrt(std::max(0., 1 - axisZ * axisZ)));
    double c = std::clamp((marineDynamics.waterLevel() - z) / extent, -1., 1.);
    // Fraction of the propeller disk below water; air thrust is neglected.
    thrust[i] *= (acos(-c) + c * sqrt(std::max(0., 1 - c * c))) / M_PI;
  }
  return thrusterMatrix.transpose() * thrust;
}
void Robot::stopThrusters() { actuator.stop(); }
void Robot::resetDynamics() {
  actuator.reset();
  forces.setZero();
  torques.setZero();
  setAccel(stateDerivative(state));
}

// Sets the robot's state vector, quaternion is renormalized on update
void Robot::setState(vXd state_) {
  if (state_.size() != 13)
    throw std::invalid_argument("Expected 13-state vehicle vector");
  c_simulator::MarineDynamics::validateState(state_);
  // Normalizing quaternion before assigning state
  quat q(state_[3], state_[4], state_[5], state_[6]);
  q.normalize();
  state_[3] = q.w();
  state_.segment(4, 3) = q.vec();
  // Update state now that the quaternion has been normalized
  state = state_;
}

// Stores world-frame COM acceleration and body angular acceleration for sensor
// synthesis.
void Robot::setAccel(const vXd &stateDot) {
  quat q(state[3], state[4], state[5], state[6]);
  const v3d bodyVelocity = state.segment(7, 3);
  const v3d bodyOmega = state.segment(10, 3);
  linAccel = q * (stateDot.segment(7, 3) + bodyOmega.cross(bodyVelocity)) +
             v3d(0, 0, GRAVITY);
  angAccel = q * stateDot.segment(10, 3);
}

void Robot::addToThrusterQue(thrusterForcesStamped commandedThrust) {
  actuator.command(commandedThrust.thrusterForces);
}
void Robot::updateThrusters(double dt) {
  actuator.advance(dt);
  setForcesTorques(actuator.forces());
}

// Converts realized thruster forces into robot body forces and torques.
void Robot::setForcesTorques(const vXd &thrusterForces) {
  vXd forcesTorques = thrusterMatrix.transpose() * thrusterForces;
  forces = forcesTorques.segment(0, 3);
  torques = forcesTorques.segment(3, 3);
}

//================================//
//       GETTER FUNCTIONS         //
//================================//
geometry_msgs::msg::Transform Robot::getLCameraTransform() {
  if (!hasLCameraTransform) {
    // The current vehicle description mounts the forward ZED at `ffc`.
    // Resolve the physical camera mount supplied by
    // riptide_descriptions2. The optical-axis rotation is published as a
    // separate child TF by physics_simulator; applying it here would also
    // rotate the OpenGL renderer's already-correct camera view.
    string fromFrameRel = name + "/ffc_camera_link";
    string toFrameRel = name + "/base_link";
    geometry_msgs::msg::Transform mount =
        safeTransform(toFrameRel, fromFrameRel, hasLCameraTransform);
    if (hasLCameraTransform)
      lCameraTransform = mount;
  }
  return lCameraTransform;
}
bool Robot::mapAvailable() {
  if (!hasMapFrame) {
    string toFrameRel = "map";
    string fromFrameRel = "world";
    safeTransform(toFrameRel, fromFrameRel, hasMapFrame);
  }
  return hasMapFrame;
}
bool Robot::dvlTransformAvailable() {
  if (!hasDVLTransform) {
    string toFrameRel = name + "/dvl_link";
    string fromFrameRel = name + "/base_link";
    safeTransform(toFrameRel, fromFrameRel, hasDVLTransform);
  }
  return hasDVLTransform;
}
bool Robot::imuTransformAvailable() {
  if (!hasIMUTransform) {
    string toFrameRel = name + "/imu_link";
    string fromFrameRel = name + "/base_link";
    safeTransform(toFrameRel, fromFrameRel, hasIMUTransform);
  }
  return hasIMUTransform;
}
bool Robot::acousticsTransformAvailable() {
  if (!hasAcousticsTransform) {
    string toFrameRel_port = name + "/acoustics_port_link";
    string toFrameRel_starboard = name + "/acoustics_starboard_link";
    string fromFrameRel = name + "/base_link";

    bool port, starboard = false;

    safeTransform(toFrameRel_port, fromFrameRel, port);
    safeTransform(toFrameRel_starboard, fromFrameRel, starboard);

    hasAcousticsTransform = port && starboard;
  }

  return hasAcousticsTransform;
}
double Robot::getMass() { return mass; }
vXd Robot::getState() { return state; }
v3d Robot::getLatestAngAccel() { return angAccel; }
v3d Robot::getLatestLinAccel() { return linAccel; }
m3d Robot::getInvInertia() { return invBodyInertia; }
c_simulator::Matrix6d Robot::getInverseMass() {
  return marineDynamics.inverseMass();
}
v3d Robot::getWaterCurrentWorld() { return waterCurrentWorld; }
double Robot::getIMUDrift() { return imu_yawDrift; }
v3d Robot::getThrusterForces() { return forces; }
v3d Robot::getThrusterTorques() { return torques; }
v3d Robot::getBaseLinkOffset() { return r_baseLink; }
string Robot::getName() { return name; }
v3d Robot::getDepthOffset() { return r_depth; }
double Robot::getDepthSigma() { return depth_sigma; }
double Robot::getDepthRate() { return depth_rate; }
v3d Robot::getIMUOffset() { return r_imu; }
v3d Robot::getIMUSigma() {
  return v3d(imu_sigmaAccel, imu_sigmaOmega, imu_sigmaAngle);
}
v3d Robot::getIMUOrientationVariance() { return imu_orientationVariance; }
v3d Robot::getIMUAngularVelocityVariance() {
  return imu_angularVelocityVariance;
}
v3d Robot::getIMULinearAccelerationVariance() {
  return imu_linearAccelerationVariance;
}
double Robot::getIMURate() { return imu_rate; }
quat Robot::getIMUQuat() { return q_imu; }
quat Robot::getGyroQuat() { return q_gyro; }
v3d Robot::getDVLOffset() { return r_dvl; }
double Robot::getDVLSigma() { return dvl_sigma; }
double Robot::getDVLRate() { return dvl_rate; }
quat Robot::getDVLQuat() { return q_dvl; }
int Robot::getThrusterCount() { return thrusterCount; }
double Robot::getAcousticsPingTime() {
  // calculate the time between the port and starboard acoustics pods recieving
  // pulses port_time - starboard_time

  // get the pod locations
  string portFrame = name + "/acoustics_port_link";
  string starboardFrame = name + "/acoustics_starboard_link";
  string worldFrame = "talos/base_inertia";

  bool success = false;
  geometry_msgs::msg::Vector3 portTranslation =
      safeTransform(worldFrame, portFrame, success).translation;
  geometry_msgs::msg::Vector3 starboardTranslation =
      safeTransform(worldFrame, starboardFrame, success).translation;

  v3d portTranslation_v3d(portTranslation.x, portTranslation.y,
                          portTranslation.z);
  v3d starboardTranslation_v3d(starboardTranslation.x, starboardTranslation.y,
                               starboardTranslation.z);
  quat worldRotation(state[3], state[4], state[5], state[6]);
  v3d worldTranslation_v3d(state[0], state[1], state[2]);

  // transform these info world frame
  portTranslation_v3d =
      (worldRotation * portTranslation_v3d) + worldTranslation_v3d;
  starboardTranslation_v3d =
      (worldRotation * starboardTranslation_v3d) + worldTranslation_v3d;

  double portDistance =
      sqrt(pow(portTranslation_v3d[0] - fakePingerPosition[0], 2) +
           pow(portTranslation_v3d[1] - fakePingerPosition[1], 2) +
           pow(portTranslation_v3d[2] - fakePingerPosition[2], 2));
  double starboardDistance =
      sqrt(pow(starboardTranslation_v3d[0] - fakePingerPosition[0], 2) +
           pow(starboardTranslation_v3d[1] - fakePingerPosition[1], 2) +
           pow(starboardTranslation_v3d[2] - fakePingerPosition[2], 2));

  return (portDistance - starboardDistance) / this->speedOfSound;
}

//================================//
//       UTILITY FUNCTIONS        //
//================================//

geometry_msgs::msg::Transform
Robot::safeTransform(string toFrame, string fromFrame, bool &transformFlag) {
  // NOTE: THIS IS A TRANSFORM BETWEEN EKF FRAMES AND IS NOT A PERFECT PHYSICAL
  // TRANSFORM FOR PHYSICS BASED TRANSFORMATIONS, USE STATE

  try {
    geometry_msgs::msg::TransformStamped t =
        tf_buffer->lookupTransform(toFrame, fromFrame, tf2::TimePointZero);
    transformFlag = true;
    return t.transform;
  } catch (const tf2::TransformException &ex) {
    RCLCPP_ERROR_SKIPFIRST_THROTTLE(node->get_logger(), *node->get_clock(),
                                    2000, "Could not transform %s to %s: %s",
                                    toFrame.c_str(), fromFrame.c_str(),
                                    ex.what());
    return geometry_msgs::msg::Transform();
  }
}
quat Robot::rpy2quat(double roll, double pitch, double yaw) {
  tf2::Quaternion tfQuat;
  tfQuat.setRPY(roll, pitch, yaw);
  return quat(tfQuat.w(), tfQuat.x(), tfQuat.y(), tfQuat.z()).normalized();
}
v3d Robot::std2v3d(std::vector<double> stdVect) {
  if (stdVect.size() < 3)
    throw std::invalid_argument("Expected at least three vector components");
  for (double v : stdVect)
    if (!std::isfinite(v))
      throw std::invalid_argument("Vector must be finite");
  return v3d(stdVect[0], stdVect[1], stdVect[2]);
}
