#include "c_simulator/RobotClass.h"

using namespace std::chrono_literals;

//================================//
//     OBJECT INITIALIZATION      //
//================================//

thrusterForcesStamped::thrusterForcesStamped(vXd thrusterForces_, double time_)
{
    thrusterForces = thrusterForces_;
    time = time_;
}

Robot::Robot()
{
    Robot("talos");
}

Robot::Robot(string name_)
{
    name = name_;
    state.resize(13);

    // INITIAL CONDITION
    state << 0, 0, -1, // Position, X Y Z
        1, 0, 0, 0,    // Orientation, W X Y Z
        0, 0, 0,       // Linear momentum, X Y Z
        0, 0, 0.5;     // Angular momentum, X Y Z

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
bool Robot::loadParams(rclcpp::Node::SharedPtr node)
{
    // Creating tf2 buffer and listener
    this->node = node;
    tf_buffer = std::make_unique<tf2_ros::Buffer>(node->get_clock());
    tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);

    ballast_stamp = node->get_clock()->now();

    // Retrieve the path to the YAML config file from the parameters
    string vehicle_config_file, simulator_config_file;
    node->declare_parameter("vehicle_config", "");
    node->declare_parameter("simulator_config", "");
    if (node->get_parameter("vehicle_config", vehicle_config_file) && node->get_parameter("simulator_config", simulator_config_file))
    {
        try
        {
            RCLCPP_INFO(node->get_logger(), "Opening vehicle config file: %s and simulator config file %s", vehicle_config_file.c_str(), simulator_config_file.c_str());
            // Loading YAML file for parsing
            YAML::Node vehicle_config = YAML::LoadFile(vehicle_config_file);
            YAML::Node simulator_config = YAML::LoadFile(simulator_config_file);
            Robot::storeConfigData(vehicle_config, simulator_config);

            // Loading params successeeded
            return true;
        }
        catch (const YAML::Exception &e)
        {
            RCLCPP_ERROR(node->get_logger(), "Failed to parse the config file: %s, line %d, col %d, pos %d", e.what(), e.mark.line, e.mark.column, e.mark.pos);
        } catch(const std::runtime_error& e)
        {
            RCLCPP_ERROR(node->get_logger(), "Failed to parse the config file: %s", e.what());
        }
    }
    else
    {
        RCLCPP_ERROR(node->get_logger(), "Failed to retrieve the config file ROS parameter.");
    }

    // Loading params failed
    return false;
}

/**
 * @brief Unpacks YAML contents and stores them into class variables
 */
void Robot::storeConfigData(const YAML::Node& vehicle_config, const YAML::Node& simulator_config)
{
    // Getting mass information
    mass = getYamlNodeAs<double>(vehicle_config, {"mass"});
    weightVector = {0, 0, -mass * GRAVITY};
    
    v3d r_com = std2v3d(getYamlNodeAs<std::vector<double>>(vehicle_config, {"com"}));

    // Getting inertia information

    std::vector<double> bodyInertiaArray = getYamlNodeAs<std::vector<double>>(simulator_config, {"vehicle_properties", name, "inertia3x3"});
    m3d bodyInertia;
    bodyInertia << bodyInertiaArray[0], bodyInertiaArray[1], bodyInertiaArray[2],
        bodyInertiaArray[3], bodyInertiaArray[4], bodyInertiaArray[5],
        bodyInertiaArray[6], bodyInertiaArray[7], bodyInertiaArray[8];
    invBodyInertia = bodyInertia.inverse();

    try
    {
        // Get bouyancy vector from feed forward
        std::vector<double> feedForward = getYamlNodeAs<std::vector<double>>(simulator_config, {"vehicle_properties", name, "base_wrench"});
    
        bouyancyVector = v3d(0, 0, -feedForward[2]) - weightVector;
        // Calculate COB from feed forward: r = T/F
        r_cob = v3d(feedForward[4] / bouyancyVector.norm(), - feedForward[3] / bouyancyVector.norm(), 0.005);
    } catch(std::runtime_error& e)
    {
        double hullVolume = getYamlNodeAs<double>(vehicle_config, {"hull_volume"});
        bouyancyVector = v3d(0, 0, WATER_DENSITY * GRAVITY * hullVolume);
        r_cob = std2v3d(getYamlNodeAs<std::vector<double>>(simulator_config, {"vehicle_properties", name, "cob"}));
    }

    //center of drag
    r_cod = std2v3d(getYamlNodeAs<std::vector<double>>(simulator_config, {"vehicle_properties", name, "cod"})) - r_com;

    //  Getting base link position relative to center of mass
    r_baseLink = std2v3d(getYamlNodeAs<std::vector<double>>(vehicle_config, {"base_link"}));

    // Getting active ballast information (if available)
    ballast_enabled = false;
    try
    {   
        double ballast_volume = getYamlNodeAs<double>(vehicle_config, {"active_ballast", "volume"});
        ballast_max_mass = WATER_DENSITY * ballast_volume;
        ballast_in_flow_rate = getYamlNodeAs<double>(vehicle_config, {"active_ballast", "in_flow_rate"});
        ballast_out_flow_rate = getYamlNodeAs<double>(vehicle_config, {"active_ballast", "out_flow_rate"});

        //all settings parsed correctly, enable active ballast
        ballast_enabled = true;
    } catch(std::runtime_error& e)
    {
        RCLCPP_INFO(node->get_logger(), "Could not parse all active ballast settings (%s). Continuing with active ballast disabled", e.what());
    }


    // Getting IMU information
    imu_enabled = false;
    try
    {
        std::vector<double> imu_pose = getYamlNodeAs<std::vector<double>>(vehicle_config, {"imu", "pose"});
        r_imu = v3d(imu_pose[0], imu_pose[1], imu_pose[2]) - r_com;
        q_imu = rpy2quat(imu_pose[3], imu_pose[4], imu_pose[5]);
        imu_rate = 1.0 / getYamlNodeAs<double>(vehicle_config, {"imu", "rate"});
        imu_yawDrift = getYamlNodeAs<double>(vehicle_config, {"imu", "yaw_drift"});
        imu_sigmaAccel = getYamlNodeAs<double>(vehicle_config, {"imu", "sigma_accel"});
        imu_sigmaOmega = getYamlNodeAs<double>(vehicle_config, {"imu", "sigma_omega"}) * M_PI / 180;
        imu_sigmaAngle = getYamlNodeAs<double>(vehicle_config, {"imu", "sigma_angle"}) * M_PI / 180;
        
        //all settings parsed correctly, enable IMU
        imu_enabled = true;
    } catch(std::runtime_error& e)
    {
        RCLCPP_INFO(node->get_logger(), "Could not parse all IMU Settings (%s). Continuing with IMU disabled", e.what());
    }

    // Getting depth sensor information
    depth_enabled = false;
    try
    {
        r_depth = std2v3d(getYamlNodeAs<std::vector<double>>(vehicle_config, {"depth", "pose"})) - r_com;
        depth_rate = 1.0 / getYamlNodeAs<double>(vehicle_config, {"depth", "rate"});
        depth_sigma = getYamlNodeAs<double>(vehicle_config, {"depth", "sigma"});

        //all settings parsed correctly, enable depth
        depth_enabled = true;
    } catch(std::runtime_error& e)
    {
        RCLCPP_INFO(node->get_logger(), "Could not parse all depth sensor settings (%s). Continuing with depth disabled", e.what());
    }

    // Getting dvl sensor information
    dvl_enabled = false;
    try
    {
        std::vector<double> dvl_pose = getYamlNodeAs<std::vector<double>>(vehicle_config, {"dvl", "pose"});
        r_dvl = v3d(dvl_pose[0], dvl_pose[1], dvl_pose[2]) - r_com;
        q_dvl = rpy2quat(dvl_pose[3], dvl_pose[4], dvl_pose[5]);
        dvl_rate = 1.0 / getYamlNodeAs<double>(vehicle_config, {"dvl", "rate"});
        dvl_sigma = getYamlNodeAs<double>(vehicle_config, {"dvl", "sigma"});

        //all settings parsed correctly, enable dvl
        dvl_enabled = true;
    } catch(std::runtime_error& e)
    {
        RCLCPP_INFO(node->get_logger(), "Could not parse all DVL sensor settings (%s). Continuing with DVL disabled", e.what());
    }

    // Drag information
    dragCoef = getYamlNodeAs<std::vector<double>>(simulator_config, {"vehicle_properties", name, "damping"});

    // Creating thruster forces -> body forces & torques matrix by looping through each thruster
    YAML::Node thrusters = vehicle_config["thrusters"];
    maxThrust = getYamlNodeAs<double>(simulator_config, {"vehicle_properties", name, "thruster_max_force"});
    thrusterCount = thrusters.size();
    RCLCPP_INFO(node->get_logger(), "%d thrusters detected.", thrusterCount);
    thrusterMatrix.resize(thrusterCount, 6);

    // Thruster info
    int row = 0;
    for (auto thruster : thrusters)
    {
        // Get thruster position and orientation
        std::vector<double> pose = getYamlNodeAs<std::vector<double>>(thruster, {"pose"});
        v3d thrusterPos = std2v3d(pose);
        quat thrusterDirection = rpy2quat(pose[3], pose[4], pose[5]);

        // Body force caused by unit thrust vector:
        v3d bodyForce = thrusterDirection * v3d(1, 0, 0);

        // Body torque caused by unit thrust vector (T = r x F):
        v3d bodyTorque = (thrusterPos - r_com).cross(bodyForce);
        
        // Storing results into matrix
        thrusterMatrix.block(row, 0, 1, 3) = bodyForce.transpose();
        thrusterMatrix.block(row, 3, 1, 3) = bodyTorque.transpose();
        row++;
    }
    // Initialize thruster forces and torques to zero
    forces.Zero();
    torques.Zero();

    acousticsEnabled = false;
    if (ACOUSTIC_DATA)
    {
        try
        {
            // acoustics stuff
            speedOfSound = getYamlNodeAs<double>(vehicle_config, {"acoustics", "speed_of_sound"});
            
            std::vector<double> fakePingerPose = getYamlNodeAs<std::vector<double>>(simulator_config, {"acoustics", "fake_pinger", "pose"});
            fakePingerPosition = v3d(fakePingerPose[0], fakePingerPose[1], fakePingerPose[2]);
            
            acousticsEnabled = true;
        } catch(std::runtime_error& e)
        {
            RCLCPP_INFO(node->get_logger(), "Could not parse acoustics information(%s). Continuing with acoustics disabled.", e.what());
        }
    } else
    {
        RCLCPP_INFO(node->get_logger(), "Acoustics is disabled.");
    }
}

//================================//
//    MATH/PHYSICS FUNCTIONS      //
//================================//

/**
 * @param linVel Linear velocity of the robot in the body frame
 * @param depth Depth of the robot
 * @return Calculated drag force vector in the body frame
 */
v3d Robot::calcDragForces(const v3d &linVel, const double &depth)
{
    // Drag coeffecients from YAML file, obtained experimentally
    // drag = (k1 + k2 * abs(v) + k3 * exp(abs(v) / k4) )* sign(v)
    double dragX = -dragCoef[2] + dragCoef[1] * abs(linVel[0]) + dragCoef[2] * exp(abs(linVel[0]) / dragCoef[3]);
    double dragY = -dragCoef[6] + dragCoef[5] * abs(linVel[1]) + dragCoef[6] * exp(abs(linVel[1]) / dragCoef[7]);
    double dragZ = -dragCoef[10] + dragCoef[9] * abs(linVel[2]) + dragCoef[10] * exp(abs(linVel[2]) / dragCoef[11]);
    // return magnitude * unit direction opposite of velocity
    return getScaleFactor(depth) * v3d(dragX, dragY, dragZ).norm() * -linVel.normalized();
}

/**
 * @param linVel Linear velocity of the robot in the body frame
 * @param angVel Angular velocity of the robot in the body frame
 * @param depth Depth of the robot
 * @return Calculated drag torque vector in the body frame
 */
v3d Robot::calcDragTorques(const v3d &linVel, const v3d &angVel, const double &depth)
{
    // Drag coeffecients from YAML file, obtained experimentally
    // Drag force resisting rotation
    // drag = (k1 + k2 * abs(v) + k3 * exp(abs(v) / k4) )* sign(v)
    double dragTorqX = -dragCoef[14] + dragCoef[13] * abs(angVel[0]) + dragCoef[14] * exp(abs(angVel[0]) / dragCoef[15]);
    double dragTorqY = -dragCoef[18] + dragCoef[17] * abs(angVel[1]) + dragCoef[18] * exp(abs(angVel[1]) / dragCoef[19]);
    double dragTorqZ = -dragCoef[22] + dragCoef[21] * abs(angVel[2]) + dragCoef[22] * exp(abs(angVel[2]) / dragCoef[23]);
    v3d dragTorques = getScaleFactor(depth) * v3d(dragTorqX, dragTorqY, dragTorqZ).norm() * -angVel.normalized();
    // Adding drag torque caused by linear drag force and offset center of drag
    dragTorques += r_cod.cross(calcDragForces(linVel, depth));
    return dragTorques;
}

/**
 * @param q Quaternion representing robot's orientation
 * @param depth Depth of the robot
 * @return Calculated bouyant torque vector due to offset COM and COB in world frame
 */
v3d Robot::calcBouyantTorque(const quat &q, const double &depth)
{
    // T = r x F
    // Need to convert distance to COB to world cordinates first though
    return (q * r_cob).cross(getScaleFactor(depth) * bouyancyVector);
}

/**
 * @brief Used to scale down bouyancy effects as the robot comes out of the water
 */
double Robot::getScaleFactor(const double &depth)
{
    double scaleFactor;
    if (depth > VEHICLE_HEIGHT / 2)
        // Vehicle is out of water, net force is only from gravity
        scaleFactor = 0;
    else if (depth < -VEHICLE_HEIGHT / 2)
        // Vehicle is fully submerged
        scaleFactor = 1;
    else
        // Vehicle is partially submerged, scale based on how out of the water the robot is
        scaleFactor = -0.5 * sin(M_PI / VEHICLE_HEIGHT * depth) + 0.5;
    return scaleFactor;
}

//================================//
//       SETTER FUNCTIONS         //
//================================//

// Sets the robot's state vector, quaternion is renormalized on update
void Robot::setState(vXd state_)
{
    // Normalizing quaternion before assigning state
    quat q(state_[3], state_[4], state_[5], state_[6]);
    q.normalize();
    state_[3] = q.w();
    state_.segment(4, 3) = q.vec();
    // Update state now that the quaternion has been normalized
    state = state_;

    // Check thruster que
    // The thruster commands are backlogged to fake delay IRL between being commanded and the thruster actually applying a force
    if (!thrusterForceQue.empty())
    {
        // If true, it has been THRUSTER_DELAY since thrust was commanded, so it should be applied
        if (node->get_clock()->now().seconds() - thrusterForceQue[0].time > THRUSTER_DELAY)
        {
            // Set forces and torques and remove element from thruster que
            setForcesTorques(thrusterForceQue[0].thrusterForces);
            thrusterForceQue.erase(thrusterForceQue.begin());
        }
    }
}

// Stores the latest acceleration vector in the world frame, used for IMU sensor data calcs
void Robot::setAccel(const vXd &stateDot)
{
    // STATE:
    // 0 1 2 3  4  5  6  7   8   9   10  11  12       <- index
    // x y z qw qx qy qz P_x P_y P_z L_x L_y L_z      <- parameter
    // STATE DOT:
    // 0  1  2  3   4   5   6   7    8    9    10   11   12    <- index
    // x' y' z' qw' qx' qy' qz' P_x' P_y' P_z' L_x' L_y' L_z'  <- parameter
    // where ' symbol is time derivative

    // a = F/m
    // Includes gravity acceleration for IMU, robot doesn't actually experience it
    linAccel = stateDot.segment(7, 3) / getTotalMass() + v3d(0, 0, GRAVITY);
    // alpha = I^-1 * T
    quat q(state[3], state[4], state[5], state[6]);
    m3d invWorldInertia = q * invBodyInertia * q.conjugate();
    angAccel = invWorldInertia * (v3d)stateDot.segment(10, 3);
}

// Converts thruster forces into robot body forces and torques stored in class
void Robot::addToThrusterQue(thrusterForcesStamped commandedThrust)
{
    thrusterForceQue.push_back(commandedThrust);
}

// Updates the active ballast, adding or removing mass if necessary
void Robot::updateActiveBallast(const ActiveBallastStates& states)
{
    if(!ballast_enabled)
    {
        return;
    }

    rclcpp::Time now = node->get_clock()->now();
    rclcpp::Duration elapsed = now - ballast_stamp;
    if(elapsed > 500ms)
    {
        RCLCPP_WARN_SKIPFIRST_THROTTLE(node->get_logger(), *node->get_clock(), 1000, "Skipping ballast update because the last update happened over a second ago.");
        return;
    }

    // assume that the state changed on the last timestep
    // two actionable cases:
    // - exaust and water are open (sink)
    // - pressure and water are open (float)
    // all other cases do nothing
    
    double deltaMass = 0;
    if(states.exaustState && states.waterState)
    {
        // intaking, increase mass
        deltaMass = WATER_DENSITY * ballast_in_flow_rate * elapsed.seconds();
    } else if(states.pressureState && states.waterState)
    {
        // expelling, decrease mass
        deltaMass = -1 * WATER_DENSITY * ballast_out_flow_rate * elapsed.seconds();
    }

    ballast_mass += deltaMass;
    if(ballast_mass < 0)
    {
        ballast_mass = 0;
    }

    if(ballast_mass > ballast_max_mass)
    {
        ballast_mass = ballast_max_mass;
    }

    ballast_stamp = now;
}

void Robot::setActiveBallastState(const ActiveBallastStates& states)
{
    ballast_states = states;
}

// Converts thruster forces into robot body forces and torques stored in class
void Robot::setForcesTorques(vXd thrusterForces)
{
    // Caps thruster forces to their limits
    thrusterForces = thrusterForces.cwiseMin(maxThrust).cwiseMax(-maxThrust);

    // Converts thruster forces to body forces
    vXd forcesTorques = thrusterMatrix.transpose() * thrusterForces;
    forces = forcesTorques.segment(0, 3);
    torques = forcesTorques.segment(3, 3);
}

//================================//
//       GETTER FUNCTIONS         //
//================================//
geometry_msgs::msg::Transform Robot::getLCameraTransform()
{
    if (!hasLCameraTransform)
    {
        string fromFrameRel = name + "/zed_left_camera_optical_frame";
        string toFrameRel = name + "/base_link";
        lCameraTransform.translation = safeTransform(toFrameRel, fromFrameRel, hasLCameraTransform).translation;
    }
    return lCameraTransform;
}
bool Robot::mapAvailable()
{
    if (!hasMapFrame)
    {
        string toFrameRel = "map";
        string fromFrameRel = "world";
        safeTransform(toFrameRel, fromFrameRel, hasMapFrame);
    }
    return hasMapFrame;
}
bool Robot::dvlTransformAvailable()
{
    if (!hasDVLTransform)
    {
        string toFrameRel = name + "/dvl_link";
        string fromFrameRel = name + "/base_link";
        safeTransform(toFrameRel, fromFrameRel, hasDVLTransform);
    }
    return hasDVLTransform;
}
bool Robot::imuTransformAvailable()
{
    if (!hasIMUTransform)
    {
        string toFrameRel = name + "/imu_link";
        string fromFrameRel = name + "/base_link";
        safeTransform(toFrameRel, fromFrameRel, hasIMUTransform);
    }
    return hasIMUTransform;
}
bool Robot::acousticsTransformAvailable()
{
    if (!hasAcousticsTransform)
    {
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
double Robot::getMass()
{
    return mass;
}
bool Robot::getBallastEnabled()
{
    return ballast_enabled;
}
double Robot::getBallastMass()
{
    return (ballast_enabled ? ballast_mass : 0);
}
ActiveBallastStates Robot::getBallastState()
{
    return ballast_states;
}
double Robot::getTotalMass()
{
    return mass + getBallastMass();
}
vXd Robot::getState()
{
    return state;
}
v3d Robot::getLatestAngAccel()
{
    return angAccel;
}
v3d Robot::getLatestLinAccel()
{
    return linAccel;
}
m3d Robot::getInvInertia()
{
    return invBodyInertia;
}
double Robot::getIMUDrift()
{
    return imu_yawDrift;
}
v3d Robot::getThrusterForces()
{
    return forces;
}
v3d Robot::getThrusterTorques()
{
    return torques;
}
v3d Robot::getBaseLinkOffset()
{
    return r_baseLink;
}
string Robot::getName()
{
    return name;
}
v3d Robot::getNetBouyantForce(const double &depth)
{
    return v3d(0, 0, -1 * getTotalMass() * GRAVITY) + getScaleFactor(depth) * bouyancyVector;
}
v3d Robot::getDepthOffset()
{
    return r_depth;
}
double Robot::getDepthSigma()
{
    return depth_sigma;
}
bool Robot::getDepthEnabled()
{
    return depth_enabled;
}
double Robot::getDepthRate()
{
    return depth_rate;
}
v3d Robot::getIMUOffset()
{
    return r_imu;
}
v3d Robot::getIMUSigma()
{
    return v3d(imu_sigmaAccel, imu_sigmaOmega, imu_sigmaAngle);
}
bool Robot::getIMUEnabled()
{
    return imu_enabled;
}
double Robot::getIMURate()
{
    return imu_rate;
}
quat Robot::getIMUQuat()
{
    return q_imu;
}
v3d Robot::getDVLOffset()
{
    return r_dvl;
}
double Robot::getDVLSigma()
{
    return dvl_sigma;
}
bool Robot::getDVLEnabled()
{
    return dvl_enabled;
}
double Robot::getDVLRate()
{
    return dvl_rate;
}
quat Robot::getDVLQuat()
{
    return q_dvl;
}
int Robot::getThrusterCount()
{
    return thrusterCount;
}
bool Robot::getAcousticsEnabled()
{
    return acousticsEnabled;
}
double Robot::getAcousticsPingTime()
{
    // calculate the time between the port and starboard acoustics pods recieving pulses
    // port_time - starboard_time

    // get the pod locations
    string portFrame = name + "/acoustics_port_link";
    string starboardFrame = name + "/acoustics_starboard_link";
    string worldFrame = name + "/base_inertia";

    bool success = false;
    geometry_msgs::msg::Vector3 portTranslation = safeTransform(worldFrame, portFrame, success).translation;
    geometry_msgs::msg::Vector3 starboardTranslation = safeTransform(worldFrame, starboardFrame, success).translation;

    v3d portTranslation_v3d(portTranslation.x, portTranslation.y, portTranslation.z);
    v3d starboardTranslation_v3d(starboardTranslation.x, starboardTranslation.y, starboardTranslation.z);
    quat worldRotation(state[3], state[4], state[5], state[6]);
    v3d worldTranslation_v3d(state[0], state[1], state[2]);

    //transform these info world frame
    portTranslation_v3d = (worldRotation * portTranslation_v3d) + worldTranslation_v3d;
    starboardTranslation_v3d = (worldRotation * starboardTranslation_v3d) + worldTranslation_v3d;

    double portDistance = sqrt(pow(portTranslation_v3d[0] - fakePingerPosition[0], 2) + pow(portTranslation_v3d[1] - fakePingerPosition[1], 2) + pow(portTranslation_v3d[2] - fakePingerPosition[2], 2));
    double starboardDistance = sqrt(pow(starboardTranslation_v3d[0] - fakePingerPosition[0], 2) + pow(starboardTranslation_v3d[1] - fakePingerPosition[1], 2) + pow(starboardTranslation_v3d[2] - fakePingerPosition[2], 2));

    return (portDistance - starboardDistance) / this->speedOfSound;
}

//================================//
//       UTILITY FUNCTIONS        //
//================================//

geometry_msgs::msg::Transform Robot::safeTransform(string toFrame, string fromFrame, bool &transformFlag)
{
    //NOTE: THIS IS A TRANSFORM BETWEEN EKF FRAMES AND IS NOT A PERFECT PHYSICAL TRANSFORM
    //FOR PHYSICS BASED TRANSFORMATIONS, USE STATE


    try
    {
        geometry_msgs::msg::TransformStamped t = tf_buffer->lookupTransform(
            toFrame, fromFrame,
            tf2::TimePointZero);
        transformFlag = true;
        return t.transform;
    }
    catch (const tf2::TransformException &ex)
    {
        RCLCPP_ERROR_SKIPFIRST_THROTTLE(node->get_logger(),
                                        *node->get_clock(),
                                        2000,
                                        "Could not transform %s to %s: %s",
                                        toFrame.c_str(), fromFrame.c_str(), ex.what());
        return geometry_msgs::msg::Transform();
    }
}
quat Robot::rpy2quat(double roll, double pitch, double yaw)
{
    tf2::Quaternion tfQuat;
    tfQuat.setRPY(roll, pitch, yaw);
    return quat(tfQuat.w(), tfQuat.x(), tfQuat.y(), tfQuat.z()).normalized();
}
v3d Robot::std2v3d(std::vector<double> stdVect)
{
    return v3d(stdVect[0], stdVect[1], stdVect[2]);
}
