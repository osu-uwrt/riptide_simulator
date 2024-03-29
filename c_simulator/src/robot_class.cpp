#include "c_simulator/RobotClass.h"

//================================//
//     OBJECT INITIALIZATION      //
//================================//

Robot::Robot()
{
    state.resize(13);

    // INITIAL CONDITION
    state << 0, 0, -1, // Position, X Y Z
        1, 0, 0, 0,    // Orientation, W X Y Z
        0, 0, 0,       // Linear momentum, X Y Z
        0, 0, 0;       // Angular momentum, X Y Z
    imuTransform = false;
    dvlTransform = false;
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

    // Retrieve the path to the YAML config file from the parameters
    std::string config_file;
    node->declare_parameter("vehicle_config", "");
    if (node->get_parameter("vehicle_config", config_file))
    {
        try
        {
            RCLCPP_INFO(node->get_logger(), "Opening config file: %s", config_file.c_str());
            // Loading YAML file for parsing
            YAML::Node config = YAML::LoadFile(config_file);
            Robot::storeConfigData(config);

            // Loading params successeeded
            return true;
        }
        catch (const YAML::Exception &e)
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
void Robot::storeConfigData(YAML::Node config)
{
    // Getting mass and bouyancy informtion
    mass = config["mass"].as<double>();
    invBodyInertia = std2v3d(config["inertia"].as<std::vector<double>>()).asDiagonal().inverse();
    weightVector = {0, 0, -mass * GRAVITY};
    std::vector<double> feedForward = config["controller"]["feed_forward"]["base_wrench"].as<std::vector<double>>();
    bouyancyVector = v3d(0, 0, -feedForward[2]) - weightVector;
    v3d r_com = std2v3d(config["com"].as<std::vector<double>>());
    // Calculate COB from feed forward: r = T/F
    r_cob = v3d(feedForward[4] / bouyancyVector.norm(), -feedForward[3] / bouyancyVector.norm(), 0.005);
    r_cod = r_cob;

    //  Getting base link position relative to center of mass
    r_baseLink = std2v3d(config["base_link"].as<std::vector<double>>());
    // Getting IMU information
    r_imu = std2v3d(config["imu"]["pose"].as<std::vector<double>>()) - r_com;
    imu_rate = 1.0 / config["imu"]["rate"].as<double>();
    imu_sigmaAccel = config["imu"]["sigma_accel"].as<double>();
    imu_sigmaOmega = config["imu"]["sigma_omega"].as<double>() * M_PI / 180;
    imu_sigmaAngle = config["imu"]["sigma_angle"].as<double>() * M_PI / 180;
    // Getting depth sensor information
    r_depth = std2v3d(config["depth"]["pose"].as<std::vector<double>>()) - r_com;
    depth_rate = 1.0 / config["depth"]["rate"].as<double>();
    depth_sigma = config["depth"]["sigma"].as<double>();
    // Getting dvl sensor information
    r_dvl = std2v3d(config["dvl"]["pose"].as<std::vector<double>>()) - r_com;
    dvl_rate = 1.0 / config["dvl"]["rate"].as<double>();
    dvl_sigma = config["dvl"]["sigma"].as<double>();

    // Drag information
    std::vector<double> dragCoefStd = config["controller"]["SMC"]["damping"].as<std::vector<double>>();
    dragCoef = Eigen::Map<vXd>(dragCoefStd.data(), dragCoefStd.size());

    // Creating thruster forces -> body forces & torques matrix by looping through each thruster
    YAML::Node thrusters = config["thrusters"];
    maxThrust = config["thruster"]["max_force"].as<double>();
    thrusterCount = thrusters.size();
    thrusterMatrix.resize(thrusterCount, 6);
    // Thruster info
    int row = 0;
    for (auto thruster : thrusters)
    {
        // Get thruster position and orientation
        std::vector<double> pose = thruster["pose"].as<std::vector<double>>();
        v3d thrusterPos = std2v3d(pose);
        // I am not really sure why I needed to put the negative in front of pose, or the transpose
        // but it matches the matrix the thruster solver gives so I'm not going to question it
        m3d rotMat = Eigen::AngleAxisd(-pose[3], v3d::UnitX()).toRotationMatrix() *
                     Eigen::AngleAxisd(-pose[4], v3d::UnitY()).toRotationMatrix() *
                     Eigen::AngleAxisd(-pose[5], v3d::UnitZ()).toRotationMatrix();

        // Body force caused by unit thrust vector:
        v3d bodyForce = rotMat.transpose() * v3d(1, 0, 0);
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
    // Adding drag torque caused by linear drag force caused by offset center of drag
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
    quat q(state[3], state[4], state[5], state[6]);
    // a = F/m
    linAccel = stateDot.segment(7, 3) / mass;
    // alpha = I^-1 * T
    m3d invWorldInertia = q * invBodyInertia * q.conjugate();
    angAccel = invWorldInertia * (v3d)stateDot.segment(10, 3);
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

// Tries to obtain DVL tf2 transform, returns whether it was successful
// Assumes that loadParams() has already been called
bool Robot::obtainDVLTransform()
{
    try
    {
        geometry_msgs::msg::TransformStamped dvlTransformation = tf_buffer->lookupTransform("talos/dvl_link", "talos/base_link", tf2::TimePointZero);
        q_dvl.w() = dvlTransformation.transform.rotation.w;
        q_dvl.x() = dvlTransformation.transform.rotation.x;
        q_dvl.y() = dvlTransformation.transform.rotation.y;
        q_dvl.z() = dvlTransformation.transform.rotation.z;
        dvlTransform = true;
        return true;
    }
    catch (const tf2::TransformException &ex)
    {
        RCLCPP_ERROR_SKIPFIRST_THROTTLE(node->get_logger(), *node->get_clock(), 500, "Could not transform to DVL frame: %s", ex.what());
        return false;
    }
    return false;
}

// Tries to obtain IMU tf2 transform, returns whether it was successful
// Assumes that loadParams() has already been called
bool Robot::obtainIMUTransform()
{
    try
    {
        geometry_msgs::msg::TransformStamped imuTransformation = tf_buffer->lookupTransform("talos/imu_link", "talos/base_link", tf2::TimePointZero);
        q_imu.w() = imuTransformation.transform.rotation.w;
        q_imu.x() = imuTransformation.transform.rotation.x;
        q_imu.y() = imuTransformation.transform.rotation.y;
        q_imu.z() = imuTransformation.transform.rotation.z;
        imuTransform = true;
        return true;
    }
    catch (const tf2::TransformException &ex)
    {
        RCLCPP_ERROR_SKIPFIRST_THROTTLE(node->get_logger(), *node->get_clock(), 500, "Could not transform to IMU frame: %s", ex.what());
        return false;
    }
    return false;
}
//================================//
//       GETTER FUNCTIONS         //
//================================//
double Robot::getMass()
{
    return mass;
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
v3d Robot::getNetBouyantForce(const double &depth)
{
    return weightVector + getScaleFactor(depth) * bouyancyVector;
}
v3d Robot::getDepthOffset()
{
    return r_depth;
}
double Robot::getDepthSigma()
{
    return depth_sigma;
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
double Robot::getIMURate()
{
    return imu_rate;
}
quat Robot::getIMUQuat()
{
    return q_imu;
}
bool Robot::hasIMUTransform()
{
    return imuTransform;
}
v3d Robot::getDVLOffset()
{
    return r_dvl;
}
double Robot::getDVLSigma()
{
    return dvl_sigma;
}
double Robot::getDVLRate()
{
    return dvl_rate;
}
quat Robot::getDVLQuat()
{
    return q_dvl;
}
bool Robot::hasDVLTransform()
{
    return dvlTransform;
}
int Robot::getThrusterCount()
{
    return thrusterCount;
}

//================================//
//       UTILITY FUNCTIONS        //
//================================//

v3d Robot::std2v3d(std::vector<double> stdVect)
{
    return v3d(stdVect[0], stdVect[1], stdVect[2]);
}