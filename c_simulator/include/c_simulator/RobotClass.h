//===================================//
//       INCLUDES/DEFINITIONS        //
//===================================//

#pragma once
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>
#include <tf2_ros/buffer.h>
#include "tf2_ros/transform_listener.h"
#include "rclcpp/rclcpp.hpp"
#include "yaml-cpp/yaml.h"
#include <cmath>

using std::cout, std::endl;

typedef Eigen::Vector3d v3d;
typedef Eigen::VectorXd vXd;
typedef Eigen::Matrix3d m3d;
typedef Eigen::MatrixXd mXd;
typedef Eigen::Quaterniond quat;

//========================//
//       CONSTANTS        //
//========================//

#define GRAVITY 9.80665    // m/s^2
#define VEHICLE_HEIGHT 0.3 // m         Used for bouyancy calcs when partially submerged

class Robot
{
public:
    Robot();

    //========================//
    //        GETTERS         //
    //========================//
    double getMass();
    vXd getState();
    m3d getInvInertia();
    mXd getThrusterMatrix();
    v3d getThrusterForces();
    v3d getThrusterTorques();
    v3d getLatestLinAccel();
    v3d getLatestAngAccel();
    v3d getBaseLinkOffset();
    v3d getNetBouyantForce(const double &depth);
    v3d getDepthOffset();
    double getDepthSigma();
    double getDepthRate();
    v3d getDVLOffset();
    quat getDVLQuat();
    double getDVLSigma();
    double getDVLRate();
    bool hasDVLTransform();
    bool obtainDVLTransform();
    v3d getIMUOffset();
    quat getIMUQuat();
    bool hasIMUTransform();
    bool obtainIMUTransform();
    v3d getIMUSigma();
    double getIMURate();

    //========================//
    //        SETTERS         //
    //========================//
    void setState(vXd state_);
    void setAccel(const vXd &stateDot);
    bool loadParams(rclcpp::Node::SharedPtr node);
    void setForcesTorques(const vXd &forcesTorques);

    //========================//
    //        MATHERS         //
    //========================//
    v3d calcBouyantTorque(const quat &q, const double &depth);
    v3d calcDragForces(const v3d &linVel, const double &depth);
    v3d calcDragTorques(const v3d &linVel, const v3d &angVel, const double &depth);

private:
    //========================//
    //       VARIABLES        //
    //========================//

    // STATE:
    // 0 1 2 3  4  5  6  7   8   9   10  11  12       <- index
    // x y z qw qx qy qz P_x P_y P_z L_x L_y L_z      <- parameter
    // where P is linear momentum, and L is angular momentum, and q is quaternion
    vXd state;
    std::unique_ptr<tf2_ros::Buffer> tf_buffer;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener{nullptr};

    int depth_rate;        // Rate in Hz that the depth sensor sends data
    v3d r_baseLink;        // Base link cordinate system relative to COM
    int imu_rate;          // Rate in Hz that IMU sends sensor data
    int dvl_rate;          // Rate in Hz that DVL sends sensor data
    v3d r_depth;           // Depth sensor location relative to COM
    v3d r_cob;             // Center of bouyancy relative to COM
    double depth_sigma;    // Depth sensor standard deviation
    v3d r_cod;             // Center of drag relative to COM
    double imu_sigmaAccel; // IMU sensor standard deviation
    double imu_sigmaOmega; // IMU sensor standard deviation
    double imu_sigmaAngle; // IMU sensor standard deviation
    double dvl_sigma;      // DVL sensor standard deviation
    v3d r_imu;             // IMU location relative to COM
    v3d r_dvl;             // DVL location relative to COM
    quat q_dvl;            // Quaternion to DVL's frame
    quat q_imu;            // Quaternion to IMU's frame
    bool imuTransform;     // IMU tf2 transform
    bool dvlTransform;     // DVL tf2 transform

    double mass;        // Robot mass, kg
    v3d forces;         // Thruster body forces
    v3d torques;        // Thruster body torques
    v3d weightVector;   // [0,0,-m*g] in world frame
    v3d bouyancyVector; // [0,0,+rho*g*V] in world frame
    v3d linAccel;       // Latest linear acceleration on robot (for faking sensor data)
    v3d angAccel;       // Latest angular acceleration on robot (for faking sensor data)

    vXd dragCoef;       // Drag coeffecients for each axis
    m3d invBodyInertia; // Inverse inertia tensor in body frame
    v3d angDragCoef;    // Angular drag coeffecients for each axis
    v3d area;           // Frontal area from each direction (used for drag)
    mXd thrusterMatrix; // 6xN matrix translating N thruster forces into body forces and torques

    //========================//
    //       FUNCTIONS        //
    //========================//
    v3d std2v3d(std::vector<double> stdVect);
    double getScaleFactor(const double &depth);
    void storeConfigData(YAML::Node config, rclcpp::Node::SharedPtr node);
};