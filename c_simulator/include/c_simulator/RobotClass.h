//=======================//
//       INCLUDES        //
//=======================//

#pragma once
#include <cmath>
#include <string>
#include <bits/stdc++.h>
#include <yaml-cpp/yaml.h>
#include <tf2_ros/buffer.h>
#include <rclcpp/rclcpp.hpp>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>
#include "c_simulator/settings.h"
#include <std_msgs/msg/string.hpp>
#include "c_simulator/Claw_Object.h"
#include <tf2_ros/transform_listener.h>

using std::string, std::cout, std::endl;
typedef Eigen::Vector3d v3d;
typedef Eigen::Vector4d v4d;
typedef Eigen::VectorXd vXd;
typedef Eigen::Matrix3d m3d;
typedef Eigen::MatrixXd mXd;
typedef Eigen::Quaterniond quat;

struct thrusterForcesStamped
{
    double time;
    vXd thrusterForces;
    thrusterForcesStamped(vXd thrusterForces_, double time_);
};


struct ActiveBallastStates
{
    bool 
        exhaustState,
        pressureState,
        waterState;
};


template<typename T>
T getYamlNodeAs(const YAML::Node& n, const std::vector<std::string>& keywords)
{
    if(keywords.empty())
    {
        throw std::runtime_error("getYamlNodeAs() requires at least one keyword.");
    }

    YAML::Node node = YAML::Clone(n);
    
    try
    {
        for(std::string s : keywords)
        {
            node = node[s];
        }

        return node.as<T>();
    } catch(YAML::Exception& e)
    {
        std::string msg = "Failed to parse value at tag " + keywords[0];
        for(size_t i = 1; i < keywords.size(); i++)
        {
            msg +=  " -> " + keywords[i];
        }
        
        msg += ": " + std::string(e.what());
        throw std::runtime_error(msg);
    }
}

class Robot
{
public:
    Robot();
    Robot(std::string name_);

    //========================//
    //        GETTERS         //
    //========================//
    vXd getState();
    double getMass();
    bool getBallastEnabled();
    double getBallastMass();
    ActiveBallastStates getBallastState();
    double getTotalMass();
    bool mapAvailable();
    m3d getInvInertia();
    std::string getName();
    int getThrusterCount();
    mXd getThrusterMatrix();
    v3d getThrusterForces();
    v3d getLatestLinAccel();
    v3d getLatestAngAccel();
    v3d getBaseLinkOffset();
    v3d getThrusterTorques();
    v3d getClawObjectForces();
    v3d getClawObjectTorques();
    v3d getEnvironmentForces();
    bool acousticsTransformAvailable();
    bool clawTransformAvailable();
    v3d getNetBouyantForce(const double &depth);
    geometry_msgs::msg::Transform getLCameraTransform();
    
    // ACOUSTICS ACCESSORS
    bool getAcousticsEnabled();
    double getAcousticsPingTime();
    
    // DEPTH ACCESSORS
    bool getDepthEnabled();
    double getDepthRate();
    v3d getDepthOffset();
    double getDepthSigma();
    
    // DVL ACCESSORS
    bool getDVLEnabled();
    double getDVLRate();
    v3d getDVLOffset();
    quat getDVLQuat();
    double getDVLSigma();
    bool dvlTransformAvailable();

    // IMU ACCESSORS
    bool getIMUEnabled();
    double getIMURate();
    v3d getIMUOffset();
    quat getIMUQuat();
    v3d getIMUSigma();
    double getIMUDrift();
    bool imuTransformAvailable();

    //========================//
    //        SETTERS         //
    //========================//
    void setState(vXd state_);
    void setAccel(const vXd &stateDot);
    bool loadParams(rclcpp::Node::SharedPtr node);
    void addToThrusterQue(thrusterForcesStamped commandedThrust);
    void setLoadedClawObject(std_msgs::msg::String object_name_msg);
    void updateActiveBallast(const ActiveBallastStates& states);
    void setActiveBallastState(const ActiveBallastStates& states);

    //========================//
    //        MATHERS         //
    //========================//
    v3d calcBouyantTorque(const quat &q, const double &depth);
    v3d calcDragForces(const v3d &linVel, const double &depth);
    v3d calcDragTorques(const v3d &linVel, const v3d &angVel, const double &depth);

    v3d claw_object_forces;                             // the forces resulting from the claw object


private:
//========================//
//       VARIABLES        //
//========================//
    std::string name;      // Robot's name

    geometry_msgs::msg::Transform lCameraTransform;
    bool hasLCameraTransform;
    bool hasIMUTransform;
    bool hasDVLTransform;
    bool hasAcousticsTransform;
    bool hasClawTransform;
    bool hasMapFrame;

    vXd state;
    rclcpp::Node::SharedPtr node;
    std::unique_ptr<tf2_ros::Buffer> tf_buffer;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener{nullptr};

    v3d r_baseLink;        // Base link cordinate system relative to COM
    v3d r_cob;             // Center of bouyancy relative to COM
    v3d r_cod;             // Center of drag relative to COM
    v3d environment_force; // Force from the environment

    //DEPTH SENSOR
    bool depth_enabled;
    int depth_rate;        // Rate in Hz that the depth sensor sends data
    v3d r_depth;           // Depth sensor location relative to COM
    double depth_sigma;    // Depth sensor standard deviation
    
    //DVL SENSOR
    bool dvl_enabled;
    int dvl_rate;          // Rate in Hz that DVL sends sensor data
    v3d r_dvl;             // DVL location relative to COM
    quat q_dvl;            // Quaternion to DVL's frame
    double dvl_sigma;      // DVL sensor standard deviation
    
    //IMU SENSOR
    bool imu_enabled;
    int imu_rate;          // Rate in Hz that IMU sends sensor data
    v3d r_imu;             // IMU location relative to COM
    quat q_imu;            // Quaternion to IMU's frame
    double imu_sigmaAccel; // IMU sensor standard deviation
    double imu_sigmaOmega; // IMU sensor standard deviation
    double imu_sigmaAngle; // IMU sensor standard deviation
    double imu_yawDrift;   // IMU yaw drift in deg/min

    // active ballast info
    bool ballast_enabled;
    double ballast_mass;                // Added mass, for cases like active ballasts, kg. From 0 to ballastMaxMass
    double ballast_max_mass;            // Maximum mass increase from active ballast, computed from volume. stored here (instead of volume) for efficiency. kg
    double ballast_in_flow_rate;        // Volume of water (m^3) entering ballast per second while intaking water
    double ballast_out_flow_rate;       // Volume of water (m^3) exiting ballast per second while expelling water
    ActiveBallastStates ballast_states;
    rclcpp::Time ballast_stamp;         // The last time the active ballast mass was updated

    std::vector<thrusterForcesStamped> thrusterForceQue;
    double mass;        // Robot mass, kg

    v3d forces;         // Thruster body forces
    v3d torques;        // Thruster body torques
    v3d weightVector;   // [0,0,-m*g] in world frame
    v3d bouyancyVector; // [0,0,+rho*g*V] in world frame
    v3d linAccel;       // Latest linear acceleration on robot (for faking sensor data)
    v3d angAccel;       // Latest angular acceleration on robot (for faking sensor data)

    int thrusterCount;            // Number of thrusters
    double maxThrust;             // Limit on thruster force, N
    std::vector<double> dragCoef; // Drag coeffecients for each axis
    m3d invBodyInertia;           // Inverse inertia tensor in body frame
    mXd thrusterMatrix;           // 6xN matrix translating N thruster forces into body forces and torques

    bool acousticsEnabled;
    float speedOfSound;           // the speed of sound in water
    v3d fakePingerPosition;       // the position of the fake pinger

    std::string loaded_object;                            // the object currently loaded into the claw, none = none
    std::map<std::string, Claw_Object> claw_objects; // the possible objects to put in the claw
    v3d origin_to_claw_translation;                     // the translation between from the origin to the claw
    v3d claw_object_torques;                             // the torque resulting from the claw object

    //========================//
    //       FUNCTIONS        //
    //========================//

    void storeConfigData(const YAML::Node& vehicle_config, const YAML::Node& simulation_config);
    v3d std2v3d(std::vector<double> stdVect);
    void setForcesTorques(vXd thrusterForces);
    double getScaleFactor(const double &depth);
    quat rpy2quat(double roll, double pitch, double yaw);
    geometry_msgs::msg::Transform safeTransform(std::string toFrame, std::string fromFrame, bool &transformFlag);
};