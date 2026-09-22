//=======================//
//       INCLUDES        //
//=======================//

#pragma once
#include "c_simulator/MarineDynamics.h"
#include "c_simulator/ThrusterDynamics.h"
#include "c_simulator/settings.h"
#include <cmath>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <yaml-cpp/yaml.h>

using std::string, std::cout, std::endl;
typedef Eigen::Vector3d v3d;
typedef Eigen::Vector4d v4d;
typedef Eigen::VectorXd vXd;
typedef Eigen::Matrix3d m3d;
typedef Eigen::MatrixXd mXd;
typedef Eigen::Quaterniond quat;

struct thrusterForcesStamped {
    double time;
    vXd thrusterForces;
    thrusterForcesStamped(vXd thrusterForces_, double time_);
};

class Robot {
  public:
    Robot();
    Robot(std::string name_);

    //========================//
    //        GETTERS         //
    //========================//
    vXd getState();
    double getMass();
    quat getDVLQuat();
    quat getIMUQuat();
    quat getGyroQuat();
    v3d getIMUSigma();
    v3d getIMUOrientationVariance();
    v3d getIMUAngularVelocityVariance();
    v3d getIMULinearAccelerationVariance();
    v3d getIMUOffset();
    v3d getDVLOffset();
    bool mapAvailable();
    double getDVLRate();
    double getIMURate();
    m3d getInvInertia();
    c_simulator::Matrix6d getInverseMass();
    v3d getWaterCurrentWorld();
    double getIMUDrift();
    v3d getDepthOffset();
    double getDVLSigma();
    double getDepthRate();
    std::string getName();
    double getDepthSigma();
    int getThrusterCount();
    mXd getThrusterMatrix();
    v3d getThrusterForces();
    v3d getLatestLinAccel();
    v3d getLatestAngAccel();
    v3d getBaseLinkOffset();
    v3d getThrusterTorques();
    bool dvlTransformAvailable();
    bool imuTransformAvailable();
    bool acousticsTransformAvailable();

    geometry_msgs::msg::Transform getLCameraTransform();
    double getAcousticsPingTime();

    //========================//
    //        SETTERS         //
    //========================//
    void setState(vXd state_);
    void setAccel(const vXd &stateDot);
    void updateThrusters(double dt);
    void stopThrusters();
    void resetDynamics();
    vXd stateDerivative(const vXd &state, double stageOffset = 0) const;
    c_simulator::Vector6d propulsionWrench(const vXd &state) const;
    vXd realizedThrusters() const {
        return actuator.forces();
    }
    double simulationTime() const {
        return actuator.time();
    }
    bool loadParams(rclcpp::Node::SharedPtr node);
    void addToThrusterQue(thrusterForcesStamped commandedThrust);

    //========================//
    //        MATHERS         //
    //========================//

  private:
    //========================//
    //       VARIABLES        //
    //========================//

    geometry_msgs::msg::Transform lCameraTransform;
    bool hasLCameraTransform;
    bool hasIMUTransform;
    bool hasDVLTransform;
    bool hasAcousticsTransform;
    bool hasMapFrame;

    vXd state;
    rclcpp::Node::SharedPtr node;
    std::unique_ptr<tf2_ros::Buffer> tf_buffer;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener{nullptr};

    double depth_rate;     // Rate in Hz that the depth sensor sends data
    v3d r_baseLink;        // Base link cordinate system relative to COM
    double imu_rate;       // Rate in Hz that IMU sends sensor data
    double dvl_rate;       // Rate in Hz that DVL sends sensor data
    v3d r_depth;           // Depth sensor location relative to COM
    double depth_sigma;    // Depth sensor standard deviation
    double imu_sigmaAccel; // IMU sensor standard deviation
    double imu_sigmaOmega; // IMU sensor standard deviation
    double imu_sigmaAngle; // IMU sensor standard deviation
    v3d imu_orientationVariance;
    v3d imu_angularVelocityVariance;
    v3d imu_linearAccelerationVariance;
    double dvl_sigma;               // DVL sensor standard deviation
    v3d r_imu;                      // IMU location relative to COM
    v3d r_dvl;                      // DVL location relative to COM
    quat q_dvl;                     // Quaternion to DVL's frame
    quat q_imu;                     // Quaternion to IMU's frame
    quat q_gyro = quat::Identity(); // FOG mounting orientation
    double imu_yawDrift;            // IMU yaw drift in deg/min
    std::string name;               // Robot's name

    double mass;  // Robot mass, kg
    v3d forces;   // Thruster body forces
    v3d torques;  // Thruster body torques
    v3d linAccel; // Latest linear acceleration on robot (for faking sensor data)
    v3d angAccel; // Latest angular acceleration on robot (for faking sensor data)

    int thrusterCount;           // Number of thrusters
    double maxThrust;            // Limit on thruster force, N
    double thrusterDelay;        // Command transport delay, s
    double thrusterRiseTime;     // First-order rise time constant, s
    double thrusterFallTime;     // First-order fall time constant, s
    double thrusterSlewRate;     // Maximum force slew rate, N/s (<= 0 disables)
    double thrusterDeadband;     // Requested forces below this magnitude are zeroed, N
    double thrusterForwardScale; // Positive-force calibration scale
    double thrusterReverseScale; // Negative-force calibration scale
    m3d invBodyInertia;          // Inverse inertia tensor in body frame
    c_simulator::MarineDynamics marineDynamics;
    c_simulator::ThrusterDynamics actuator;
    std::vector<v3d> thrusterPositions, thrusterDirections;
    v3d currentAmplitude = v3d::Zero();
    double currentFrequency = 0, propellerRadius = .05;
    v3d waterCurrentWorld;
    mXd thrusterMatrix; // 6xN matrix translating N thruster forces into body
                        // forces and torques

    float speedOfSound;     // the speed of sound in water
    v3d fakePingerPosition; // the position of the fake pinger

    //========================//
    //       FUNCTIONS        //
    //========================//
    void storeConfigData(YAML::Node vehicle_config, YAML::Node simulation_config);
    v3d std2v3d(std::vector<double> stdVect);
    void setForcesTorques(const vXd &thrusterForces);

    quat rpy2quat(double roll, double pitch, double yaw);
    geometry_msgs::msg::Transform safeTransform(std::string toFrame, std::string fromFrame, bool &transformFlag);
};
