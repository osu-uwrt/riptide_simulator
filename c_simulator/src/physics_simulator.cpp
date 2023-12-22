//===============================//
//           INCLUDES            //
//===============================//
#include <chrono>
#include <memory>
#include <string>
#include <random>
#include <functional>
#include <rclcpp/rclcpp.hpp>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <std_msgs/msg/bool.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <std_msgs/msg/string.hpp>
#include "c_simulator/RobotClass.h"
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <robot_localization/srv/set_pose.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <riptide_msgs2/msg/kill_switch_report.hpp>
#include <riptide_msgs2/msg/dshot_partial_telemetry.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>

using std::placeholders::_1;
using namespace std::chrono_literals;

//===============================//
//            SETTINGS           //
//===============================//

#define STATE_PUB_TIME 0.05        // Time, in seconds, between simulator publishing state (for RViz)
#define SENSOR_NOISE_ENABLED false // Noise is added to sensor data when true

//===============================//
/*      NOTES/ASSUMPTIONS        //
//===============================//
1) Assumes thruster force curves are accuarate (if not, what the controller does IRL may not reflect response in sim)
2) There is no collision detection, if the robot hits an object it will just phase through it
3) Bouyant force of robot is approximated as a cylinder when robot is partially submereged
4) Assumes thrusters will not be running while out of water
5) Assumes center of drag is at center of bouyancy
6) Everything is in SI units (kg, m, s, N)
7) Assumes robot is on Earth :P
8) This will cook your CPU
*/

class PhysicsSimNode : public rclcpp::Node
{
public:
    //================================//
    //         SIM START UP           //
    //================================//
    PhysicsSimNode() : Node("physics_simulator")
    {
        enabled = false;

        // Create publishers and subscribers
        imuPub = this->create_publisher<sensor_msgs::msg::Imu>("vectornav/imu", 10);
        firmwareKillPub = this->create_publisher<std_msgs::msg::Bool>("state/kill", 10);
        statePub = this->create_publisher<geometry_msgs::msg::Pose>("simulator/state", 10);
        dvlPub = this->create_publisher<geometry_msgs::msg::TwistWithCovarianceStamped>("dvl_twist", 10);
        depthPub = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("depth/pose", 10);
        thrusterSub = this->create_subscription<std_msgs::msg::Float32MultiArray>("thruster_forces", 10, std::bind(&PhysicsSimNode::forceCallback, this, _1));
        softwareKillSub = this->create_subscription<riptide_msgs2::msg::KillSwitchReport>("command/software_kill", 10, std::bind(&PhysicsSimNode::killSwitchCallback, this, _1));
        thrusterTelemetryPub = this->create_publisher<riptide_msgs2::msg::DshotPartialTelemetry>("state/thrusters/telemetry", 10);

        // Create timers
        auto statePubTime = std::chrono::duration<double>((double)STATE_PUB_TIME);
        statePubTimer = this->create_wall_timer(statePubTime, std::bind(&PhysicsSimNode::publishState, this));
        this->thrusterTelemetryTimer = this->create_wall_timer(std::chrono::duration<double>((double)0.5), std::bind(&PhysicsSimNode::pubThrusterTelemetry, this));

        // Services for setting odom and simulator
        poseClient = this->create_client<robot_localization::srv::SetPose>("/talos/set_pose");
        poseService = this->create_service<robot_localization::srv::SetPose>("set_sim_pose", std::bind(&PhysicsSimNode::setSim, this, _1));
    }

    void pubThrusterTelemetry(){
        // publish a telemtry message with no disabled flags

        riptide_msgs2::msg::DshotPartialTelemetry msgLow;
        riptide_msgs2::msg::DshotPartialTelemetry msgHigh;
        msgLow.disabled_flags = 0;
        msgHigh.disabled_flags = 0;
        msgLow.esc_telemetry[0].thruster_ready = true;
        msgLow.esc_telemetry[1].thruster_ready = true;
        msgLow.esc_telemetry[2].thruster_ready = true;
        msgLow.esc_telemetry[3].thruster_ready = true;        
        msgHigh.esc_telemetry[0].thruster_ready = true;
        msgHigh.esc_telemetry[1].thruster_ready = true;
        msgHigh.esc_telemetry[2].thruster_ready = true;
        msgHigh.esc_telemetry[3].thruster_ready = true;
        msgLow.start_thruster_num = 0;
        msgHigh.start_thruster_num = 4;

        this->thrusterTelemetryPub->publish(msgLow);
        this->thrusterTelemetryPub->publish(msgHigh);
    }

    /**
     * @brief Loads parameters from the robot's yaml file into the Robot object
     * @returns wether the parameters were loaded successfully
     */
    bool init()
    {
        RCLCPP_INFO(this->get_logger(), "Getting simulator starting parameter data");
        // Tries to load robot parameter data
        bool loaded = robot.loadParams(shared_from_this());
        if (loaded)
        {
            //  Once YAML file params have been loaded, create timers to publish sensor data
            auto imuRate = std::chrono::duration<double>(robot.getIMURate());
            auto dvlRate = std::chrono::duration<double>(robot.getDVLRate());
            auto depthRate = std::chrono::duration<double>(robot.getDepthRate());
            RCLCPP_INFO(this->get_logger(), "Timers created to fake sensor data");
            imuTimer = this->create_wall_timer(imuRate, std::bind(&PhysicsSimNode::publishFakeIMUData, this));
            dvlTimer = this->create_wall_timer(dvlRate, std::bind(&PhysicsSimNode::publishFakeDVLData, this));
            depthTimer = this->create_wall_timer(depthRate, std::bind(&PhysicsSimNode::publishFakeDepthData, this));
        }
        return loaded;
    }

    //================================//
    //      PHYSICS FUNCTIONS         //
    //================================//

    /**
     * @brief Solves system of ODEs representing robot physics using fourth order Runge-Kutta numerical method.
     * This function is runs in an infinite loop to run simulator, which also spins the node
     */
    void rungeKutta4()
    {
        rclcpp::Time currentTime;
        rclcpp::Time prevTime = rclcpp::Clock(RCL_SYSTEM_TIME).now();
        while (rclcpp::ok())
        {
            // Time since last loop, sets step size of simulation
            currentTime = rclcpp::Clock(RCL_SYSTEM_TIME).now();
            double stepSize = (currentTime - prevTime).seconds();
            prevTime = currentTime;

            // For more info: https://en.wikipedia.org/wiki/Runge%E2%80%93Kutta_methods
            vXd state = robot.getState();

            // Evaluate derivative vector field at key points
            vXd K1 = calcStateDot(state);
            vXd K2 = calcStateDot(state + stepSize / 2.0 * K1);
            vXd K3 = calcStateDot(state + stepSize / 2.0 * K2);
            vXd K4 = calcStateDot(state + stepSize * K3);

            // Update the robots state based on weighted average of sampled points
            vXd stateDot = stepSize / 6.0 * (K1 + 2 * K2 + 2 * K3 + K4);
            robot.setState(state + stateDot);
            robot.setAccel(stateDot);

            // Proccess any pending callbacks or timers
            rclcpp::spin_some(shared_from_this());
        }
        RCLCPP_INFO(this->get_logger(), "Shutting down simulator");
    }

private:
    /**
     * @param state Robot's state vector where derivative is to be evaluated at
     * @return The rate of change of the robot's state vector
     */
    vXd calcStateDot(const vXd &state)
    {
        // STATE:
        // 0 1 2 3  4  5  6  7   8   9   10  11  12       <- index
        // x y z qw qx qy qz P_x P_y P_z L_x L_y L_z      <- parameter
        // where P is linear momentum, and L is angular momentum, and q is quaternion
        // STATE DOT:
        // 0  1  2  3   4   5   6   7    8    9    10   11   12    <- index
        // x' y' z' qw' qx' qy' qz' P_x' P_y' P_z' L_x' L_y' L_z'  <- parameter
        // where ' symbol is time derivative

        vXd stateDot(13);

        // Get quaternion from state
        quat q = state2quat(state);
        // Convert the inverse inertia tensor from body to world frame
        m3d invWorldInertia = q * robot.getInvInertia() * q.conjugate();
        // angular velocity = inverse inertia tensor * angular momentum
        v3d angularVel = invWorldInertia * state.segment(10, 3);
        // linear velocity = linear momentum/mass
        v3d linearVel = state.segment(7, 3) / robot.getMass();
        double depth = state[2];

        // Linear velocities
        stateDot.segment(0, 3) = linearVel;
        // Change in linear momentum dP/dt = F (Newton's second law)
        stateDot.segment(7, 3) = calcForces(q, linearVel, depth);

        // Rate of change of quaternion
        stateDot.segment(3, 4) = calcQuaternionDot(q, angularVel);
        // Change in angular momentum dL/dt = T (Newton's second law)
        stateDot.segment(10, 3) = calcTorques(q, linearVel, angularVel, depth);

        return stateDot;
    }

    /**
     * @param q Quaternion representing the robot's orientation
     * @param linVel Linear velocity of the robot in the world frame
     * @return Net force acting on the robot's COM in the world frame
     */
    v3d calcForces(const quat &q, const v3d &linVel, const double &depth)
    {
        // Linear velocity is converted to body frame before passing it to drag force function
        v3d body_forces = robot.getThrusterForces() + robot.calcDragForces(q.conjugate() * linVel, depth);
        v3d world_forces = q * body_forces + robot.getNetBouyantForce(depth);
        return world_forces;
    }

    /**
     * @param q Quaternion representing the robot's orientation
     * @param angVel Angular velocity of the robot in the world frame
     * @return The net torque acting on the robot in the world frame
     */
    v3d calcTorques(const quat &q, const v3d &linVel, const v3d &angVel, const double &depth)
    {
        // Angular and linear velocity is converted to body frame before passing it to drag torque function
        v3d body_torques = robot.getThrusterTorques() + robot.calcDragTorques(q.conjugate() * linVel, q.conjugate() * angVel, depth);
        v3d world_torques = q * body_torques + robot.calcBouyantTorque(q, depth);
        return world_torques;
    }

    /**
     * @param q Quaternion representing the robot's orientation
     * @param angVel Angular velocity of the robot in the world frame
     * @return The rate of change of the quaternion elements as a vector [w', x', y', z']
     */
    Eigen::Vector4d calcQuaternionDot(quat q, const v3d &angVel)
    {
        // For more information on what's happening here, read these gems
        // https://math.unm.edu/~vageli/papers/rrr.pdf
        // https://graphics.pixar.com/pbm2001/pdf/notesg.pdf

        quat omegaQuat(0, angVel.x(), angVel.y(), angVel.z());
        quat qDot = (omegaQuat * q);
        Eigen::Vector4d qDotVect = {qDot.w(), qDot.x(), qDot.y(), qDot.z()};
        return qDotVect / 2.0;
    }

    //================================//
    //       FAKING SENSOR DATA       //
    //================================//
    void publishFakeDepthData()
    {
        // Get depth and add noise to data
        vXd state = robot.getState();
        double depthData = state[2];
        if (SENSOR_NOISE_ENABLED)
            depthData = randomNorm(depthData, robot.getDepthSigma());

        // Send message with depth sensor info
        geometry_msgs::msg::PoseWithCovarianceStamped depthMsg;
        depthMsg.pose.pose.position.z = depthData;
        depthMsg.pose.covariance[14] = robot.getDepthSigma();
        depthMsg.header.stamp = rclcpp::Clock(RCL_ROS_TIME).now();
        depthMsg.header.frame_id = "odom";
        depthPub->publish(depthMsg);
    }
    void publishFakeDVLData()
    {
        if (robot.hasDVLTransform())
        {
            vXd state = robot.getState();
            // Get quaternion from state
            quat q = state2quat(state);
            // Convert the inverse inertia tensor from body to world frame
            m3d invWorldInertia = q * robot.getInvInertia() * q.conjugate();
            // angular velocity = inverse inertia tensor * angular momentum
            v3d angularVel = invWorldInertia * state.segment(10, 3);
            // linear velocity = linear momentum/mass
            v3d linearVel = state.segment(7, 3) / robot.getMass();

            // V_dvl = V_robot + w x r
            v3d dvlData = linearVel + angularVel.cross(q * robot.getDVLOffset());
            // Transform velocity from world frame -> robot frame -s> DVL frame
            dvlData = robot.getDVLQuat() * (q.conjugate() * dvlData);

            // Add nonise to sensor data if enabled, otherwise don't
            if (SENSOR_NOISE_ENABLED)
                dvlData = randomNorm(dvlData, robot.getDVLSigma());
            // Send message with DVL sensor info
            geometry_msgs::msg::TwistWithCovarianceStamped dvlMsg;
            dvlMsg.twist.twist.linear.x = dvlData[0];
            dvlMsg.twist.twist.linear.y = dvlData[1];
            dvlMsg.twist.twist.linear.z = dvlData[2];
            // Add covariance to message
            double dvlVariance = robot.getDVLSigma() * robot.getDVLSigma();
            dvlMsg.twist.covariance[21] = dvlVariance;
            dvlMsg.twist.covariance[28] = dvlVariance;
            dvlMsg.twist.covariance[35] = dvlVariance;
            // dvlMsg.twist.covariance[14] = robot.getDVLSigma();
            dvlMsg.header.stamp = rclcpp::Clock(RCL_ROS_TIME).now();
            dvlMsg.header.frame_id = "talos/dvl_link";
            dvlPub->publish(dvlMsg);
        }
        // DVL transform not obtained yet, get try to get it
        else
        {
            if (robot.obtainDVLTransform())
            {
                // Transform successfully obtained, rerun function to publish data
                publishFakeDVLData();
            }
            else
            {
                // Failed to get transform, report error in terminal
                RCLCPP_ERROR(this->get_logger(), "Could not publish fake DVL data, transform not obtained yet");
            }
        }
    }
    void publishFakeIMUData()
    {
        if (robot.hasIMUTransform())
        {
            vXd state = robot.getState();
            // Get quaternion from state
            quat q = state2quat(state);
            // Convert the inverse inertia tensor from body to world frame
            m3d invWorldInertia = q * robot.getInvInertia() * q.conjugate();
            // angular velocity = inverse inertia tensor * angular momentum
            v3d angularVel = invWorldInertia * state.segment(10, 3);

            // a_imu = a_body + alpha x r + w x w x r
            // All vectors are in world frame
            // No Coriolis force is needed since the IMU is not moving relative to the robot
            v3d imuAccel = robot.getLatestLinAccel() + robot.getLatestAngAccel().cross(q * robot.getIMUOffset()) + angularVel.cross(angularVel.cross(q * robot.getIMUOffset()));

            // Transform vectors from world -> robot -> IMU
            angularVel = robot.getIMUQuat() * (q.conjugate() * angularVel);
            imuAccel = robot.getIMUQuat() * (q.conjugate() * imuAccel);
            // Transform orientation from robot -> IMU
            q = q * robot.getIMUQuat();

            // Add nonise to sensor data if enabled, otherwise don't
            v3d imu_sigma = robot.getIMUSigma();
            if (SENSOR_NOISE_ENABLED)
            {
                imuAccel = randomNorm(imuAccel, imu_sigma[0]);
                angularVel = randomNorm(angularVel, imu_sigma[1]);
                // Add noise to orientation
                v3d randomAxis;
                randomAxis.setRandom().normalize();
                Eigen::AngleAxisd randomAngle(imu_sigma[2], randomAxis);
                q = randomAngle * q;
            }
            // Send message with IMU sensor info
            sensor_msgs::msg::Imu imuMsg;
            // Setting linear acceleration message info
            imuMsg.linear_acceleration.x = imuAccel[0];
            imuMsg.linear_acceleration.y = imuAccel[1];
            imuMsg.linear_acceleration.z = imuAccel[2];
            // Setting angular velocity message info
            imuMsg.angular_velocity.x = angularVel[0];
            imuMsg.angular_velocity.y = angularVel[1];
            imuMsg.angular_velocity.z = angularVel[2];
            // Setting orientation message info
            imuMsg.orientation.w = q.w();
            imuMsg.orientation.x = q.x();
            imuMsg.orientation.y = q.y();
            imuMsg.orientation.z = q.z();
            // Add covariance to message
            double orientationVariance = imu_sigma[2] * imu_sigma[2];
            imuMsg.orientation_covariance[0] = orientationVariance;
            imuMsg.orientation_covariance[4] = orientationVariance;
            imuMsg.orientation_covariance[8] = orientationVariance;
            double angVelVariance = imu_sigma[1] * imu_sigma[1];
            imuMsg.angular_velocity_covariance[0] = angVelVariance;
            imuMsg.angular_velocity_covariance[4] = angVelVariance;
            imuMsg.angular_velocity_covariance[8] = angVelVariance;
            double accelVariance = imu_sigma[0] * imu_sigma[0];
            imuMsg.linear_acceleration_covariance[0] = accelVariance;
            imuMsg.linear_acceleration_covariance[4] = accelVariance;
            imuMsg.linear_acceleration_covariance[8] = accelVariance;
            // Publish message
            imuMsg.header.stamp = rclcpp::Clock(RCL_ROS_TIME).now();
            imuMsg.header.frame_id = "talos/imu_link";
            imuPub->publish(imuMsg);
        }
        // IMU transform has not been obtained yet, try to get it
        else
        {
            if (robot.obtainIMUTransform())
            {
                // Transform successfully obtained, rerun function to publish data
                publishFakeIMUData();
            }
            else
            {
                // Failed to get transform, report error in terminal
                RCLCPP_ERROR(this->get_logger(), "Could not publish fake IMU data, transform not obtained yet");
            }
        }
    }

    //================================//
    //         CALLBACK STUFF         //
    //================================//

    // Once new thruster forces are available, update robot's forces/torques
    void forceCallback(const std_msgs::msg::Float32MultiArray &thrusterForces)
    {
        // Discard any forces if robot is disabled
        if (enabled)
            robot.setForcesTorques(convert2eigen(thrusterForces.data));
        else
            robot.setForcesTorques(vXd::Zero(robot.getThrusterCount()));
    }

    // Mirros software kill to firmware kill. This is done to make the enable/disable button in RViz work
    void killSwitchCallback(const riptide_msgs2::msg::KillSwitchReport &softwareKillMsg)
    {
        if (softwareKillMsg.kill_switch_id == 1)
        {
            std_msgs::msg::Bool message;
            message.data = softwareKillMsg.switch_asserting_kill;
            enabled = !message.data;
            if (!enabled)
                robot.setForcesTorques(vXd::Zero(robot.getThrusterCount()));
            firmwareKillPub->publish(message);
        }
    }

    // Function called by "set_sim_pose" service. Updates simulator and EKF to requested position
    void setSim(const std::shared_ptr<robot_localization::srv::SetPose::Request> poseRequest)
    {
        // Create request to EKF, wait for availablility
        auto request = std::make_shared<robot_localization::srv::SetPose::Request>();
        geometry_msgs::msg::Pose desiredPose = poseRequest->pose.pose.pose;
        request->pose.pose.pose = desiredPose;
        request->pose.pose.covariance = {0.0};
        request->pose.header.stamp = rclcpp::Clock(RCL_ROS_TIME).now();
        request->pose.header.frame_id = "odom";
        while (!poseClient->wait_for_service(1s))
        {
            if (!rclcpp::ok())
            {
                RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the set simulator pose service. Exiting.");
                break;
            }
        }
        // Send request
        auto result = poseClient->async_send_request(request);

        // Set state in physics simulator as well
        vXd state(13);
        // Setting position of state to desired
        state[0] = desiredPose.position.x;
        state[1] = desiredPose.position.y;
        state[2] = desiredPose.position.z;
        // Setting orientation of state to desired
        state[3] = desiredPose.orientation.w;
        state[4] = desiredPose.orientation.x;
        state[5] = desiredPose.orientation.y;
        state[6] = desiredPose.orientation.z;
        robot.setState(state);
    }

    // Called on a timer, publishes robot's true state for RViz
    void publishState()
    {
        auto message = geometry_msgs::msg::Pose();
        vXd state = robot.getState();
        quat q = state2quat(state);
        v3d baseLinkOffset = q * robot.getBaseLinkOffset();

        // Setting position
        // Sim uses COM for robot position, need to add offset for position relative to base link
        message.position.x = state[0] - baseLinkOffset.x();
        message.position.y = state[1] - baseLinkOffset.y();
        message.position.z = state[2] - baseLinkOffset.z();
        // Setting orientation
        message.orientation.w = q.w();
        message.orientation.x = q.x();
        message.orientation.y = q.y();
        message.orientation.z = q.z();
        // Publishing state
        statePub->publish(message);
    }

    //================================//
    //       UTILITY FUNCTIONS        //
    //================================//

    // Returns a normalied quaternion from state vector
    quat state2quat(const vXd &state)
    {
        quat q;
        q.w() = state[3];
        q.vec() = state.segment(4, 3);
        return q.normalized();
    }
    // Returns a vector with added normal distribution noise added to each element
    v3d randomNorm(const v3d &vector, const double &stdv)
    {
        v3d noisyVector;
        noisyVector[0] = randomNorm(vector[0], stdv);
        noisyVector[1] = randomNorm(vector[1], stdv);
        noisyVector[2] = randomNorm(vector[2], stdv);
        return noisyVector;
    }
    // Returns a number with added normal distribution noise
    double randomNorm(const double &mean, const double &stdv)
    {
        std::default_random_engine generator;
        std::normal_distribution<double> distribution(mean, stdv);
        return double(distribution(generator));
    }
    // Converts a std type vector into and Eigen vector
    vXd convert2eigen(std::vector<float> stdVector)
    {
        Eigen::VectorXf eigenVector = Eigen::Map<Eigen::VectorXf>(stdVector.data(), stdVector.size());
        return eigenVector.cast<double>();
    }

    //================================//
    //          VARIABLES             //
    //================================//
    Robot robot;
    bool enabled;
    rclcpp::TimerBase::SharedPtr dvlTimer;
    rclcpp::TimerBase::SharedPtr imuTimer;
    rclcpp::TimerBase::SharedPtr depthTimer;
    rclcpp::TimerBase::SharedPtr statePubTimer;
    rclcpp::TimerBase::SharedPtr killSwitchTimer;
    rclcpp::TimerBase::SharedPtr thrusterTelemetryTimer;
    
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imuPub;
    rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr statePub;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr firmwareKillPub;
    rclcpp::Client<robot_localization::srv::SetPose>::SharedPtr poseClient;
    rclcpp::Service<robot_localization::srv::SetPose>::SharedPtr poseService;
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr thrusterSub;
    rclcpp::Publisher<geometry_msgs::msg::TwistWithCovarianceStamped>::SharedPtr dvlPub;
    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr depthPub;
    rclcpp::Subscription<riptide_msgs2::msg::KillSwitchReport>::SharedPtr softwareKillSub;
    rclcpp::Publisher<riptide_msgs2::msg::DshotPartialTelemetry>::SharedPtr thrusterTelemetryPub;
    
};

//=========================//
//          MAIN           //
//=========================//
int main(int argc, char *argv[])
{
    // Create PhysicsSimNode
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PhysicsSimNode>();
    // Load parameters into Robot class, don't start if unseccessful
    bool startUpSuccess = node->init();
    if (startUpSuccess)
    {
        // Parameters loaded sucessfully, start node
        RCLCPP_INFO(node->get_logger(), "Simulator node starting...");
        node->rungeKutta4();
        rclcpp::shutdown();
    }
    else
    {
        // Parameters failed to load, report error
        RCLCPP_FATAL(node->get_logger(), "SIM FAILED TO START: Could not load robot parameters");
    }
    return 0;
}