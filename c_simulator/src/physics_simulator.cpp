//===============================//
/*     TABLE OF CONTENTS         //
//===============================//
16  - Notes/assumptions
29  - Includes
67  - Sim start up
130 - Physics functions
255 - Collision functions
574 - Faking sensor data
727 - Callback functions
839 - Utility functions
874 - Variables
901 - Main

//===============================//
//      NOTES/ASSUMPTIONS        //
//===============================//
1) Change settings in the "setting.h" file in the include directory
2) Assumes thruster force curves are accuarate (if not, what the controller does IRL may not reflect response in sim)
3) Bouyant force of robot is approximated as a cylinder when robot is partially submereged
4) Collision detection only works with boxes and assumes obstacles are stationary
5) Assumes thrusters will not be running while out of water
6) Assumes center of drag is at center of bouyancy
7) Everything is in SI units (kg, m, s, N)
8) This will cook your CPU

//===============================//
//           INCLUDES            */
//===============================//
#include <set>
#include <chrono>
#include <memory>
#include <string>
#include <random>
#include <functional>
#include <filesystem>
#include <rclcpp/rclcpp.hpp>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <urdf_model/model.h>
#include <std_msgs/msg/bool.hpp>
#include "c_simulator/settings.h"
#include <sensor_msgs/msg/imu.hpp>
#include <std_msgs/msg/string.hpp>
#include "c_simulator/RobotClass.h"
#include <urdf_parser/urdf_parser.h>
#include <nav_msgs/msg/odometry.hpp>
#include "c_simulator/collisionBox.h"
#include <geometry_msgs/msg/pose.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <robot_localization/srv/set_pose.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <riptide_msgs2/msg/kill_switch_report.hpp>
#include <riptide_msgs2/msg/dshot_partial_telemetry.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>

using std::string, std::cout, std::endl;
typedef Eigen::Vector3d v3d;
typedef Eigen::Vector4d v4d;
typedef Eigen::VectorXd vXd;
typedef Eigen::Matrix3d m3d;
typedef Eigen::MatrixXd mXd;
typedef Eigen::Quaterniond quat;

using std::placeholders::_1;
namespace fs = std::filesystem;
using namespace std::chrono_literals;

class PhysicsSimNode : public rclcpp::Node
{
public:
    //================================//
    //         SIM START UP           //
    //================================//
    PhysicsSimNode() : Node("physics_simulator")
    {
        enabled = false;
        string name = this->get_namespace();
        robot = Robot(name.substr(1)); // Removing the starting '/' with substring

        // Create publishers and subscribers
        imuPub = this->create_publisher<sensor_msgs::msg::Imu>("vectornav/imu", 10);
        firmwareKillPub = this->create_publisher<std_msgs::msg::Bool>("state/kill", 10);
        statePub = this->create_publisher<geometry_msgs::msg::Pose>("simulator/state", 10);
        dvlPub = this->create_publisher<geometry_msgs::msg::TwistWithCovarianceStamped>("dvl_twist", 10);
        depthPub = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("depth/pose", 10);
        collisionBoxPub = this->create_publisher<visualization_msgs::msg::MarkerArray>("simulator/collisionMarkers", 10);
        thrusterTelemetryPub = this->create_publisher<riptide_msgs2::msg::DshotPartialTelemetry>("state/thrusters/telemetry", 10);
        thrusterSub = this->create_subscription<std_msgs::msg::Float32MultiArray>("thruster_forces", 10, std::bind(&PhysicsSimNode::forceCallback, this, _1));
        softwareKillSub = this->create_subscription<riptide_msgs2::msg::KillSwitchReport>("command/software_kill", 10, std::bind(&PhysicsSimNode::killSwitchCallback, this, _1));

        // Create timers
        auto statePubTime = std::chrono::duration<double>((double)STATE_PUB_TIME);
        statePubTimer = this->create_wall_timer(statePubTime, std::bind(&PhysicsSimNode::publishState, this));
        thrusterTelemetryTimer = this->create_wall_timer(0.5s, std::bind(&PhysicsSimNode::pubThrusterTelemetry, this));

        // Services for setting odom and simulator
        poseClient = this->create_client<robot_localization::srv::SetPose>("/" + robot.getName() + "/set_pose");
        poseService = this->create_service<robot_localization::srv::SetPose>("set_sim_pose", std::bind(&PhysicsSimNode::setSim, this, _1));

        // TF broadcaster
        tf_broadcaster = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    }

    /**
     * @brief Loads parameters from the robot's yaml file into the Robot object
     * @returns wether the parameters were loaded successfully
     */
    bool init()
    {
        // Tries to load robot parameter data
        RCLCPP_INFO(this->get_logger(), "Loading robot's parameter data from YAML..");
        bool paramsLoaded = robot.loadParams(shared_from_this());
        RCLCPP_INFO(this->get_logger(), "Loading collision files...");
        bool collisionBoxesLoaded = loadCollisionFiles();
        // Loaded successfully if true
        if (paramsLoaded && collisionBoxesLoaded)
        {
            //  Once YAML file params have been loaded, create timers to publish sensor data
            auto imuRate = std::chrono::duration<double>(robot.getIMURate());
            auto dvlRate = std::chrono::duration<double>(robot.getDVLRate());
            auto depthRate = std::chrono::duration<double>(robot.getDepthRate());
            RCLCPP_INFO(this->get_logger(), "Initization successful. Timers created to fake sensor data");
            imuTimer = this->create_wall_timer(imuRate, std::bind(&PhysicsSimNode::publishFakeIMUData, this));
            dvlTimer = this->create_wall_timer(dvlRate, std::bind(&PhysicsSimNode::publishFakeDVLData, this));
            depthTimer = this->create_wall_timer(depthRate, std::bind(&PhysicsSimNode::publishFakeDepthData, this));
        }
        else
        {
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
     * @brief Solves system of ODEs representing robot physics using fourth order Runge-Kutta numerical method.
     * This function is runs in an infinite loop to run the simulator, which also spins the ROS node
     *
     * Physics equations have been set up as a system of first order equations so that:
     * dx/dt = f(x)
     *
     * Where x is the robot's state as a column vector:
     * x = [x y z q_w q_x q_y q_z P_x P_y P_z L_x L_y L_z]^T     <- parameter
     *      0 1 2 3   4   5   6   7   8   9   10  11  12         <- index
     * with P meaning linear momentum, and L meaning angular momentum, and q meaning quaternion
     * Everything in the state vector is in world frame
     */
    void rungeKutta4()
    {
        // Clock time setup
        rclcpp::Time currentTime;
        rclcpp::Time prevTime = rclcpp::Clock(RCL_SYSTEM_TIME).now();
        startTime = this->get_clock()->now();

        // Start simulation infinite loop
        while (rclcpp::ok())
        {
            // Time since last loop, sets step size of simulation
            currentTime = rclcpp::Clock(RCL_SYSTEM_TIME).now();
            double stepSize = (currentTime - prevTime).seconds();
            prevTime = currentTime;

            // Get state, and modify it to account for any collisions
            vXd state = robot.getState();
            state = handleCollisions(state);

            // For more info on Runge Kutta:
            // https://www.youtube.com/watch?v=HOWJp8NV5xU
            // https://en.wikipedia.org/wiki/Runge%E2%80%93Kutta_methods
            // Evaluate derivative vector field at key points
            vXd K1 = calcStateDot(state);
            vXd K2 = calcStateDot(state + stepSize / 2.0 * K1);
            vXd K3 = calcStateDot(state + stepSize / 2.0 * K2);
            vXd K4 = calcStateDot(state + stepSize * K3);

            // Update the robots state based on weighted average of sampled points
            vXd stateDelta = stepSize / 6.0 * (K1 + 2 * K2 + 2 * K3 + K4);
            robot.setState(state + stateDelta);
            robot.setAccel(stateDelta / stepSize);

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
        // Rate of change of quaternion
        stateDot.segment(3, 4) = calcQuaternionDot(q, angularVel);
        // Change in linear momentum dP/dt = F (Newton's second law)
        stateDot.segment(7, 3) = calcForces(q, linearVel, depth);
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
        // Some forces are easier to calculate in the body frame, like thruster forces and drag forces, doing those first
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
    //      COLLISION FUNCTIONS       //
    //================================//

    /**
     * @brief Loops through all collision elements and adds impulses based on collision information
     * @param state Robot's state vector where collisions are evaluated at
     * @returns Modified state vector with added collision impulses
     */
    vXd handleCollisions(vXd state)
    {
        // Get state information
        quat q = state2quat(state);
        m3d invWorldInertia = q * robot.getInvInertia() * q.conjugate();
        v3d angularVel = invWorldInertia * state.segment(10, 3);
        v3d linearVel = state.segment(7, 3) / robot.getMass();

        // Loop through all combination of boxes
        for (collisionBox &robotBox : robotBoxes)
        {
            // Need to update robot collision boxes to robot's current position
            robotBox.updateLocation(state);
            for (collisionBox obstacleBox : obstacleBoxes)
            {
                // Compute the collision on selected boxes
                collisionResult collision = computeCollision(robotBox, obstacleBox);

                // A collision occoured, calculate how to change state in response to collision
                if (collision.collided)
                {
                    RCLCPP_DEBUG(this->get_logger(), "%s has collided with %s", robot.getName().c_str(), obstacleBox.getName().c_str());
                    // Compute impulse
                    // Using impulse method described here:
                    // https://research.ncl.ac.uk/game/mastersdegree/gametechnologies/physicstutorials/5collisionresponse/Physics%20-%20Collision%20Response.pdf
                    // And also here: https://hitokageproduction.com/article/11
                    v3d r_a = collision.collisionPoint - state.segment(0, 3);
                    double v_rel = (linearVel + angularVel.cross(r_a)).dot(collision.unitDirection);
                    double impulse = -(1.0 + COEF_OF_RESTITUTION) * v_rel /
                                     (1.0 / robot.getMass() + (invWorldInertia * (r_a.cross(collision.unitDirection))).cross(r_a).dot(collision.unitDirection));
                    // Seperate the objects
                    state.segment(0, 3) = collision.unitDirection * collision.depth + state.segment(0, 3);
                    // Update momentums
                    state.segment(7, 3) = state.segment(7, 3) + impulse * collision.unitDirection;
                    state.segment(10, 3) = state.segment(10, 3) + r_a.cross(impulse * collision.unitDirection);
                }
            }
        }
        return state;
    }

    /**
     * @brief Determines collision information for two collision boxes using the Seperating Axis Theorem (SAT) in 3D
     * @param box1 Collision box of the robot (must make sure to update position beforehand)
     * @param box2 Collision box of the obstacle
     * @returns Struct containing wether there was a collision, collision point, depth, and collision normal
     */
    collisionResult computeCollision(collisionBox &box1, collisionBox &box2)
    {
        // For more on Seperating Axis Theorem:
        // https://research.ncl.ac.uk/game/mastersdegree/gametechnologies/previousinformation/physics4collisiondetection/2017%20Tutorial%204%20-%20Collision%20Detection.pdf
        // Create axes to check projections on
        v3d axes[] = {box1.getAxis(0),
                      box1.getAxis(1),
                      box1.getAxis(2),
                      box2.getAxis(0),
                      box2.getAxis(1),
                      box2.getAxis(2),
                      box1.getAxis(0).cross(box2.getAxis(0)).normalized(),
                      box1.getAxis(0).cross(box2.getAxis(1)).normalized(),
                      box1.getAxis(0).cross(box2.getAxis(2)).normalized(),
                      box1.getAxis(1).cross(box2.getAxis(0)).normalized(),
                      box1.getAxis(1).cross(box2.getAxis(1)).normalized(),
                      box1.getAxis(1).cross(box2.getAxis(2)).normalized(),
                      box1.getAxis(2).cross(box2.getAxis(0)).normalized(),
                      box1.getAxis(2).cross(box2.getAxis(1)).normalized(),
                      box1.getAxis(2).cross(box2.getAxis(2)).normalized()};

        // Variable set up
        v3d minAxis;
        double overlap;
        double minOverlap = std::numeric_limits<double>::infinity();

        // Loop through all axes
        for (v3d axis : axes)
        {
            // If axis is zero vector, don't compute it. (Can occour because vectors from cross products could be alligned)
            if (axis.norm() < 0.9)
                continue;
            // If the center of box1 is to the right of box2, flip axis so it becomes to the left
            if (box1.getCenter().dot(axis) > box2.getCenter().dot(axis))
                axis = -axis;

            // Calculate the overlap between the projections of the two boxes onto the axis
            // Think about this like shining a flashlight onto the shape and looking at it's shadow
            // This finds the overlap between the shape's "shadows" along the axis to be tested
            overlap = box1.maxProjection(axis) - box2.minProjection(axis);

            // New smallest overlap found
            if (overlap < minOverlap)
            {
                minOverlap = overlap;
                minAxis = axis;
                if (minOverlap <= 0)
                    //  *gasp!* there is a way to look at the boxes without any overlap
                    //  The shapes can't be colliding then, return standard no-collision result
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
        // Ok, we have the axis that has the smallest overlap, but need to find if it belongs to first or second box
        // Need to know which box it belongs to because it determines which box's vertex to use as the collision point
        // Note that the minAxis might not be one of the three axes alligned with a box edge since the cross product of the axes could be the minAxes
        // The dot product is taken with each of the box axes, the one with the largest matches the box the most. This informs us on what box's vertex to use for the collision point
        (boxAxes.transpose() * minAxis).cwiseAbs().maxCoeff(&index);
        if (index < 3)
            // A face of the first box is getting intersected, thus a vertex from the second box must be intersecting it
            point = box2.minVertex(minAxis);
        else
            // A face of the second box is getting intersected, thus a vertex from the first box must be intersecting it
            point = box1.maxVertex(minAxis);

        // It's an edge on edge collision if true
        if (!box1.isInBox(point) || !box2.isInBox(point))
        {
            // The collision point needs a bit of persuasion to get to a reasonable location :P
            // Shuffle the point back and force between the two boxes to iteratively move the vertex closer to the edge collision point
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
     * @brief Goes through collision folder and reads URDF files to populate robotBoxes and obstacleBoxes vector with elements
     * @returns Whether the files could be read succesfully
     */
    bool loadCollisionFiles()
    {
        // Get folder path
        string collisionFolder;
        this->declare_parameter("collision_folder", "");
        this->get_parameter("collision_folder", collisionFolder);

        // Check to make sure robot folder exists
        fs::path robotFolder = fs::path(collisionFolder) / "robots";
        if (!(fs::exists(robotFolder) && fs::is_directory(robotFolder)))
        {
            RCLCPP_FATAL(this->get_logger(), "Robot folder could not be fould: %s", robotFolder.c_str());
            return false;
        }
        // Loop through each URDF in robot folder
        for (const auto &entry : fs::directory_iterator(robotFolder))
        {
            // Check if the file has a ".urdf" extension
            if (entry.path().extension() != ".urdf")
            {
                RCLCPP_ERROR(this->get_logger(), "Encountered a non .urdf file: %s", entry.path().c_str());
                continue;
            }
            urdf::ModelInterfaceSharedPtr robotModel = urdf::parseURDFFile(entry.path());
            if (!robotModel)
            {
                RCLCPP_FATAL(this->get_logger(), "Failed to parse URDF file %s", entry.path().c_str());
                return false;
            }
            // If URDF robot's name matches the robot that is being simulated, unpack file
            if (robotModel->getName() == robot.getName())
            {
                robotBoxes = unpackURDF(robotModel, v3d(0, 0, 0), quat(1, 0, 0, 0));
                break;
            }
        }
        if (robotBoxes.empty())
        {
            RCLCPP_FATAL(this->get_logger(), "Could not find any robot collision boxes");
            return false;
        }
        // Check to make sure obstacle foldeer exists
        fs::path obstacleFolder = fs::path(collisionFolder) / "obstacles";
        if (!(fs::exists(obstacleFolder)) && fs::is_directory(obstacleFolder))
        {
            RCLCPP_FATAL(this->get_logger(), "Obstacle folder could not be fould: %s", robotFolder.c_str());
            return false;
        }
        // Load YAML file storing obstacle positions
        string obstacleConfigPath;
        this->declare_parameter("obstacle_config", "");
        this->get_parameter("obstacle_config", obstacleConfigPath);
        cout << obstacleConfigPath << endl;
        YAML::Node obstacleConfig = YAML::LoadFile(obstacleConfigPath);
        YAML::Node obstacleList = obstacleConfig["/**/riptide_mapping2"]["ros__parameters"]["init_data"];
        // Add available obstacles to list of names
        std::set<string> obstacleNames;
        for (const auto &obstacle : obstacleList)
            obstacleNames.insert(obstacle.first.as<string>());
        // Loop through each URDF in obstacle folder
        for (const auto &entry : fs::directory_iterator(obstacleFolder))
        {
            // Check if the file has a ".urdf" extension
            if (entry.path().extension() != ".urdf")
            {
                RCLCPP_ERROR(this->get_logger(), "Encountered a non .urdf file, skipping: %s", entry.path().c_str());
                continue;
            }
            urdf::ModelInterfaceSharedPtr obstacleModel = urdf::parseURDFFile(entry.path());
            if (!obstacleModel)
            {
                RCLCPP_ERROR(this->get_logger(), "Failed to parse URDF file %s", entry.path().c_str());
                return false;
            }
            // Check if object is one of the known task objects
            if (obstacleNames.count(obstacleModel->getName()) > 0) // name matches obstacle name list
            {
                // Get position and orientation of object
                double poseX = obstacleList[obstacleModel->getName()]["pose"]["x"].as<double>();
                double poseY = obstacleList[obstacleModel->getName()]["pose"]["y"].as<double>();
                double poseZ = obstacleList[obstacleModel->getName()]["pose"]["z"].as<double>();
                double poseYaw = obstacleList[obstacleModel->getName()]["pose"]["yaw"].as<double>();
                tf2::Quaternion tf2Quat;
                tf2Quat.setRPY(0, 0, poseYaw);
                tf2Quat.normalize();
                Eigen::AngleAxisd yawing(poseYaw * M_PI / 180, v3d::UnitZ());
                quat objQuat(yawing);
                // Unpack URDF, set position and orientation to the object
                std::vector<collisionBox> newBoxes = unpackURDF(obstacleModel,
                                                                v3d(poseX,
                                                                    poseY,
                                                                    poseZ),
                                                                objQuat);
                // Add all new boxes to obstacle list
                for (collisionBox newBox : newBoxes)
                    obstacleBoxes.push_back(newBox);
                RCLCPP_DEBUG(this->get_logger(), "Succesfully added %s collision data into simulator", obstacleModel->getName().c_str());
            }
            else
            {
                // Obstacle does not match any known obstacle
                RCLCPP_ERROR(this->get_logger(), "Obstacle %s does not match list of known obstacles, skipping obstacle", obstacleModel->getName().c_str());
                continue;
            }
        }
        // Add box to list
        collisionBox floor = collisionBox("floor",
                                          25,
                                          50,
                                          1,
                                          v3d(0, 0, -2.6336),
                                          v3d(0, 0, 0),
                                          quat(1, 0, 0, 0),
                                          quat(1, 0, 0, 0));
        obstacleBoxes.push_back(floor);
        if (obstacleBoxes.empty())
        {
            RCLCPP_FATAL(this->get_logger(), "No obstacle could not be found or properly parsed");
            return false;
        }
        return true;
    }

    /**
     * @brief Unpacks collision information from URDF model and stores it's content into a collisionBox object. Note this only works for boxes
     * @param model URDF model
     * @param basePosition Location of orgin of obstacle in world frame
     * @param baseOrientation Orientation of obstacle represented by quaternion.
     * @returns vector containing collision boxes for each object of URDF
     */
    std::vector<collisionBox> unpackURDF(urdf::ModelInterfaceSharedPtr model, v3d basePosition, quat baseOrientation)
    {
        std::vector<collisionBox> newBoxes;
        // Loop through each link in the URDF
        for (const auto &link_pair : model->links_)
        {
            const urdf::LinkSharedPtr &link = link_pair.second;
            // Loop through each collision element in each link
            for (const auto &collision : link->collision_array)
            {
                const urdf::CollisionSharedPtr &collision_ptr = collision;
                // Check if it's a box (current collision implementation only works with boxes)
                if (collision_ptr->geometry->type == urdf::Geometry::BOX)
                {
                    const urdf::BoxSharedPtr &box = std::static_pointer_cast<urdf::Box>(collision_ptr->geometry);
                    // Get information from box element
                    double length = box->dim.x;
                    double width = box->dim.y;
                    double height = box->dim.z;
                    // Get box posititon relative to obstacle's base
                    v3d baseOffset(collision_ptr->origin.position.x,
                                   collision_ptr->origin.position.y,
                                   collision_ptr->origin.position.z);
                    // Get box orientation relative to obstacle's base
                    quat baseOrientationOffset(collision_ptr->origin.rotation.w,
                                               collision_ptr->origin.rotation.x,
                                               collision_ptr->origin.rotation.y,
                                               collision_ptr->origin.rotation.z);

                    // Add box to list
                    collisionBox newBox = collisionBox(link->name,
                                                       length,
                                                       width,
                                                       height,
                                                       basePosition,
                                                       baseOffset,
                                                       baseOrientation,
                                                       baseOrientationOffset);
                    newBoxes.push_back(newBox);
                }
                else
                {
                    RCLCPP_ERROR(this->get_logger(), "Link %s uses a non-box geometry, only boxes are supportede. Skipping element", link->name.c_str());
                }
            }
        }
        return newBoxes;
    }

    /**
     * @brief Publishes a marker array containing each collision box for robot and obstacle.
     * This allows RViz to visualize the collision elements for information and debugging
     */
    void publishCollisionMarkers()
    {
        // Combine obstacleBoxes and robotBoxes into a single list to loop through
        std::vector<collisionBox> combinedBoxList = obstacleBoxes;
        combinedBoxList.insert(combinedBoxList.begin(), robotBoxes.begin(), robotBoxes.end());
        // Loop through each collision box and add to marker array
        visualization_msgs::msg::MarkerArray markerArray;
        int markerID = 0;
        rclcpp::Time stampTime = this->now();
        for (collisionBox boxInfo : combinedBoxList)
        {
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

    // Fabricates fake depth sensor data from robot's state and publishes it to topic. Called on a timer
    void publishFakeDepthData()
    {
        // Get depth and add noise to data
        double depthData = robot.getState().z();
        if (SENSOR_NOISE_ENABLED)
            depthData = randomNorm(depthData, robot.getDepthSigma());

        // Send message with depth sensor info
        geometry_msgs::msg::PoseWithCovarianceStamped depthMsg;
        depthMsg.pose.pose.position.z = depthData;
        depthMsg.pose.covariance[14] = robot.getDepthSigma() * robot.getDepthSigma();
        depthMsg.header.stamp = this->get_clock()->now();
        depthMsg.header.frame_id = "odom";
        depthPub->publish(depthMsg);
    }

    // Fabricates fake DVL data from robot's state and publishes it to topic. Called on a timer
    void publishFakeDVLData()
    {
        if (robot.dvlTransformAvailable())
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
            // Transform velocity from world frame -> robot frame -> DVL frame
            dvlData = robot.getDVLQuat().conjugate() * (q.conjugate() * dvlData);

            // Add nonise to sensor data if enabled, otherwise don't
            if (SENSOR_NOISE_ENABLED)
                dvlData = randomNorm(dvlData, robot.getDVLSigma());
            // Send message with DVL sensor info
            geometry_msgs::msg::TwistWithCovarianceStamped dvlMsg;
            dvlMsg.twist.twist.linear.x = dvlData.x();
            dvlMsg.twist.twist.linear.y = dvlData.y();
            dvlMsg.twist.twist.linear.z = dvlData.z();
            // Add covariance to message
            double dvlVariance = robot.getDVLSigma() * robot.getDVLSigma();
            dvlMsg.twist.covariance[21] = dvlVariance;
            dvlMsg.twist.covariance[28] = dvlVariance;
            dvlMsg.twist.covariance[35] = dvlVariance;
            // Send message
            dvlMsg.header.stamp = this->get_clock()->now();
            dvlMsg.header.frame_id = robot.getName() + "/dvl_link";
            dvlPub->publish(dvlMsg);
        }
    }

    // Fabricates fake IMU data from robot's state and publishes it to topic. Called on a timer
    void publishFakeIMUData()
    {
        if (robot.imuTransformAvailable())
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
            angularVel = robot.getIMUQuat().conjugate() * (q.conjugate() * angularVel);
            imuAccel = robot.getIMUQuat().conjugate() * (q.conjugate() * imuAccel);
            // Transform orientation from robot -> IMU
            q = q * robot.getIMUQuat();
            // Add nonise to sensor data if enabled, otherwise don't
            v3d imu_sigma = robot.getIMUSigma();
            if (SENSOR_NOISE_ENABLED)
            {
                imuAccel = randomNorm(imuAccel, imu_sigma[0]);
                angularVel = randomNorm(angularVel, imu_sigma[1]);
                // Create a rotation some random amount about some random axis
                v3d randomAxis;
                randomAxis.setRandom().normalize();
                Eigen::AngleAxisd randomAngleNoise(imu_sigma[2], randomAxis);
                // Drift = drift_speed * elapsed_time
                double elapsedTime = (this->get_clock()->now() - startTime).seconds() / 60.0;
                double driftAngle = elapsedTime * robot.getIMUDrift() * M_PI / 180;
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
            imuMsg.header.stamp = this->get_clock()->now();
            imuMsg.header.frame_id = robot.getName() + "/imu_link";
            imuPub->publish(imuMsg);
        }
    }

    //================================//
    //       CALLBACK FUNCTIONS       //
    //================================//

    // Publishes fake thruster telemtry messages to make controller work
    void pubThrusterTelemetry()
    {
        // Create messages
        riptide_msgs2::msg::DshotPartialTelemetry msgLow;
        riptide_msgs2::msg::DshotPartialTelemetry msgHigh;
        // Set message data to enable thrusters
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

        // Publish thruster telemetery messages
        this->thrusterTelemetryPub->publish(msgLow);
        this->thrusterTelemetryPub->publish(msgHigh);
    }

    // Once new thruster forces are available, add them to thruster que
    void forceCallback(const std_msgs::msg::Float32MultiArray &thrusterForces)
    {
        // Create a zero vector with the current time
        thrusterForcesStamped commandedThrust(vXd::Zero(robot.getThrusterCount()), this->get_clock()->now().seconds());
        // Only set forces if robot's enabled
        if (enabled)
            commandedThrust.thrusterForces = convert2eigen(thrusterForces.data);
        robot.addToThrusterQue(commandedThrust);
    }

    // Mirrors software kill to firmware kill. This is done to make the enable/disable button in RViz work
    void killSwitchCallback(const riptide_msgs2::msg::KillSwitchReport &softwareKillMsg)
    {
        // Make sure the correct kill switch is being called
        if (softwareKillMsg.kill_switch_id == 1)
        {
            std_msgs::msg::Bool message;
            message.data = softwareKillMsg.switch_asserting_kill;
            enabled = !message.data;
            firmwareKillPub->publish(message);
            // Turn off thrusters if disabled
            if (!enabled)
                robot.addToThrusterQue(thrusterForcesStamped(vXd::Zero(robot.getThrusterCount()),
                                                             this->get_clock()->now().seconds()));
        }
    }

    // Function called by "/set_sim_pose" service. Updates simulator and EKF to requested position
    void setSim(const std::shared_ptr<robot_localization::srv::SetPose::Request> poseRequest)
    {
        // Create request to EKF, wait for availablility
        auto request = std::make_shared<robot_localization::srv::SetPose::Request>();
        geometry_msgs::msg::Pose desiredPose = poseRequest->pose.pose.pose;
        request->pose.pose.pose = desiredPose;
        request->pose.pose.covariance = {0.0};
        request->pose.header.stamp = this->get_clock()->now();
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

    /**
     * @brief Called on a timer, publishes the following simulator information:
     * 1) Robot's pose for RViz
     * 2) Collision boxes marker array for RViz
     * 3) TF frames for simulated robot and cameras for Zed SDK image faking
     */
    void publishState()
    {
        if (robot.mapAvailable())
        {
            // Update collision box markers for RViz display
            publishCollisionMarkers();

            // Getting state information from simulator
            vXd state = robot.getState();
            quat q = state2quat(state);
            v3d baseLinkOffset = q * robot.getBaseLinkOffset(); // Converting baseLink to world frame

            // Getting position
            // Sim uses COM for robot position, need to add offset for position relative to base link
            geometry_msgs::msg::Pose poseMsg;
            poseMsg.position.x = state.x() - baseLinkOffset.x();
            poseMsg.position.y = state.y() - baseLinkOffset.y();
            poseMsg.position.z = state.z() - baseLinkOffset.z();
            // Setting orientation
            poseMsg.orientation.w = q.w();
            poseMsg.orientation.x = q.x();
            poseMsg.orientation.y = q.y();
            poseMsg.orientation.z = q.z();

            // Setting up TF messages
            geometry_msgs::msg::TransformStamped robotFrame;
            geometry_msgs::msg::TransformStamped cameraFrameL;
            rclcpp::Time clockTime = this->get_clock()->now();
            robotFrame.header.stamp = clockTime;
            cameraFrameL.header.stamp = clockTime;
            // Frame names
            robotFrame.header.frame_id = "odom";
            robotFrame.child_frame_id = "simulator/" + robot.getName() + "/base_link";
            cameraFrameL.header.frame_id = robotFrame.child_frame_id;
            cameraFrameL.child_frame_id = "simulator/" + robot.getName() + "/zed2i/left_optical";
            // Set message position
            robotFrame.transform.translation.x = state.x();
            robotFrame.transform.translation.y = state.y();
            robotFrame.transform.translation.z = state.z();
            cameraFrameL.transform = robot.getLCameraTransform();
            // Set message orientation
            robotFrame.transform.rotation.w = q.w();
            robotFrame.transform.rotation.x = q.x();
            robotFrame.transform.rotation.y = q.y();
            robotFrame.transform.rotation.z = q.z();

            // Broadcast and publish messages
            statePub->publish(poseMsg);
            tf_broadcaster->sendTransform(robotFrame);
            tf_broadcaster->sendTransform(cameraFrameL);
        }
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
    string name;
    Robot robot;
    bool enabled;
    rclcpp::Time startTime;
    std::vector<collisionBox> robotBoxes;
    rclcpp::TimerBase::SharedPtr dvlTimer;
    rclcpp::TimerBase::SharedPtr imuTimer;
    rclcpp::TimerBase::SharedPtr depthTimer;
    std::vector<collisionBox> obstacleBoxes;
    rclcpp::TimerBase::SharedPtr statePubTimer;
    rclcpp::TimerBase::SharedPtr killSwitchTimer;
    rclcpp::TimerBase::SharedPtr thrusterTelemetryTimer;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imuPub;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster;
    rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr statePub;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr firmwareKillPub;
    rclcpp::Client<robot_localization::srv::SetPose>::SharedPtr poseClient;
    rclcpp::Service<robot_localization::srv::SetPose>::SharedPtr poseService;
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr thrusterSub;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr collisionBoxPub;
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
        RCLCPP_FATAL(node->get_logger(), "SIM FAILED TO START: Could not initialize");
    }
    return 0;
}