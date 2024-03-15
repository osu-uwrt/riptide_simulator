

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

//===============================//
//           INCLUDES            */
//===============================//
#include "rclcpp/rclcpp.hpp"
#include <glad/glad.h>
#include <chrono>
#include <GLFW/glfw3.h>
#define STB_IMAGE_IMPLEMENTATION
#include <glm/glm.hpp>
#include <stb_image.h>
#include <shader.hpp>
#include <camera.hpp>
#include <iostream>
#include <filesystem>
#include <fstream>
#include <obstackeShader.hpp>
#include <waterShader.hpp>
#include <object.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>

using std::string, std::cout, std::endl;
using namespace std::chrono_literals;
namespace fs = std::filesystem;

class zedFakerNode;
std::shared_ptr<zedFakerNode> node;
class zedFakerNode : public rclcpp::Node
{
public:
    zedFakerNode() : Node("zed_faker")
    {
        // Create a TF broadcaster
        tfBroadcaster = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

        // Create camera info and image publishers
        depthPub = this->create_publisher<sensor_msgs::msg::Image>("zed/zed_node/depth/depth_registered", 10);
        zedInfoPub = this->create_publisher<sensor_msgs::msg::CameraInfo>("zed/zed_node/left/camera_info", 10);
        imagePub = this->create_publisher<sensor_msgs::msg::Image>("zed/zed_node/left_raw/image_raw_color", 10);
        depthInfoPub = this->create_publisher<sensor_msgs::msg::CameraInfo>("zed/zed_node/depth/camera_info", 10);

        // Create timers
        std::chrono::duration<double> imgPubTime(1.0 / 1.0);
        imgPubTimer = this->create_wall_timer(1s, std::bind(&zedFakerNode::publishImages, this));

        // Setup OpenGL, shaders, and objects
        openGlSetup();
        shaderSetup();
        objectSetup();
        renderFBO = FBO(true);
    }

    //===============================//
    //        IMAGE CREATION         //
    //===============================//
    void fakeImages()
    {
        waitForTF();
        while (rclcpp::ok() && !glfwWindowShouldClose(window))
        {
            glEnable(GL_DEPTH_TEST);
            // Get the time between frames, used for camera movement
            double currentFrame = glfwGetTime();
            deltaTime = currentFrame - lastFrame;
            lastFrame = currentFrame;

            // std::cout << "FPS: " << 1.0 / deltaTime << std::endl;

            // Process any keyboard presses
            processInput(window);

            // Clear screen and depth buffer
            // renderFBO.use();
            // renderFBO.clear();

            objectShader.render(objects, flyAroundCamera);
            // waterShader.render(objects, flyAroundCamera, renderFBO, objectShader);
            //    Render objects
            //    Move camera, swap framm buffers
            //    Render reflections
            //    render skybox
            //    Move camera back, render water
            //    render skybox

            // Check what camera is being used
            // If using vehicle camera
            // Distort image
            // Add blur
            // Add overlay
            // else
            // render talos
            glfwSwapBuffers(window);
            glfwPollEvents();

            // Proccess any pending callbacks or timers

            rclcpp::spin_some(shared_from_this());
        }
        glfwTerminate();
    };

private:
    //===============================//
    //            SETUP              //
    //===============================//
    bool openGlSetup()
    {
        // OpenGL initialization
        glfwInit();
        glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
        glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
        glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);

        // Creating window
        window = glfwCreateWindow(1920, 1080, "Zed Faker", glfwGetPrimaryMonitor(), NULL);
        if (window == NULL)
        {
            // Window failed to be created
            std::cout << "Failed to create GLFW window" << std::endl;
            glfwTerminate();
            return false;
        }

        // Setup window and callback functions
        glfwMakeContextCurrent(window);
        glfwSwapInterval(1);
        glfwSetFramebufferSizeCallback(window, resizeWindow);
        glfwSetCursorPosCallback(window, mouseCallback);

        // Get GLAD library function pointers
        if (!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress))
        {
            std::cout << "Failed to initialize GLAD" << std::endl;
            return false;
        }
        // STBI image library setup
        stbi_set_flip_vertically_on_load(true);
        return true;
    }

    void shaderSetup()
    {
        // Get shader folder path
        string shaderFolder, vsPath, fsPath;
        this->declare_parameter("shader_folder", "");
        this->get_parameter("shader_folder", shaderFolder);
        // Get texture folder path
        string textureFolder, causticsPath;
        this->declare_parameter("texture_folder", "");
        this->get_parameter("texture_folder", textureFolder);
        causticsPath = fs::path(textureFolder) / "caustics";

        // Create object shader
        vsPath = fs::path(shaderFolder) / "object.vs";
        fsPath = fs::path(shaderFolder) / "object.fs";
        objectShader = ObjectShader(vsPath.c_str(), fsPath.c_str(), causticsPath.c_str());

        // Create water shader
        vsPath = fs::path(shaderFolder) / "water.vs";
        fsPath = fs::path(shaderFolder) / "water.fs";
        waterShader = WaterShader(vsPath.c_str(), fsPath.c_str());

        // Create post processing shaders
        //
    }
    void objectSetup()
    {
        // Get texture folder path
        string textureFolder, texturePath;
        this->get_parameter("texture_folder", textureFolder);

        // Task objects
        texturePath = fs::path(textureFolder) / "task_objects" / "buoys.png";
        Object bouy(texturePath, 1.2f, 1.2f, glm::vec3(0, 4.0, -.85));
        texturePath = fs::path(textureFolder) / "task_objects" / "torpedoes.png";
        Object torpedo(texturePath, 1.2f, 1.2f, glm::vec3(2, 3.0, -.75), 0, 0, -30);
        objects.push_back(bouy);
        objects.push_back(torpedo);

        // Surrounding ground
        std::vector<Object>
            poolObjects = generatePoolObjects(textureFolder);
        for (Object poolObject : poolObjects)
            objects.push_back(poolObject);
    }

    void waitForTF()
    {
    }

    //===============================//
    //      CALLBACK FUNCTIONS       //
    //===============================//
    void publishImages()
    {
        sensor_msgs::msg::Image img, depth;
        imagePub->publish(img);
        depthPub->publish(depth);
    }
    void processInput(GLFWwindow *window)
    {
        if (glfwGetKey(window, GLFW_KEY_ESCAPE) == GLFW_PRESS)
            glfwSetWindowShouldClose(window, true);

        // Handle camera movements
        if (cameraMode == FLY_ARROUND_MODE)
        {
            if (glfwGetKey(window, GLFW_KEY_W) == GLFW_PRESS)
                flyAroundCamera.ProcessKeyboard(FORWARD, deltaTime);
            if (glfwGetKey(window, GLFW_KEY_S) == GLFW_PRESS)
                flyAroundCamera.ProcessKeyboard(BACKWARD, deltaTime);
            if (glfwGetKey(window, GLFW_KEY_A) == GLFW_PRESS)
                flyAroundCamera.ProcessKeyboard(LEFT, deltaTime);
            if (glfwGetKey(window, GLFW_KEY_D) == GLFW_PRESS)
                flyAroundCamera.ProcessKeyboard(RIGHT, deltaTime);
            if (glfwGetKey(window, GLFW_KEY_SPACE) == GLFW_PRESS)
                flyAroundCamera.ProcessKeyboard(UP, deltaTime);
            if (glfwGetKey(window, GLFW_KEY_LEFT_SHIFT) == GLFW_PRESS)
                flyAroundCamera.ProcessKeyboard(DOWN, deltaTime);
        }

        // Show april tag if true
        if (glfwGetKey(window, GLFW_KEY_T) == GLFW_PRESS)
            int a = 1;

        // Toggle camera mode
        if (glfwGetKey(window, GLFW_KEY_TAB) == GLFW_PRESS)
        {
            if (!cameraModeFlag)
            {
                cameraModeFlag = true;
                if (cameraMode == ROBOT_MODE)
                {
                    // TODO move flyaround camera position
                    cameraMode = FLY_ARROUND_MODE;
                }
                else
                    cameraMode = ROBOT_MODE;
            }
        }
        else
            cameraModeFlag = false;
    }
    static void resizeWindow(GLFWwindow *window, int width, int height)
    {
        glViewport(0, 0, width, height);
    }

    void moveCamera(double xpos, double ypos)
    {
        // The user can't use the mouse to control the robot's POV
        if (cameraMode == ROBOT_MODE)
            return;

        // Figure out how much the mouse moved
        float xoffset = lastX - xpos;
        float yoffset = lastY - ypos; // reversed since y-coordinates go from bottom to top
        lastX = xpos;
        lastY = ypos;

        // Update where the camera is looking
        flyAroundCamera.ProcessMouseMovement(xoffset, yoffset);
    }
    static void mouseCallback(GLFWwindow *window, double xpos, double ypos)
    {
        node->moveCamera(xpos, ypos);
    }

    std::vector<Object> generatePoolObjects(string textureFolder)
    {
        fs::path poolFolder = fs::path(textureFolder) / "pool";
        std::vector<Object> poolObjects;
        // POOL FLOOR
        // Made up of 9 objects, 4 corner objects, 4 side objects, 1 middle object.
        // Kiddle
        string poolTexture = poolFolder / "Pool_Cross.jpg";
        Object middleFloor(poolTexture, POOL_LENGTH - 3 * LANE_WIDTH, POOL_WIDTH - 3 * LANE_WIDTH, glm::vec3(0, 0, -POOL_DEPTH), 0, -90, -90, 17, 7);
        // Sides
        poolTexture = poolFolder / "Pool_Black_T.jpg";
        Object longSideTopFloor(poolTexture, POOL_LENGTH - 3 * LANE_WIDTH, LANE_WIDTH * 1.5, glm::vec3(POOL_WIDTH / 2 - 0.75 * LANE_WIDTH, 0, -POOL_DEPTH), 0, -90, -90, 17, 1);
        Object longSideBottomFloor(poolTexture, POOL_LENGTH - 3 * LANE_WIDTH, LANE_WIDTH * 1.5, glm::vec3(-POOL_WIDTH / 2 + 0.75 * LANE_WIDTH, 0, -POOL_DEPTH), 0, 90, -90, 17, 1);
        poolTexture = poolFolder / "Pool_Blue_T.jpg";
        Object shortSideRightFloor(poolTexture, LANE_WIDTH * 1.5, POOL_WIDTH - 3 * LANE_WIDTH, glm::vec3(0, POOL_LENGTH / 2 - 0.75 * LANE_WIDTH, -POOL_DEPTH), 0, 90, 90, 1, 7);
        Object shortSideLeftFloor(poolTexture, LANE_WIDTH * 1.5, POOL_WIDTH - 3 * LANE_WIDTH, glm::vec3(0, -POOL_LENGTH / 2 + 0.75 * LANE_WIDTH, -POOL_DEPTH), 0, 90, -90, 1, 7);
        // Corners
        poolTexture = poolFolder / "Pool_Corner.jpg";
        Object topRightCornerPool(poolTexture, LANE_WIDTH * 1.5, LANE_WIDTH * 1.5, glm::vec3(POOL_WIDTH / 2 - 0.75 * LANE_WIDTH, POOL_LENGTH / 2 - 0.75 * LANE_WIDTH, -POOL_DEPTH), 0, -90, 90, 1, 1);
        Object bottomRightCornerPool(poolTexture, LANE_WIDTH * 1.5, LANE_WIDTH * 1.5, glm::vec3(-POOL_WIDTH / 2 + 0.75 * LANE_WIDTH, POOL_LENGTH / 2 - 0.75 * LANE_WIDTH, -POOL_DEPTH), 0, 90, 90, 1, 1);
        Object topLeftCornerPool(poolTexture, LANE_WIDTH * 1.5, LANE_WIDTH * 1.5, glm::vec3(POOL_WIDTH / 2 - 0.75 * LANE_WIDTH, -POOL_LENGTH / 2 + 0.75 * LANE_WIDTH, -POOL_DEPTH), 0, -90, -90, 1, 1);
        Object bottomLeftCornerPool(poolTexture, LANE_WIDTH * 1.5, LANE_WIDTH * 1.5, glm::vec3(-POOL_WIDTH / 2 + 0.75 * LANE_WIDTH, -POOL_LENGTH / 2 + 0.75 * LANE_WIDTH, -POOL_DEPTH), 0, 90, -90, 1, 1);

        // POOL WALLS
        // Wall Sides
        poolTexture = poolFolder / "Pool_Wall_Black.jpg";
        Object topSidePool(poolTexture, POOL_LENGTH - 3 * LANE_WIDTH, POOL_DEPTH + LEDGE_HEIGHT, glm::vec3(POOL_WIDTH / 2, 0, (LEDGE_HEIGHT - POOL_DEPTH) / 2), 0, 0, 90, 17, 1);
        Object bottomSidePool(poolTexture, POOL_LENGTH - 3 * LANE_WIDTH, POOL_DEPTH + LEDGE_HEIGHT, glm::vec3(-POOL_WIDTH / 2, 0, (LEDGE_HEIGHT - POOL_DEPTH) / 2), 0, 0, 90, 17, 1);
        poolTexture = poolFolder / "Pool_Wall_Blue.jpg";
        Object leftSidePool(poolTexture, POOL_WIDTH - 3 * LANE_WIDTH, POOL_DEPTH + LEDGE_HEIGHT, glm::vec3(0, -POOL_LENGTH / 2, (LEDGE_HEIGHT - POOL_DEPTH) / 2), 0, 0, 0, 7, 1);
        Object rightSidePool(poolTexture, POOL_WIDTH - 3 * LANE_WIDTH, POOL_DEPTH + LEDGE_HEIGHT, glm::vec3(0, POOL_LENGTH / 2, (LEDGE_HEIGHT - POOL_DEPTH) / 2), 0, 0, 0, 7, 1);

        // Add objects to list and return
        poolObjects.push_back(middleFloor);
        poolObjects.push_back(longSideTopFloor);
        poolObjects.push_back(longSideBottomFloor);
        poolObjects.push_back(shortSideRightFloor);
        poolObjects.push_back(shortSideLeftFloor);
        poolObjects.push_back(bottomRightCornerPool);
        poolObjects.push_back(topRightCornerPool);
        poolObjects.push_back(topLeftCornerPool);
        poolObjects.push_back(bottomLeftCornerPool);
        poolObjects.push_back(topSidePool);
        poolObjects.push_back(bottomSidePool);
        poolObjects.push_back(rightSidePool);
        poolObjects.push_back(leftSidePool);
        return poolObjects;
    }
    //===============================//
    //          VARIABLES            //
    //===============================//
    float lastFrame = 0.0;
    double deltaTime = 0.0;
    bool cameraModeFlag = false;
    float lastX = IMG_WIDTH / 2.0;
    float lastY = IMG_HEIGHT / 2.0;
    Shader testShader;
    ObjectShader objectShader;
    WaterShader waterShader;
    Camera vehicleCamera = Camera();
    Camera flyAroundCamera = Camera();
    CameraMode cameraMode = FLY_ARROUND_MODE;

    std::vector<Object> objects;
    GLFWwindow *window;
    FBO renderFBO = FBO();
    rclcpp::TimerBase::SharedPtr imgPubTimer;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tfBroadcaster;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr depthPub;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr imagePub;
    rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr zedInfoPub;
    rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr depthInfoPub;
};

//===============================//
//            MAIN               //
//===============================//
int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    node = std::make_shared<zedFakerNode>();
    node->fakeImages();
    // rclcpp::spin(node);
    rclcpp::shutdown();
    return 1;
}