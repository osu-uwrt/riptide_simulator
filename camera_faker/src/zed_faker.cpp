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
#include <tf2/exceptions.h>
#include <tf2_ros/buffer.h>
#include "tf2_ros/transform_listener.h"
#include <geometry_msgs/msg/pose.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <frameShader.hpp>
#include <blurShader.hpp>

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
        tfBuffer = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tfListener = std::make_shared<tf2_ros::TransformListener>(*tfBuffer);

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
        screenFBO = FBO(true, true);
        render1FBO = FBO(true);
        render2FBO = FBO(true);
    }

    //===============================//
    //        IMAGE CREATION         //
    //===============================//
    void fakeImages()
    {
        // waitForTF();
        while (rclcpp::ok() && !glfwWindowShouldClose(window))
        {
            // Starting things
            clearBuffers();
            //  Get the time between frames, used for camera movement
            double currentFrame = glfwGetTime();
            deltaTime = currentFrame - lastFrame;
            lastFrame = currentFrame;
            processInput(window);

            std::cout << "FPS: " << 1.0 / deltaTime << std::endl;
            // Render differenetly depending on what camera is being used
            if (true)
            {
                // updateRobotCamera();
                render1FBO.use();
                objectShader.render(objects, flyAroundCamera);
                waterShader.render(objects, flyAroundCamera, render1FBO, objectShader);

                // Post processing effects
                render2FBO.use();
                frameShader.renderDistorted(render1FBO);         // Camera distortion
                blurShader.renderBlurred(render2FBO, screenFBO); // Blur
                frameShader.render(vehicleOverlay);              // Vehicle overlay
            }
            else
            {
                screenFBO.use();
                objectShader.render(objects, flyAroundCamera);
                waterShader.render(objects, flyAroundCamera, screenFBO, objectShader);
            }

            glfwSwapBuffers(window);
            glfwPollEvents();

            // Proccess any pending ROS callbacks or timers
            rclcpp::spin_some(shared_from_this());
        }
        glfwTerminate();
    };

private:
    //===============================//
    //     RENDER FUNCTIONS          //
    //===============================//
    void clearBuffers()
    {
        screenFBO.clear();
        render1FBO.clear();
        render2FBO.clear();
    }
    void updateRobotCamera()
    {
        string cameraFrame = "odom/simulator/talos/zed2i/left_optical";
        geometry_msgs::msg::TransformStamped t = tfBuffer->lookupTransform(
            cameraFrame, "world",
            tf2::TimePointZero);
        vehicleCamera.Front;
        vehicleCamera.Right;
        vehicleCamera.Up;
    }

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
        string textureFolder, causticsPath, waterPath;
        this->declare_parameter("texture_folder", "");
        this->get_parameter("texture_folder", textureFolder);
        waterPath = fs::path(textureFolder) / "water";
        causticsPath = fs::path(textureFolder) / "water" / "caustics";

        // Create post processing shaders
        vsPath = fs::path(shaderFolder) / "frame.vs";
        fsPath = fs::path(shaderFolder) / "frame.fs";
        frameShader = FrameShader(vsPath, fsPath);

        // Making the object shader takes a lot of time to load the 200+ caustic textures xD
        // Use the newly created frameShader to display a loading screen for some eye candy
        Texture loadingScreen = Texture(fs::path(textureFolder) / "overlays" / "Loading_Screen.jpg");
        frameShader.render(loadingScreen);
        glfwSwapBuffers(window);

        // Create object shader
        vsPath = fs::path(shaderFolder) / "object.vs";
        fsPath = fs::path(shaderFolder) / "object.fs";
        objectShader = ObjectShader(vsPath, fsPath, causticsPath);

        // Create water shader
        vsPath = fs::path(shaderFolder) / "water.vs";
        fsPath = fs::path(shaderFolder) / "water.fs";
        waterShader = WaterShader(vsPath, fsPath, waterPath);

        // Create blur shader
        vsPath = fs::path(shaderFolder) / "blur.vs";
        fsPath = fs::path(shaderFolder) / "blur.fs";
        blurShader = BlurShader(vsPath, fsPath);
    }
    void objectSetup()
    {
        // Get texture folder path
        string textureFolder, texturePath;
        this->get_parameter("texture_folder", textureFolder);

        // Overlay graphic
        vehicleOverlay = Texture(fs::path(textureFolder) / "overlays" / "talos_L_overlay.png");

        // Task objects
        texturePath = fs::path(textureFolder) / "task_objects" / "buoys.png";
        Object bouy(texturePath, 1.2f, 1.2f, glm::vec3(0, 4.0, -.85));
        texturePath = fs::path(textureFolder) / "task_objects" / "torpedoes.png";
        Object torpedo(texturePath, 1.2f, 1.2f, glm::vec3(2, 3.0, -.75), 0, 0, -30);
        objects.push_back(bouy);
        objects.push_back(torpedo);

        // Surrounding ground
        std::vector<Object> poolObjects = generatePoolObjects(textureFolder);
        for (Object poolObject : poolObjects)
            objects.push_back(poolObject);
    }
    // Go into an infinite loop until the tf transform becomes available
    void waitForTF()
    {
        bool tfFlag = true;
        while (rclcpp::ok() && tfFlag)
        {
            try
            {
                geometry_msgs::msg::TransformStamped t = tfBuffer->lookupTransform(
                    "test", "world",
                    tf2::TimePointZero);
                tfFlag = false;
            }
            catch (const tf2::TransformException &ex)
            {
            }
        }
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
        // Close window
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

        // Show april tag when T is pressed
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
    //===============================//
    //      UTILITY FUNCTIONS        //
    //===============================//

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
    FrameShader frameShader;
    BlurShader blurShader;
    Camera vehicleCamera = Camera();
    Camera flyAroundCamera = Camera();
    CameraMode cameraMode = FLY_ARROUND_MODE;
    Texture vehicleOverlay;

    std::unique_ptr<tf2_ros::Buffer> tfBuffer;
    std::shared_ptr<tf2_ros::TransformListener> tfListener;
    std::vector<Object> objects;
    GLFWwindow *window;
    FBO render1FBO;
    FBO render2FBO;
    FBO screenFBO;
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
    rclcpp::shutdown();
    return 1;
}