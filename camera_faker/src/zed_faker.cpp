//===============================//
/*     TABLE OF CONTENTS         //
//===============================//

//===============================//
//      NOTES/ASSUMPTIONS        //
//===============================//
1) Change settings in the "setting.h" file in the include directory
2) If you want good frame rate, don't run in a virtual machine or WSL, do naitive linux boot
3) The graphics are rendered using OpenGL, to understand how it works, I'd highly reccommend going through the getting started section of https://learnopengl.com/Getting-started/OpenGL

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
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <frameShader.hpp>
#include <blurShader.hpp>
#include <assimp/Importer.hpp>
#include <assimp/scene.h>
#include <assimp/postprocess.h>
#include <modelShader.hpp>
#include <textShader.hpp>

using std::string, std::vector, std::cout, std::endl;
using namespace std::chrono_literals;
namespace fs = std::filesystem;

// This initialization is needed for callback functions
class zedFakerNode;
std::shared_ptr<zedFakerNode> node;
class zedFakerNode : public rclcpp::Node
{
public:
    zedFakerNode() : Node("zed_faker")
    {
        robotName = this->get_namespace();

        // Create a TF broadcaster
        tfBuffer = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tfListener = std::make_shared<tf2_ros::TransformListener>(*tfBuffer);
        staticBroadcaster = std::make_unique<tf2_ros::StaticTransformBroadcaster>(this);

        // Create camera info and image publishers
        depthPub = this->create_publisher<sensor_msgs::msg::Image>("zed/zed_node/depth/depth_registered", 10);
        cameraInfoPub = this->create_publisher<sensor_msgs::msg::CameraInfo>("zed/zed_node/left/camera_info", 10);
        imagePub = this->create_publisher<sensor_msgs::msg::Image>("zed/zed_node/left/image_rect_color", 10);
        depthInfoPub = this->create_publisher<sensor_msgs::msg::CameraInfo>("zed/zed_node/depth/camera_info", 10);

        // Create timers
        std::chrono::duration<double> imgPubTime(1.0 / FRAME_RATE);
        imgPubTimer = this->create_wall_timer(imgPubTime, std::bind(&zedFakerNode::publishImages, this));

        // Setup OpenGL, shaders, and objects
        openGlSetup();
        shaderSetup();
        objectSetup();
        // Set up FBO's (Things that can be rendered to)
        screenFBO = FBO(true);
        render1FBO = FBO(false);
        render2FBO = FBO(false);
        robotFinalFBO = FBO(false);
        flycamFinalFBO = FBO(false);
    }

    //===============================//
    //        IMAGE CREATION         //
    //===============================//
    void fakeImages()
    {
        waitForTF();
        while (rclcpp::ok() && !glfwWindowShouldClose(window))
        {
            // Starting things
            clearBuffers();

            // Process mouse and keyboard inputs
            processKeyboard(window);
            glfwPollEvents();

            // Get the time between frames, used for camera movement
            double currentFrame = glfwGetTime();
            deltaTime = currentFrame - lastFrame;
            lastFrame = currentFrame;

            // Update positions
            updateRobotCamera();
            updateRobotModel();

            // ROBOT'S VIEW
            //-------------------------------------------------------
            // The robot's view is rendered every loop regardless of what camera is being used to display onto window
            // This is because even if not being displayed, the images are still needed to publish fake ROS cameara images from the robot's POV
            robotFinalFBO.use();
            objectShader.render(objects, robotCamera);
            waterShader.render(objects, robot, robotCamera, render1FBO);
            // Post processing effects
            frameShader.render(vehicleOverlay); // Vehicle overlay

            // FLY AROUND VIEW
            //-------------------------------------------------------
            // Only render if need to
            if (cameraMode == FLY_ARROUND_MODE)
            {
                flycamFinalFBO.use();
                modelShader.render(robot, flyAroundCamera);
                objectShader.render(objects, flyAroundCamera);
                waterShader.render(objects, robot, flyAroundCamera, flycamFinalFBO);
                // Display F3 screen if active
                if (f3Screen)
                {
                    displayF3Screen();
                }
            }

            // Render desired view to the screen
            screenFBO.use();
            if (cameraMode == ROBOT_MODE)
                frameShader.render(robotFinalFBO);
            else
                frameShader.render(flycamFinalFBO);
            glfwSwapBuffers(window);

            // Proccess any pending ROS2 callbacks or timers
            rclcpp::spin_some(shared_from_this());
        }
        // Cleanup
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
        robotFinalFBO.clear();
        flycamFinalFBO.clear();
    }

    // Updates the robot's camera to match pose from new TF messages
    void updateRobotCamera()
    {
        try
        {
            // Get camera TF frame
            string targetFrame = "simulator" + robotName + "/zed2i/left_optical";
            geometry_msgs::msg::TransformStamped t = tfBuffer->lookupTransform(
                "world", targetFrame,
                tf2::TimePointZero);

            // Set camera position
            robotCamera.setPosition(t.transform.translation.x,
                                    t.transform.translation.y,
                                    t.transform.translation.z);
            // Set camera orientation
            tf2::Quaternion q(t.transform.rotation.x,
                              t.transform.rotation.y,
                              t.transform.rotation.z,
                              t.transform.rotation.w);
            tf2::Matrix3x3 rotM(q);
            double roll, pitch, yaw;
            rotM.getEulerYPR(yaw, pitch, roll);
            robotCamera.setYPR(yaw, pitch, roll);
        }
        catch (const tf2::TransformException &ex)
        {
            glfwSetWindowShouldClose(window, true);
        }
    }

    void updateRobotModel()
    {
        try
        {
            // Get camera TF frame
            string targetFrame = "simulator" + robotName + "/base_link";
            geometry_msgs::msg::TransformStamped t = tfBuffer->lookupTransform(
                "world", targetFrame,
                tf2::TimePointZero);

            // Set camera position
            robot.setPosition(t.transform.translation.x,
                              t.transform.translation.y,
                              t.transform.translation.z);
            // Set camera orientation
            tf2::Quaternion q(t.transform.rotation.x,
                              t.transform.rotation.y,
                              t.transform.rotation.z,
                              t.transform.rotation.w);
            tf2::Matrix3x3 rotM(q);
            double roll, pitch, yaw;
            rotM.getRPY(roll, pitch, yaw);
            robot.setRPY(roll, pitch, yaw);
        }
        catch (const tf2::TransformException &ex)
        {
            cout << "ROBOT TRANSFORM FAILED" << endl;
            glfwSetWindowShouldClose(window, true);
        }
    }

    // Draws f3 screen text onto screen
    void displayF3Screen()
    {
        string f3Text;
        int margin = MARGIN;
        int yPos = IMG_HEIGHT - TEXT_HEIGHT - MARGIN;
        // FPS Couner
        f3Text = "FPS: " + std::to_string((int)(1 / deltaTime));
        textShader.render(f3Text, margin, yPos);
        yPos -= LINE_SPACING * TEXT_HEIGHT;
        // Camera positions
        f3Text = "flyCamera XYZ: " + std::to_string(flyAroundCamera.getPosition().x) + " / " + std::to_string(flyAroundCamera.getPosition().y) + " / " + std::to_string(flyAroundCamera.getPosition().z);
        textShader.render(f3Text, margin, yPos);
        yPos -= LINE_SPACING * TEXT_HEIGHT;
        f3Text = "talosCamera XYZ: " + std::to_string(robotCamera.getPosition().x) + " / " + std::to_string(robotCamera.getPosition().y) + " / " + std::to_string(robotCamera.getPosition().z);
        textShader.render(f3Text, margin, yPos);
        yPos -= LINE_SPACING * TEXT_HEIGHT;
        // Biome :P
        f3Text = "Biome: minecraft:woollett_aquatics_center";
        textShader.render(f3Text, margin, yPos);
        yPos -= LINE_SPACING * TEXT_HEIGHT;
        // ROS time
        f3Text = "ROS Time: " + std::to_string(this->get_clock()->now().seconds());
        textShader.render(f3Text, margin, yPos);
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
        window = glfwCreateWindow(IMG_WIDTH, IMG_HEIGHT, "Zed Faker", NULL, NULL);
        if (window == NULL)
        {
            // Window failed to be created
            std::cout << "Failed to create GLFW window" << std::endl;
            glfwTerminate();
            return false;
        }

        // Setup window and callback functions
        glfwMakeContextCurrent(window);
        glfwSetCursorPosCallback(window, mouseCallback);
        glfwSetFramebufferSizeCallback(window, resizeWindow);

        // VSync on (only swaps pixels onto screen every monitor refresh, not faster)
        glfwSwapInterval(1);

        // Get GLAD library function pointers
        if (!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress))
        {
            std::cout << "Failed to initialize GLAD" << std::endl;
            return false;
        }
        // STBI image library setup
        stbi_set_flip_vertically_on_load(true);

        int width, height;
        glfwGetWindowSize(window, &width, &height);
        glfwSetWindowAspectRatio(window, IMG_WIDTH, IMG_HEIGHT);

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

        // Create model shader
        vsPath = fs::path(shaderFolder) / "model.vs";
        fsPath = fs::path(shaderFolder) / "model.fs";
        modelShader = ModelShader(vsPath, fsPath);

        // Create water shader
        vsPath = fs::path(shaderFolder) / "water.vs";
        fsPath = fs::path(shaderFolder) / "water.fs";
        waterShader = WaterShader(vsPath, fsPath, waterPath, objectShader, modelShader);

        // Create blur shader
        vsPath = fs::path(shaderFolder) / "blur.vs";
        fsPath = fs::path(shaderFolder) / "blur.fs";
        blurShader = BlurShader(vsPath, fsPath);

        // Create text shader
        string fontFolder, fontPath;
        this->declare_parameter("font_folder", "");
        this->get_parameter("font_folder", fontFolder);
        vsPath = fs::path(shaderFolder) / "text.vs";
        fsPath = fs::path(shaderFolder) / "text.fs";
        fontPath = fs::path(fontFolder) / "Minecraft.ttf";
        textShader = TextShader(vsPath, fsPath, fontPath);

        // Next up is waiting for TF frames so display another loading screen in this function while all the path variables are here
        loadingScreen = Texture(fs::path(textureFolder) / "overlays" / "Loading_Screen2.jpg");
        frameShader.render(loadingScreen);
        glfwSwapBuffers(window);
    }
    geometry_msgs::msg::TransformStamped getStaticTransformForObject(const std::string &name, const Object &object)
    {
        geometry_msgs::msg::TransformStamped static_transform;
        static_transform.header.frame_id = "world";
        static_transform.child_frame_id = "simulator/" + name;

        glm::vec3 xyz = object.getPositionXYZ();
        static_transform.transform.translation.x = xyz.x;
        static_transform.transform.translation.y = xyz.y;
        static_transform.transform.translation.z = xyz.z;

        glm::vec3 rpy = object.getOrientationRPY();
        tf2::Quaternion tf2Quat;
        tf2Quat.setRPY(rpy.x, rpy.y, rpy.z);
        tf2Quat.normalize();
        static_transform.transform.rotation = tf2::toMsg(tf2Quat);
        return static_transform;
    }
    void objectSetup()
    {
        // PLANE OBJECTS
        //-------------------------------------------------------
        // Get texture folder path
        string textureFolder, texturePath;
        this->get_parameter("texture_folder", textureFolder);

        // Overlay graphic
        vehicleOverlay = Texture(fs::path(textureFolder) / "overlays" / "talos_L_overlay.png");

        // Task objects
        texturePath = fs::path(textureFolder) / "task_objects" / "buoys.png";
        Object buoy(texturePath, 1.2f, 1.2f, glm::vec3(7.84, 10.37, -1.0), 0, 0, 260 + 90);
        texturePath = fs::path(textureFolder) / "task_objects" / "torpedoes.png";
        Object torpedo(texturePath, 1.2f, 1.2f, glm::vec3(2.0, -3.0, -1.0), 0, 0, 90 + 90);
        texturePath = fs::path(textureFolder) / "task_objects" / "gate.png";
        Object gate(texturePath, 3.124f, 1.6f, glm::vec3(3.89, 6.49, -1.3), 0, 0, 260 + 90);
        objects.push_back(buoy);
        objects.push_back(torpedo);
        objects.push_back(gate);

        // before adding environment objects to the vector, publish the objects we care about as static transforms
        // this will help with visualization of the ground-truth positions in RViz as well as provide infrastructure
        // for automated testing of the system
        std::vector<geometry_msgs::msg::TransformStamped> transforms;
        transforms.push_back(getStaticTransformForObject("buoy", buoy));
        transforms.push_back(getStaticTransformForObject("torpedo", torpedo));
        transforms.push_back(getStaticTransformForObject("gate", gate));
        staticBroadcaster->sendTransform(transforms);

        // Surrounding ground
        std::vector<Object> poolObjects = generatePoolObjects(textureFolder);
        for (Object poolObject : poolObjects)
            objects.push_back(poolObject);

        // 3D MODELS
        //------------------------------
        string modelFolder, modelPath;
        this->declare_parameter("model_folder", "");
        this->get_parameter("model_folder", modelFolder);
        modelPath = fs::path(modelFolder) / "talos.obj";
        robot = Model(modelPath, METER, glm::vec3(2, 0, -.5));
    }
    // Go into an infinite loop until the tf transform becomes available
    void waitForTF()
    {
        bool tfFlag = true;
        while (rclcpp::ok() && tfFlag)
        {
            try
            {
                string targetFrame = "simulator" + robotName + "/zed2i/left_optical";
                geometry_msgs::msg::TransformStamped t = tfBuffer->lookupTransform(
                    targetFrame, "world",
                    tf2::TimePointZero);
                tfFlag = false;
            }
            catch (const tf2::TransformException &ex)
            {
                // cout << ex.what() << endl;
            }
        }
    }
    //===============================//
    //      CALLBACK FUNCTIONS       //
    //===============================//
    void publishImages()
    {
        sensor_msgs::msg::Image imgMsg, depthMsg;

        // Set up image message info
        string robotPrefix = (robotName[0] == '/' ? robotName.substr(1) : robotName);
        imgMsg.header.frame_id = robotPrefix + "/zed_left_camera_optical_frame";
        imgMsg.header.stamp = this->get_clock()->now();
        imgMsg.height = IMG_HEIGHT;
        imgMsg.width = IMG_WIDTH;
        imgMsg.encoding = "rgb8";
        imgMsg.step = 3 * IMG_WIDTH;
        imgMsg.is_bigendian = false;
        // Set up depth message info
        depthMsg.height = IMG_HEIGHT;
        depthMsg.width = IMG_WIDTH;
        depthMsg.encoding = "32FC1";
        depthMsg.step = sizeof(float) * IMG_WIDTH;
        depthMsg.is_bigendian = false;
        // Copy image data into message
        std::vector<uint8_t> imgData;
        std::vector<uint8_t> depthData;
        robotFinalFBO.copyColorData(imgData);
        robotFinalFBO.copyDepthData(depthData);
        imgMsg.data = imgData;
        depthMsg.data = depthData;

        // Publish messages
        imagePub->publish(imgMsg);
        depthPub->publish(depthMsg);

        // Fill in camera info messages
        sensor_msgs::msg::CameraInfo camInfo, depthInfo;
        camInfo.k[0] = CAMERA_FX; // Making the camera's 3x3 intrinsic matrix
        camInfo.k[2] = CAMERA_CX; // [fx 0  cx
        camInfo.k[4] = CAMERA_FY; //  0  fy cy
        camInfo.k[5] = CAMERA_CY; //  0  0  1 ]
        camInfo.k[8] = 1.0;
        depthInfo = camInfo;

        // Publish camera info messages
        cameraInfoPub->publish(camInfo);
        depthInfoPub->publish(depthInfo);
    }

    // Handles keyboard input
    void processKeyboard(GLFWwindow *window)
    {
        // Close window
        if (isPressed(GLFW_KEY_ESCAPE))
            glfwSetWindowShouldClose(window, true);

        // Handle camera movements
        if (cameraMode == FLY_ARROUND_MODE)
        {
            // WASD + Space/LShift movement
            if (isPressed(GLFW_KEY_W))
                flyAroundCamera.ProcessKeyboard(FORWARD, deltaTime);
            if (isPressed(GLFW_KEY_S))
                flyAroundCamera.ProcessKeyboard(BACKWARD, deltaTime);
            if (isPressed(GLFW_KEY_A))
                flyAroundCamera.ProcessKeyboard(LEFT, deltaTime);
            if (isPressed(GLFW_KEY_D))
                flyAroundCamera.ProcessKeyboard(RIGHT, deltaTime);
            if (isPressed(GLFW_KEY_SPACE))
                flyAroundCamera.ProcessKeyboard(UP, deltaTime);
            if (isPressed(GLFW_KEY_LEFT_SHIFT))
                flyAroundCamera.ProcessKeyboard(DOWN, deltaTime);
        }

        // Show april tag when T is pressed
        if (isPressed(GLFW_KEY_T))
            int a = 1; // TODO

        // Toggle camera mode
        if ((isPressed(GLFW_KEY_TAB) || isPressed(GLFW_KEY_F4)) && !cameraModeFlag)
        {
            cameraModeFlag = true;
            if (cameraMode == ROBOT_MODE)
                cameraMode = FLY_ARROUND_MODE;
            else
                cameraMode = ROBOT_MODE;
        }
        else
            cameraModeFlag = false;

        // Toggle F3 menu
        if (isPressed(GLFW_KEY_F3) && !f3ScreenFlag)
        {
            f3ScreenFlag = true;
            f3Screen = !f3Screen;
        }
        else
            f3ScreenFlag = false;
    }
    static void resizeWindow(GLFWwindow *window, int width, int height)
    {
        glViewport(0, 0, width, height);
    }

    // Handles fly around camera movement, called from mouse callback
    void moveCamera(double xpos, double ypos)
    {
        // The user can't use the mouse to control the robot's POV
        if (cameraMode == ROBOT_MODE)
            return;

        // Figure out how much the mouse moved
        float xoffset = lastX - xpos;
        float yoffset = lastY - ypos;
        lastX = xpos;
        lastY = ypos;

        // Update where the camera is looking
        flyAroundCamera.ProcessMouseMovement(xoffset, yoffset);
    }
    // Handles mouse movement
    static void mouseCallback(GLFWwindow *window, double xpos, double ypos)
    {
        // Need to do this goofy thing because callback function can't be an object specific function
        node->moveCamera(xpos, ypos);
    }
    //===============================//
    //      UTILITY FUNCTIONS        //
    //===============================//

    bool isPressed(int GLFW_KEY_XX)
    {
        return glfwGetKey(window, GLFW_KEY_XX) == GLFW_PRESS;
    }

    //===============================//
    //          VARIABLES            //
    //===============================//
    bool f3Screen = false;
    float lastFrame = 0.0;
    double deltaTime = 0.0;
    bool f3ScreenFlag = false;
    bool cameraModeFlag = false;
    float lastX = IMG_WIDTH / 2.0;
    float lastY = IMG_HEIGHT / 2.0;
    Camera robotCamera = Camera();
    Camera flyAroundCamera = Camera();
    CameraMode cameraMode = FLY_ARROUND_MODE;

    Model robot;
    FBO screenFBO;
    FBO render1FBO;
    FBO render2FBO;
    string robotName;
    FBO robotFinalFBO;
    FBO flycamFinalFBO;
    GLFWwindow *window;
    TextShader textShader;
    BlurShader blurShader;
    vector<Object> objects;
    Texture vehicleOverlay;
    WaterShader waterShader;
    FrameShader frameShader;
    ModelShader modelShader;
    ObjectShader objectShader;
    rclcpp::TimerBase::SharedPtr imgPubTimer;
    std::unique_ptr<tf2_ros::Buffer> tfBuffer;
    std::shared_ptr<tf2_ros::TransformListener> tfListener;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tfBroadcaster;
    std::unique_ptr<tf2_ros::StaticTransformBroadcaster> staticBroadcaster;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr depthPub;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr imagePub;
    rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr cameraInfoPub;
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