//===============================//
/*      NOTES/ASSUMPTIONS        //
//===============================//

The graphics are made using OpenGL.
To understand how it works, please go through the Getting Started section of:
https://learnopengl.com/Getting-started/OpenGL
(Much of the code from this website was used and modified for this project)

1) Change settings in the "setting.h" file in the include directory
2) Add new objects and models or modify existing ones in the "scene_info.yaml"
3) There isn't a proper lighting system implemented (future project?)
4) If you want good frame rate, don't run in a virtual machine or WSL, do naitive linux boot

//===============================//
//           INCLUDES            */
//===============================//

// General Libraries
//--------------------------------------------
#include <chrono>
#include <fstream>
#include <iostream>
#include <filesystem>
#include <yaml-cpp/yaml.h>
// ROS Libraries
//---------------------------------------------
#include <tf2/exceptions.h>
#include <tf2_ros/buffer.h>
#include "rclcpp/rclcpp.hpp"
#include <sensor_msgs/msg/image.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include "tf2_ros/transform_listener.h"
#include <sensor_msgs/msg/camera_info.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
// OpenGL Libraries
//-------------------------------------------------
#define STB_IMAGE_IMPLEMENTATION
#include <shader.hpp>
#include <camera.hpp>
#include <object.hpp>
#include <glm/glm.hpp>
#include <glad/glad.h>
#include <stb_image.h>
#include <GLFW/glfw3.h>
#include <assimp/scene.h>
#include <blurShader.hpp>
#include <textShader.hpp>
#include <modelShader.hpp>
#include <frameShader.hpp>
#include <waterShader.hpp>
#include <objectShader.hpp>
#include <skyboxShader.hpp>
#include <causticsManager.hpp>
#include <assimp/Importer.hpp>
#include <assimp/postprocess.h>

using std::string, std::vector, std::cout, std::endl;
using namespace std::chrono_literals;
using glm::vec3;

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
        imagePub = this->create_publisher<sensor_msgs::msg::Image>("zed/zed_node/left/image_rect_color", 10);
        depthPub = this->create_publisher<sensor_msgs::msg::Image>("zed/zed_node/depth/depth_registered", 10);
        depthInfoPub = this->create_publisher<sensor_msgs::msg::CameraInfo>("zed/zed_node/depth/camera_info", 10);
        cameraInfoPub = this->create_publisher<sensor_msgs::msg::CameraInfo>("zed/zed_node/left/camera_info", 10);

        // Create timers
        std::chrono::duration<double> imgPubTime(1.0 / FRAME_RATE);
        imgPubTimer = this->create_wall_timer(imgPubTime, std::bind(&zedFakerNode::publishImages, this));

        // Go through setup
        folderSetup();
        openGlSetup();
        shaderSetup();
        sceneSetup();
        waitForTF();
        fboSetup();
    }

    //===============================//
    //        IMAGE CREATION         //
    //===============================//
    void fakeImages()
    {
        //  RENDER LOOP
        //----------------------------------------------------------
        while (rclcpp::ok() && !glfwWindowShouldClose(window))
        {
            // POLL UPDATES
            //------------------------------------------------------
            // Proccess any pending ROS2 callbacks or timers
            rclcpp::spin_some(shared_from_this());
            // Clear the screen
            clearBuffers();

            // Get the time between frames, used for camera movement
            double currentFrame = glfwGetTime();
            deltaTime = currentFrame - lastFrame;
            lastFrame = currentFrame;
            // Process mouse and keyboard inputs, updates
            processKeyboard(window);
            glfwPollEvents();
            updateRobot();

            // ROBOT'S VIEW
            //-------------------------------------------------------
            // The robot's view is rendered every loop regardless of what camera is being used to display onto window
            // This is because even if not being displayed, the images are still needed to publish fake ROS cameara images from the robot's POV
            robotFBO.use();
            objectShader.render(objects, robotCamera);
            modelShader.render(models, robotCamera);
            skyboxShader.render(robotCamera);
            waterShader.render(objects, modelsAndRobot, robotCamera, robotFBO);
            // Post processing effects
            frameShader.render(vehicleOverlay); // Vehicle overlay

            // FLY AROUND VIEW
            //-------------------------------------------------------
            // Only render if need to
            if (cameraMode == FLY_ARROUND_MODE)
            {
                flycamFBO.use();
                objectShader.render(objects, flyAroundCamera);
                modelShader.render(modelsAndRobot, flyAroundCamera);
                skyboxShader.render(flyAroundCamera);
                waterShader.render(objects, modelsAndRobot, flyAroundCamera, flycamFBO);
                //  Display F3 screen if active
                if (f3Screen)
                    displayF3Screen();
            }

            // Render desired view to the screen
            screenFBO.use();
            frameShader.render(cameraMode == ROBOT_MODE ? robotFBO : flycamFBO);

            glfwSwapBuffers(window);
        }
        // Cleanup
        glfwDestroyWindow(window);
        glfwTerminate();
    };

private:
    //===============================//
    //            SETUP              //
    //===============================//

    void fboSetup()
    {
        screenFBO = FBO(true);
        render1FBO = FBO(false); // Not used atm, but would be needed to do other post processing like blur or distortion
        render2FBO = FBO(false); // Not used atm, but would be needed to do other post processing like blur or distortion
        robotFBO = FBO(false);
        flycamFBO = FBO(false);
    }
    void folderSetup()
    {
        string modelFolder_, textureFolder_, fontFolder_, shaderFolder_;
        // Declare and get ROS parameters with folder locations
        this->declare_parameter("model_folder", "");
        this->get_parameter("model_folder", modelFolder_);
        this->declare_parameter("texture_folder", "");
        this->get_parameter("texture_folder", textureFolder_);
        this->declare_parameter("font_folder", "");
        this->get_parameter("font_folder", fontFolder_);
        this->declare_parameter("shader_folder", "");
        this->get_parameter("shader_folder", shaderFolder_);

        // Save folder locations as a file system variable
        modelFolder = std::filesystem::path(modelFolder_);
        textureFolder = std::filesystem::path(textureFolder_);
        fontFolder = std::filesystem::path(fontFolder_);
        shaderFolder = std::filesystem::path(shaderFolder_);
    }
    /**
     * @brief Creats all the objects and adds them into the scene
     * This is where the object size, position, orientation, texture, etc. is set
     * Loads in 3D robot model
     */
    void sceneSetup()
    {
        // ADDING 2D/3D OBJECTS
        //------------------------------------------------------
        // creating objects from "scene_info.yaml"
        string sceneFile;
        YAML::Node scene;
        this->declare_parameter("scene_config", "");
        this->get_parameter("scene_config", sceneFile);
        try
        {
            // Loading YAML file for parsing
            RCLCPP_INFO(this->get_logger(), "Opening scene YAML file: %s", sceneFile.c_str());
            scene = YAML::LoadFile(sceneFile);
            objectSetup(scene["objects"]);
            robotSetup(scene["robot"]);
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(node->get_logger(), "Failed to parse the config file: %s", e.what());
        }

        // APRIL TAG
        //------------------------------------------------------
        bool relativeToTag = scene["april_tag"]["relative_to_tag"].as<bool>();
        // Adding April tag (DON'T PUSH TO OBJECTS LIST)
        vec3 tagPosition = vec3(toDouble(scene["april_tag"]["pose"]["x"]), toDouble(scene["april_tag"]["pose"]["y"]), 0);
        float tagYaw = toDouble(scene["april_tag"]["pose"]["yaw"]);
        string tagPath = textureFolder / "objects" / "April Tag.jpg";
        if (relativeToTag)
        {
            aprilTag = Object(tagPath, 0.6096f, 0.9144, vec3(0, 0, -0.345), vec3(0, 0, 0));
            waterShader.applyTagOffset(tagPosition, tagYaw);
        }
        else
            aprilTag = Object(tagPath, 0.6096f, 0.9144, tagPosition + vec3(0, 0, -0.345), vec3(0, 0, tagYaw));

        // ADDING POOL ENVIROMENT
        //------------------------------------------------------
        vector<Object> poolObjects = generatePoolObjects(textureFolder);
        for (Object poolObject : poolObjects)
        {
            if (relativeToTag)
                poolObject.applyTagOffset(tagPosition, tagYaw);
            objects.push_back(poolObject);
        }
        // Making robot overlay graphic
        vehicleOverlay = Texture(textureFolder / "overlays" / "talos_L_overlay.png");
    }

    /**
     * @brief Unpacks scene YAML information and makes a 2D or 3D object and into the scene
     */
    void objectSetup(YAML::Node objectsInfo)
    {
        vector<geometry_msgs::msg::TransformStamped> transforms;
        // Go through each input in the objects section of YAML file
        for (const auto &objectInfoPair : objectsInfo)
        {
            YAML::Node objectInfo = objectInfoPair.second;

            vec3 position(toDouble(objectInfo["pose"]["x"]),
                          toDouble(objectInfo["pose"]["y"]),
                          toDouble(objectInfo["pose"]["z"]));
            vec3 rpy(toDouble(objectInfo["pose"]["roll"]),
                     toDouble(objectInfo["pose"]["pitch"]),
                     toDouble(objectInfo["pose"]["yaw"]));
            // Object is a 2D image
            if (objectInfo["image"].IsDefined())
            {
                // Unpack values from YAML into useful variables
                string imagePath = textureFolder / "objects" / objectInfo["image"]["file"].as<string>();
                double imgWidth = objectInfo["image"]["size"][0].as<double>();
                double imgHeight = objectInfo["image"]["size"][1].as<double>();
                // Use this information to create a Object to add to "list" that needs rendered
                objects.push_back(Object(imagePath,
                                         imgWidth,
                                         imgHeight,
                                         position,
                                         rpy));
            }
            // Object is a 3D model
            else if (objectInfo["model"].IsDefined())
            {
                // Unpack values from YAML into useful variables
                string modelPath = modelFolder / objectInfo["model"]["file"].as<string>();
                double units = parseUnits(objectInfo["model"]["units"]);
                vector<double> modelOffset(6, 0.0);
                if (objectInfo["model_offset"].IsDefined())
                    modelOffset = objectInfo["model_offset"].as<vector<double>>();
                vec3 offsetPos(modelOffset[0], modelOffset[1], modelOffset[2]);
                vec3 offsetRPY(modelOffset[3], modelOffset[4], modelOffset[5]);
                vec3 color(-1, -1, -1);
                if (objectInfo["model"]["color"].IsDefined())
                    color = vec3(objectInfo["model"]["color"][0].as<double>(),  // R
                                 objectInfo["model"]["color"][1].as<double>(),  // G
                                 objectInfo["model"]["color"][2].as<double>()); // B
                // Use this information to create a Model to add to "list" that needs rendered
                models.push_back(Model(modelPath,
                                       units,
                                       position,
                                       offsetPos,
                                       rpy,
                                       offsetRPY,
                                       color));
            }
            else
            {
                RCLCPP_ERROR(this->get_logger(), "Object %s did not contain 'image:' or 'model:'", objectInfoPair.first.as<string>().c_str());
                continue;
            }
            // Send a transform message with all the objects once complete
            transforms.push_back(getStaticTransformForObject(objectInfoPair.first.as<string>(), position, glm::radians(rpy)));
        }
        staticBroadcaster->sendTransform(transforms);
    }

    /**
     * @brief Unpacks scene YAML file to create a robot model
     */
    void robotSetup(YAML::Node robotInfo)
    {
        // Unpack values from YAML into useful variables
        string modelPath = modelFolder / robotInfo["model"]["file"].as<string>();
        double units = parseUnits(robotInfo["model"]["units"]);
        vector<double> modelOffset = robotInfo["model"]["model_offset"].as<vector<double>>();
        vec3 offsetPos(modelOffset[0], modelOffset[1], modelOffset[2]);
        vec3 offsetRPY(modelOffset[3], modelOffset[4], modelOffset[5]);
        // Get model color if it has one, negative values indicate it doesn't
        vec3 color(-1, -1, -1);
        if (robotInfo["model"]["color"].IsDefined())
            color = vec3(robotInfo["model"]["color"][0].as<double>(),  // R
                         robotInfo["model"]["color"][1].as<double>(),  // G
                         robotInfo["model"]["color"][2].as<double>()); // B
        // Use this information to create a Model robot object
        Model robot(modelPath, units, vec3(), offsetPos, vec3(), offsetRPY, color);
        modelsAndRobot = models;
        modelsAndRobot.push_back(robot);
    }

    /**
     * @brief Setups up OpenGL and related libraries; creats window
     * @return whether the setup was successful
     */
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
        setWindowIcon(window);
        // STBI image library setup
        stbi_set_flip_vertically_on_load(true);
        return true;
    }

    // Set window icon in task bar to UWRT logo
    void setWindowIcon(GLFWwindow *window)
    {
        string iconPath = textureFolder / "overlays" / "uwrt_icon.png";
        int width, height, channels;
        unsigned char *image = stbi_load(iconPath.c_str(), &width, &height, &channels, 4); // Force 4 channels (RGBA)
        if (image)
        {
            GLFWimage icons[1];
            icons[0].width = width;
            icons[0].height = height;
            icons[0].pixels = image;
            glfwSetWindowIcon(window, 1, icons);
            stbi_image_free(image);
        }
        else
            std::cerr << "Failed to load icon: " << iconPath << std::endl;
    }

    // Creates shaders objects from file locations
    void shaderSetup()
    {
        // Get file paths
        string waterPath = textureFolder / "water";
        string causticsPath = textureFolder / "water" / "caustics";

        // Create post processing shaders
        string vsPath = shaderFolder / "frame.vs";
        string fsPath = shaderFolder / "frame.fs";
        frameShader = FrameShader(vsPath, fsPath);

        // Loading the causings takes a lot of time to load the 200+ caustic textures xD
        // Use the newly created frameShader to display a loading screen for some eye candy
        Texture loadingScreen = Texture(textureFolder / "overlays" / "Loading_Screen.jpg");
        frameShader.render(loadingScreen);
        glfwSwapBuffers(window);
        CausticsManager::loadCaustics(causticsPath);

        // Create object shader
        vsPath = shaderFolder / "object.vs";
        fsPath = shaderFolder / "object.fs";
        objectShader = ObjectShader(vsPath, fsPath);

        // Create model shader
        vsPath = shaderFolder / "model.vs";
        fsPath = shaderFolder / "model.fs";
        modelShader = ModelShader(vsPath, fsPath);

        // Create blur shader
        vsPath = shaderFolder / "blur.vs";
        fsPath = shaderFolder / "blur.fs";
        blurShader = BlurShader(vsPath, fsPath);

        // Create text shader
        vsPath = shaderFolder / "text.vs";
        fsPath = shaderFolder / "text.fs";
        string fontPath = fontFolder / "Minecraft.ttf";
        textShader = TextShader(vsPath, fsPath, fontPath);

        // Create skybox shader
        vsPath = shaderFolder / "skybox.vs";
        fsPath = shaderFolder / "skybox.fs";                                          // Order needs to be:
        vector<string> faces = {textureFolder / "skybox" / "Daylight Box_Right.jpg",  // Needs to be right
                                textureFolder / "skybox" / "Daylight Box_Left.jpg",   // Needs to be left
                                textureFolder / "skybox" / "Daylight Box_Bottom.jpg", // Needs to be bottom
                                textureFolder / "skybox" / "Daylight Box_Top.jpg",    // Needs to be top
                                textureFolder / "skybox" / "Daylight Box_Front.jpg",  // Needs to be front
                                textureFolder / "skybox" / "Daylight Box_Back.jpg"};  // Needs to be back
        skyboxShader = SkyboxShader(vsPath, fsPath, faces);

        // Create water shader
        vsPath = shaderFolder / "water.vs";
        fsPath = shaderFolder / "water.fs";
        waterShader = WaterShader(vsPath, fsPath, waterPath, objectShader, modelShader, skyboxShader);
    }

    // Go into an infinite loop until the TF transform becomes available
    void waitForTF()
    {
        // Next up is waiting for TF frames so display another loading screen
        Texture loadingScreen(textureFolder / "overlays" / "Loading_Screen2.jpg");
        frameShader.render(loadingScreen);
        glfwSwapBuffers(window);

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
    //     RENDER FUNCTIONS          //
    //===============================//

    // clears all frame buffers with the sky color
    void clearBuffers()
    {
        screenFBO.clear();
        render1FBO.clear();
        render2FBO.clear();
        robotFBO.clear();
        flycamFBO.clear();
    }

    void updateRobot()
    {
        // ROBOT CAMERA UPDATE
        //---------------------------------------------------------------------------
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
            tf2::Matrix3x3 rotM(q);             // Converting quaternioin to yaw pitch roll
            double roll, pitch, yaw;            // ^
            rotM.getEulerYPR(yaw, pitch, roll); // ^
            robotCamera.setYPR(yaw, pitch, roll);
        }
        catch (const tf2::TransformException &ex)
        {
            glfwSetWindowShouldClose(window, true);
        }

        // ROBOT MODEL UPDATE
        //----------------------------------------------------------------------
        try
        {
            // Get camera TF frame
            string targetFrame = "simulator" + robotName + "/base_link";
            geometry_msgs::msg::TransformStamped t = tfBuffer->lookupTransform(
                "world", targetFrame,
                tf2::TimePointZero);

            // Set camera position
            modelsAndRobot.back().setPosition(t.transform.translation.x,
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
            modelsAndRobot.back().setRPY(glm::degrees(roll), glm::degrees(pitch), glm::degrees(yaw));
        }
        catch (const tf2::TransformException &ex)
        {
            cout << "ROBOT TRANSFORM FAILED" << endl;
            glfwSetWindowShouldClose(window, true);
        }
    }

    // Draws f3 screen text onto screen (only shown in fly around mode)
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
    //      CALLBACK FUNCTIONS       //
    //===============================//

    // Copies raw OpenGL image data into ROS message and publishes
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
        vector<uint8_t> imgData;
        vector<uint8_t> depthData;
        robotFBO.copyColorData(imgData);
        robotFBO.copyDepthData(depthData);
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

        // Show april tag when T is pressed (toggle)
        if (isPressed(GLFW_KEY_T) && !aprilFlag)
        {
            aprilFlag = true;
            if (aprilFlag2)
                objects.push_back(aprilTag);
            else
                objects.pop_back();
            aprilFlag2 = !aprilFlag2;
        }
        else if (!isPressed(GLFW_KEY_T))
            aprilFlag = false;

        // Toggle camera mode
        if ((isPressed(GLFW_KEY_TAB) || isPressed(GLFW_KEY_F5)) && !cameraModeFlag)
        {
            cameraModeFlag = true;
            cameraMode = (cameraMode == ROBOT_MODE) ? FLY_ARROUND_MODE : ROBOT_MODE;
        }
        else if (!(isPressed(GLFW_KEY_TAB) || isPressed(GLFW_KEY_F5)))
            cameraModeFlag = false;

        // Toggle F3 menu
        if (isPressed(GLFW_KEY_F3) && !f3ScreenFlag)
        {
            f3ScreenFlag = true;
            f3Screen = !f3Screen;
        }
        else if (!isPressed(GLFW_KEY_F3))
            f3ScreenFlag = false;
    }
    static void resizeWindow(GLFWwindow *window, int width, int height)
    {
        // Goodbye compiler warnings
        (void)window;
        (void)width;
        (void)height;
        // Someone should actually make this work lol, I can't do better because I am pretty sure it's a WSL bug
        glViewport(0, 0, IMG_WIDTH, IMG_HEIGHT);
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
        (void)window; // Goodbye compiler warnings
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

    // Safely convert a YAML parameter to a double ex. toDouble(robotInfo["pose"]["x"])
    double toDouble(YAML::Node node)
    {
        try
        {
            return node.as<double>();
        }
        catch (const std::exception &e)
        {
            return 0.0;
        }
    }

    double parseUnits(YAML::Node units)
    {
        // See if the user inputted a number
        try
        {
            return units.as<double>();
        }
        catch (const YAML::BadConversion &)
        {
        }
        // Convert string to Units enum
        auto it = stringToUnits.find(units.as<std::string>());
        if (it != stringToUnits.end())
            return unitsToDouble.at(it->second);
        // Log an error if an invalid string is provided and return the default unit (METER)
        RCLCPP_INFO(this->get_logger(), "Invalid string for model units: %s", units.as<string>().c_str());
        return unitsToDouble.at(METER);
    }

    geometry_msgs::msg::TransformStamped getStaticTransformForObject(const std::string &name, glm::vec3 position, glm::vec3 rpy)
    {
        geometry_msgs::msg::TransformStamped static_transform;
        static_transform.header.frame_id = "world";
        static_transform.child_frame_id = "simulator/" + name;

        // Get object position
        static_transform.transform.translation.x = position.x;
        static_transform.transform.translation.y = position.y;
        static_transform.transform.translation.z = position.z;

        // Get object orientation
        tf2::Quaternion tf2Quat;
        tf2Quat.setRPY(rpy.x, rpy.y, rpy.z);
        tf2Quat.normalize();
        static_transform.transform.rotation = tf2::toMsg(tf2Quat);
        return static_transform;
    }
    //===============================//
    //          VARIABLES            //
    //===============================//
    bool f3Screen = false;
    float lastFrame = 0.0;
    double deltaTime = 0.0;
    bool aprilFlag = false;
    bool aprilFlag2 = true;
    bool f3ScreenFlag = false;
    bool cameraModeFlag = false;
    float lastX = IMG_WIDTH / 2.0;
    float lastY = IMG_HEIGHT / 2.0;
    Camera robotCamera = Camera();
    Camera flyAroundCamera = Camera();
    CameraMode cameraMode = FLY_ARROUND_MODE;

    FBO robotFBO;
    FBO flycamFBO;
    FBO screenFBO;
    FBO render1FBO;
    FBO render2FBO;
    Object aprilTag;
    string robotName;
    GLFWwindow *window;
    vector<Model> models;
    vector<Model> modelsAndRobot;
    TextShader textShader;
    BlurShader blurShader;
    vector<Object> objects;
    Texture vehicleOverlay;
    WaterShader waterShader;
    FrameShader frameShader;
    ModelShader modelShader;
    SkyboxShader skyboxShader;
    ObjectShader objectShader;
    std::filesystem::path fontFolder;
    std::filesystem::path modelFolder;
    std::filesystem::path shaderFolder;
    std::filesystem::path textureFolder;
    rclcpp::TimerBase::SharedPtr imgPubTimer;
    std::unique_ptr<tf2_ros::Buffer> tfBuffer;
    std::shared_ptr<tf2_ros::TransformListener> tfListener;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tfBroadcaster;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr depthPub;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr imagePub;
    std::unique_ptr<tf2_ros::StaticTransformBroadcaster> staticBroadcaster;
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