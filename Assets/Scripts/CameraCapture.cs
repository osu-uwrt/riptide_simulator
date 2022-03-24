using UnityEngine;  
using UnityEngine.Rendering;
using System.Collections;
using System;
using Unity.Robotics.ROSTCPConnector;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;
using RosMessageTypes.Std;
using RosMessageTypes.Sensor;
using RosMessageTypes.BuiltinInterfaces;

public class CameraCapture : MonoBehaviour {

    // Public vars.
    [Tooltip("Image dimensions.")]
    public int imageWidth = 1280, imageHeight = 720;
    [Tooltip("ID for images.")]
    public string frameId = "odom";
    [Tooltip("Camera info topic to publish to.")]
    public string infoTopic = "/puddles/stereo/right/camera_info";
    [Tooltip("Raw image topic to publish to.")]
    public string imageTopic = "/puddles/stereo/right/image_raw";
    [Tooltip("Time between sending image messages.")]
    public float publishMessageFrequency = 1/24f;

    // Private vars.
    private ROSConnection ros; // ROS connection for publishing data.
    private TimeMsg timeMsg; // http://docs.ros.org/en/melodic/api/std_msgs/html/msg/Time.html
    private HeaderMsg headerMsg; // http://docs.ros.org/en/lunar/api/std_msgs/html/msg/Header.html
    private ImageMsg rawImageMsg; // http://docs.ros.org/en/noetic/api/sensor_msgs/html/msg/Image.html
    private CameraInfoMsg cameraInfoMsg; // http://docs.ros.org/en/melodic/api/sensor_msgs/html/msg/CameraInfo.html
    private Camera mainCamera; // In-game camera object.
    private Rect rect;
    private RenderTexture renderTexture;
    private RenderTexture destinationRenderTexture;
    private Texture2D imageTexture;
    private byte[] imageBytes; // Array to store raw image data.
    private bool flipAcrossX; // Determines if the image should be flipped about the x-axis.

    void Start() {
        // Initialize variables.
        var graphicDevice = SystemInfo.graphicsDeviceType; // Get the utilized graphics API
        flipAcrossX = graphicDevice == GraphicsDeviceType.OpenGLCore || graphicDevice == GraphicsDeviceType.OpenGLES2 ||
        graphicDevice == GraphicsDeviceType.OpenGLES3 || graphicDevice == GraphicsDeviceType.Vulkan ? true : false;
        mainCamera = GetComponent<Camera>();
        rect = new Rect(0, 0, imageWidth, imageHeight); 
        renderTexture = new RenderTexture(imageWidth, imageHeight, 24);
        destinationRenderTexture = new RenderTexture(imageWidth, imageHeight, 24);
        renderTexture.Create();
        destinationRenderTexture.Create();
        imageTexture = new Texture2D(imageWidth, imageHeight, TextureFormat.RGB24, false);
        imageBytes = new byte[imageWidth * imageHeight * 3];

        mainCamera.targetTexture = renderTexture; // Tell mainCamera to render to renderTexture.

        // Create ROS connection and create publishers.
        ros = ROSConnection.GetOrCreateInstance();
        ros.RegisterPublisher<CameraInfoMsg>(infoTopic);
        ros.RegisterPublisher<ImageMsg>(imageTopic);

        // Create necessary messages for sending images.
        timeMsg = new TimeMsg(System.DateTime.Now.Second, (uint)System.DateTime.Now.Millisecond);
        headerMsg = new HeaderMsg(timeMsg, frameId);
        rawImageMsg = new ImageMsg(headerMsg, (uint)imageHeight, (uint)imageWidth, "rgb8", Convert.ToByte(false), (uint)imageWidth * (uint)3, null);
    }

    void OnEnable() {
        StartCoroutine(cameraCaptureCoroutine());
    }

    void OnDisable() {
        StopCoroutine(cameraCaptureCoroutine());
    }

    IEnumerator cameraCaptureCoroutine() {
        while (true) {
            // We should only read the screen buffer after rendering is complete.
            yield return new WaitForEndOfFrame(); 

            // Create mainCamera camera info message.
            cameraInfoMsg = CameraInfoGenerator.ConstructCameraInfoMessage(mainCamera, headerMsg, 0.0f, 0.01f);

            /* Capture mainCamera image.
               TODO: Figure out a more efficent way to capture and transfer images.
               Look into Graphics.Blit and Shaders (compute shaders, image effect shaders, etc.).
            */
            RenderTexture oldRenderTexture = RenderTexture.active;
            RenderTexture.active = renderTexture;
            mainCamera.Render();
            if (flipAcrossX) {
                Graphics.Blit(renderTexture, destinationRenderTexture, new Vector2(1.0f, -1.0f), new Vector2(0.0f, 1.0f));
                Graphics.Blit(destinationRenderTexture, renderTexture);
            }
            imageTexture.ReadPixels(rect, 0, 0);
            imageTexture.Apply();
            RenderTexture.active = oldRenderTexture;
            imageBytes = imageTexture.GetRawTextureData();

            // Update header message with proper time stamp and sequence ID.
            timeMsg.sec = System.DateTime.Now.Second;
            timeMsg.nanosec = (uint)System.DateTime.Now.Millisecond;
            headerMsg.stamp = timeMsg;
            //headerMsg.seq++;  

            // Update raw image message with updated header and data.
            rawImageMsg.data = imageBytes;
            rawImageMsg.header = headerMsg;

            // Publish messages.
            ros.Publish(imageTopic, rawImageMsg);
            ros.Publish(infoTopic, cameraInfoMsg);

            yield return new WaitForSeconds(publishMessageFrequency);
        }
    }
}