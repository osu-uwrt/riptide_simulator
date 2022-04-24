using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

using Unity.Robotics.ROSTCPConnector;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;
using RosMessageTypes.Sensor;

[RequireComponent(typeof(Camera))]
[RequireComponent(typeof(FRJ.Sensor.RGBCamera))]
public class RGBCameraPublisher : MonoBehaviour
{

  [SerializeField] private string _topicName = "image";
  [SerializeField] private string _frameId   = "camera";

  [SerializeField] private string _infoName = "left";
  [SerializeField] private float _baseline = 0.0f;

  private readonly static DateTime UNIX_EPOCH = new DateTime(1970, 1, 1, 0, 0, 0, 0, DateTimeKind.Utc);

  private string _camInfoTopic;
  private float _timeElapsed = 0f;
  private float _timeStamp   = 0f;

  private ROSConnection _ros;
  private ImageMsg _message;
  private CameraInfoMsg _camInfomessage;

  private Camera _unityCam;

  private FRJ.Sensor.RGBCamera _camera;
    
  void Start()
  {
    // Get Rotate Lidar
    this._camera = GetComponent<FRJ.Sensor.RGBCamera>();
    this._camera.Init();
    this._unityCam = GetComponent<Camera>();

    // setup ROS
    this._ros = ROSConnection.instance;
    this._camInfoTopic = "/tempest/stereo/" + _infoName + "/camera_info";
    this._ros.RegisterPublisher<ImageMsg>(this._topicName);
    this._ros.RegisterPublisher<CameraInfoMsg>(this._camInfoTopic);

    this._camInfomessage = new CameraInfoMsg();
    // setup ROS Message
    this._message = new ImageMsg(now(), (uint)this._camera.height, (uint)this._camera.width, "rgb8", Convert.ToByte(false), (uint)this._camera.width * (uint)3, null);
    this._message.header.frame_id = this._frameId;
    this._message.encoding = "rgb8";
  }

    void Update()
    {
        this._timeElapsed += Time.deltaTime;

        if(this._timeElapsed > (1f/this._camera.scanRate))
        {
            // Update ROS Message
            this._message.header = now();
            this._message.header.frame_id = this._frameId;
            this._message.data = this._camera.data;
            this._ros.Send(this._topicName, this._message);

            this._camInfomessage = CameraInfoGenerator.ConstructCameraInfoMessage(this._unityCam, this._message.header, this._baseline);

            this._ros.Send(this._camInfoTopic, this._camInfomessage);

            // Update time
            this._timeElapsed = 0;
            this._timeStamp = Time.time;
        }
    }

    private static RosMessageTypes.Std.HeaderMsg now() {
      TimeSpan timeSpan = DateTime.Now.ToUniversalTime() - UNIX_EPOCH;
      double msecs = timeSpan.TotalMilliseconds;
      int secs = (int) (msecs / 1000);
      uint nsecs = (uint) ((msecs / 1000 - secs) * 1e+9);
      RosMessageTypes.Std.HeaderMsg header = new RosMessageTypes.Std.HeaderMsg();
      RosMessageTypes.BuiltinInterfaces.TimeMsg time = new RosMessageTypes.BuiltinInterfaces.TimeMsg();

      time.sec = secs;
      time.nanosec = (uint) nsecs;
      header.stamp = time;
      return header;
    }
}
