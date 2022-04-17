using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System.Threading;

using Unity.Robotics.ROSTCPConnector;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;
using RosMessageTypes.Sensor;
using RosMessageTypes.Geometry;

[RequireComponent(typeof(FRJ.Sensor.IMU))]
public class IMUPublisher : MonoBehaviour
{

    [SerializeField] private string _topicName = "imu/raw_data";
    [SerializeField] private string _frameId = "imu_link";

    private double lastUpdate;

    private ROSConnection _ros;
    public ImuMsg _message;

    private FRJ.Sensor.IMU _imu;

    Thread imuTh;

    void Start()
    {
        // Get Rotate Lidar
        this._imu = GetComponent<FRJ.Sensor.IMU>();

        // setup ROS
        this._ros = ROSConnection.GetOrCreateInstance();
        this._ros.RegisterPublisher<ImuMsg>(this._topicName);

        // setup ROS Message
        this._message = new ImuMsg();
        this._message.header.frame_id = this._frameId;
        lastUpdate = (DateTime.Now.ToUniversalTime() - UNIX_EPOCH).TotalMilliseconds;
        imuTh = new Thread(IMU);
        imuTh.Start();
    }
    private readonly static DateTime UNIX_EPOCH = new DateTime(1970, 1, 1, 0, 0, 0, 0, DateTimeKind.Utc);
    private static RosMessageTypes.Std.HeaderMsg now()
    {
        TimeSpan timeSpan = DateTime.Now.ToUniversalTime() - UNIX_EPOCH;
        double msecs = timeSpan.TotalMilliseconds;
        uint secs = (uint)(msecs / 1000);
        uint nsecs = (uint)((msecs / 1000 - secs) * 1e+9);
        RosMessageTypes.Std.HeaderMsg header = new RosMessageTypes.Std.HeaderMsg();
        RosMessageTypes.BuiltinInterfaces.TimeMsg time = new RosMessageTypes.BuiltinInterfaces.TimeMsg();
        time.sec = (int)secs;
        time.nanosec = nsecs;
        header.stamp = time;
        return header;
    }
    void IMU()
    {
        while (true)
        {
            if((DateTime.Now.ToUniversalTime() - UNIX_EPOCH).TotalMilliseconds-lastUpdate >= .06) {
                lastUpdate = (DateTime.Now.ToUniversalTime() - UNIX_EPOCH).TotalMilliseconds;

                // Update IMU data
                this._imu.UpdateIMU();

                // Update ROS Message
                this._message.header = now();
                Quaternion<FLU> orientation_ros = new Quaternion<FLU>(this._imu.GeometryQuaternion.x,
                                                                      this._imu.GeometryQuaternion.y,
                                                                      this._imu.GeometryQuaternion.z,
                                                                      this._imu.GeometryQuaternion.w).To<FLU>();
                QuaternionMsg orientation =
                    new QuaternionMsg(orientation_ros.x,
                                      orientation_ros.y,
                                      orientation_ros.z,
                                      orientation_ros.w);
                this._message.orientation = orientation;
                Vector3<FLU> angular_velocity_ros = new Vector3<FLU>(this._imu.AngularVelocity).To<FLU>();
                Vector3Msg angular_velocity =
                    new Vector3Msg(angular_velocity_ros.x,
                                   angular_velocity_ros.y,
                                   angular_velocity_ros.z);
                this._message.angular_velocity = angular_velocity;
                Vector3<FLU> linear_acceleration_ros = new Vector3<FLU>(this._imu.LinearAcceleration).To<FLU>();
                Vector3Msg linear_acceleration =
                    new Vector3Msg(linear_acceleration_ros.x,
                                   linear_acceleration_ros.y,
                                   linear_acceleration_ros.z);
                this._message.linear_acceleration = linear_acceleration;
                this._ros.Publish(this._topicName, this._message);
            }
            
        }
    }
}
