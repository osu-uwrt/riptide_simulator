using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

using Unity.Robotics.ROSTCPConnector;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;
using RosMessageTypes.Sensor;
using RosMessageTypes.Geometry;

[RequireComponent(typeof(FRJ.Sensor.DVL))]
public class DVLPublisher : MonoBehaviour
{

    [SerializeField] private string _topicName = "dvl_twist";
    [SerializeField] private string _frameId = "dvl";

    private float _timeElapsed = 0f;
    private float _timeStamp = 0f;

    private ROSConnection _ros;
    public TwistWithCovarianceStampedMsg _message;

    private FRJ.Sensor.DVL _dvl;

    void Start()
    {
        // Get Rotate Lidar
        this._dvl = GetComponent<FRJ.Sensor.DVL>();

        // setup ROS
        this._ros = ROSConnection.GetOrCreateInstance() ;
        this._ros.RegisterPublisher<ImuMsg>(this._topicName);

        // setup ROS Message
        this._message = new TwistWithCovarianceStampedMsg();
        this._message.header.frame_id = this._frameId;
    }

    void FixedUpdate()
    {
            this._timeStamp = Time.time;

            // Update IMU data
            this._dvl.UpdateDVL();

            // Update ROS Message
            int sec = (int)Math.Truncate(this._timeStamp);
            uint nanosec = (uint)((this._timeStamp - sec) * 1e+9);
            this._message.header.stamp.sec = sec;
            this._message.header.stamp.nanosec = nanosec;
            Vector3<FLU> angular_velocity_ros = new Vector3<FLU>(this._dvl.AngularVelocity).To<FLU>();
            Vector3Msg angular_velocity =
                new Vector3Msg(angular_velocity_ros.x,
                               angular_velocity_ros.y,
                               angular_velocity_ros.z);
            this._message.twist.twist.angular = angular_velocity;
            Vector3<FLU> linear_velocity_ros = new Vector3<FLU>(this._dvl.LinearVelocity).To<FLU>();
            Vector3Msg linear_velocity =
                new Vector3Msg(linear_velocity_ros.x,
                               linear_velocity_ros.y,
                               linear_velocity_ros.z);
            this._message.twist.twist.linear = linear_velocity;

            Vector3 angularVar = new Vector3(_dvl.setting.angVelSigma.x * _dvl.setting.angVelSigma.x, _dvl.setting.angVelSigma.y * _dvl.setting.angVelSigma.y, _dvl.setting.angVelSigma.z * _dvl.setting.angVelSigma.z);
            Vector3 linearVar = new Vector3(_dvl.setting.linVelSigma.x * _dvl.setting.linVelSigma.x, _dvl.setting.linVelSigma.y * _dvl.setting.linVelSigma.y, _dvl.setting.linVelSigma.z * _dvl.setting.linVelSigma.z);
            this._message.twist.covariance = new double[] {linearVar.x,0,          0,          0,            0,          0,
                                                            0,          linearVar.y,0,          0,            0,          0,
                                                            0,          0,          linearVar.z,0,            0,          0,
                                                            0,          0,          0,          angularVar.x, 0,          0,
                                                            0,          0,          0,          0,            angularVar.y,0,
                                                            0,          0,          0,          0,            0,          angularVar.z};
            this._ros.Publish(this._topicName, this._message);
        
    }
}
