using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using PuddlePos = RosMessageTypes.UnityRoboticsDemo.Vector3Msg;
using PuddlesImuData = RosMessageTypes.UnityRoboticsDemo.ImuMsg;
using PuddlesThrustData = RosMessageTypes.UnityRoboticsDemo.Float32MultiArrayMsg;

public class RosSubscriberExample : MonoBehaviour
{
    public GameObject vehicle;

    void Start()
    {
        ROSConnection.GetOrCreateInstance().Subscribe<PuddlePos>("puddles/position", PositionChange);
        ROSConnection.GetOrCreateInstance().Subscribe<PuddlesImuData>("puddles/imu/data", ImuDataChange);
        ROSConnection.GetOrCreateInstance().Subscribe<PuddlesThrustData>("puddles/thruster_forces", ThrustChange);
        

    }

    void PositionChange(PuddlePos PositionMessage)
    {
        Debug.Log(PositionMessage);
    }
    
    void ImuDataChange(PuddlesImuData ImuDataMessage)
    {
        Debug.Log(ImuDataMessage);
    }
    
    void ThrustChange(PuddlesThrustData ThrustDataMessage)
    {
        Debug.Log(ThrustDataMessage);
    }
}
