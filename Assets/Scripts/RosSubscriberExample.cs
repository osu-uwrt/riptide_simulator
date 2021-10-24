using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
// using PuddlePos = RosMessageTypes.UnityRoboticsDemo.Vector3Msg;
// using PuddlesImuData = RosMessageTypes.UnityRoboticsDemo.ImuMsg;
// using PuddlesThrustData = RosMessageTypes.UnityRoboticsDemo.Float32MultiArrayMsg;
using GazeboModel = RosMessageTypes.UnityRoboticsDemo.ModelStatesMsg;
using GazeboPose = RosMessageTypes.Geometry.PoseMsg;
using GazeboQuaternion = RosMessageTypes.Geometry.QuaternionMsg;

public class RosSubscriberExample : MonoBehaviour
{
    public GameObject vehicle;
    
    void Start()
    {
        // ROSConnection.GetOrCreateInstance().Subscribe<PuddlePos>("puddles/position", PositionChange);
        // ROSConnection.GetOrCreateInstance().Subscribe<PuddlesImuData>("puddles/imu/data", ImuDataChange);
        // ROSConnection.GetOrCreateInstance().Subscribe<PuddlesThrustData>("puddles/thruster_forces", ThrustChange);
        ROSConnection.GetOrCreateInstance().Subscribe<GazeboModel>("gazebo/model_states", ModelChange);
    }

    // void PositionChange(PuddlePos positionMessage)
    // {
    //     Debug.Log(positionMessage);
    // }
    
    // void ImuDataChange(PuddlesImuData imuDataMessage)
    // {
    //     Debug.Log(imuDataMessage);
    // }
    
    // void ThrustChange(PuddlesThrustData thrustDataMessage)
    // {
        
    // }

    void ModelChange(GazeboModel modelMessage)
    {
        for(int i = 0; i <= modelMessage.name.Length-1; i++) {
            if (modelMessage.name[i].Equals("puddles")) {
                GazeboPose pose = modelMessage.pose[i];
               // GazeboQuaternion quaternion = pose.orientation;
                Vector3 rectifiedPos = new Vector3((float)pose.position.x, (float)pose.position.z, (float)pose.position.y); // Translation between reference frames
              //  Vector3 euler = new Vector3((float)pose.orientation.x, (float)pose.orientation.z, (float)pose.orientation.y);
                vehicle.transform.position = rectifiedPos;
            }
        }
    }
}
