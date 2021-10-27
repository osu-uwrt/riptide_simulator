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
    // Vehicle to update
    public GameObject vehicle;
    
    void Start()
    {
        // ROSConnection.GetOrCreateInstance().Subscribe<PuddlePos>("puddles/position", PositionChange);
        // ROSConnection.GetOrCreateInstance().Subscribe<PuddlesImuData>("puddles/imu/data", ImuDataChange);
        // ROSConnection.GetOrCreateInstance().Subscribe<PuddlesThrustData>("puddles/thruster_forces", ThrustChange);
        // Subcription to gazebo/model_states, used to retrieve the physical position and orientation of the vehicle.
        ROSConnection.GetOrCreateInstance().Subscribe<GazeboModel>("gazebo/model_states", UpdateVehicle);
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

    void UpdateVehicle(GazeboModel modelMessage)
    {
        for(int i = 0; i <= modelMessage.name.Length-1; i++) {
            if (modelMessage.name[i].Equals("puddles")) {
                // Get the vehicles pose information
                GazeboPose pose = modelMessage.pose[i];

                // Translation between reference frames
                Vector3 rectifiedPosition = new Vector3(-(float)pose.position.y, (float)pose.position.z, (float)pose.position.x); 
                
                GazeboQuaternion modelOrientation = pose.orientation;
                Quaternion rectifiedOrientation = new Quaternion(-(float)pose.orientation.y,(float)pose.orientation.z,(float)pose.orientation.x,(float)pose.orientation.w);

                // Transform the vehicle
                vehicle.transform.position = rectifiedPosition;
                vehicle.transform.rotation = rectifiedOrientation;
            }
        }
    }
}
