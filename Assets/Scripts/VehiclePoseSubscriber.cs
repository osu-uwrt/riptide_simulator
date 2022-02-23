using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using GazeboModelStates = RosMessageTypes.RiptideGazebo.ModelStatesMsg;
using GazeboPose = RosMessageTypes.Geometry.PoseMsg;
using GazeboQuaternion = RosMessageTypes.Geometry.QuaternionMsg;
public class VehiclePoseSubscriber : MonoBehaviour
{
    // Vehicle to update
    public GameObject vehicle;
    public string modelMessageName;

    private int vehicleModelMessageIndex = -1;
    
    void Start()
    {
        // Subcription to gazebo/model_states, used to retrieve the physical position and orientation of the vehicle.
        ROSConnection.GetOrCreateInstance().Subscribe<GazeboModelStates>("gazebo/model_states", UpdateVehicle);
    }

    void UpdateVehicle(GazeboModelStates modelMessage)
    {
        // TODO: We should NOT be looping through all of the game objects being simulated by gazebo.
        // Instead, we should just be publishing the vehicle's position from gazebo and using it here 
        // to update the model in unity.
        if (vehicleModelMessageIndex == -1) {
            for(int i = 0; i <= modelMessage.name.Length - 1; i++) {
                if (modelMessage.name[i].Equals(modelMessageName)) {
                    vehicleModelMessageIndex = i;
                    break;
                }
            }
        }

        // Get the vehicles pose information
        GazeboPose pose = modelMessage.pose[vehicleModelMessageIndex];

        // Translation between reference frames
        Vector3 rectifiedPosition = new Vector3(-(float)pose.position.y, (float)pose.position.z, (float)pose.position.x); 
        Quaternion rectifiedOrientation = new Quaternion((float)pose.orientation.y,-(float)pose.orientation.z,-(float)pose.orientation.x,(float)pose.orientation.w);

        // Transform the vehicle
        vehicle.transform.position = rectifiedPosition;
        vehicle.transform.rotation = rectifiedOrientation;
    }
}
