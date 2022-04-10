using System.Collections.Generic;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;
using RosMessageTypes.Std;
using RosMessageTypes.Actionlib;

namespace RosMessageTypes.Riptide
{
    public class FollowTrajectoryActionResult : ActionResult<FollowTrajectoryResult>
    {
        public const string k_RosMessageName = "riptide_msgs/FollowTrajectoryActionResult";
        public override string RosMessageName => k_RosMessageName;


        public FollowTrajectoryActionResult() : base()
        {
            this.result = new FollowTrajectoryResult();
        }

        public FollowTrajectoryActionResult(HeaderMsg header, GoalStatusMsg status, FollowTrajectoryResult result) : base(header, status)
        {
            this.result = result;
        }
        public static FollowTrajectoryActionResult Deserialize(MessageDeserializer deserializer) => new FollowTrajectoryActionResult(deserializer);

        FollowTrajectoryActionResult(MessageDeserializer deserializer) : base(deserializer)
        {
            this.result = FollowTrajectoryResult.Deserialize(deserializer);
        }
        public override void SerializeTo(MessageSerializer serializer)
        {
            serializer.Write(this.header);
            serializer.Write(this.status);
            serializer.Write(this.result);
        }

    }
}
