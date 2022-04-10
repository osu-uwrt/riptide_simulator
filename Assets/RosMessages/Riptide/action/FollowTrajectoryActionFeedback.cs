using System.Collections.Generic;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;
using RosMessageTypes.Std;
using RosMessageTypes.Actionlib;

namespace RosMessageTypes.Riptide
{
    public class FollowTrajectoryActionFeedback : ActionFeedback<FollowTrajectoryFeedback>
    {
        public const string k_RosMessageName = "riptide_msgs/FollowTrajectoryActionFeedback";
        public override string RosMessageName => k_RosMessageName;


        public FollowTrajectoryActionFeedback() : base()
        {
            this.feedback = new FollowTrajectoryFeedback();
        }

        public FollowTrajectoryActionFeedback(HeaderMsg header, GoalStatusMsg status, FollowTrajectoryFeedback feedback) : base(header, status)
        {
            this.feedback = feedback;
        }
        public static FollowTrajectoryActionFeedback Deserialize(MessageDeserializer deserializer) => new FollowTrajectoryActionFeedback(deserializer);

        FollowTrajectoryActionFeedback(MessageDeserializer deserializer) : base(deserializer)
        {
            this.feedback = FollowTrajectoryFeedback.Deserialize(deserializer);
        }
        public override void SerializeTo(MessageSerializer serializer)
        {
            serializer.Write(this.header);
            serializer.Write(this.status);
            serializer.Write(this.feedback);
        }

    }
}
