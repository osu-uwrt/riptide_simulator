using System.Collections.Generic;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;


namespace RosMessageTypes.Riptide
{
    public class FollowTrajectoryAction : Action<FollowTrajectoryActionGoal, FollowTrajectoryActionResult, FollowTrajectoryActionFeedback, FollowTrajectoryGoal, FollowTrajectoryResult, FollowTrajectoryFeedback>
    {
        public const string k_RosMessageName = "riptide_msgs/FollowTrajectoryAction";
        public override string RosMessageName => k_RosMessageName;


        public FollowTrajectoryAction() : base()
        {
            this.action_goal = new FollowTrajectoryActionGoal();
            this.action_result = new FollowTrajectoryActionResult();
            this.action_feedback = new FollowTrajectoryActionFeedback();
        }

        public static FollowTrajectoryAction Deserialize(MessageDeserializer deserializer) => new FollowTrajectoryAction(deserializer);

        FollowTrajectoryAction(MessageDeserializer deserializer)
        {
            this.action_goal = FollowTrajectoryActionGoal.Deserialize(deserializer);
            this.action_result = FollowTrajectoryActionResult.Deserialize(deserializer);
            this.action_feedback = FollowTrajectoryActionFeedback.Deserialize(deserializer);
        }

        public override void SerializeTo(MessageSerializer serializer)
        {
            serializer.Write(this.action_goal);
            serializer.Write(this.action_result);
            serializer.Write(this.action_feedback);
        }

    }
}
