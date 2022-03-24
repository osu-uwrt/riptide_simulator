using System.Collections.Generic;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;
using RosMessageTypes.Std;
using RosMessageTypes.Actionlib;

namespace RosMessageTypes.Riptide
{
    public class FollowTrajectoryActionGoal : ActionGoal<FollowTrajectoryGoal>
    {
        public const string k_RosMessageName = "riptide_msgs/FollowTrajectoryActionGoal";
        public override string RosMessageName => k_RosMessageName;


        public FollowTrajectoryActionGoal() : base()
        {
            this.goal = new FollowTrajectoryGoal();
        }

        public FollowTrajectoryActionGoal(HeaderMsg header, GoalIDMsg goal_id, FollowTrajectoryGoal goal) : base(header, goal_id)
        {
            this.goal = goal;
        }
        public static FollowTrajectoryActionGoal Deserialize(MessageDeserializer deserializer) => new FollowTrajectoryActionGoal(deserializer);

        FollowTrajectoryActionGoal(MessageDeserializer deserializer) : base(deserializer)
        {
            this.goal = FollowTrajectoryGoal.Deserialize(deserializer);
        }
        public override void SerializeTo(MessageSerializer serializer)
        {
            serializer.Write(this.header);
            serializer.Write(this.goal_id);
            serializer.Write(this.goal);
        }

    }
}
