using System.Collections.Generic;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;
using RosMessageTypes.Std;
using RosMessageTypes.Actionlib;

namespace RosMessageTypes.Riptide
{
    public class ShroudTestActionGoal : ActionGoal<ShroudTestGoal>
    {
        public const string k_RosMessageName = "riptide_msgs/ShroudTestActionGoal";
        public override string RosMessageName => k_RosMessageName;


        public ShroudTestActionGoal() : base()
        {
            this.goal = new ShroudTestGoal();
        }

        public ShroudTestActionGoal(HeaderMsg header, GoalIDMsg goal_id, ShroudTestGoal goal) : base(header, goal_id)
        {
            this.goal = goal;
        }
        public static ShroudTestActionGoal Deserialize(MessageDeserializer deserializer) => new ShroudTestActionGoal(deserializer);

        ShroudTestActionGoal(MessageDeserializer deserializer) : base(deserializer)
        {
            this.goal = ShroudTestGoal.Deserialize(deserializer);
        }
        public override void SerializeTo(MessageSerializer serializer)
        {
            serializer.Write(this.header);
            serializer.Write(this.goal_id);
            serializer.Write(this.goal);
        }

    }
}
