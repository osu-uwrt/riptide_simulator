using System.Collections.Generic;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;
using RosMessageTypes.Std;
using RosMessageTypes.Actionlib;

namespace RosMessageTypes.Riptide
{
    public class ThrusterTestActionGoal : ActionGoal<ThrusterTestGoal>
    {
        public const string k_RosMessageName = "riptide_msgs/ThrusterTestActionGoal";
        public override string RosMessageName => k_RosMessageName;


        public ThrusterTestActionGoal() : base()
        {
            this.goal = new ThrusterTestGoal();
        }

        public ThrusterTestActionGoal(HeaderMsg header, GoalIDMsg goal_id, ThrusterTestGoal goal) : base(header, goal_id)
        {
            this.goal = goal;
        }
        public static ThrusterTestActionGoal Deserialize(MessageDeserializer deserializer) => new ThrusterTestActionGoal(deserializer);

        ThrusterTestActionGoal(MessageDeserializer deserializer) : base(deserializer)
        {
            this.goal = ThrusterTestGoal.Deserialize(deserializer);
        }
        public override void SerializeTo(MessageSerializer serializer)
        {
            serializer.Write(this.header);
            serializer.Write(this.goal_id);
            serializer.Write(this.goal);
        }

    }
}
