using System.Collections.Generic;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;
using RosMessageTypes.Std;
using RosMessageTypes.Actionlib;

namespace RosMessageTypes.Riptide
{
    public class SickTrickActionGoal : ActionGoal<SickTrickGoal>
    {
        public const string k_RosMessageName = "riptide_msgs/SickTrickActionGoal";
        public override string RosMessageName => k_RosMessageName;


        public SickTrickActionGoal() : base()
        {
            this.goal = new SickTrickGoal();
        }

        public SickTrickActionGoal(HeaderMsg header, GoalIDMsg goal_id, SickTrickGoal goal) : base(header, goal_id)
        {
            this.goal = goal;
        }
        public static SickTrickActionGoal Deserialize(MessageDeserializer deserializer) => new SickTrickActionGoal(deserializer);

        SickTrickActionGoal(MessageDeserializer deserializer) : base(deserializer)
        {
            this.goal = SickTrickGoal.Deserialize(deserializer);
        }
        public override void SerializeTo(MessageSerializer serializer)
        {
            serializer.Write(this.header);
            serializer.Write(this.goal_id);
            serializer.Write(this.goal);
        }

    }
}
