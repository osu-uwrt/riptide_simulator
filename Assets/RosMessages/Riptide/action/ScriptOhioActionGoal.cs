using System.Collections.Generic;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;
using RosMessageTypes.Std;
using RosMessageTypes.Actionlib;

namespace RosMessageTypes.Riptide
{
    public class ScriptOhioActionGoal : ActionGoal<ScriptOhioGoal>
    {
        public const string k_RosMessageName = "riptide_msgs/ScriptOhioActionGoal";
        public override string RosMessageName => k_RosMessageName;


        public ScriptOhioActionGoal() : base()
        {
            this.goal = new ScriptOhioGoal();
        }

        public ScriptOhioActionGoal(HeaderMsg header, GoalIDMsg goal_id, ScriptOhioGoal goal) : base(header, goal_id)
        {
            this.goal = goal;
        }
        public static ScriptOhioActionGoal Deserialize(MessageDeserializer deserializer) => new ScriptOhioActionGoal(deserializer);

        ScriptOhioActionGoal(MessageDeserializer deserializer) : base(deserializer)
        {
            this.goal = ScriptOhioGoal.Deserialize(deserializer);
        }
        public override void SerializeTo(MessageSerializer serializer)
        {
            serializer.Write(this.header);
            serializer.Write(this.goal_id);
            serializer.Write(this.goal);
        }

    }
}
