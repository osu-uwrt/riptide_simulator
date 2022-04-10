using System.Collections.Generic;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;
using RosMessageTypes.Std;
using RosMessageTypes.Actionlib;

namespace RosMessageTypes.Riptide
{
    public class ExecuteTaskActionGoal : ActionGoal<ExecuteTaskGoal>
    {
        public const string k_RosMessageName = "riptide_msgs/ExecuteTaskActionGoal";
        public override string RosMessageName => k_RosMessageName;


        public ExecuteTaskActionGoal() : base()
        {
            this.goal = new ExecuteTaskGoal();
        }

        public ExecuteTaskActionGoal(HeaderMsg header, GoalIDMsg goal_id, ExecuteTaskGoal goal) : base(header, goal_id)
        {
            this.goal = goal;
        }
        public static ExecuteTaskActionGoal Deserialize(MessageDeserializer deserializer) => new ExecuteTaskActionGoal(deserializer);

        ExecuteTaskActionGoal(MessageDeserializer deserializer) : base(deserializer)
        {
            this.goal = ExecuteTaskGoal.Deserialize(deserializer);
        }
        public override void SerializeTo(MessageSerializer serializer)
        {
            serializer.Write(this.header);
            serializer.Write(this.goal_id);
            serializer.Write(this.goal);
        }

    }
}
