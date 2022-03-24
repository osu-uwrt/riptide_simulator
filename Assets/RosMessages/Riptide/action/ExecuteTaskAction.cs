using System.Collections.Generic;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;


namespace RosMessageTypes.Riptide
{
    public class ExecuteTaskAction : Action<ExecuteTaskActionGoal, ExecuteTaskActionResult, ExecuteTaskActionFeedback, ExecuteTaskGoal, ExecuteTaskResult, ExecuteTaskFeedback>
    {
        public const string k_RosMessageName = "riptide_msgs/ExecuteTaskAction";
        public override string RosMessageName => k_RosMessageName;


        public ExecuteTaskAction() : base()
        {
            this.action_goal = new ExecuteTaskActionGoal();
            this.action_result = new ExecuteTaskActionResult();
            this.action_feedback = new ExecuteTaskActionFeedback();
        }

        public static ExecuteTaskAction Deserialize(MessageDeserializer deserializer) => new ExecuteTaskAction(deserializer);

        ExecuteTaskAction(MessageDeserializer deserializer)
        {
            this.action_goal = ExecuteTaskActionGoal.Deserialize(deserializer);
            this.action_result = ExecuteTaskActionResult.Deserialize(deserializer);
            this.action_feedback = ExecuteTaskActionFeedback.Deserialize(deserializer);
        }

        public override void SerializeTo(MessageSerializer serializer)
        {
            serializer.Write(this.action_goal);
            serializer.Write(this.action_result);
            serializer.Write(this.action_feedback);
        }

    }
}
