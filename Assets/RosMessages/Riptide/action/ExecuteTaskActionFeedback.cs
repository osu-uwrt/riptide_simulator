using System.Collections.Generic;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;
using RosMessageTypes.Std;
using RosMessageTypes.Actionlib;

namespace RosMessageTypes.Riptide
{
    public class ExecuteTaskActionFeedback : ActionFeedback<ExecuteTaskFeedback>
    {
        public const string k_RosMessageName = "riptide_msgs/ExecuteTaskActionFeedback";
        public override string RosMessageName => k_RosMessageName;


        public ExecuteTaskActionFeedback() : base()
        {
            this.feedback = new ExecuteTaskFeedback();
        }

        public ExecuteTaskActionFeedback(HeaderMsg header, GoalStatusMsg status, ExecuteTaskFeedback feedback) : base(header, status)
        {
            this.feedback = feedback;
        }
        public static ExecuteTaskActionFeedback Deserialize(MessageDeserializer deserializer) => new ExecuteTaskActionFeedback(deserializer);

        ExecuteTaskActionFeedback(MessageDeserializer deserializer) : base(deserializer)
        {
            this.feedback = ExecuteTaskFeedback.Deserialize(deserializer);
        }
        public override void SerializeTo(MessageSerializer serializer)
        {
            serializer.Write(this.header);
            serializer.Write(this.status);
            serializer.Write(this.feedback);
        }

    }
}
