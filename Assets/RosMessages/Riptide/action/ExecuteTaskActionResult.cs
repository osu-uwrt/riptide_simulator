using System.Collections.Generic;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;
using RosMessageTypes.Std;
using RosMessageTypes.Actionlib;

namespace RosMessageTypes.Riptide
{
    public class ExecuteTaskActionResult : ActionResult<ExecuteTaskResult>
    {
        public const string k_RosMessageName = "riptide_msgs/ExecuteTaskActionResult";
        public override string RosMessageName => k_RosMessageName;


        public ExecuteTaskActionResult() : base()
        {
            this.result = new ExecuteTaskResult();
        }

        public ExecuteTaskActionResult(HeaderMsg header, GoalStatusMsg status, ExecuteTaskResult result) : base(header, status)
        {
            this.result = result;
        }
        public static ExecuteTaskActionResult Deserialize(MessageDeserializer deserializer) => new ExecuteTaskActionResult(deserializer);

        ExecuteTaskActionResult(MessageDeserializer deserializer) : base(deserializer)
        {
            this.result = ExecuteTaskResult.Deserialize(deserializer);
        }
        public override void SerializeTo(MessageSerializer serializer)
        {
            serializer.Write(this.header);
            serializer.Write(this.status);
            serializer.Write(this.result);
        }

    }
}
