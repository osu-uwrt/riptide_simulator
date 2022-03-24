using System.Collections.Generic;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;
using RosMessageTypes.Std;
using RosMessageTypes.Actionlib;

namespace RosMessageTypes.Riptide
{
    public class ThrusterTestActionResult : ActionResult<ThrusterTestResult>
    {
        public const string k_RosMessageName = "riptide_msgs/ThrusterTestActionResult";
        public override string RosMessageName => k_RosMessageName;


        public ThrusterTestActionResult() : base()
        {
            this.result = new ThrusterTestResult();
        }

        public ThrusterTestActionResult(HeaderMsg header, GoalStatusMsg status, ThrusterTestResult result) : base(header, status)
        {
            this.result = result;
        }
        public static ThrusterTestActionResult Deserialize(MessageDeserializer deserializer) => new ThrusterTestActionResult(deserializer);

        ThrusterTestActionResult(MessageDeserializer deserializer) : base(deserializer)
        {
            this.result = ThrusterTestResult.Deserialize(deserializer);
        }
        public override void SerializeTo(MessageSerializer serializer)
        {
            serializer.Write(this.header);
            serializer.Write(this.status);
            serializer.Write(this.result);
        }

    }
}
