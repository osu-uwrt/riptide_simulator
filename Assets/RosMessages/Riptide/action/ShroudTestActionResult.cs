using System.Collections.Generic;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;
using RosMessageTypes.Std;
using RosMessageTypes.Actionlib;

namespace RosMessageTypes.Riptide
{
    public class ShroudTestActionResult : ActionResult<ShroudTestResult>
    {
        public const string k_RosMessageName = "riptide_msgs/ShroudTestActionResult";
        public override string RosMessageName => k_RosMessageName;


        public ShroudTestActionResult() : base()
        {
            this.result = new ShroudTestResult();
        }

        public ShroudTestActionResult(HeaderMsg header, GoalStatusMsg status, ShroudTestResult result) : base(header, status)
        {
            this.result = result;
        }
        public static ShroudTestActionResult Deserialize(MessageDeserializer deserializer) => new ShroudTestActionResult(deserializer);

        ShroudTestActionResult(MessageDeserializer deserializer) : base(deserializer)
        {
            this.result = ShroudTestResult.Deserialize(deserializer);
        }
        public override void SerializeTo(MessageSerializer serializer)
        {
            serializer.Write(this.header);
            serializer.Write(this.status);
            serializer.Write(this.result);
        }

    }
}
