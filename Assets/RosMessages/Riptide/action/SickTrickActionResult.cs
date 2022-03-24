using System.Collections.Generic;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;
using RosMessageTypes.Std;
using RosMessageTypes.Actionlib;

namespace RosMessageTypes.Riptide
{
    public class SickTrickActionResult : ActionResult<SickTrickResult>
    {
        public const string k_RosMessageName = "riptide_msgs/SickTrickActionResult";
        public override string RosMessageName => k_RosMessageName;


        public SickTrickActionResult() : base()
        {
            this.result = new SickTrickResult();
        }

        public SickTrickActionResult(HeaderMsg header, GoalStatusMsg status, SickTrickResult result) : base(header, status)
        {
            this.result = result;
        }
        public static SickTrickActionResult Deserialize(MessageDeserializer deserializer) => new SickTrickActionResult(deserializer);

        SickTrickActionResult(MessageDeserializer deserializer) : base(deserializer)
        {
            this.result = SickTrickResult.Deserialize(deserializer);
        }
        public override void SerializeTo(MessageSerializer serializer)
        {
            serializer.Write(this.header);
            serializer.Write(this.status);
            serializer.Write(this.result);
        }

    }
}
