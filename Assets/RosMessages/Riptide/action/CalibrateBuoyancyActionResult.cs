using System.Collections.Generic;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;
using RosMessageTypes.Std;
using RosMessageTypes.Actionlib;

namespace RosMessageTypes.Riptide
{
    public class CalibrateBuoyancyActionResult : ActionResult<CalibrateBuoyancyResult>
    {
        public const string k_RosMessageName = "riptide_msgs/CalibrateBuoyancyActionResult";
        public override string RosMessageName => k_RosMessageName;


        public CalibrateBuoyancyActionResult() : base()
        {
            this.result = new CalibrateBuoyancyResult();
        }

        public CalibrateBuoyancyActionResult(HeaderMsg header, GoalStatusMsg status, CalibrateBuoyancyResult result) : base(header, status)
        {
            this.result = result;
        }
        public static CalibrateBuoyancyActionResult Deserialize(MessageDeserializer deserializer) => new CalibrateBuoyancyActionResult(deserializer);

        CalibrateBuoyancyActionResult(MessageDeserializer deserializer) : base(deserializer)
        {
            this.result = CalibrateBuoyancyResult.Deserialize(deserializer);
        }
        public override void SerializeTo(MessageSerializer serializer)
        {
            serializer.Write(this.header);
            serializer.Write(this.status);
            serializer.Write(this.result);
        }

    }
}
