using System.Collections.Generic;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;
using RosMessageTypes.Std;
using RosMessageTypes.Actionlib;

namespace RosMessageTypes.Riptide
{
    public class CalibrateDragActionResult : ActionResult<CalibrateDragResult>
    {
        public const string k_RosMessageName = "riptide_msgs/CalibrateDragActionResult";
        public override string RosMessageName => k_RosMessageName;


        public CalibrateDragActionResult() : base()
        {
            this.result = new CalibrateDragResult();
        }

        public CalibrateDragActionResult(HeaderMsg header, GoalStatusMsg status, CalibrateDragResult result) : base(header, status)
        {
            this.result = result;
        }
        public static CalibrateDragActionResult Deserialize(MessageDeserializer deserializer) => new CalibrateDragActionResult(deserializer);

        CalibrateDragActionResult(MessageDeserializer deserializer) : base(deserializer)
        {
            this.result = CalibrateDragResult.Deserialize(deserializer);
        }
        public override void SerializeTo(MessageSerializer serializer)
        {
            serializer.Write(this.header);
            serializer.Write(this.status);
            serializer.Write(this.result);
        }

    }
}
