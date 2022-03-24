using System.Collections.Generic;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;
using RosMessageTypes.Std;
using RosMessageTypes.Actionlib;

namespace RosMessageTypes.Riptide
{
    public class CalibrateDragActionFeedback : ActionFeedback<CalibrateDragFeedback>
    {
        public const string k_RosMessageName = "riptide_msgs/CalibrateDragActionFeedback";
        public override string RosMessageName => k_RosMessageName;


        public CalibrateDragActionFeedback() : base()
        {
            this.feedback = new CalibrateDragFeedback();
        }

        public CalibrateDragActionFeedback(HeaderMsg header, GoalStatusMsg status, CalibrateDragFeedback feedback) : base(header, status)
        {
            this.feedback = feedback;
        }
        public static CalibrateDragActionFeedback Deserialize(MessageDeserializer deserializer) => new CalibrateDragActionFeedback(deserializer);

        CalibrateDragActionFeedback(MessageDeserializer deserializer) : base(deserializer)
        {
            this.feedback = CalibrateDragFeedback.Deserialize(deserializer);
        }
        public override void SerializeTo(MessageSerializer serializer)
        {
            serializer.Write(this.header);
            serializer.Write(this.status);
            serializer.Write(this.feedback);
        }

    }
}
