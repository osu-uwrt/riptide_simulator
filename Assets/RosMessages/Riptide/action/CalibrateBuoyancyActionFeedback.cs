using System.Collections.Generic;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;
using RosMessageTypes.Std;
using RosMessageTypes.Actionlib;

namespace RosMessageTypes.Riptide
{
    public class CalibrateBuoyancyActionFeedback : ActionFeedback<CalibrateBuoyancyFeedback>
    {
        public const string k_RosMessageName = "riptide_msgs/CalibrateBuoyancyActionFeedback";
        public override string RosMessageName => k_RosMessageName;


        public CalibrateBuoyancyActionFeedback() : base()
        {
            this.feedback = new CalibrateBuoyancyFeedback();
        }

        public CalibrateBuoyancyActionFeedback(HeaderMsg header, GoalStatusMsg status, CalibrateBuoyancyFeedback feedback) : base(header, status)
        {
            this.feedback = feedback;
        }
        public static CalibrateBuoyancyActionFeedback Deserialize(MessageDeserializer deserializer) => new CalibrateBuoyancyActionFeedback(deserializer);

        CalibrateBuoyancyActionFeedback(MessageDeserializer deserializer) : base(deserializer)
        {
            this.feedback = CalibrateBuoyancyFeedback.Deserialize(deserializer);
        }
        public override void SerializeTo(MessageSerializer serializer)
        {
            serializer.Write(this.header);
            serializer.Write(this.status);
            serializer.Write(this.feedback);
        }

    }
}
