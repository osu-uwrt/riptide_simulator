using System.Collections.Generic;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;
using RosMessageTypes.Std;
using RosMessageTypes.Actionlib;

namespace RosMessageTypes.Riptide
{
    public class SickTrickActionFeedback : ActionFeedback<SickTrickFeedback>
    {
        public const string k_RosMessageName = "riptide_msgs/SickTrickActionFeedback";
        public override string RosMessageName => k_RosMessageName;


        public SickTrickActionFeedback() : base()
        {
            this.feedback = new SickTrickFeedback();
        }

        public SickTrickActionFeedback(HeaderMsg header, GoalStatusMsg status, SickTrickFeedback feedback) : base(header, status)
        {
            this.feedback = feedback;
        }
        public static SickTrickActionFeedback Deserialize(MessageDeserializer deserializer) => new SickTrickActionFeedback(deserializer);

        SickTrickActionFeedback(MessageDeserializer deserializer) : base(deserializer)
        {
            this.feedback = SickTrickFeedback.Deserialize(deserializer);
        }
        public override void SerializeTo(MessageSerializer serializer)
        {
            serializer.Write(this.header);
            serializer.Write(this.status);
            serializer.Write(this.feedback);
        }

    }
}
