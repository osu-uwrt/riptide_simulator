using System.Collections.Generic;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;
using RosMessageTypes.Std;
using RosMessageTypes.Actionlib;

namespace RosMessageTypes.Riptide
{
    public class ThrusterTestActionFeedback : ActionFeedback<ThrusterTestFeedback>
    {
        public const string k_RosMessageName = "riptide_msgs/ThrusterTestActionFeedback";
        public override string RosMessageName => k_RosMessageName;


        public ThrusterTestActionFeedback() : base()
        {
            this.feedback = new ThrusterTestFeedback();
        }

        public ThrusterTestActionFeedback(HeaderMsg header, GoalStatusMsg status, ThrusterTestFeedback feedback) : base(header, status)
        {
            this.feedback = feedback;
        }
        public static ThrusterTestActionFeedback Deserialize(MessageDeserializer deserializer) => new ThrusterTestActionFeedback(deserializer);

        ThrusterTestActionFeedback(MessageDeserializer deserializer) : base(deserializer)
        {
            this.feedback = ThrusterTestFeedback.Deserialize(deserializer);
        }
        public override void SerializeTo(MessageSerializer serializer)
        {
            serializer.Write(this.header);
            serializer.Write(this.status);
            serializer.Write(this.feedback);
        }

    }
}
