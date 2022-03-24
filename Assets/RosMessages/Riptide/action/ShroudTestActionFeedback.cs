using System.Collections.Generic;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;
using RosMessageTypes.Std;
using RosMessageTypes.Actionlib;

namespace RosMessageTypes.Riptide
{
    public class ShroudTestActionFeedback : ActionFeedback<ShroudTestFeedback>
    {
        public const string k_RosMessageName = "riptide_msgs/ShroudTestActionFeedback";
        public override string RosMessageName => k_RosMessageName;


        public ShroudTestActionFeedback() : base()
        {
            this.feedback = new ShroudTestFeedback();
        }

        public ShroudTestActionFeedback(HeaderMsg header, GoalStatusMsg status, ShroudTestFeedback feedback) : base(header, status)
        {
            this.feedback = feedback;
        }
        public static ShroudTestActionFeedback Deserialize(MessageDeserializer deserializer) => new ShroudTestActionFeedback(deserializer);

        ShroudTestActionFeedback(MessageDeserializer deserializer) : base(deserializer)
        {
            this.feedback = ShroudTestFeedback.Deserialize(deserializer);
        }
        public override void SerializeTo(MessageSerializer serializer)
        {
            serializer.Write(this.header);
            serializer.Write(this.status);
            serializer.Write(this.feedback);
        }

    }
}
