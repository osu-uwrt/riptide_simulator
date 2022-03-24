using System.Collections.Generic;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;
using RosMessageTypes.Std;
using RosMessageTypes.Actionlib;

namespace RosMessageTypes.Riptide
{
    public class ScriptOhioActionFeedback : ActionFeedback<ScriptOhioFeedback>
    {
        public const string k_RosMessageName = "riptide_msgs/ScriptOhioActionFeedback";
        public override string RosMessageName => k_RosMessageName;


        public ScriptOhioActionFeedback() : base()
        {
            this.feedback = new ScriptOhioFeedback();
        }

        public ScriptOhioActionFeedback(HeaderMsg header, GoalStatusMsg status, ScriptOhioFeedback feedback) : base(header, status)
        {
            this.feedback = feedback;
        }
        public static ScriptOhioActionFeedback Deserialize(MessageDeserializer deserializer) => new ScriptOhioActionFeedback(deserializer);

        ScriptOhioActionFeedback(MessageDeserializer deserializer) : base(deserializer)
        {
            this.feedback = ScriptOhioFeedback.Deserialize(deserializer);
        }
        public override void SerializeTo(MessageSerializer serializer)
        {
            serializer.Write(this.header);
            serializer.Write(this.status);
            serializer.Write(this.feedback);
        }

    }
}
