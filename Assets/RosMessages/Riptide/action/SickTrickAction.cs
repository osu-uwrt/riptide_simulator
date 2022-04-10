using System.Collections.Generic;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;


namespace RosMessageTypes.Riptide
{
    public class SickTrickAction : Action<SickTrickActionGoal, SickTrickActionResult, SickTrickActionFeedback, SickTrickGoal, SickTrickResult, SickTrickFeedback>
    {
        public const string k_RosMessageName = "riptide_msgs/SickTrickAction";
        public override string RosMessageName => k_RosMessageName;


        public SickTrickAction() : base()
        {
            this.action_goal = new SickTrickActionGoal();
            this.action_result = new SickTrickActionResult();
            this.action_feedback = new SickTrickActionFeedback();
        }

        public static SickTrickAction Deserialize(MessageDeserializer deserializer) => new SickTrickAction(deserializer);

        SickTrickAction(MessageDeserializer deserializer)
        {
            this.action_goal = SickTrickActionGoal.Deserialize(deserializer);
            this.action_result = SickTrickActionResult.Deserialize(deserializer);
            this.action_feedback = SickTrickActionFeedback.Deserialize(deserializer);
        }

        public override void SerializeTo(MessageSerializer serializer)
        {
            serializer.Write(this.action_goal);
            serializer.Write(this.action_result);
            serializer.Write(this.action_feedback);
        }

    }
}
