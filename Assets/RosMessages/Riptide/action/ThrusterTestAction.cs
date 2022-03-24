using System.Collections.Generic;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;


namespace RosMessageTypes.Riptide
{
    public class ThrusterTestAction : Action<ThrusterTestActionGoal, ThrusterTestActionResult, ThrusterTestActionFeedback, ThrusterTestGoal, ThrusterTestResult, ThrusterTestFeedback>
    {
        public const string k_RosMessageName = "riptide_msgs/ThrusterTestAction";
        public override string RosMessageName => k_RosMessageName;


        public ThrusterTestAction() : base()
        {
            this.action_goal = new ThrusterTestActionGoal();
            this.action_result = new ThrusterTestActionResult();
            this.action_feedback = new ThrusterTestActionFeedback();
        }

        public static ThrusterTestAction Deserialize(MessageDeserializer deserializer) => new ThrusterTestAction(deserializer);

        ThrusterTestAction(MessageDeserializer deserializer)
        {
            this.action_goal = ThrusterTestActionGoal.Deserialize(deserializer);
            this.action_result = ThrusterTestActionResult.Deserialize(deserializer);
            this.action_feedback = ThrusterTestActionFeedback.Deserialize(deserializer);
        }

        public override void SerializeTo(MessageSerializer serializer)
        {
            serializer.Write(this.action_goal);
            serializer.Write(this.action_result);
            serializer.Write(this.action_feedback);
        }

    }
}
