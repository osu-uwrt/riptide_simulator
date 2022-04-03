using System.Collections.Generic;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;


namespace RosMessageTypes.Riptide
{
    public class ShroudTestAction : Action<ShroudTestActionGoal, ShroudTestActionResult, ShroudTestActionFeedback, ShroudTestGoal, ShroudTestResult, ShroudTestFeedback>
    {
        public const string k_RosMessageName = "riptide_msgs/ShroudTestAction";
        public override string RosMessageName => k_RosMessageName;


        public ShroudTestAction() : base()
        {
            this.action_goal = new ShroudTestActionGoal();
            this.action_result = new ShroudTestActionResult();
            this.action_feedback = new ShroudTestActionFeedback();
        }

        public static ShroudTestAction Deserialize(MessageDeserializer deserializer) => new ShroudTestAction(deserializer);

        ShroudTestAction(MessageDeserializer deserializer)
        {
            this.action_goal = ShroudTestActionGoal.Deserialize(deserializer);
            this.action_result = ShroudTestActionResult.Deserialize(deserializer);
            this.action_feedback = ShroudTestActionFeedback.Deserialize(deserializer);
        }

        public override void SerializeTo(MessageSerializer serializer)
        {
            serializer.Write(this.action_goal);
            serializer.Write(this.action_result);
            serializer.Write(this.action_feedback);
        }

    }
}
