using System.Collections.Generic;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;


namespace RosMessageTypes.Riptide
{
    public class CalibrateBuoyancyAction : Action<CalibrateBuoyancyActionGoal, CalibrateBuoyancyActionResult, CalibrateBuoyancyActionFeedback, CalibrateBuoyancyGoal, CalibrateBuoyancyResult, CalibrateBuoyancyFeedback>
    {
        public const string k_RosMessageName = "riptide_msgs/CalibrateBuoyancyAction";
        public override string RosMessageName => k_RosMessageName;


        public CalibrateBuoyancyAction() : base()
        {
            this.action_goal = new CalibrateBuoyancyActionGoal();
            this.action_result = new CalibrateBuoyancyActionResult();
            this.action_feedback = new CalibrateBuoyancyActionFeedback();
        }

        public static CalibrateBuoyancyAction Deserialize(MessageDeserializer deserializer) => new CalibrateBuoyancyAction(deserializer);

        CalibrateBuoyancyAction(MessageDeserializer deserializer)
        {
            this.action_goal = CalibrateBuoyancyActionGoal.Deserialize(deserializer);
            this.action_result = CalibrateBuoyancyActionResult.Deserialize(deserializer);
            this.action_feedback = CalibrateBuoyancyActionFeedback.Deserialize(deserializer);
        }

        public override void SerializeTo(MessageSerializer serializer)
        {
            serializer.Write(this.action_goal);
            serializer.Write(this.action_result);
            serializer.Write(this.action_feedback);
        }

    }
}
