using System.Collections.Generic;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;


namespace RosMessageTypes.Riptide
{
    public class CalibrateDragAction : Action<CalibrateDragActionGoal, CalibrateDragActionResult, CalibrateDragActionFeedback, CalibrateDragGoal, CalibrateDragResult, CalibrateDragFeedback>
    {
        public const string k_RosMessageName = "riptide_msgs/CalibrateDragAction";
        public override string RosMessageName => k_RosMessageName;


        public CalibrateDragAction() : base()
        {
            this.action_goal = new CalibrateDragActionGoal();
            this.action_result = new CalibrateDragActionResult();
            this.action_feedback = new CalibrateDragActionFeedback();
        }

        public static CalibrateDragAction Deserialize(MessageDeserializer deserializer) => new CalibrateDragAction(deserializer);

        CalibrateDragAction(MessageDeserializer deserializer)
        {
            this.action_goal = CalibrateDragActionGoal.Deserialize(deserializer);
            this.action_result = CalibrateDragActionResult.Deserialize(deserializer);
            this.action_feedback = CalibrateDragActionFeedback.Deserialize(deserializer);
        }

        public override void SerializeTo(MessageSerializer serializer)
        {
            serializer.Write(this.action_goal);
            serializer.Write(this.action_result);
            serializer.Write(this.action_feedback);
        }

    }
}
