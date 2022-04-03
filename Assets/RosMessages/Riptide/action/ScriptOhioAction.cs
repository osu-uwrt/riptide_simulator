using System.Collections.Generic;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;


namespace RosMessageTypes.Riptide
{
    public class ScriptOhioAction : Action<ScriptOhioActionGoal, ScriptOhioActionResult, ScriptOhioActionFeedback, ScriptOhioGoal, ScriptOhioResult, ScriptOhioFeedback>
    {
        public const string k_RosMessageName = "riptide_msgs/ScriptOhioAction";
        public override string RosMessageName => k_RosMessageName;


        public ScriptOhioAction() : base()
        {
            this.action_goal = new ScriptOhioActionGoal();
            this.action_result = new ScriptOhioActionResult();
            this.action_feedback = new ScriptOhioActionFeedback();
        }

        public static ScriptOhioAction Deserialize(MessageDeserializer deserializer) => new ScriptOhioAction(deserializer);

        ScriptOhioAction(MessageDeserializer deserializer)
        {
            this.action_goal = ScriptOhioActionGoal.Deserialize(deserializer);
            this.action_result = ScriptOhioActionResult.Deserialize(deserializer);
            this.action_feedback = ScriptOhioActionFeedback.Deserialize(deserializer);
        }

        public override void SerializeTo(MessageSerializer serializer)
        {
            serializer.Write(this.action_goal);
            serializer.Write(this.action_result);
            serializer.Write(this.action_feedback);
        }

    }
}
