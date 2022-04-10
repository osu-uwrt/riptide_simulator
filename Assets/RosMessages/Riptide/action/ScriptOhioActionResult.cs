using System.Collections.Generic;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;
using RosMessageTypes.Std;
using RosMessageTypes.Actionlib;

namespace RosMessageTypes.Riptide
{
    public class ScriptOhioActionResult : ActionResult<ScriptOhioResult>
    {
        public const string k_RosMessageName = "riptide_msgs/ScriptOhioActionResult";
        public override string RosMessageName => k_RosMessageName;


        public ScriptOhioActionResult() : base()
        {
            this.result = new ScriptOhioResult();
        }

        public ScriptOhioActionResult(HeaderMsg header, GoalStatusMsg status, ScriptOhioResult result) : base(header, status)
        {
            this.result = result;
        }
        public static ScriptOhioActionResult Deserialize(MessageDeserializer deserializer) => new ScriptOhioActionResult(deserializer);

        ScriptOhioActionResult(MessageDeserializer deserializer) : base(deserializer)
        {
            this.result = ScriptOhioResult.Deserialize(deserializer);
        }
        public override void SerializeTo(MessageSerializer serializer)
        {
            serializer.Write(this.header);
            serializer.Write(this.status);
            serializer.Write(this.result);
        }

    }
}
