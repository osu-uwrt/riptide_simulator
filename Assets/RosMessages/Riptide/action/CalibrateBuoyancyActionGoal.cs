using System.Collections.Generic;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;
using RosMessageTypes.Std;
using RosMessageTypes.Actionlib;

namespace RosMessageTypes.Riptide
{
    public class CalibrateBuoyancyActionGoal : ActionGoal<CalibrateBuoyancyGoal>
    {
        public const string k_RosMessageName = "riptide_msgs/CalibrateBuoyancyActionGoal";
        public override string RosMessageName => k_RosMessageName;


        public CalibrateBuoyancyActionGoal() : base()
        {
            this.goal = new CalibrateBuoyancyGoal();
        }

        public CalibrateBuoyancyActionGoal(HeaderMsg header, GoalIDMsg goal_id, CalibrateBuoyancyGoal goal) : base(header, goal_id)
        {
            this.goal = goal;
        }
        public static CalibrateBuoyancyActionGoal Deserialize(MessageDeserializer deserializer) => new CalibrateBuoyancyActionGoal(deserializer);

        CalibrateBuoyancyActionGoal(MessageDeserializer deserializer) : base(deserializer)
        {
            this.goal = CalibrateBuoyancyGoal.Deserialize(deserializer);
        }
        public override void SerializeTo(MessageSerializer serializer)
        {
            serializer.Write(this.header);
            serializer.Write(this.goal_id);
            serializer.Write(this.goal);
        }

    }
}
