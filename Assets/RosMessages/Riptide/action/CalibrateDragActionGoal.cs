using System.Collections.Generic;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;
using RosMessageTypes.Std;
using RosMessageTypes.Actionlib;

namespace RosMessageTypes.Riptide
{
    public class CalibrateDragActionGoal : ActionGoal<CalibrateDragGoal>
    {
        public const string k_RosMessageName = "riptide_msgs/CalibrateDragActionGoal";
        public override string RosMessageName => k_RosMessageName;


        public CalibrateDragActionGoal() : base()
        {
            this.goal = new CalibrateDragGoal();
        }

        public CalibrateDragActionGoal(HeaderMsg header, GoalIDMsg goal_id, CalibrateDragGoal goal) : base(header, goal_id)
        {
            this.goal = goal;
        }
        public static CalibrateDragActionGoal Deserialize(MessageDeserializer deserializer) => new CalibrateDragActionGoal(deserializer);

        CalibrateDragActionGoal(MessageDeserializer deserializer) : base(deserializer)
        {
            this.goal = CalibrateDragGoal.Deserialize(deserializer);
        }
        public override void SerializeTo(MessageSerializer serializer)
        {
            serializer.Write(this.header);
            serializer.Write(this.goal_id);
            serializer.Write(this.goal);
        }

    }
}
