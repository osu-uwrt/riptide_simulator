//Do not edit! This file was generated by Unity-ROS MessageGeneration.
using System;
using System.Linq;
using System.Collections.Generic;
using System.Text;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;

namespace RosMessageTypes.Riptide
{
    [Serializable]
    public class ControlStatusLinearMsg : Message
    {
        public const string k_RosMessageName = "riptide_msgs2/ControlStatusLinear";
        public override string RosMessageName => k_RosMessageName;

        public Std.HeaderMsg header;
        public ControlStatusMsg x;
        public ControlStatusMsg y;
        public ControlStatusMsg z;

        public ControlStatusLinearMsg()
        {
            this.header = new Std.HeaderMsg();
            this.x = new ControlStatusMsg();
            this.y = new ControlStatusMsg();
            this.z = new ControlStatusMsg();
        }

        public ControlStatusLinearMsg(Std.HeaderMsg header, ControlStatusMsg x, ControlStatusMsg y, ControlStatusMsg z)
        {
            this.header = header;
            this.x = x;
            this.y = y;
            this.z = z;
        }

        public static ControlStatusLinearMsg Deserialize(MessageDeserializer deserializer) => new ControlStatusLinearMsg(deserializer);

        private ControlStatusLinearMsg(MessageDeserializer deserializer)
        {
            this.header = Std.HeaderMsg.Deserialize(deserializer);
            this.x = ControlStatusMsg.Deserialize(deserializer);
            this.y = ControlStatusMsg.Deserialize(deserializer);
            this.z = ControlStatusMsg.Deserialize(deserializer);
        }

        public override void SerializeTo(MessageSerializer serializer)
        {
            serializer.Write(this.header);
            serializer.Write(this.x);
            serializer.Write(this.y);
            serializer.Write(this.z);
        }

        public override string ToString()
        {
            return "ControlStatusLinearMsg: " +
            "\nheader: " + header.ToString() +
            "\nx: " + x.ToString() +
            "\ny: " + y.ToString() +
            "\nz: " + z.ToString();
        }

#if UNITY_EDITOR
        [UnityEditor.InitializeOnLoadMethod]
#else
        [UnityEngine.RuntimeInitializeOnLoadMethod]
#endif
        public static void Register()
        {
            MessageRegistry.Register(k_RosMessageName, Deserialize);
        }
    }
}