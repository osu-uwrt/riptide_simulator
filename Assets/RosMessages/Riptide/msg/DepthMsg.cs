//Do not edit! This file was generated by Unity-ROS MessageGeneration.
using System;
using System.Linq;
using System.Collections.Generic;
using System.Text;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;

namespace RosMessageTypes.Riptide
{
    [Serializable]
    public class DepthMsg : Message
    {
        public const string k_RosMessageName = "riptide_msgs2/Depth";
        public override string RosMessageName => k_RosMessageName;

        public Std.HeaderMsg header;
        public float depth;
        public float variance;

        public DepthMsg()
        {
            this.header = new Std.HeaderMsg();
            this.depth = 0.0f;
            this.variance = 0.0f;
        }

        public DepthMsg(Std.HeaderMsg header, float depth, float variance)
        {
            this.header = header;
            this.depth = depth;
            this.variance = variance;
        }

        public static DepthMsg Deserialize(MessageDeserializer deserializer) => new DepthMsg(deserializer);

        private DepthMsg(MessageDeserializer deserializer)
        {
            this.header = Std.HeaderMsg.Deserialize(deserializer);
            deserializer.Read(out this.depth);
            deserializer.Read(out this.variance);
        }

        public override void SerializeTo(MessageSerializer serializer)
        {
            serializer.Write(this.header);
            serializer.Write(this.depth);
            serializer.Write(this.variance);
        }

        public override string ToString()
        {
            return "DepthMsg: " +
            "\nheader: " + header.ToString() +
            "\ndepth: " + depth.ToString() +
            "\nvariance: " + variance.ToString();
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