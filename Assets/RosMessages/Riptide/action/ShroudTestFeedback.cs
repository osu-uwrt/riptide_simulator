//Do not edit! This file was generated by Unity-ROS MessageGeneration.
using System;
using System.Linq;
using System.Collections.Generic;
using System.Text;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;

namespace RosMessageTypes.Riptide
{
    [Serializable]
    public class ShroudTestFeedback : Message
    {
        public const string k_RosMessageName = "riptide_msgs/ShroudTest";
        public override string RosMessageName => k_RosMessageName;

        // feedback
        public int[] sequence;

        public ShroudTestFeedback()
        {
            this.sequence = new int[0];
        }

        public ShroudTestFeedback(int[] sequence)
        {
            this.sequence = sequence;
        }

        public static ShroudTestFeedback Deserialize(MessageDeserializer deserializer) => new ShroudTestFeedback(deserializer);

        private ShroudTestFeedback(MessageDeserializer deserializer)
        {
            deserializer.Read(out this.sequence, sizeof(int), deserializer.ReadLength());
        }

        public override void SerializeTo(MessageSerializer serializer)
        {
            serializer.WriteLength(this.sequence);
            serializer.Write(this.sequence);
        }

        public override string ToString()
        {
            return "ShroudTestFeedback: " +
            "\nsequence: " + System.String.Join(", ", sequence.ToList());
        }

#if UNITY_EDITOR
        [UnityEditor.InitializeOnLoadMethod]
#else
        [UnityEngine.RuntimeInitializeOnLoadMethod]
#endif
        public static void Register()
        {
            MessageRegistry.Register(k_RosMessageName, Deserialize, MessageSubtopic.Feedback);
        }
    }
}