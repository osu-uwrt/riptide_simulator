//Do not edit! This file was generated by Unity-ROS MessageGeneration.
using System;
using System.Linq;
using System.Collections.Generic;
using System.Text;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;

namespace RosMessageTypes.Riptide
{
    [Serializable]
    public class SickTrickResult : Message
    {
        public const string k_RosMessageName = "riptide_msgs/SickTrick";
        public override string RosMessageName => k_RosMessageName;

        // result definition

        public SickTrickResult()
        {
        }
        public static SickTrickResult Deserialize(MessageDeserializer deserializer) => new SickTrickResult(deserializer);

        private SickTrickResult(MessageDeserializer deserializer)
        {
        }

        public override void SerializeTo(MessageSerializer serializer)
        {
        }

        public override string ToString()
        {
            return "SickTrickResult: ";
        }

#if UNITY_EDITOR
        [UnityEditor.InitializeOnLoadMethod]
#else
        [UnityEngine.RuntimeInitializeOnLoadMethod]
#endif
        public static void Register()
        {
            MessageRegistry.Register(k_RosMessageName, Deserialize, MessageSubtopic.Result);
        }
    }
}