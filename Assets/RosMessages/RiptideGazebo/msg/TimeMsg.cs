//Do not edit! This file was generated by Unity-ROS MessageGeneration.
using System;
using System.Linq;
using System.Collections.Generic;
using System.Text;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;

namespace RosMessageTypes.RiptideGazebo
{
    [Serializable]
    public class TimeMsg : Message
    {
        public const string k_RosMessageName = "riptide_gazebo/Time";
        public override string RosMessageName => k_RosMessageName;

        public int sec;
        public int nsec;

        public TimeMsg()
        {
            this.sec = 0;
            this.nsec = 0;
        }

        public TimeMsg(int sec, int nsec)
        {
            this.sec = sec;
            this.nsec = nsec;
        }

        public static TimeMsg Deserialize(MessageDeserializer deserializer) => new TimeMsg(deserializer);

        private TimeMsg(MessageDeserializer deserializer)
        {
            deserializer.Read(out this.sec);
            deserializer.Read(out this.nsec);
        }

        public override void SerializeTo(MessageSerializer serializer)
        {
            serializer.Write(this.sec);
            serializer.Write(this.nsec);
        }

        public override string ToString()
        {
            return "TimeMsg: " +
            "\nsec: " + sec.ToString() +
            "\nnsec: " + nsec.ToString();
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