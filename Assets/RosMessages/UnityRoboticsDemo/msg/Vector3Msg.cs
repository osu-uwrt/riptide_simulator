//Do not edit! This file was generated by Unity-ROS MessageGeneration.
using System;
using System.Linq;
using System.Collections.Generic;
using System.Text;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;

namespace RosMessageTypes.UnityRoboticsDemo
{
    [Serializable]
    public class Vector3Msg : Message
    {
        public const string k_RosMessageName = "unity_robotics_demo_msgs/Vector3";
        public override string RosMessageName => k_RosMessageName;

        //  This represents a vector in free space. 
        //  It is only meant to represent a direction. Therefore, it does not
        //  make sense to apply a translation to it (e.g., when applying a 
        //  generic rigid transformation to a Vector3, tf2 will only apply the
        //  rotation). If you want your data to be translatable too, use the
        //  geometry_msgs/Point message instead.
        public double x;
        public double y;
        public double z;

        public Vector3Msg()
        {
            this.x = 0.0;
            this.y = 0.0;
            this.z = 0.0;
        }

        public Vector3Msg(double x, double y, double z)
        {
            this.x = x;
            this.y = y;
            this.z = z;
        }

        public static Vector3Msg Deserialize(MessageDeserializer deserializer) => new Vector3Msg(deserializer);

        private Vector3Msg(MessageDeserializer deserializer)
        {
            deserializer.Read(out this.x);
            deserializer.Read(out this.y);
            deserializer.Read(out this.z);
        }

        public override void SerializeTo(MessageSerializer serializer)
        {
            serializer.Write(this.x);
            serializer.Write(this.y);
            serializer.Write(this.z);
        }

        public override string ToString()
        {
            return "Vector3Msg: " +
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