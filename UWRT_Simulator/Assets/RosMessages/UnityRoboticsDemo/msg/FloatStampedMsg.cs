//Do not edit! This file was generated by Unity-ROS MessageGeneration.
using System;
using System.Linq;
using System.Collections.Generic;
using System.Text;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;
using RosMessageTypes.Std;

namespace RosMessageTypes.UnityRoboticsDemo
{
    [Serializable]
    public class FloatStampedMsg : Message
    {
        public const string k_RosMessageName = "unity_robotics_demo_msgs/FloatStamped";
        public override string RosMessageName => k_RosMessageName;

        //  Copyright (c) 2016 The UUV Simulator Authors.
        //  All rights reserved.
        // 
        //  Licensed under the Apache License, Version 2.0 (the "License");
        //  you may not use this file except in compliance with the License.
        //  You may obtain a copy of the License at
        // 
        //      http://www.apache.org/licenses/LICENSE-2.0
        // 
        //  Unless required by applicable law or agreed to in writing, software
        //  distributed under the License is distributed on an "AS IS" BASIS,
        //  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
        //  See the License for the specific language governing permissions and
        //  limitations under the License.
        public HeaderMsg header;
        public double data;

        public FloatStampedMsg()
        {
            this.header = new HeaderMsg();
            this.data = 0.0;
        }

        public FloatStampedMsg(HeaderMsg header, double data)
        {
            this.header = header;
            this.data = data;
        }

        public static FloatStampedMsg Deserialize(MessageDeserializer deserializer) => new FloatStampedMsg(deserializer);

        private FloatStampedMsg(MessageDeserializer deserializer)
        {
            this.header = HeaderMsg.Deserialize(deserializer);
            deserializer.Read(out this.data);
        }

        public override void SerializeTo(MessageSerializer serializer)
        {
            serializer.Write(this.header);
            serializer.Write(this.data);
        }

        public override string ToString()
        {
            return "FloatStampedMsg: " +
            "\nheader: " + header.ToString() +
            "\ndata: " + data.ToString();
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
