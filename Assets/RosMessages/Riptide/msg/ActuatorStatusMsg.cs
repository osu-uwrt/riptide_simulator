//Do not edit! This file was generated by Unity-ROS MessageGeneration.
using System;
using System.Linq;
using System.Collections.Generic;
using System.Text;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;

namespace RosMessageTypes.Riptide
{
    [Serializable]
    public class ActuatorStatusMsg : Message
    {
        public const string k_RosMessageName = "riptide_msgs2/ActuatorStatus";
        public override string RosMessageName => k_RosMessageName;

        public const byte CLAW_ERROR = 0;
        public const byte CLAW_UNKNOWN = 1;
        public const byte CLAW_OPENED = 2;
        public const byte CLAW_CLOSED = 3;
        public const byte TORPEDO_ERROR = 0;
        public const byte TORPEDO_DISARMED = 1;
        public const byte TORPEDO_CHARGING = 2;
        public const byte TORPEDO_CHARGED = 3;
        public const byte TORPEDO_FIRING = 4;
        public const byte TORPEDO_FIRED = 5;
        public const byte DROPPER_ERROR = 0;
        public const byte DROPPER_READY = 1;
        public const byte DROPPER_DROPPING = 2;
        public const byte DROPPER_DROPPED = 3;
        public byte claw_state;
        public byte torpedo1_state;
        public byte torpedo2_state;
        public byte dropper1_state;
        public byte dropper2_state;

        public ActuatorStatusMsg()
        {
            this.claw_state = 0;
            this.torpedo1_state = 0;
            this.torpedo2_state = 0;
            this.dropper1_state = 0;
            this.dropper2_state = 0;
        }

        public ActuatorStatusMsg(byte claw_state, byte torpedo1_state, byte torpedo2_state, byte dropper1_state, byte dropper2_state)
        {
            this.claw_state = claw_state;
            this.torpedo1_state = torpedo1_state;
            this.torpedo2_state = torpedo2_state;
            this.dropper1_state = dropper1_state;
            this.dropper2_state = dropper2_state;
        }

        public static ActuatorStatusMsg Deserialize(MessageDeserializer deserializer) => new ActuatorStatusMsg(deserializer);

        private ActuatorStatusMsg(MessageDeserializer deserializer)
        {
            deserializer.Read(out this.claw_state);
            deserializer.Read(out this.torpedo1_state);
            deserializer.Read(out this.torpedo2_state);
            deserializer.Read(out this.dropper1_state);
            deserializer.Read(out this.dropper2_state);
        }

        public override void SerializeTo(MessageSerializer serializer)
        {
            serializer.Write(this.claw_state);
            serializer.Write(this.torpedo1_state);
            serializer.Write(this.torpedo2_state);
            serializer.Write(this.dropper1_state);
            serializer.Write(this.dropper2_state);
        }

        public override string ToString()
        {
            return "ActuatorStatusMsg: " +
            "\nclaw_state: " + claw_state.ToString() +
            "\ntorpedo1_state: " + torpedo1_state.ToString() +
            "\ntorpedo2_state: " + torpedo2_state.ToString() +
            "\ndropper1_state: " + dropper1_state.ToString() +
            "\ndropper2_state: " + dropper2_state.ToString();
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