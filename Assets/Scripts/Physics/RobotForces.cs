using UnityEngine;
using System.Collections.Generic;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Riptide;
using RosMessageTypes.Std;

namespace physics
{
    public class RobotForces : MonoBehaviour
    {
        [Tooltip("Game object to apply thruster forces")]
        public GameObject underWaterObj;

        [Tooltip("Thruster forces topic")]
        public string forcesTopic = "/tempest/command/pwm";

        public Vector3 robotCOM;

        private Rigidbody vehicle;

        private List<Thruster> thrusters;

        private ROSConnection ros;
        
        private uint[] lastThrust;
        [SerializeField]
        private uint thrusterZeroValue;

        void Start()
        {
            vehicle = gameObject.GetComponent<Rigidbody>();
            vehicle.centerOfMass = robotCOM;
            lastThrust = new uint[8];
            for(int i = 0; i<8; i++)
            {
                lastThrust[i] = thrusterZeroValue;
            }

            // grab all of the thrusters on the vehicle to get their positions and force capabilities
            thrusters = new List<Thruster>(FindObjectsOfType<Thruster>());
            
            //create ros connection
            ros = ROSConnection.GetOrCreateInstance();
            ros.Subscribe<PwmStampedMsg>(forcesTopic, recieveNewThrust);
        }

        async void FixedUpdate()
        {
            for(int i = 0; i < thrusters.Count; i++)
            {
                thrusters[i].ApplyForce((double)lastThrust[i]);
            }
        }

        void recieveNewThrust(PwmStampedMsg msg) {           
            for(int i=0; i<msg.pwm.Length; i++) {
                lastThrust[i] = msg.pwm[i];
            }
        }
    }
}