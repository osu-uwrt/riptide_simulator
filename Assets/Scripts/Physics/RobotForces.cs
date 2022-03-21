using UnityEngine;
using System.Collections;
using System.Collections.Generic;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Std;

namespace physics
{
    public class RobotForces : MonoBehaviour
    {
        [Tooltip("Game object to apply thruster forces")]
        public GameObject underWaterObj;

        [Tooltip("Thruster forces topic")]
        public string forcesTopic = "/puddles/thruster_forces";

        ROSConnection ros;

        public Vector3 robotCOM;

        private Rigidbody vehicle;

        private List<Thruster> thrusters;
        private uint[] lastThrust;

        void Start()
        {
            vehicle = gameObject.GetComponent<Rigidbody>();
            vehicle.centerOfMass = robotCOM;

            // grab all of the thrusters on the vehicle to get their positions and force capabilities
            thrusters = new List<Thruster>(FindObjectsOfType<Thruster>());
            
            //create ros connection
            ros = ROSConnection.GetOrCreateInstance();
            ros.Subscribe<Float32MultiArrayMsg>(forcesTopic, recieveNewThrust);
            
        }

        async void FixedUpdate()
        {
            for(int i = 0; i < thrusters.Count; i++)
            {
                thrusters[i].ApplyForce((double)lastThrust[i]);
                
            }
        }

        async void recieveNewThrust(Float32MultiArrayMsg msg) {
            Debug.Log("received thrust");
            
            uint[] thrust = new uint[8];
            for(int i=0; i<msg.data.Length; i++) {
                thrust[i] = (uint) msg.data[i];
            }

            lastThrust = thrust;
        }
    }
}