using UnityEngine;
using System.Collections;
using System.Collections.Generic;

namespace physics
{
    public class RobotForces : MonoBehaviour
    {
        [Tooltip("Game object to apply thruster forces")]
        public GameObject underWaterObj;

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

        }

        async void FixedUpdate()
        {
            for(int i = 0; i < thrusters.Count; i++)
            {
                thrusters[i].ApplyForce((double)lastThrust[i]);
            }
        }

        void recieveNewThrust(uint[] thrust){
            lastThrust = thrust;
        }
    }
}