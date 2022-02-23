using UnityEngine;
using System.Collections;
using System.Collections.Generic;

namespace physics
{
    public class RobotForces : MonoBehaviour
    {
        [Tooltip("Game object to apply thruster forces")]
        public GameObject underWaterObj;

        private Rigidbody vehicle;

        private List<Thruster> thrusters;

        void Start()
        {
            vehicle = gameObject.GetComponent<Rigidbody>();

            // grab all of the thrusters on the vehicle to get their positions and force capabilities
            thrusters = new List<Thruster>(FindObjectsOfType<Thruster>());

        }

        void FixedUpdate()
        {
            foreach(Thruster thruster in thrusters)
            {
                thruster.ApplyForce();
            }
        }

        void recieveNewThrust(int[] thrust){

        }

    }
}