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
            Component[] components = underWaterObj.GetComponents(typeof(Component));
            foreach (Component component in components)
            {
                string name = component.toString();
                if (name.ToLower().Contains("thruster"))
                {
                    thrusters.Add(gameObject.GetComponent<Thruster>(name));
                }
            }

        }

        void FixedUpdate()
        {}

    }
}