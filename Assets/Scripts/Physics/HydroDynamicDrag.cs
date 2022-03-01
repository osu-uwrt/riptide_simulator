using UnityEngine;
using System.Collections;
using System.Collections.Generic;

namespace physics
{
    public class HydroDynamicDrag : MonoBehaviour
    {
        [Tooltip("Game object to apply drag to")]
        public GameObject underWaterObj;

        [Tooltip("linear drag coeficients")]
        public Vector3 linearLinearDrag, linearQuadraticDrag;
        [Tooltip("angular drag coeficients")]
        public Vector3 angularLinearDrag, angularQuadraticDrag;

        private Rigidbody vehicle;

        //The density of the water the vehicle is traveling in
        private float rhoWater = 1027f;

        void Start()
        {
            vehicle = gameObject.GetComponent<Rigidbody>();
        }

        void FixedUpdate()
        {
            // calcluate the forces applied to the rigidbody
            Vector3 linearForces = calcLinearForces();
            Vector3 angularTorqe = calcAngularTorques();

            // apply the forces about the rigidbody COM
            vehicle.AddRelativeForce(linearForces, ForceMode.Impulse);
            vehicle.AddRelativeTorque(angularTorqe, ForceMode.Impulse);
        }

        // Calculate true density based on how much of the vehicle is currently submerged
        double getAdjRho()
        {
            return rhoWater * 1.0;
        }

        Vector3 calcLinearForces()
        {
            //get the local velocity of the vehicle
            Vector3 localVelocity = vehicle.GetRelativePointVelocity(new Vector3(0, 0, 0));

            // Calculate the linear drag forces
            Vector3 linDrag = new Vector3(
                localVelocity.x * linearLinearDrag.x,
                localVelocity.y * linearLinearDrag.y,
                localVelocity.z * linearLinearDrag.z
            );

            // Calculate the quardratic drag forces
            Vector3 quadDrag = new Vector3(
                localVelocity.x * localVelocity.x * linearQuadraticDrag.x,
                localVelocity.y * localVelocity.y * linearQuadraticDrag.y,
                localVelocity.z * localVelocity.z * linearQuadraticDrag.z
            );

            // flip the sign here to make this negative feedback
            return -(linDrag + quadDrag);
        }

        Vector3 calcAngularTorques()
        {
            Vector3 localAngVel = vehicle.angularVelocity;

            // Calculate the linear drag forces
            Vector3 linDrag = new Vector3(
                localAngVel.x * angularLinearDrag.x,
                localAngVel.y * angularLinearDrag.y,
                localAngVel.z * angularLinearDrag.z
            );

            // Calculate the quardratic drag forces
            Vector3 quadDrag = new Vector3(
                localAngVel.x * localAngVel.x * angularQuadraticDrag.x,
                localAngVel.y * localAngVel.y * angularQuadraticDrag.y,
                localAngVel.z * localAngVel.z * angularQuadraticDrag.z
            );

            // flip the sign here to make this negative feedback
            return -(linDrag + quadDrag);
        }
    }
}