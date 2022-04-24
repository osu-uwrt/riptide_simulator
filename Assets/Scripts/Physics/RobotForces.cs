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
        [SerializeField]
        private GameObject thrusterPrefab;
        [SerializeField]
        private List<Vector3> thrusterPositions;
        [SerializeField]
        private List<Vector3> thrusterRotations;
        [SerializeField]
        private Transform solidWorksOrigin;

        private ROSConnection ros;
        private List<GameObject> thrusterFab;
        private uint[] lastThrust;
        [SerializeField]
        private uint thrusterZeroValue;

        void Start()
        {
            thrusters = new List<Thruster>();
            thrusterFab= new List<GameObject>();
            if(thrusterPositions.Count != thrusterRotations.Count)
            {
                Debug.LogError("Thruster positions don't equal amount of thruster rotations!");
            }
            vehicle = gameObject.GetComponent<Rigidbody>();
            vehicle.centerOfMass = robotCOM;
            lastThrust = new uint[thrusterPositions.Count];
            for(int i = 0; i< thrusterPositions.Count; i++)
            {

                lastThrust[i] = thrusterZeroValue;
                GameObject thr = Instantiate(thrusterPrefab, transform,false);
                thr.transform.localPosition = (thrusterPositions[i] / 100)+solidWorksOrigin.localPosition;
                thr.transform.localRotation = Quaternion.Euler(thrusterRotations[i]);

                thrusters.Add(thr.GetComponent<Thruster>());
                thrusterFab.Add(thr.transform.GetChild(0).gameObject.transform.GetChild(0).gameObject);
            }

            
            //create ros connection
            ros = ROSConnection.GetOrCreateInstance();
            ros.Subscribe<PwmStampedMsg>(forcesTopic, recieveNewThrust);
        }

        void FixedUpdate()
        {
            for (int i = 0; i < thrusters.Count; i++)
            {
                float force = thrusters[i].ApplyForce((double)lastThrust[i], i);
                if(force > 0){
                    thrusterFab[i].GetComponent<Renderer>().material.color = Color.red;
                }
            }
        }

        void recieveNewThrust(PwmStampedMsg msg) {   
               
            for(int i=0; i<msg.pwm.Length; i++) {
                //print(i+" | "+msg.pwm[i]); 
                lastThrust[i] = msg.pwm[i];
            }
        }
    }
}