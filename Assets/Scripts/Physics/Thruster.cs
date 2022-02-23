using UnityEngine;
using System;

public class Thruster : MonoBehaviour
{
    [Tooltip("Game object to apply drag to")]
    public GameObject underWaterObj;

    [Tooltip("Local thruster position from base link")]
    public Vector3 position;

    [Tooltip("Local thruster orientation from base link")]
    public Vector3 orientation;

    public string type = "T200";

    private Rigidbody vehicle;

    private float rhoWater = 1027f;

    private double actingForce;

    private GameObject dummyThruster;

    void Start()
    {
        vehicle = gameObject.GetComponent<Rigidbody>();
        
        // spawn a child game object that represents the position of the thruster facing its forward direction
        // dummyThruster = 

        // set the thruster to be a child of the parent object
    }

    Boolean BelowWater(){
        return true;
    }

    public void ApplyForce()
    {
        // check if thruster is below water before applying a force
        if(BelowWater()){
            // apply the force to the rigidbody
            Vector3 robotThrustForce = new Vector3(0, 0, 0);

        }
    }

    // receieve new thrust from the robot code in demanded units
    public void setForce(double demand)
    {
        actingForce = 0;
        if (type.ToUpper().Contains("T200"))
        {
            actingForce = GetT200Thrust((int)demand);
        }
        else
        {
            actingForce = 0;
            Debug.Log("Thruster type not set, not applying force");
        }
    }

    // An approximated thrust curve in newtons for the T200 thruster from blue robotics
    double GetT200Thrust(int pwm)
    {
        double force = 0.0;
        double adjPwm = pwm / 1000.0;
        if (Math.Abs(pwm - 1500) > 25)
        {
            force = 366.55 * Math.Pow(adjPwm, 2) - 1084.9 * adjPwm + 804.31;
        }
        return force;
    }
}