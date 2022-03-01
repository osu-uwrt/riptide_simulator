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
        
    }

    Boolean BelowWater(){
        return true;
    }

    public void ApplyForce(double force)
    {
        // check if thruster is below water before applying a force
        setForce(force, BelowWater());
    }

    // receieve new thrust from the robot code in demanded units
    public void setForce(double demand, bool wantForce)
    {
        float actingForce = 0.0f;
        if (type.ToUpper().Contains("T200"))
        {
            actingForce = GetT200Thrust((int)demand);
        }
        else
        {
            actingForce = 0.0f;
            Debug.Log("Thruster type not set, not applying force");
        }

        if(wantForce){
            Vector3 worldCoord = gameObject.transform.TransformPoint(position);
            Vector3 scaledForce = Vector3.Normalize(orientation) * actingForce;

            vehicle.AddForceAtPosition(scaledForce, worldCoord, ForceMode.Impulse);
        }
    }

    // An approximated thrust curve in newtons for the T200 thruster from blue robotics
    float GetT200Thrust(int pwm)
    {
        double force = 0.0;
        double adjPwm = pwm / 1000.0;
        if (Math.Abs(pwm - 1500) > 25)
        {
            force = 366.55 * Math.Pow(adjPwm, 2) - 1084.9 * adjPwm + 804.31;
            force = force * Math.Sign(pwm - 1500);
        }
        return (float)force;
    }
}