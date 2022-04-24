using UnityEngine;
using System;

public class Thruster : MonoBehaviour
{

    public string type = "T200";

    public GameObject thrusterVectorVisualizer;

    private Rigidbody vehicle;

    private float rhoWater = 1027f;

    private double actingForce;

    private Vector3 worldCoord;

    void Awake()
    {
        vehicle = gameObject.GetComponentInParent<Rigidbody>();


        //Debug.Log("" + position);
        //Debug.Log("" + gameObject.transform.position);
        
        //worldCoord = gameObject.transform.TransformPoint(transform.localPosition);

        //Debug.Log("" + worldCoord);

        if(thrusterVectorVisualizer != null){
            GameObject tmp = Instantiate(thrusterVectorVisualizer,transform);
            
            tmp.gameObject.transform.localScale = new Vector3(0.001f, 0.001f, 0.001f);
            thrusterVectorVisualizer = tmp;
        }
    }

    void Update(){
        //Debug.Log("" + position);
        //Debug.Log("" + gameObject.transform.position);
        
        //worldCoord = gameObject.transform.TransformPoint(transform.localPosition);

    }

    Boolean BelowWater(){
        return true;
    }

    public float ApplyForce(double force, int thrusterNum)
    {
        // check if thruster is below water before applying a force
        return setForce(force, BelowWater(), thrusterNum);
    }

    // receieve new thrust from the robot code in demanded units
    public float setForce(double demand, bool wantForce, int thrusterNum)
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
            worldCoord = transform.TransformPoint(transform.localPosition);
            Vector3 scaledForce = transform.forward * actingForce; // this number is not correct
            Debug.Log("Applying " + actingForce + " at thruster " + thrusterNum + " at rotation: " + scaledForce );
            vehicle.AddForceAtPosition(scaledForce, worldCoord, ForceMode.Impulse);
        }

        return actingForce;
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

        return (float)force/50;
    }
}