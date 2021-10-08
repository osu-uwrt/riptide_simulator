using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class KeyboardMovement : MonoBehaviour
{
	public float mSpeed;
	public GameObject vehicle;
	public Rigidbody rb;

    void Start()
    {
   	mSpeed = 5f;
    }

    void FixedUpdate()
    {
    	if (Input.GetKey (KeyCode.UpArrow)) {
    		rb.AddForce(transform.up * 100);
    	}
        if (Input.GetKey (KeyCode.DownArrow)) {
    		rb.AddForce(transform.up * -100);
    	}
        if (Input.GetKey (KeyCode.LeftArrow)) {
    		rb.AddForce(transform.right * -100);
    	}
        if (Input.GetKey (KeyCode.RightArrow)) {
    		rb.AddForce(transform.right * 100);
    	}

    }
}
