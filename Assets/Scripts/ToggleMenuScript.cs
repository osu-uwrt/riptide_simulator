using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class ToggleMenuScript : MonoBehaviour
{
    public Canvas canvasObject;
    public Camera mainCamera;

    void Update()
    {
        if (Input.GetKeyDown("tab")) {
            canvasObject.enabled = !canvasObject.enabled;
            mainCamera.GetComponent<PlayerCamera>().enabled = !mainCamera.GetComponent<PlayerCamera>().enabled;
        }
    }
}
