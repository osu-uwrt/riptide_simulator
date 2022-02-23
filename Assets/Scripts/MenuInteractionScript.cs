using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;
using UnityEngine.Rendering;
using UnityEngine.Rendering.Universal;

public class MenuInteractionScript : MonoBehaviour
{
    public GameObject leftCameraObject;
    public GameObject rightCameraObject;
    public GameObject downwardCameraObject;
    public Toggle leftCameraToggle;
    public Toggle rightCameraToggle;
    public Toggle downwardCameraToggle;
    public Volume postProcessingVolume;

    private Vignette vignette;
    private FilmGrain filmGrain;

    void Start() {
        leftCameraToggle.enabled = leftCameraObject != null;
        rightCameraToggle.enabled = rightCameraObject != null;
        downwardCameraToggle.enabled = downwardCameraObject != null;

        postProcessingVolume.profile.TryGet<Vignette>(out vignette);
        postProcessingVolume.profile.TryGet<FilmGrain>(out filmGrain);
    }

    public void toggleLeftCamera(bool newValue) {
        leftCameraObject.SetActive(newValue);
    }

    public void toggleRightCamera(bool newValue) {
        rightCameraObject.SetActive(newValue);
    }

    public void toggleDownwardCamera(bool newValue) {
        downwardCameraObject.SetActive(newValue);
    }

    public void changeFilmGrainValue(float newValue) {
        filmGrain.intensity.value = newValue;
    }

    public void changeVignetteValue(float newValue) {
        vignette.intensity.value = newValue;
    }
}
