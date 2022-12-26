using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Microsoft.MixedReality.Toolkit.Input;
using UnityEngine.UI;
using Microsoft.MixedReality.Toolkit;
using UnityEngine.Events;

public class GazeButton : MonoBehaviour {

    public bool Enabled = true;

    public GazeProvider gazeProvider;
    public LayerMask gazableMask;
    public Image image;
    public Color startColor;
    public Color stopColor;
    public ParticleSystem ps;

    private const float FILL_DURATION = 2;
    private const float TRANSITION_DURATION = 1.2f;
    private const float PRE_GAZE_DURATION = 0.4f;

    private bool isReallyGazing = false;
    private bool isGazing = false;
    private float preGazeTimer = 0;
    private float timer = 0;
    private float currentRotationY = 0;

    public enum State {
        Scanning,
        NotScanning,
        Transitioning,
    }
    public State state { get; private set; } = State.NotScanning;

    private bool isScanning = false;

    public UnityEvent scanningToogleEvent;

    void Start() {
        image.color = startColor;
        ps.Pause();
    }

    public void TransitionState(bool noAction = false) {
        image.fillAmount = 0;
        if (state == State.Scanning) {
            currentRotationY = 180;
            image.color = startColor;
            isScanning = false;
            if (!noAction) scanningToogleEvent.Invoke();
        } else {
            currentRotationY = 0;
            image.color = stopColor;
            isScanning = true;
            if (!noAction) scanningToogleEvent.Invoke();
        }
        timer = 0;
        state = State.Transitioning;
    }

    void Update() {
        // Debug.Log($"State: {state}");
        Ray ray = new Ray(CoreServices.InputSystem.GazeProvider.GazeOrigin, CoreServices.InputSystem.GazeProvider.GazeDirection);
        isGazing = Physics.Raycast(ray, out RaycastHit hitInfo, 10, gazableMask);
        isGazing = Enabled && isGazing;
        if (state == State.Transitioning) {
            timer += Time.deltaTime;
            transform.localRotation = Quaternion.Euler(0, currentRotationY + 180 * timer / TRANSITION_DURATION, 0);
            if (timer >= TRANSITION_DURATION) {
                timer = 0;
                state = isScanning ? State.Scanning : State.NotScanning;  // transition done
            }
        } else {
            if (isGazing) {
                if (isReallyGazing) {
                    //Debug.Log("Gazing!");
                    timer += Time.deltaTime;
                } else {
                    preGazeTimer += Time.deltaTime;
                    if(preGazeTimer >= PRE_GAZE_DURATION) {
                        isReallyGazing = true;
                        preGazeTimer = 0;
                    }
                }
            } else {
                timer -= Time.deltaTime;
                if (timer <= 0) {
                    isReallyGazing = false;
                }
                preGazeTimer = 0;
            }
            // Debug.Log(ray);
            timer = Mathf.Clamp(timer, 0, FILL_DURATION);
            image.fillAmount = timer / FILL_DURATION;
            if (timer >= FILL_DURATION) {
                TransitionState();
            }
        }
        if(isGazing) {
            ps.transform.position = hitInfo.point;
            if(Random.value < 0.15)
                ps.Emit(1);
            //if (ps.isPaused)
            //    ps.Play();
        } else {
            //ps.Pause();
        }
    }
}
