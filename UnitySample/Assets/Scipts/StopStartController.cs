using System.Collections;
using System.Collections.Generic;
using System.IO;
using UnityEngine;

public class StopStartController : MonoBehaviour
{
    public bool IsScanning = false;
    public ResearchModeVideoStream stream;

    public void StartScanning() {
        stream.ToggleRawDataStreamingEvent();
        IsScanning = true;
        Debug.Log("Sent Start signal to 'ToggleRawDataStreamingEvent'!");
    }

    public void StopScanning() {
        stream.ToggleRawDataStreamingEvent();
        IsScanning = false;
        Debug.Log("Sent Stop signal to 'ToggleRawDataStreamingEvent'!");
        //TODO call internal 'Stop Scanning' Binding
    }
}
