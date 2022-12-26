using System;
using System.Collections;
using System.Collections.Generic;
using System.Threading;
using System.Threading.Tasks;
using UnityEngine;
using TMPro;
//using System.Runtime.Serialization.Formatters.Binary;
#if WINDOWS_UWP
using Windows.Networking.Sockets;
using Windows.Storage.Streams;
#endif

public class TCPClient : MonoBehaviour
{
    #region Unity Functions

    private void Awake()
    {
        ConnectionStatusLED.material.color = Color.red;
        gazeButton.Enabled = false;
    }
    private void OnApplicationFocus(bool focus)
    {
        if (!focus)
        {
#if WINDOWS_UWP
            StopConnection();
#endif
        }
    }
    #endregion // Unity Functions

    [SerializeField]
    List<string> hostIPAddresses = new List<string>();
    
    [SerializeField]
    string port;

    public Renderer ConnectionStatusLED;
    public TMP_Text ConnectButtonText;
    public bool Connected { get; private set; } = false;
    public string ConnectedHostIP { get; private set; }

    public GazeButton gazeButton;

    public ResearchModeVideoStream videoStream;

    public int PendingMessageCount { get; private set; } = 0;  // no need to be accurate so no atomic
    public int MaxPendingMessageCount = 20;

#if WINDOWS_UWP
    /**
     * Package structure:
     *  1-byte PREAMBLE (defined below)
     *  uint8_t package type (enum PackageType)
     *  NUL-terminated string for name
     *  uint32_t content size
     *  bytes
     */

    private const byte Preamble = 0xCE;
    enum PackageType : byte {
        SignleString,
        SingleInt,
        Bytes,
        ListOfString,
        PackageTypeCount,
    };

    StreamSocket socket = null;
    public DataWriter dw;
    public DataReader dr;

    private Task<(Color, float[], string)> readingTask = null;

    private async void StartConnection()
    {
        if (ConnectionStatusLED.material.color == Color.yellow) return;  // already trying to connect
        ConnectionStatusLED.material.color = Color.yellow;

        foreach(var hostIPAddress in hostIPAddresses)
        {
            try
            {
                if(socket != null) socket.Dispose();
                socket = new StreamSocket();

                var hostName = new Windows.Networking.HostName(hostIPAddress);
                ConnectButtonText.text = "Trying " + hostIPAddress;

                var cts = new CancellationTokenSource();
                cts.CancelAfter(2000);  // cancel after 2 seconds
                var connectAsync = socket.ConnectAsync(hostName, port);

                var connectTask = connectAsync.AsTask(cts.Token);
                try
                {
                    await connectTask;   
                }
                catch (TaskCanceledException)
                {
                    socket.Dispose();
                    continue;  // try the next host
                }
            }
            catch (Exception ex)
            {
                SocketErrorStatus webErrorStatus = SocketError.GetStatus(ex.GetBaseException().HResult);
                ConnectButtonText.text = webErrorStatus.ToString() != "Unknown" ? webErrorStatus.ToString() : ex.Message;
                continue;
            }
                
            // Connected
            ConnectedHostIP = hostIPAddress;
            dw = new DataWriter(socket.OutputStream);
            dw.UnicodeEncoding = Windows.Storage.Streams.UnicodeEncoding.Utf8;
            dw.ByteOrder = Windows.Storage.Streams.ByteOrder.LittleEndian;


            dr = new DataReader(socket.InputStream);
            dr.InputStreamOptions = InputStreamOptions.None;
            dr.UnicodeEncoding = Windows.Storage.Streams.UnicodeEncoding.Utf8;
            dr.ByteOrder = Windows.Storage.Streams.ByteOrder.LittleEndian;

            Connected = true;
            ConnectionStatusLED.material.color = Color.green;
            ConnectButtonText.text = "Disconnect from " + ConnectedHostIP;

            gazeButton.Enabled = true;

            return;
        }

        // Fail to connect
        socket?.Dispose();
        ConnectionStatusLED.material.color = Color.red;
        ConnectButtonText.text = "Connect to Server";
    }

    private void StopConnection()
    {
        try
        {
            if (videoStream.RawDataStreaming) videoStream.ToggleRawDataStreamingEvent();
            if (videoStream.PointCloudStreaming) videoStream.TogglePointCloudStreamingEvent();

            Connected = false;

            gazeButton.Enabled = false;

            ConnectionStatusLED.material.color = Color.red;
            ConnectButtonText.text = "Connect to Server";

            PendingMessageCount = 0;

            if (gazeButton.state == GazeButton.State.Scanning)
            {
                gazeButton.TransitionState(true);  // change state but no triggering action
            }

            dw?.DetachStream();
            dw?.Dispose();
            dw = null;

            dr?.DetachStream();
            dr?.Dispose();
            dr = null;

            readingTask = null;  // discard the task

            socket?.Dispose();
        }
        catch (Exception ex)
        {
            videoStream.text.text += "[TCPClient] " + ex.Message + "\n";
        }
    }

    public async void SendUINT16Async(string header, ushort[] data, bool canDrop = true)
    {
        if (canDrop && PendingMessageCount >= MaxPendingMessageCount) return;

        PendingMessageCount++;
        try
        {
            // Write header
            dw.WriteByte(Preamble);
            dw.WriteByte((byte) PackageType.Bytes);
            dw.WriteString(header);
            dw.WriteByte(0);  // NUL-termination

            // Write length in bytes
            dw.WriteInt32(data.Length * sizeof(ushort));

            // Write actual data
            dw.WriteBytes(UINT16ToBytes(data));

            // Send out
            await dw.StoreAsync();
            await dw.FlushAsync();
        }
        catch (Exception ex)
        {
            SocketErrorStatus webErrorStatus = SocketError.GetStatus(ex.GetBaseException().HResult);
            Debug.Log(webErrorStatus.ToString() != "Unknown" ? webErrorStatus.ToString() : ex.Message);
        }
        PendingMessageCount--;
    }

    public async void SendRawDataAsync(string header, long timestamp, ushort[] ahatDepth, float[] rigToWorld, float[] interaction, bool canDrop = true)
    {
        if (canDrop && PendingMessageCount >= MaxPendingMessageCount) return;

        PendingMessageCount++;
        try
        {
            // Write header
            dw.WriteByte(Preamble);
            dw.WriteByte((byte) PackageType.Bytes);
            dw.WriteString(header);
            dw.WriteByte(0);  // NUL-termination

            // Write length in bytes
            dw.WriteInt32(sizeof(long) + ahatDepth.Length * sizeof(ushort) + rigToWorld.Length * sizeof(float) + interaction.Length * sizeof(float));

            // Write actual data
            dw.WriteInt64(timestamp);
            dw.WriteBytes(UINT16ToBytes(ahatDepth));
            dw.WriteBytes(FloatToBytes(rigToWorld));
            dw.WriteBytes(FloatToBytes(interaction));

            // Send out
            await dw.StoreAsync();
            await dw.FlushAsync();
        }
        catch (Exception ex)
        {
            SocketErrorStatus webErrorStatus = SocketError.GetStatus(ex.GetBaseException().HResult);
            Debug.Log(webErrorStatus.ToString() != "Unknown" ? webErrorStatus.ToString() : ex.Message);
        }
        PendingMessageCount--;
    }

    public async void SendFloatAsync(string header, float[] data, bool canDrop = true)
    {
        if (canDrop && PendingMessageCount >= MaxPendingMessageCount) return;

        PendingMessageCount++;
        try
        {
            // Write header
            dw.WriteByte(Preamble);
            dw.WriteByte((byte) PackageType.Bytes);
            dw.WriteString(header);
            dw.WriteByte(0);  // NUL-termination

            // Write length in bytes
            dw.WriteInt32(data.Length * sizeof(float));
            
            // Write actual data
            dw.WriteBytes(FloatToBytes(data));

            // Send out
            await dw.StoreAsync();
            await dw.FlushAsync();
        }
        catch (Exception ex)
        {
            SocketErrorStatus webErrorStatus = SocketError.GetStatus(ex.GetBaseException().HResult);
            Debug.Log(webErrorStatus.ToString() != "Unknown" ? webErrorStatus.ToString() : ex.Message);
        }
        PendingMessageCount--;
    }

    // NOTE: [Zikai] the convention is inconsistent with above, but we are not using them anyway for now
    public async void SendSpatialImageAsync(byte[] LFImage, byte[] RFImage, long ts_left, long ts_right)
    {
        if (PendingMessageCount >= MaxPendingMessageCount) return;

        PendingMessageCount++;
        try
        {
            // Write header
            dw.WriteString("f"); // header "f"

            // Write Length
            dw.WriteInt32(LFImage.Length + RFImage.Length);
            dw.WriteInt64(ts_left);
            dw.WriteInt64(ts_right);

            // Write actual data
            dw.WriteBytes(LFImage);
            dw.WriteBytes(RFImage);

            // Send out
            await dw.StoreAsync();
            await dw.FlushAsync();
        }
        catch (Exception ex)
        {
            SocketErrorStatus webErrorStatus = SocketError.GetStatus(ex.GetBaseException().HResult);
            Debug.Log(webErrorStatus.ToString() != "Unknown" ? webErrorStatus.ToString() : ex.Message);
        }
        PendingMessageCount--;
    }


    public async void SendSpatialImageAsync(byte[] LRFImage, long ts_left, long ts_right)
    {
        if (PendingMessageCount >= MaxPendingMessageCount) return;

        PendingMessageCount++;
        try
        {
            // Write header
            dw.WriteString("f"); // header "f"

            // Write Timestamp and Length
            dw.WriteInt32(LRFImage.Length);
            dw.WriteInt64(ts_left);
            dw.WriteInt64(ts_right);

            // Write actual data
            dw.WriteBytes(LRFImage);

            // Send out
            await dw.StoreAsync();
            await dw.FlushAsync();
        }
        catch (Exception ex)
        {
            SocketErrorStatus webErrorStatus = SocketError.GetStatus(ex.GetBaseException().HResult);
            Debug.Log(webErrorStatus.ToString() != "Unknown" ? webErrorStatus.ToString() : ex.Message);
        }
        PendingMessageCount--;
    }

    private async Task<(Color, float[], string)> ReadPointCloudFromPC(DataReader dr) 
    {
        try 
        {
            if (dr.UnconsumedBufferLength > 0)  // if state not clean, clear all buffer
            {
                dr.ReadBuffer(dr.UnconsumedBufferLength);
                return (new Color(), null, "Recovering...");
            }

            // Keep finding Preamble
            while (true)
            {
                await dr.LoadAsync(1);
                if (dr.ReadByte() != Preamble)
                {
                    // videoStream.text.text += "Not Preamble" + "\n";
                    continue;
                }

                await dr.LoadAsync(1);
                byte t = dr.ReadByte();
                if (t != (byte) PackageType.Bytes) 
                {
                    // videoStream.text.text += "Unexpected type: " + t.ToString() + "\n";
                    continue;
                    // return (new Color(), null, "Unexpected type: " + t.ToString());
                }

                await dr.LoadAsync(2);  // NOTICE: here we hard code the name to be length-1 string
                byte identifier = dr.ReadByte();
                if (identifier != (byte) 'P') 
                {
                    // videoStream.text.text += "Unknown identifier: " + identifier.ToString() + "\n";
                    continue;
                    // return (new Color(), null, "Unknown identifier: " + identifier.ToString());
                }
                identifier = dr.ReadByte();
                if (identifier != (byte) '\0') 
                {
                    // videoStream.text.text += "Unknown ending: " + identifier.ToString() + "\n";
                    continue;
                    // return (new Color(), null, "Unknown ending: " + identifier.ToString());
                }

                break;
            }

            await dr.LoadAsync(4);
            uint bytesToRead = dr.ReadUInt32();
            await dr.LoadAsync(bytesToRead);

            // Sanity check
            if (bytesToRead < 3 * sizeof(float) || bytesToRead % sizeof(float) != 0) {
                return (new Color(), null, "Invalid float array size (bytes): " + bytesToRead.ToString());
            }

            if (bytesToRead > 1000000 * sizeof(float)) {  // normally like 35000
                return (new Color(), null, "Unreasonably large point cloud: " + bytesToRead.ToString());
            }

            // Point color
            Color pointColor = new Color(dr.ReadSingle(), dr.ReadSingle(), dr.ReadSingle());
            bytesToRead -= 3 * sizeof(float);

            // Points
            var bytes = new byte[bytesToRead];
            dr.ReadBytes(bytes);
            return (pointColor, BytesToFloat(bytes), null);
        }
        catch (Exception ex)
        {
            return (new Color(), null, ex.Message);
        }
    }
#endif

    void LateUpdate()
    {
#if WINDOWS_UWP
        if (Connected && dr != null) 
        {
            if (readingTask != null && readingTask.IsCompleted)
            {
                (Color color, float[] pointCloud, string err) = readingTask.Result;
                if (err != null)
                {
                    videoStream.text.text += "[TCPClient] " + err + "\n";
                } 
                else
                {
                    videoStream.pointColor = color;
                    videoStream.RenderPointCloudFromPC(pointCloud);
                }
                readingTask = null;
            }

            if (readingTask == null) 
            {
                readingTask = ReadPointCloudFromPC(dr);  // restart async receiving
            }
        }
#endif
    }


    #region Helper Function
    byte[] UINT16ToBytes(ushort[] data)
    {
        byte[] ushortInBytes = new byte[data.Length * sizeof(ushort)];
        System.Buffer.BlockCopy(data, 0, ushortInBytes, 0, ushortInBytes.Length);
        return ushortInBytes;
    }
    byte[] FloatToBytes(float[] data)
    {
        byte[] floatInBytes = new byte[data.Length * sizeof(float)];
        System.Buffer.BlockCopy(data, 0, floatInBytes, 0, floatInBytes.Length);
        return floatInBytes;
    }
    float[] BytesToFloat(byte[] data)
    {
        if ((data.Length % sizeof(float)) != 0) return null;
        float[] f = new float[data.Length / sizeof(float)];
        System.Buffer.BlockCopy(data, 0, f, 0, data.Length);
        return f;
    }
    #endregion

    #region Button Callback
    public void ConnectToServerEvent()
    {
#if WINDOWS_UWP
        if (!Connected) StartConnection();
        else StopConnection();
#endif
    }
    #endregion
}
