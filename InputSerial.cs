using UnityEngine;
using System.Collections;
using System.Collections.Generic;
using System.Threading;
using System.IO.Ports;

public class InputSerial : MonoBehaviour
{
    public struct EnginesPacket
    {
        //public int throttle;
        //public float roll;
        //public float pitch;
        //public bool button1;
        public int LeftLever;
        public int RightLever;
    }
    public EnginesPacket CurrentEngineValue;

    enum PacketTypes
    {
        LeftLever = 0, RightLever
    };

    private static InputSerial thisinstance;
    public string ComPort = "";
   // public int[] values = new int[2];

    public SerialPort SerialPort;

    private Thread thread;
    bool looping = false;
    string data = "";
   // public int value;
   
    public static InputSerial instance
    {
        get
        {
            // If a singleton instance is null we must create one, otherwise
            // we return a reference to the existing one, thus ensuring there is
            // always exactly one instance.

            if (thisinstance == null)
            {
                GameObject go = new GameObject("InputInstance");

                // Get a reference to the component, this will be our singleton instance 
                thisinstance = go.AddComponent<InputSerial>();

                // Prevent this object from getting unloaded/destroyed when changing scenes
                DontDestroyOnLoad(go);

                // Run initialization function
                thisinstance.Init();
            }

            // Return the instance
            return thisinstance;
        }
    }

    public void Wake()
    {
        print(this.name + " awake");
    }

    private void Init()
    {
        // TODO auto detect com port
        ComPort = "COM3";
        looping = true;
        OpenPort();
        StartThread();
    }

    void OpenPort()
    {
        if (SerialPort == null)
        {
            SerialPort = new SerialPort(@"\\.\" + ComPort); // format to force Unity to recognize ports beyond COM9
            SerialPort.BaudRate = 9600;
            SerialPort.DataBits = 8;
            SerialPort.Parity = Parity.None;
            SerialPort.ReadTimeout = 1000; // miliseconds
        }

        try
        {
            SerialPort.Open();
            Debug.Log("Initialize Serial Port: " + ComPort);
        }

        catch (System.Exception error)
        {
            Debug.Log("Error opening " + ComPort + "\n" + error.Message);
        }
    }
    
    void ClosePort()
    {
        if (SerialPort != null && SerialPort.IsOpen)
        {
            SerialPort.Close();
        }
    }

    private void OnApplicationQuit()
    {
        if (SerialPort != null && SerialPort.IsOpen)
        {
            print("Close serialPort");
            SerialPort.Close();
        }

        StopThread();
    }

    public void StartThread()
    {
        thread = new Thread(ThreadLoop);
        thread.Start();
    }

    EnginesPacket ParseEngines(string message)
    {
        EnginesPacket Packet = new EnginesPacket();

        string[] ReadValues = message.Split(',');
        ReadValues = message.Split(",".ToCharArray(), System.StringSplitOptions.RemoveEmptyEntries);
        //values = message.Split(',').Select(sValue => sValue.Trim()).ToArray();

        bool success = ReadValues.Length == 2 ? true : false;
      //  print(ReadValues.Length);
       
        if (success)
        {
            success = success && int.TryParse(ReadValues[(int)PacketTypes.LeftLever], out Packet.LeftLever);
        }
        if (success)
        {
            success = success && int.TryParse(ReadValues[(int)PacketTypes.RightLever], out Packet.RightLever);
           // print(success);

        }

        if (success == false)
        {
            Debug.LogError("Bad Serial Packet: " + message);
        }

        return Packet;
    }

    // Thread Loop
    public void ThreadLoop()
    {
        print("Thread start");
        Debug.Log("hi");
        // Looping
        while (looping)
        {
           // print("looping");

            try
            {
                data = SerialPort.ReadLine();
                CurrentEngineValue = ParseEngines(data);
           
                //Debug.Log(data);
            }
            catch (System.Exception ex)
            {
                print(ex.Message);
            }

            //try
            //{
            //    SendSerial();
            //}
            //catch (System.Exception ex)
            //{
            //    //print(ex.Message);
            //}
        }
    }
    private void Update()
    {
        //print(CurrentEngineValue.LeftLever + "," + CurrentEngineValue.RightLever);
    }
    // Stop the thread (by setting the loop bool to false, causing the thread while loop to stop.
    public void StopThread()
    {
        //thread.Abort();
        lock (this)
        {
            looping = false;
        }
    }
}
