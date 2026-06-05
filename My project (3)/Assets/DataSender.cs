using UnityEngine;
using System;
using System.Net.Sockets;
using System.Collections.Generic;


public class DataSender : MonoBehaviour
{
    public string remoteIP = "140.93.1.220";   // my ownPC IP address, laas secure: "140.93.89.215", Desktop IP *Norikura address which I got from Guilhem: "140.93.1.173", work laptop: 140.93.97.25, miyanoura:140.93.1.220
    public int remotePort = 5000;               // UDP port
    private UdpClient udpClient;

    // Anchors
    public Transform headAnchor;
    public Transform leftControllerAnchor;
    public Transform rightControllerAnchor;
    private TouchScreenKeyboard overlayKeyboard;


    void Start()
    {
        
        udpClient = new UdpClient();
        try
        {
            udpClient.Connect(remoteIP, remotePort);
            Debug.Log("Successfully Connected");
        }
        catch (Exception e)
        {
            Debug.LogError("UDP Connect error: " + e);
        }
        headAnchor = GameObject.Find("CenterEyeAnchor").transform;
        leftControllerAnchor = GameObject.Find("LeftControllerAnchor").transform;
        rightControllerAnchor = GameObject.Find("RightControllerAnchor").transform;


    }

    void Update()
    {

        // Refresh OVR input state
        OVRInput.Update();

        // Get transforms
        Matrix4x4 headMatrix = headAnchor.localToWorldMatrix;
        Matrix4x4 leftMatrix = leftControllerAnchor.localToWorldMatrix;
        Matrix4x4 rightMatrix = rightControllerAnchor.localToWorldMatrix;

        // Prepare list for floats
        List<float> dataList = new List<float>();

        // --- Pose data (3 matrices × 16 floats each)
        AddMatrixToList(headMatrix, dataList);
        AddMatrixToList(leftMatrix, dataList);
        AddMatrixToList(rightMatrix, dataList);

        // --- Analog Inputs ---
        // Joysticks
        Vector2 leftJoy = OVRInput.Get(OVRInput.Axis2D.PrimaryThumbstick, OVRInput.Controller.LTouch);
        Vector2 rightJoy = OVRInput.Get(OVRInput.Axis2D.PrimaryThumbstick, OVRInput.Controller.RTouch);

        // Triggers
        float leftIndex = OVRInput.Get(OVRInput.Axis1D.PrimaryIndexTrigger, OVRInput.Controller.LTouch);
        float rightIndex = OVRInput.Get(OVRInput.Axis1D.PrimaryIndexTrigger, OVRInput.Controller.RTouch);
        float leftGrip = OVRInput.Get(OVRInput.Axis1D.PrimaryHandTrigger, OVRInput.Controller.LTouch);
        float rightGrip = OVRInput.Get(OVRInput.Axis1D.PrimaryHandTrigger, OVRInput.Controller.RTouch);

        dataList.AddRange(new float[] {
            leftJoy.x, leftJoy.y,
            rightJoy.x, rightJoy.y,
            leftIndex, rightIndex,
            leftGrip, rightGrip
        });

        // --- Digital Inputs (pack as 1 = pressed, 0 = not) ---
        // Common buttons: A/B (right), X/Y (left), thumbstick clicks, triggers
        int btnA = OVRInput.Get(OVRInput.Button.One, OVRInput.Controller.RTouch) ? 1 : 0;
        int btnB = OVRInput.Get(OVRInput.Button.Two, OVRInput.Controller.RTouch) ? 1 : 0;
        int btnX = OVRInput.Get(OVRInput.Button.One, OVRInput.Controller.LTouch) ? 1 : 0;
        int btnY = OVRInput.Get(OVRInput.Button.Two, OVRInput.Controller.LTouch) ? 1 : 0;

        int thumbL = OVRInput.Get(OVRInput.Button.PrimaryThumbstick, OVRInput.Controller.LTouch) ? 1 : 0;
        int thumbR = OVRInput.Get(OVRInput.Button.PrimaryThumbstick, OVRInput.Controller.RTouch) ? 1 : 0;
        int trigL = OVRInput.Get(OVRInput.Button.PrimaryIndexTrigger, OVRInput.Controller.LTouch) ? 1 : 0;
        int trigR = OVRInput.Get(OVRInput.Button.PrimaryIndexTrigger, OVRInput.Controller.RTouch) ? 1 : 0;
        int gripL = OVRInput.Get(OVRInput.Button.PrimaryHandTrigger, OVRInput.Controller.LTouch) ? 1 : 0;
        int gripR = OVRInput.Get(OVRInput.Button.PrimaryHandTrigger, OVRInput.Controller.RTouch) ? 1 : 0;

        // Store as floats for easy packing (Python side can read as ints)
        dataList.AddRange(new float[] {
            btnA, btnB, btnX, btnY,
            thumbL, thumbR, trigL, trigR, gripL, gripR
        });

        // --- Convert to bytes and send ---
        byte[] packet = new byte[dataList.Count * 4];
        Buffer.BlockCopy(dataList.ToArray(), 0, packet, 0, packet.Length);
        Debug.Log("Received Data");
        udpClient.Send(packet, packet.Length);
        Debug.Log("Data sent");
    }

    void AddMatrixToList(Matrix4x4 m, List<float> list)
    {
        list.Add(m.m00); list.Add(m.m01); list.Add(m.m02); list.Add(m.m03);
        list.Add(m.m10); list.Add(m.m11); list.Add(m.m12); list.Add(m.m13);
        list.Add(m.m20); list.Add(m.m21); list.Add(m.m22); list.Add(m.m23);
        list.Add(m.m30); list.Add(m.m31); list.Add(m.m32); list.Add(m.m33);
    }

    void OnApplicationQuit()
    {
        if (udpClient != null) udpClient.Close();
    }
}