using System;
using System.Xml.Serialization;
using RosMessageTypes.BuiltinInterfaces;
using RosMessageTypes.Geometry;
using RosMessageTypes.Rosgraph;
using RosMessageTypes.Std;
using Unity.Robotics.ROSTCPConnector;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;
using Unity.VisualScripting;
using UnityEngine;



public class ROSDataSender : MonoBehaviour
{
    public string remoteIP = "140.93.1.220";   // PC IP address, laas secure: "140.93.89.215", Desktop IP address which I got from Guilhem: "140.93.1.173", work laptop: 140.93.97.25
    public int remotePort = 5000;               // UDP port
    public ROSConnection ros;

    public string headTopic = "/quest/head_pose";
    public string leftTopic = "/quest/left_controller_pose";
    public string rightTopic = "/quest/right_controller_pose";
    public string analogTopic = "/quest/analog";
    public string digitalTopic = "/quest/buttons";

    // Anchors
    public Transform headAnchor;
    public Transform leftControllerAnchor;
    public Transform rightControllerAnchor;
    private TouchScreenKeyboard overlayKeyboard;


    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        ros.Connect(remoteIP, remotePort);
        ros.RegisterPublisher<PoseStampedMsg>(headTopic);
        ros.RegisterPublisher<PoseStampedMsg>(leftTopic);
        ros.RegisterPublisher<PoseStampedMsg>(rightTopic);
        ros.RegisterPublisher<Float32MultiArrayMsg>(analogTopic);
        ros.RegisterPublisher<Int32MultiArrayMsg>(digitalTopic);

        Debug.Log("Successfully Connected");
        
        headAnchor = GameObject.Find("CenterEyeAnchor").transform;
        leftControllerAnchor = GameObject.Find("LeftControllerAnchor").transform;
        rightControllerAnchor = GameObject.Find("RightControllerAnchor").transform;
    }

    void Update()
    {
        if (ros.HasConnectionError)
        {
            Debug.Log("ROS Connection error, skipping publish");
            return;
        }

        OVRInput.Update();

        PublishPose(headTopic, headAnchor);
        PublishPose(leftTopic, leftControllerAnchor);
        PublishPose(rightTopic, rightControllerAnchor);

        PublishAnalog();
        PublishDigital();

        Debug.Log("Data sent");
    }
    TimeMsg GetUnityTimeMsg()
    {
        double t = Time.realtimeSinceStartupAsDouble;
        int sec = (int)Math.Floor(t);
        uint nanosec = (uint)((t - sec) * 1e-9);
        return new TimeMsg(sec, nanosec);
    }
    void PublishPose(string topic, Transform t)
    {
        Matrix4x4 m = t.localToWorldMatrix;
        Vector3 pos = m.GetColumn(3);
        Quaternion rot = m.rotation;

        PoseStampedMsg msg = new PoseStampedMsg()
        {
            header = new HeaderMsg
            {
                stamp = GetUnityTimeMsg(),
                frame_id = "world"
            },
            pose = new PoseMsg
            {
                position = new PointMsg(
                    pos.x,
                    pos.y,
                    pos.z),

                orientation = new QuaternionMsg(
                    rot.x,
                    rot.y,
                    rot.z,
                    rot.w)
            }
        };
        ros.Publish(topic, msg);
    }

    void PublishAnalog()
    {
        Vector2 leftJoy = OVRInput.Get(OVRInput.Axis2D.PrimaryThumbstick, OVRInput.Controller.LTouch);
        Vector2 rightJoy = OVRInput.Get(OVRInput.Axis2D.PrimaryThumbstick, OVRInput.Controller.RTouch);
        float leftIndex = OVRInput.Get(OVRInput.Axis1D.PrimaryIndexTrigger, OVRInput.Controller.LTouch);
        float rightIndex = OVRInput.Get(OVRInput.Axis1D.PrimaryIndexTrigger, OVRInput.Controller.RTouch);
        float leftGrip = OVRInput.Get(OVRInput.Axis1D.PrimaryHandTrigger, OVRInput.Controller.LTouch);
        float rightGrip = OVRInput.Get(OVRInput.Axis1D.PrimaryHandTrigger, OVRInput.Controller.RTouch);
        Float32MultiArrayMsg msg = new Float32MultiArrayMsg
        {
            data = new float[]
            {
                leftJoy.x, leftJoy.y,
                rightJoy.x, rightJoy.y,
                leftIndex, rightIndex,
                leftGrip, rightGrip,
            }
        };
        ros.Publish(analogTopic, msg);
    }

    void PublishDigital()
    {
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
        
        Int32MultiArrayMsg msg = new Int32MultiArrayMsg
        {
            data = new int[]
            {
                btnA, btnB, btnX, btnY,
                thumbL, thumbR, trigL, trigR, gripL, gripR

            }
        };
        ros.Publish(digitalTopic, msg);
    }
}