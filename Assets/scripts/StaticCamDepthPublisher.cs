using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Sensor;
using System;

public class StaticCamDepthPublisher : MonoBehaviour
{
    [Header("Camera / RenderTexture")]
    public Camera cam;
    public RenderTexture depthRT;

    [Header("ROS")]
    public string depthTopic = "/static_cam/depth/image_raw";
    public string depthInfoTopic = "/static_cam/depth/camera_info";
    public string frameId = "static_cam_depth_optical_frame";

    [Header("Intrinsics")]
    public int width = 640, height = 640;
    public double fx = 900, fy = 900, cx = 320, cy = 320;

    public double[] D = new double[] { 0, 0, 0, 0, 0 };

    [Header("Timing")]
    public int fps = 15;
    public int latencyMs = 30;

    private ROSConnection ros;
    private double nextPubTime;

    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();

        ros.RegisterPublisher<ImageMsg>(depthTopic);
        ros.RegisterPublisher<CameraInfoMsg>(depthInfoTopic);

        nextPubTime = Time.timeAsDouble;

        InvokeRepeating(nameof(PublishCameraInfo), 0.2f, 1f);
    }

    void Update()
    {
        if (Time.timeAsDouble < nextPubTime) return;
        nextPubTime += 1.0 / fps;

        double stamp = Time.timeAsDouble - latencyMs * 1e-3;

        var msg = CaptureDepth(stamp);
        ros.Publish(depthTopic, msg);
    }

    ImageMsg CaptureDepth(double stamp)
    {
        var prev = RenderTexture.active;
        RenderTexture.active = depthRT;

        var tex = new Texture2D(depthRT.width, depthRT.height, TextureFormat.RFloat, false, true);
        tex.ReadPixels(new Rect(0, 0, depthRT.width, depthRT.height), 0, 0);
        tex.Apply();

        float[] raw = tex.GetRawTextureData<float>().ToArray();
        Destroy(tex);

        RenderTexture.active = prev;

        byte[] bytes = new byte[raw.Length * sizeof(float)];
        Buffer.BlockCopy(raw, 0, bytes, 0, bytes.Length);

        var msg = new ImageMsg();
        SetStamp(ref msg, stamp);

        msg.header.frame_id = frameId;
        msg.height = (uint)depthRT.height;
        msg.width = (uint)depthRT.width;
        msg.encoding = "32FC1";
        msg.step = (uint)(depthRT.width * sizeof(float));
        msg.is_bigendian = 0;
        msg.data = bytes;

        return msg;
    }

    void PublishCameraInfo()
    {
        double stamp = Time.timeAsDouble - latencyMs * 1e-3;
        var info = BuildCameraInfo();
        SetStamp(ref info, stamp);
        ros.Publish(depthInfoTopic, info);
    }

    CameraInfoMsg BuildCameraInfo()
    {
        var msg = new CameraInfoMsg();
        msg.header.frame_id = frameId;
        msg.width = (uint)width;
        msg.height = (uint)height;

        msg.k = new double[] { fx, 0, cx,
                               0, fy, cy,
                               0, 0, 1 };

        msg.r = new double[] { 1,0,0,
                               0,1,0,
                               0,0,1 };

        msg.p = new double[] { fx, 0, cx, 0,
                               0, fy, cy, 0,
                               0, 0, 1, 0 };

        msg.d = D;
        msg.distortion_model = "plumb_bob";

        return msg;
    }

    static void SetStamp(ref ImageMsg msg, double stamp)
    {
        int sec = (int)Math.Floor(stamp);
        uint nsec = (uint)((stamp - sec) * 1e9);
        msg.header.stamp.sec = sec;
        msg.header.stamp.nanosec = nsec;
    }

    static void SetStamp(ref CameraInfoMsg msg, double stamp)
    {
        int sec = (int)Math.Floor(stamp);
        uint nsec = (uint)((stamp - sec) * 1e9);
        msg.header.stamp.sec = sec;
        msg.header.stamp.nanosec = nsec;
    }
}