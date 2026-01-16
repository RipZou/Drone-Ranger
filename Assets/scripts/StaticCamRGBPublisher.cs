using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Sensor;
using System;

public class StaticCamRGBPublisher : MonoBehaviour
{
    [Header("Camera / RenderTexture")]
    public Camera cam;
    public RenderTexture colorRT;

    [Header("ROS")]
    public string rgbTopic = "/static_cam/rgb/image_raw";
    public string rgbInfoTopic = "/static_cam/rgb/camera_info";
    public string frameId = "static_cam_rgb_optical_frame";

    [Header("Intrinsics")]
    public int width = 1280, height = 720;
    public double fx = 900, fy = 900, cx = 640, cy = 360;

    public double[] D = new double[] { 0,0,0,0,0 };

    [Header("Timing")]
    public int fps = 15;
    public int latencyMs = 30;

    private ROSConnection ros;
    private double nextPubTime;

    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();

        ros.RegisterPublisher<ImageMsg>(rgbTopic);
        ros.RegisterPublisher<CameraInfoMsg>(rgbInfoTopic);

        nextPubTime = Time.timeAsDouble;

        // 每秒发布一次 CameraInfo
        InvokeRepeating(nameof(PublishCameraInfo), 0.2f, 1f);
    }

    void Update()
    {
        if (Time.timeAsDouble < nextPubTime) return;
        nextPubTime += 1.0 / fps;

        double stamp = Time.timeAsDouble - latencyMs * 1e-3;

        var msg = CaptureRGB(stamp);
        ros.Publish(rgbTopic, msg);
    }

    ImageMsg CaptureRGB(double stamp)
    {
        var prev = RenderTexture.active;
        RenderTexture.active = colorRT;

        var tex = new Texture2D(colorRT.width, colorRT.height, TextureFormat.RGB24, false, false);
        tex.ReadPixels(new Rect(0, 0, colorRT.width, colorRT.height), 0, 0);
        tex.Apply();
        byte[] data = tex.GetRawTextureData();
        Destroy(tex);

        RenderTexture.active = prev;

        var msg = new ImageMsg();

        SetStamp(ref msg, stamp);
        msg.header.frame_id = frameId;
        msg.height = (uint)colorRT.height;
        msg.width = (uint)colorRT.width;
        msg.encoding = "rgb8";
        msg.is_bigendian = 0;
        msg.step = (uint)(colorRT.width * 3);
        msg.data = data;
        return msg;
    }

    void PublishCameraInfo()
    {
        double stamp = Time.timeAsDouble - latencyMs * 1e-3;
        var info = BuildCameraInfo();
        SetStamp(ref info, stamp);
        ros.Publish(rgbInfoTopic, info);
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