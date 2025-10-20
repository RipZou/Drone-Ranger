using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Sensor;
using System;

public class OakLitePublisher : MonoBehaviour
{
    [Header("Cameras / RenderTextures")]
    public Camera colorCam;
    public Camera depthCam;
    public RenderTexture colorRT;
    public RenderTexture depthRT;

    [Header("Frame IDs")]
    public string rgbFrameId   = "oak_rgb_optical_frame";
    public string depthFrameId = "oak_depth_optical_frame";

    [Header("ROS Topics")]
    public string rgbTopic       = "/oak/rgb/image_raw";
    public string depthTopic     = "/oak/depth/image_rect_raw";
    public string rgbInfoTopic   = "/oak/rgb/camera_info";
    public string depthInfoTopic = "/oak/depth/camera_info";

    [Header("Intrinsics")]
    public int width = 1280, height = 720;
    public double fx = 900, fy = 900, cx = 640, cy = 360;
    public double[] D = new double[] { 0, 0, 0, 0, 0 };

    [Header("Timing")]
    public int fps = 30;          
    public int latencyMs = 40;   


    private ROSConnection ros;
    private double nextTime;      

    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();

        ros.RegisterPublisher<ImageMsg>(rgbTopic);
        ros.RegisterPublisher<ImageMsg>(depthTopic);
        ros.RegisterPublisher<CameraInfoMsg>(rgbInfoTopic);
        ros.RegisterPublisher<CameraInfoMsg>(depthInfoTopic);

        if (colorCam != null && depthCam != null)
        {
            var proj = BuildProjection(width, height, fx, fy, cx, cy, 0.2f, 10f);
            colorCam.projectionMatrix = proj;
            depthCam.projectionMatrix = proj;
        }

        InvokeRepeating(nameof(PublishCameraInfo), 0.2f, 1.0f);

        nextTime = Time.timeAsDouble;
    }

    void Update()
    {
        if (Time.timeAsDouble < nextTime) return;
        nextTime += 1.0 / Mathf.Max(1, fps);

        double stamp = Time.timeAsDouble - latencyMs * 1e-3;

        var rgbMsg = CaptureRGB(stamp);
        rgbMsg.header.frame_id = rgbFrameId;
        ros.Publish(rgbTopic, rgbMsg);

        var depthMsg = CaptureDepth(stamp);
        depthMsg.header.frame_id = depthFrameId;
        ros.Publish(depthTopic, depthMsg);
    }

    void PublishCameraInfo()
    {
        double stamp = Time.timeAsDouble - latencyMs * 1e-3;

        var rgbInfo = BuildCameraInfo(rgbFrameId);
        SetStamp(ref rgbInfo, stamp);
        ros.Publish(rgbInfoTopic, rgbInfo);

        var depthInfo = BuildCameraInfo(depthFrameId);
        SetStamp(ref depthInfo, stamp);
        ros.Publish(depthInfoTopic, depthInfo);
    }

    CameraInfoMsg BuildCameraInfo(string frameId)
    {
        var msg = new CameraInfoMsg();
        msg.header.frame_id = frameId;
        msg.width  = (uint)width;
        msg.height = (uint)height;
        msg.k = new double[] { fx, 0, cx,  0, fy, cy,  0, 0, 1 };
        msg.p = new double[] { fx, 0, cx, 0,
                                0, fy, cy, 0,
                                0, 0, 1,  0 };
        msg.r = new double[] { 1,0,0, 0,1,0, 0,0,1 };
        msg.d = D;
        msg.distortion_model = "plumb_bob";
        return msg;
    }

    static Matrix4x4 BuildProjection(int w, int h, double fx, double fy, double cx, double cy, float zn, float zf)
    {
        float A  = (float)(2.0 * fx / w);
        float B  = (float)(2.0 * fy / h);
        float C  = (float)(1.0 - 2.0 * cx / w);
        float Dd = (float)(2.0 * cy / h - 1.0);

        Matrix4x4 m = new Matrix4x4();
        m[0,0] = A;   m[0,1] = 0;   m[0,2] = C;                             m[0,3] = 0;
        m[1,0] = 0;   m[1,1] = B;   m[1,2] = Dd;                            m[1,3] = 0;
        m[2,0] = 0;   m[2,1] = 0;   m[2,2] = -(zf + zn) / (zf - zn);        m[2,3] = -(2f * zf * zn) / (zf - zn);
        m[3,0] = 0;   m[3,1] = 0;   m[3,2] = -1f;                           m[3,3] = 0;
        return m;
    }

    static void SetStamp(ref ImageMsg msg, double stamp)
    {
        int sec = (int)Math.Floor(stamp);
        uint nsec = (uint)Math.Round((stamp - Math.Floor(stamp)) * 1e9);
        if (nsec >= 1000000000u) { sec += 1; nsec -= 1000000000u; }
        msg.header.stamp.sec = sec;
        msg.header.stamp.nanosec = nsec;
    }

    static void SetStamp(ref CameraInfoMsg msg, double stamp)
    {
        int sec = (int)Math.Floor(stamp);
        uint nsec = (uint)Math.Round((stamp - Math.Floor(stamp)) * 1e9);
        if (nsec >= 1000000000u) { sec += 1; nsec -= 1000000000u; }
        msg.header.stamp.sec = sec;
        msg.header.stamp.nanosec = nsec;
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
        msg.header.frame_id = rgbFrameId;
        msg.height = (uint)colorRT.height;
        msg.width  = (uint)colorRT.width;
        msg.encoding = "rgb8";
        msg.is_bigendian = 0;
        msg.step = (uint)(colorRT.width * 3);
        msg.data = data;
        return msg;
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
        msg.header.frame_id = depthFrameId;
        msg.height = (uint)depthRT.height;
        msg.width  = (uint)depthRT.width;
        msg.encoding = "32FC1";
        msg.is_bigendian = 0;
        msg.step = (uint)(depthRT.width * sizeof(float));
        msg.data = bytes;
        return msg;
    }
}