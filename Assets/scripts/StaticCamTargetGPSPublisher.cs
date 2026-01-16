using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Sensor;
using System;

public class StaticCamTargetGPSPublisher : MonoBehaviour
{
    [Header("Camera & Depth")]
    public Camera cam;
    public RenderTexture depthRT;

    [Header("Intrinsics")]
    public int width = 640;
    public int height = 640;
    public double fx = 900;
    public double fy = 900;
    public double cx = 320;
    public double cy = 320;

    [Header("World → GPS reference (must match PyBullet!)")]
    public Transform worldOrigin;
    public double originLatDeg = 37.7749;
    public double originLonDeg = -122.4194;
    public double originAltM = 30.0;

    [Header("ROS")]
    public string topic = "/static_cam/target_gps";
    public string frameId = "static_cam_target";

    [Header("Timing")]
    public int fps = 5;
    public int latencyMs = 30;

    private ROSConnection ros;
    private double nextTime;

    const double EarthRadius = 6378137.0;

    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        ros.RegisterPublisher<NavSatFixMsg>(topic);

        if (cam == null) Debug.LogError("Camera is NULL.");
        if (depthRT == null) Debug.LogError("depthRT is NULL.");

        nextTime = Time.timeAsDouble;
    }

    void Update()
    {
        if (Time.timeAsDouble < nextTime) return;
        nextTime += 1.0 / Mathf.Max(1, fps);

        double stamp = Time.timeAsDouble - latencyMs * 1e-3;

        // --- Get depth from center ---
        int u = width / 2;
        int v = height / 2;

        float depth = SampleDepthPixel(u, v);
        if (depth <= 0 || float.IsNaN(depth)) return;

        // --- Pixel + depth → camera space point ---
        Vector3 pCam = PixelToCam(u, v, depth);

        // --- Camera space → world space ---
        Vector3 pWorld = cam.transform.TransformPoint(pCam);

        // --- World → ENU relative to origin ---
        Vector3 originPos = worldOrigin != null ? worldOrigin.position : Vector3.zero;
        Vector3 dWorld = pWorld - originPos;

        // ❗修正 Unity 的 Z=South → North
        double dEast  = dWorld.x;
        double dNorth = -dWorld.z;
        double dUp    = dWorld.y;

        // --- ENU → GPS ---
        double lat0 = originLatDeg * Mathf.Deg2Rad;
        double dLat = dNorth / EarthRadius;
        double dLon = dEast / (EarthRadius * Math.Cos(lat0));

        double targetLat = originLatDeg + dLat * Mathf.Rad2Deg;
        double targetLon = originLonDeg + dLon * Mathf.Rad2Deg;
        double targetAlt = originAltM + dUp;

        // --- Pack into ROS message ---
        var msg = new NavSatFixMsg();
        SetStamp(ref msg, stamp);
        msg.header.frame_id = frameId;

        msg.latitude = targetLat;
        msg.longitude = targetLon;
        msg.altitude = targetAlt;

        msg.status.status = NavSatStatusMsg.STATUS_NO_FIX;
        msg.status.service = NavSatStatusMsg.SERVICE_GPS;
        msg.position_covariance = new double[9];
        msg.position_covariance_type = NavSatFixMsg.COVARIANCE_TYPE_UNKNOWN;

        ros.Publish(topic, msg);
    }

    float SampleDepthPixel(int u, int v)
    {
        RenderTexture prev = RenderTexture.active;
        RenderTexture.active = depthRT;

        Texture2D tex = new Texture2D(depthRT.width, depthRT.height, TextureFormat.RFloat, false, true);
        tex.ReadPixels(new Rect(0, 0, depthRT.width, depthRT.height), 0, 0);
        tex.Apply();

        float d = tex.GetPixel(u, v).r;

        UnityEngine.Object.Destroy(tex);
        RenderTexture.active = prev;
        return d;
    }

    Vector3 PixelToCam(int u, int v, float depth)
    {
        double x = ((double)u - cx) / fx * depth;
        double y = ((double)v - cy) / fy * depth;
        double z = depth;
        return new Vector3((float)x, (float)y, (float)z);
    }

    static void SetStamp(ref NavSatFixMsg msg, double stamp)
    {
        int sec = (int)Math.Floor(stamp);
        uint nsec = (uint)((stamp - sec) * 1e9);
        msg.header.stamp.sec = sec;
        msg.header.stamp.nanosec = nsec;
    }
}