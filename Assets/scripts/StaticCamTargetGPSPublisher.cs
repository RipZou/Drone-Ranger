using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Sensor;
using System;

public class StaticCamTargetGPSPublisher : MonoBehaviour
{
    [Header("Camera")]
    public Camera cam;

    [Header("World → GPS reference")]
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

    [Header("Debug")]
    public float maxRayDistance = 200f;

    private ROSConnection ros;
    private double nextTime;

    const double EarthRadius = 6378137.0;

    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        ros.RegisterPublisher<NavSatFixMsg>(topic);

        if (cam == null)
            Debug.LogError("[StaticCamTargetGPS] Camera is NULL");

        nextTime = Time.timeAsDouble;
    }

    void Update()
    {
        if (Time.timeAsDouble < nextTime) return;
        nextTime += 1.0 / Mathf.Max(1, fps);

        double stamp = Time.timeAsDouble - latencyMs * 1e-3;

        // ---------- 1. ray from camera center ----------
        Ray ray = cam.ViewportPointToRay(new Vector3(0.5f, 0.5f, 0f));

        if (!Physics.Raycast(ray, out RaycastHit hit, maxRayDistance))
            return;

        Vector3 pWorld = hit.point;

        // ---------- 2. debug visualization ----------
        Debug.DrawLine(ray.origin, pWorld, Color.red, 1.5f);

        // ---------- 3. world → ENU ----------
        Vector3 originPos = worldOrigin != null ? worldOrigin.position : Vector3.zero;
        Vector3 dWorld = pWorld - originPos;

        double dEast  = dWorld.x;
        double dNorth = -dWorld.z;   // Unity Z forward = South
        double dUp    = dWorld.y;

        // ---------- 4. ENU → GPS ----------
        double lat0 = originLatDeg * Mathf.Deg2Rad;
        double dLat = dNorth / EarthRadius;
        double dLon = dEast  / (EarthRadius * Math.Cos(lat0));

        double lat = originLatDeg + dLat * Mathf.Rad2Deg;
        double lon = originLonDeg + dLon * Mathf.Rad2Deg;
        double alt = originAltM + dUp;

        // ---------- 5. publish ----------
        var msg = new NavSatFixMsg();
        SetStamp(ref msg, stamp);
        msg.header.frame_id = frameId;

        msg.latitude  = lat;
        msg.longitude = lon;
        msg.altitude  = alt;

        msg.status.status = NavSatStatusMsg.STATUS_FIX;
        msg.status.service = NavSatStatusMsg.SERVICE_GPS;
        msg.position_covariance = new double[9];
        msg.position_covariance_type = NavSatFixMsg.COVARIANCE_TYPE_UNKNOWN;

        ros.Publish(topic, msg);
    }

    static void SetStamp(ref NavSatFixMsg msg, double stamp)
    {
        int sec = (int)Math.Floor(stamp);
        uint nsec = (uint)((stamp - sec) * 1e9);
        msg.header.stamp.sec = sec;
        msg.header.stamp.nanosec = nsec;
    }
}