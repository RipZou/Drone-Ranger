using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Sensor;
using System;

public class DroneGpsPublisher : MonoBehaviour
{
    [Header("Drone Transform")]
    public Transform drone;   

    [Header("World → GPS reference")]
    public Transform worldOrigin;
    public double originLatDeg = 37.7749;
    public double originLonDeg = -122.4194;
    public double originAltM  = 30.0;

    [Header("ROS")]
    public string topic  = "/drone/gps";
    public string frameId = "drone_gps";

    [Header("Timing")]
    public int fps = 10;
    public int latencyMs = 40;

    private ROSConnection ros;
    private double nextTime;

    const double EarthRadius = 6378137.0;

    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        ros.RegisterPublisher<NavSatFixMsg>(topic);

        if (drone == null)
            drone = transform;

        nextTime = Time.timeAsDouble;
    }

    void Update()
    {
        if (Time.timeAsDouble < nextTime) return;
        nextTime += 1.0 / Mathf.Max(1, fps);

        double stamp = Time.timeAsDouble - latencyMs * 1e-3;

        // --- World position of drone ---
        Vector3 droneWorld = drone.position;

        // --- Relative to world origin ---
        Vector3 originPos = worldOrigin != null ? worldOrigin.position : Vector3.zero;
        Vector3 dWorld = droneWorld - originPos;

        // Unity → ENU
        double dEast  = dWorld.x;
        double dNorth = -dWorld.z;   // Unity Z forward = South
        double dUp    = dWorld.y;

        // ENU → GPS
        double lat0 = originLatDeg * Mathf.Deg2Rad;
        double dLat = dNorth / EarthRadius;
        double dLon = dEast / (EarthRadius * Math.Cos(lat0));

        double lat = originLatDeg + dLat * Mathf.Rad2Deg;
        double lon = originLonDeg + dLon * Mathf.Rad2Deg;
        double alt = originAltM + dUp;

        // --- Pack ROS message ---
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
