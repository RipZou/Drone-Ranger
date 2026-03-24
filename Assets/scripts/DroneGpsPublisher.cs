using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Sensor;
using System;
using CesiumForUnity;
using Unity.Mathematics;

public class DroneGpsPublisher : MonoBehaviour
{
    [Header("Drone Transform")]
    public Transform drone;   

    [Header("World → GPS reference")]
    public Transform worldOrigin;
    public double originLatDeg = 37.7749;
    public double originLonDeg = -122.4194;
    public double originAltM  = 30.0;

    [Header("Cesium (preferred)")]
    [Tooltip("If set, Unity ENU positions will be converted to WGS84 GPS via CesiumGeoreference (recommended).")]
    public CesiumGeoreference georeference;

    [Tooltip("If true and Cesium georeference is found, use Cesium for GPS conversion instead of the local EarthRadius approximation.")]
    public bool useCesiumGeoreference = true;

    [Header("ROS")]
    public string topic  = "/drone/gps";
    public string frameId = "drone_gps";

    [Header("Timing")]
    public int fps = 10;
    public int latencyMs = 40;
    [Tooltip("Delay publishing /drone/gps so home isn't set from an early, pre-odom drone pose.")]
    public float publishAfterSeconds = 1.0f;

    private ROSConnection ros;
    private double nextTime;
    private double startTime;

    const double EarthRadius = 6378137.0;

    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        ros.RegisterPublisher<NavSatFixMsg>(topic);

        if (drone == null)
            drone = transform;

        startTime = Time.timeAsDouble;
        if (useCesiumGeoreference && georeference == null)
            georeference = FindObjectOfType<CesiumGeoreference>();

        nextTime = Time.timeAsDouble;
    }

    void Update()
    {
        // Prevent mission "home" being set by an early, pre-odom drone position.
        // You can tune this in the Inspector if needed.
        if (Time.timeAsDouble - startTime < publishAfterSeconds) return;

        if (Time.timeAsDouble < nextTime) return;
        nextTime += 1.0 / Mathf.Max(1, fps);

        double stamp = Time.timeAsDouble - latencyMs * 1e-3;

        // --- Unity world position of drone ---
        Vector3 droneWorld = drone.position;
        double lat;
        double lon;
        double alt;

        // --- Preferred: Cesium (exact WGS84) ---
        if (useCesiumGeoreference && georeference != null)
        {
            // CesiumGeoreference expects coordinates in its local space.
            Vector3 droneLocal = georeference.transform.InverseTransformPoint(droneWorld);
            double3 unityLocalD = new double3(droneLocal.x, droneLocal.y, droneLocal.z);
            double3 ecef = georeference.TransformUnityPositionToEarthCenteredEarthFixed(unityLocalD);
            double3 lonLatHeight = CesiumWgs84Ellipsoid.EarthCenteredEarthFixedToLongitudeLatitudeHeight(ecef);
            lon = lonLatHeight.x; // degrees
            lat = lonLatHeight.y; // degrees
            alt = lonLatHeight.z; // meters above ellipsoid
        }
        else
        {
            // --- Fallback: local EarthRadius approximation ---
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

            lat = originLatDeg + dLat * Mathf.Rad2Deg;
            lon = originLonDeg + dLon * Mathf.Rad2Deg;
            alt = originAltM + dUp;
        }

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

        try
        {
            ros.Publish(topic, msg);
        }
        catch (System.Exception e)
        {
            Debug.LogWarning($"[DroneGpsPublisher] Publish failed on {topic}: {e.Message}. Re-registering publisher and retrying...");
            // If ROSConnection registry got reset due to reconnect/domain reload, re-register and retry once.
            ros.RegisterPublisher<NavSatFixMsg>(topic);
            ros.Publish(topic, msg);
        }
    }

    static void SetStamp(ref NavSatFixMsg msg, double stamp)
    {
        int sec = (int)Math.Floor(stamp);
        uint nsec = (uint)((stamp - sec) * 1e9);
        msg.header.stamp.sec = sec;
        msg.header.stamp.nanosec = nsec;
    }
}
