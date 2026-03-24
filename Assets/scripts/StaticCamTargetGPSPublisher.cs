using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Sensor;
using System;
using CesiumForUnity;
using Unity.Mathematics;

public class StaticCamTargetGPSPublisher : MonoBehaviour
{
    [Header("Camera")]
    public Camera cam;

    [Header("World → GPS reference")]
    public Transform worldOrigin;
    public double originLatDeg = 37.7749;
    public double originLonDeg = -122.4194;
    public double originAltM = 30.0;

    [Header("Cesium (preferred)")]
    [Tooltip("If set, Unity points will be converted to WGS84 GPS via CesiumGeoreference (recommended).")]
    public CesiumGeoreference georeference;

    [Tooltip("If true and Cesium georeference is found, use Cesium for GPS conversion instead of the local EarthRadius approximation.")]
    public bool useCesiumGeoreference = true;

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
    private double startTime;

    [Tooltip("Delay publishing /static_cam/target_gps so the scene settles before the first raycast.")]
    public float publishAfterSeconds = 1.0f;

    const double EarthRadius = 6378137.0;

    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        ros.RegisterPublisher<NavSatFixMsg>(topic);

        if (cam == null)
            Debug.LogError("[StaticCamTargetGPS] Camera is NULL");

        startTime = Time.timeAsDouble;
        if (useCesiumGeoreference && georeference == null)
            georeference = FindObjectOfType<CesiumGeoreference>();

        nextTime = Time.timeAsDouble;
    }

    void Update()
    {
        if (Time.timeAsDouble - startTime < publishAfterSeconds) return;

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

        double lat;
        double lon;
        double alt;

        // ---------- 3. world → GPS ----------
        if (useCesiumGeoreference && georeference != null)
        {
            // CesiumGeoreference expects coordinates in its local space.
            Vector3 pLocal = georeference.transform.InverseTransformPoint(pWorld);
            double3 unityLocalD = new double3(pLocal.x, pLocal.y, pLocal.z);
            double3 ecef = georeference.TransformUnityPositionToEarthCenteredEarthFixed(unityLocalD);
            double3 lonLatHeight = CesiumWgs84Ellipsoid.EarthCenteredEarthFixedToLongitudeLatitudeHeight(ecef);
            lon = lonLatHeight.x; // degrees
            lat = lonLatHeight.y; // degrees
            alt = lonLatHeight.z; // meters above ellipsoid
        }
        else
        {
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

            lat = originLatDeg + dLat * Mathf.Rad2Deg;
            lon = originLonDeg + dLon * Mathf.Rad2Deg;
            alt = originAltM + dUp;
        }

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

        try
        {
            ros.Publish(topic, msg);
        }
        catch (System.Exception e)
        {
            Debug.LogWarning($"[StaticCamTargetGPSPublisher] Publish failed on {topic}: {e.Message}. Re-registering publisher and retrying...");
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