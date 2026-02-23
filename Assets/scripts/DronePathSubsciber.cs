/*
 * DronePathSubsciber.cs
 * Subscribes to /planning/path (geometry_msgs/PoseArray), NED -> Unity, draws path with LineRenderer + Gizmos.
 * Message types inherit from Message so ROSConnection.Subscribe<T> accepts them.
 */
using System;
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;

// geometry_msgs/Point
[Serializable]
public class PathPointMsg : Message
{
    public const string k_RosMessageName = "geometry_msgs/Point";
    public override string RosMessageName => k_RosMessageName;
    public double x, y, z;
}

// geometry_msgs/Quaternion
[Serializable]
public class PathQuatMsg : Message
{
    public const string k_RosMessageName = "geometry_msgs/Quaternion";
    public override string RosMessageName => k_RosMessageName;
    public double x, y, z, w;
}

// geometry_msgs/Pose
[Serializable]
public class PathPoseMsg : Message
{
    public const string k_RosMessageName = "geometry_msgs/Pose";
    public override string RosMessageName => k_RosMessageName;
    public PathPointMsg position;
    public PathQuatMsg orientation;
}

// builtin_interfaces/Time
[Serializable]
public class PathTimeMsg : Message
{
    public const string k_RosMessageName = "builtin_interfaces/Time";
    public override string RosMessageName => k_RosMessageName;
    public int sec;
    public uint nanosec;
}

// std_msgs/Header
[Serializable]
public class PathHeaderMsg : Message
{
    public const string k_RosMessageName = "std_msgs/Header";
    public override string RosMessageName => k_RosMessageName;
    public PathTimeMsg stamp;
    public string frame_id;
}

// geometry_msgs/PoseArray
[Serializable]
public class PathPoseArrayMsg : Message
{
    public const string k_RosMessageName = "geometry_msgs/PoseArray";
    public override string RosMessageName => k_RosMessageName;
    public PathHeaderMsg header;
    public PathPoseMsg[] poses;

    public static PathPoseArrayMsg Deserialize(MessageDeserializer d)
    {
        var msg = new PathPoseArrayMsg();
        msg.header = new PathHeaderMsg();
        msg.header.stamp = new PathTimeMsg();
        d.Read(out msg.header.stamp.sec);
        d.Read(out msg.header.stamp.nanosec);
        d.Read(out msg.header.frame_id);
        int len = d.ReadLength();
        msg.poses = new PathPoseMsg[len];
        for (int i = 0; i < len; i++)
        {
            var pose = new PathPoseMsg();
            pose.position = new PathPointMsg();
            pose.orientation = new PathQuatMsg();
            d.Read(out pose.position.x);
            d.Read(out pose.position.y);
            d.Read(out pose.position.z);
            d.Read(out pose.orientation.x);
            d.Read(out pose.orientation.y);
            d.Read(out pose.orientation.z);
            d.Read(out pose.orientation.w);
            msg.poses[i] = pose;
        }
        return msg;
    }
}

public class DronePathSubsciber : MonoBehaviour
{
    [Header("ROS")]
    public string pathTopic = "/planning/path";

    [Header("Visual")]
    public LineRenderer lineRenderer;
    public float lineWidth = 0.4f;
    public Color lineColor = new Color(0.2f, 0.9f, 0.2f, 1f);
    public bool drawInWorldSpace = true;
    public bool showPointMarkers = true;
    public float pointRadius = 0.3f;
    [Tooltip("Log path when received (disable after debugging)")]
    public bool debugLogPath = true;

    private ROSConnection ros;
    private Vector3[] cachedPositions;
    private int cachedCount;

    void Start()
    {
        // Register our message type so MessageRegistry.GetRosMessageName<T>() returns "geometry_msgs/PoseArray"
        // (otherwise Subscribe sends empty message class and ROS reports "Unknown message class ''")
        MessageRegistry.Register<PathPoseArrayMsg>(
            PathPoseArrayMsg.k_RosMessageName,
            PathPoseArrayMsg.Deserialize);

        ros = ROSConnection.GetOrCreateInstance();
        ros.Subscribe<PathPoseArrayMsg>(pathTopic, OnPathReceived);
        Debug.Log("[DronePathSubsciber] Subscribed to " + pathTopic);

        if (lineRenderer == null)
        {
            lineRenderer = gameObject.AddComponent<LineRenderer>();
            lineRenderer.useWorldSpace = drawInWorldSpace;
            Shader shader = Shader.Find("Sprites/Default");
            if (shader == null) shader = Shader.Find("Unlit/Color");
            lineRenderer.material = new Material(shader);
            lineRenderer.material.color = lineColor;
            lineRenderer.startColor = lineColor;
            lineRenderer.endColor = lineColor;
            lineRenderer.startWidth = lineWidth;
            lineRenderer.endWidth = lineWidth * 0.5f;
            lineRenderer.positionCount = 0;
            lineRenderer.loop = false;
        }
    }

    static Vector3 NedToUnity(double north, double east, double down)
    {
        return new Vector3(
            (float)east,
            (float)(-down),
            (float)north
        );
    }

    void OnPathReceived(PathPoseArrayMsg msg)
    {
        if (msg == null || msg.poses == null || msg.poses.Length == 0)
        {
            if (lineRenderer != null)
                lineRenderer.positionCount = 0;
            cachedCount = 0;
            if (debugLogPath) Debug.Log("[DronePathSubsciber] Path empty or null.");
            return;
        }

        int n = msg.poses.Length;
        if (cachedPositions == null || cachedPositions.Length < n)
            cachedPositions = new Vector3[n];

        int valid = 0;
        for (int i = 0; i < n; i++)
        {
            var pose = msg.poses[i];
            if (pose == null || pose.position == null)
            {
                if (debugLogPath && i == 0)
                    Debug.LogWarning("[DronePathSubsciber] pose or pose.position is null - check deserialization. NED uses position.x=North, position.y=East, position.z=Down.");
                continue;
            }
            var p = pose.position;
            cachedPositions[valid++] = NedToUnity(p.x, p.y, p.z);
        }
        cachedCount = valid;

        if (lineRenderer != null)
        {
            lineRenderer.positionCount = cachedCount;
            for (int i = 0; i < cachedCount; i++)
                lineRenderer.SetPosition(i, cachedPositions[i]);
        }

        if (debugLogPath && cachedCount > 0)
            Debug.Log(string.Format("[DronePathSubsciber] Path: {0} points, first Unity pos={1}", cachedCount, cachedPositions[0]));
    }

    void OnDrawGizmos()
    {
        if (!showPointMarkers || cachedPositions == null || cachedCount == 0)
            return;
        Gizmos.color = lineColor;
        for (int i = 0; i < cachedCount; i++)
        {
            Vector3 p = cachedPositions[i];
            if (drawInWorldSpace)
                Gizmos.DrawWireSphere(p, pointRadius);
            else
                Gizmos.DrawWireSphere(transform.TransformPoint(p), pointRadius);
        }
    }
}
