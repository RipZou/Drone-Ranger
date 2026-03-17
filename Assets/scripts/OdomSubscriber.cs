using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Nav;

/// <summary>
/// Universal Drone Pose Subscriber for Unity <-> ROS2 (PyBullet or PX4)
/// Supports both NED & ENU frames, position/yaw/full pose modes, and smoothing.
/// </summary>
public class DronePoseSubscriber : MonoBehaviour
{

    public enum PoseMode { PositionOnly, PositionPlusYaw, FullPose }

    [Header("ROS Settings")]
    [Tooltip("ROS odometry topic (odom_bridge publishes ENU to /drone/odom)")]
    public string topicName = "/drone/odom";

    [Tooltip("odom_bridge publishes ENU. Leave unchecked. Check only if using raw NED source.")]
    public bool odomIsNED = false;

    [Header("Control")]
    public bool applyPoseFromROS = true;

    [Header("Target Object")]
    [Tooltip("The transform that will follow the drone pose (default: this object)")]
    public Transform target;

    [Header("Pose Mode")]
    public PoseMode poseMode = PoseMode.FullPose;

    [Header("Unity world origin (drone takeoff place)")]
    [Tooltip("If set, drone position = this transform's position + odom. Place this empty at your desired takeoff spot in the scene.")]
    public Transform worldOrigin;

    [Header("Offsets (optional)")]
    public Vector3 positionOffset = Vector3.zero;
    public float yawOffsetDeg = 0f;

    [Header("Smoothing (optional)")]
    [Tooltip("0 = no smoothing, higher = smoother rotation")]
    public float rotSmooth = 3f;

    [Header("Debug")]
    [Tooltip("Log received position to Console every N seconds (0 = disable)")]
    public float logIntervalSec = 1f;
    [Tooltip("Log subscribe params and first message (recommended for testing)")]
    public bool logConnectionAndParams = true;

    private ROSConnection ros;
    private bool initialized = false;
    private Vector3 worldOriginPosition;
    private float lastLogTime;
    private bool firstMessageLogged;
    private int messageCount;

    void Awake()
    {
        if (target == null) target = transform;
    }

    void Start()
    {
        if (worldOrigin != null)
            worldOriginPosition = worldOrigin.position;
        else
            worldOriginPosition = target.position;

        ros = ROSConnection.GetOrCreateInstance();
        ros.Subscribe<OdometryMsg>(topicName, OnOdom);

        if (logConnectionAndParams)
        {
            string originInfo = worldOrigin != null ? ("worldOrigin=" + worldOrigin.name + " @ " + worldOriginPosition) : ("worldOrigin=this.position @ " + worldOriginPosition);
            Debug.Log($"[DronePoseSubscriber] Subscribe OK | topic={topicName} | target={target.name} | odomIsNED={odomIsNED} | {originInfo}");
        }
        else
            Debug.Log($"[DronePoseSubscriber] Subscribed to " + topicName + (worldOrigin != null ? ", worldOrigin: " + worldOrigin.name : " (using current position as origin)"));

        var rb = target.GetComponent<Rigidbody>();
        if (rb != null) { rb.useGravity = false; rb.isKinematic = true; }
    }

    void OnOdom(OdometryMsg msg)
    {
        messageCount++;
        if (logConnectionAndParams && !firstMessageLogged)
        {
            firstMessageLogged = true;
            Debug.Log($"[DronePoseSubscriber] First /drone/odom received -> ROS link OK. Raw pose x={msg.pose.pose.position.x:F3} y={msg.pose.pose.position.y:F3} z={msg.pose.pose.position.z:F3}");
        }

        if (!applyPoseFromROS) return;
        var p = msg.pose.pose.position;
        var q = msg.pose.pose.orientation;

        Vector3 posUnity;
        Quaternion rotUnity = Quaternion.identity;

        if (odomIsNED)
        {
            // --- NED -> Unity (X=East, Y=Up, Z=North)
            posUnity = new Vector3(
                (float)p.y,      // East  -> Unity X
                (float)(-p.z),   // Down  -> Unity Y (flip)
                (float)p.x       // North -> Unity Z
            );

            // --- NED Quaternion -> Unity Quaternion ---
            Quaternion qRos = new Quaternion((float)q.x, (float)q.y, (float)q.z, (float)q.w);
            Matrix4x4 R_ned = Matrix4x4.Rotate(qRos);

            Matrix4x4 W = new Matrix4x4();
            W.SetRow(0, new Vector4(0, 1, 0, 0));   // Xu = Ye
            W.SetRow(1, new Vector4(0, 0, -1, 0));  // Yu = -Zd
            W.SetRow(2, new Vector4(1, 0, 0, 0));   // Zu = Xn
            W.SetRow(3, new Vector4(0, 0, 0, 1));

            Matrix4x4 B = new Matrix4x4();
            B.SetRow(0, new Vector4(0, 0, 1, 0));   // Xf <- Zu
            B.SetRow(1, new Vector4(1, 0, 0, 0));   // Yr <- Xu
            B.SetRow(2, new Vector4(0, -1, 0, 0));  // Zd <- -Yu
            B.SetRow(3, new Vector4(0, 0, 0, 1));

            Matrix4x4 R_unity = W * R_ned * B;

            Vector3 forward = R_unity.GetColumn(2).normalized;
            Vector3 up = R_unity.GetColumn(1).normalized;
            Quaternion rotFull = Quaternion.LookRotation(forward, up);

            if (poseMode == PoseMode.PositionPlusYaw)
            {
                float yaw = Quaternion.LookRotation(forward, Vector3.up).eulerAngles.y + yawOffsetDeg;
                rotUnity = Quaternion.Euler(0f, yaw, 0f);
            }
            else if (poseMode == PoseMode.FullPose)
            {
                rotUnity = Quaternion.Euler(0f, yawOffsetDeg, 0f) * rotFull;
            }
        }
        else
        {
            // --- ENU -> Unity (used by PyBullet) ---
            posUnity = new Vector3(
                (float)p.x,   // East  -> Unity X
                (float)p.z,   // Up    -> Unity Y
                (float)p.y    // North -> Unity Z
            );

            Quaternion qENU = new Quaternion((float)q.x, (float)q.y, (float)q.z, (float)q.w);
            Quaternion qUnityFull = new Quaternion(qENU.x, qENU.z, qENU.y, qENU.w);

            if (poseMode == PoseMode.FullPose)
                rotUnity = qUnityFull * Quaternion.Euler(0f, yawOffsetDeg, 0f);
            else if (poseMode == PoseMode.PositionPlusYaw)
                rotUnity = Quaternion.Euler(0f, (qUnityFull.eulerAngles).y + yawOffsetDeg, 0f);
        }

        // --- Apply Transform: Unity world origin + odom (so drone starts at scene-defined takeoff place) ---
        target.position = worldOriginPosition + posUnity + positionOffset;

        if (logIntervalSec > 0f && Time.time - lastLogTime >= logIntervalSec)
        {
            lastLogTime = Time.time;
            Debug.Log($"[DronePose] /drone/odom msg#={messageCount} -> Unity position=({target.position.x:F2}, {target.position.y:F2}, {target.position.z:F2})");
        }

        if (poseMode != PoseMode.PositionOnly)
        {
            if (rotSmooth > 0f && initialized)
            {
                float alpha = 1f - Mathf.Exp(-rotSmooth * Time.deltaTime);
                target.rotation = Quaternion.Slerp(target.rotation, rotUnity, alpha);
            }
            else
            {
                target.rotation = rotUnity;
                initialized = true;
            }
        }
    }
}