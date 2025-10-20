// use "ros2 run px4_to_unity odom_bridge" to start the bridge
// make sure TCP connection between Unity and ROS2 is established.

using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Nav;

public class OdomSubscriberSafe : MonoBehaviour
{
    public enum PoseMode { PositionOnly, PositionPlusYaw, FullPose }

    [Header("ROS")]
    public string topicName = "/drone/odom";     // Odometry topic
    public bool odomIsNED = true;                // NED(Z down) -> tick this; ENU(Z up) -> untick

    [Header("Apply To")]
    public Transform target;                     // Drone root transform (the one with your visualizer)
    public PoseMode poseMode = PoseMode.PositionPlusYaw;

    [Header("Offsets (optional)")]
    public Vector3 positionOffset = Vector3.zero;
    public float yawOffsetDeg = 0f;

    [Header("Smoothing (optional)")]
    public float rotSmooth = 0f;                 // 0=无平滑；>0 时开启指数平滑

    private ROSConnection ros;

    void Awake()
    {
        if (target == null) target = transform;
    }

    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        ros.Subscribe<OdometryMsg>(topicName, OnOdom);

        var rb = target.GetComponent<Rigidbody>();
        if (rb != null) { rb.useGravity = false; rb.isKinematic = true; }
    }

    void OnOdom(OdometryMsg msg)
    {
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

            // --- NED/FRD -> Unity ---
            Quaternion qRos = new Quaternion((float)q.x, (float)q.y, (float)q.z, (float)q.w);
            Matrix4x4 R_ned = Matrix4x4.Rotate(qRos);

            Matrix4x4 W = new Matrix4x4();
            W.SetRow(0, new Vector4(0, 1, 0, 0));   // Xu = Ye
            W.SetRow(1, new Vector4(0, 0,-1, 0));   // Yu = -Zd
            W.SetRow(2, new Vector4(1, 0, 0, 0));   // Zu = Xn
            W.SetRow(3, new Vector4(0, 0, 0, 1));

            Matrix4x4 B = new Matrix4x4();
            B.SetRow(0, new Vector4(0, 0, 1, 0));   // Xf <- Zu
            B.SetRow(1, new Vector4(1, 0, 0, 0));   // Yr <- Xu
            B.SetRow(2, new Vector4(0,-1, 0, 0));   // Zd <- -Yu
            B.SetRow(3, new Vector4(0, 0, 0, 1));

            // R_unity = W * R_ned * B
            Matrix4x4 R_unity = W * R_ned * B;

            // forward/up（Vector4 -> Vector3）
            Vector4 f4 = R_unity.GetColumn(2); // forward
            Vector4 u4 = R_unity.GetColumn(1); // up
            Vector3 f  = new Vector3(f4.x, f4.y, f4.z).normalized;
            Vector3 u  = new Vector3(u4.x, u4.y, u4.z).normalized;

            Quaternion rotFull = Quaternion.LookRotation(f, u);

            if (poseMode == PoseMode.PositionPlusYaw)
            {
                float yaw = Quaternion.LookRotation(f, Vector3.up).eulerAngles.y + yawOffsetDeg;
                rotUnity = Quaternion.Euler(0f, yaw, 0f);
            }
            else if (poseMode == PoseMode.FullPose)
            {
                rotUnity = Quaternion.Euler(0f, yawOffsetDeg, 0f) * rotFull;
            }
        }
        else
        {
            // --- ENU -> Unity ---
            posUnity = new Vector3(
                (float)p.x,   // East  -> Unity X
                (float)p.z,   // Up    -> Unity Y
                (float)p.y    // North -> Unity Z
            );

            Quaternion qENU = new Quaternion((float)q.x, (float)q.y, (float)q.z, (float)q.w);
            // ENU -> Unity 
            Quaternion qUnityFull = new Quaternion(qENU.x, qENU.z, qENU.y, qENU.w);

            if (poseMode == PoseMode.FullPose)
                rotUnity = qUnityFull * Quaternion.Euler(0f, yawOffsetDeg, 0f);
            else if (poseMode == PoseMode.PositionPlusYaw)
                rotUnity = Quaternion.Euler(0f, (qUnityFull.eulerAngles).y + yawOffsetDeg, 0f);
        }

        // --- Transform ---
        target.position = posUnity + positionOffset;

        if (poseMode != PoseMode.PositionOnly)
        {
            if (rotSmooth > 0f)
            {
                float a = 1f - Mathf.Exp(-rotSmooth * Time.deltaTime); 
                target.rotation = Quaternion.Slerp(target.rotation, rotUnity, a);
            }
            else
            {
                target.rotation = rotUnity;
            }
        }
    }
}