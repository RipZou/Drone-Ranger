/*
 * GridBuildDebugSubscriber.cs
 * Subscribes to /planning/grid_build_pose (geometry_msgs/PoseStamped) and
 * /planning/grid_build_depth_info (std_msgs/String). Logs pose (NED + Unity) and depth summary for debugging.
 */
using System;
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;

// geometry_msgs/PoseStamped (header + pose; pose in NED: x=north, y=east, z=down)
[Serializable]
public class GridBuildPoseStampedMsg : Message
{
    public const string k_RosMessageName = "geometry_msgs/PoseStamped";
    public override string RosMessageName => k_RosMessageName;
    public PathHeaderMsg header;
    public PathPoseMsg pose;

    public static GridBuildPoseStampedMsg Deserialize(MessageDeserializer d)
    {
        var msg = new GridBuildPoseStampedMsg();
        msg.header = new PathHeaderMsg();
        msg.header.stamp = new PathTimeMsg();
        d.Read(out msg.header.stamp.sec);
        d.Read(out msg.header.stamp.nanosec);
        d.Read(out msg.header.frame_id);
        msg.pose = new PathPoseMsg();
        msg.pose.position = new PathPointMsg();
        msg.pose.orientation = new PathQuatMsg();
        d.Read(out msg.pose.position.x);
        d.Read(out msg.pose.position.y);
        d.Read(out msg.pose.position.z);
        d.Read(out msg.pose.orientation.x);
        d.Read(out msg.pose.orientation.y);
        d.Read(out msg.pose.orientation.z);
        d.Read(out msg.pose.orientation.w);
        return msg;
    }
}

// std_msgs/String
[Serializable]
public class GridBuildStringMsg : Message
{
    public const string k_RosMessageName = "std_msgs/String";
    public override string RosMessageName => k_RosMessageName;
    public string data;

    public static GridBuildStringMsg Deserialize(MessageDeserializer d)
    {
        var msg = new GridBuildStringMsg();
        d.Read(out msg.data);
        return msg;
    }
}

public class GridBuildDebugSubscriber : MonoBehaviour
{
    [Header("ROS")]
    public string gridBuildPoseTopic = "/planning/grid_build_pose";
    public string gridBuildDepthInfoTopic = "/planning/grid_build_depth_info";

    [Header("Logging")]
    [Tooltip("Log pose and depth info every time a message is received")]
    public bool logPose = true;
    public bool logDepthInfo = true;

    private ROSConnection ros;

    static Vector3 NedToUnity(double north, double east, double down)
    {
        return new Vector3((float)east, (float)(-down), (float)north);
    }

    void Start()
    {
        MessageRegistry.Register<GridBuildPoseStampedMsg>(
            GridBuildPoseStampedMsg.k_RosMessageName,
            GridBuildPoseStampedMsg.Deserialize);
        MessageRegistry.Register<GridBuildStringMsg>(
            GridBuildStringMsg.k_RosMessageName,
            GridBuildStringMsg.Deserialize);

        ros = ROSConnection.GetOrCreateInstance();
        ros.Subscribe<GridBuildPoseStampedMsg>(gridBuildPoseTopic, OnGridBuildPoseReceived);
        ros.Subscribe<GridBuildStringMsg>(gridBuildDepthInfoTopic, OnGridBuildDepthInfoReceived);
        Debug.Log("[GridBuildDebugSubscriber] Subscribed to " + gridBuildPoseTopic + " and " + gridBuildDepthInfoTopic);
    }

    void OnGridBuildPoseReceived(GridBuildPoseStampedMsg msg)
    {
        if (msg == null || !logPose) return;
        var p = msg.pose?.position;
        var o = msg.pose?.orientation;
        if (p == null) return;
        // NED: x=north, y=east, z=down
        double n = p.x, e = p.y, d = p.z;
        Vector3 unityPos = NedToUnity(n, e, d);
        string stampStr = msg.header?.stamp != null
            ? msg.header.stamp.sec + "." + msg.header.stamp.nanosec.ToString("D9")
            : "?";
        Debug.Log("[GridBuildPose] stamp=" + stampStr + " frame_id=" + (msg.header?.frame_id ?? "?") +
            " NED(n,e,d)=(" + n.ToString("F3") + "," + e.ToString("F3") + "," + d.ToString("F3") + ")" +
            " Unity(x,y,z)=(" + unityPos.x.ToString("F3") + "," + unityPos.y.ToString("F3") + "," + unityPos.z.ToString("F3") + ")" +
            (o != null ? " quat(x,y,z,w)=(" + o.x + "," + o.y + "," + o.z + "," + o.w + ")" : ""));
    }

    void OnGridBuildDepthInfoReceived(GridBuildStringMsg msg)
    {
        if (msg == null || !logDepthInfo) return;
        Debug.Log("[GridBuildDepthInfo] " + (string.IsNullOrEmpty(msg.data) ? "(empty)" : msg.data));
    }
}
