/*
 * OccupancyGridVisualizer.cs
 * Subscribes to /planning/occupancy_cloud (geometry_msgs/PoseArray). Each pose.position = NED (n,e,d) of an occupied voxel.
 * Replaces all previous points on each message (no accumulation). Uses same NED->Unity as path (world frame at grid build time).
 */
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;

public class OccupancyGridVisualizer : MonoBehaviour
{
    [Header("ROS")]
    public string occupancyTopic = "/planning/occupancy_cloud";

    [Header("Visual")]
    public float pointScale = 0.25f;
    public Color pointColor = Color.red;
    [Tooltip("Max point GameObjects to create (downsample if message has more)")]
    public int maxPointsToShow = 2500;
    public bool showInGameView = true;
    public bool showInSceneView = true;
    [Tooltip("Log each received message (disable after debugging)")]
    public bool debugLogReceive = true;

    private ROSConnection ros;
    private Vector3[] cachedWorldPositions;
    private int cachedCount;
    private Transform pointsRoot;

    static Vector3 NedToUnity(double north, double east, double down)
    {
        return new Vector3((float)east, (float)(-down), (float)north);
    }

    void Start()
    {
        MessageRegistry.Register<PathPoseArrayMsg>(
            PathPoseArrayMsg.k_RosMessageName,
            PathPoseArrayMsg.Deserialize);

        ros = ROSConnection.GetOrCreateInstance();
        ros.Subscribe<PathPoseArrayMsg>(occupancyTopic, OnOccupancyReceived);
        Debug.Log("[OccupancyGridVisualizer] Subscribed to " + occupancyTopic + " (frame_id will be used as world NED)");

        var go = new GameObject("OccupancyPoints");
        go.transform.SetParent(transform, false);
        pointsRoot = go.transform;
    }

    void OnOccupancyReceived(PathPoseArrayMsg msg)
    {
        if (msg == null || msg.poses == null)
        {
            ClearPoints();
            cachedCount = 0;
            if (debugLogReceive) Debug.Log("[OccupancyGridVisualizer] Received null or empty poses, cleared.");
            return;
        }

        int n = msg.poses.Length;
        if (n == 0)
        {
            ClearPoints();
            cachedCount = 0;
            if (debugLogReceive) Debug.Log("[OccupancyGridVisualizer] Received 0 poses, cleared.");
            return;
        }

        if (cachedWorldPositions == null || cachedWorldPositions.Length < n)
            cachedWorldPositions = new Vector3[n];

        int valid = 0;
        for (int i = 0; i < n; i++)
        {
            var pose = msg.poses[i];
            if (pose == null || pose.position == null) continue;
            var p = pose.position;
            cachedWorldPositions[valid++] = NedToUnity(p.x, p.y, p.z);
        }
        cachedCount = valid;

        if (debugLogReceive)
            Debug.Log("[OccupancyGridVisualizer] Received " + n + " poses, valid=" + valid + ", first Unity pos=" + (valid > 0 ? cachedWorldPositions[0].ToString() : "N/A"));

        if (showInGameView)
        {
            ClearPoints();
            int toCreate = Mathf.Min(cachedCount, maxPointsToShow);
            float step = toCreate >= cachedCount ? 1f : (float)cachedCount / toCreate;
            for (int i = 0; i < toCreate; i++)
            {
                int idx = Mathf.Min((int)(i * step), cachedCount - 1);
                CreatePointMarker(cachedWorldPositions[idx]);
            }
            if (debugLogReceive && toCreate > 0)
                Debug.Log("[OccupancyGridVisualizer] Created " + toCreate + " red point markers.");
        }
    }

    void ClearPoints()
    {
        if (pointsRoot == null) return;
        for (int i = pointsRoot.childCount - 1; i >= 0; i--)
            Destroy(pointsRoot.GetChild(i).gameObject);
    }

    void CreatePointMarker(Vector3 worldPos)
    {
        var go = GameObject.CreatePrimitive(PrimitiveType.Sphere);
        go.name = "occ_pt";
        go.transform.SetParent(pointsRoot, false);
        go.transform.position = worldPos;
        go.transform.localScale = Vector3.one * pointScale;
        var renderer = go.GetComponent<Renderer>();
        if (renderer != null)
        {
            var shader = Shader.Find("Universal Render Pipeline/Lit") ?? Shader.Find("Standard") ?? Shader.Find("Unlit/Color");
            if (shader != null)
            {
                renderer.material = new Material(shader);
                renderer.material.color = pointColor;
            }
        }
        var collider = go.GetComponent<Collider>();
        if (collider != null)
            collider.enabled = false;
    }

    void OnDrawGizmos()
    {
        if (!showInSceneView || cachedWorldPositions == null || cachedCount == 0)
            return;
        Gizmos.color = pointColor;
        int step = cachedCount > maxPointsToShow ? cachedCount / maxPointsToShow : 1;
        for (int i = 0; i < cachedCount; i += step)
            Gizmos.DrawWireSphere(cachedWorldPositions[i], pointScale * 0.5f);
    }
}
