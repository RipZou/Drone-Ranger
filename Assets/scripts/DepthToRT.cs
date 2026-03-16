// Assets/Scripts/DepthToRT_NoScreen.cs
/*
 * DepthToRT_NoScreen.cs
 *
 * Purpose:
 *   Capture the camera's linear eye-space depth (in meters) and write it into
 *   a floating-point RenderTexture (RFloat) using a GPU CommandBuffer.
 *
 *   This script is designed for off-screen depth capture and does NOT render
 *   anything to the Game View or screen.
 *
 * Key Characteristics:
 *   - Outputs linear depth in meters (not raw/non-linear depth buffer values)
 *   - Uses a hidden shader (Hidden/LinearEyeDepth_CMD) for depth conversion
 *   - Runs fully on the GPU via CommandBuffer (no OnRenderImage, no CPU readback)
 *   - Camera renders to a dummy RenderTexture to drive the render loop
 *
 * Data Flow:
 *   Camera depth buffer (_CameraDepthTexture)
 *     -> Hidden/LinearEyeDepth_CMD shader
 *     -> depthRT (RenderTextureFormat.RFloat, meters)
 *
 * What this script does NOT do:
 *   - No physics or dynamics simulation
 *   - No PyBullet or Flightmare dependency
 *   - No control, estimation, or filtering logic
 *   - No on-screen rendering
 *
 * Typical Use Cases:
 *   - Robotics / perception simulation
 *   - ROS / ROS2 depth sensor pipelines
 *   - Ground-truth depth generation
 *   - Off-screen depth capture for ML or computer vision
 *
 * Notes:
 *   - Requires a Camera component on the same GameObject
 *   - linearDepthMat must use shader: Hidden/LinearEyeDepth_CMD
 *   - depthRT stores depth values in meters (RFloat)
 */
using UnityEngine;
using UnityEngine.Rendering;

[RequireComponent(typeof(Camera))]
public class DepthToRT_NoScreen : MonoBehaviour
{
    [Header("Output depth RT (meters, RFloat)")]
    public RenderTexture depthRT;                

    [Header("Optional dummy RT (to drive camera)")]
    public RenderTexture dummyRT;               

    [Header("Material (Hidden/LinearEyeDepth_CMD)")]
    public Material linearDepthMat;             

    [Header("RT size")]
    public int width = 640;
    public int height = 640;

    [Header("Culling Mask (which layers write to depth)")]
    public LayerMask renderLayers = ~0; // set in Inspector; e.g. exclude OccupancyDebug

    Camera cam;
    CommandBuffer cb;

    void Awake()
    {
        cam = GetComponent<Camera>();

        cam.depthTextureMode |= DepthTextureMode.Depth;

        cam.enabled = true;
        cam.clearFlags = CameraClearFlags.SolidColor;
        cam.backgroundColor = Color.black;
        cam.cullingMask = renderLayers;
        cam.nearClipPlane = 0.01f;
        cam.farClipPlane  = 1000f;

        if (dummyRT == null)
        {
            dummyRT = new RenderTexture(width, height, 24, RenderTextureFormat.ARGB32)
            {
                name = "DepthCam_dummyRT",
                useMipMap = false,
                autoGenerateMips = false
            };
            dummyRT.Create();
            Debug.Log("[DepthToRT] Created dummyRT.");
        }

        if (depthRT == null)
        {
            depthRT = new RenderTexture(width, height, 0, RenderTextureFormat.RFloat)
            {
                name = "DepthMeters_RFloat",
                enableRandomWrite = false,
                useMipMap = false,
                autoGenerateMips = false
            };
            depthRT.Create();
            Debug.Log("[DepthToRT] Created depthRT (RFloat).");
        }

        cam.targetTexture = dummyRT;

        Debug.Log($"[DepthToRT] Awake | mode={cam.depthTextureMode}, targetTexture={cam.targetTexture?.name}");
    }

    void OnEnable()
    {
        if (linearDepthMat == null)
        {
            Debug.LogError("[DepthToRT] linearDepthMat is NULL. 请把材质（使用 Hidden/LinearEyeDepth_CMD）拖进来。");
            return;
        }
        if (depthRT == null)
        {
            Debug.LogError("[DepthToRT] depthRT is NULL.");
            return;
        }

        if (cb == null)
        {
            cb = new CommandBuffer { name = "CopyLinearDepth_To_depthRT" };
            cb.Blit(BuiltinRenderTextureType.None, new RenderTargetIdentifier(depthRT), linearDepthMat, 0);

            cam.AddCommandBuffer(CameraEvent.AfterDepthTexture, cb);
            Debug.Log("[DepthToRT] CommandBuffer added at AfterDepthTexture.");
        }
    }

    void OnDisable()
    {
        if (cb != null)
        {
            cam.RemoveCommandBuffer(CameraEvent.AfterDepthTexture, cb);
            cb.Release();
            cb = null;
        }
    }
}