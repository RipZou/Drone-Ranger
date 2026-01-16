// Assets/Scripts/DepthToRT_NoScreen.cs
using UnityEngine;
using UnityEngine.Rendering;

[RequireComponent(typeof(Camera))]
public class DepthToRT_NoScreen : MonoBehaviour
{
    [Header("Output depth RT (meters, RFloat)")]
    public RenderTexture depthRT;                // 真正给 Publisher 读的 32F 深度图（线性、米）

    [Header("Optional dummy RT (to drive camera)")]
    public RenderTexture dummyRT;                // 只用于让相机离屏渲染（避免打到主屏）

    [Header("Material (Hidden/LinearEyeDepth_CMD)")]
    public Material linearDepthMat;              // 采样 _CameraDepthTexture 并做 LinearEyeDepth

    [Header("RT size")]
    public int width = 640;
    public int height = 640;

    Camera cam;
    CommandBuffer cb;

    void Awake()
    {
        cam = GetComponent<Camera>();

        // 1) 让相机生成 _CameraDepthTexture
        cam.depthTextureMode |= DepthTextureMode.Depth;

        // 2) 不打到主屏：我们用 targetTexture 驱动
        cam.enabled = true;
        cam.clearFlags = CameraClearFlags.SolidColor;
        cam.backgroundColor = Color.black;
        cam.cullingMask = ~0;
        cam.nearClipPlane = 0.01f;
        cam.farClipPlane  = 1000f;

        // 3) 自动准备 dummyRT（若未指定）
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

        // 4) 自动准备 depthRT（若未指定）
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

        // 5) 绑定 dummyRT 到相机，这样相机会被 Unity 每帧正常驱动（但不打到主屏）
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
            // 让材质把 _CameraDepthTexture 线性化为米，并写入 depthRT
            cb.Blit(BuiltinRenderTextureType.None, new RenderTargetIdentifier(depthRT), linearDepthMat, 0);

            // 关键：当相机已经生成完 _CameraDepthTexture 后再执行
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