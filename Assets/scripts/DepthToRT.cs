// Assets/Scripts/DepthToRT.cs
using UnityEngine;
using UnityEngine.Rendering;

[RequireComponent(typeof(Camera))]
public class DepthToRT : MonoBehaviour
{
    [Header("Output RT (RFloat or RHalf)")]
    public RenderTexture target;            

    [Header("Material")]
    public Material linearDepthMat;        

    private Camera cam;
    private CommandBuffer cb;

    void Awake()
    {
        cam = GetComponent<Camera>();

        // 让相机生成 _CameraDepthTexture
        cam.depthTextureMode |= DepthTextureMode.Depth;

        // 用 Solid Color 最稳（避免 Skybox/后处理把内容清掉）
        cam.clearFlags = CameraClearFlags.SolidColor;
        cam.backgroundColor = Color.black;

        cam.cullingMask = ~0;
        cam.nearClipPlane = 0.01f;
        cam.farClipPlane  = 1000f;

        Debug.Log($"[DepthToRT] Awake | mode={cam.depthTextureMode}");
    }

    void OnEnable()
    {
        if (cb == null)
        {
            cb = new CommandBuffer { name = "CopyLinearDepthToRT" };
            cb.Blit(BuiltinRenderTextureType.None, new RenderTargetIdentifier(target), linearDepthMat, 0);
            cam.AddCommandBuffer(CameraEvent.AfterDepthTexture, cb);
            Debug.Log("[DepthToRT] CommandBuffer added.");
        }
    }

    void LateUpdate()
    {
        cam.Render();
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