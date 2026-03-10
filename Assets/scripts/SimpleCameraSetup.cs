// Assets/Scripts/SimpleCameraSetup.cs
// 核心思路：始终只用 Main Camera 渲染，切换视角时改变 Main Camera 的跟随行为
// 这样 Cesium 3D Tiles 始终能正确基于 Main Camera 加载 LOD
using UnityEngine;

public class SimpleCameraSetup : MonoBehaviour
{
    public enum CameraMode { Overview, Chase, FPV }

    [Header("Main Camera (must be the scene's MainCamera)")]
    public Camera mainCam;

    [Header("Drone to Follow")]
    public Transform droneTransform;

    [Header("Overview Settings")]
    public Vector3 overviewPosition = new Vector3(-88, 200, 109);
    public Vector3 overviewRotation = new Vector3(90, 0, 0);

    [Header("Chase Camera Settings")]
    public Vector3 chaseOffset = new Vector3(0, 4, -8);
    public float smoothPos = 8f;
    public float smoothRot = 8f;

    [Header("FPV Settings")]
    public Vector3 fpvLocalOffset = new Vector3(0f, 0f, 0.5f);
    public Vector3 fpvLocalEuler  = Vector3.zero;

    [Header("Default Mode")]
    public CameraMode defaultMode = CameraMode.Chase;

    CameraMode currentMode;
    bool initialized = false;

    void Start()
    {
        // 如果没指定，自动找 Main Camera
        if (mainCam == null)
            mainCam = Camera.main;

        if (mainCam == null)
        {
            Debug.LogError("[CameraSetup] No Main Camera found!");
            return;
        }

        // 确保 Main Camera 启用且没有 targetTexture（渲染到屏幕）
        mainCam.enabled = true;
        mainCam.targetTexture = null;

        // 禁用其他非必要相机（OverviewCamera, ChaseCamera, FPVCamera 如果还存在的话）
        // 这些现在不再需要，因为我们只用 Main Camera
        DisableExtraCameras();

        currentMode = defaultMode;
        Debug.Log($"[CameraSetup] Ready. Mode={currentMode}. Press 1/2/3 to switch.");
    }

    void Update()
    {
        if (Input.GetKeyDown(KeyCode.Alpha1)) { currentMode = CameraMode.Overview; Debug.Log("[CameraSetup] Overview"); }
        if (Input.GetKeyDown(KeyCode.Alpha2)) { currentMode = CameraMode.Chase;    Debug.Log("[CameraSetup] Chase"); }
        if (Input.GetKeyDown(KeyCode.Alpha3)) { currentMode = CameraMode.FPV;      Debug.Log("[CameraSetup] FPV"); }
    }

    void LateUpdate()
    {
        if (mainCam == null) return;

        switch (currentMode)
        {
            case CameraMode.Overview:
                UpdateOverview();
                break;
            case CameraMode.Chase:
                UpdateChase();
                break;
            case CameraMode.FPV:
                UpdateFPV();
                break;
        }
    }

    void UpdateOverview()
    {
        mainCam.transform.position = overviewPosition;
        mainCam.transform.rotation = Quaternion.Euler(overviewRotation);
    }

    void UpdateChase()
    {
        if (droneTransform == null) return;

        Vector3 dronePos = droneTransform.position;
        Quaternion droneRot = droneTransform.rotation;

        Vector3 desiredPos = dronePos + droneRot * chaseOffset;
        float posAlpha = 1f - Mathf.Exp(-smoothPos * Time.deltaTime);
        mainCam.transform.position = Vector3.Lerp(mainCam.transform.position, desiredPos, posAlpha);

        Vector3 lookTarget = dronePos + droneRot * (Vector3.forward * 5f);
        Quaternion desiredRot = Quaternion.LookRotation(lookTarget - mainCam.transform.position, Vector3.up);
        float rotAlpha = 1f - Mathf.Exp(-smoothRot * Time.deltaTime);
        mainCam.transform.rotation = Quaternion.Slerp(mainCam.transform.rotation, desiredRot, rotAlpha);
    }

    void UpdateFPV()
    {
        if (droneTransform == null) return;

        mainCam.transform.position = droneTransform.TransformPoint(fpvLocalOffset);
        mainCam.transform.rotation = droneTransform.rotation * Quaternion.Euler(fpvLocalEuler);
    }

    void DisableExtraCameras()
    {
        // 查找 Drone 下的子相机，如果有的话禁用（它们现在不需要渲染到屏幕了）
        string[] names = { "OverviewCamera", "ChaseCamera", "FPVCamera" };
        foreach (string n in names)
        {
            var go = GameObject.Find(n);
            if (go != null)
            {
                var cam = go.GetComponent<Camera>();
                if (cam != null && cam != mainCam)
                {
                    cam.enabled = false;
                    Debug.Log($"[CameraSetup] Disabled extra camera: {n}");
                }
            }
        }
    }
}