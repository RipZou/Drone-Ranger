using UnityEngine;

public class SimpleCameraSetup : MonoBehaviour
{
    [Header("Cameras")]
    public Camera overviewCamera, chaseCamera, fpvCamera;

    [Header("Drone to Follow")]
    public Transform droneTransform;    
    Rigidbody droneRb;

    [Header("Chase Camera Settings")]
    public Vector3 chaseOffset = new Vector3(0, 8, -15); 
    public float smoothPos = 8f, smoothRot = 8f;
    public bool chaseLookAt = true;

    [Header("FPV Settings")]
    public Vector3 fpvLocalOffset = new Vector3(0f, 0f, 0.5f);
    public Vector3 fpvLocalEuler = Vector3.zero;

    int currentCameraIndex = 1; 

    void Awake() {
        if (droneTransform) droneRb = droneTransform.GetComponent<Rigidbody>();
    }

    void Start() {
        if (Camera.main) Camera.main.enabled = false; // 防止还有别的相机在渲染
        SetActiveCamera(1);
        Debug.Log("SimpleCameraSetup ready. 1/2/3 switch cameras.");
    }

    void Update() {
        if (Input.GetKeyDown(KeyCode.Alpha1)) SetActiveCamera(0);
        if (Input.GetKeyDown(KeyCode.Alpha2)) SetActiveCamera(1);
        if (Input.GetKeyDown(KeyCode.Alpha3)) SetActiveCamera(2);
    }

    void LateUpdate()
    {
        if (!droneTransform) return;

        Vector3 dronePos = droneRb ? droneRb.position : droneTransform.position;
        Quaternion droneRot = droneRb ? droneRb.rotation : droneTransform.rotation;

        if (chaseCamera)
        {
            Vector3 desiredPos = droneRot * chaseOffset + dronePos; 
            chaseCamera.transform.position = Vector3.Lerp(
                chaseCamera.transform.position, desiredPos,
                1f - Mathf.Exp(-smoothPos * Time.deltaTime));

            Quaternion desiredRot = chaseLookAt
                ? Quaternion.LookRotation((dronePos + droneRot * Vector3.forward * 5f) - chaseCamera.transform.position, Vector3.up)
                : droneRot;

            chaseCamera.transform.rotation = Quaternion.Slerp(
                chaseCamera.transform.rotation, desiredRot,
                1f - Mathf.Exp(-smoothRot * Time.deltaTime));
        }

        if (fpvCamera)
        {
            fpvCamera.transform.position = droneRot * fpvLocalOffset + dronePos;
            fpvCamera.transform.rotation = droneRot * Quaternion.Euler(fpvLocalEuler);
        }
    }

    void SetActiveCamera(int index)
    {
        currentCameraIndex = index;
        if (overviewCamera) overviewCamera.enabled = (index == 0);
        if (chaseCamera)    chaseCamera.enabled    = (index == 1);
        if (fpvCamera)      fpvCamera.enabled      = (index == 2);
    }
}