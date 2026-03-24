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
    [Tooltip("While angular error between locked chase heading and drone is within this (degrees), do not update the lock. Above it, smoothly align to the drone heading.")]
    public float chaseAttitudeDeadbandDegrees = 90f;
    [Tooltip("During alignment: angular speed (deg/s) = remaining angle (deg) × this gain, so speed falls naturally as error shrinks; capped by max below.")]
    public float chaseAttitudeAlignGainDegreesPerSecondPerDegree = 1.8f;
    [Tooltip("Maximum angular speed (deg/s) during alignment, limiting how aggressive rotation is when the error is large.")]
    public float chaseAttitudeAlignMaxDegreesPerSecond = 80f;
    [Tooltip("When remaining angle is below this (degrees), treat as aligned: snap lock to drone and exit alignment.")]
    public float chaseAttitudeAlignEpsilonDegrees = 0.15f;
    public bool chaseLookAt = true;

    [Header("FPV Settings")]
    public Vector3 fpvLocalOffset = new Vector3(0f, 0f, 0.5f);
    public Vector3 fpvLocalEuler = Vector3.zero;

    [Header("Main Camera Policy")]
    [Tooltip("Do NOT disable Unity's Main Camera by default.\n"
        + "CesiumGeoreference with _useMainCamera=1 often depends on it to update geospatial transforms.\n"
        + "Set to true only if you are sure CesiumGeoreference uses another camera.")]
    public bool disableUnityMainCamera = false;

    int currentCameraIndex = 1;
    Quaternion chaseAttitudeRef = Quaternion.identity;
    bool chaseAttitudeAligning;

    void Awake() {
        if (droneTransform) {
            droneRb = droneTransform.GetComponent<Rigidbody>();
            chaseAttitudeRef = droneTransform.rotation;
        }
    }

    void Start() {
        // Important: keep Camera.main enabled by default.
        // Otherwise CesiumGeoreference (when configured to use Main Camera) may stop updating,
        // causing the globe/objects to appear incorrect or "empty".
        if (disableUnityMainCamera && Camera.main) Camera.main.enabled = false;
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

        float attError = Quaternion.Angle(chaseAttitudeRef, droneRot);
        // Proportional to remaining error: faster when far, slower near target (avoids deadband-normalized lerp, which stays near-constant speed for large deadbands).
        float alignSpeed = Mathf.Min(
            attError * chaseAttitudeAlignGainDegreesPerSecondPerDegree,
            chaseAttitudeAlignMaxDegreesPerSecond);
        float alignMaxDelta = alignSpeed * Time.deltaTime;

        // Alignment state: outside deadband starts a run; keep rotating until error <= epsilon, then snap and lock.
        if (chaseAttitudeAligning) {
            if (attError <= chaseAttitudeAlignEpsilonDegrees) {
                chaseAttitudeAligning = false;
                chaseAttitudeRef = droneRot;
            } else {
                chaseAttitudeRef = Quaternion.RotateTowards(chaseAttitudeRef, droneRot, alignMaxDelta);
            }
        } else if (attError > chaseAttitudeDeadbandDegrees) {
            chaseAttitudeAligning = true;
            chaseAttitudeRef = Quaternion.RotateTowards(chaseAttitudeRef, droneRot, alignMaxDelta);
        }

        if (chaseCamera)
        {
            Vector3 desiredPos = chaseAttitudeRef * chaseOffset + dronePos;
            chaseCamera.transform.position = Vector3.Lerp(
                chaseCamera.transform.position, desiredPos,
                1f - Mathf.Exp(-smoothPos * Time.deltaTime));

            Quaternion desiredRot = chaseLookAt
                ? Quaternion.LookRotation((dronePos + chaseAttitudeRef * Vector3.forward * 5f) - chaseCamera.transform.position, Vector3.up)
                : chaseAttitudeRef;

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
