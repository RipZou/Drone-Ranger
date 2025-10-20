using UnityEngine;

public class QuickDroneVisualizer : MonoBehaviour
{
    private GameObject[] propellers = new GameObject[4];

    void Start()
    {
        CreateSimpleDrone();
    }

    void CreateSimpleDrone()
    {
        // Main body (black box)
        GameObject body = GameObject.CreatePrimitive(PrimitiveType.Cube);
        body.transform.parent = transform;
        body.transform.localPosition = Vector3.zero;
        body.transform.localScale = new Vector3(0.5f, 0.1f, 0.5f);
        body.GetComponent<Renderer>().material.color = Color.black;
        body.name = "DroneBody";

        // 4 arms with propellers
        for (int i = 0; i < 4; i++)
        {
            // Create arm (gray cylinder)
            GameObject arm = GameObject.CreatePrimitive(PrimitiveType.Cylinder);
            arm.transform.parent = transform;
            arm.transform.localScale = new Vector3(0.05f, 0.3f, 0.05f);

            // Position arm at 45, 135, 225, 315 degrees
            float angle = (i * 90f + 45f) * Mathf.Deg2Rad;
            arm.transform.localPosition = new Vector3(
                Mathf.Cos(angle) * 0.35f,
                0,
                Mathf.Sin(angle) * 0.35f
            );
            arm.transform.localRotation = Quaternion.Euler(0, 0, 90);
            arm.GetComponent<Renderer>().material.color = Color.gray;
            arm.name = $"Arm_{i}";

            // Create motor (small black cylinder)
            GameObject motor = GameObject.CreatePrimitive(PrimitiveType.Cylinder);
            motor.transform.parent = arm.transform;
            motor.transform.localPosition = new Vector3(0.3f, 0, 0);
            motor.transform.localScale = new Vector3(0.8f, 0.15f, 0.8f);
            motor.GetComponent<Renderer>().material.color = Color.black;
            motor.name = $"Motor_{i}";

            // Create propeller (thin gray cylinder)
            GameObject propeller = GameObject.CreatePrimitive(PrimitiveType.Cylinder);
            propeller.transform.parent = motor.transform;
            propeller.transform.localPosition = new Vector3(0, 0.1f, 0);
            propeller.transform.localScale = new Vector3(4f, 0.02f, 0.4f);
            propeller.transform.localRotation = Quaternion.Euler(90, 0, 0);

            propeller.GetComponent<Renderer>().material.color = new Color(0.3f, 0.3f, 0.3f); // Dark gray
            propeller.name = $"Propeller_{i}";

            propellers[i] = propeller;
        }

        for (int i = 0; i < 4; i++)
        {
            GameObject leg = GameObject.CreatePrimitive(PrimitiveType.Cylinder);
            leg.transform.parent = transform;
            leg.transform.localScale = new Vector3(0.02f, 0.15f, 0.02f);

            float angle = (i * 90f + 45f) * Mathf.Deg2Rad;
            leg.transform.localPosition = new Vector3(
                Mathf.Cos(angle) * 0.2f,
                -0.15f,
                Mathf.Sin(angle) * 0.2f
            );
            leg.GetComponent<Renderer>().material.color = Color.white;
            leg.name = $"LandingLeg_{i}";
        }
    }

    void Update()
    {
        if (propellers[0] != null)
        {
            float rotationSpeed = 1000f * Time.deltaTime;

            // Counter-rotating propellers (diagonal pairs spin same direction)
            propellers[0].transform.Rotate(0, rotationSpeed, 0);  // Front-left: clockwise
            propellers[1].transform.Rotate(0, -rotationSpeed, 0); // Front-right: counter-clockwise
            propellers[2].transform.Rotate(0, -rotationSpeed, 0); // Back-left: counter-clockwise
            propellers[3].transform.Rotate(0, rotationSpeed, 0);  // Back-right: clockwise
        }
    }
}