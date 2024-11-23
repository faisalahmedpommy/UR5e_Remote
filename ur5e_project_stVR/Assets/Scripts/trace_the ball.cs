using System;
using UnityEngine;

public class UR5eIKController : MonoBehaviour
{
    public ArticulationBody[] Joints; // All robot joints
    public Transform sphere; // Sphere to manipulate the robot arm
    public float maxReachDistance = 1.0f; // Maximum reach of the robot
    public float learningRate = 1.0f; // Increased learning rate for faster IK corrections
    public int maxIterations = 300; // Maximum iterations for IK convergence (reduced for faster response)
    public float threshold = 0.001f; // Threshold for stopping IK
    public float smoothFactor = 0.3f; // Increased smoothing factor for faster movements
    public float wrist2MinAngle = -50f; // Minimal wrist angle (rotated upwards by 50 degrees)
    public float wrist2MaxAngle = 50f; // Max wrist angle (rotated downwards by 50 degrees)
    public float shoulderMinAngle = -180f; // Minimum rotation for the shoulder link
    public float shoulderMaxAngle = 180f; // Maximum rotation for the shoulder link
    public float elbowMinAngle = 20f; // Minimum elbow angle (20 degrees)
    public float elbowMaxAngle = 150f; // Maximum elbow angle (150 degrees)

    // UR5e DH parameters
    private float[] d = { 0.1625f, 0.0f, 0.0f, 0.1333f, 0.0997f, 0.0996f }; // Link offsets
    private float[] a = { 0.0f, -0.425f, -0.3922f, 0.0f, 0.0f, 0.0f }; // Link lengths
    private float[] alpha = { Mathf.PI / 2, 0, 0, Mathf.PI / 2, -Mathf.PI / 2, 0 }; // Link twists

    public int wrist2Index = 4; // Index for wrist 2 joint (elbow)
    public int wrist3Index = 5; // Index for wrist 3 joint
    public int shoulderIndex = 0; // Shoulder link index
    public float wristRelativeAngle = 0.0f; // Desired relative angle between wrist 2 and wrist 3

    private void Start()
    {
        // Check if the sphere has been assigned via the inspector
        if (sphere == null)
        {
            // Try to find the GameObject named "Sphere" and assign its transform
            sphere = GameObject.Find("Sphere")?.transform;

            // Error handling if the sphere is still null
            if (sphere == null)
            {
                Debug.LogError("Sphere object not found! Please assign the sphere variable in the inspector or check the object name.");
            }
        }
    }

    private void Update()
    {
        if (sphere == null)
        {
            Debug.LogWarning("Sphere is not assigned. Update skipped.");
            return; // Skip update if the sphere is not assigned
        }

        Vector3 targetPosition = sphere.position; // Get sphere position as the target

        // Check if the sphere is within reach of the robot
        if (Vector3.Distance(Joints[0].transform.position, targetPosition) > maxReachDistance)
        {
            targetPosition = (targetPosition - Joints[0].transform.position).normalized * maxReachDistance + Joints[0].transform.position;
        }

        // Solve inverse kinematics to get the joint angles for the target position
        float[] targetJointAngles = SolveIK(targetPosition);

        // Clamp shoulder and wrist 2 angles
        targetJointAngles[shoulderIndex] = Mathf.Clamp(targetJointAngles[shoulderIndex], shoulderMinAngle * Mathf.Deg2Rad, shoulderMaxAngle * Mathf.Deg2Rad);
        targetJointAngles[wrist2Index] = Mathf.Clamp(targetJointAngles[wrist2Index], wrist2MinAngle * Mathf.Deg2Rad, wrist2MaxAngle * Mathf.Deg2Rad);
        targetJointAngles[wrist3Index] = targetJointAngles[wrist2Index] + wristRelativeAngle;

        // Apply the joint angles smoothly to the robot
        ApplyJointAnglesSmoothly(targetJointAngles);
    }

    // Function to solve inverse kinematics using Jacobian-based method
    float[] SolveIK(Vector3 targetPosition)
    {
        float[] jointAngles = GetJointAngles(); // Get the current joint angles

        for (int iter = 0; iter < maxIterations; iter++)
        {
            Vector3 currentEndEffectorPos = GetEndEffectorPosition(jointAngles); // Get the current end effector position
            Vector3 error = targetPosition - currentEndEffectorPos; // Compute the error

            if (error.magnitude < threshold) // If error is small enough, break the loop
            {
                Debug.Log("Target reached within threshold.");
                break;
            }

            // Calculate the Jacobian matrix (as a set of Vector3s representing the partial derivatives)
            Vector3[] jacobian = CalculateJacobian(jointAngles);

            // Adjust each joint angle based on the Jacobian and the positional error
            for (int i = 0; i < jointAngles.Length; i++)
            {
                // Update each joint angle using the Jacobian's contribution for that joint
                jointAngles[i] += learningRate * Vector3.Dot(jacobian[i], error);

                // Clamp the elbow (wrist 2) between 20 and 150 degrees
                if (i == wrist2Index)
                {
                    // Convert radians to degrees for clamping
                    float elbowAngleDegrees = jointAngles[i] * Mathf.Rad2Deg;

                    // Clamp between 20 and 150 degrees
                    elbowAngleDegrees = Mathf.Clamp(elbowAngleDegrees, elbowMinAngle, elbowMaxAngle);

                    // Convert back to radians
                    jointAngles[i] = elbowAngleDegrees * Mathf.Deg2Rad;
                }
            }
        }

        return jointAngles;
    }

    // Get the current joint angles
    float[] GetJointAngles()
    {
        float[] jointAngles = new float[Joints.Length];
        for (int i = 0; i < Joints.Length; i++)
        {
            jointAngles[i] = Joints[i].xDrive.target * Mathf.Deg2Rad; // Get current angles in radians
        }
        return jointAngles;
    }

    // Calculate the forward kinematics using DH parameters
    Vector3 GetEndEffectorPosition(float[] jointAngles)
    {
        Matrix4x4 T = Matrix4x4.identity; // Start with identity matrix
        for (int i = 0; i < jointAngles.Length; i++)
        {
            T *= DHMatrix(a[i], alpha[i], d[i], jointAngles[i]); // Multiply by the transformation matrix for each joint
        }
        // Return the position of the end effector
        Vector3 endEffectorPosition = new Vector3(T.m03, T.m13, T.m23);
        return endEffectorPosition;
    }

    // Compute DH transformation matrix for a given joint
    Matrix4x4 DHMatrix(float a, float alpha, float d, float theta)
    {
        float cosTheta = Mathf.Cos(theta);
        float sinTheta = Mathf.Sin(theta);
        float cosAlpha = Mathf.Cos(alpha);
        float sinAlpha = Mathf.Sin(alpha);
        Matrix4x4 T = new Matrix4x4();
        T[0, 0] = cosTheta;
        T[0, 1] = -sinTheta * cosAlpha;
        T[0, 2] = sinTheta * sinAlpha;
        T[0, 3] = a * cosTheta;
        T[1, 0] = sinTheta;
        T[1, 1] = cosTheta * cosAlpha;
        T[1, 2] = -cosTheta * sinAlpha;
        T[1, 3] = a * sinTheta;
        T[2, 0] = 0;
        T[2, 1] = sinAlpha;
        T[2, 2] = cosAlpha;
        T[2, 3] = d;
        T[3, 0] = 0;
        T[3, 1] = 0;
        T[3, 2] = 0;
        T[3, 3] = 1;
        return T;
    }

    // Compute the Jacobian matrix numerically
    Vector3[] CalculateJacobian(float[] jointAngles)
    {
        Vector3[] jacobian = new Vector3[jointAngles.Length];
        Vector3 endEffectorPos = GetEndEffectorPosition(jointAngles);
        float delta = 0.001f; // Small delta for numerical derivative
        for (int i = 0; i < jointAngles.Length; i++)
        {
            // Clone the original joint angles and perturb the ith joint angle
            float[] perturbedAngles = (float[])jointAngles.Clone();
            perturbedAngles[i] += delta;

            // Get the new end effector position after perturbing
            Vector3 perturbedPos = GetEndEffectorPosition(perturbedAngles);

            // Compute the derivative (rate of change of position with respect to joint angle i)
            jacobian[i] = (perturbedPos - endEffectorPos) / delta;
        }
        return jacobian;
    }

    // Apply the joint angles smoothly to the robot
    void ApplyJointAnglesSmoothly(float[] targetJointAngles)
    {
        for (int i = 0; i < Joints.Length; i++)
        {
            float targetAngle = targetJointAngles[i] * Mathf.Rad2Deg; // Convert radians to degrees

            // Smooth the transition between the current angle and the target angle
            float currentAngle = Joints[i].xDrive.target;
            float smoothedAngle = Mathf.Lerp(currentAngle, targetAngle, smoothFactor); // Smooth interpolation

            // Apply the smoothed joint angle to the articulation drive
            ArticulationDrive drive = Joints[i].xDrive;
            drive.target = smoothedAngle;
            Joints[i].xDrive = drive;
        }
    }
}

