using UnityEngine;

public class SphereMovement : MonoBehaviour
{
    public float speed = 5f;

    void Update()
    {
        // Move sphere in X, Y, and Z direction using arrow keys or WASD keys
        float moveX = Input.GetAxis("Horizontal") * speed * Time.deltaTime; // Left-right movement
        float moveY = 0; // Not changing Y-axis for now
        float moveZ = Input.GetAxis("Vertical") * speed * Time.deltaTime; // Forward-backward movement

        // Apply movement to the sphere
        transform.Translate(new Vector3(moveX, moveY, moveZ));
    }
}

