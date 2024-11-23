using UnityEngine;
using Valve.VR;
 
public class VR_Controller : MonoBehaviour
{
    public SteamVR_Action_Boolean grabAction;    // SteamVR input action for grabbing (trigger or grip)
    public SteamVR_Input_Sources handType;       // Which controller (left or right hand)
 
    private GameObject grabbedObject = null;     // The object being grabbed
    private Transform objectOriginalParent;      // To store the original parent of the grabbed object
    private bool isGrabbing = false;
 
    void Update()
    {
        // If grab button is pressed
        if (grabAction.GetStateDown(handType))
        {
            TryGrabObject();  // Try to grab the object
        }
 
        // If grab button is released
        if (grabAction.GetStateUp(handType))
        {
            ReleaseObject();  // Release the grabbed object
        }
 
        // Move the grabbed object with the controller
        if (isGrabbing && grabbedObject != null)
        {
            MoveObjectWithController();
        }
    }
 
    // Try to grab an object in front of the controller
    private void TryGrabObject()
    {
        RaycastHit hit;
        if (Physics.Raycast(transform.position, transform.forward, out hit, 10))  // 10 units max distance
        {
            if (hit.transform.CompareTag("Grabbable"))  // Tag objects as "Grabbable"
            {
                grabbedObject = hit.transform.gameObject;
                objectOriginalParent = grabbedObject.transform.parent;  // Store its original parent
                grabbedObject.transform.SetParent(this.transform);  // Make the object follow the controller
                isGrabbing = true;
            }
        }
    }
 
    // Release the grabbed object
    private void ReleaseObject()
    {
        if (grabbedObject != null)
        {
            grabbedObject.transform.SetParent(objectOriginalParent);  // Return it to its original parent (if any)
            grabbedObject = null;
        }
        isGrabbing = false;
    }
 
    // Move the object with the controller
    private void MoveObjectWithController()
    {
        // Since the object is now a child of the controller, it will automatically follow the controller's position and rotation.
        // If you want to manually control the position or apply some constraints, you can do that here.
    }
}
