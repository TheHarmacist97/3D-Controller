using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public enum CameraState
{
    Third_Person, //free; orientation of next movement provided by camera script
    Over_The_Shoulder //locked to character/aiming; orientation of next movement depends on character script
}

public class CameraController : MonoBehaviour
{
    public CameraState cameraState;
    [SerializeField] private Camera cam;
    [SerializeField] private TPSController character;
    [SerializeField] private Transform lookTarget;
    [SerializeField] private Transform posTarget;
    
    // Start is called before the first frame update
    void Start()
    {
        
    }

    // Update is called once per frame
    void Update()
    {
        
    }


    public void DoAimPunch(Vector3 location)
    {

    }
}
