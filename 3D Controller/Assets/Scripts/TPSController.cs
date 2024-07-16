using System;
using Unity.Mathematics;
using UnityEditor;
using UnityEngine;


public enum PhysicState
{
    Jumping,
    Falling,
    Grounded,
    TraversingSlope,
    SlidingDown,
}

public class TPSController : MonoBehaviour
{
    [Header("Jump")]
    [SerializeField] private float jumpImpulse;
    [SerializeField] private int jumps;

    [Space(10)]

    [Header("Lateral Movement")]
    [Tooltip("Sliding Acceleration"), SerializeField] private float slidingAccel;
    [Tooltip("Acceleration while jumping"), SerializeField] private float jumpAccel;
    [Tooltip("Acceleration while falling"), SerializeField] private float fallAccel;
    [Tooltip("On Incline Acceleration"), SerializeField] private float inclineModifier;
    [Tooltip("On Decline Acceleration"), SerializeField] private float declineModifier;
    [Tooltip("Grounded Acceleration"), SerializeField] private float acceleration = 1.0f;
    [SerializeField] private float maxSpeed;

    [Space(10)]

    [Header("Physics")]
    public PhysicState physicState;

    [SerializeField, Range(0f, 0.99f)] private float groundedLimit; //max incline for grounded state 
    [SerializeField, Range(0f, 0.99f)] private float slopeLimit; //max incline for slopeTraversal

    [SerializeField] private float groundCheckDistance;
    [SerializeField] private LayerMask groundLayer;

    [Header("Character")]
    [SerializeField] private bool isAiming;

    [Header("Camera Related")]
    [SerializeField] private Vector2 sensitivity;
    [SerializeField] private Transform cameraParent;
    [SerializeField] Transform model;

    private CapsuleCollider characterCollider;
    private Rigidbody rb;

    private bool jumpSignal, canJump, isGrounded;

    private Vector3 headingDir, lateralMovement;
    private Vector2 lookDir;
    private int currentJumps;

    private float groundCheckRadius;
    private Vector3 groundCheckOffset;
    private Ray groundRay;
    private RaycastHit groundRayHit;
    private Vector3 adjustmentVector;

    private void Awake()
    {
        rb = GetComponent<Rigidbody>();
        groundRay = new(transform.position, Vector3.down);
        characterCollider = GetComponent<CapsuleCollider>();
        groundCheckRadius = characterCollider.radius * 0.99f;

        //swap if groundLimit is greater than slopeLimit
        if (groundedLimit < slopeLimit)
        {
            (groundedLimit, slopeLimit) = (slopeLimit, groundedLimit);
        }
    }

    // Update is called once per frame
    void Update()
    {
        GetState();
        GetInput();
        UpdateVisualState();
    }

    private void UpdateVisualState()
    {
        if(isAiming)
        {
            model.rotation = Quaternion.Euler(0f, lookDir.x, 0f);
        }
        else if(headingDir.sqrMagnitude>0f)
        {
            model.rotation = Quaternion.LookRotation(headingDir, Vector3.up);
        }
        cameraParent.rotation = Quaternion.Euler(lookDir.y, lookDir.x, 0f);
    }

    private void GetState()
    {
        groundCheckOffset = transform.position + ((groundCheckRadius + .01f) * transform.up);
        groundRay = new Ray(groundCheckOffset, -transform.up);
        isGrounded = Physics.SphereCast(groundRay, groundCheckRadius, out groundRayHit, groundCheckDistance, groundLayer);

        if (isGrounded)
        {
            currentJumps = 0;
        }
        else if (currentJumps == 0)
        {
            currentJumps = 1;
        }

        canJump = isGrounded || currentJumps < jumps;
        adjustmentVector = canJump ? groundRayHit.normal : Vector3.up;

        GetCurrentPhysicState();
    }

    private void GetCurrentPhysicState()
    {
        if (!isGrounded)
        {
            physicState = rb.velocity.y > 0 ? PhysicState.Jumping : PhysicState.Falling;
            return;
        }

        //now is grounded
        if (adjustmentVector.y >= groundedLimit)
        {
            physicState = PhysicState.Grounded;
        }
        else if (adjustmentVector.y >= slopeLimit)
        {
            physicState = PhysicState.TraversingSlope;
        }
        else
        {
            physicState = PhysicState.SlidingDown;
        }

    }

    private void GetInput()
    {
        GetMouseFunctions();
        GetLateralMovement();
        if (Input.GetKeyDown(KeyCode.Space) && canJump)
        {
            jumpSignal = true;
        }
    }

    private void GetSpecialAbilities()
    {

    }

    private void GetMouseFunctions()
    {
        if (Input.GetMouseButtonDown(1))
        {
            isAiming = !isAiming;
        }
        lookDir.x += Input.GetAxisRaw("Mouse X") * sensitivity.x; //yaw
        lookDir.y += Input.GetAxisRaw("Mouse Y") * sensitivity.y; //pitch

        //lookDir.y = Mathf.Clamp(-40f, 180f, lookDir.y);
    }

    private void GetLateralMovement()
    {
        headingDir.x = Input.GetAxisRaw("Horizontal");
        headingDir.z = Input.GetAxisRaw("Vertical");

        //headingDir.Normalize();  //could do this as well if we Quantizing doesn't work
        headingDir = QuantizeLateralMovement(headingDir);
        headingDir = RotateVector(headingDir, -lookDir.x);
        headingDir -= (Vector3.Dot(headingDir, adjustmentVector) * adjustmentVector); //projection on current plane
        headingDir.Normalize();
    }

    private Vector3 RotateVector(Vector3 input, float degrees)
    {
        float rads = Mathf.Deg2Rad * degrees;
        Vector3 rotVec = Vector3.zero;
        float s = Mathf.Sin(rads);
        float c = Mathf.Cos(rads);

        rotVec.x = c*input.x - s*input.z;
        rotVec.z = s*input.x + c*input.z;

        return rotVec;
    }

    private void FixedUpdate()
    {
        float stateBasedAcceleration = GetAcceleration();
        rb.AddForce(headingDir * stateBasedAcceleration, ForceMode.VelocityChange);
        Vector3 gravity = GetStateBasedGravity();
        rb.AddForce(gravity, ForceMode.Acceleration);
        headingDir = Vector3.zero;
        if (jumpSignal)
        {
            rb.AddForce(Vector3.up * jumpImpulse, ForceMode.VelocityChange);
            jumpSignal = false;
            currentJumps++;
        }
    }

    private Vector3 GetStateBasedGravity()
    {
        return physicState switch
        {
            PhysicState.Grounded or PhysicState.TraversingSlope => Vector3.zero,
            PhysicState.Jumping or PhysicState.Falling or PhysicState.SlidingDown => Physics.gravity,
            _ => Vector3.zero,
        };
    }

    private float GetAcceleration()
    {
        return physicState switch
        {
            PhysicState.Jumping => jumpAccel,
            PhysicState.Falling => fallAccel,
            PhysicState.Grounded => acceleration,
            PhysicState.TraversingSlope => Mathf.Max(acceleration + (headingDir.y > 0 ? -inclineModifier : declineModifier), 0f),
            PhysicState.SlidingDown => slidingAccel,
            _ => 0,
        };
    }

    private Vector3 QuantizeLateralMovement(Vector3 inputVector)
    {
        float x = Mathf.Abs(inputVector.x);
        float y = Mathf.Abs(inputVector.y);

        if (x + y == 2)
        {
            return new Vector3(inputVector.x * 0.707f, 0f, inputVector.z * 0.707f);
        }
        else
        {
            return inputVector;
        }
    }

    private void OnDrawGizmos()
    {
        Handles.color = Color.yellow;
        Gizmos.DrawRay(transform.position + transform.up, headingDir);
    }
}
