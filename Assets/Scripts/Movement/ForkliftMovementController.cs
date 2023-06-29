using ForkliftDemo.InputSystem;
using UnityEngine;
using UnityEngine.Serialization;
using ForkliftDemo.ExtensionMethods;

namespace ForkliftDemo.Movement
{
    public class ForkliftMovementController : MonoBehaviour
    {
        [FormerlySerializedAs("inputSystem")]
        [SerializeField]
        private PlayerInputSystem playerInputSystem;
        [SerializeField]
        private Rigidbody liftRigidbody;
        [SerializeField]
        private Rigidbody forkliftRigidbody;
        [SerializeField]
        private Transform forkliftForceAnchor;
        [SerializeField]
        [Range(0, Mathf.PI / 2)]
        private float maxSteeringAngle = Mathf.PI / 2;
        [SerializeField]
        private float steeringSpeed = 0.05f;
        [SerializeField]
        private float drivingForceValue = 0.2f;

        [SerializeField]
        private Transform leftBackWheelPivot;
        [SerializeField]
        private Transform leftFrontWheelPivot;
        [SerializeField]
        private Transform rightBackWheelPivot;
        [SerializeField]
        private Transform rightFrontWheelPivot;
        [SerializeField]
        private LayerMask DriveableLayerMask;
        [SerializeField]
        private float suspensionDesiredHeight = 1f;
        [SerializeField]
        private float suspensionSpringStrength = 100f;
        [SerializeField]
        private float suspensionSpringDamping = 20f;

        private float steeringAngle;
        private Vector3 steeringForwardDirection = Vector3.forward;
        private Vector3 drivingForce;
        private Transform[] wheelPivots;

        private void Awake()
        {
            wheelPivots = new Transform[] { leftBackWheelPivot, leftFrontWheelPivot, rightBackWheelPivot, rightFrontWheelPivot };
        }

        private void Start()
        {
            playerInputSystem.OnLiftingUpdated += OnLiftingUpdated;
            playerInputSystem.OnDrivingUpdated += OnDrivingUpdated;
        }

        private void FixedUpdate()
        {
            HandleSuspension();
        }

        private void OnDestroy()
        {
            playerInputSystem.OnLiftingUpdated -= OnLiftingUpdated;
            playerInputSystem.OnDrivingUpdated -= OnDrivingUpdated;
        }

        private void OnLiftingUpdated(float inputValue)
        {

        }

        private void OnDrivingUpdated(Vector2 inputValue)
        {
            HandleSteering(inputValue.x);
            HandleDriving(inputValue.y);
        }

        private void HandleSuspension()
        {
            foreach (var wheelPivot in wheelPivots)
            {
                var didRaycastHit = Physics.Raycast(wheelPivot.position, transform.up * -1, out var hitInfo, suspensionDesiredHeight, DriveableLayerMask);
                if (didRaycastHit)
                {
                    var wheelWorldVelocity = forkliftRigidbody.GetPointVelocity(wheelPivot.position);
                    var offset = suspensionDesiredHeight - hitInfo.distance;
                    var upwardsVelocity = Vector3.Dot(transform.up, wheelWorldVelocity); // project wheel world velocity onto Y axis relative to transform of the forklift

                    var suspensionForceValue = offset * suspensionSpringStrength - upwardsVelocity * suspensionSpringDamping;
                    forkliftRigidbody.AddForceAtPosition(transform.up * suspensionForceValue, wheelPivot.position);
                }
            }
        }

        private void HandleDriving(float drivingInput)
        {
            drivingForce = drivingInput * steeringForwardDirection.normalized * drivingForceValue * Time.fixedDeltaTime;
            forkliftRigidbody.AddForceAtPosition(drivingForce, forkliftForceAnchor.position);
        }

        private void HandleSteering(float steeringInput)
        {
            steeringAngle += steeringInput * steeringSpeed * Time.fixedDeltaTime;
            steeringAngle = Mathf.Clamp(steeringAngle, -maxSteeringAngle, maxSteeringAngle);

            var slerpRatio = steeringAngle.Map(0, 1, -maxSteeringAngle, maxSteeringAngle);
            slerpRatio = Mathf.Clamp01(slerpRatio);

            var normalVectorToPlaneOfRotation = transform.up;
            var baseForwardDirection = transform.forward;

            var quartenion = Quaternion.Slerp(
                Quaternion.LookRotation(transform.right * -1, normalVectorToPlaneOfRotation),
                Quaternion.LookRotation(transform.right, normalVectorToPlaneOfRotation),
                slerpRatio);

            steeringForwardDirection = quartenion * baseForwardDirection;
        }

        private void OnDrawGizmos()
        {
            Gizmos.color = Color.blue;
            Gizmos.DrawLine(transform.position, transform.position + steeringForwardDirection * 20);

            Gizmos.color = Color.magenta;
            Gizmos.DrawLine(transform.position, transform.position + drivingForce);
        }
    }
}
