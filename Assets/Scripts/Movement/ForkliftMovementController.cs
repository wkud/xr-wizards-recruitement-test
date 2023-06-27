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

        private float steeringAngle;
        private Vector3 steeringForwardDirection = Vector3.forward;
        private Vector3 drivingForce;

        private void Start()
        {
            playerInputSystem.OnLiftingUpdated += OnLiftingUpdated;
            playerInputSystem.OnDrivingUpdated += OnDrivingUpdated;
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
