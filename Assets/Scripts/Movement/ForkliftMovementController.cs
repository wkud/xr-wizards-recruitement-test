using ForkliftDemo.InputSystem;
using UnityEngine;
using UnityEngine.Serialization;
using System.Linq;
using ForkliftDemo.Movement.VehicleAxisControllers;

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
        //[SerializeField]
        //[Range(0, Mathf.PI / 2)]
        //private float maxSteeringAngle = Mathf.PI / 2;
        //[SerializeField]
        //private float steeringSpeed = 0.05f;
        //[SerializeField]
        //private float drivingForceValue = 0.2f;

        [SerializeField]
        private Transform[] wheelPivots;
        [SerializeField]
        private Transform[] driveWheelPivots;
        [SerializeField]
        private LayerMask DriveableLayerMask;
        [SerializeField]
        private SuspensionAxisController suspensionController;
        [SerializeField]
        private TractionController tractionController;
        [SerializeField]
        private AccelerationAxisController accelerationController;

        //private float steeringAngle;
        //private Vector3 steeringForwardDirection = Vector3.forward;
        //private Vector3 drivingForce;

        private void FixedUpdate()
        {
            var driveInput = playerInputSystem.DrivingInputValue;
            HandleVehiclePhysics(driveInput.y);
        }

        private void HandleVehiclePhysics(float accelerationInput)
        {
            foreach (var wheelPivot in wheelPivots)
            {
                var didRaycastHit = Physics.Raycast(wheelPivot.position, transform.up * -1, out var hitInfo, suspensionController.SuspensionDesiredHeight, DriveableLayerMask);
                if (!didRaycastHit)
                {
                    continue;
                }

                Vector3 resultantForce = default;
                var wheelWorldVelocity = forkliftRigidbody.GetPointVelocity(wheelPivot.position);

                var suspensionForce = suspensionController.CalculateSuspensionForce(hitInfo.distance, transform.up, wheelWorldVelocity);
                resultantForce += suspensionForce;
                var tractionInSteeringAxis = tractionController.CalculateTractionForce(wheelPivot.right, wheelWorldVelocity);
                resultantForce += tractionInSteeringAxis;

                if (driveWheelPivots.Contains(wheelPivot) && accelerationInput > 0)
                {
                    resultantForce += accelerationController.CalculateAccelerationForce(transform.forward, forkliftRigidbody.velocity, accelerationInput);
                }

                if (accelerationInput == 0)
                {
                    var tractionForceInAccelerationAxis = tractionController.CalculateTractionForce(transform.forward, wheelWorldVelocity);
                    resultantForce += tractionForceInAccelerationAxis;
                }

                // TODO add if (accelerationInput < 0) => reverse acceleration
                forkliftRigidbody.AddForceAtPosition(resultantForce, wheelPivot.position);
                Debug.Log("Force: " + resultantForce + "; Acceleration: " + accelerationInput+"; Velocity: "+forkliftRigidbody.velocity);
            }
        }

        //private void HandleDriving(float drivingInput)
        //{
        //    drivingForce = drivingInput * steeringForwardDirection.normalized * drivingForceValue * Time.fixedDeltaTime;
        //    forkliftRigidbody.AddForceAtPosition(drivingForce, forkliftForceAnchor.position);
        //}

        //private void HandleSteering(float steeringInput)
        //{
        //    steeringAngle += steeringInput * steeringSpeed * Time.fixedDeltaTime;
        //    steeringAngle = Mathf.Clamp(steeringAngle, -maxSteeringAngle, maxSteeringAngle);

        //    var slerpRatio = steeringAngle.Map(0, 1, -maxSteeringAngle, maxSteeringAngle);
        //    slerpRatio = Mathf.Clamp01(slerpRatio);

        //    var normalVectorToPlaneOfRotation = transform.up;
        //    var baseForwardDirection = transform.forward;

        //    var quartenion = Quaternion.Slerp(
        //        Quaternion.LookRotation(transform.right * -1, normalVectorToPlaneOfRotation),
        //        Quaternion.LookRotation(transform.right, normalVectorToPlaneOfRotation),
        //        slerpRatio);

        //    steeringForwardDirection = quartenion * baseForwardDirection;
        //}

        //private void OnDrawGizmos()
        //{
        //    Gizmos.color = Color.blue;
        //    Gizmos.DrawLine(transform.position, transform.position + steeringForwardDirection * 20);

        //    Gizmos.color = Color.magenta;
        //    Gizmos.DrawLine(transform.position, transform.position + drivingForce);
        //}
    }
}
