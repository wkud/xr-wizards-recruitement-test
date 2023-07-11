using ForkliftDemo.InputSystem;
using UnityEngine;
using UnityEngine.Serialization;
using System.Linq;
using ForkliftDemo.Movement.VehicleAxisControllers;
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
        [Range(0, MathExtension.HALF_PI)]
        private float maxSteeringAngle = MathExtension.HALF_PI;
        [SerializeField]
        [Range(0, 1)]
        private float steeringSpeed = 0.3f;

        [SerializeField]
        private Transform[] wheelPivots;
        [SerializeField]
        private Transform[] driveWheelPivots;
        [SerializeField]
        private Transform[] steeringWheelPivots;
        [FormerlySerializedAs("DriveableLayerMask")]
        [SerializeField]
        private LayerMask DriveableFloorLayerMask;
        [SerializeField]
        private SuspensionAxisController suspensionController;
        [SerializeField]
        private TractionController tractionController;
        [SerializeField]
        private AccelerationAxisController accelerationController;

        private float steeringAngle;

        private void FixedUpdate()
        {
            var driveInput = playerInputSystem.DrivingInputValue;
            HandleVehiclePhysics(driveInput.y);
            HandleSteering(driveInput.x);
        }

        private void HandleVehiclePhysics(float accelerationInput)
        {
            foreach (var wheelPivot in wheelPivots)
            {
                var didRaycastHit = Physics.Raycast(wheelPivot.position, transform.up * -1, out var hitInfo, suspensionController.SuspensionDesiredHeight, DriveableFloorLayerMask);
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

                if (driveWheelPivots.Contains(wheelPivot) && accelerationInput != 0)
                {
                    resultantForce += accelerationController.CalculateAccelerationForce(transform.forward, forkliftRigidbody.velocity, accelerationInput);
                }

                if (accelerationInput == 0)
                {
                    var tractionForceInAccelerationAxis = tractionController.CalculateTractionForce(transform.forward, wheelWorldVelocity);
                    resultantForce += tractionForceInAccelerationAxis;
                }

                forkliftRigidbody.AddForceAtPosition(resultantForce, wheelPivot.position);
            }
        }

        private void HandleSteering(float steeringInput)
        {
            var desiredSteeringAngle = steeringInput * maxSteeringAngle;
            steeringAngle = Mathf.Lerp(steeringAngle, desiredSteeringAngle, steeringSpeed);
            steeringAngle = Mathf.Clamp(steeringAngle, -maxSteeringAngle, maxSteeringAngle);

            foreach (var steeringWheelPivot in steeringWheelPivots)
            {
                steeringWheelPivot.localRotation = Quaternion.Euler(new Vector3(0, steeringAngle * Mathf.Rad2Deg, 0));
            }
        }
    }
}
