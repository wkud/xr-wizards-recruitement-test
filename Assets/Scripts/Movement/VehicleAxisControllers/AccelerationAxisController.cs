using ForkliftDemo.ExtensionMethods;
using System;
using UnityEngine;

namespace ForkliftDemo.Movement.VehicleAxisControllers
{
    [Serializable]
    class AccelerationAxisController
    {
        [SerializeField]
        private float maxDrivingSpeed = 30;
        [SerializeField]
        private float maxAcceleration = 3;

        private const float LOW_SPEED_THRESHOLD = 0.5f;

        public Vector3 CalculateAccelerationForce(Vector3 vehicleLocalAxisZ, Vector3 vehicleRigidbodyVelicity, float accelerationInput)
        {
            if (accelerationInput > 0)
            {
                var speedInDrivingDirection = Vector3.Dot(vehicleLocalAxisZ, vehicleRigidbodyVelicity);
                Debug.Log(speedInDrivingDirection);
                float normalizedSpeed = NormalizeSpeed(speedInDrivingDirection);
                var availableTorque = GetAccelerationByDrivingSpeed(normalizedSpeed) * accelerationInput;
                return vehicleLocalAxisZ * availableTorque;
            }
            // TODO add braking
            // TODO apply anti slipping (same method as in SteeringPhysics)

            return Vector3.zero;
        }

        private float NormalizeSpeed(float speedInDrivingDirection)
        {
            return Mathf.Abs(Mathf.Clamp01(speedInDrivingDirection / maxDrivingSpeed));
        }

        private float GetAccelerationByDrivingSpeed(float normalizedDrivingSpeed)
        {
            if (normalizedDrivingSpeed <= LOW_SPEED_THRESHOLD)
            {
                return maxAcceleration;
            }
            if (normalizedDrivingSpeed < 1)
            {
                return Mathf.Lerp(maxAcceleration, 0, normalizedDrivingSpeed.Map(LOW_SPEED_THRESHOLD, 1));
            }

            return 0; // when normalizedSpeed >= 1
        }
    }
}
