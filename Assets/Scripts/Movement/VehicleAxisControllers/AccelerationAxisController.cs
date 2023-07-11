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
        private float maxAcceleration = 2;
        [SerializeField]
        private float maxReversingSpeed = 10;

        private const float LOW_SPEED_THRESHOLD = 0.5f;

        public Vector3 CalculateAccelerationForce(Vector3 vehicleLocalAxisZ, Vector3 vehicleRigidbodyVelicity, float accelerationInput)
        {
            var speedInDrivingDirection = Vector3.Dot(vehicleLocalAxisZ, vehicleRigidbodyVelicity);
            float normalizedSpeed = NormalizeSpeed(speedInDrivingDirection, accelerationInput < 0);
            var availableTorque = GetAccelerationByDrivingSpeed(normalizedSpeed) * accelerationInput;
            return vehicleLocalAxisZ * availableTorque;
        }

        private float NormalizeSpeed(float speedInDrivingDirection, bool isReversing)
        {
            var maxSpeed = isReversing ? maxReversingSpeed : maxDrivingSpeed;
            return Mathf.Abs(Mathf.Clamp01(speedInDrivingDirection / maxSpeed));
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
