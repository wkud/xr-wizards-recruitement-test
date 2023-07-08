using System;
using UnityEngine;

namespace ForkliftDemo.Movement.VehicleAxisControllers
{
    [Serializable]
    class SteeringAxisController
    {
        [SerializeField]
        [Range(0, 1)]
        private float wheelFrictionFactor = 0.5f;
        [SerializeField]
        private float wheelMass = 0.1f;

        public Vector3 CalculateSteeringForce(Vector3 vehicleLocalAxisX, Vector3 wheelWorldVelocity)
        {
            var steeringVelocity = Vector3.Dot(vehicleLocalAxisX, wheelWorldVelocity);
            float desiredVelocityChange = -steeringVelocity * wheelFrictionFactor;
            float desiredAcceleration = desiredVelocityChange / Time.fixedDeltaTime;
            return vehicleLocalAxisX * wheelMass * desiredAcceleration;
        }
    }
}
