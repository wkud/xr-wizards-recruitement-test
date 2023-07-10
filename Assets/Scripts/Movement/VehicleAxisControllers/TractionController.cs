using System;
using UnityEngine;

namespace ForkliftDemo.Movement.VehicleAxisControllers
{
    [Serializable]
    class TractionController
    {
        [SerializeField]
        [Range(0, 1)]
        private float wheelFrictionFactor = 0.5f;
        [SerializeField]
        private float wheelMass = 0.1f;

        public Vector3 CalculateTractionForce(Vector3 vehicleLocalTractionAxis, Vector3 wheelWorldVelocity)
        {
            var steeringVelocity = Vector3.Dot(vehicleLocalTractionAxis, wheelWorldVelocity);
            float desiredVelocityChange = -steeringVelocity * wheelFrictionFactor;
            float desiredAcceleration = desiredVelocityChange / Time.fixedDeltaTime;
            return vehicleLocalTractionAxis * wheelMass * desiredAcceleration;
        }
    }
}
