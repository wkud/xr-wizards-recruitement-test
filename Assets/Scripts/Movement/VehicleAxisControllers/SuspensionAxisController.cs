using System;
using UnityEngine;

namespace ForkliftDemo.Movement.VehicleAxisControllers
{
    [Serializable]
    class SuspensionAxisController
    {
        [field: SerializeField]
        public float SuspensionDesiredHeight { get; private set; } = 1f;

        [SerializeField]
        private float suspensionSpringStrength = 100f;
        [SerializeField]
        private float suspensionSpringDamping = 20f;

        public Vector3 CalculateSuspensionForce(float currentSuspensionHeight, Vector3 vehicleLocalAxisY, Vector3 wheelWorldVelocity)
        {
            var offset = SuspensionDesiredHeight - currentSuspensionHeight;
            var upwardsVelocity = Vector3.Dot(vehicleLocalAxisY, wheelWorldVelocity); // project wheel world velocity onto Y axis relative to transform of the forklift

            var suspensionForceValue = offset * suspensionSpringStrength - upwardsVelocity * suspensionSpringDamping;
            return vehicleLocalAxisY * suspensionForceValue;
        }
    }
}
